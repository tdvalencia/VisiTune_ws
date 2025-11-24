import os
import sys
import struct
import wave
import time
import threading

import serial
from serial.tools import list_ports
from PIL import Image
import numpy as np

# ---------- Protocol constants (mirror packets.h) ----------

PACKET_SOF0    = 0xA5
PACKET_SOF1    = 0x5A
PACKET_VERSION = 0x01

PKT_DATA_IMAGE = 0x01  # must match your packets.h
PKT_DATA_AUDIO = 0x02  # must match your packets.h

PKT_FLAG_LAST  = 1 << 0

MAX_SIZE       = 0xFFFF
OVERHEAD       = 11                 # 2 SOF + 7 header + 2 CRC
MAX_DATA_SIZE  = MAX_SIZE - OVERHEAD  # max payload bytes

CRC_POLY = 0x1021
CRC_INIT = 0xFFFF

# ---------- Threading primitives ----------

header_ack_event = threading.Event()   # 'H'
ok_ack_event     = threading.Event()   # 'S'
audio_req_event  = threading.Event()   # 'A'
error_event      = threading.Event()   # 'E' (optional)

TX_LOCK = threading.Lock()
rx_thread_running = False


# ---------- CRC16-CCITT (same logic as your C function) ----------

def crc16_ccitt(data: bytes) -> int:
    remainder = CRC_INIT
    for b in data:
        remainder ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if remainder & 0x8000:
                remainder = ((remainder << 1) ^ CRC_POLY) & 0xFFFF
            else:
                remainder = (remainder << 1) & 0xFFFF
    return remainder


# ---------- Generic menu helpers ----------

def choose_from_list(prompt, options):
    if not options:
        raise RuntimeError(f"No options available for: {prompt}")

    print(prompt)
    for i, opt in enumerate(options, 1):
        print(f"  {i}) {opt}")

    while True:
        choice = input("> ").strip().lower()
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(options):
                return options[idx - 1]
        print("Please enter a valid number from the list")


def prompt_port():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)

    options = [f"{p.device} — {p.description}" for p in ports]
    selected = choose_from_list("Select a COM port:", options)
    return selected.split(" — ", 1)[0]


def prompt_baud():
    baud_options = [
        "115200", "230400", "460800", "921600",
        "1000000", "2000000", "2500000", "3000000", "4000000"
    ]
    selected = choose_from_list("Select a baud rate:", baud_options)
    return int(selected)


# ---------- RX worker (reads bytes and sets events) ----------

def rx_worker(ser: serial.Serial):
    global rx_thread_running
    rx_thread_running = True
    print("[RX] Thread started.")
    try:
        while rx_thread_running:
            b = ser.read(1)
            if not b:
                continue  # timeout, keep looping

            if b == b'H':
                header_ack_event.set()
            elif b == b'S':
                ok_ack_event.set()
            elif b == b'A':
                audio_req_event.set()
            elif b == b'E':
                error_event.set()
                print("[RX] Received error 'E' from MCU.")
            else:
                # You can uncomment this to see all bytes:
                # print(f"[RX] Ignored byte: {b!r}")
                pass
    finally:
        print("[RX] Thread exiting.")


# ---------- Event-based wait helpers ----------

def wait_header_ack(label: str = "", timeout: float = 1.0):
    if not header_ack_event.wait(timeout=timeout):
        raise TimeoutError(f"Timeout waiting for header ACK 'H' ({label})")
    header_ack_event.clear()


def wait_ok_ack(label: str = "", timeout: float = 1.0):
    if not ok_ack_event.wait(timeout=timeout):
        raise TimeoutError(f"Timeout waiting for OK 'S' ({label})")
    ok_ack_event.clear()


def wait_audio_req(label: str = "", timeout: float = None):
    """
    Wait for 'A' (next audio chunk). If timeout is None, wait forever.
    """
    if not audio_req_event.wait(timeout=timeout):
        raise TimeoutError(f"Timeout waiting for audio request 'A' ({label})")
    audio_req_event.clear()


# ---------- Image-specific helpers ----------

def prompt_image_file():
    exts = (".png", ".jpg", ".jpeg", ".bmp")
    imgs = [f for f in os.listdir(".") if f.lower().endswith(exts)]
    imgs.sort()
    if imgs:
        return choose_from_list("Select an image file from the current folder:", imgs)
    print("No image files (.png/.jpg/.jpeg/.bmp) found in current directory.")
    sys.exit(1)


def prompt_dimensions(default_w=480, default_h=320):
    print(f"Target dimensions (enter to accept default {default_w}x{default_h})")
    w_str = input("Width  (pixels): ").strip()
    h_str = input("Height (pixels): ").strip()

    w = default_w if not w_str else int(w_str)
    h = default_h if not h_str else int(h_str)
    return w, h


def image_to_rgb565_be_bytes(image_path: str, W: int, H: int) -> bytes:
    """
    Open image, convert to RGB, resize to (W, H),
    and return bytes in row-major order, RGB565 big-endian per pixel.
    """
    im = Image.open(image_path).convert('RGB').resize((W, H), Image.LANCZOS)

    out = bytearray()
    for y in range(H):
        for x in range(W):
            r, g, b = im.getpixel((x, y))
            rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            out.extend(struct.pack('>H', rgb565))  # big-endian
    return bytes(out)  # length = W * H * 2


def build_image_payload(x: int, y: int, w: int, h: int, pixel_bytes: bytes) -> bytes:
    """
    [x,y,w,h] (u16 BE) + w*h*2 bytes of RGB565
    """
    header = struct.pack('>4H', x, y, w, h)  # big-endian 16-bit fields
    return header + pixel_bytes


def compute_max_rows_per_packet(W: int) -> int:
    """
    Given width W, find maximum tile height so that:
      8 (x,y,w,h) + 2*W*h <= MAX_DATA_SIZE
    """
    max_rows = (MAX_DATA_SIZE - 8) // (2 * W)
    if max_rows <= 0:
        raise ValueError("Width too large to fit any rows in a packet.")
    return int(max_rows)


# ---------- Audio-specific helpers ----------

def prompt_wav_file():
    wavs = [f for f in os.listdir(".") if f.lower().endswith(".wav")]
    wavs.sort()
    if wavs:
        return choose_from_list("Select a WAV file from the current folder:", wavs)
    print("No .wav files found in the current directory.")
    sys.exit(1)


def load_wav_mono_16bit(path: str):
    """
    Load a WAV and return (samples_int16_little_endian, sample_rate).
    If stereo, average to mono.
    """
    with wave.open(path, 'rb') as wf:
        n_channels = wf.getnchannels()
        sampwidth  = wf.getsampwidth()
        n_frames   = wf.getnframes()
        framerate  = wf.getframerate()

        raw = wf.readframes(n_frames)

    if sampwidth != 2:
        raise ValueError("Expected 16-bit PCM WAV")

    # Interpret as int16 little-endian
    data = np.frombuffer(raw, dtype='<i2')  # < = little-endian

    if n_channels == 2:
        data = data.reshape(-1, 2).mean(axis=1).astype('<i2')

    return data, framerate


# ---------- Packet builder ----------

def build_packet(data_type: int,
                 flags: int,
                 seq: int,
                 payload: bytes) -> bytes:
    """
    Mirrors your pkt_build on the PC side.
    """
    if len(payload) > MAX_DATA_SIZE:
        raise ValueError(f"Payload too large for single packet: {len(payload)} bytes")

    buf = bytearray()
    buf.append(PACKET_SOF0)
    buf.append(PACKET_SOF1)

    # Header fields
    version   = PACKET_VERSION
    seq_high  = (seq >> 8) & 0xFF
    seq_low   = seq & 0xFF
    len_field = len(payload)
    len_high  = (len_field >> 8) & 0xFF
    len_low   = len_field & 0xFF

    buf.append(version)
    buf.append(data_type & 0xFF)
    buf.append(flags & 0xFF)
    buf.append(seq_high)
    buf.append(seq_low)
    buf.append(len_high)
    buf.append(len_low)

    # Payload
    buf.extend(payload)

    # Compute CRC over [version..end of payload]
    crc_input = bytes(buf[2:])  # same as &buf[2] in C
    crc = crc16_ccitt(crc_input)
    crc_high = (crc >> 8) & 0xFF
    crc_low  = crc & 0xFF

    buf.append(crc_high)
    buf.append(crc_low)

    return bytes(buf)


# ---------- Core send function using events ----------

def send_packet_with_handshake(ser: serial.Serial,
                               data_type: int,
                               flags: int,
                               seq: int,
                               payload: bytes,
                               label: str = ""):
    """
    Send a single packet (header + payload + CRC) and block until
    'H' then 'S' are received (via events).
    """
    pkt = build_packet(data_type, flags, seq, payload)
    header = pkt[:9]
    body   = pkt[9:]

    # Send header
    header_ack_event.clear()
    with TX_LOCK:
        ser.write(header)
        ser.flush()
    print(f"[TX] Header sent ({label}, len={len(header)})")

    # Wait for 'H'
    wait_header_ack(label=label)

    # Send body
    ok_ack_event.clear()
    with TX_LOCK:
        ser.write(body)
        ser.flush()
    print(f"[TX] Body sent   ({label}, len={len(body)})")

    # Wait for 'S'
    wait_ok_ack(label=label)


# ---------- Packet preparation (no I/O) ----------

def make_image_packets(image_path: str, W: int, H: int):
    """
    Precompute all image packets (seq, flags, payload, debug_label).
    """
    print(f"Converting image '{image_path}' to {W}x{H} RGB565...")
    full_bytes = image_to_rgb565_be_bytes(image_path, W, H)
    print(f"Image converted: {len(full_bytes)} bytes total.")

    max_rows = compute_max_rows_per_packet(W)
    print(f"Max rows per packet at width {W}: {max_rows}")

    packets = []
    seq = 0
    y0 = 0
    while y0 < H:
        h_tile = min(max_rows, H - y0)
        tile_px_bytes = 2 * W * h_tile
        start = (y0 * W * 2)
        chunk = full_bytes[start:start + tile_px_bytes]

        x = 0
        w = W
        payload = build_image_payload(x, y0, w, h_tile, chunk)
        flags = PKT_FLAG_LAST if (y0 + h_tile >= H) else 0

        label = f"IMG seq={seq}, y={y0}, h={h_tile}"
        packets.append((seq, flags, payload, label))

        seq = (seq + 1) & 0xFFFF
        y0 += h_tile

    print(f"Prepared {len(packets)} image packets.")
    return packets


def make_audio_packets(wav_path: str):
    """
    Precompute all audio packets (seq, flags, payload, debug_label).
    """
    print(f"Loading WAV file '{wav_path}'...")
    samples, fs = load_wav_mono_16bit(wav_path)
    print(f"WAV loaded: {len(samples)} samples @ {fs} Hz.")

    raw = samples.tobytes()
    total_bytes = len(raw)
    print(f"Total audio bytes: {total_bytes}")

    AUDIO_MAX_PAYLOAD = MAX_DATA_SIZE
    packets = []
    seq = 0
    offset = 0
    pkt_index = 0

    while offset < total_bytes:
        remaining = total_bytes - offset
        this_len  = min(remaining, AUDIO_MAX_PAYLOAD)
        payload   = raw[offset:offset + this_len]
        offset   += this_len

        flags = 0
        if offset >= total_bytes:
            flags |= PKT_FLAG_LAST

        label = f"AUD seq={seq}, idx={pkt_index}, bytes={len(payload)}"
        packets.append((seq, flags, payload, label))

        pkt_index += 1
        seq = (seq + 1) & 0xFFFF

    print(f"Prepared {len(packets)} audio packets.")
    return packets


# ---------- High-level streaming modes ----------

def send_image_only(ser: serial.Serial, image_path: str, W: int, H: int):
    img_packets = make_image_packets(image_path, W, H)

    print("\n[MODE] Image-only transfer")
    for seq, flags, payload, label in img_packets:
        send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label)
    print("[MODE] Image-only transfer complete.")


def send_audio_only(ser: serial.Serial, wav_path: str):
    aud_packets = make_audio_packets(wav_path)

    print("\n[MODE] Audio-only transfer")

    if not aud_packets:
        print("No audio packets to send.")
        return

    # First packet: send immediately (no 'A' gating)
    seq, flags, payload, label = aud_packets[0]
    print("[AUDIO] Sending first audio packet immediately.")
    send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label)

    # Remaining packets: gate on 'A'
    idx = 1
    while idx < len(aud_packets):
        wait_audio_req(label=f"audio idx={idx}")
        seq, flags, payload, label = aud_packets[idx]
        print(f"[AUDIO] MCU requested next chunk -> {label}")
        send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label)
        idx += 1

    print("[MODE] Audio-only transfer complete.")


def send_image_and_audio_interleaved(ser: serial.Serial,
                                     image_path: str,
                                     W: int,
                                     H: int,
                                     wav_path: str):
    """
    Event-loop style:
      - Precompute all image + audio packets.
      - Send first audio packet immediately.
      - Then loop:
          * if 'A' pending and audio packets left:
              send next audio packet
          * else if image packets left:
              send one image packet
          * else:
              done
    """
    img_packets = make_image_packets(image_path, W, H)
    aud_packets = make_audio_packets(wav_path)

    print("\n[MODE] Interleaved Image + Audio")

    if not aud_packets:
        print("No audio packets, falling back to image-only.")
        for seq, flags, payload, label in img_packets:
            send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label)
        return

    # First audio packet: send immediately (no 'A' gating)
    aud_idx = 0
    seq, flags, payload, label = aud_packets[aud_idx]
    print("[INTERLEAVE] Sending first audio packet immediately.")
    send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label)
    aud_idx += 1

    img_idx = 0
    total_img = len(img_packets)
    total_aud = len(aud_packets)

    # Main event-loop
    while img_idx < total_img or aud_idx < total_aud:
        did_something = False

        # 1) Prioritize audio if MCU requested next chunk
        if aud_idx < total_aud and audio_req_event.is_set():
            audio_req_event.clear()
            seq, flags, payload, label = aud_packets[aud_idx]
            print(f"[INTERLEAVE] MCU requested audio -> {label}")
            send_packet_with_handshake(ser, PKT_DATA_AUDIO, flags, seq, payload, label)
            aud_idx += 1
            did_something = True
            continue  # re-check quickly to keep up with MCU

        # 2) If no pending audio request, send an image tile (if any remain)
        if img_idx < total_img:
            seq, flags, payload, label = img_packets[img_idx]
            print(f"[INTERLEAVE] Sending image tile -> {label}")
            send_packet_with_handshake(ser, PKT_DATA_IMAGE, flags, seq, payload, label)
            img_idx += 1
            did_something = True
            continue

        # 3) If audio left but no 'A' yet, just idle briefly
        if aud_idx < total_aud and not did_something:
            time.sleep(0.001)

    print("[MODE] Interleaved Image + Audio complete.")


# ---------- Main ----------

def main():
    global rx_thread_running

    port = prompt_port()
    baud = prompt_baud()

    mode = choose_from_list(
        "What do you want to do?",
        [
            "Image over packets",
            "Audio WAV over packets",
            "Image + Audio (interleaved)"
        ]
    )

    print("\n=== Configuration ===")
    print(f"Port : {port}")
    print(f"Baud : {baud}")
    print(f"Mode : {mode}")
    print("=====================\n")

    with serial.Serial(port, baud, timeout=0.1) as ser:
        ser.reset_input_buffer()

        # Start RX thread once; it services all modes
        rx_thread = threading.Thread(target=rx_worker, args=(ser,), daemon=True)
        rx_thread.start()

        try:
            if mode.startswith("Image over"):
                image_path = prompt_image_file()
                W, H = prompt_dimensions(default_w=480, default_h=320)
                print(f"\nImage file : {image_path}")
                print(f"Target size: {W} x {H}\n")
                send_image_only(ser, image_path, W, H)

            elif mode.startswith("Audio WAV"):
                wav_path = prompt_wav_file()
                print(f"\nWAV file: {wav_path}\n")
                send_audio_only(ser, wav_path)

            else:
                # Interleaved
                image_path = prompt_image_file()
                W, H = prompt_dimensions(default_w=480, default_h=320)
                print(f"\nImage file : {image_path}")
                print(f"Target size: {W} x {H}\n")

                wav_path = prompt_wav_file()
                print(f"\nWAV file: {wav_path}\n")

                send_image_and_audio_interleaved(ser, image_path, W, H, wav_path)

        finally:
            # Stop RX thread
            rx_thread_running = False
            # Give it a moment to exit cleanly
            time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCanceled by user.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
