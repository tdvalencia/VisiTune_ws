import os
import sys
import struct
import wave
import serial
from serial.tools import list_ports
from PIL import Image

# ---------- Protocol constants (mirror packets.h) ----------

PACKET_SOF0    = 0xA5
PACKET_SOF1    = 0x5A
PACKET_VERSION = 0x01

PKT_DATA_IMAGE = 0x01  # must match your packets.h
PKT_DATA_AUDIO = 0x02  # must match your packets.h

PKT_FLAG_LAST  = 1 << 0

MAX_SIZE       = 0xFFFF
OVERHEAD       = 11               # 2 SOF + 7 header + 2 CRC
MAX_DATA_SIZE  = MAX_SIZE - OVERHEAD  # max payload bytes

CRC_POLY = 0x1021
CRC_INIT = 0xFFFF


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


def wait_for_byte(ser, expected: bytes, label: str):
    """
    Block until we receive the expected single byte from the microcontroller.
    Ignores timeouts and any other bytes.
    """
    print(f"Waiting for {label} ({expected!r}) from device...")
    while True:
        b = ser.read(1)
        if not b:
            continue  # timeout, keep waiting
        if b == expected:
            print(f"{label} received.")
            return
        # ignore other bytes


def wait_for_header_ack(ser):
    """Wait specifically for header ACK 'H' from the MCU."""
    wait_for_byte(ser, b'H', "header ACK")


def wait_for_S(ser):
    """Wait specifically for generic OK 'S' from the MCU."""
    wait_for_byte(ser, b'S', "OK/CRC ACK")


def wait_for_A(ser):
    """Wait for 'A' = 'next audio chunk' from the MCU."""
    wait_for_byte(ser, b'A', "next-audio request")


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
    import numpy as np
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

    # Append payload
    buf.extend(payload)

    # Compute CRC over [version..end of payload]
    crc_input = bytes(buf[2:])  # same as &buf[2], len+7 in C
    crc = crc16_ccitt(crc_input)
    crc_high = (crc >> 8) & 0xFF
    crc_low  = crc & 0xFF

    buf.append(crc_high)
    buf.append(crc_low)

    return bytes(buf)


# ---------- Image sending ----------

def send_image_as_packets(ser, image_path: str, W: int, H: int):
    """
    Convert the image, tile it, build packets, and send over UART
    using:
      - header (9 bytes), wait for 'H'
      - body (payload+CRC), wait for 'S'
    """
    print(f"Converting image '{image_path}' to {W}x{H} RGB565...")
    full_bytes = image_to_rgb565_be_bytes(image_path, W, H)
    print(f"Image converted: {len(full_bytes)} bytes total.")

    max_rows = compute_max_rows_per_packet(W)
    print(f"Max rows per packet at width {W}: {max_rows}")

    seq = 0
    total_bytes_sent = 0
    total_packets = 0

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

        pkt = build_packet(PKT_DATA_IMAGE, flags, seq, payload)

        header = pkt[:9]
        body   = pkt[9:]

        # 1) Send header
        ser.write(header)
        ser.flush()
        print(f"Sent header for seq={seq}, y={y0}, h={h_tile}")

        # 2) Wait for 'H'
        wait_for_header_ack(ser)

        # 3) Send body
        ser.write(body)
        ser.flush()
        print(f"Sent body for seq={seq}, size={len(body)} bytes")

        total_packets += 1
        total_bytes_sent += len(pkt)
        print(f"Full packet seq={seq}, size={len(pkt)} bytes")

        # 4) Wait for 'S'
        wait_for_S(ser)

        seq = (seq + 1) & 0xFFFF
        y0 += h_tile

    print(f"\nImage transfer complete.")
    print(f"Total packets sent: {total_packets}")
    print(f"Total bytes sent  : {total_bytes_sent}")


# ---------- Audio sending (packetized) ----------

def send_audio_as_packets(ser, wav_path: str):
    """
    Send WAV audio in packet form using the same packet protocol.
    Protocol (matches your STM side idea):
      - First packet: send immediately (header -> 'H' -> body -> 'S')
      - For subsequent packets:
          wait for 'A' (MCU requests next audio chunk),
          then do header/'H'/body/'S' sequence.
      - Last packet sets PKT_FLAG_LAST.
    """
    print(f"Loading WAV file '{wav_path}'...")
    samples, fs = load_wav_mono_16bit(wav_path)
    print(f"WAV loaded: {len(samples)} samples @ {fs} Hz.")

    # convert to raw little-endian bytes
    raw = samples.tobytes()
    total_bytes = len(raw)
    print(f"Total audio bytes: {total_bytes}")

    AUDIO_MAX_PAYLOAD = MAX_DATA_SIZE # bytes of raw audio per packet
    seq = 0
    offset = 0
    pkt_index = 0

    with ser:  # NOTE: caller can also manage this; leaving here is safe if used standalone
        # we assume the MCU is ready for the first audio packet immediately

        while offset < total_bytes:
            remaining = total_bytes - offset
            this_len  = min(remaining, AUDIO_MAX_PAYLOAD)
            payload   = raw[offset: offset + this_len]
            offset   += this_len

            flags = 0
            if offset >= total_bytes:
                flags |= PKT_FLAG_LAST

            pkt = build_packet(PKT_DATA_AUDIO, flags, seq, payload)
            header = pkt[:9]
            body   = pkt[9:]

            if pkt_index == 0:
                # First packet: send immediately
                print("Sending first audio packet...")
            else:
                # Wait for 'A' from MCU (DAC finished, wants next chunk)
                wait_for_A(ser)

            # Send header
            ser.write(header)
            ser.flush()
            print(f"Sent audio header for seq={seq}, bytes={len(payload)}, flags=0x{flags:02X}")

            # Wait for 'H' ACK
            wait_for_header_ack(ser)

            # Send body (payload + CRC)
            ser.write(body)
            ser.flush()
            print(f"Sent audio body for seq={seq}, body size={len(body)} bytes")

            # Wait for 'S' ACK after CRC + DMA start
            wait_for_S(ser)

            pkt_index += 1
            seq = (seq + 1) & 0xFFFF

    print(f"\nAudio transfer complete.")
    print(f"Packets sent: {pkt_index}")


# ---------- Main ----------

def main():
    port = prompt_port()
    baud = prompt_baud()

    mode = choose_from_list(
        "What do you want to send?",
        ["Image over packets", "Audio WAV over packets"]
    )

    print("\n=== Configuration ===")
    print(f"Port : {port}")
    print(f"Baud : {baud}")
    print(f"Mode : {mode}")
    print("=====================\n")

    # Open the serial port once and share it
    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()

        if mode.startswith("Image"):
            image_path = prompt_image_file()
            W, H = prompt_dimensions(default_w=480, default_h=320)
            print(f"\nImage file : {image_path}")
            print(f"Target size: {W} x {H}\n")
            send_image_as_packets(ser, image_path, W, H)

        else:
            wav_path = prompt_wav_file()
            print(f"\nWAV file: {wav_path}\n")
            send_audio_as_packets(ser, wav_path)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCanceled by user.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
