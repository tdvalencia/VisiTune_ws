import os
import sys
import struct
import serial
from serial.tools import list_ports
from PIL import Image

# ---------- Protocol constants (mirror packets.h) ----------

PACKET_SOF0    = 0xA5
PACKET_SOF1    = 0x5A
PACKET_VERSION = 0x01

PKT_DATA_IMAGE = 0x01  # from your enum
PKT_FLAG_LAST  = 1 << 0

MAX_SIZE       = 0x0FFF
OVERHEAD       = 11
MAX_DATA_SIZE  = MAX_SIZE - OVERHEAD  # max payload (header+image data) bytes

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


# ---------- Utility to choose from a list ----------

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
    options = [f"{p.device} — {p.description}" for p in ports]
    if ports:
        selected = choose_from_list("Select a COM port:", options)
        return selected.split(" — ", 1)[0]
    else:
        print("No serial ports found.")
        sys.exit(1)


def prompt_baud():
    baud_options = ["115200", "230400", "460800", "921600", "1000000", "2000000", "2500000",  "3000000", "4000000"]  # add more if you want
    selected = choose_from_list("Select a baud rate:", baud_options)
    return int(selected)



def prompt_image_file():
    exts = (".png", ".jpg", ".jpeg", ".bmp")
    imgs = [f for f in os.listdir(".") if f.lower().endswith(exts)]
    imgs.sort()
    if imgs:
        return choose_from_list("Select an image file from the current folder:", imgs)
    else:
        print("No image files (.png/.jpg/.jpeg/.bmp) found in current directory.")
        sys.exit(1)


def prompt_dimensions(default_w=480, default_h=320):
    print(f"Target dimensions (enter to accept default {default_w}x{default_h})")
    w_str = input("Width  (pixels): ").strip()
    h_str = input("Height (pixels): ").strip()

    if not w_str:
        w = default_w
    else:
        w = int(w_str)

    if not h_str:
        h = default_h
    else:
        h = int(h_str)

    return w, h


# ---------- Image conversion to RGB565 big-endian ----------

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


# ---------- Build one image "payload" (like create_image_data) ----------

def build_image_payload(x: int, y: int, w: int, h: int, pixel_bytes: bytes) -> bytes:
    """
    Equivalent to your create_image_data:
    [x,y,w,h] (u16 BE) + w*h*2 bytes of RGB565
    """
    header = struct.pack('>4H', x, y, w, h)  # big-endian 16-bit fields
    return header + pixel_bytes


# ---------- Build full packet (SOF + header + payload + CRC) ----------

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


# ---------- Handshake: wait for 'S' from MCU ----------

def wait_for_S(ser):
    """
    Block until we receive a single 'S' byte from the microcontroller.
    Ignores timeouts and any other bytes.
    """
    print("Waiting for 'S' from device...")
    while True:
        b = ser.read(1)
        if b == b'S':
            print("Got 'S' from device.")
            return
        # ignore other bytes / timeouts


# ---------- Tiling logic and sending over UART ----------

def compute_max_rows_per_packet(W: int) -> int:
    """
    Given width W, find maximum tile height so that:
      8 (x,y,w,h) + 2*W*h <= MAX_DATA_SIZE
    """
    max_rows = (MAX_DATA_SIZE - 8) // (2 * W)
    if max_rows <= 0:
        raise ValueError("Width too large to fit any rows in a packet.")
    return int(max_rows)


def send_image_as_packets(ser, image_path: str, W: int, H: int):
    """
    Convert the image, tile it, build packets, and send over UART
    using a two-step handshake per packet:
      1) Send 9-byte header, wait for 'H' (header ACK) from MCU.
      2) Send payload+CRC, then wait for 'S' (OK) from MCU.
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
        # tile height is min(max_rows, remaining rows)
        h_tile = min(max_rows, H - y0)
        tile_px_bytes = 2 * W * h_tile
        start = (y0 * W * 2)
        chunk = full_bytes[start:start + tile_px_bytes]

        # Build image payload for this tile
        x = 0
        w = W
        payload = build_image_payload(x, y0, w, h_tile, chunk)

        # Set LAST flag on final tile
        flags = PKT_FLAG_LAST if (y0 + h_tile >= H) else 0

        # Build full packet (SOF + header + payload + CRC)
        pkt = build_packet(PKT_DATA_IMAGE, flags, seq, payload)

        # Split into header (9 bytes) and body (payload + CRC)
        header = pkt[:9]
        body   = pkt[9:]

        # 1) Send header first
        ser.write(header)
        ser.flush()
        print(f"Sent header for seq={seq}, y={y0}, h={h_tile}")

        # 2) Wait for MCU to ACK header ('H') after it arms DMA
        wait_for_header_ack(ser)

        # 3) Now send payload+CRC
        ser.write(body)
        ser.flush()
        print(f"Sent body for seq={seq}, size={len(body)} bytes")

        total_packets += 1
        total_bytes_sent += len(pkt)
        print(f"Full packet seq={seq}, size={len(pkt)} bytes")

        # 4) Wait for MCU to say "OK, ready for next packet" after CRC check
        wait_for_S(ser)

        seq = (seq + 1) & 0xFFFF
        y0 += h_tile

    print(f"\nDone.")
    print(f"Total packets sent: {total_packets}")
    print(f"Total bytes sent  : {total_bytes_sent}")



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
    """
    Wait specifically for header ACK 'H' from the MCU.
    """
    wait_for_byte(ser, b'H', "header ACK")







# ---------- Main ----------

def main():
    port = prompt_port()
    baud = prompt_baud()
    image_path = prompt_image_file()
    W, H = prompt_dimensions(default_w=480, default_h=320)

    print("\n=== Configuration ===")
    print(f"Port       : {port}")
    print(f"Baud       : {baud}")
    print(f"Image file : {image_path}")
    print(f"Target size: {W} x {H}")
    print("=====================\n")

    with serial.Serial(port, baud, timeout=1) as ser:
        # clear any stale incoming data
        ser.reset_input_buffer()

        send_image_as_packets(ser, image_path, W, H)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCanceled by user.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
