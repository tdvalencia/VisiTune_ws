# save as convert_to_rgb565_be.py
from PIL import Image
import sys, struct

in_path  = sys.argv[1]          # e.g. input.png
out_path = sys.argv[2]          # e.g. output_240x135.rgb565
W = int(sys.argv[3])            # target width
H = int(sys.argv[4])            # target height

im = Image.open(in_path).convert('RGB').resize((W, H), Image.LANCZOS)
with open(out_path, 'wb') as f:
    for y in range(H):
        for x in range(W):
            r, g, b = im.getpixel((x, y))
            rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            # big-endian: high byte first
            f.write(struct.pack('>H', rgb565))
