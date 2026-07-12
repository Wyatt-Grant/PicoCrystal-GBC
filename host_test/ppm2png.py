#!/usr/bin/env python3
"""Minimal dependency-free PPM (P6) -> PNG converter, stdlib only."""
import sys, struct, zlib

def read_ppm(path):
    with open(path, 'rb') as f:
        data = f.read()
    assert data[:2] == b'P6'
    pos = 2
    vals = []
    while len(vals) < 3:
        while data[pos] in b' \t\r\n':
            pos += 1
        if data[pos:pos+1] == b'#':
            while data[pos] not in b'\r\n':
                pos += 1
            continue
        start = pos
        while data[pos] not in b' \t\r\n':
            pos += 1
        vals.append(int(data[start:pos]))
    pos += 1  # single whitespace after maxval
    w, h, maxval = vals
    pixels = data[pos:pos + w*h*3]
    return w, h, pixels

def write_png(path, w, h, pixels):
    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload +
                struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    sig = b'\x89PNG\r\n\x1a\n'
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)  # 8-bit RGB
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)  # filter type 0 per scanline
        raw.extend(pixels[y*stride:(y+1)*stride])
    idat = zlib.compress(bytes(raw), 9)
    with open(path, 'wb') as f:
        f.write(sig)
        f.write(chunk(b'IHDR', ihdr))
        f.write(chunk(b'IDAT', idat))
        f.write(chunk(b'IEND', b''))

if __name__ == '__main__':
    for path in sys.argv[1:]:
        w, h, pixels = read_ppm(path)
        out = path.rsplit('.', 1)[0] + '.png'
        write_png(out, w, h, pixels)
        print(f"{path} -> {out} ({w}x{h})")
