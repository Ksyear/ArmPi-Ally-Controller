#!/usr/bin/env python3
"""Generate res/app.ico (16-256 px) with the standard library only: a robot arm on a mecanum base."""
import math
import struct
import sys
import zlib
from pathlib import Path


def render(size: int) -> bytes:
    ss = 4  # supersampling
    n = size * ss
    bg = (28, 31, 38)
    orange = (255, 140, 40)
    light = (235, 238, 242)
    px = [[(0, 0, 0, 0)] * n for _ in range(n)]

    def inside_round_rect(x, y, r):
        r *= n
        cx = min(max(x, r), n - r)
        cy = min(max(y, r), n - r)
        return (x - cx) ** 2 + (y - cy) ** 2 <= r * r

    def seg_dist(x, y, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
        return math.hypot(x - (ax + t * dx), y - (ay + t * dy))

    # arm joints in unit coordinates (y down)
    j = [(0.30, 0.70), (0.30, 0.42), (0.62, 0.28), (0.78, 0.44)]
    for yy in range(n):
        for xx in range(n):
            x, y = xx + 0.5, yy + 0.5
            if not inside_round_rect(x, y, 0.22):
                continue
            c = bg
            u, v = x / n, y / n
            # base with wheels
            if 0.16 <= u <= 0.84 and 0.70 <= v <= 0.80:
                c = light
            for wx in (0.24, 0.76):
                if math.hypot(u - wx, v - 0.84) <= 0.07:
                    c = orange
            # arm links and joints
            for a, b in zip(j, j[1:]):
                if seg_dist(u, v, *a, *b) <= 0.045:
                    c = orange
            for p in j[:3]:
                if math.hypot(u - p[0], v - p[1]) <= 0.06:
                    c = light
            # gripper jaws
            if seg_dist(u, v, 0.78, 0.44, 0.86, 0.38) <= 0.025 or seg_dist(u, v, 0.78, 0.44, 0.86, 0.52) <= 0.025:
                c = light
            px[yy][xx] = (*c, 255)
    out = bytearray()
    for y in range(size):
        out.append(0)  # PNG filter: none
        for x in range(size):
            acc = [0, 0, 0, 0]
            for dy in range(ss):
                for dx in range(ss):
                    p = px[y * ss + dy][x * ss + dx]
                    for k in range(4):
                        acc[k] += p[k]
            out += bytes(a // (ss * ss) for a in acc)
    return bytes(out)


def png(size: int) -> bytes:
    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(render(size), 9)) + chunk(b"IEND", b"")


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "res" / "app.ico"
    sizes = [16, 24, 32, 48, 64, 128, 256]
    images = [png(s) for s in sizes]
    header = struct.pack("<HHH", 0, 1, len(sizes))
    offset = 6 + 16 * len(sizes)
    entries = b""
    for s, img in zip(sizes, images):
        entries += struct.pack("<BBBBHHII", s % 256, s % 256, 0, 0, 1, 32, len(img), offset)
        offset += len(img)
    out.write_bytes(header + entries + b"".join(images))
    preview = out.with_name("app_256.png")
    preview.write_bytes(images[-1])
    print(f"wrote {out} ({out.stat().st_size} bytes) and {preview.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
