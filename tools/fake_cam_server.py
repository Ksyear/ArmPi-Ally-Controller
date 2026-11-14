#!/usr/bin/env python3
"""Synthetic table camera for autograsp tests (Python standard library only).

Draws coloured blocks and balls on a grey table and serves them as an MJPEG-style
multipart PNG stream, like the mock robot's camera. Table point (x, y) in metres maps to
pixels with the fixed synthetic camera  u = 320 - 1500*y,  v = 555 - 1500*x  (640x480).

  tools/fake_cam_server.py --port 28180 --objects "cube,red,0.18,0.00;ball,blue,0.15,-0.08"
                           [--write-calib vision_calib.json] [--bind 127.0.0.1] [--follow-robot HOST:PORT]

Object positions are world coordinates (= the robot frame when the robot starts). With --follow-robot the camera
subscribes to the mock robot's rosbridge /armpi/status and moves the objects as the base drives (odom + base_cmd),
so base motion can be tested in closed loop.

Endpoints
  /stream               multipart/x-mixed-replace PNG frames, about 10 fps
  /control?remove=N     hide object N (as if the robot lifted it)
  /control?reset=1      show all objects again
  /state                JSON list of objects and whether they are shown
"""
import argparse
import base64
import json
import math
import os
import random
import socket
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

W, H = 640, 480
PX_PER_M = 1500.0
COLORS = {"red": (220, 30, 30), "blue": (30, 60, 220), "green": (40, 180, 60), "yellow": (230, 210, 40)}
SIZES = {"cube": 0.03, "ball": 0.04}


def to_pixel(x, y):
    return 320.0 - PX_PER_M * y, 555.0 - PX_PER_M * x


def parse_objects(text):
    objs = []
    for part in filter(None, (p.strip() for p in text.split(";"))):
        shape, color, x, y = (s.strip() for s in part.split(","))
        if shape not in SIZES or color not in COLORS:
            raise SystemExit("object must be cube|ball, red|blue|green|yellow: %r" % part)
        x, y = float(x), float(y)
        if not (-0.5 < x < 0.5 and -0.5 < y < 0.5):
            raise SystemExit("object position out of range: %r" % part)
        objs.append({"shape": shape, "color": color, "x": x, "y": y, "shown": True})
    if len(objs) > 16:
        raise SystemExit("at most 16 objects")
    return objs


def make_png(rows):
    def chunk(tag, data):
        return struct.pack("!I", len(data)) + tag + data + struct.pack("!I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!IIBBBBB", W, H, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Scene:
    def __init__(self, objs):
        self.objs = objs
        self.lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0)       # robot x, y, yaw in the world (odom)
        self.vel = (0.0, 0.0, 0.0)        # base_cmd vx, vy, wz (robot frame)
        self.pose_t = time.monotonic()
        self.drawn_pose = None
        rnd = random.Random(1)
        # Table: grey with mild fixed noise (so thresholds see a realistic, non-flat background).
        self.background = [bytearray(max(0, min(255, 128 + rnd.randint(-6, 6))) for _ in range(W * 3)) for _ in range(H)]
        self.png = b""
        self.render()

    def robot_pose(self):
        # odom arrives at about 2 Hz: extrapolate with the commanded velocity in between
        with self.lock:
            (x, y, th), (vx, vy, wz), t0 = self.pose, self.vel, self.pose_t
        dt = min(1.0, time.monotonic() - t0)
        return (x + (vx * math.cos(th) - vy * math.sin(th)) * dt,
                y + (vx * math.sin(th) + vy * math.cos(th)) * dt, th + wz * dt)

    def set_pose(self, pose, vel):
        with self.lock:
            self.pose, self.vel, self.pose_t = pose, vel, time.monotonic()

    def render(self):
        rows = [bytearray(r) for r in self.background]
        with self.lock:
            objs = [dict(o) for o in self.objs]
        rx, ry, rth = self.robot_pose()
        self.drawn_pose = (round(rx, 4), round(ry, 4), round(rth, 4))
        for o in objs:
            if not o["shown"]:
                continue
            dx, dy = o["x"] - rx, o["y"] - ry  # world -> robot frame
            ox = math.cos(rth) * dx + math.sin(rth) * dy
            oy = -math.sin(rth) * dx + math.cos(rth) * dy
            cu, cv = to_pixel(ox, oy)
            half = SIZES[o["shape"]] * PX_PER_M / 2.0
            r, g, b = COLORS[o["color"]]
            for v in range(max(0, int(cv - half) - 1), min(H, int(cv + half) + 2)):
                row = rows[v]
                for u in range(max(0, int(cu - half) - 1), min(W, int(cu + half) + 2)):
                    du, dv = u + 0.5 - cu, v + 0.5 - cv
                    inside = (abs(du) <= half and abs(dv) <= half) if o["shape"] == "cube" else (du * du + dv * dv <= half * half)
                    if inside:
                        row[u * 3:u * 3 + 3] = bytes((r, g, b))
        png = make_png(rows)
        with self.lock:
            self.png = png

    def frame(self):
        rx, ry, rth = self.robot_pose()
        if (round(rx, 4), round(ry, 4), round(rth, 4)) != self.drawn_pose:
            self.render()
        with self.lock:
            return self.png

    def control(self, query):
        with self.lock:
            if "remove" in query:
                try:
                    i = int(query["remove"][0])
                except ValueError:
                    return False
                if not 0 <= i < len(self.objs):
                    return False
                self.objs[i]["shown"] = False
            elif "reset" in query:
                for o in self.objs:
                    o["shown"] = True
            else:
                return False
        self.render()
        return True

    def state(self):
        with self.lock:
            return json.dumps(self.objs)


def ws_send_text(sock, text):
    data = text.encode("utf-8")
    mask = os.urandom(4)
    head = bytes([0x81])
    n = len(data)
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack("!H", n)
    else:
        head += bytes([0x80 | 127]) + struct.pack("!Q", n)
    sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))


def ws_read_frame(sock, buf):
    def need(k):
        nonlocal buf
        while len(buf) < k:
            chunk = sock.recv(65536)
            if not chunk:
                raise OSError("closed")
            buf += chunk
    need(2)
    op, n = buf[0] & 0x0F, buf[1] & 0x7F
    pos = 2
    if n == 126:
        need(4)
        n = struct.unpack("!H", buf[2:4])[0]
        pos = 4
    elif n == 127:
        need(10)
        n = struct.unpack("!Q", buf[2:10])[0]
        pos = 10
    if n > 4 * 1024 * 1024:
        raise OSError("frame too large")
    need(pos + n)
    return op, buf[pos:pos + n], buf[pos + n:]


def follow_robot(host, port, scene):
    """Subscribe to /armpi/status on the mock robot's rosbridge and keep the scene's robot pose current."""
    while True:
        try:
            with socket.create_connection((host, port), timeout=5) as s:
                key = base64.b64encode(os.urandom(16)).decode()
                s.sendall(("GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                           "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (host, port, key)).encode())
                buf = b""
                while b"\r\n\r\n" not in buf:
                    chunk = s.recv(4096)
                    if not chunk or len(buf) > 65536:
                        raise OSError("no handshake")
                    buf += chunk
                head, buf = buf.split(b"\r\n\r\n", 1)
                if b" 101" not in head.split(b"\r\n")[0]:
                    raise OSError("no upgrade")
                s.settimeout(None)
                ws_send_text(s, json.dumps({"op": "subscribe", "topic": "/armpi/status"}))
                while True:
                    op, payload, buf = ws_read_frame(s, buf)
                    if op == 8:
                        break
                    if op != 1:
                        continue
                    try:
                        msg = json.loads(payload)
                        if msg.get("topic") != "/armpi/status":
                            continue
                        st = json.loads(msg["msg"]["data"])
                        od, cmd = st.get("odom") or {}, st.get("base_cmd") or {}
                        scene.set_pose((float(od.get("x", 0)), float(od.get("y", 0)), float(od.get("yaw", 0))),
                                       (float(cmd.get("vx", 0)), float(cmd.get("vy", 0)), float(cmd.get("wz", 0))))
                    except (ValueError, KeyError, TypeError):
                        continue
        except OSError:
            time.sleep(1.0)


def calib_json(z_table):
    pts = [(0.12, 0.0), (0.20, 0.0), (0.16, 0.08), (0.16, -0.08), (0.24, 0.0), (0.14, 0.05)]
    points = []
    for x, y in pts:
        u, v = to_pixel(x, y)
        points.append({"u": u, "v": v, "x": x, "y": y})
    return json.dumps({
        "version": 1, "image_w": W, "image_h": H,
        "observe_pose": {"x": 0.1047, "y": 0.0, "z": 0.0753, "pitch": -1.0263, "gripper": 0.0},
        "z_table": z_table, "parallax": [0.0, 0.0], "points": points,
        "H": [0.0, -PX_PER_M, 320.0, -PX_PER_M, 0.0, 555.0, 0.0, 0.0, 1.0],
        "saved_at": "synthetic",
    }, indent=2) + "\n"


def handler(scene, fps):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, body, ctype="text/plain; charset=utf-8"):
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        img = scene.frame()
                        self.wfile.write(b"--frame\r\nContent-Type: image/png\r\nContent-Length: %d\r\n\r\n" % len(img)
                                         + img + b"\r\n")
                        self.wfile.flush()
                        time.sleep(1.0 / fps)
                except OSError:
                    return
            elif url.path == "/control":
                ok = scene.control(parse_qs(url.query))
                self.reply(200 if ok else 400, "ok\n" if ok else "bad request\n")
            elif url.path == "/state":
                self.reply(200, scene.state() + "\n", "application/json")
            else:
                self.reply(404, "not found\n")

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=28180)
    ap.add_argument("--objects", default="cube,red,0.18,0.00;ball,blue,0.15,-0.08")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--z-table", type=float, default=-0.05)
    ap.add_argument("--write-calib", help="write a vision_calib.json matching this synthetic camera, then keep serving")
    ap.add_argument("--follow-robot", metavar="HOST:PORT", help="mock robot rosbridge: move objects as the base drives")
    args = ap.parse_args()
    if args.write_calib:
        with open(args.write_calib, "w", encoding="utf-8") as fp:
            fp.write(calib_json(args.z_table))
    scene = Scene(parse_objects(args.objects))
    if args.follow_robot:
        host, _, port = args.follow_robot.rpartition(":")
        threading.Thread(target=follow_robot, args=(host or "127.0.0.1", int(port), scene), daemon=True).start()
    srv = ThreadingHTTPServer((args.bind, args.port), handler(scene, max(1.0, min(30.0, args.fps))))
    srv.daemon_threads = True
    print("fake camera on http://%s:%d/stream (%d objects)" % (args.bind, args.port, len(scene.objs)), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
