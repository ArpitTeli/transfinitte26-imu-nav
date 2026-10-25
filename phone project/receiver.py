"""HTTP receiver for the 'Sensor Logger' phone app (Android/iOS) HTTP-push stream.

In Sensor Logger: Settings > Data Streaming > enable 'Push to HTTP', URL = http://<pc-ip>:<port>/data,
and tick Accelerometer, Gyroscope, Magnetometer. Each POST holds {"payload":[{"name","time"(ns),"values":{x,y,z}},...]}.
Parsed samples go onto a queue as (sensor, t_seconds, x, y, z) with sensor in {'acc','gyr','mag'}.
"""
import json, queue, socket, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NAMES = {"accelerometer": "acc", "totalacceleration": "tacc", "gyroscope": "gyr",
         "magnetometer": "mag", "uncalibratedmagnetometer": None}


def local_ips():
    ips = set()
    try:
        for ai in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(ai[4][0])
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0]); s.close()
    except OSError:
        pass
    return sorted(i for i in ips if not i.startswith("127."))


class Receiver:
    def __init__(self, q, port=8000):
        self.q, self.port = q, port
        self.server = None
        self.last_rx = 0.0
        self.n_posts = 0
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    msg = json.loads(self.rfile.read(n) or b"{}")
                    outer._ingest(msg)
                    self.send_response(200)
                except Exception:
                    self.send_response(400)
                self.end_headers()

            def do_GET(self):
                self.send_response(200); self.send_header("Content-Type", "text/plain"); self.end_headers()
                self.wfile.write(b"IMU receiver is running. POST Sensor Logger JSON to /data\n")

            def log_message(self, *a):
                pass

        self._H = H

    def _ingest(self, msg):
        import time
        self.n_posts += 1
        self.last_rx = time.time()
        for s in msg.get("payload", []):
            key = NAMES.get(str(s.get("name", "")).lower())
            v = s.get("values", {})
            if key and all(k in v for k in "xyz"):
                self.q.put((key, s["time"] * 1e-9, float(v["x"]), float(v["y"]), float(v["z"])))

    def start(self):
        self.server = ThreadingHTTPServer(("0.0.0.0", self.port), self._H)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def stop(self):
        if self.server:
            self.server.shutdown(); self.server.server_close(); self.server = None
