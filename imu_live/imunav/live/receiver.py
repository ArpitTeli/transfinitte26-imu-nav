"""UDP receiver thread: decode packets, align streams, feed the tracker, record to CSV."""
import socket
import threading
import time

from .align import StreamAligner
from .protocol import ACC, decode


def local_ip():
    """The laptop's LAN address (no packet is sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class Receiver(threading.Thread):
    def __init__(self, tracker, port=5555, fs=400.0, record_path=None):
        super().__init__(daemon=True)
        self.tracker, self.port, self.fs = tracker, port, fs
        self.aligner = StreamAligner(fs)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # room for ~1 MB of packets (Windows' default is 64 KB) while this thread waits on the GUI's redraw
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self._rec_lock = threading.Lock()
        self.rec, self.record_path, self.rec_rows = None, None, 0
        self._newest = None                      # newest phone time received (s), before alignment
        self._rec_after = None                   # a new file skips samples up to here (they belong to the last one)
        self.set_record(record_path)
        self.last_t = None                       # phone time (s) of the newest aligned sample
        self.packets = self.lost = self.bad = 0
        self.last_rx = 0.0
        self._last_seq = None
        self._acc_count, self._rate_t0, self.acc_rate = 0, time.monotonic(), 0.0
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                seq, records = decode(data)
            except ValueError:
                self.bad += 1
                continue
            self._account(seq, len(records))
            self._ingest(records)

    def _account(self, seq, n):
        self.packets += 1
        self.last_rx = time.monotonic()
        if self._last_seq is not None:
            gap = (seq - self._last_seq) & 0xFFFFFFFF
            if 1 < gap < 1_000_000:
                self.lost += gap - 1
        self._last_seq = seq

    def _ingest(self, records):
        b = self.aligner.buf
        for typ, t_ns, x, y, z in records:
            t = t_ns * 1e-9
            last = b[typ][-1][0] if b[typ] else None
            if last is not None and t < last - 1.0:  # phone app restarted: timestamps went back
                self.aligner = StreamAligner(self.fs)
                self.tracker.reset()
                b = self.aligner.buf
                self._newest = self._rec_after = None
            self.aligner.push(typ, t, (x, y, z))
            if self._newest is None or t > self._newest:
                self._newest = t
            if typ == ACC:
                self._acc_count += 1
        now = time.monotonic()
        if now - self._rate_t0 >= 1.0:
            self.acc_rate = self._acc_count / (now - self._rate_t0)
            self._acc_count, self._rate_t0 = 0, now
        for t, g, a, gap in self.aligner.pop_ready_gaps():
            self.tracker.add(t, g, a, gap)
            self.last_t = t
            with self._rec_lock:
                if self._rec_after is not None:
                    if self._rec_after - 1.0 < t <= self._rec_after:
                        continue                 # held back by the aligner from before the file switch
                    self._rec_after = None
                if self.rec:
                    self.rec.write(f"{t:.6f},{g[0]:.6f},{g[1]:.6f},{g[2]:.6f},{a[0]:.5f},{a[1]:.5f},{a[2]:.5f},{gap:.4f}\n")
                    self.rec_rows += 1

    def set_record(self, path):
        """Close the current recording (if any) and continue into `path` (None = stop recording).
        Safe to call from the GUI thread while packets arrive: one walk per file."""
        with self._rec_lock:
            if self.rec:
                self.rec.close()
            self.rec = open(path, "w", buffering=1 << 16) if path else None
            self.record_path, self.rec_rows = path, 0
            self._rec_after = self._newest           # the new file starts after everything received so far
            if self.rec:
                self.rec.write("t,gx,gy,gz,ax,ay,az,gap\n")   # gap: seconds of data missing before this row

    def stop(self):
        self.running = False
        self.sock.close()
        if self.is_alive():
            self.join(timeout=2.0)        # let the thread finish its packet before the file closes under it
        self.set_record(None)
