"""ctypes binding to the project's fusion filter (filter/fusion.c compiled to filter/fusion.dll)."""
import ctypes as C, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
FDIR = os.path.join(HERE, "filter")
DLL = os.path.join(FDIR, "fusion.dll" if os.name == "nt" else "libfusion.so")


def build():
    out = DLL
    cmd = [sys.executable, "-m", "ziglang", "cc", "-shared", "-O2", "-o", out,
           os.path.join(FDIR, "fusion.c"), os.path.join(FDIR, "fusion_dll.c")]
    subprocess.check_call(cmd, cwd=FDIR)


def _lib():
    if not os.path.exists(DLL):
        build()
    L = C.CDLL(DLL)
    fp = C.POINTER(C.c_float)
    L.fl_create.restype = C.c_void_p
    L.fl_free.argtypes = [C.c_void_p]
    L.fl_set.argtypes = [C.c_void_p, C.c_char_p, C.c_float]
    L.fl_init.argtypes = [C.c_void_p, fp, fp, C.c_int]
    L.fl_step.argtypes = [C.c_void_p, fp, fp, fp, C.c_int, C.c_float, fp]
    return L


class Filter:
    """Phone-mode fusion: IMU + magnetometer + ZUPT (a phone has no wheel encoders)."""
    NAMES = ["x", "y", "yaw", "v", "bg", "ba", "roll", "pitch", "yaw_q", "still",
             "mag_ok", "mag_rej", "zupt_ok", "zupt_rej", "mag_ref_set", "_"]

    def __init__(self, params=None):
        self.L = _lib()
        self.h = self.L.fl_create()
        self.params = {"enc_v": 0, "enc_w": 0}
        self.params.update(params or {})
        for k, v in self.params.items():
            if not self.L.fl_set(self.h, k.encode(), float(v)):
                raise KeyError("unknown filter parameter: " + k)
        self.out = (C.c_float * 16)()
        self.ready = False

    def init(self, a0, g0=None):
        a = (C.c_float * 3)(*a0)
        g = (C.c_float * 3)(*(g0 if g0 is not None else (0, 0, 0)))
        self.L.fl_init(self.h, a, g, 1 if g0 is not None else 0)
        self.ready = True

    def step(self, a, g, m, dt):
        has = m is not None
        A = (C.c_float * 3)(*a); G = (C.c_float * 3)(*g); M = (C.c_float * 3)(*(m if has else (0, 0, 0)))
        self.L.fl_step(self.h, A, G, M, 1 if has else 0, dt, self.out)
        return dict(zip(self.NAMES, self.out))

    def __del__(self):
        try:
            self.L.fl_free(self.h)
        except Exception:
            pass
