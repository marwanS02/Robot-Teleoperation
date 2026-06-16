# realtime_autocal.py
# EMG realtime with one-shot per-user calibration at startup.
# Baseline → guided 5-class capture → quick head fine-tune → fast T*/τ tune → run.

import os, sys, time, json, threading, queue, warnings
from collections import deque
import numpy as np
from dataclasses import dataclass

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import socket
import math
import struct

# ---------------- Performance measurement ----------------
MEASURE_PERFORMANCE = True   # <-- set to True to run guided online eval after calibration
EVAL_REPS = 6                 # repetitions of full class sequence
EVAL_HOLD_SEC = 4.0           # hold duration per class
EVAL_REST_SEC = 1.0           # rest between classes

# ---------------- IMU ----------------
QUAT_SCALE = 16384.0
IMU_FRESH_S = 0.5
# ---------------- Voting ----------------
VOTE_WINDOW = 7        # how many recent predictions to look at (~7 * 30ms ≈ 210 ms)
VOTE_MIN_FRAMES = 3    # don't use voting until we have at least this many frames
VOTE_MAJ_FRAC = 0.8    # fraction of window that must agree to switch class
# ---------------- τ behavior ----------------
USE_TAU_DEFAULTS = True   # <-- set to True to keep TAU_STAR and skip τ calibration
# ---------------- Confidence / variance gate ----------------
USE_VAR_GATE = False           # enable/disable variance-based gating
VAR_MIN = 0.010               # minimum variance across classes to accept non-REST
VAR_REQUIRE_TOP = 0.60        # also require max(P) >= this to accept non-REST
IMU_FRESH_S = 0.5
EMG_FRESH_S = 0.5   # how long EMG can be silent before we force REST
ENABLE_HSTATE_TX = False  # off during baseline + calibration

REST_CLASS = 0  # <-- set this to your real "rest" class index
HSTATE_FRESH_S = 0.5  # e.g. after 0.5 s of silence -> force 1

_last_hstate_rx_wall = 0.0
_last_hstate_value = 1  # start in "nothing" state

# ---------------- Decision low-pass (class-level) ----------------
MIN_CLASS_HOLD_S = 0.25  # minimum time to keep a class before allowing a switch (250 ms)



import time

VIB_DUR_MS = {
    2: 50,   # near border
    3: 150,   # hit/exceeded
    4: 250,   # grasp success
}

vib_active_until_ms = 0.0
_last_hstate_for_clamp = 1  # start in "nothing" state


def on_hstate_update(hstate: int):
    """
    Edge-triggered clamp:
    - Only start a new REST clamp when hstate CHANGES to 2/3/4.
    - Clamp lasts a fixed duration (VIB_DUR_MS[hstate]) and then releases.
    - When hstate returns to 1, we immediately drop the clamp.
    """
    global vib_active_until_ms, _last_hstate_for_clamp

    now_ms = time.time() * 1000.0

    if hstate in (2, 3, 4):
        # Only start a new clamp on state change (avoid continuous extension)
        if hstate != _last_hstate_for_clamp:
            dur = VIB_DUR_MS.get(hstate, 0)
            vib_active_until_ms = now_ms + dur

    elif hstate == 1:
        # Explicitly end clamp as soon as robot says "nothing"
        vib_active_until_ms = now_ms

    _last_hstate_for_clamp = hstate


def get_output_class(voted_class: int) -> int:
    """
    Return the class we actually send to the robot.
    While vibration is active, force REST_CLASS.
    Voting and probabilities are left untouched.
    """
    now_ms = time.time() * 1000.0
    if now_ms < vib_active_until_ms:
        return REST_CLASS
    return int(voted_class)


def quat_to_rpy(qw,qx,qy,qz):
    n = math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz) or 1.0
    qw,qx,qy,qz = qw/n, qx/n, qy/n, qz/n
    siny_cosp = 2*(qw*qz + qx*qy);  cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    sinp = 2*(qw*qy - qz*qx);       pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
    sinr_cosp = 2*(qw*qx + qy*qz);  cosr_cosp = 1 - 2*(qx*qx + qz*qz)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    return roll, pitch, yaw

CLASS_SINK = ("127.0.0.1", 6001)
HSTATE_PORT = 6010  # must match C++ HSTATE_UDP_PORT
HSTATE_QUEUE_MAX = 128
hstate_queue = queue.Queue(maxsize=HSTATE_QUEUE_MAX)


# ---------------- I/O ----------------
PORT = "COM16"
BAUD = 115200
N_STREAMS = 4
N_CH = 8

# ---------------- Sampling/windows ----------------
FS_AGG = 200.0
WINDOW_MS = 200
HOP_MS = 30
ENV_MS = 50
BASELINE_WARMUP_SEC = 3.0
DISPLAY_SEC = 5.0

# ---------------- Model ----------------
RUN_DIR = r"..\\train\\models\\EMGModel\\2025-10-16_08-58-00_with_loss_0.5539"
USE_GPU = True

CLASS_NAMES = ["rest","extension","flexion","radial_flexion","ulnar_flexion"]
IDX_REST = 0
# Defaults used until calibration finishes (then overwritten)
T_STAR = 0.929
# TAU_STAR = np.array([0.688, 0.121, 0.715, 0.307, 0.344], dtype=np.float32)
TAU_STAR = np.array([0.1, 0.121, 0.65, 0.307, 0.344], dtype=np.float32)
# ---------------- Filters ----------------
USE_FILTERS = True
BP_LO, BP_HI = 20.0, 90.0
BP_ORDER = 4
NOTCH_Q = 30.0

# ---------------- Smoothing ----------------
SMOOTH_TAU_S = 0.20
SMOOTH_ADAPT = True
SMOOTH_TOP_LO = 0.35
SMOOTH_TOP_HI = 0.75
SMOOTH_MEDIAN_K = 1

# ---------------- Calibration recipe ----------------
# Total capture ~ short: baseline + 5 prompts
CALIB_BASELINE_SEC = 4.0                 # baseline stabilization before capture
CALIB_PER_CLASS_SEC = 4.0                # default time to hold each gesture
CALIB_PER_CLASS_SEC_RADIAL = 5.0         # longer for radial_flexion (more samples)
CALIB_REST_BETWEEN_SEC = 3.0             # tiny neutral pause between classes
CALIB_GET_READY_SEC = 2.0                # time to get ready before each gesture

CALIB_ORDER = ["rest", "extension", "flexion",  "radial_flexion", "ulnar_flexion"]

HEAD_EPOCHS = 4
HEAD_LR = 5e-4
BATCH_SIZE = 64

# Temperature tune (fast, differentiable)
TEMP_TUNE_STEPS = 150
TEMP_TUNE_LR = 0.05

# τ tuning (fast quantiles on val probs)
TAU_MIN = 0.05
TAU_MAX = 0.95
TAU_QUANTILES = np.linspace(0.20, 0.95, 10)  # per-class candidate thresholds
# Radial flexion is allowed to pass with lower variance / top prob
RADIAL_IDX = CLASS_NAMES.index("radial_flexion")  # should be 3

VAR_MIN_RADIAL = 0.005          # allow flatter distribution for radial
VAR_REQUIRE_TOP_RADIAL = 0.55   # allow smaller peak prob for radial
# ---------------- Calibration mode ----------------
USE_AUTOCAL = True   # True  = run full guided calibration
                     # False = skip calibration; use preset T_STAR / TAU_STAR

# ---------------- Serial parsing ----------------
def parse_line(line: str):
    s = line.strip()
    if len(s) < 3 or not s[1].isdigit():
        return None
    tag = s[0]
    parts = [p.strip() for p in s.split(',')]
    try:
        K = int(s[1])
    except ValueError:
        return None
    if tag == 'e':
        if len(parts) < 10: return None
        try:
            t_us = int(parts[1])
            vals = [int(p) for p in parts[2:10]]
        except ValueError:
            return None
        return ('e', K, t_us, vals)
    if tag == 'i':
        if len(parts) < 12: return None
        try:
            t_us = int(parts[1])
            qw,qx,qy,qz = [int(parts[i]) for i in range(2,6)]
        except ValueError:
            return None
        return ('i', K, t_us, (qw,qx,qy,qz))
    return None

def hstate_rx_thread(stop_event: threading.Event, q_hstate: queue.Queue):
    """
    Listens for float32 hstate values from the C++ app on 127.0.0.1:HSTATE_PORT
    and pushes them into q_hstate (non-blocking).

    Additionally:
    - If no packet has been seen for HSTATE_FRESH_S seconds and the last
      known hstate != 1, we synthesize hstate=1 ("nothing") and send it.
    """
    global _last_hstate_rx_wall, _last_hstate_value

    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", HSTATE_PORT))
        sock.setblocking(False)
        print(f"[HSTATE] Listening on 127.0.0.1:{HSTATE_PORT}")
    except Exception as e:
        print(f"[HSTATE] Failed to bind: {e}")
        if sock is not None:
            sock.close()
        return

    while not stop_event.is_set():
        # -------- 1) Try to receive a packet (non-blocking) --------
        try:
            data, addr = sock.recvfrom(1024)
        except BlockingIOError:
            data = None
        except OSError:
            data = None

        now_s = time.time()
        age = now_s - _last_hstate_rx_wall

        # -------- 2) Silence watchdog: if too old → force hstate=1 --------
        if age > HSTATE_FRESH_S and _last_hstate_value != 1:
            try:
                q_hstate.put_nowait(1.0)
            except queue.Full:
                # drop oldest and retry once
                try:
                    _ = q_hstate.get_nowait()
                    q_hstate.put_nowait(1.0)
                except queue.Empty:
                    pass
            on_hstate_update(1)
            _last_hstate_value = 1
            # Optionally set this, if you want to "ack" the synthetic event:
            # _last_hstate_rx_wall = now_s

        # -------- 3) Real packet arrived --------
        if data and len(data) >= 4:
            (hstate,) = struct.unpack("<f", data[:4])

            # Push raw float to queue for Feather forwarding
            try:
                q_hstate.put_nowait(hstate)
            except queue.Full:
                try:
                    _ = q_hstate.get_nowait()
                except queue.Empty:
                    pass
                try:
                    q_hstate.put_nowait(hstate)
                except queue.Full:
                    pass  # still full → drop

            # Update vib clamp
            try:
                h_int = int(round(hstate))
                on_hstate_update(h_int)
            except Exception:
                pass

            # Update "last seen" info
            _last_hstate_rx_wall = now_s
            _last_hstate_value = int(round(hstate))

        # -------- 4) Avoid busy spin --------
        time.sleep(0.001)

    if sock is not None:
        sock.close()
    print("[HSTATE] Thread exiting")



def serial_reader(stop_event: threading.Event, q: queue.Queue, q_hstate: queue.Queue):
    """
    Robust serial reader with auto-reopen:
    - Keeps trying to open PORT while stop_event is not set.
    - If a read/write error occurs (USB unplug, Feather reset, PermissionError, etc.),
      it closes the port and retries after a short delay.
    - Logs a concise message once per failure cycle instead of spamming.
    """
    import serial

    REOPEN_DELAY_S = 1.0     # delay before trying to reopen after a failure
    HSTATE_MIN_DT   = 0.02   # send at most 50 Hz

    while not stop_event.is_set():
        ser = None
        try:
            ser = serial.Serial(
                PORT,
                BAUD,
                timeout=0.1,      # read timeout
                write_timeout=0   # NON-BLOCKING write
            )
            print(f"[Serial] Opened {PORT} @ {BAUD}")
        except Exception as e:
            # Opening failed (device not present, in use, etc.)
            print(f"[Serial] Open failed on {PORT}: {e}. Retrying in {REOPEN_DELAY_S:.1f}s...")
            if stop_event.wait(REOPEN_DELAY_S):
                break
            continue

        last_hstate_sent = 0.0

        try:
            with ser:
                while not stop_event.is_set():
                    # ---------- 1) Normal EMG/IMU read ----------
                    try:
                        line = ser.readline().decode("utf-8", errors="ignore")
                    except (serial.SerialException, OSError, PermissionError) as e:
                        # Hard error: port likely gone or unusable
                        print(f"[Serial] Port error on {PORT} during read: {e}. Closing and will retry.")
                        break
                    except Exception:
                        # Soft glitch: ignore this line
                        line = ""

                    if line:
                        try:
                            item = parse_line(line)
                            if item:
                                q.put_nowait(item)
                        except queue.Full:
                            # Drop oldest to avoid blocking
                            try:
                                _ = q.get_nowait()
                            except Exception:
                                pass
                        except Exception:
                            # Parse or queue glitch → ignore
                            pass

                    # ---------- 2) Forward any pending hstate to Feather ----------
                    hstate = None
                    try:
                        # Drain queue, keep only the latest hstate
                        while True:
                            hstate = q_hstate.get_nowait()
                    except queue.Empty:
                        pass

                    if ENABLE_HSTATE_TX and hstate is not None:
                        now = time.time()
                        if now - last_hstate_sent >= HSTATE_MIN_DT:
                            frame = b"\xAA\x55" + struct.pack("<f", float(hstate))
                            try:
                                ser.write(frame)
                            except (serial.SerialException, OSError, PermissionError) as e:
                                # Treat PermissionError as a hard USB/driver error
                                print(f"[Serial] Port error on {PORT} during write: {e}. Closing and will retry.")
                                break
                            except Exception as e:
                                # Non-fatal glitch: log once if you care, or silence
                                # print(f"[Serial] hstate write failed (non-fatal): {e}")
                                pass
                            last_hstate_sent = now

                    # avoid busy loop if nothing is happening
                    time.sleep(0.001)

        finally:
            print(f"[Serial] Port {PORT} closed.")

        # If we're stopping, don't try to reopen
        if stop_event.is_set():
            break

        # Short delay before reopen attempt
        print(f"[Serial] Will retry opening {PORT} in {REOPEN_DELAY_S:.1f}s...")
        if stop_event.wait(REOPEN_DELAY_S):
            break

    print("[Serial] Reader thread exiting.")


# ---------------- Merging ----------------
class StreamMerger:
    def __init__(self, fs_agg=200.0):
        self.buffers = [deque() for _ in range(N_STREAMS)]
        self.expected = 0
        self.sample_index = 0
        self.fs = fs_agg
        self.t_samples = deque(maxlen=int(DISPLAY_SEC*fs_agg)*2)
        self.y_ch = [deque(maxlen=int(DISPLAY_SEC*fs_agg)*2) for _ in range(N_CH)]
        self.skips = 0
    def push(self, s, t_us, vals):
        self.buffers[s].append((t_us, vals))
    def merge_some(self, max_merge=2000):
        merges = 0
        while merges < max_merge:
            if any(len(self.buffers[s]) == 0 for s in range(N_STREAMS)):
                break
            s = self.expected
            t_us, vals = self.buffers[s].popleft()
            t = self.sample_index / self.fs
            self.t_samples.append(t)
            for ch in range(N_CH):
                self.y_ch[ch].append(vals[ch])
            self.sample_index += 1
            self.expected = (self.expected + 1) % N_STREAMS
            merges += 1
        lens = [len(self.buffers[s]) for s in range(N_STREAMS)]
        if max(lens) - min(lens) > 50:
            self.expected = int(np.argmax(lens))
            self.skips += 1
    def pull_latest_block(self, needed):
        if len(self.y_ch[0]) < needed:
            return None
        X = np.stack([np.fromiter(self.y_ch[ch], dtype=np.float32)[-needed:] for ch in range(N_CH)], axis=0)
        return X

# ---------------- Preprocess ----------------
def moving_rms(x: np.ndarray, win: int) -> np.ndarray:
    if win <= 1:
        return np.sqrt(x)
    pad = win - 1
    x2 = x**2
    csum = np.cumsum(np.pad(x2, ((0,0),(pad,0))), axis=1)
    wsum = csum[:, win:] - csum[:, :-win]
    left = np.repeat(wsum[:, :1], pad, axis=1)
    rms = np.sqrt(np.concatenate([left, wsum], axis=1) / float(win))
    return rms

class StatefulFilters:
    def __init__(self, fs, use_filters=True):
        self.fs = fs
        self.use = use_filters
        self.have_scipy = False
        self.bp = None
        self.notch = None
        self.zi_bp = None
        self.zi_notch = None
        if self.use:
            try:
                from scipy.signal import butter, iirnotch, lfilter_zi
                self.have_scipy = True
                b_bp, a_bp = butter(BP_ORDER, [BP_LO/(fs/2.0), BP_HI/(fs/2.0)], btype='band')
                self.bp = (b_bp, a_bp)
                if NOTCH_Q is not None:
                    b_n, a_n = iirnotch(50.0/(fs/2.0), NOTCH_Q)
                    self.notch = (b_n, a_n)
                self.zi_bp = [lfilter_zi(b_bp, a_bp) * 0.0 for _ in range(N_CH)] if self.bp else [None]*N_CH
                self.zi_notch = [lfilter_zi(*self.notch) * 0.0 for _ in range(N_CH)] if self.notch else [None]*N_CH
            except Exception as e:
                warnings.warn(f"[Filters] SciPy unavailable or failed ({e}); proceeding without filters.")
                self.have_scipy = False
                self.use = False
    def apply(self, x: np.ndarray) -> np.ndarray:
        if not self.use or not self.have_scipy:
            return x
        from scipy.signal import lfilter
        y = x
        if self.bp is not None:
            b,a = self.bp
            y_f = np.empty_like(y)
            for ch in range(N_CH):
                y_f[ch], self.zi_bp[ch] = lfilter(b, a, y[ch], zi=self.zi_bp[ch])
            y = y_f
        if self.notch is not None:
            b,a = self.notch
            y_f = np.empty_like(y)
            for ch in range(N_CH):
                y_f[ch], self.zi_notch[ch] = lfilter(b, a, y[ch], zi=self.zi_notch[ch])
            y = y_f
        return y

class Preprocessor:
    def __init__(self, fs, win_ms, env_ms):
        self.fs = fs
        self.win_samp = int(round(fs * win_ms / 1000.0))
        self.env_win = int(max(1, round(fs * env_ms / 1000.0)))
        self.filters = StatefulFilters(fs, USE_FILTERS)
        self.baseline = np.ones(N_CH, dtype=np.float32)
        self.have_baseline = False
        self._warm_envelopes = []
    def warmup_baseline(self, raw_win: np.ndarray):
        env = self._compute_envelope(raw_win)
        self._warm_envelopes.append(env)
        need_warm = int(BASELINE_WARMUP_SEC * self.fs / max(1, self.win_samp//2))
        if len(self._warm_envelopes) >= max(6, need_warm):
            big = np.concatenate(self._warm_envelopes, axis=1)
            base = np.median(big, axis=1)
            base[base == 0] = 1.0
            self.baseline = base.astype(np.float32)
            self.have_baseline = True
            self._warm_envelopes.clear()
    def _compute_envelope(self, raw_win: np.ndarray) -> np.ndarray:
        x = raw_win - raw_win.mean(axis=1, keepdims=True)
        x = self.filters.apply(x)
        x = np.abs(x)
        env = moving_rms(x, self.env_win)
        return env
    def process(self, raw_win: np.ndarray) -> np.ndarray:
        env = self._compute_envelope(raw_win)
        env_norm = env / (self.baseline[:, None] + 1e-6)
        return env_norm.astype(np.float32)

# ---------------- Model & postproc ----------------
def undot(d):
    out = {}
    for k, v in d.items():
        cur = out
        parts = k.split(".")
        for p in parts[:-1]:
            cur = cur.setdefault(p, {})
        cur[parts[-1]] = v
    return out

def load_model(run_dir: str):
    import torch
    sys.path.insert(0, os.path.dirname(run_dir))
    from networks import EMGModel
    device = torch.device("cuda:0" if (USE_GPU and torch.cuda.is_available()) else "cpu")
    with open(os.path.join(run_dir, "hyperparameters.json"), "r", encoding="utf-8") as f:
        hp = undot(json.load(f))
    model = EMGModel(hp).to(device)
    ckpt = torch.load(os.path.join(run_dir, "best_model_state.pth"), map_location=device)
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()
    native_classes = hp.get("model", {}).get("num_classes", None)
    print(f"[Model] Loaded. Native classes: {native_classes}, val_loss={ckpt.get('val_loss')}")
    return model, device

def softmax_np(x, axis=-1):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / np.sum(e, axis=axis, keepdims=True)

def merge_probs_9_to_5(p9: np.ndarray) -> np.ndarray:
    """
    Merge native 9-class probabilities into 5 classes:
      0: REST (+ mild flexion)
      1: extension
      2: flexion (strong)
      3: radial_flexion
      4: ulnar_flexion
    """
    out = np.zeros((p9.shape[0], 5), dtype=np.float32)

    # REST: 0,1,3,5,7 (mild flexion goes here now)
    out[:, 0] = p9[:, 0] + p9[:, 1] + p9[:, 3] + p9[:, 5] + p9[:, 7]

    # extension: 2
    out[:, 1] = p9[:, 2]

    # flexion (strong): 4
    out[:, 2] = p9[:, 4]

    # radial_flexion: 6
    out[:, 3] = p9[:, 6]

    # ulnar_flexion: 8
    out[:, 4] = p9[:, 8]

    return out


def predict_with_tau(P5: np.ndarray, tau: np.ndarray, fallback_idx: int) -> np.ndarray:
    meets = P5 >= tau.reshape(1, -1)
    any_pass = meets.any(axis=1)
    scores = P5.copy()
    scores[~meets] = -1.0
    yhat = scores.argmax(axis=1)
    yhat[~any_pass] = fallback_idx
    return yhat


# ---------------- Smoother ----------------
class ProbSmoother:
    def __init__(self, K, hop_s=0.03, tau_s=0.20,
                 adapt=True, top_lo=0.35, top_hi=0.75, k_median=1):
        import collections
        self.K = K
        self.hop_s = float(hop_s)
        self.tau_s = max(1e-3, float(tau_s))
        self.adapt = bool(adapt)
        self.top_lo = float(top_lo)
        self.top_hi = float(top_hi)
        self.alpha_base = 1.0 - np.exp(-self.hop_s / self.tau_s)
        self.P = None
        self.k_median = max(1, int(k_median))
        self.hist = [collections.deque(maxlen=self.k_median) for _ in range(K)]
    def _alpha_adaptive(self, top_prob):
        if not self.adapt: return self.alpha_base
        if top_prob <= self.top_lo: mult = 0.65
        elif top_prob >= self.top_hi: mult = 1.25
        else:
            r = (top_prob - self.top_lo) / max(1e-6, (self.top_hi - self.top_lo))
            mult = 0.65 + r * (1.25 - 0.65)
        alpha = self.alpha_base * mult
        return float(np.clip(alpha, 0.02, 0.50))
    def step(self, P_new: np.ndarray) -> np.ndarray:
        P_new = np.asarray(P_new, dtype=np.float32)
        P_new /= max(1e-9, P_new.sum())
        if self.P is None:
            self.P = P_new.copy()
            for k in range(self.K):
                self.hist[k].clear()
                self.hist[k].append(float(P_new[k]))
            return self.P.copy()
        alpha = self._alpha_adaptive(float(P_new.max()))
        self.P = (1.0 - alpha) * self.P + alpha * P_new
        if self.k_median > 1:
            for k in range(self.K):
                self.hist[k].append(float(self.P[k]))
                self.P[k] = np.median(self.hist[k])
        s = float(self.P.sum())
        if s > 0: self.P /= s
        return self.P.copy()

# ---------------- UI ----------------
class HistogramUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EMG → Model → Smoothed Probabilities (5 classes)")
        cw = QtWidgets.QWidget(); self.setCentralWidget(cw)
        v = QtWidgets.QVBoxLayout(cw)

        self.prompt = QtWidgets.QLabel("Calibrating…")
        self.prompt.setAlignment(QtCore.Qt.AlignCenter)
        self.prompt.setStyleSheet("font-size: 20px; padding: 6px;")
        v.addWidget(self.prompt)

        self.plot = pg.PlotWidget()
        self.plot.setLabel('left', 'Probability'); self.plot.setLabel('bottom', 'Class')
        self.plot.setYRange(0, 1.05); self.plot.showGrid(x=True, y=True, alpha=0.25)
        v.addWidget(self.plot)

        self.x_idx = np.arange(len(CLASS_NAMES))
        self.width = 0.7
        self.bg = pg.BarGraphItem(x=self.x_idx, height=np.zeros_like(self.x_idx, dtype=float), width=self.width)
        self.plot.addItem(self.bg)

        self.text_items = []
        for i, name in enumerate(CLASS_NAMES):
            ti = pg.TextItem(name, anchor=(0.5, 1.5), angle=0)
            self.plot.addItem(ti); ti.setPos(self.x_idx[i], 0.0); self.text_items.append(ti)

        self.pred_label = QtWidgets.QLabel("Prediction: —")
        self.pred_label.setAlignment(QtCore.Qt.AlignCenter)
        self.pred_label.setStyleSheet("font-size: 22px; font-weight: 600; padding: 6px;")
        v.addWidget(self.pred_label)

    def set_prompt(self, text):
        self.prompt.setText(text)

    def update_hist(self, probs5: np.ndarray, pred_idx: int):
        h = probs5.astype(float)
        self.bg.setOpts(height=h)
        for i, ti in enumerate(self.text_items):
            ti.setText(f"{CLASS_NAMES[i]}\n{h[i]:.2f}")
            ti.setPos(self.x_idx[i], max(0.02, h[i]) + 0.03)
        self.pred_label.setText(f"Prediction: {CLASS_NAMES[pred_idx]}")

# ---------------- Calibration containers ----------------
@dataclass
class CalibBatch:
    X: list   # list of np.ndarray (8,T) envelope-normalized windows
    y: list   # list of int in 0..4 (merged class ids)

# ---------------- Controller ----------------
class AppController:
    def __init__(self):
        self.q = queue.Queue(maxsize=8192)
        self.stop_event = threading.Event()

        # serial reader (COM10) + hstate queue
        self.reader_thread = threading.Thread(
            target=serial_reader,
            args=(self.stop_event, self.q, hstate_queue),
            daemon=True
        )

        # NEW: hstate UDP listener thread
        self.hstate_thread = threading.Thread(
            target=hstate_rx_thread,
            args=(self.stop_event, hstate_queue),
            daemon=True
        )


        # merger/windowing
        self.merger = StreamMerger(fs_agg=FS_AGG)
        self.win_samp = int(round(FS_AGG * WINDOW_MS / 1000.0))
        self.hop_samp = int(round(FS_AGG * HOP_MS / 1000.0))
        self._slide_buf = np.zeros((N_CH, 0), dtype=np.float32)

        # preprocessing
        self.pp = Preprocessor(FS_AGG, WINDOW_MS, ENV_MS)

        # model
        self.model, self.device = load_model(RUN_DIR)

        # runtime params (will be updated by calibration)
        self.T_star = float(T_STAR)
        self.tau_star = TAU_STAR.copy()

        self.calibrated = False
        self.collecting = False

        # voting state (majority vote on top of τ-gated prediction)
        self.vote_hist = deque(maxlen=VOTE_WINDOW)
        self.stable_pred = IDX_REST

        # -------- class-level low-pass (dwell/hysteresis) --------
        self.final_pred = IDX_REST
        self.last_switch_wall = time.time()



        # smoother
        self.smoother = ProbSmoother(
            K=5, hop_s=HOP_MS/1000.0, tau_s=SMOOTH_TAU_S,
            adapt=SMOOTH_ADAPT, top_lo=SMOOTH_TOP_LO, top_hi=SMOOTH_TOP_HI, k_median=SMOOTH_MEDIAN_K,
        )

        # UI
        self.ui = HistogramUI()

        # timers
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_pitch_deg = 0.0
        self.last_imu_wall = 0.0

        # launch calibration thread
        self.calib_thread = threading.Thread(target=self.run_calibration, daemon=True)
        # --- NEW: EMG last-seen wall clock timestamp ---
        self.last_emg_wall = 0.0
        
        # --- Performance evaluation state ---
        self.measure_perf = MEASURE_PERFORMANCE
        self.eval_active = False          # whether we are currently logging eval frames
        self.eval_current_true = None     # current ground-truth class id during eval

        # Global logs (all reps)
        self.eval_true_labels = []        # list[int]
        self.eval_pred_labels = []        # list[int]

        # Per-repetition metrics
        self.eval_rep_acc = []            # list[float], accuracy per repetition
        self.eval_rep_cm = []             # list[np.ndarray of shape (K,K)]



    # -------- Threads control --------
    def start(self):
        self.reader_thread.start()
        self.hstate_thread.start()
        self.calib_thread.start()

    def stop(self):
        self.stop_event.set()
        try: self.reader_thread.join(timeout=0.5)
        except Exception: pass
        try: self.hstate_thread.join(timeout=0.5)
        except Exception: pass
        try: self.sock.close()
        except Exception: pass


    # -------- Queue drain --------
    def _drain_queue(self, max_items=4000):
        drained = 0; now = time.time()
        while drained < max_items and not self.q.empty():
            item = self.q.get_nowait(); drained += 1
            tag = item[0]
            if tag == 'e':
                _, s, t_us, vals = item
                self.merger.push(s, t_us, vals)
                # --- NEW: mark EMG as fresh ---
                self.last_emg_wall = now
            else:
                _, s, t_us, (qw,qx,qy,qz) = item
                qw,qx,qy,qz = (qw/QUAT_SCALE, qx/QUAT_SCALE, qy/QUAT_SCALE, qz/QUAT_SCALE)
                _, pitch, _ = quat_to_rpy(qw,qx,qy,qz)
                self.last_pitch_deg = float(pitch); self.last_imu_wall = now

    # -------- Model forward --------
    def _forward_logits(self, env_win_norm: np.ndarray) -> np.ndarray:
        import torch
        x = torch.from_numpy(env_win_norm[None, ...]).to(self.device, non_blocking=True)
        with torch.no_grad():
            logits = self.model(x).detach().cpu().numpy().astype(np.float64)  # (1,C)
        return logits[0]

    # -------- Tick (UI + streaming) --------
    def tick(self):
        self._drain_queue(); self.merger.merge_some()
        latest = self.merger.pull_latest_block(self.win_samp)
        if latest is None: return

        if self._slide_buf.shape[1] == 0:
            self._slide_buf = latest.copy()
        else:
            append = latest[:, -max(0, self.hop_samp):]
            self._slide_buf = np.concatenate([self._slide_buf[:, self.hop_samp:], append], axis=1)
        if self._slide_buf.shape[1] < self.win_samp: return

        raw_win = self._slide_buf[:, -self.win_samp:]

        # Baseline warmup
        if not self.pp.have_baseline:
            self.pp.warmup_baseline(raw_win)
            self.ui.set_prompt("Calibrating… baseline")
            self.ui.update_hist(np.zeros(5, dtype=float), pred_idx=IDX_REST)
            return

        # Compute envelope and normalize manually so we can also adapt baseline later
        env = self.pp._compute_envelope(raw_win)  # (8, T)
        env_norm = env / (self.pp.baseline[:, None] + 1e-6)
        env_norm = env_norm.astype(np.float32)


        # During calibration, only update prompt/hist for feedback; don't send outputs
        if not self.calibrated:
            self.ui.update_hist(np.zeros(5, dtype=float), pred_idx=IDX_REST)
            return

        # Normal runtime
        logits_native = self._forward_logits(env_norm)
        P_native_cal = softmax_np(logits_native / self.T_star)
        Cnative = P_native_cal.shape[0]
        if Cnative == 9:
            P5_raw = merge_probs_9_to_5(P_native_cal[None, :])[0]
        elif Cnative == 5:
            P5_raw = P_native_cal.astype(np.float32)
        else:
            return

        P5_smooth = self.smoother.step(P5_raw)
        y_tau = predict_with_tau(P5_smooth[None, :], self.tau_star, IDX_REST)[0]

        # IMU pitch gate to REST
        gate_active = (time.time() - self.last_imu_wall) < IMU_FRESH_S
        if gate_active and abs(self.last_pitch_deg) > 25.0:
            y_tau = IDX_REST
            P5_smooth = np.array([1., 0., 0., 0., 0.], dtype=np.float32)

        # Variance-based low-confidence gate
        if USE_VAR_GATE:
            y_tau, P5_smooth = self._apply_variance_gate(P5_smooth, int(y_tau))

        # --- NEW: EMG COM / stream watchdog -> hard REST ---
        now = time.time()
        emg_fresh = (now - self.last_emg_wall) < EMG_FRESH_S
        if not emg_fresh:
            # No EMG for too long → force REST and reset voting state
            y_tau = IDX_REST
            P5_smooth = np.array([1., 0., 0., 0., 0.], dtype=np.float32)
            self.vote_hist.clear()
            self.stable_pred = IDX_REST

        # ---- Voting layer: majority over recent τ-gated predictions ----
        y_final = self._update_voting(int(y_tau))

        # ---- Class-level low-pass (dwell time) ----
        y_slow = self._lowpass_decision(int(y_final))

        # Clamp output class to REST while vibration is active
        output_class = get_output_class(int(y_slow))
        # ----- Slow baseline adaptation on confident REST -----
        # if output_class == IDX_REST and P5_smooth[0] > 0.8 and self.pp.have_baseline:
        #     # Robust estimate of current "rest" envelope per channel
        #     env_med = np.median(env, axis=1).astype(np.float32)  # (8,)

        #     # Very slow EMA update (e.g. alpha = 0.01)
        #     alpha = 0.01
        #     self.pp.baseline = (1.0 - alpha) * self.pp.baseline + alpha * env_med


        # NEW: if evaluation is active, log ground-truth vs output_class
        if self.eval_active and (self.eval_current_true is not None):
            self.eval_true_labels.append(int(self.eval_current_true))
            self.eval_pred_labels.append(int(output_class))


        # send
        try:
            self.sock.sendto(f"{int(output_class)}\n".encode("utf-8"), CLASS_SINK)
        except Exception:
            pass

        # UI (optional: show what the robot actually gets)
        self.ui.set_prompt("Ready")
        self.ui.update_hist(P5_smooth, int(output_class))



    @staticmethod
    def _enforce_tau_structure(tau,
                            min_gap=0.05,
                            min_rest=0.05,
                            min_nonrest=0.80,
                            tau_max=0.95):
        """
        Enforce:
        - τ_rest >= min_rest
        - τ_nonrest >= max(min_nonrest, τ_rest + min_gap)
        - τ_k <= tau_max    for all k
        - τ_nonrest_k >= τ_rest + min_gap
        """
        tau = np.array(tau, dtype=np.float32)

        # 1) Rest class (index 0): at least min_rest, but not above tau_max
        tau_rest = float(np.clip(tau[0], min_rest, tau_max))

        # 2) Non-rest must be at least:
        #    (a) min_nonrest
        #    (b) rest + min_gap
        nonrest_floor = max(min_nonrest, tau_rest + min_gap)

        tau_nonrest = tau[1:]
        tau_nonrest = np.clip(np.maximum(tau_nonrest, nonrest_floor), nonrest_floor, tau_max)

        tau[0] = tau_rest
        tau[1:] = tau_nonrest
        return tau
    
    def _update_voting(self, y_tau: int) -> int:
        """
        Update majority-vote buffer with the latest τ-gated prediction (y_tau)
        and return a 'stable' class index.

        Logic:
        - Append y_tau to a fixed-length history.
        - Compute majority class in the history.
        - Only switch stable_pred if:
            * we have at least VOTE_MIN_FRAMES samples, and
            * majority class covers at least VOTE_MAJ_FRAC of the window.
        - Otherwise keep previous stable_pred.
        """
        self.vote_hist.append(int(y_tau))
        hist = list(self.vote_hist)
        total = len(hist)

        # if buffer is very small, just use y_tau directly initially
        if total < VOTE_MIN_FRAMES:
            self.stable_pred = int(y_tau)
            return self.stable_pred

        counts = np.bincount(hist, minlength=len(CLASS_NAMES))
        maj = int(counts.argmax())
        maj_count = int(counts[maj])

        # majority must be strong enough to change
        if maj_count >= max(2, int(np.ceil(VOTE_MAJ_FRAC * total))):
            self.stable_pred = maj

        return self.stable_pred

    def _apply_variance_gate(self, P5_smooth: np.ndarray, y_tau: int):
        """
        Apply a low-confidence gate based on how 'flat' the probability
        distribution is across classes.

        We use *class-dependent* thresholds:
        - For radial_flexion we are more forgiving: lower VAR_MIN and lower top-prob requirement.
        - For all other non-REST classes we use the global stricter thresholds.
        """
        P = np.asarray(P5_smooth, dtype=np.float32)
        # safety: re-normalize
        s = float(P.sum())
        if s > 0:
            P = P / s

        var = float(P.var())
        top = float(P.max())

        # Pick thresholds depending on candidate class
        if y_tau == RADIAL_IDX:
            vmin = VAR_MIN_RADIAL
            top_req = VAR_REQUIRE_TOP_RADIAL
        elif y_tau == IDX_REST:
            # For REST we don't gate by variance at all; just accept it
            return IDX_REST, np.array([1., 0., 0., 0., 0.], dtype=np.float32)
        else:
            vmin = VAR_MIN
            top_req = VAR_REQUIRE_TOP

        # If confidence is low (flat probs OR low top prob), force REST
        if var < vmin or top < top_req:
            return IDX_REST, np.array([1., 0., 0., 0., 0.], dtype=np.float32)

        # otherwise leave as-is
        return int(y_tau), P


    # -------- Calibration pipeline --------
    def run_calibration(self):
        """
        1) Wait for baseline to be ready
        2) (Optionally) guided capture per class (CALIB_ORDER); store env windows & labels
        3) Head-only fine-tune (few epochs)
        4) Learn T* on a held-out split (Adam)
        5) Choose τ per class from validation P5 quantiles
        6) Set calibrated flag
        """
        global ENABLE_HSTATE_TX   # declare once, at the top

        # 1) wait baseline
        t0 = time.time()
        self.ui.set_prompt("Calibrating… baseline")
        while not self.pp.have_baseline and not self.stop_event.is_set():
            time.sleep(0.05)
        # small extra stabilization
        time.sleep(max(0.0, CALIB_BASELINE_SEC - (time.time() - t0)))

        # ---- Fast path: skip calibration, use preset T*/τ ----
        if not USE_AUTOCAL:
            self.calibrated = True
            self.ui.set_prompt("Ready (precalibrated)")
            print("[Calibration] USE_AUTOCAL=False → skipping guided calibration.")
            print(f"[Calibration] Using preset T*={self.T_star:.3f}, τ={np.round(self.tau_star, 3)}")
            # Now allow hstate to be forwarded to the Feather
            ENABLE_HSTATE_TX = True
            print("[HSTATE] Enabling serial hstate TX to Feather (precalibrated).")
            return

        # 2) guided capture
        calib = self._capture_guided()
        if len(calib.X) < 20:
            print("[Calibration] Not enough samples; using defaults.")
            self.calibrated = True
            self.ui.set_prompt("Ready (defaults)")
            # still allow hstate forwarding
            ENABLE_HSTATE_TX = True
            print("[HSTATE] Enabling serial hstate TX to Feather (defaults).")
            return

        # build train/val split (per class 80/20)
        X = np.stack(calib.X, axis=0)  # (N,8,T)
        y = np.array(calib.y, dtype=np.int64)  # 0..4
        tr_idx, va_idx = self._split_per_class(y, train_frac=0.8)
        Xtr, ytr = X[tr_idx], y[tr_idx]
        Xva, yva = X[va_idx], y[va_idx]

        # 3) head-only fine-tune
        self.ui.set_prompt("Calibrating… tuning head")
        self._finetune_head(Xtr, ytr)

        # 4) learn temperature on val logits
        self.ui.set_prompt("Calibrating… tuning T*")
        logits_val = self._batch_logits(Xva)  # (N, Cnative)
        T_opt = self._learn_temperature(logits_val, yva)
        self.T_star = float(T_opt)

        # 5) τ handling
        if USE_TAU_DEFAULTS:
            # Keep whatever was in self.tau_star (from TAU_STAR in globals)
            self.ui.set_prompt("Calibrating… using default τ")
            print(f"[Calibration] Using default τ: {np.round(self.tau_star,3)}")
        else:
            # τ from quantiles on validation probs (with T*)
            self.ui.set_prompt("Calibrating… tuning τ")
            P5_val = self._to_P5_probs(logits_val, self.T_star)
            tau = self._choose_tau_by_quantiles(P5_val, yva)

            # --- Enforce the desired structure: non-rest τ > rest τ ---
            tau = self._enforce_tau_structure(
                tau,
                min_gap=0.05,
                min_rest=0.35,
                min_nonrest=0.50,
                tau_max=0.95
            )
            self.tau_star = tau.astype(np.float32)

        # done
        self.calibrated = True
        self.ui.set_prompt("Ready")
        print(f"[Calibration] Done. T*={self.T_star:.3f}  τ={np.round(self.tau_star,3)}")

        # Now allow hstate to be forwarded to the Feather
        ENABLE_HSTATE_TX = True
        print("[HSTATE] Enabling serial hstate TX to Feather.")

        # OPTIONAL: run guided online evaluation with the full runtime pipeline
        if self.measure_perf and not self.stop_event.is_set():
            self._run_online_eval()



    def _capture_guided(self) -> CalibBatch:
        """
        Displays prompts and records (8,T) windows while user holds each pose.
        Uses current preprocessor and sliding window.
        """
        batch = CalibBatch(X=[], y=[])
        label_to_id = {n: i for i, n in enumerate(CLASS_NAMES)}

        pause_between = CALIB_REST_BETWEEN_SEC

        for name in CALIB_ORDER:
            cls = label_to_id[name]

            # --- per-class hold time: radial_flexion gets more data ---
            if name == "radial_flexion":
                hold_sec = CALIB_PER_CLASS_SEC_RADIAL
            else:
                hold_sec = CALIB_PER_CLASS_SEC

            # ---------- Get-ready phase ----------
            get_ready_msg = f"Get ready: {name} (starts in {CALIB_GET_READY_SEC:.0f}s)"
            print(f"[Calib] {get_ready_msg}")
            self.ui.set_prompt(get_ready_msg)

            t_ready = time.time()
            while (time.time() - t_ready) < CALIB_GET_READY_SEC and not self.stop_event.is_set():
                time.sleep(0.01)

            # ---------- Hold phase ----------
            hold_msg = f"Hold: {name} ({hold_sec:.0f}s)"
            print(f"[Calib] {hold_msg}")
            self.ui.set_prompt(hold_msg)

            t_start = time.time()
            while (time.time() - t_start) < hold_sec and not self.stop_event.is_set():
                latest = self.merger.pull_latest_block(self.win_samp)
                if latest is not None and self.pp.have_baseline:
                    raw_win = latest[:, -self.win_samp:]
                    env_norm = self.pp.process(raw_win)
                    batch.X.append(env_norm)
                    batch.y.append(cls)

                # approx hop cadence
                time.sleep(self.hop_samp / FS_AGG)

            # ---------- Relax / neutral pause ----------
            self.ui.set_prompt("Relax…")
            print("[Calib] Relax…")
            t_relax = time.time()
            while (time.time() - t_relax) < pause_between and not self.stop_event.is_set():
                time.sleep(0.01)

        return batch



    def _split_per_class(self, y, train_frac=0.8):
        tr_idx, va_idx = [], []
        for c in range(5):
            idxs = np.where(y == c)[0]
            if len(idxs) == 0: continue
            np.random.shuffle(idxs)
            k = int(round(train_frac * len(idxs)))
            tr_idx.extend(idxs[:k]); va_idx.extend(idxs[k:])
        return np.array(tr_idx, dtype=np.int64), np.array(va_idx, dtype=np.int64)

    def _finetune_head(self, Xtr, ytr):
        import torch
        from torch import nn, optim
        self.model.eval()
        # freeze all, unfreeze classifier
        for p in self.model.parameters(): p.requires_grad = False
        head = getattr(self.model, "classifier", None)
        assert head is not None, "Model must expose `classifier`"
        for p in head.parameters(): p.requires_grad = True

        ds_n = Xtr.shape[0]
        crit = nn.CrossEntropyLoss()
        opt = optim.Adam(head.parameters(), lr=HEAD_LR)

        steps_per_epoch = max(1, int(np.ceil(ds_n / BATCH_SIZE)))
        for ep in range(HEAD_EPOCHS):
            running = 0.0; n = 0
            perm = np.random.permutation(ds_n)
            for b in range(steps_per_epoch):
                sl = perm[b*BATCH_SIZE : (b+1)*BATCH_SIZE]
                xb = torch.from_numpy(Xtr[sl]).float().to(self.device)   # (B,8,T)
                yb = torch.from_numpy(ytr[sl]).long().to(self.device)    # (B,)
                opt.zero_grad()
                logits = self.model(xb)         # (B, Cnative=9 or 5); model expects (B,C,T)
                if logits.shape[1] == 9:
                    # train on merged 5 labels: NLL(log(P5))
                    P9 = torch.softmax(logits, dim=1)
                    M = self._merge_matrix_9x5_torch(self.device)
                    P5 = P9 @ M
                    loss = torch.nn.NLLLoss()(torch.log(P5.clamp_min(1e-9)), yb)
                else:
                    loss = crit(logits, yb)
                loss.backward(); opt.step()
                running += float(loss.item())*len(sl); n += len(sl)
            print(f"[Head] Epoch {ep+1}/{HEAD_EPOCHS}  Loss={running/max(1,n):.4f}")

        # keep eval mode
        self.model.eval()

    def _batch_logits(self, X):
        import torch
        outs = []
        with torch.no_grad():
            for i in range(0, X.shape[0], BATCH_SIZE):
                xb = torch.from_numpy(X[i:i+BATCH_SIZE]).float().to(self.device)
                lo = self.model(xb).detach().cpu().numpy()
                outs.append(lo)
        return np.concatenate(outs, axis=0)  # (N, Cnative)

    def _to_P5_probs(self, logits_native: np.ndarray, T: float) -> np.ndarray:
        Pnat = softmax_np(logits_native / float(T))
        if Pnat.shape[1] == 9:
            return merge_probs_9_to_5(Pnat)
        return Pnat.astype(np.float32)

    def _learn_temperature(self, logits_val: np.ndarray, y_true5: np.ndarray) -> float:
        """
        Optimize a single T on val set (fast Adam on NLL of merged P5).
        """
        import torch
        from torch import nn, optim
        device = self.device
        logits = torch.from_numpy(logits_val).float().to(device)
        targets = torch.from_numpy(y_true5.astype(np.int64)).to(device)
        logT = torch.tensor(np.log(max(1e-3, 0.9)), dtype=torch.float32, device=device, requires_grad=True)
        M = self._merge_matrix_9x5_torch(device)
        nll = nn.NLLLoss()
        opt = optim.Adam([logT], lr=TEMP_TUNE_LR)
        for _ in range(TEMP_TUNE_STEPS):
            opt.zero_grad()
            T = torch.exp(logT)
            scaled = logits / T
            if logits.shape[1] == 9:
                P9 = torch.softmax(scaled, dim=1)
                P5 = (P9 @ M).clamp_min(1e-12)
            else:
                P5 = torch.softmax(scaled, dim=1).clamp_min(1e-12)
            loss = nll(torch.log(P5), targets)
            loss.backward(); opt.step()
        T_opt = float(torch.exp(logT).clamp(1e-3, 10.0).item())
        return T_opt

    def _choose_tau_by_quantiles(self, P5_val: np.ndarray, y_true5: np.ndarray) -> np.ndarray:
        """
        For each class k, sweep quantile candidates of P5[:,k] as τ_k.
        Keep others at 0 (no min gate), but apply fallback-to-REST logic overall.
        Joint small grid for speed: coordinate ascent 1 pass.
        """
        tau = np.clip(np.median(P5_val, axis=0), TAU_MIN, TAU_MAX)  # init
        # one pass coordinate update
        for k in range(5):
            cand = np.clip(np.quantile(P5_val[:, k], TAU_QUANTILES), TAU_MIN, TAU_MAX)
            best_acc = -1.0; best = tau[k]
            for tk in cand:
                tau_try = tau.copy(); tau_try[k] = tk
                yhat = predict_with_tau(P5_val, tau_try, IDX_REST)
                acc = (yhat == y_true5).mean()
                if acc > best_acc:
                    best_acc = acc; best = tk
            tau[k] = best
        return tau
    
    def _run_online_eval(self):
        """
        Guided online evaluation using the *full* runtime pipeline:
        - IMU gate
        - τ gating
        - smoothing
        - voting
        - hstate/vibration clamp

        It reuses the GUI prompt to instruct you which gesture to hold.

        Records:
        - Global accuracy + confusion matrix over all repetitions.
        - Per-repetition accuracy + confusion matrix.
        """
        label_to_id = {n: i for i, n in enumerate(CLASS_NAMES)}

        print("\n[Eval] Measuring online performance with current pipeline.")
        print(f"[Eval] Reps: {EVAL_REPS}, hold per class: {EVAL_HOLD_SEC:.1f}s, rest between: {EVAL_REST_SEC:.1f}s.")

        # reset logs
        self.eval_true_labels = []
        self.eval_pred_labels = []
        self.eval_rep_acc = []
        self.eval_rep_cm = []

        for rep in range(EVAL_REPS):
            if self.stop_event.is_set():
                break

            print(f"\n[Eval] === Repetition {rep+1}/{EVAL_REPS} ===")

            # remember where this repetition starts in the global arrays
            rep_start_idx = len(self.eval_true_labels)

            for name in CALIB_ORDER:
                if self.stop_event.is_set():
                    break

                cls = label_to_id[name]

                # Get-ready phase
                msg = f"[Eval] Get ready for: {name}. Start in 2 seconds…"
                print(msg)
                self.ui.set_prompt(msg)
                t0 = time.time()
                while (time.time() - t0) < 2.0 and not self.stop_event.is_set():
                    time.sleep(0.01)

                # Hold phase
                msg = f"[Eval] HOLD {name} NOW for {EVAL_HOLD_SEC:.1f} s"
                print(msg)
                self.ui.set_prompt(msg)

                self.eval_current_true = cls
                self.eval_active = True

                t_hold = time.time()
                while (time.time() - t_hold) < EVAL_HOLD_SEC and not self.stop_event.is_set():
                    time.sleep(0.01)

                # stop logging for this class
                self.eval_active = False
                self.eval_current_true = None

                # Relax phase
                print("[Eval] Relax…")
                self.ui.set_prompt("Relax…")
                t_relax = time.time()
                while (time.time() - t_relax) < EVAL_REST_SEC and not self.stop_event.is_set():
                    time.sleep(0.01)

            # ----- per-repetition metrics -----
            rep_end_idx = len(self.eval_true_labels)
            if rep_end_idx > rep_start_idx:
                y_true_rep = np.array(self.eval_true_labels[rep_start_idx:rep_end_idx], dtype=np.int32)
                y_pred_rep = np.array(self.eval_pred_labels[rep_start_idx:rep_end_idx], dtype=np.int32)

                K = len(CLASS_NAMES)
                cm_rep = np.zeros((K, K), dtype=np.int64)
                for t, p in zip(y_true_rep, y_pred_rep):
                    if 0 <= t < K and 0 <= p < K:
                        cm_rep[t, p] += 1

                total_rep = int(cm_rep.sum())
                correct_rep = int(np.trace(cm_rep))
                acc_rep = (correct_rep / total_rep) if total_rep > 0 else 0.0

                self.eval_rep_acc.append(acc_rep)
                self.eval_rep_cm.append(cm_rep)

                print(f"\n[Eval] --- Rep {rep+1} summary ---")
                print(f"Frames in rep: {total_rep}")
                print(f"Accuracy rep {rep+1}: {acc_rep*100.0:.2f}%")
                self._print_confusion(cm_rep)
            else:
                print(f"[Eval] Rep {rep+1}: no frames recorded (skipped).")

        # Ensure flags are off
        self.eval_active = False
        self.eval_current_true = None

        # -------- Global metrics over all reps --------
        if len(self.eval_true_labels) == 0:
            print("[Eval] No frames recorded; skipping global metrics.")
            return

        y_true = np.array(self.eval_true_labels, dtype=np.int32)
        y_pred = np.array(self.eval_pred_labels, dtype=np.int32)

        K = len(CLASS_NAMES)
        cm = np.zeros((K, K), dtype=np.int64)
        for t, p in zip(y_true, y_pred):
            if 0 <= t < K and 0 <= p < K:
                cm[t, p] += 1

        total = int(cm.sum())
        correct = int(np.trace(cm))
        acc = (correct / total) if total > 0 else 0.0

        print("\n========== ONLINE EVALUATION RESULTS (GLOBAL) ==========")
        print(f"Total frames: {total}")
        print(f"Overall accuracy: {acc * 100.0:.2f}%\n")
        self._print_confusion(cm)

        # -------- Optional: save to disk for later analysis --------
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            out_path = os.path.join(
                RUN_DIR,
                f"online_eval_{timestamp}.npz"
            )

            # Stack per-rep confusion matrices if we have any
            rep_cm_stack = np.stack(self.eval_rep_cm, axis=0) if len(self.eval_rep_cm) > 0 else None
            rep_acc_arr  = np.array(self.eval_rep_acc, dtype=np.float32)

            np.savez(
                out_path,
                y_true=y_true,
                y_pred=y_pred,
                cm_global=cm,
                acc_global=np.array(acc, dtype=np.float32),
                rep_acc=rep_acc_arr,
                rep_cm=rep_cm_stack
            )
            print(f"\n[Eval] Saved evaluation results to:\n  {out_path}")
        except Exception as e:
            print(f"[Eval] Failed to save evaluation results: {e}")

        print("\n[Eval] Done.")


    def _print_confusion(self, cm: np.ndarray):
        """
        Pretty-print confusion matrix with class names.
        Rows = true, cols = predicted.
        """
        K = len(CLASS_NAMES)
        w = 15

        print("Confusion matrix (rows = true, cols = predicted):")
        header = " " * w + "".join(f"{name:>{w}}" for name in CLASS_NAMES)
        print(header)
        for i in range(K):
            row_name = f"{CLASS_NAMES[i]:<{w}}"
            row_vals = "".join(f"{int(cm[i, j]):{w}d}" for j in range(K))
            print(row_name + row_vals)


    def _lowpass_decision(self, candidate: int) -> int:
        """
        Low-pass filter on class decisions:
        - 'candidate' is the current stable class from voting.
        - We only allow a change if MIN_CLASS_HOLD_S has elapsed
          since the last accepted switch.
        This suppresses rapid back-and-forth flips between classes.
        """
        now = time.time()

        # First call safety
        if self.final_pred is None:
            self.final_pred = int(candidate)
            self.last_switch_wall = now
            return self.final_pred

        candidate = int(candidate)

        if candidate != self.final_pred:
            # Only accept switch after dwell time
            if (now - self.last_switch_wall) >= MIN_CLASS_HOLD_S:
                self.final_pred = candidate
                self.last_switch_wall = now

        return self.final_pred

    @staticmethod
    def _merge_matrix_9x5_torch(device):
        import torch
        M = torch.zeros(9, 5, device=device)

        # REST (+ mild flexion): 0,1,3,5,7
        M[0, 0] = 1
        M[1, 0] = 1
        M[3, 0] = 1   # mild flexion → REST
        M[5, 0] = 1
        M[7, 0] = 1

        # extension: 2
        M[2, 1] = 1

        # flexion (strong): 4
        M[4, 2] = 1

        # radial_flexion: 6
        M[6, 3] = 1

        # ulnar_flexion: 8
        M[8, 4] = 1

        return M


# ---------------- main ----------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    controller = AppController()
    controller.ui.show()
    controller.start()

    def on_close():
        controller.stop(); time.sleep(0.2)
    app.aboutToQuit.connect(on_close)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()