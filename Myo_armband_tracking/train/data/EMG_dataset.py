from .base_dataset import BaseDataset
import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt
import torch
from torchvision.transforms import ToTensor, RandomHorizontalFlip
from PIL import Image, ImageOps
import torchvision.transforms as transforms
import torchvision.transforms.functional as F
import random
import numpy as np
import matplotlib.pyplot as plt
import json
import random
from matplotlib.ticker import MultipleLocator
import numpy as np
import matplotlib.pyplot as plt
import json, random



class EMGDataset(BaseDataset):
    def __init__(self, *args, transform = None, **kwargs): 
        super().__init__(*args,**kwargs)
        self.transform = transform
        self.data_file = os.path.join(self.root_path,self.dataset_name)
        self.prepare_data(self.data_file) # function to extract info from the file
                
        
    def prepare_data(self, file):
        data = np.load(file, allow_pickle=True)
        self.samples = data["X"]          # preprocessed windows
        self.labels  = data["y"]
        self.meta    = json.loads(data["meta"].item())

        # Optional raw provenance (only exists after you add it during compile)
        self.sess = data["sess"] if "sess" in data.files else None
        self.i0   = data["i0"]   if "i0"   in data.files else None
        self.i1   = data["i1"]   if "i1"   in data.files else None

        self._raw_session_cache = {}  # simple cache: session_name -> raw (N,8) float32


        
    def __len__(self):
        return len(self.labels) 
        # how many samples we have based on how many labels
    
        
    def __getitem__(self,idx):
        #idx will be used for first dimension only since each sample
        #should include all info from the other dimensions
        sample = self.samples[idx,:,:]
        sample = torch.from_numpy(sample).float()
        label = self.labels[idx]
        label = torch.tensor(label, dtype=torch.long)

        return {'signal':sample.squeeze(),'label': label} #return a dict 
    
    def visualize(self,idx):
        sample = self.samples[idx,:,:]
        label_id = int(self.labels[idx])
        LABELS = self.meta["labels"]
        label_name = LABELS[str(label_id)] if isinstance(LABELS, dict) else LABELS[label_id]
        fs = self.meta["fs"]
        WIN_MS = self.meta["win_ms"]

        # --- Time axis ---
        T = sample.shape[1]
        t = np.arange(T) / fs

        # --- EMG strip styling ---
        # Use a fixed per-channel vertical scale so traces look like clinical strips.
        # 'offset' defines channel spacing; 'amp_step' sets the vertical scale unit (grid step).
        trace_peak = max(1e-6, float(np.max(np.abs(sample))))
        offset    = 1.6 * trace_peak         # channel spacing
        amp_step  = 0.2 * trace_peak         # vertical major grid step (~like 100 µV in a.u.)
        time_major = 0.10                     # 100 ms major grid
        time_minor = 0.02                     # 20 ms minor grid

        fig, ax = plt.subplots(figsize=(12, 6), dpi=120)
        ax.set_facecolor("white")
        for spine in ax.spines.values():
            spine.set_color("black")

        # Black EMG traces with per-channel isoelectric baselines
        for ch in range(8):
            base = ch * offset
            y_shifted = sample[ch] + base
            ax.plot(t, y_shifted, color="black", lw=1.2)
            # baseline (isoelectric)
            ax.hlines(base, t[0], t[-1], colors="#bbbbbb", linestyles="--", linewidth=0.8)
            # inline channel label at t=0
            ax.text(t[0] - 0.005, base, f"ch{ch}", va='center', ha='right',
                    fontsize=9, color="black")

            # Major: 100 ms (x) × amp_step (y); Minor: 20 ms × amp_step/5

        ax.set_axisbelow(True)
        ax.xaxis.set_major_locator(MultipleLocator(time_major))
        ax.xaxis.set_minor_locator(MultipleLocator(time_minor))
        ax.yaxis.set_major_locator(MultipleLocator(amp_step))
        ax.yaxis.set_minor_locator(MultipleLocator(amp_step/5))

        ax.grid(True, which='major', color='#cfcfcf', linewidth=0.9)
        ax.grid(True, which='minor', color='#e9e9e9', linestyle='--', linewidth=0.6)

        # Keep the same horizontal start point, hide y tick labels
        ax.set_yticklabels([])
        ax.set_xlabel("Time [s]")
        ax.set_title(f"Electromyogram-style strip — Label {label_id}: {label_name.replace('_',' ').title()}")

        # --- Calibration bars (bottom-right corner) ---
        # 100 ms × 1 amp_step, placed where traces won't overlap.
        x_end = float(t[-1])
        x_cal_w = 0.10   # 100 ms bar
        y_bottom = -0.8 * offset       # below ch0 baseline to avoid overlapping signals
        x_start = x_end - 0.14         # inset from right edge

        # vertical (amplitude) bar
        ax.plot([x_start, x_start], [y_bottom, y_bottom + amp_step], color='black', lw=2)
        ax.text(x_start - 0.003, y_bottom + amp_step/2, f"{amp_step:.2f} a.u.",
                va='center', ha='right', fontsize=9)

        # horizontal (time) bar
        ax.plot([x_start, x_start + x_cal_w], [y_bottom, y_bottom], color='black', lw=2)
        ax.text(x_start + x_cal_w/2, y_bottom - 0.06*offset, "100 ms",
                va='top', ha='center', fontsize=9)

        # Tight layout and show
        plt.tight_layout()
        plt.show()

        print(f"Index: {idx}")
        print(f"Label ID: {label_id}")
        print(f"Label Name: {label_name}")
        print(f"Shape: {sample.shape} (C,T) | Duration: {WIN_MS/1000:.2f} s")

    def _load_session_raw(self, session_name: str) -> np.ndarray:
        """
        Load raw samples.csv for a session and return (N,8) float32 array.
        Cached to avoid re-reading CSV every time.
        """
        if session_name in self._raw_session_cache:
            return self._raw_session_cache[session_name]

        sessions_root = self.meta.get("sessions_root", None)
        if sessions_root is None:
            raise RuntimeError("meta['sessions_root'] missing. Can't locate original session data on disk.")

        samples_p = os.path.join(sessions_root, session_name, "samples.csv")
        if not os.path.exists(samples_p):
            raise FileNotFoundError(f"Can't find: {samples_p}")

        # Minimal robust read (you can replace with your read_csv_smart if you import it)
        df = pd.read_csv(samples_p, sep=';', engine='python', on_bad_lines='skip')
        cols = [f"ch{k}" for k in range(8)]
        raw = df[cols].astype(np.float32).values  # (N,8)

        # Keep cache small: only keep last loaded session
        self._raw_session_cache.clear()
        self._raw_session_cache[session_name] = raw
        return raw


    def get_raw_window(self, idx: int) -> np.ndarray:
        """
        Returns raw window as (8,T) from the original samples.csv, using saved provenance.
        """
        if self.sess is None or self.i0 is None or self.i1 is None:
            raise RuntimeError(
                "Raw provenance (sess/i0/i1) not found in NPZ. "
                "Recompile after saving sess/i0/i1 in build_dataset()."
            )

        session_name = str(self.sess[idx])
        i0 = int(self.i0[idx])
        i1 = int(self.i1[idx])

        raw_session = self._load_session_raw(session_name)   # (N,8)
        if not (0 <= i0 < i1 <= raw_session.shape[0]):
            raise IndexError(f"Bad window indices for idx={idx}: i0={i0}, i1={i1}, N={raw_session.shape[0]}")

        raw_win = raw_session[i0:i1].T  # (8,T)
        return raw_win


    def visualize_raw(self, idx: int):
        """
        Plot RAW EMG window (before preprocess_window), for dataset index idx.
        """
        raw_win = self.get_raw_window(idx)  # (8,T)

        label_id = int(self.labels[idx])
        LABELS = self.meta["labels"]
        label_name = LABELS[str(label_id)] if isinstance(LABELS, dict) else LABELS[label_id]
        fs = self.meta["fs"]
        WIN_MS = self.meta["win_ms"]

        T = raw_win.shape[1]
        t = np.arange(T) / fs

        trace_peak = max(1e-6, float(np.max(np.abs(raw_win))))
        offset     = 1.6 * trace_peak
        amp_step   = 0.2 * trace_peak
        time_major = 0.10
        time_minor = 0.02

        fig, ax = plt.subplots(figsize=(12, 6), dpi=120)
        ax.set_facecolor("white")
        for spine in ax.spines.values():
            spine.set_color("black")

        for ch in range(8):
            base = ch * offset
            y_shifted = raw_win[ch] + base
            ax.plot(t, y_shifted, color="black", lw=1.2)
            ax.hlines(base, t[0], t[-1], colors="#bbbbbb", linestyles="--", linewidth=0.8)
            ax.text(t[0] - 0.005, base, f"ch{ch}", va='center', ha='right', fontsize=9, color="black")

        ax.set_axisbelow(True)
        ax.xaxis.set_major_locator(MultipleLocator(time_major))
        ax.xaxis.set_minor_locator(MultipleLocator(time_minor))
        ax.yaxis.set_major_locator(MultipleLocator(amp_step))
        ax.yaxis.set_minor_locator(MultipleLocator(amp_step/5))
        ax.grid(True, which='major', color='#cfcfcf', linewidth=0.9)
        ax.grid(True, which='minor', color='#e9e9e9', linestyle='--', linewidth=0.6)

        ax.set_yticklabels([])
        ax.set_xlabel("Time [s]")
        ax.set_title(f"RAW EMG (pre-preprocessing) — Label {label_id}: {label_name.replace('_',' ').title()}")

        # --- Calibration bars (bottom-right corner), same as visualize() ---
        x_end   = float(t[-1])
        x_cal_w = 0.10  # 100 ms bar
        y_bottom = -0.8 * offset     # below ch0 baseline
        x_start  = x_end - 0.14      # inset from right edge

        # vertical (amplitude) bar
        ax.plot([x_start, x_start], [y_bottom, y_bottom + amp_step], color='black', lw=2)
        ax.text(x_start - 0.003, y_bottom + amp_step/2, f"{amp_step:.2f} a.u.",
                va='center', ha='right', fontsize=9)

        # horizontal (time) bar
        ax.plot([x_start, x_start + x_cal_w], [y_bottom, y_bottom], color='black', lw=2)
        ax.text(x_start + x_cal_w/2, y_bottom - 0.06*offset, "100 ms",
                va='top', ha='center', fontsize=9)

        # Ensure the calibration bars are inside the y-limits
        y0, y1 = ax.get_ylim()
        ax.set_ylim(min(y0, y_bottom - 0.2*offset), y1)


        plt.tight_layout()
        plt.show()

        print(f"Index: {idx}")
        print(f"Session: {str(self.sess[idx])}")
        print(f"Window i0..i1: {int(self.i0[idx])}..{int(self.i1[idx])} (samples)")
        print(f"Label ID: {label_id}")
        print(f"Label Name: {label_name}")
        print(f"Shape: {raw_win.shape} (C,T) | Duration: {WIN_MS/1000:.2f} s")
