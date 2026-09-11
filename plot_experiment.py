"""Reusable chart helpers for lab-note reports (see 레이저 세라믹 가공/실험 기록 매뉴얼.md
section 6). Call one of these right after an experiment step instead of
hand-writing matplotlib per report -- pass out_path directly into that
report's "<주제> 사진" Obsidian folder so there's no separate copy step.

ponytail: only the 3 chart shapes actually used so far (Z-focus curve,
XY edge-sweep curve, a value distribution). Add a new one here when a
report needs a genuinely different shape, not preemptively.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_z_scan(samples, out_path, peak_z=None, title="Focus Z-scan"):
    """samples: list of [z_mm, score] pairs, e.g. lastResult['samples']."""
    z = [s[0] for s in samples]
    score = [s[1] for s in samples]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(z, score, marker=".", markersize=3, linewidth=1)
    if peak_z is not None:
        ax.axvline(peak_z, color="magenta", linestyle="--", linewidth=1, label=f"peak z={peak_z:.4f}mm")
        ax.legend()
    ax.set_xlabel("Z (mm)")
    ax.set_ylabel("focus score")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_xy_sweep(samples, out_path, edges=None, center=None, axis_label="X (mm)", title="Edge sweep"):
    """samples: list of [pos_mm, signal] pairs. edges: optional (pos1, pos2)
    of detected edge crossings. center: optional midpoint to mark."""
    pos = [s[0] for s in samples]
    sig = [s[1] for s in samples]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(pos, sig, marker=".", markersize=3, linewidth=1)
    if edges:
        for e in edges:
            ax.axvline(e, color="orange", linestyle="--", linewidth=1)
    if center is not None:
        ax.axvline(center, color="magenta", linestyle="-", linewidth=1.5, label=f"center={center:.4f}mm")
        ax.legend()
    ax.set_xlabel(axis_label)
    ax.set_ylabel("scatter/brightness signal")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_drift_over_time(records, out_path, title="Focus drift over time"):
    """records: list of (elapsed_min, best_offset_mm) from repeated focus
    scans -- how far the CV-found peak has moved since the run started."""
    t = [r[0] for r in records]
    offset = [r[1] for r in records]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, offset, marker="o", markersize=4, linewidth=1)
    ax.axhline(offset[0], color="gray", linestyle=":", linewidth=1, label=f"first={offset[0]:.4f}mm")
    ax.set_xlabel("elapsed (min)")
    ax.set_ylabel("best focus offset (mm)")
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_value_distribution(values, out_path, xlabel="value", title="Distribution"):
    """values: flat list of numbers, e.g. repeated latency-check impliedLagS."""
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(values, bins=min(20, max(5, len(values) // 2)))
    mean = sum(values) / len(values)
    ax.axvline(mean, color="magenta", linestyle="--", label=f"mean={mean:.4g}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("count")
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def demo():
    import tempfile, os
    d = tempfile.mkdtemp()
    plot_z_scan([[i * 0.01, 100 - (i - 10) ** 2] for i in range(21)], os.path.join(d, "z.png"), peak_z=0.1)
    plot_xy_sweep([[i * 0.1, abs(i - 25)] for i in range(51)], os.path.join(d, "xy.png"), edges=(1.0, 4.0), center=2.5)
    plot_value_distribution([0.05, 0.06, 0.055, 0.061, 0.048], os.path.join(d, "dist.png"))
    plot_drift_over_time([(0, -0.66), (5, -0.65), (10, -0.68)], os.path.join(d, "drift.png"))
    assert all(os.path.getsize(os.path.join(d, f)) > 0 for f in ("z.png", "xy.png", "dist.png", "drift.png"))
    print("OK", d)


if __name__ == "__main__":
    demo()
