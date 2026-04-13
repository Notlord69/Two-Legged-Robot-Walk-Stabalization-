"""
================================================================================
PROJECT SICLO1 — POST-RUN TELEMETRY ANALYSIS
================================================================================

Reads telemetry.csv from a session folder and produces 4 matplotlib figures.
No PyBullet dependency. Runs fully headless (Agg backend) unless --show is used.

Output files written to session folder:
  com_trajectory.png  — COM x/y/z vs time
  contact_forces.png  — left + right foot force (N) vs time
  timing.png          — compute time (µs) per cycle + violation markers
  stability.png       — stability margin + stability status vs time

Usage:
    python3 analyze.py sessions/2026-04-13_14-30-00/
    python3 analyze.py sessions/2026-04-13_14-30-00/ --show

Author : Siclo1 Project Team
Date   : April 2026
================================================================================
"""

import argparse
import os
import numpy as np

# CSV column indices — must match CSV_HEADER in telemetry.py
_COL_TIME    = 0
_COL_CYCLE   = 1
_COL_ERR     = 2
_COL_COM_X   = 3
_COL_COM_Y   = 4
_COL_COM_Z   = 5
_COL_L_CONT  = 6
_COL_R_CONT  = 7
_COL_STAB    = 8
_COL_L_FORCE = 9
_COL_R_FORCE = 10
_COL_MARGIN  = 11
_COL_COMP_US = 12


def analyze(session_path: str, show: bool = False) -> None:
    """Generate 4 PNG analysis figures from telemetry.csv.

    session_path : folder containing telemetry.csv
    show         : if True, call plt.show() after saving (requires X11 display)
    """
    csv_path = os.path.join(session_path, 'telemetry.csv')
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(
            f"telemetry.csv not found in {session_path}"
        )

    import matplotlib
    if not show:
        matplotlib.use('Agg')   # headless — must be set before pyplot import
    import matplotlib.pyplot as plt

    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data[np.newaxis, :]  # single-row CSV → (1, 16) array

    t       = data[:, _COL_TIME]
    com_x   = data[:, _COL_COM_X]
    com_y   = data[:, _COL_COM_Y]
    com_z   = data[:, _COL_COM_Z]
    l_force = data[:, _COL_L_FORCE]
    r_force = data[:, _COL_R_FORCE]
    stab    = data[:, _COL_STAB]
    margin  = data[:, _COL_MARGIN]
    comp_us = data[:, _COL_COMP_US]
    err     = data[:, _COL_ERR]

    # ── Figure 1: COM trajectory ──────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('Centre of Mass Trajectory')
    for ax, col, ylabel, color in zip(
        axes,
        [com_x, com_y, com_z],
        ['X (m)', 'Y (m)', 'Z (m)'],
        ['tab:blue', 'tab:orange', 'tab:green'],
    ):
        ax.plot(t, col, color=color, linewidth=0.8)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    _save_fig(fig, session_path, 'com_trajectory.png')
    plt.close(fig)

    # ── Figure 2: Contact forces ──────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, l_force, label='Left (N)',  color='tab:blue',   linewidth=0.8)
    ax.plot(t, r_force, label='Right (N)', color='tab:orange', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Contact Force (N)')
    ax.set_title('Foot Contact Forces')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_fig(fig, session_path, 'contact_forces.png')
    plt.close(fig)

    # ── Figure 3: Timing ──────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, comp_us, color='tab:purple', linewidth=0.5, label='Compute (µs)')
    viol_mask = err > 0
    if viol_mask.any():
        ax.scatter(t[viol_mask], comp_us[viol_mask],
                   color='red', s=12, zorder=5, label='Violation')
    ax.axhline(10_000, color='red', linestyle='--', linewidth=0.8,
               label='10 ms limit')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Compute Time (µs)')
    ax.set_title('Cycle Timing')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_fig(fig, session_path, 'timing.png')
    plt.close(fig)

    # ── Figure 4: Stability ───────────────────────────────────────────────────
    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax1.plot(t, margin, color='tab:green', linewidth=0.8, label='Margin (m)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Stability Margin (m)', color='tab:green')
    ax1.tick_params(axis='y', labelcolor='tab:green')
    ax2 = ax1.twinx()
    ax2.plot(t, stab, color='tab:red', linewidth=0.5, alpha=0.6, label='Status')
    ax2.set_ylabel('Stability Status', color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    ax1.set_title('Stability Margin & Status')
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_fig(fig, session_path, 'stability.png')
    plt.close(fig)

    if show:
        plt.show()


def _save_fig(fig, session_path: str, filename: str) -> None:
    path = os.path.join(session_path, filename)
    fig.savefig(path, dpi=120)
    print(f"[analyze] Saved {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Siclo1 post-run telemetry analysis'
    )
    parser.add_argument('session', help='Path to session folder (contains telemetry.csv)')
    parser.add_argument('--show', action='store_true',
                        help='Open figures interactively after saving (requires X11)')
    args = parser.parse_args()
    analyze(args.session, show=args.show)


if __name__ == '__main__':
    main()
