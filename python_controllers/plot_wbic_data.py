#!/usr/bin/env python3
"""
plot_wbic_data.py
=================
Python equivalent of matlab/plot_wbic_data.m.

Loads a wbic_log_*.npz file produced by the Python WBIC node (wbic_node.py)
from the logs/ directory and generates the same six figures:

    Figure 1 — Body Orientation Tracking  (roll, pitch, yaw vs command)
    Figure 2 — Body Position Tracking     (x, y, z vs command, in mm)
    Figure 3 — Angular Velocity Tracking  (ωx, ωy, ωz vs command, in deg/s)
    Figure 4 — Linear Velocity Tracking   (vx, vy, vz vs command, in m/s)
    Figure 5 — Foot Position Tracking     (3×4 grid, per leg × axis, in mm)
    Figure 6 — Foot Force Tracking        (3×4 grid, per leg × force component, in N)

All available figures are saved as PNGs to logs/.

Usage
-----
    python plot_wbic_data.py              # auto-selects latest log
    python plot_wbic_data.py logs/wbic_log_2025-05-10_14-00-00.npz
"""

import sys
import os
import glob
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Matplotlib style ─────────────────────────────────────────────────────────
matplotlib.rcParams.update({
    'figure.facecolor':  '#1a1a2e',
    'axes.facecolor':    '#16213e',
    'axes.edgecolor':    '#4a4a6a',
    'axes.labelcolor':   '#e0e0e0',
    'axes.titlecolor':   '#e0e0e0',
    'xtick.color':       '#b0b0c0',
    'ytick.color':       '#b0b0c0',
    'text.color':        '#e0e0e0',
    'grid.color':        '#2a2a4a',
    'grid.linestyle':    '--',
    'grid.alpha':        0.6,
    'legend.facecolor':  '#1a1a2e',
    'legend.edgecolor':  '#4a4a6a',
    'figure.titlesize':  13,
    'axes.titlesize':    10,
    'axes.labelsize':    9,
    'xtick.labelsize':   8,
    'ytick.labelsize':   8,
    'legend.fontsize':   8,
    'lines.linewidth':   1.4,
})

ACTUAL_COLOR = '#4fc3f7'   # light blue
CMD_COLOR    = '#ef5350'   # red-ish
ACTUAL_STYLE = '-'
CMD_STYLE    = '--'

LEG_NAMES  = ['FR', 'FL', 'RR', 'RL']
FORCE_DIMS = ['Fx', 'Fy', 'Fz']
POS_DIMS   = ['X',  'Y',  'Z']
RPY_LABELS = ['Roll', 'Pitch', 'Yaw']
VEL_LABELS = ['vx', 'vy', 'vz']
OMEGA_LABELS = [r'$\omega_x$', r'$\omega_y$', r'$\omega_z$']

# ── Helpers ───────────────────────────────────────────────────────────────────

def rmse(a, b):
    """Root-mean-square error between two equal-length 1-D arrays."""
    return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))


def _axstyle(ax, xlabel=None, ylabel=None, title=None, legend=True):
    ax.grid(True)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    if legend:
        ax.legend(loc='best')


# ── Figure builders ───────────────────────────────────────────────────────────

def fig_orientation(t, data):
    """Figure 1 — Body Orientation Tracking."""
    rpy     = np.degrees(data['rpy'])      # (3, T)
    rpy_cmd = np.degrees(data['rpy_cmd'])  # (3, T)

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    fig.canvas.manager.set_window_title('Body Orientation')
    fig.suptitle('Body Orientation Tracking')

    for i, ax in enumerate(axes):
        ax.plot(t, rpy[i],     ACTUAL_STYLE, color=ACTUAL_COLOR, label='Actual')
        ax.plot(t, rpy_cmd[i], CMD_STYLE,    color=CMD_COLOR,    label='Cmd')
        err = rmse(rpy[i], rpy_cmd[i])
        _axstyle(ax,
                 xlabel='Time (s)' if i == 2 else None,
                 ylabel=f'{RPY_LABELS[i]} (deg)',
                 title=f'{RPY_LABELS[i]}  |  RMSE: {err:.2f} deg')

    fig.tight_layout()
    return fig, 'Body Orientation'


def fig_position(t, data):
    """Figure 2 — Body Position Tracking (mm)."""
    pos     = data['pos']     * 1000   # (3, T) → mm
    pos_cmd = data['pos_cmd'] * 1000

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    fig.canvas.manager.set_window_title('Body Position')
    fig.suptitle('Body Position Tracking')

    for i, ax in enumerate(axes):
        ax.plot(t, pos[i],     ACTUAL_STYLE, color=ACTUAL_COLOR, label='Actual')
        ax.plot(t, pos_cmd[i], CMD_STYLE,    color=CMD_COLOR,    label='Cmd')
        err = rmse(pos[i], pos_cmd[i])
        _axstyle(ax,
                 xlabel='Time (s)' if i == 2 else None,
                 ylabel=f'{POS_DIMS[i]} (mm)',
                 title=f'{POS_DIMS[i]}  |  RMSE: {err:.1f} mm')

    fig.tight_layout()
    return fig, 'Body Position'


def fig_angular_velocity(t, data):
    """Figure 3 — Angular Velocity Tracking (deg/s)."""
    if 'omega' not in data or 'omega_cmd' not in data:
        return None, None

    omega     = np.degrees(data['omega'])      # (3, T)
    omega_cmd = np.degrees(data['omega_cmd'])  # (3, T)

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    fig.canvas.manager.set_window_title('Angular Velocity')
    fig.suptitle('Angular Velocity Tracking')

    for i, ax in enumerate(axes):
        ax.plot(t, omega[i],     ACTUAL_STYLE, color=ACTUAL_COLOR, label='Actual')
        ax.plot(t, omega_cmd[i], CMD_STYLE,    color=CMD_COLOR,    label='Cmd')
        err = rmse(omega[i], omega_cmd[i])
        _axstyle(ax,
                 xlabel='Time (s)' if i == 2 else None,
                 ylabel=f'{OMEGA_LABELS[i]} (deg/s)',
                 title=f'{OMEGA_LABELS[i]}  |  RMSE: {err:.2f} deg/s')

    fig.tight_layout()
    return fig, 'Angular Velocity'


def fig_linear_velocity(t, data):
    """Figure 4 — Linear Velocity Tracking (m/s)."""
    if 'vel' not in data or 'vel_cmd' not in data:
        return None, None

    vel     = data['vel']      # (3, T)
    vel_cmd = data['vel_cmd']  # (3, T)

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    fig.canvas.manager.set_window_title('Linear Velocity')
    fig.suptitle('Linear Velocity Tracking')

    for i, ax in enumerate(axes):
        ax.plot(t, vel[i],     ACTUAL_STYLE, color=ACTUAL_COLOR, label='Actual')
        ax.plot(t, vel_cmd[i], CMD_STYLE,    color=CMD_COLOR,    label='Cmd')
        err = rmse(vel[i], vel_cmd[i])
        _axstyle(ax,
                 xlabel='Time (s)' if i == 2 else None,
                 ylabel=f'{VEL_LABELS[i]} (m/s)',
                 title=f'{VEL_LABELS[i]}  |  RMSE: {err:.3f} m/s')

    fig.tight_layout()
    return fig, 'Linear Velocity'


def fig_foot_position(t, data):
    """Figure 5 — Foot Position Tracking (mm), 3×4 grid."""
    if 'foot_pos' not in data or 'foot_pos_cmd' not in data:
        return None, None

    fp     = data['foot_pos']     * 1000   # (12, T) → mm
    fp_cmd = data['foot_pos_cmd'] * 1000

    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title('Foot Position')
    fig.suptitle('Foot Position Tracking')
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.55, wspace=0.35)

    for dim in range(3):           # rows: X, Y, Z
        for leg in range(4):       # cols: FR, FL, RR, RL
            ax  = fig.add_subplot(gs[dim, leg])
            idx = leg * 3 + dim    # flat index into (12,)
            ax.plot(t, fp[idx],     ACTUAL_STYLE, color=ACTUAL_COLOR, label='Actual', linewidth=1.0)
            ax.plot(t, fp_cmd[idx], CMD_STYLE,    color=CMD_COLOR,    label='Cmd',    linewidth=0.8)
            err = rmse(fp[idx], fp_cmd[idx])
            if dim == 0:
                ax.set_title(LEG_NAMES[leg])
            ax.set_ylabel(f'{POS_DIMS[dim]} (mm)')
            if dim == 2:
                ax.set_xlabel('Time (s)')
            ax.text(0.98, 0.95, f'RMSE: {err:.1f}',
                    transform=ax.transAxes, ha='right', va='top',
                    fontsize=7, color='#aaaaaa')
            ax.grid(True)

    # Single legend in the top-right subplot
    handles = [
        plt.Line2D([0], [0], color=ACTUAL_COLOR, linestyle=ACTUAL_STYLE, label='Actual'),
        plt.Line2D([0], [0], color=CMD_COLOR,    linestyle=CMD_STYLE,    label='Cmd'),
    ]
    fig.legend(handles=handles, loc='upper right', framealpha=0.6, fontsize=8)
    return fig, 'Foot Position'


def fig_foot_force(t, data):
    """Figure 6 — Foot Force Tracking (N), 3×4 grid."""
    if 'foot_force_cmd' not in data or 'foot_force_actual' not in data:
        return None, None

    ff_act = data['foot_force_actual']   # (12, T)
    ff_cmd = data['foot_force_cmd']      # (12, T)

    fig = plt.figure(figsize=(13, 8))
    fig.canvas.manager.set_window_title('Foot Force')
    fig.suptitle('Foot Force Tracking  (Blue=Actual, Red=Cmd)')
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.55, wspace=0.35)

    for dim in range(3):           # rows: Fx, Fy, Fz
        for leg in range(4):       # cols: FR, FL, RR, RL
            ax  = fig.add_subplot(gs[dim, leg])
            idx = leg * 3 + dim
            ax.plot(t, ff_act[idx], ACTUAL_STYLE, color=ACTUAL_COLOR, label='Actual', linewidth=1.0)
            ax.plot(t, ff_cmd[idx], CMD_STYLE,    color=CMD_COLOR,    label='Cmd',    linewidth=0.8)
            err = rmse(ff_act[idx], ff_cmd[idx])
            if dim == 0:
                ax.set_title(LEG_NAMES[leg])
            ax.set_ylabel(f'{FORCE_DIMS[dim]} (N)')
            if dim == 2:
                ax.set_xlabel('Time (s)')
            ax.text(0.98, 0.95, f'RMSE: {err:.1f}',
                    transform=ax.transAxes, ha='right', va='top',
                    fontsize=7, color='#aaaaaa')
            ax.grid(True)

    handles = [
        plt.Line2D([0], [0], color=ACTUAL_COLOR, linestyle=ACTUAL_STYLE, label='Actual'),
        plt.Line2D([0], [0], color=CMD_COLOR,    linestyle=CMD_STYLE,    label='Cmd'),
    ]
    fig.legend(handles=handles, loc='upper right', framealpha=0.6, fontsize=8)
    return fig, 'Foot Force'


# ── Summary printer ────────────────────────────────────────────────────────────

def print_summary(t, data):
    print('=' * 40)
    print('SUMMARY')
    print(f'Duration: {t[-1]:.1f} s\n')

    rpy     = np.degrees(data['rpy'])
    rpy_cmd = np.degrees(data['rpy_cmd'])
    print('Body Orientation RMSE:')
    for i, lbl in enumerate(RPY_LABELS):
        print(f'  {lbl:<6}: {rmse(rpy[i], rpy_cmd[i]):.2f} deg')

    pos     = data['pos']     * 1000
    pos_cmd = data['pos_cmd'] * 1000
    print('\nBody Position RMSE:')
    for i, lbl in enumerate(POS_DIMS):
        print(f'  {lbl}: {rmse(pos[i], pos_cmd[i]):.1f} mm')

    if 'foot_pos' in data and 'foot_pos_cmd' in data:
        fp     = data['foot_pos']     * 1000
        fp_cmd = data['foot_pos_cmd'] * 1000
        print('\nFoot Position RMSE (mm):')
        print(f'       {"FR":>6} {"FL":>6} {"RR":>6} {"RL":>6}')
        for dim in range(3):
            vals = [rmse(fp[leg*3+dim], fp_cmd[leg*3+dim]) for leg in range(4)]
            print(f'  {POS_DIMS[dim]}:  {vals[0]:6.1f} {vals[1]:6.1f} {vals[2]:6.1f} {vals[3]:6.1f}')

    if 'foot_force_cmd' in data and 'foot_force_actual' in data:
        ff_act = data['foot_force_actual']
        ff_cmd = data['foot_force_cmd']
        print('\nFoot Force RMSE (N):')
        print(f'       {"FR":>6} {"FL":>6} {"RR":>6} {"RL":>6}')
        for dim in range(3):
            vals = [rmse(ff_act[leg*3+dim], ff_cmd[leg*3+dim]) for leg in range(4)]
            print(f'  {FORCE_DIMS[dim]}: {vals[0]:6.1f} {vals[1]:6.1f} {vals[2]:6.1f} {vals[3]:6.1f}')

    print('=' * 40)


# ── Entry point ───────────────────────────────────────────────────────────────

def select_log_file(log_dir='logs'):
    """Interactively pick a log file, or accept one from argv."""
    if len(sys.argv) > 1:
        path = sys.argv[1]
        if not os.path.isfile(path):
            sys.exit(f'File not found: {path}')
        return path

    pattern = os.path.join(log_dir, 'wbic_log_*.npz')
    files   = sorted(glob.glob(pattern))
    if not files:
        sys.exit(f'No log files found matching {pattern}')

    print('Available log files:')
    for i, f in enumerate(files):
        print(f'  {i+1}: {os.path.basename(f)}')

    if len(files) == 1:
        sel = 1
    else:
        raw = input('Select file number (or Enter for latest): ').strip()
        sel = int(raw) if raw.isdigit() else len(files)

    return files[sel - 1]


def main():
    log_dir  = 'logs'
    log_path = select_log_file(log_dir)

    npz  = np.load(log_path, allow_pickle=True)
    data = {k: npz[k] for k in npz.files}
    t    = data['t']

    print(f'Loaded: {os.path.basename(log_path)}  ({t[-1]:.1f} s)\n')

    print_summary(t, data)
    print()

    # Build all figures
    builders = [
        fig_orientation,
        fig_position,
        fig_angular_velocity,
        fig_linear_velocity,
        fig_foot_position,
        fig_foot_force,
    ]

    saved = []
    for build in builders:
        fig, name = build(t, data)
        if fig is None:
            continue
        out_path = os.path.join(log_dir, f'{name}.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {name}.png')
        saved.append(out_path)

    print(f'\nAll figures saved to {log_dir}/')
    plt.show()


if __name__ == '__main__':
    main()
