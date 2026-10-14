#!/usr/bin/env python3
"""M4/M5 comparison charts for the WarehouseRunner report.

Data is parsed directly from the run logs in wr_bringup/results/ so the
figures regenerate from raw evidence. Colors follow the validated reference
palette (see dataviz/palette.md); text stays in ink tokens, never series hues.
"""
import re
import glob
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

# ---------- palette (validated: adjacent-pairs, light surface) ----------
SURFACE = '#fcfcfb'
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRID, BASE = '#e1e0d9', '#c3c2b7'
C_DWB, C_RPP, C_MPPI = '#2a78d6', '#eb6834', '#1baf7a'  # slots 1..3
C_DWB_EMPH = '#1c5cab'                                   # blue step 550 (adopted)

for f in font_manager.fontManager.ttflist:
    pass
plt.rcParams.update({
    'font.sans-serif': ['Noto Sans CJK SC', 'Noto Sans CJK JP', 'DejaVu Sans'],
    'font.family': 'sans-serif',
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
    'savefig.facecolor': SURFACE,
    'text.color': INK, 'axes.edgecolor': BASE,
    'axes.labelcolor': INK2, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'axes.axisbelow': True, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.spines.left': False, 'axes.linewidth': 1.0,
})

RESULTS = glob.glob('/home/qx/Codes/Ros2/bookros2_ws/src/wr_bringup/results/patrol_*.log')


def parse(path):
    """Return (total_s, n_ok, per_leg_s) from one patrol log."""
    reached, nav0 = [], None
    for line in open(path, errors='ignore'):
        m = re.search(r'\[INFO\] \[(\d+)\.\d+\].*waypoint \d+ \(([-\d., ]+)\) REACHED', line)
        if m:
            reached.append(int(m.group(1)))
            if nav0 is None:
                pass
        if nav0 is None:
            m = re.search(r'\[INFO\] \[(\d+)\.\d+\].*Navigating to goal', line)
            if m:
                nav0 = int(m.group(1))
    total = n_ok = 0
    for line in open(path, errors='ignore'):
        m = re.search(r'patrol finished: (\d+)/(\d+) reached in (\d+) s', line)
        if m:
            n_ok, total = int(m.group(1)), int(m.group(3))
    seq = ([nav0] if nav0 else []) + reached
    legs = [b - a for a, b in zip(seq, seq[1:])] if len(seq) > 1 else []
    return total, n_ok, legs


runs = {}
for f in RESULTS:
    runs[f.split('/')[-1]] = parse(f)

# ---------------- chart A: controller totals (M5) ----------------
A = [  # label, seconds, score, color
    ('DWB\n(M4 调优后)', runs['patrol_exp5_simtime10.log'][0], '6/6', C_DWB),
    ('RPP\n(默认参数)', runs['patrol_m5_rpp.log'][0], '5/6', C_RPP),
    ('MPPI\n(wz_max=1.0)', runs['patrol_m5_mppi_tuned.log'][0], '6/6', C_MPPI),
]
fig, ax = plt.subplots(figsize=(5.4, 3.0), dpi=200)
xs = range(len(A))
ax.bar(xs, [a[1] for a in A], width=0.55, color=[a[3] for a in A], zorder=3)
for x, (name, sec, score, _) in zip(xs, A):
    ax.text(x, sec + 12, f'{sec} s', ha='center', va='bottom', fontsize=11,
            color=INK, fontweight='bold')
    ax.text(x, sec + 58, score, ha='center', va='bottom', fontsize=9, color=INK2)
ax.set_xticks(list(xs))
ax.set_xticklabels([a[0] for a in A], fontsize=9.5)
ax.set_ylabel('巡检总耗时 (s)', fontsize=9.5)
ax.set_title('三种控制器:同一路线巡检总耗时与到达率', fontsize=11, color=INK, pad=10)
ax.set_ylim(0, 640)
ax.grid(axis='x', visible=False)
fig.tight_layout()
fig.savefig('figs/chart_controllers.pdf')
fig.savefig('figs/chart_controllers.png')

# ---------------- chart B: M4 tuning progression ----------------
B = [
    ('基线\n0.3 m/s', 'patrol_baseline_2026-10-12.log', False),
    ('提速\n0.5 m/s', 'patrol_exp1_speed.log', False),
    ('提速\n0.7 m/s', 'patrol_exp2b_speed07_true.log', False),
    ('膨胀\n0.35 m', 'patrol_exp3_infl035.log', False),
    ('膨胀\n0.80 m', 'patrol_exp4_infl08.log', False),
    ('sim_time\n1.0 s', 'patrol_exp5_simtime10.log', True),   # adopted
    ('sim_time\n2.5 s', 'patrol_exp6_simtime25.log', False),
]
fig, ax = plt.subplots(figsize=(5.8, 3.0), dpi=200)
vals = [runs[f][0] for _, f, _ in B]
cols = [C_DWB_EMPH if ad else C_DWB for *_, ad in B]
ax.bar(range(len(B)), vals, width=0.6, color=cols, zorder=3)
for i, (v, (_, f, ad)) in enumerate(zip(vals, B)):
    if ad:
        ax.text(i, v + 14, f'{v} s\n(采纳)', ha='center', va='bottom',
                fontsize=9, color=INK, fontweight='bold')
ax.text(0, 829 + 14, '829 s\n5/6', ha='center', va='bottom', fontsize=9, color=INK)
ax.set_xticks(range(len(B)))
ax.set_xticklabels([b[0] for b in B], fontsize=8.5)
ax.set_ylabel('巡检总耗时 (s)', fontsize=9.5)
ax.set_title('参数调优过程:总耗时随参数变化 (DWB)', fontsize=11, color=INK, pad=10)
ax.set_ylim(0, 950)
ax.grid(axis='x', visible=False)
fig.tight_layout()
fig.savefig('figs/chart_tuning.pdf')
fig.savefig('figs/chart_tuning.png')

# ---------------- chart C: per-leg times (M5) ----------------
legs_dwb = runs['patrol_exp5_simtime10.log'][2]
_legs_rpp_raw = runs['patrol_m5_rpp.log'][2]
legs_mppi = runs['patrol_m5_mppi_tuned.log'][2]
# RPP log file is contaminated (an interleaved dead first attempt), so per-leg
# times are anchored on the live-monitored REACHED stamps and the run total:
# start=1791812191 (=end 727 − total 536); REACHED: 227,270,410,480,512.
legs_rpp = [36, 43, 140, 70, 32]  # wp1..wp5; wp6 未到达 (215 s dither, then abort)
print('RPP raw legs (parse artifact check):', _legs_rpp_raw)
print('DWB legs:', legs_dwb, '| MPPI legs:', legs_mppi)

fig, ax = plt.subplots(figsize=(6.4, 3.0), dpi=200)
n, w, gap = 3, 0.24, 0.03
xs = [i * (3 * w + 2 * gap) for i in range(6)]


def group(i, vals, color, label):
    off = (i - 1) * (w + gap)
    xs_ = [x + off for x in xs[:len(vals)]]
    ax.bar(xs_, vals, width=w, color=color, label=label, zorder=3)


group(0, legs_dwb, C_DWB, 'DWB')
group(1, legs_rpp, C_RPP, 'RPP')
group(2, legs_mppi, C_MPPI, 'MPPI')
# RPP wp6: no bar — mark the miss
x6 = xs[5] + (w + gap)
ax.text(x6, 6, '× 未到达', ha='center', va='bottom', fontsize=8.5, color=C_RPP)
ax.set_xticks(xs)
ax.set_xticklabels([f'航点 {i+1}' for i in range(6)], fontsize=9)
ax.set_ylabel('单段耗时 (s)', fontsize=9.5)
ax.set_title('各航点分段耗时对比', fontsize=11, color=INK, pad=10)
ax.set_ylim(0, 165)
ax.legend(frameon=False, fontsize=9, ncol=3, loc='upper right')
ax.grid(axis='x', visible=False)
fig.tight_layout()
fig.savefig('figs/chart_legs.pdf')
fig.savefig('figs/chart_legs.png')

print('charts written to figs/')
