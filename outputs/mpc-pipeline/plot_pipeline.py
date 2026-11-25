"""Reproducible offline figures; no ROS, robot connection, or telemetry."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parent
data = np.genfromtxt(ROOT/'solver-examples.csv', delimiter=',', names=True)
clear, nearby, blocked = [data[data['scenario'] == i] for i in range(3)]
assert len(clear) == len(nearby) == 21 and len(blocked) == 1
assert blocked['blocked'][0] == 1
for rows in (clear, nearby):
    assert np.max(np.abs(rows['vx']) + np.abs(rows['vy'])) <= .100001
assert nearby['x'].max() <= .100001

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                     'axes.titlesize': 13, 'axes.labelsize': 11,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.titlepad': 12, 'figure.facecolor': 'white'})
blue, orange, red = '#2563A6', '#B65B13', '#B83240'
fig, axes = plt.subplots(2, 2, figsize=(13.8, 10))
fig.suptitle('Python coordinator + C++ MPC: behavior and timing', fontsize=18)

ax = axes[0, 0]
ax.plot(clear['x'], clear['y'], '-o', color=blue, ms=3, label='Clear: 21 predicted positions')
ax.plot(nearby['x'], nearby['y'], '-s', color=orange, ms=3, label='Obstacle at (0.65, 0) m')
ax.axvline(.1, color=orange, ls='--', lw=1.2)
ax.text(.108, .025, 'x ≤ 0.10 m\n0.65 − 0.55 clearance', color=orange)
ax.scatter([.30], [.12], marker='*', s=180, color='#4E5967', label='Local goal (0.30, 0.12) m')
ax.scatter([0], [0], marker='x', s=80, color=red, zorder=5,
           label='Obstacle at 0.45 m: blocked at origin')
ax.set(xlabel='Forward x (m)', ylabel='Left y (m)', xlim=(-.015, .33), ylim=(-.015, .16),
       title='A  Actual optimizer predictions over 2 seconds')
ax.legend(loc='upper left', fontsize=9, frameon=False)
ax.set_aspect('equal', adjustable='box')
ax.grid(alpha=.15)

ax = axes[0, 1]
for rows, label, color, marker in [(clear, 'Clear', blue, 'o'), (nearby, 'Nearby obstacle', orange, 's')]:
    for component, style in [('vx', '-'), ('vy', '--')]:
        ax.step(rows['t'][:-1], rows[component][:-1], where='post', color=color,
                linestyle=style, label=f'{label}: {component}')
ax.axhline(.1, color='#707985', ls='--', lw=1)
ax.text(.95, .103, '|vx| + |vy| limit = 0.10 m/s', fontsize=10)
ax.set(xlabel='Prediction time (s)', ylabel='Velocity component (m/s)', xlim=(0, 2), ylim=(-.005, .115),
       title='B  MPC shifts translation into the clear direction')
ax.legend(loc='center right', frameon=False, fontsize=9)
ax.grid(alpha=.15)

ax = axes[1, 0]
diamond = np.array([[.1, 0], [0, .1], [-.1, 0], [0, -.1]])
ax.add_patch(Polygon(diamond, facecolor=blue, alpha=.14, edgecolor='none'))
ax.plot(*np.vstack([diamond, diamond[:1]]).T, color=blue, lw=1.5)
ax.scatter([.1, .05], [0, .05], s=45, color=[blue, orange], zorder=3)
ax.annotate('Forward: 0.10 m/s', (.1, 0), xytext=(.005, -.045),
            arrowprops={'arrowstyle': '-', 'color': '#707985'}, fontsize=10)
ax.annotate('Diagonal: vx = vy = 0.05\nEuclidean speed ≈ 0.071 m/s', (.05, .05), xytext=(-.105, .09),
            arrowprops={'arrowstyle': '-', 'color': '#707985'}, fontsize=10)
ax.axhline(0, color='#707985', lw=.6)
ax.axvline(0, color='#707985', lw=.6)
ax.set(xlabel='vx (m/s)', ylabel='vy (m/s)', xlim=(-.12, .12), ylim=(-.115, .125),
       title='C  Allowed MPC velocities: |vx| + |vy| ≤ 0.10')
ax.set_aspect('equal', adjustable='box')
ax.grid(alpha=.15)

ax = axes[1, 1]
ax.broken_barh([(0, 150)], (1.8, .55), facecolors=blue)
ax.broken_barh([(0, 250)], (.8, .55), facecolors=orange)
ax.text(154, 2.075, '150 ms', va='center')
ax.text(254, 1.075, '250 ms', va='center')
ticks = np.arange(0, 301, 50)
ax.scatter(ticks, np.full_like(ticks, .15, dtype=float), marker='|', s=160, color='#4E5967')
ax.text(0, -.28, 'Dispatch ticks every 50 ms; shown aligned for illustration', fontsize=9)
ax.set(xlim=(0, 310), ylim=(-.45, 2.65), xlabel='Time since request creation (ms)',
       title='D  Each accepted reply retains its request age')
ax.set_yticks([2.075, 1.075, .15], ['Reply acceptance', 'Cached command', '20 Hz timer'])
ax.set_xticks(ticks)
ax.grid(axis='x', alpha=.15)

fig.tight_layout(rect=(0, .06, 1, .95), h_pad=2., w_pad=2.)
fig.text(.5, .015, 'A–B: one solve per synthetic scenario using this repository’s C++ solver; not measured robot motion.\n'
         'C–D: code-derived limits. MPC has 20 × 0.1 s steps; Python dispatch adds its own acceleration and collision checks.',
         ha='center', fontsize=10, color='#4E5967')
fig.savefig(ROOT/'mpc-pipeline-plots.png', dpi=180)
fig.savefig(ROOT/'mpc-pipeline-plots.pdf')
print('Saved', ROOT/'mpc-pipeline-plots.png')
