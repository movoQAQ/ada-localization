"""Compare replay estimates and the saved optimized map; these are not ground truth."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('--baseline', required=True)
p.add_argument('--fixed', required=True)
p.add_argument('--keyframes', required=True)
p.add_argument('--output', required=True)
a = p.parse_args()
original = np.asarray(json.loads(Path(a.baseline).read_text())['backend_odom'])[:, 1:4]
fixed = np.asarray(json.loads(Path(a.fixed).read_text())['backend_odom'])[:, 1:4]
keyframes = np.loadtxt(a.keyframes)
optimized = keyframes[:, 2:5]
fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
series = [(original, 'Original online', '#bd4932'),
          (fixed, 'Fixed online', '#999999'),
          (optimized, 'Fixed optimized map', '#147d92')]
for xyz, label, color in series:
    xyz = xyz - xyz[0]
    distance = np.linalg.norm(xyz[-1])
    for ax in axes:
        ax.plot(xyz[:, 0], xyz[:, 1], color=color, lw=1.2,
                label=f'{label}: {distance:.3f} m (3D endpoint)')
        ax.scatter(*xyz[-1, :2], color=color, marker='x', s=45)
for ax in axes:
    ax.scatter(0, 0, color='black', s=35, marker='o', zorder=5)
    ax.set(xlabel='Relative X (m)', ylabel='Relative Y (m)', aspect='equal')
    ax.grid(alpha=.2)
axes[0].set_title('Full trajectory estimates')
axes[0].legend(fontsize=8)
axes[1].set(xlim=(-3, 3), ylim=(-3, 3), title='Start / finish neighborhood')
fig.suptitle('SEU library Mid-360 | XY projection, no ground truth')
fig.savefig(a.output, dpi=160)
print(json.dumps({'keyframes': len(keyframes),
    'first_stamp_s': float(keyframes[0, 1]), 'last_stamp_s': float(keyframes[-1, 1]),
    'optimized_endpoint_delta_xyz_m': (optimized[-1]-optimized[0]).tolist(),
    'optimized_endpoint_displacement_m': float(np.linalg.norm(optimized[-1]-optimized[0]))}, indent=2))
