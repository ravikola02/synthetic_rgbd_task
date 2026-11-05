# Final sweep: output/final_sweep

pybullet, 30 test seeds × 5 noise draws (ideal: 1 draw per seed, repeated for the pairing); seeds: every ideal stage succeeds. Success: rate (k/n), [95 % exact interval], p = exact McNemar vs ideal (paired draws). Accuracy: median [95 % seed-bootstrap interval] over draws passing the 15 mm criterion.

## Success by stage

| condition | detector: hole detected (right hole, < 15 mm) | detector: within 3 mm (accuracy threshold) | fit: success (< 5 mm, < 10°) | fit: pose found (< 15 mm, < 10°) | grasp: lifted (> 50 mm, both jaws) |
|---|---|---|---|---|---|
| ideal | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] |
| rules | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 77 % (115/150) [69–83], p = 5.8e-11 |
| axial | 56 % (84/150) [48–64], p = 2.7e-20 | 53 % (79/150) [44–61], p = 8.5e-22 | 99 % (149/150) [96–100], p = 1 | 99 % (149/150) [96–100], p = 1 | 63 % (95/150) [55–71], p = 5.6e-17 |
| edge | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 63 % (95/150) [55–71], p = 5.6e-17 |
| edge + axial | 39 % (58/150) [31–47], p = 4e-28 | 37 % (56/150) [30–46], p = 1e-28 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 65 % (98/150) [57–73], p = 4.4e-16 |
| bias ±2 % | 100 % (150/150) [98–100], p = 1 | 0 % (0/150) [0–2], p = 1.4e-45 | 0 % (0/150) [0–2], p = 1.4e-45 | 100 % (150/150) [98–100], p = 1 | 66 % (99/150) [58–74], p = 8.9e-16 |
| full model | 54 % (81/150) [46–62], p = 3.4e-21 | 0 % (0/150) [0–2], p = 1.4e-45 | 0 % (0/150) [0–2], p = 1.4e-45 | 99 % (149/150) [96–100], p = 1 | 57 % (85/150) [48–65], p = 5.4e-20 |

Edge + axial vs axial (paired, exact McNemar): hole detected (right hole, < 15 mm) p = 0.0049; within 3 mm (accuracy threshold) p = 0.013; success (< 5 mm, < 10°) p = 1; pose found (< 15 mm, < 10°) p = 1; lifted (> 50 mm, both jaws) p = 0.77

## Accuracy (median [95 % seed-bootstrap])

| condition | detector: hole position error (mm) | detector: hole axis error (°) | fit: translation error (mm) | fit: rotation error (°) | grasp: hole position error at grasp time (mm) | grasp: approach direction error (°) |
|---|---|---|---|---|---|---|
| ideal | 0.79 [0.53–0.85] | 0.75 [0.62–0.77] | 0.37 [0.33–0.43] | 0.65 [0.55–0.71] | 0.36 [0.29–0.43] | 0.60 [0.50–0.63] |
| rules | 0.76 [0.69–0.90] | 0.71 [0.64–0.79] | 0.45 [0.38–0.47] | 0.58 [0.46–0.84] | 0.44 [0.35–0.52] | 0.50 [0.42–0.60] |
| axial | 0.89 [0.73–0.97] | 0.93 [0.79–1.06] | 0.77 [0.75–0.81] | 0.88 [0.79–0.98] | 0.77 [0.68–0.83] | 0.81 [0.64–0.97] |
| edge | 0.55 [0.45–0.70] | 1.12 [0.99–1.34] | 0.41 [0.38–0.45] | 0.67 [0.57–0.79] | 0.52 [0.40–0.65] | 0.98 [0.82–1.06] |
| edge + axial | 0.73 [0.63–0.83] | 1.26 [1.09–1.46] | 0.86 [0.82–0.91] | 1.03 [0.86–1.21] | 0.73 [0.59–0.89] | 0.98 [0.85–1.15] |
| bias ±2 % | 10.21 [9.14–11.08] | 0.69 [0.63–0.74] | 10.70 [10.55–10.81] | 0.60 [0.49–0.74] | 10.04 [9.11–10.73] | 0.66 [0.60–0.72] |
| full model | 10.40 [10.14–10.63] | 1.15 [0.99–1.31] | 10.88 [10.03–11.15] | 0.97 [0.89–1.14] | 10.34 [10.04–10.47] | 1.17 [1.01–1.30] |
