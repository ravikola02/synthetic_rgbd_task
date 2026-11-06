# Final sweep: output/final_sweep

pybullet, 30 test seeds × 5 noise draws (ideal: 1 draw per seed, repeated for the pairing); seeds: every ideal stage succeeds. Success: rate (k/n), [95 % exact interval], p = exact McNemar vs ideal (paired draws). Accuracy: median [95 % seed-bootstrap interval] over draws passing the 15 mm criterion.

## Success by stage

| condition | detector: hole detected (right hole, < 15 mm) | detector: within 3 mm (accuracy threshold) | fit: success (< 5 mm, < 10°) | fit: pose found (< 15 mm, < 10°) | grasp: lifted (> 50 mm, both jaws) |
|---|---|---|---|---|---|
| ideal | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] | 100 % (150/150) [98–100] |
| rules | 97 % (145/150) [92–99], p = 0.062 | 97 % (145/150) [92–99], p = 0.062 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 77 % (115/150) [69–83], p = 5.8e-11 |
| axial | 63 % (94/150) [54–70], p = 2.8e-17 | 60 % (90/150) [52–68], p = 1.7e-18 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 70 % (105/150) [62–77], p = 5.7e-14 |
| edge | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 73 % (110/150) [66–80], p = 1.8e-12 |
| edge + axial | 39 % (59/150) [31–48], p = 8.1e-28 | 37 % (56/150) [30–46], p = 1e-28 | 100 % (150/150) [98–100], p = 1 | 100 % (150/150) [98–100], p = 1 | 78 % (117/150) [71–84], p = 2.3e-10 |
| bias ±2 % | 100 % (150/150) [98–100], p = 1 | 0 % (0/150) [0–2], p = 1.4e-45 | 0 % (0/150) [0–2], p = 1.4e-45 | 100 % (150/150) [98–100], p = 1 | 79 % (118/150) [71–85], p = 4.7e-10 |
| full model | 54 % (81/150) [46–62], p = 3.4e-21 | 0 % (0/150) [0–2], p = 1.4e-45 | 0 % (0/150) [0–2], p = 1.4e-45 | 99 % (149/150) [96–100], p = 1 | 69 % (103/150) [61–76], p = 1.4e-14 |

Edge + axial vs axial (paired, exact McNemar): hole detected (right hole, < 15 mm) p = 6.5e-05; within 3 mm (accuracy threshold) p = 9.6e-05; success (< 5 mm, < 10°) p = 1; pose found (< 15 mm, < 10°) p = 1; lifted (> 50 mm, both jaws) p = 0.096

## Accuracy (median [95 % seed-bootstrap])

| condition | detector: hole position error (mm) | detector: hole axis error (°) | fit: translation error (mm) | fit: rotation error (°) | grasp: hole position error at grasp time (mm) | grasp: approach direction error (°) |
|---|---|---|---|---|---|---|
| ideal | 0.69 [0.52–0.81] | 0.67 [0.58–0.75] | 0.35 [0.32–0.40] | 0.57 [0.49–0.67] | 0.44 [0.38–0.54] | 0.63 [0.53–0.71] |
| rules | 0.73 [0.59–0.79] | 0.66 [0.60–0.73] | 0.41 [0.37–0.47] | 0.58 [0.48–0.74] | 0.55 [0.48–0.64] | 0.51 [0.46–0.62] |
| axial | 0.83 [0.72–0.88] | 0.99 [0.80–1.16] | 0.75 [0.69–0.78] | 0.87 [0.80–0.94] | 0.72 [0.65–0.82] | 0.80 [0.68–0.94] |
| edge | 0.51 [0.41–0.62] | 1.06 [0.90–1.23] | 0.38 [0.34–0.42] | 0.56 [0.49–0.67] | 0.40 [0.30–0.51] | 1.06 [0.98–1.23] |
| edge + axial | 0.67 [0.55–0.73] | 1.30 [1.13–1.42] | 0.79 [0.73–0.83] | 0.94 [0.80–1.02] | 0.62 [0.55–0.71] | 1.10 [0.93–1.22] |
| bias ±2 % | 11.04 [9.08–11.29] | 0.71 [0.61–0.74] | 10.69 [10.55–10.83] | 0.55 [0.48–0.68] | 10.64 [9.10–10.85] | 0.68 [0.61–0.79] |
| full model | 10.51 [10.19–10.71] | 1.28 [1.08–1.43] | 11.05 [10.22–11.17] | 0.90 [0.81–0.97] | 10.36 [10.01–10.59] | 1.18 [1.00–1.30] |
