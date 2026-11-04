# RealSense D435 Depth Simulation: Literature Notes

Nov 3, 2026 · online version: https://claude.ai/artifact/NDdykcHzKFRfK1Si1WQWwF

## Purpose and scope

The D435 has been modelled many times, at three levels of fidelity: physics-based IR stereo simulation, analytic noise models fitted to measurements, and learned noise. This doc collects that work so our camera model can reuse proven choices and cite them.

Our setting: a Unitree G1 with its head-mounted D435 (848x480, 0.5-0.65 m to the parts) looking at an item Line 8 40x40 clear-anodized aluminium profile and a powder-coated die-cast zinc angle bracket (8.5 mm holes). The questions are which effects matter on such parts, and which existing models we can copy or validate against.

Each section lists papers or tools with what they model; the last two sections turn this into decisions for our model and a reading order.

## Physics-based active stereo simulators

The strongest prior work simulates the sensing process itself: render left and right IR images with the projected dot pattern and real materials, then run stereo matching. Material-dependent holes and outliers then emerge instead of being hand-coded. All three below target the RealSense D415, the D435's sibling with the same D4 matcher.

| Work | Venue | What is simulated | Sensor | Code |
| --- | --- | --- | --- | --- |
| [Zhang et al., Close the Optical Sensing Domain Gap by Physics-Grounded Active Stereo Sensor Simulation](https://angli66.github.io/active-sensor-sim) | IEEE T-RO 2023, vol. 39(3) | Material acquisition, ray-traced IR rendering, IR noise, stereo depth estimation; real time (60+ FPS) | D415 (modules configurable) | In SAPIEN: `sapien.sensor.StereoDepthSensor` |
| [Dai et al., DREDS](https://pku-epic.github.io/DREDS) | ECCV 2022 | Blender ray tracing of IR stereo patterns, depth from them; randomised specular / transparent / diffuse materials; 130K synthetic RGB-D | Active stereo resembling D415 ([paper](https://arxiv.org/pdf/2208.03792)) | [PKU-EPIC/DREDS](https://github.com/PKU-EPIC/DREDS) |
| [Liu et al., RaSim](https://arxiv.org/abs/2404.03962) | ICRA 2024 | Depth synthesis imitating the sensor's imaging principle, plus range-aware rendering | Not named on the abstract page | Not stated |

Relevance for us: Zhang et al. is the closest match to a Level-2 (IR stereo) D435 model and is already packaged as a reusable sensor class. DREDS shows the same idea specifically for specular objects, and pairs it with a real D415/D435 test set (see Datasets).

## NVIDIA ecosystem: Isaac Sim, Newton, FoundationStereo

Isaac Sim 5.1 already ships a stereo depth noise model that works in the disparity domain; it does not simulate IR images or real stereo matching. Newton documents no depth noise model. FoundationStereo is the main NVIDIA work on synthetic-to-real stereo, but it targets RGB stereo pairs.

| Tool | What it provides for depth | Status for D435 |
| --- | --- | --- |
| [Isaac Sim 5.1 Depth Sensors](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera_depth.html) (`SingleViewDepthSensor`) | Post-process on one camera view: renders disparity, adds noise, confidence and outlier filtering; output annotator `DepthSensorDistance` next to ideal `DistanceToImagePlane`. Intended for stereo cameras only | Ships a D455 asset (`/Isaac/Sensors/Intel/RealSense/rsd455.usd`), no D435. Docs say to tune attributes until rendered depth matches real depth |
| [Depth sensor schema](https://docs.omniverse.nvidia.com/kit/docs/omni.usd.schema.omni_sensors/110.3.0/omni.sensors.schema/omni.sensors.schema.OmniSensorDepthSensorSingleViewAPI.html) (`omni:rtx:post:depthSensor:*`) | Attributes and defaults from the installed 5.1 schema: baselineMM 55, focalLengthPixel 897, sensorSizePixel 1280, maxDisparityPixel 110, noiseMean 0.25, noiseSigma 0.25, noiseDownscaleFactorPixel 1, confidenceThreshold 0.95, outlierRemovalEnabled 3 (px), minDistance 0.5, maxDistance 1e7 | Can be set to D435 values (B = 50 mm, f = 447 px at 848x480). Units of noise and distances are not documented |
| [Newton](https://www.therobotreport.com/nvidia-launches-newton-physics-engine-gr00t-ai-corl-2025/) (NVIDIA, Google DeepMind, Disney Research; Linux Foundation; beta Sep 2025) | Physics engine on Warp + OpenUSD. [PyPI](https://pypi.org/project/newton/) lists a `sensor_camera` example (v1.7.0) | No stereo or depth noise model found in its [docs](https://newton-physics.github.io/newton/1.7.0/guide/overview.html) |
| [FoundationStereo](https://nvlabs.github.io/FoundationStereo) (CVPR 2025) | Stereo network trained on \~1M synthetic stereo pairs; zero-shot; compared against D435 point clouds | Not a sensor model; IR-pair input untested on the project page |

Relevance for us: the Isaac depth sensor is a ready Level-1.5 baseline (disparity noise + filtering) we can configure as a D435 and compare against our own model. Its defaults resemble a D415-class sensor, so they must be changed explicitly.

## Analytic and empirical noise models

The common analytic form treats depth noise as Gaussian, split into axial (along the ray) and lateral (in the image) parts, with axial noise growing with distance and steeply with incidence angle near grazing. For the D435 this form was fitted by Ahn et al. (2019); Intel's own tuning paper gives the distance-squared term from stereo geometry.

Axial noise model in the form used after Nguyen et al. (Kinect) and Ahn et al. (D435), as written in [Nguyen-style model, arXiv 2412.15040](https://arxiv.org/html/2412.15040v1), with depth z and incidence angle θ:

```latex
\sigma_z(z,\theta) = a + b\,z + c\,z^2 + d\,z^{n}\,\frac{\theta^2}{(\pi/2-\theta)^2}
```

Correction after reading Ahn et al. (2019) in full (docs/793350771-ANM.pdf, Eq. 2): the D435 fit uses θ, not θ², in the angle term: σ_z = 0.001063 + 0.0007278 z + 0.003949 z² + 0.022 z^{3/2} θ/(π/2 − θ)² (m). The angle coefficients were set by hand ("heuristically obtained"), not fitted. Nguyen et al.'s Kinect model is the one with θ².

Intel's theoretical floor for the RMS error on a well-textured plane (BKM, in docs/), with subpixel ≈ 0.08 px:

```latex
\sigma_z = \frac{z^2 \cdot \text{subpixel}}{f_{px} \cdot B}
```

| Work | Sensor | What it gives | Use for us |
| --- | --- | --- | --- |
| Ahn et al., Analysis and Noise Modeling of the Intel RealSense D435 for Mobile Robots (UR 2019), full text in docs/793350771-ANM.pdf | D435 | Axial and lateral Gaussian noise vs distance and incidence angle; lateral noise treated as independent of distance and angle | Direct functional form for our noise term; coefficients still to be read from the paper |
| [Halmetschlager-Funek et al., An Empirical Evaluation of Ten Depth Cameras](https://www.acin.tuwien.ac.at/?p=8369) (IEEE RAM 2018/19) | 10 sensors: structured light, active stereo, ToF | Bias, precision, lateral noise, 4 light levels, 6 materials including aluminium, multi-sensor NaN ratio; data and quadratic error models downloadable | Only study found that measures aluminium as a material; check whether the D435 is among the ten |
| Intel BKM tuning paper (docs/) | D400 family | Z² error formula, subpixel 0.05-0.1 px, MinZ, presets (High Accuracy vs High Density) | Noise floor and validation number |
| [Nguyen-style models for other sensors (Fankhauser n = 1.5, Khoshelham n = -0.5)](https://arxiv.org/html/2412.15040v1) | Kinect v1/v2, PMD Flexx2 | Same form with different exponents | Shows the form transfers across sensors |

No formal measurement study of D435 error on metal or specular parts was found; evidence there is from forum reports (large invalid holes on reflective floors and sheet metal) and from the physics-based simulators above.

## Learned noise and robot-learning practice

Robot-learning papers that deploy depth policies on real robots rarely simulate the IR process; they ray-cast ideal depth and add a hand-built noise stack (distance-dependent Gaussian, edge dropout, hole patches, blur). Ablations show the stack matters: removing it raised terrain reconstruction error by \~38% in DPL. Learned approaches either restore real depth (DREDS) or learn from mixed sim and real data (ActiveZero).

| Work | Type | Noise components | Reported effect |
| --- | --- | --- | --- |
| [DPL: Depth-only Perceptive Humanoid Locomotion](https://arxiv.org/html/2510.07152v1) (arXiv 2025) | Hand-built, ray-cast depth | Axial Gaussian σ\_z(z, θ) with tuned coefficients; lateral noise ∝ z; edge-conditioned dropout (Sobel); per-pixel dropout from an uncertainty map; self-occlusion by robot mesh; border crop and resize. Camera: Orbbec 355L, not D435 | Terrain MAE 3.25 cm with noise model vs 4.48 cm without |
| [Rudin et al., Parkour in the Wild](https://arxiv.org/abs/2505.11164) (arXiv 2025, ANYmal D) | Hand-built | Per the paper text (not checked here): edge dropout and shuffling, Perlin-noise hole patches set to max depth, Gaussian blur, for a D435i | Zero-shot deployment |
| [Planche et al., DepthSynth](https://arxiv.org/abs/1702.08558) (3DV 2017) | Physics-style pipeline from CAD | Simulates the full sensing mechanism: sensor noise, material reflectance, surface geometry | Improves recognition networks trained on it |
| [Dai et al., DREDS / SwinDRNet](https://pku-epic.github.io/DREDS) (ECCV 2022) | Simulated data + learned restoration | Physics-based D415 simulation, then a network restores depth on specular and transparent objects | Restoration at \~30 FPS; works on D415 and D435 data |
| ActiveZero, Liu et al. (CVPR 2022, [arXiv 2112.02772](https://arxiv.org/pdf/2112.02772)) | Learned active stereo | Supervised on simulated IR stereo, self-supervised on real IR (temporal IR reprojection loss) | Reported to beat a commercial sensor on real data; code haosulab/active\_zero |

Relevance for us: the robot-learning stack (Gaussian axial noise + edge dropout + holes) is the cheap, well-precedented baseline. It encodes no material, so on aluminium it can only be tuned, not predicted.

## Real datasets for validation

Two public datasets pair real RealSense depth with accurate ground truth on reflective objects: HAMMER (D435) and DREDS-STD (D415). Neither contains aluminium profiles, so they can validate the type of error (holes and outliers on specular surfaces), not our exact parts.

| Dataset | Sensors | Content | Ground truth | Use for us |
| --- | --- | --- | --- | --- |
| [HAMMER](https://www.github.com/Junggy/HAMMER-dataset) ([paper](https://ar5iv.arxiv.org/html/2205.04565)) | RealSense D435, L515, Lucid Helios ToF, polarisation RGB | 13 indoor scenes; materials grouped diffuse / reflective / transparent | Depth rendered per sensor from aligned 3D scans | Best match: real D435 vs GT on reflective objects; compare our simulated error statistics per material class |
| [DREDS STD](https://pku-epic.github.io/DREDS) | RealSense D415 | STD-CatKnown 27,000 RGB-D of 42 objects in 25 scenes; STD-CatNovel 11,000 of 8 objects | Object CAD + poses | Specular and transparent objects; D415 matcher is the same family |
| [Halmetschlager-Funek et al. data](https://www.acin.tuwien.ac.at/?p=8369) | 10 depth cameras | Planar targets; 6 materials including aluminium; 4 light levels | Measured setup | Numbers for aluminium vs diffuse targets, if the D435 is included |

If none fits closely enough, the fallback is to validate only against the datasheet KPIs and the BKM noise curve, and to say so in the report.

## Implications for our camera model

The literature supports a layered model: a well-precedented geometric noise stack that runs on any backend's ideal depth, plus a material-aware layer for aluminium that only physics-based work models directly. The experiment can then compare these layers, which mirrors the three levels found in the literature.

| Effect | Supported by | Needs from our frame | Backend |
| --- | --- | --- | --- |
| Axial noise growing with z² | Intel BKM formula; Ahn et al. D435 fit | Ideal depth, K, baseline | Both |
| Incidence-angle term (noise rises near grazing) | Nguyen-style model as used for D435 by Ahn et al. | Normals (from depth or mesh) | Both |
| Lateral noise / edge smearing | Ahn et al. (lateral: constant 41–43 mm, 90th percentile, no trend; not usable at our scale); DPL lateral ∝ z | Depth, instance edges | Both |
| Holes at depth discontinuities | DPL edge dropout; Parkour in the Wild; Isaac outlier removal | Depth gradients | Both |
| Disparity quantisation and confidence filtering | Isaac depth sensor schema (confidence 0.95, outlier 3 px) | Disparity from depth | Both (Isaac native) |
| Left invalid band, MinZ | Datasheet section 4.5, Table 4-12 | Depth, K, baseline | Both |
| Specular dropouts and outliers on metal | Zhang et al. T-RO 2023; DREDS; HAMMER; forum reports | Material ID, roughness, normal vs camera direction | Rule-based in both; emergent only with IR stereo rendering |
| Material-dependent IR return (anodized vs bare vs painted) | Zhang et al. (material acquisition); Halmetschlager-Funek (aluminium target) | Material table (optical fields) | Rule-based |

Two routes for the metal effects:

1. Rule-based, in `camera_model/d435.py`: dropout and outlier probability from incidence angle, roughness and material. Cheap, transparent, same on PyBullet and Isaac; must be tuned, can be checked against HAMMER statistics.
2. Physics-based, via SAPIEN's `StereoDepthSensor` or Isaac RTX IR rendering plus SGM: effects emerge from rendering. Stronger claim, much more effort, and PyBullet cannot do it.

Isaac's built-in `SingleViewDepthSensor` set to D435 values is a free extra baseline for the experiment, independent of our own code.

## Reading order and open questions

Read Ahn et al. first for D435 coefficients, then Zhang et al. for the physics-based route, then HAMMER for validation data.

1. Ahn et al., Analysis and Noise Modeling of the Intel RealSense D435 for Mobile Robots, UR 2019: full text (docs/793350771-ANM.pdf) read by the AI agent on 2026-11-04. Axial Eq. 2 used as the ablation (θ = 0); the angle term is hand-set and the lateral values (41–43 mm) describe edge scatter, see NOTES.md Q2.
2. [Zhang et al., T-RO 2023](https://angli66.github.io/active-sensor-sim): how materials and IR noise are parameterised; try the SAPIEN `StereoDepthSensor` notebook.
3. [HAMMER](https://www.github.com/Junggy/HAMMER-dataset): D435 error statistics on reflective vs diffuse objects.
4. [Isaac Sim 5.1 depth sensors](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera_depth.html): units and meaning of `noiseMean`, `noiseSigma`, `confidenceThreshold`.
5. [DREDS](https://pku-epic.github.io/DREDS) and its supplement: D415 projector and pattern parameters.
6. [Halmetschlager-Funek et al.](https://www.acin.tuwien.ac.at/?p=8369): aluminium target results and the downloadable error models.
7. Carfagni et al., Metrological and Critical Characterization of the Intel D415 Stereo Depth Camera, Sensors 19(3):489, 2019 (doi 10.3390/s19030489): close-range (0.1-1 m) metrology, not opened yet.

Open questions:

- [ ] Is the D435 among the ten cameras in Halmetschlager-Funek et al., and what is its precision on aluminium?
- [ ] What units do Isaac's `noiseMean` / `noiseSigma` use (disparity px?), and does the single-view model simulate occlusion between the two imagers?
- [ ] Does Newton's camera sensor have any noise model, or only ideal depth?
- [ ] Is a real D435 capture of anodized aluminium available anywhere (forums, datasets) to tune the specular rule?

## Sources

All links above were opened by the AI agent on the as-of date, except Carfagni et al. (access blocked) and the full text of Parkour in the Wild (only the abstract or secondary descriptions read). The full text of Ahn et al. was later read by the AI agent (2026-11-04). Intel datasheet and BKM tuning paper: `docs/` in the project repository.
