# KneeScanner

A 3D knee scanning and registration system designed to reconstruct and align the knee from multiple LiDAR scans captured on an iPhone/iPad.

The project explores **3D point-cloud processing, geometric registration, dimensionality reduction, and non-rigid deformation** to combine scans captured from different positions into a consistent 3D model.

> **Status:** Active development

---

## Overview

The goal of KneeScanner is to investigate whether consumer LiDAR hardware can be used to capture and reconstruct a 3D model of the knee while the leg is moved through different positions.

A single scan provides only a partial view of the knee. By capturing multiple scans from different positions and registering them into a common coordinate system, the system aims to produce a more complete representation of the underlying geometry.

The project combines an **iOS scanning application** with a **point-cloud processing and registration pipeline**.

### Core pipeline

```text
LiDAR Scan
    ↓
Point Cloud Preprocessing
    ↓
Downsampling
    ↓
Scale / Centre Normalisation
    ↓
PCA Alignment
    ↓
Rigid Registration (ICP)
    ↓
Non-Rigid Registration (CPD)
    ↓
Combined 3D Knee Model
```

---

## Technologies

### Software

- **Swift** — iOS application and LiDAR data capture
- **Python** — point-cloud processing and algorithm development
- **C++** — performance-critical registration experiments
- **Open3D** — 3D geometry and point-cloud processing
- **Eigen** — linear algebra
- **CMake** — C++ build configuration

### Concepts

- 3D point clouds
- LiDAR scanning
- Principal Component Analysis (PCA)
- Iterative Closest Point (ICP)
- Coherent Point Drift (CPD)
- Non-rigid registration
- Spatial transformations
- Point-cloud downsampling
- Numerical optimisation
- 3D reconstruction

---

## Point-Cloud Registration

A major part of the project is aligning scans captured from different knee positions.

Because the scans are not initially in the same coordinate system, several stages of registration are used.

### PCA Alignment

Principal Component Analysis is used to determine the dominant geometric axes of each point cloud.

This provides an initial estimate of orientation before more computationally expensive registration algorithms are applied.

### ICP Registration

Iterative Closest Point is then used to refine the rigid transformation between point clouds.

The algorithm iteratively:

1. Finds corresponding points.
2. Estimates the transformation between them.
3. Applies the transformation.
4. Repeats until convergence.

### CPD Registration

The project also investigates **Coherent Point Drift (CPD)** for non-rigid registration.

Unlike rigid registration, CPD allows the geometry to deform during alignment. This is important when scans of the knee are captured in different positions and the surrounding soft tissue changes shape.

A custom implementation of the CPD algorithm has been developed to investigate the underlying mathematics and provide greater control over the registration process.

---

## Why This Project?

Consumer LiDAR sensors provide relatively accessible 3D scanning capabilities, but producing a useful model from multiple scans is significantly more difficult than simply acquiring the data.

The project therefore focuses on the problems between **"I have several point clouds"** and **"I have one coherent 3D model."**

This involves investigating:

- How scans can be normalised before registration
- How good initial transformations affect convergence
- How point-cloud density affects computation
- How rigid and non-rigid registration behave on anatomical geometry
- How landmark information can be incorporated into registration
- How numerical optimisation affects the resulting geometry
- How to detect and diagnose unstable registration behaviour

---

## Development

The project is being developed incrementally, with individual components tested independently before being incorporated into the complete pipeline.

Current work includes investigating the behaviour of non-rigid registration and improving the stability of CPD when aligning real-world knee scans.

This has involved analysing failure modes such as:

- Localised deformation
- Excessive point displacement
- Unstable transformations
- Sensitivity to initial alignment
- Effects of downsampling
- Landmark constraints influencing the deformation field

These experiments are being used to better understand the behaviour of the underlying registration algorithms rather than treating them as black-box operations.

---

## Project Structure

```text
KneeScanner/
├── iOS/
│   └── LiDAR scanning application
│
├── Python/
│   └── Point-cloud processing and algorithm development
│
├── C++/
│   └── Registration implementation and experiments
│
├── data/
│   └── Example / test data
│
└── README.md
```

*Project structure may change as development continues.*

---

## Results

The system is currently capable of processing and registering 3D point-cloud data through multiple stages of alignment.

The primary focus at this stage is improving the robustness of the registration pipeline when working with real-world scans.

Example results and visualisations will be added as the reconstruction pipeline develops.

---

## Future Work

Planned improvements include:

- [ ] Improve CPD stability and convergence
- [ ] Improve correspondence estimation
- [ ] Investigate constrained/landmark-based registration
- [ ] Optimise registration performance
- [ ] Improve automated scan alignment
- [ ] Combine multiple scans into a complete surface
- [ ] Improve visualisation of registration results
- [ ] Evaluate accuracy against known reference geometry
- [ ] Integrate the complete processing pipeline into the iOS application

---

## Project Goals

The long-term goal is to develop a robust system capable of taking multiple LiDAR scans of a knee and automatically producing a consistent 3D representation suitable for further geometric analysis.

The project is primarily an exploration of **3D computer vision, point-cloud registration, numerical optimisation, and applied computational geometry**.

---

## Author

**James Senior**

Computer Science student | 3D Computer Vision | AI/ML | Autonomous Systems

[GitHub](https://github.com/JamesSenior)
