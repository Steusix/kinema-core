# Planar 2-DOF Robotic Arm: Inverse Kinematics & PID Control

A real-time C++20 physics and control simulator for a 2-Degree-of-Freedom (2-DOF) planar manipulator. The project demonstrates analytical Inverse Kinematics (IK) using the Law of Cosines, decoupled joint-space PID controllers, and interactive visualization.

![Simulation Demo](assets/demo.gif)

## Features
* **Analytical Inverse Kinematics:** Real-time calculation of target joint angles ($\theta_1, \theta_2$) for any reachable $(x, y)$ Cartesian target, with workspace boundary clamping.
* **Joint-Space PID Control:** Independent PID controllers driving each joint smoothly toward target angles without overshoot.
* **Interactive Visualization:** Real-time 60 FPS rendering using `Raylib`, featuring target selection via mouse click, end-effector trajectory trail, and live telemetry overlay.
* **Modern C++20 & Eigen:** Linear algebra and vector transformations powered by `Eigen3`, built cross-platform with `CMake`.

## Mathematical Model

### Forward Kinematics
Given link lengths $L_1, L_2$ and joint angles $\theta_1, \theta_2$:
$$x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2)$$
$$y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)$$

### Inverse Kinematics (Elbow-Down / Elbow-Up)
For a target position $(x, y)$ clamped to $r \le L_1 + L_2$:
$$\cos(\theta_2) = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$
$$\theta_1 = \text{atan2}(y, x) - \text{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)$$

## Build & Run

### Prerequisites
* C++20 compatible compiler (`GCC 11+`, `Clang 14+`, or `MSVC`)
* `CMake 3.16+`
* `Git` (for automatic dependency fetching via `FetchContent`)

### Compilation
```bash
cmake -B build
cmake --build build --config Release
./build/arm_sim