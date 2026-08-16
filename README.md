## Team Members

| S. No. | Name | Roll Number | Email |
|---|---|---|---|
| 1 | Majji Jayesh | cb.sc.u4aie24128 | cb.sc.u4aie24128@cb.students.amrita.edu |
| 2 | Kotyada Siva Adityan | cb.sc.u4aie24126 | cb.sc.u4aie24126@cb.students.amrita.edu |
| 3 | Mandavalli Vishnu Teja | cb.sc.u4aie24130 | cb.sc.u4aie24130@cb.students.amrita.edu |
| 4 | Muvva Venkata Mukesh| cb.sc.u4aie24138 | cb.sc.u4aie24138@cb.students.amrita.edu |
| 5 | Narni Srinivas | cb.sc.u4aie24140 | cb.sc.u4aie24140@cb.students.amrita.edu |

# Drones---Trajectory-Tracking-for-a-Multicopter-under-a-Quaternion-Representation

## Overview

This project implements a quaternion-based trajectory tracking controller for a quadcopter UAV. The objective is to enable a drone to accurately follow predefined three-dimensional trajectories while maintaining stable orientation using quaternion mathematics.

Unlike conventional Euler angle based controllers, this project employs quaternion representation to eliminate singularities such as gimbal lock and provide smooth attitude control during aggressive maneuvers.

The project follows the hierarchical control framework presented in the paper:

> Huu Thien Nguyen et al.,
> "Trajectory Tracking for a Multicopter under a Quaternion Representation,"
> IFAC Papers OnLine, 2020.

---

## Project Objectives

- Model quadcopter dynamics using quaternion representation
- Implement quaternion kinematics and rigid body dynamics
- Implement differential flatness based trajectory generation
- Design a feedback linearization position controller
- Design a computed torque attitude controller
- Validate the controller through simulation
- Deploy the controller on a real quadcopter platform

---

## Motivation

Traditional quadcopter controllers commonly represent orientation using Euler angles (Roll-Pitch-Yaw). Although intuitive, Euler angles suffer from singularities (gimbal lock) and become unreliable during large rotations.

Quaternion representation provides

- Singularity-free orientation
- Smooth rotational interpolation
- Better numerical stability
- Efficient attitude computation

This project demonstrates how quaternion mathematics can be applied to practical quadcopter trajectory tracking.

---



### High-Level Controller

The position controller

- receives the desired trajectory
- computes position error
- generates desired acceleration
- computes desired thrust
- computes the reference quaternion

Controller type

- Feedback Linearization
- PID Correction
- Differential Flatness

---

### Low-Level Controller

The attitude controller

- tracks the reference quaternion
- computes control torques
- stabilizes drone orientation

Controller type

- Computed Torque Control (CTC)

---

## Mathematical Model

The project implements

### Rotation Kinematics

- Unit Quaternion
- Quaternion Derivative
- Rotation Matrix

### Dynamics

- Translational Dynamics
- Rotational Dynamics
- Newton-Euler Equations

### Differential Flatness

Flat Output


## Control Architecture

The controller is implemented using a hierarchical two-layer architecture.

