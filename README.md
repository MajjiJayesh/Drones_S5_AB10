<p align="center">
  <img src="Images/Amrita Logo.jpeg" width="180">
</p>

<h2 align="center">
Differential Flatness and Quaternion-Based Trajectory Tracking of a Quadrotor
</h2>

<p align="center">
MATLAB/Simulink Implementation and Simulation
</p>

---

## Team Members

| S. No. | Name | Roll Number | Email |
|---|---|---|---|
| 1 | Majji Jayesh | cb.sc.u4aie24128 | cb.sc.u4aie24128@cb.students.amrita.edu |
| 2 | Kotyada Siva Adityan | cb.sc.u4aie24126 | cb.sc.u4aie24126@cb.students.amrita.edu |
| 3 | Mandavalli Vishnu Teja | cb.sc.u4aie24130 | cb.sc.u4aie24130@cb.students.amrita.edu |
| 4 | Muvva Venkata Mukesh| cb.sc.u4aie24138 | cb.sc.u4aie24138@cb.students.amrita.edu |
| 5 | Narni Srinivas | cb.sc.u4aie24140 | cb.sc.u4aie24140@cb.students.amrita.edu |

# Trajectory Tracking for a Multicopter under a Quaternion Representation


## Abstract

This project implements and visualizes a hierarchical trajectory-tracking control framework for a multicopter using an attitude quaternion representation. The work is based on the paper **"Trajectory Tracking for a Multicopter under a Quaternion Representation"** by Huu Thien Nguyen, Ngoc Thinh Nguyen, Ionela Prodan, and Fernando Lobo Pereira.

The proposed approach separates the trajectory-tracking problem into two hierarchical control levels. The high-level position controller uses the differential-flatness properties of the multicopter to construct a feedback-linearization control law. This controller generates the required thrust and reference attitude quaternion from the position tracking error. The low-level attitude controller uses a computed-torque control law to stabilize the three controlled quaternion components.

The multicopter is represented using nonlinear translational and rotational dynamics. The attitude is represented using a unit quaternion rather than Euler angles, avoiding the singularities and gimbal-lock problems associated with Euler-angle representations.

The project implements the mathematical model, position controller, quaternion-based attitude controller, computed-torque control, aerodynamic drag model, numerical simulation, and visualization. The numerical simulation generates the position, velocity, acceleration, quaternion, thrust, and torque histories. The resulting motion is subsequently visualized using MuJoCo as a realistic quadcopter animation with four rotors, rotor rotation, attitude, ground environment, shadows, camera following, reference trajectory, actual trajectory, and the three reference positions used in the paper.

---

# 1. Introduction

Multicopters and quadrotors are widely used in applications such as aerial photography, package delivery, inspection, surveillance, and autonomous navigation. A fundamental requirement in these applications is the ability of the vehicle to follow a desired three-dimensional trajectory accurately.

The control of a multicopter is challenging because its translational and rotational motions are nonlinear and strongly coupled. A common approach is to use a hierarchical control structure in which the translational and rotational subsystems are controlled separately.

The reference paper proposes a two-layer hierarchical control scheme based on quaternion attitude representation. The high-level controller is responsible for trajectory tracking and generates the required thrust and reference quaternion. The low-level controller uses computed-torque control to stabilize the attitude around the desired quaternion.

The paper specifically develops:

1. A nonlinear multicopter model using quaternion attitude representation.
2. A differential-flatness representation of the multicopter.
3. A feedback-linearization position controller.
4. A computed-torque attitude controller.
5. A simulation model including aerodynamic drag.
6. Experimental validation using a nano-drone quadcopter.

The proposed control structure is therefore:

```text
Desired trajectory
       |
       v
+-----------------------+
| Position Controller   |
| Feedback Linearization|
+-----------------------+
       |
       | Desired thrust T
       | Reference quaternion qr
       v
+-----------------------+
| Attitude Controller   |
| Computed Torque       |
+-----------------------+
       |
       | Torque tau
       v
+-----------------------+
| Multicopter Dynamics  |
+-----------------------+
       |
       v
Actual position
Actual attitude
       |
       +-------------------- feedback --------------------+
