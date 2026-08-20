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

# Abstract

This project implements a hierarchical trajectory-tracking control framework for a multicopter based directly on the methodology presented in the base paper, **“Trajectory Tracking for a Multicopter under a Quaternion Representation.”** The implementation follows the proposed mathematical model and control architecture without modifying the core methodology. The approach combines **differential flatness, feedback linearization, quaternion-based attitude representation, and computed-torque control** to achieve accurate trajectory tracking for a multicopter.

The multicopter is modeled using nonlinear translational and rotational dynamics, with its attitude represented using unit quaternions. Differential flatness is employed to express the system states and control inputs in terms of the flat output, consisting of the three-dimensional position and one quaternion component. The high-level position controller uses position, velocity, and integral tracking errors to generate the required translational acceleration. From this desired acceleration, the required thrust and reference quaternion are obtained using the flatness-based formulation.

The generated reference quaternion is then provided to the low-level attitude controller. A computed-torque control law is used to compensate for the nonlinear rotational dynamics and accurately track the desired attitude. The quaternion formulation avoids the singularities associated with Euler-angle representations while maintaining a continuous representation of the multicopter orientation. The complete implementation includes the nonlinear multicopter model, quaternion kinematics, differential-flatness formulation, feedback-linearized position control, computed-torque attitude control, trajectory generation, and disturbance/drag modeling.

The implemented system is evaluated through simulation and trajectory-tracking experiments. The results demonstrate accurate position and attitude tracking and show that the controller can compensate for external disturbances while maintaining the desired trajectory.

# Introduction

Multicopters are nonlinear aerial vehicles whose motion is governed by coupled translational and rotational dynamics. For autonomous flight and trajectory tracking, the vehicle must simultaneously control its position in three-dimensional space and its orientation. The nonlinear relationship between the vehicle attitude, thrust direction, and translational motion makes accurate trajectory tracking a challenging control problem.

In this project, the multicopter attitude is represented using **unit quaternions** instead of conventional Euler angles. A quaternion provides a compact representation of three-dimensional orientation while avoiding the singularities and gimbal-lock problems that can occur with Euler-angle representations. The quaternion is therefore used directly in the dynamic model and control formulation.

The implemented control system follows a **hierarchical control architecture** consisting of a high-level position controller and a low-level attitude controller. The high-level controller receives the desired trajectory and the current vehicle state and determines the required translational acceleration. Using the differential-flatness formulation, this acceleration is converted into the required thrust and reference quaternion. The reference quaternion represents the attitude required for the multicopter to generate the desired translational motion.

The low-level controller is responsible for tracking this reference attitude. A computed-torque control strategy is used to compensate for the nonlinear rotational dynamics of the multicopter. Quaternion tracking errors are used to generate the corrective control action, allowing the actual attitude to converge toward the desired attitude.

The overall control process can therefore be represented as

\[
\text{Desired Trajectory}
\rightarrow
\text{Position Controller}
\rightarrow
\text{Desired Acceleration}
\rightarrow
\left(T,q_r\right)
\rightarrow
\text{Attitude Controller}
\rightarrow
\text{Multicopter Dynamics}.
\]

The methodology implemented in this project is based on the differential-flatness property of the multicopter system. The flat output is defined using the vehicle position and one quaternion component, allowing the remaining states and control inputs to be obtained algebraically from the flat output and its derivatives. This provides a systematic way of generating the thrust and attitude required to follow a given trajectory.

The position controller applies feedback linearization to the nonlinear translational dynamics. The resulting tracking problem is transformed into a linear error-dynamics problem, where proportional, derivative, and integral feedback terms are used to reduce the position tracking error. The resulting desired acceleration is subsequently used to calculate the reference quaternion and thrust.

For attitude control, the implementation uses computed-torque control. The nonlinear coupling present in the rotational dynamics is compensated using the system's inertia matrix, quaternion kinematics, and angular-velocity-dependent terms. This allows the attitude controller to focus on reducing the quaternion tracking error and provides fast convergence of the attitude toward its reference.

The complete implementation therefore integrates the multicopter dynamic model, quaternion representation, differential flatness, feedback linearization, and computed-torque control into a single trajectory-tracking framework. Simulation and experimental results are used to evaluate the ability of the implemented controller to track the desired trajectory and reject external disturbances.
