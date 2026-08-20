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

$$
\text{Desired Trajectory}
\rightarrow
\text{Position Controller}
\rightarrow
\text{Desired Acceleration}
\rightarrow
(T,q_r)
\rightarrow
\text{Attitude Controller}
\rightarrow
\text{Multicopter Dynamics}
$$

The methodology implemented in this project is based on the differential-flatness property of the multicopter system. The flat output is defined using the vehicle position and one quaternion component, allowing the remaining states and control inputs to be obtained algebraically from the flat output and its derivatives. This provides a systematic way of generating the thrust and attitude required to follow a given trajectory.

The position controller applies feedback linearization to the nonlinear translational dynamics. The resulting tracking problem is transformed into a linear error-dynamics problem, where proportional, derivative, and integral feedback terms are used to reduce the position tracking error. The resulting desired acceleration is subsequently used to calculate the reference quaternion and thrust.

For attitude control, the implementation uses computed-torque control. The nonlinear coupling present in the rotational dynamics is compensated using the system's inertia matrix, quaternion kinematics, and angular-velocity-dependent terms. This allows the attitude controller to focus on reducing the quaternion tracking error and provides fast convergence of the attitude toward its reference.

The complete implementation therefore integrates the multicopter dynamic model, quaternion representation, differential flatness, feedback linearization, and computed-torque control into a single trajectory-tracking framework. Simulation and experimental results are used to evaluate the ability of the implemented controller to track the desired trajectory and reject external disturbances.

# Methodology

## 1. Overall Methodology

The project implements the **two-layer hierarchical trajectory-tracking control scheme** proposed in the base paper. The methodology is based on the complete nonlinear multicopter model expressed using a unit quaternion, followed by differential-flatness-based feedback linearization for position control and computed-torque control for attitude stabilization. The same control structure described in the paper is implemented without changing the core methodology. :contentReference[oaicite:0]{index=0}

The complete control methodology consists of the following stages:

1. Model the multicopter translational and rotational dynamics.
2. Represent the multicopter attitude using a unit quaternion.
3. Obtain the rotation matrix from the quaternion.
4. Relate quaternion derivatives to the body angular velocity.
5. Establish the differential-flatness representation of the multicopter.
6. Select the flat output as the three-dimensional position and the fourth quaternion component.
7. Calculate the required thrust and quaternion components from the desired translational acceleration.
8. Use the high-level feedback-linearization position controller to generate the corrected desired acceleration.
9. Generate the reference quaternion and thrust from the corrected acceleration.
10. Send the three vector components of the reference quaternion to the low-level attitude controller.
11. Use computed-torque control to compensate for the nonlinear rotational dynamics.
12. Track the reference quaternion using proportional, derivative, and integral quaternion feedback.
13. Propagate the resulting translational and rotational dynamics.
14. Evaluate the trajectory-tracking performance through simulation and experimental results.

The architecture is therefore

$$
\text{Reference Position}
\rightarrow
\text{Position Error}
\rightarrow
\text{Feedback Linearization}
\rightarrow
\left(T,q_r\right)
\rightarrow
\text{Computed-Torque Attitude Control}
\rightarrow
\text{Multicopter Dynamics}.
$$

The low-level attitude controller is operated at a higher frequency than the high-level position controller so that the attitude subsystem can track the generated reference quaternion sufficiently fast. :contentReference[oaicite:1]{index=1}

---

## 2. Multicopter State Representation

The multicopter position is represented by

$$
\xi =
\begin{bmatrix}
x & y & z
\end{bmatrix}^{T},
$$

where $x$, $y$, and $z$ represent the position of the multicopter in the global frame.

The body angular velocity is

$$
\omega =
\begin{bmatrix}
\omega_x & \omega_y & \omega_z
\end{bmatrix}^{T}.
$$

The attitude is represented using the unit quaternion

$$
q =
\begin{bmatrix}
q_0 & q_1 & q_2 & q_3
\end{bmatrix}^{T},
$$

where $q_0$ is the scalar component and

$$
q =
\begin{bmatrix}
q_1 & q_2 & q_3
\end{bmatrix}^{T}
$$

contains the remaining three components.

The quaternion satisfies the unit-norm constraint

$$
q_0^2+q_1^2+q_2^2+q_3^2=1,
$$

with

$$
q_0\geq0.
$$

Therefore, the scalar component can be reconstructed from the three vector components as

$$
q_0=
\sqrt{1-q_1^2-q_2^2-q_3^2}.
$$

This reduced representation is important because the control formulation directly uses the three components $q_1,q_2,q_3$, while $q_0$ is obtained from the unit-norm constraint. :contentReference[oaicite:2]{index=2}

---

## 3. Quaternion Rotation Matrix

The orientation of the multicopter body frame with respect to the global frame is represented by the rotation matrix

$$
R=
\begin{bmatrix}
1-2(q_2^2+q_3^2)
&
2(q_1q_2-q_0q_3)
&
2(q_0q_2+q_1q_3)
\\
2(q_1q_2+q_0q_3)
&
1-2(q_1^2+q_3^2)
&
2(q_2q_3-q_0q_1)
\\
2(q_1q_3-q_0q_2)
&
2(q_0q_1+q_2q_3)
&
1-2(q_1^2+q_2^2)
\end{bmatrix}.
$$

The rotation matrix determines the direction of the multicopter thrust in the global coordinate frame and therefore directly connects the attitude to the translational motion. :contentReference[oaicite:3]{index=3}

---

## 4. Quaternion Kinematics

The relationship between quaternion derivative and body angular velocity is

$$
\dot q=\frac{1}{2}q\otimes\bar{\omega},
$$

where

$$
\bar{\omega}=
\begin{bmatrix}
0 & \omega_x & \omega_y & \omega_z
\end{bmatrix}^{T}.
$$

The corresponding inverse relationship is

$$
\begin{bmatrix}
0\\
\omega_x\\
\omega_y\\
\omega_z
\end{bmatrix}
=
2
\begin{bmatrix}
q_0&q_1&q_2&q_3\\
-q_1&q_0&q_3&-q_2\\
-q_2&-q_3&q_0&q_1\\
-q_3&q_2&-q_1&q_0
\end{bmatrix}
\begin{bmatrix}
\dot q_0\\
\dot q_1\\
\dot q_2\\
\dot q_3
\end{bmatrix}.
$$

Using the reduced quaternion vector, this is written compactly as

$$
\omega=Q(q)\dot q.
$$

The matrix $Q(q)$ is obtained using the unit-quaternion relation

$$
q_0=
\sqrt{1-q_1^2-q_2^2-q_3^2}.
$$

These equations provide the connection between quaternion motion and rotational dynamics required by the computed-torque controller. :contentReference[oaicite:4]{index=4}

---

## 5. Nonlinear Translational Dynamics

The translational motion of the multicopter is described by Newton's second law:

$$
m\ddot{\xi}=mg+Re_zT
$$

where $m$ is the multicopter mass, $g$ is the gravitational acceleration, $R$ is the rotation matrix, $e_z$ is the unit vector along the vertical axis, and $T$ is the total thrust.

The position vector is

$$
\xi=
\begin{bmatrix}
x\\
y\\
z
\end{bmatrix}
$$

and the vertical unit vector is

$$
e_z=
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
$$

Expanding the translational dynamics gives

$$
\begin{bmatrix}
\ddot{x}\\
\ddot{y}\\
\ddot{z}
\end{bmatrix}
=
\begin{bmatrix}
0\\
0\\
-g
\end{bmatrix}
+
\frac{T}{m}
\begin{bmatrix}
2(q_0q_2+q_1q_3)\\
2(q_2q_3-q_0q_1)\\
q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}
$$

This nonlinear translational model describes how the thrust and multicopter attitude determine the acceleration of the vehicle.
