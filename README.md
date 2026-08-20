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

# Trajectory Tracking of a Multicopter Under Quaternion Representation

## Abstract

This project implements a nonlinear trajectory-tracking framework for a multicopter using an attitude-quaternion representation and a hierarchical control architecture.

The implementation is based on the methodology presented in:

> H. T. Nguyen, N. T. Nguyen, I. Prodan, and F. L. Pereira,  
> **"Trajectory Tracking for a Multicopter under a Quaternion Representation,"**  
> IFAC-PapersOnLine, vol. 53, no. 2, pp. 5731–5736, 2020.

The main objective is to reproduce the mathematical modeling and trajectory-tracking methodology of the paper in a numerical simulation environment and visualize the resulting multicopter motion using MuJoCo.

The implemented framework consists of:

1. Quaternion-based nonlinear multicopter dynamics.
2. Differential-flatness-based representation.
3. Feedback-linearization position controller.
4. Reference quaternion generation from the desired translational acceleration.
5. Computed-torque attitude controller.
6. Quaternion-based attitude propagation.
7. Numerical integration of the nonlinear dynamics.
8. Trajectory and quaternion response visualization.
9. Real-time multicopter visualization using MuJoCo.

The controller is organized hierarchically. The high-level position controller receives the desired position and actual position and generates the required thrust and reference attitude. The low-level attitude controller tracks the generated quaternion reference using computed-torque control.

---

# 1. Introduction

Multicopters are nonlinear, underactuated aerial vehicles whose translational motion is strongly coupled with their attitude. A change in attitude modifies the direction of the thrust vector and consequently affects the translational motion of the vehicle.

Traditional Euler-angle representations are intuitive but can suffer from singularities and gimbal-lock problems. The reference paper therefore formulates the multicopter attitude using unit quaternions.

A quaternion provides a compact representation of the vehicle attitude while avoiding the singularities associated with Euler-angle parameterizations.

The reference paper develops a complete nonlinear multicopter model using quaternion attitude representation and exploits differential flatness to construct a hierarchical trajectory-tracking controller.

The proposed structure contains two control levels:

- **High-level position controller:** feedback-linearization controller.
- **Low-level attitude controller:** computed-torque controller.

The paper explicitly describes this hierarchical architecture, where the position controller generates thrust and a reference quaternion, while the attitude controller tracks the last three components of the reference quaternion. :contentReference[oaicite:1]{index=1}

---

# 2. Objectives

The objectives of this project are:

- To implement the nonlinear multicopter model using quaternion representation.
- To implement the differential-flatness-based trajectory representation.
- To implement the feedback-linearization position controller.
- To generate the desired attitude quaternion from translational acceleration.
- To implement the computed-torque attitude controller.
- To numerically integrate the nonlinear multicopter dynamics.
- To compare reference and actual trajectories.
- To analyze quaternion response with respect to time.
- To visualize the multicopter trajectory in three dimensions.
- To visualize the resulting drone motion using MuJoCo.

---

# 3. Mathematical Model

## 3.1 Quaternion Representation

The attitude of the multicopter is represented using the unit quaternion

$$
q =
\begin{bmatrix}
q_0 & q_1 & q_2 & q_3
\end{bmatrix}^{T}
$$

where

$$
q_0 \geq 0
$$

and the quaternion satisfies the unit-norm constraint

$$
q_0^2+q_1^2+q_2^2+q_3^2=1.
$$

The implementation uses the reduced quaternion

$$
\mathbf{q} =
\begin{bmatrix}
q_1 & q_2 & q_3
\end{bmatrix}^{T}
$$

and reconstructs the scalar component using

$$
q_0 =
\sqrt{1-q_1^2-q_2^2-q_3^2}.
$$

This is Equation (6) of the reference paper. :contentReference[oaicite:2]{index=2}

---

# 4. Quaternion Rotation Matrix

The rotation matrix corresponding to the quaternion is

$$
R =
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

This rotation matrix maps the body-frame orientation into the global frame and determines the direction of the thrust vector.

The matrix is given as Equation (2) in the paper. :contentReference[oaicite:3]{index=3}

---

# 5. Quaternion Kinematics

The quaternion derivative is related to the angular velocity by

$$
\dot q =
\frac{1}{2}
q\otimes
\bar{\omega}
$$

where

$$
\bar{\omega}
=
\begin{bmatrix}
0 & \omega_x & \omega_y & \omega_z
\end{bmatrix}^{T}.
$$

The paper also provides the equivalent relationship

$$
\omega = Q(q)\dot{\mathbf q}.
$$

This relationship is used to transform between the reduced quaternion derivative and the body angular velocity. :contentReference[oaicite:4]{index=4}

---

# 6. Multicopter Translational Dynamics

Let the position of the multicopter be

$$
\xi =
\begin{bmatrix}
x & y & z
\end{bmatrix}^{T}.
$$

The translational dynamics are

$$
m\ddot{\xi}
=
mg + R e_z T
$$

where

$$
g =
\begin{bmatrix}
0\\
0\\
-g
\end{bmatrix},
\qquad
e_z =
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}.
$$

Therefore,

$$
m
\begin{bmatrix}
\ddot{x}\\
\ddot{y}\\
\ddot{z}
\end{bmatrix}
=
m
\begin{bmatrix}
0\\
0\\
-g
\end{bmatrix}
+
R
\begin{bmatrix}
0\\
0\\
T
\end{bmatrix}.
$$

The gravity vector acts in the negative global $z$ direction, while the thrust direction is determined by the multicopter attitude.

This is Equation (7) of the paper. :contentReference[oaicite:5]{index=5}

---

# 7. Rotational Dynamics

The multicopter is modeled as a three-dimensional rigid body.

The rotational dynamics are

$$
J\dot{\omega}
+
\omega\times(J\omega)
=
\tau
$$

where

$$
J =
\operatorname{diag}(J_x,J_y,J_z)
$$

is the inertia tensor and

$$
\tau =
\begin{bmatrix}
\tau_x & \tau_y & \tau_z
\end{bmatrix}^{T}
$$

is the applied body torque.

The corresponding angular acceleration is

$$
\dot{\omega}
=
J^{-1}
\left[
\tau-\omega\times(J\omega)
\right].
$$

This corresponds to Equation (8) in the paper. :contentReference[oaicite:6]{index=6}

---

# 8. Differential Flatness

The multicopter system is differentially flat.

A nonlinear system can be represented generally as

$$
\dot{x}(t)=f(x(t),u(t)).
$$

A system is differentially flat if a flat output exists from which the system states and inputs can be reconstructed using the output and a finite number of its derivatives.

The paper defines the multicopter flat output as

$$
z =
\begin{bmatrix}
x & y & z & q_3
\end{bmatrix}^{T}.
$$

Thus, the flat output contains:

- $x$ position
- $y$ position
- $z$ position
- $q_3$ quaternion component

The flat-output formulation allows the system states and control inputs to be expressed algebraically using the flat output and its derivatives. :contentReference[oaicite:7]{index=7}

---

# 9. Flatness-Based Thrust

From the translational dynamics, the required thrust magnitude is

$$
T
=
m
\sqrt{
\ddot{x}^{2}
+
\ddot{y}^{2}
+
(\ddot{z}+g)^{2}
}.
$$

This equation determines the thrust required to generate the desired translational acceleration.

This is Equation (17) in the paper and is derived again in Appendix A. :contentReference[oaicite:8]{index=8}

---

# 10. Flatness-Based Quaternion Representation

The flatness formulation expresses the quaternion components as functions of the desired translational acceleration and the selected $q_3$ reference:

$$
q_{0r}
=
\Gamma_{q_0}
(\ddot{\xi}^{*},q_{3r})
$$

$$
q_{1r}
=
\Gamma_{q_1}
(\ddot{\xi}^{*},q_{3r})
$$

$$
q_{2r}
=
\Gamma_{q_2}
(\ddot{\xi}^{*},q_{3r}).
$$

The complete reference quaternion is

$$
q_r =
\begin{bmatrix}
q_{0r} & q_{1r} & q_{2r} & q_{3r}
\end{bmatrix}^{T}.
$$

The paper allows $q_{3r}$ to be specified by the user, providing flexibility for yaw motion. :contentReference[oaicite:9]{index=9}

---

# 11. Explicit Flatness Relations

The translational dynamics lead to the following relations:

$$
\frac{m\ddot{x}}{T}
=
2(q_0q_2+q_1q_3)
$$

$$
\frac{m\ddot{y}}{T}
=
2(q_2q_3-q_0q_1)
$$

and

$$
\frac{m(\ddot{z}+g)}{T}
=
q_0^2-q_1^2-q_2^2+q_3^2.
$$

Together with

$$
q_0^2+q_1^2+q_2^2+q_3^2=1.
$$

These relations are given in Appendix A of the paper. :contentReference[oaicite:10]{index=10}

---

# 12. Reference Thrust

Squaring and summing the translational equations gives

$$
T
=
m
\sqrt{
\ddot{x}^{2}
+
\ddot{y}^{2}
+
(\ddot{z}+g)^{2}
}.
$$

This relationship is obtained directly from the unit-quaternion constraint and the translational dynamics. :contentReference[oaicite:11]{index=11}

---

# 13. Reference Quaternion Components

For the selected $q_{3r}$, the scalar quaternion component is

$$
q_{0r}
=
\frac{1}{\sqrt{2}}
\sqrt{
1+
\frac{
(\ddot{z}+g)
}{
\sqrt{
\ddot{x}^{2}
+
\ddot{y}^{2}
+
(\ddot{z}+g)^{2}
}
}
-
2q_{3r}^{2}
}.
$$

The remaining components are obtained from the flatness relations:

$$
q_{1r}
=
\frac{
\ddot{x}q_{3r}
-
\frac{1}{\sqrt{2}}\ddot{y}
}{
\left[
\frac{\ddot{z}+g}
{\sqrt{\ddot{x}^{2}+\ddot{y}^{2}+(\ddot{z}+g)^{2}}}
-2q_{3r}^{2}+1
\right]
+
\sqrt{\ddot{x}^{2}+\ddot{y}^{2}+(\ddot{z}+g)^{2}}
}
$$

and

$$
q_{2r}
=
\frac{
\ddot{y}q_{3r}
+
\frac{1}{\sqrt{2}}\ddot{x}
}{
\left[
\frac{\ddot{z}+g}
{\sqrt{\ddot{x}^{2}+\ddot{y}^{2}+(\ddot{z}+g)^{2}}}
-2q_{3r}^{2}+1
\right]
+
\sqrt{\ddot{x}^{2}+\ddot{y}^{2}+(\ddot{z}+g)^{2}}
}.
$$

These expressions follow from Appendix A of the reference paper. :contentReference[oaicite:12]{index=12}

---

# 14. Feedback-Linearization Position Controller

The desired translational acceleration is corrected using the position tracking error.

Define

$$
\Delta \xi
=
\xi_r-\xi.
$$

The virtual acceleration command is

$$
\ddot{\xi}^{*}
=
\ddot{\xi}_r
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\Delta\dot{\xi}
+
K_{i\xi}
\int\Delta\xi\,dt.
$$

where

$$
K_{p\xi},
\quad
K_{d\xi},
\quad
K_{i\xi}
$$

are diagonal positive-definite gain matrices.

This is the position-controller law given in Equation (20). :contentReference[oaicite:13]{index=13}

---

# 15. Reference Quaternion and Thrust Generation

The corrected acceleration is supplied to the flatness representation:

$$
q_{ir}
=
\Gamma_{q_i}
(\ddot{\xi}^{*},q_{3r}),
\qquad
i\in\{0,1,2\}
$$

and

$$
T
=
\Gamma_T(\ddot{\xi}^{*}).
$$

Therefore, the position controller generates two main outputs:

1. Desired thrust $T$.
2. Desired attitude quaternion $q_r$.

This corresponds to Equation (19). :contentReference[oaicite:14]{index=14}

---

# 16. Closed-Loop Translational Dynamics

The feedback-linearization controller makes the actual acceleration follow the virtual acceleration:

$$
\ddot{\xi}
=
\ddot{\xi}^{*}.
$$

Substituting the position-control law gives the closed-loop error dynamics

$$
\ddot{\xi}
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\Delta\dot{\xi}
+
K_{i\xi}
\int\Delta\xi\,dt
=
0.
$$

This is the stable closed-loop translational system presented as Equation (21) in the paper. :contentReference[oaicite:15]{index=15}

---

# 17. Component-Wise Translational Relations

The paper verifies the feedback-linearization result using:

$$
\frac{2(q_{0r}q_{2r}+q_{1r}q_{3r})T}{m}
=
\ddot{x}^{*}
$$

$$
\frac{2(q_{2r}q_{3r}-q_{0r}q_{1r})T}{m}
=
\ddot{y}^{*}
$$

and

$$
\frac{
(q_{0r}^{2}-q_{1r}^{2}-q_{2r}^{2}+q_{3r}^{2})T
}{m}
-g
=
\ddot{z}^{*}.
$$

The quaternion constraint is

$$
q_{0r}^{2}
+
q_{1r}^{2}
+
q_{2r}^{2}
+
q_{3r}^{2}
=
1.
$$

These are Equations (22a)–(22d). :contentReference[oaicite:16]{index=16}

---

# 18. Computed-Torque Attitude Controller

The rotational controller uses computed-torque control.

The torque command is

$$
\tau
=
JQ(q)\tilde{\tau}
+
JD_q
\left[
Q(q)\dot{q}
\right]\dot{q}
+
\left[
Q(q)\dot{q}
\right]
\times
\left[
JQ(q)\dot{q}
\right].
$$

Here:

- $J$ is the inertia tensor.
- $Q(q)$ is the quaternion transformation matrix.
- $D_q[\cdot]$ is the Jacobian with respect to the reduced quaternion.
- $\tilde{\tau}$ is the corrective attitude-control term.

This is Equation (26) of the paper. :contentReference[oaicite:17]{index=17}

---

# 19. Attitude Tracking Error

The quaternion tracking error is

$$
\Delta q
=
q_r-q.
$$

Its derivative is

$$
\Delta\dot{q}
=
\dot{q}_r-\dot{q}.
$$

The integral error is

$$
\int\Delta q\,dt.
$$

---

# 20. Corrective Attitude Control Law

The corrective term is

$$
\tilde{\tau}
=
\ddot{q}_r
+
K_{pq}\Delta q
+
K_{dq}\Delta\dot{q}
+
K_{iq}
\int\Delta q\,dt.
$$

where

$$
K_{pq},
\quad
K_{dq},
\quad
K_{iq}
$$

are diagonal positive-definite matrices.

This is Equation (27) of the reference paper. :contentReference[oaicite:18]{index=18}

---

# 21. Reduced Quaternion Transformation Matrix

The relationship

$$
\omega=Q(q)\dot{q}
$$

uses the matrix

$$
Q(q)
=
\frac{2}{q_0}
\begin{bmatrix}
q_0^2+q_1^2
&
q_1q_2+q_0q_3
&
q_1q_3-q_0q_2
\\
q_1q_2-q_0q_3
&
q_0^2+q_2^2
&
q_2q_3+q_0q_1
\\
q_1q_3+q_0q_2
&
q_2q_3-q_0q_1
&
q_0^2+q_3^2
\end{bmatrix}.
$$

The implementation uses this matrix to transform between reduced quaternion rates and angular velocity.

---

# 22. Complete Nonlinear Torque Representation

Before introducing the computed-torque controller, the nonlinear rotational input can also be written as

$$
\tau
=
J
\left[
Q(q)\ddot{q}
+
D_q
\left[
Q(q)\dot{q}
\right]\dot{q}
\right]
+
\left[
Q(q)\dot{q}
\right]
\times
\left[
JQ(q)\dot{q}
\right].
$$

This is the flatness-based torque representation given as Equation (18). :contentReference[oaicite:19]{index=19}

---

# 23. Hierarchical Control Architecture

The overall control structure is:

```text
                 Desired trajectory
                        ξr
                         |
                         v
             +-----------------------+
             | Position Controller   |
             | Feedback Linearization|
             +-----------------------+
                    |          |
                    |          |
                    v          v
                 Thrust T     qr
                                |
                                v
                   +-----------------------+
                   | Attitude Controller   |
                   | Computed Torque (CTC) |
                   +-----------------------+
                                |
                                v
                             Torque τ
                                |
                                v
                   +-----------------------+
                   | Multicopter Dynamics |
                   +-----------------------+
                       |             |
                       |             |
                       v             v
                    Position      Attitude
                       ξ             q
                       |             |
                       +-------------+
                              |
                              v
                         Feedback
