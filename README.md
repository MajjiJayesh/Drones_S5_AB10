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

The translational dynamics are obtained directly from Newton's second law:

$$
m\ddot{\xi}=mg+Re_zT,
$$

where

$$
g=
\begin{bmatrix}
0&0&-g
\end{bmatrix}^{T},
$$

and

$$
e_z=
\begin{bmatrix}
0&0&1
\end{bmatrix}^{T}.
$$

Here, $m$ is the multicopter mass, $T$ is the thrust magnitude, and $R$ is the quaternion-derived rotation matrix. :contentReference[oaicite:5]{index=5}

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
2(q_0q_2+q_1q_3)
\\
2(q_2q_3-q_0q_1)
\\
q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}.
$$

This equation is the fundamental nonlinear translational model used by the flatness-based position controller. :contentReference[oaicite:6]{index=6}

---

## 6. Nonlinear Rotational Dynamics

The rotational subsystem is modeled as a three-dimensional rigid body:

$$
J\dot{\omega}+\omega\times(J\omega)=\tau,
$$

where

$$
J=
\operatorname{diag}(J_x,J_y,J_z)
$$

is the inertia tensor and

$$
\tau=
\begin{bmatrix}
\tau_x&\tau_y&\tau_z
\end{bmatrix}^{T}
$$

is the applied torque. :contentReference[oaicite:7]{index=7}

The explicit angular-velocity dynamics are

$$
\dot{\omega}_x
=
\frac{J_y-J_z}{J_x}\omega_y\omega_z
+
\frac{\tau_x}{J_x},
$$

$$
\dot{\omega}_y
=
\frac{J_z-J_x}{J_y}\omega_z\omega_x
+
\frac{\tau_y}{J_y},
$$

$$
\dot{\omega}_z
=
\frac{J_x-J_y}{J_z}\omega_x\omega_y
+
\frac{\tau_z}{J_z}.
$$

These nonlinear rotational equations are later compensated by the computed-torque controller. :contentReference[oaicite:8]{index=8}

---

## 7. Differential Flatness

The nonlinear multicopter system can generally be written as

$$
\dot{x}(t)=f(x(t),u(t)).
$$

A system is differentially flat if there exists a flat output $z(t)$ from which the system states and inputs can be algebraically reconstructed using the flat output and a finite number of its derivatives:

$$
z(t)=\Upsilon
\left(
x(t),u(t),\dot{u}(t),\ldots,u^{(s)}(t)
\right),
$$

$$
x(t)=
\Upsilon_x
\left(
z(t),\dot{z}(t),\ldots,z^{(r)}(t)
\right),
$$

$$
u(t)=
\Upsilon_u
\left(
z(t),\dot{z}(t),\ldots,z^{(r+1)}(t)
\right).
$$

For the multicopter used in this project, the flat output is

$$
z=
\begin{bmatrix}
x&y&z&q_3
\end{bmatrix}^{T}.
$$

Therefore, the three-dimensional position and the fourth quaternion component are sufficient to construct the remaining required states and control inputs. :contentReference[oaicite:9]{index=9}

---

## 8. Flatness-Based Thrust Calculation

Starting from the translational dynamics and the quaternion unit-norm constraint, the thrust can be obtained directly from the desired translational acceleration.

The required thrust is

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

This equation is one of the most important equations in the implementation because it converts the desired translational acceleration into the required total thrust. :contentReference[oaicite:10]{index=10}

The derivation begins from

$$
\frac{m\ddot{x}}{T}
=
2(q_0q_2+q_1q_3),
$$

$$
\frac{m\ddot{y}}{T}
=
2(q_2q_3-q_0q_1),
$$

and

$$
\frac{m(\ddot{z}+g)}{T}
=
q_0^2-q_1^2-q_2^2+q_3^2.
$$

Together with

$$
q_0^2+q_1^2+q_2^2+q_3^2=1,
$$

squaring and adding the three acceleration equations produces the thrust relationship above. :contentReference[oaicite:11]{index=11}

---

## 9. Flatness-Based Quaternion Reconstruction

After calculating the required thrust, the quaternion components required to generate the desired translational acceleration are reconstructed.

Define

$$
D=
\sqrt{
\ddot{x}^{2}
+
\ddot{y}^{2}
+
(\ddot{z}+g)^{2}
}.
$$

The scalar quaternion component is

$$
q_0
=
\frac{1}{\sqrt{2}}
\sqrt{
\frac{\ddot{z}+g}{D}
-2q_3^2+1
}.
$$

The remaining quaternion components are calculated as

$$
q_1=
\frac{
\ddot{x}q_3
-
\frac{1}{\sqrt{2}}\ddot{y}
\sqrt{
\frac{\ddot{z}+g}{D}
-2q_3^2+1
}
}{
\ddot{z}+g+D
},
$$

and

$$
q_2=
\frac{
\ddot{y}q_3
+
\frac{1}{\sqrt{2}}\ddot{x}
\sqrt{
\frac{\ddot{z}+g}{D}
-2q_3^2+1
}
}{
\ddot{z}+g+D
}.
$$

Thus, the quaternion components can be represented compactly as

$$
q_i=\Gamma_{q_i}(\ddot{\xi},q_3),
\qquad
i\in\{0,1,2\},
$$

while

$$
T=\Gamma_T(\ddot{\xi}).
$$

The important point is that the reference quaternion is not independently selected from roll, pitch, and yaw. It is generated from the required translational acceleration together with the selected reference value of $q_3$. :contentReference[oaicite:12]{index=12}

---

## 10. Flatness-Based Torque Representation

The relationship

$$
\omega=Q(q)\dot q
$$

is substituted into the nonlinear rotational dynamics.

This produces the torque representation

$$
\tau
=
J
\left[
Q(q)\ddot q
+
D_q[Q(q)\dot q]\dot q
\right]
+
[Q(q)\dot q]
\times
\left[
JQ(q)\dot q
\right].
$$

Here, $D_q[\cdot]$ represents the Jacobian with respect to the quaternion vector $q$.

This equation provides the nonlinear torque representation required for the computed-torque controller. :contentReference[oaicite:13]{index=13}

---

# 11. Two-Layer Hierarchical Control

The control architecture is divided into two layers.

### High-Level Position Controller

The high-level controller receives the desired position $\xi_r$ and the actual position $\xi$. It calculates:

- corrected desired acceleration $\ddot{\xi}^{*}$,
- reference quaternion $q_r$,
- required thrust $T$.

The reference quaternion is

$$
q_r=
\begin{bmatrix}
q_{0r}&q_{1r}&q_{2r}&q_{3r}
\end{bmatrix}^{T}.
$$

Only the three vector components

$$
q_r=
\begin{bmatrix}
q_{1r}&q_{2r}&q_{3r}
\end{bmatrix}^{T}
$$

are passed to the low-level attitude controller.

### Low-Level Attitude Controller

The low-level controller receives the reference quaternion components and tracks them using computed-torque control.

The two-layer structure allows the translational and rotational dynamics to be handled separately while maintaining the complete nonlinear model. :contentReference[oaicite:14]{index=14}

---

# 12. Feedback-Linearization Position Controller

The position tracking error is defined as

$$
\Delta\xi=\xi_r-\xi.
$$

The corrective desired acceleration is

$$
\ddot{\xi}^{*}
=
\ddot{\xi}_r
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\dot{\Delta\xi}
+
K_{i\xi}\int\Delta\xi\,dt.
$$

where

$$
K_{p\xi},\quad
K_{d\xi},\quad
K_{i\xi}
$$

are diagonal positive-definite gain matrices.

The reference quaternion is then obtained from

$$
q_{ir}
=
\Gamma_{q_i}
\left(
\ddot{\xi}^{*},q_{3r}
\right),
\qquad
i\in\{0,1,2\},
$$

and the required thrust is

$$
T=
\Gamma_T(\ddot{\xi}^{*}).
$$

Therefore, the position controller does not directly command roll, pitch, and yaw. Instead, it first determines the acceleration required to correct the trajectory and then uses the flatness equations to determine the corresponding thrust and quaternion. :contentReference[oaicite:15]{index=15}

---

# 13. Closed-Loop Position Dynamics

Using the generated reference quaternion and thrust in the nonlinear translational dynamics gives

$$
\begin{bmatrix}
\ddot{x}\\
\ddot{y}\\
\ddot{z}
\end{bmatrix}
=
\begin{bmatrix}
\ddot{x}^{*}\\
\ddot{y}^{*}\\
\ddot{z}^{*}
\end{bmatrix}.
$$

Consequently, the position tracking error satisfies

$$
\ddot{\Delta\xi}
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\dot{\Delta\xi}
+
K_{i\xi}
\int\Delta\xi\,dt
=
0.
$$

Thus, the nonlinear translational dynamics are transformed into a linear closed-loop error system. :contentReference[oaicite:16]{index=16}

The corresponding translational state is

$$
x=
\begin{bmatrix}
\xi\\
\dot{\xi}
\end{bmatrix},
$$

and the state-space representation is

$$
\dot{x}=A_xx+B_x\ddot{\xi}^{*},
$$

with

$$
A_x=
\begin{bmatrix}
0_{3\times3}&I_{3\times3}\\
0_{3\times3}&0_{3\times3}
\end{bmatrix},
$$

and

$$
B_x=
\begin{bmatrix}
0_{3\times3}\\
I_{3\times3}
\end{bmatrix}.
$$

:contentReference[oaicite:17]{index=17}

---

# 14. Computed-Torque Attitude Controller

The low-level attitude controller uses **Computed Torque Control (CTC)**.

The torque command is

$$
\tau
=
JQ(q)\tau^{*}
+
JD_q[Q(q)\dot q]\dot q
+
[Q(q)\dot q]
\times
[JQ(q)\dot q].
$$

The corrective virtual control is

$$
\tau^{*}
=
\ddot q_r
+
K_{pq}\Delta q
+
K_{dq}\dot{\Delta q}
+
K_{iq}\int\Delta q\,dt,
$$

where the quaternion tracking error is

$$
\Delta q=q_r-q.
$$

The gain matrices

$$
K_{pq},\quad K_{dq},\quad K_{iq}
$$

are diagonal positive-definite matrices.

Therefore, the computed-torque controller compensates for the nonlinear terms in the rotational dynamics while the corrective term drives the quaternion tracking error toward zero. :contentReference[oaicite:18]{index=18}

---

# 15. Quaternion Tracking and Unit-Norm Property

The attitude controller directly tracks only

$$
q_1,\quad q_2,\quad q_3.
$$

The scalar component is reconstructed using

$$
q_0=
\sqrt{
1-q_1^2-q_2^2-q_3^2
}.
$$

Similarly, the reference scalar component is

$$
q_{0r}
=
\sqrt{
1-q_{1r}^2-q_{2r}^2-q_{3r}^2
}.
$$

When

$$
q_1\rightarrow q_{1r},
\qquad
q_2\rightarrow q_{2r},
\qquad
q_3\rightarrow q_{3r},
$$

the unit-norm constraint ensures that

$$
q_0\rightarrow q_{0r}.
$$

Therefore, controlling the three vector components is sufficient to guarantee convergence of the complete quaternion under the formulation used in the paper. :contentReference[oaicite:19]{index=19}

The controller gains are selected such that the attitude error converges faster than the position error:

$$
\Delta q\rightarrow0
\quad
\text{faster than}
\quad
\Delta\xi\rightarrow0.
$$

This is required for effective hierarchical trajectory tracking. :contentReference[oaicite:20]{index=20}

---

# 16. Quaternion-to-Euler Conversion

For applications where roll, pitch, and yaw are required, the quaternion is converted using

$$
\phi=
\operatorname{atan2}
\left(
2(q_0q_1+q_2q_3),
1-2(q_1^2+q_2^2)
\right),
$$

$$
\theta=
\arcsin
\left(
2(q_0q_2-q_1q_3)
\right),
$$

$$
\psi=
\operatorname{atan2}
\left(
2(q_0q_3+q_1q_2),
1-2(q_2^2+q_3^2)
\right).
$$

Here,

$$
\phi=\text{roll},
\qquad
\theta=\text{pitch},
\qquad
\psi=\text{yaw}.
$$

The quaternion remains the fundamental attitude representation used by the control methodology. The Euler-angle conversion is used when required for interpretation or interfacing with systems that accept roll, pitch, and yaw. :contentReference[oaicite:21]{index=21}

---

# 17. Drag Model Used in Simulation

For a more realistic simulation, the paper adds aerodynamic drag to the translational dynamics:

$$
m\ddot{\xi}
=
mg+Re_zT+F_D.
$$

The drag force is modeled as

$$
F_D
=
-\frac{1}{2}
\rho C_D A
|\dot{\xi}|
\dot{\xi},
$$

where:

- $C_D$ is the drag coefficient,
- $\rho$ is the fluid density,
- $A$ is the multicopter cross-sectional area,
- $\dot{\xi}$ is the translational velocity.

Importantly, this drag force is included in the simulation model but is not included in the controller design itself. :contentReference[oaicite:22]{index=22}

---

# 18. Complete Implemented Control Sequence

The complete methodology implemented from the paper can therefore be summarized as follows:

### Step 1 — Reference trajectory

The desired trajectory provides

$$
\xi_r(t),
\qquad
\dot{\xi}_r(t),
\qquad
\ddot{\xi}_r(t).
$$

### Step 2 — Position error

The tracking error is calculated as

$$
\Delta\xi=\xi_r-\xi.
$$

### Step 3 — Corrective acceleration

The feedback-linearization controller calculates

$$
\ddot{\xi}^{*}
=
\ddot{\xi}_r
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\dot{\Delta\xi}
+
K_{i\xi}\int\Delta\xi\,dt.
$$

### Step 4 — Flatness transformation

The corrected acceleration is converted into

$$
T=\Gamma_T(\ddot{\xi}^{*})
$$

and

$$
q_{ir}
=
\Gamma_{q_i}(\ddot{\xi}^{*},q_{3r}).
$$

### Step 5 — Reference quaternion

The complete reference quaternion is

$$
q_r=
\begin{bmatrix}
q_{0r}&q_{1r}&q_{2r}&q_{3r}
\end{bmatrix}^{T}.
$$

### Step 6 — Quaternion tracking error

The low-level controller calculates

$$
\Delta q=q_r-q.
$$

### Step 7 — Computed-torque control

The corrective virtual control is

$$
\tau^{*}
=
\ddot q_r
+
K_{pq}\Delta q
+
K_{dq}\dot{\Delta q}
+
K_{iq}\int\Delta q\,dt.
$$

The actual torque command is

$$
\tau
=
JQ(q)\tau^{*}
+
JD_q[Q(q)\dot q]\dot q
+
[Q(q)\dot q]
\times
[JQ(q)\dot q].
$$

### Step 8 — Multicopter dynamics

The torque is applied to

$$
J\dot{\omega}
+
\omega\times(J\omega)
=
\tau.
$$

The translational dynamics are simultaneously governed by

$$
m\ddot{\xi}=mg+Re_zT.
$$

### Step 9 — Quaternion propagation

The updated angular velocity is used in

$$
\dot q=
\frac{1}{2}q\otimes\bar{\omega}
$$

to update the multicopter attitude.

### Step 10 — Feedback

The updated position, quaternion, and angular velocity are fed back into the controllers, closing the hierarchical control loop.

---

# 19. Control Parameters Used in the Paper Implementation

The simulation uses the following position-controller gains:

$$
K_{p\xi}
=
\operatorname{diag}(60,60,60),
$$

$$
K_{d\xi}
=
\operatorname{diag}(6,6,6),
$$

$$
K_{i\xi}
=
\operatorname{diag}(0.1,0.1,0.1).
$$

The attitude-controller gains are

$$
K_{pq}
=
\operatorname{diag}(2000,2000,2000),
$$

$$
K_{dq}
=
\operatorname{diag}(10,10,10),
$$

$$
K_{iq}
=
\operatorname{diag}(10,10,5).
$$

These are the gains reported for the simulation in the base paper. :contentReference[oaicite:23]{index=23}

---

# 20. Simulation Model Parameters

The multicopter simulation model uses

$$
m=0.025\;\mathrm{kg},
$$

$$
J_x=J_y=4.856\times10^{-3}\;\mathrm{kg\,m^2},
$$

$$
J_z=8.801\times10^{-3}\;\mathrm{kg\,m^2},
$$

$$
C_D=0.8,
$$

$$
\rho=1.225\;\mathrm{kg/m^3},
$$

and

$$
A=0.01425\;\mathrm{m^2}.
$$

The simulation uses the `ode4` solver with sampling time

$$
T_s=0.01\;\mathrm{s}.
$$

The reference value of the flat-output quaternion component used in the reported simulation is

$$
q_{3r}=0.
$$

:contentReference[oaicite:24]{index=24}

---

# 21. Most Important Mathematical Formulation

The equations that directly contribute to the implemented control methodology are:

### Unit quaternion

$$
q_0^2+q_1^2+q_2^2+q_3^2=1.
$$

### Quaternion kinematics

$$
\dot q=\frac{1}{2}q\otimes\bar{\omega}.
$$

### Quaternion-angular velocity relationship

$$
\omega=Q(q)\dot q.
$$

### Rotation matrix

$$
R=
\begin{bmatrix}
1-2(q_2^2+q_3^2)&2(q_1q_2-q_0q_3)&2(q_0q_2+q_1q_3)\\
2(q_1q_2+q_0q_3)&1-2(q_1^2+q_3^2)&2(q_2q_3-q_0q_1)\\
2(q_1q_3-q_0q_2)&2(q_0q_1+q_2q_3)&1-2(q_1^2+q_2^2)
\end{bmatrix}.
$$

### Translational dynamics

$$
m\ddot{\xi}=mg+Re_zT.
$$

### Rotational dynamics

$$
J\dot{\omega}+\omega\times(J\omega)=\tau.
$$

### Flat output

$$
z=
\begin{bmatrix}
x&y&z&q_3
\end{bmatrix}^{T}.
$$

### Required thrust

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

### Position corrective acceleration

$$
\ddot{\xi}^{*}
=
\ddot{\xi}_r
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\dot{\Delta\xi}
+
K_{i\xi}\int\Delta\xi\,dt.
$$

### Flatness-based reference quaternion

$$
q_{ir}
=
\Gamma_{q_i}
\left(
\ddot{\xi}^{*},q_{3r}
\right).
$$

### Position closed-loop dynamics

$$
\ddot{\Delta\xi}
+
K_{p\xi}\Delta\xi
+
K_{d\xi}\dot{\Delta\xi}
+
K_{i\xi}\int\Delta\xi\,dt
=
0.
$$

### Quaternion tracking error

$$
\Delta q=q_r-q.
$$

### Computed-torque virtual control

$$
\tau^{*}
=
\ddot q_r
+
K_{pq}\Delta q
+
K_{dq}\dot{\Delta q}
+
K_{iq}\int\Delta q\,dt.
$$

### Computed torque

$$
\tau
=
JQ(q)\tau^{*}
+
JD_q[Q(q)\dot q]\dot q
+
[Q(q)\dot q]\times[JQ(q)\dot q].
$$

### Quaternion scalar reconstruction

$$
q_0=
\sqrt{1-q_1^2-q_2^2-q_3^2}.
$$

### Drag model

$$
F_D
=
-\frac{1}{2}
\rho C_D A
|\dot{\xi}|
\dot{\xi}.
$$

### Simulation dynamics with drag

$$
m\ddot{\xi}
=
mg+Re_zT+F_D.
$$

These equations form the mathematical core of the implemented differential-flatness, feedback-linearization, and computed-torque trajectory-tracking methodology. :contentReference[oaicite:25]{index=25} :contentReference[oaicite:26]{index=26} :contentReference[oaicite:27]{index=27} :contentReference[oaicite:28]{index=28}
