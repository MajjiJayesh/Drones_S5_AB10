<p align="center">
  <img src="Images/Amrita Logo.jpeg" width="700">
</p>

<h2 align="center">
Trajectory Tracking for a Multicopter under a Quaternion Representation
</h2>


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

This project implements the multicopter trajectory-tracking control methodology presented in the base paper using a quaternion-based nonlinear dynamic model. The approach combines differential flatness, feedback linearization, and computed-torque control to achieve accurate position and attitude tracking. The multicopter attitude is represented using unit quaternions, avoiding the singularities associated with Euler-angle representations. The high-level position controller uses position, velocity, and integral tracking errors to generate the required translational acceleration. Differential-flatness relations then convert this acceleration into the required thrust and reference quaternion. A low-level computed-torque controller tracks the reference quaternion while compensating for the nonlinear rotational dynamics. The complete methodology is implemented in simulation to evaluate trajectory tracking and the controller's response to disturbances.

# Introduction

Multicopters are nonlinear, underactuated aerial vehicles whose position and attitude are strongly coupled. Accurate trajectory tracking therefore requires a control strategy capable of handling both the nonlinear translational and rotational dynamics of the vehicle. Conventional Euler-angle representations can also introduce singularities and complicate the mathematical formulation of three-dimensional attitude control.

This project implements the multicopter trajectory-tracking methodology presented in the base paper using a quaternion-based dynamic model. Quaternions are used to represent the vehicle attitude because they provide a compact representation of three-dimensional orientation without the singularities associated with Euler angles.

The implemented control architecture uses differential flatness to establish a relationship between the desired trajectory and the required thrust and attitude. A high-level feedback-linearization controller generates the required translational acceleration from the position tracking error. This acceleration is then transformed into the required thrust and reference quaternion using the differential-flatness formulation.

A low-level computed-torque controller is used to track the reference quaternion while compensating for the nonlinear rotational dynamics of the multicopter. The complete system combines the nonlinear dynamic model, quaternion kinematics, differential flatness, feedback linearization, and computed-torque attitude control into a hierarchical trajectory-tracking framework. The implementation is evaluated through simulation to study position and attitude tracking performance.

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
## 2. Methodology Flow Diagram

The complete control architecture implemented in this project is shown below.

```mermaid
flowchart TD
    A[Desired Trajectory] --> B[Position Controller]
    B --> C[Differential Flatness]
    C --> D[Reference Quaternion]
    C --> E[Required Thrust]
    D --> F[Computed Torque Attitude Controller]
    E --> G[Multicopter Dynamics]
    F --> G
    G --> H[Feedback]
    H --> B
    H --> F
```

The architecture consists of a high-level position controller and a low-level attitude controller. The position controller generates the required acceleration, which is converted through differential flatness into the reference quaternion and thrust. The computed-torque attitude controller generates the required control torque, and both thrust and torque are applied to the multicopter dynamics. The resulting states are fed back to the controllers.
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

The low-level attitude controller is operated at a higher frequency than the high-level position controller so that the attitude subsystem can track the generated reference quaternion sufficiently fast. 

---

---

## 3. Basis Spline (B-Spline) Trajectory Generation

To provide smooth, physically feasible reference trajectories for the differential-flatness controller, Basis Splines (B-splines) are utilized. Differential-flatness-based control requires at least $C^2$ continuity (continuous position, velocity, and acceleration) so that the computed thrust and attitude references do not experience sudden discontinuous jumps.

A parametric B-spline curve of degree $p$ defined by $n+1$ control points $\mathbf{P}_0, \mathbf{P}_1, \dots, \mathbf{P}_n \in \mathbb{R}^3$ is given by:

$$
C(u) = \sum_{i=0}^{n} N_{i,p}(u)\mathbf{P}_i, \quad u \in [0, 1]
$$

where $N_{i,p}(u)$ are the B-spline basis functions and $u$ is the normalized spline parameter.

---

### 3.1 Clamped Knot Vector

The parameter domain is partitioned by a non-decreasing knot vector:

$$
X_i = [ u_0, u_1, \dots, u_m ], \quad m = n + p + 1
$$

To strictly guarantee that the trajectory starts exactly at the initial waypoint and terminates exactly at the target waypoint, a **clamped (open) knot vector** is constructed with an endpoint multiplicity of (p + 1):

$$
X_i = [ \underbrace{0, 0, \dots, 0}_{p+1 \text{ times}}, u_{p+1}, \dots, u_{m-p-1}, \underbrace{1, 1, \dots, 1}_{p+1 \text{ times}} ]
$$

The internal knots are uniformly spaced:

$$
u_{p+1+j} = \frac{j+1}{n - p + 1}, \quad j = 0, 1, \dots, n-p-1
$$

This property guarantees the boundary conditions:

$$
C(0) = \mathbf{P}_0 = \mathbf{P}_{\text{start}}, \qquad C(1) = \mathbf{P}_n = \mathbf{P}_{\text{goal}}
$$

---
### 3.2 Cox–de Boor Recursive Formulation

The basis functions $N_{i,p}(u)$ are evaluated recursively using the Cox–de Boor recurrence relation:

$$
N_{i,0}(u) = 
\begin{cases}
1, & \text{if } u_i \leq u < u_{i+1} \\
0, & \text{otherwise}
\end{cases}
$$

For degree $p \geq 1$:

$$
N_{i,p}(u) = \frac{u - u_i}{u_{i+p} - u_i} N_{i,p-1}(u) + \frac{u_{i+p+1} - u}{u_{i+p+1} - u_{i+1}} N_{i+1,p-1}(u)
$$

with the convention that terms with zero denominators evaluate to zero ($0/0 = 0$).

---

### 3.3 Analytical Trajectory Derivatives

Differential flatness requires exact reference velocity and acceleration. The first derivative of a B-spline curve of degree $p$ is analytically represented as another B-spline curve of degree $p-1$:

$$
C'(u) = \frac{dC(u)}{du} = \sum_{i=0}^{n-1} N_{i+1, p-1}(u) \mathbf{Q}_i
$$

where the derivative control points $\mathbf{Q}_i \in \mathbb{R}^3$ are computed directly from the original control points:

$$
\mathbf{Q}_i = \frac{p}{u_{i+p+1} - u_{i+1}} (\mathbf{P}_{i+1} - \mathbf{P}_i), \quad i = 0, \dots, n-1
$$

Similarly, the second derivative $C''(u) = \frac{d^2C(u)}{du^2}$ is obtained by differentiating $C'(u)$ with respect to $u$:

$$
C''(u) = \sum_{i=0}^{n-2} N_{i+2, p-2}(u) \mathbf{R}_i
$$

where:

$$
\mathbf{R}_i = \frac{p-1}{u_{i+p+1} - u_{i+2}} (\mathbf{Q}_{i+1} - \mathbf{Q}_i), \quad i = 0, \dots, n-2
$$

---

### 3.4 Smooth $C^2$ Quintic Time Parameterization

To map real flight time $t \in [0, T]$ (where $T$ is the total flight duration) to the spline parameter $u \in [0, 1]$ while enforcing zero velocity and zero acceleration at takeoff ($t=0$) and touchdown ($t=T$), a **quintic smoothstep function** is employed:

Let normalized time be:

$$
s(t) = \frac{t}{T} \in [0, 1]
$$

The parameter scaling $u(s)$ and its time derivatives are:

$$
u(s) = 10s^3 - 15s^4 + 6s^5
$$

$$
\dot{u}(s) = \frac{du}{dt} = \frac{1}{T} \left( 30s^2 - 60s^3 + 30s^4 \right)
$$

$$
\ddot{u}(s) = \frac{d^2u}{dt^2} = \frac{1}{T^2} \left( 60s - 180s^2 + 120s^3 \right)
$$

Notice that at boundaries:
- At $t = 0$: $u(0) = 0$, $\dot{u}(0) = 0$, $\ddot{u}(0) = 0$.
- At $t = T$: $u(1) = 1$, $\dot{u}(1) = 0$, $\ddot{u}(1) = 0$.

Applying the chain rule, the continuous reference position $\xi_r(t)$, reference velocity $\dot{\xi}_r(t)$, and reference acceleration $\ddot{\xi}_r(t)$ fed into the high-level position controller are:

$$
\xi_r(t) = C(u(t))
$$

$$
\dot{\xi}_r(t) = C'(u(t)) \dot{u}(t)
$$

$$
\ddot{\xi}_r(t) = C''(u(t)) \dot{u}^2(t) + C'(u(t)) \ddot{u}(t)
$$

This guarantees that the multicopter smoothly departs from a hover state ($\dot{\xi}_r(0) = \mathbf{0}, \ddot{\xi}_r(0) = \mathbf{0}$) and comes to a complete rest at the goal ($\dot{\xi}_r(T) = \mathbf{0}, \ddot{\xi}_r(T) = \mathbf{0}$) without velocity or acceleration discontinuities.

## 4. Multicopter State Representation

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

This reduced representation is important because the control formulation directly uses the three components $q_1,q_2,q_3$, while $q_0$ is obtained from the unit-norm constraint.

---

## 5. Quaternion Rotation Matrix

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

The rotation matrix determines the direction of the multicopter thrust in the global coordinate frame and therefore directly connects the attitude to the translational motion. 

---

## 6. Quaternion Kinematics

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
\omega_x=2(-q_1\dot{q}_0+q_0\dot{q}_1-q_3\dot{q}_2+q_2\dot{q}_3)
$$

$$
\omega_y=2(-q_2\dot{q}_0+q_3\dot{q}_1+q_0\dot{q}_2-q_1\dot{q}_3)
$$

$$
\omega_z=2(-q_3\dot{q}_0-q_2\dot{q}_1+q_1\dot{q}_2+q_0\dot{q}_3)
$$

Using the reduced quaternion vector, this relationship can be written as

$$
\omega=Q(q)\dot{q}
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

These equations provide the connection between quaternion motion and rotational dynamics required by the computed-torque controller.

---

## 7. Nonlinear Translational Dynamics

The translational motion of the multicopter is described by Newton's second law:

$$
m\ddot{\boldsymbol{\xi}}=-mg\mathbf e_z+TR(q)\mathbf e_z
$$

where $m$ is the multicopter mass, $g$ is the gravitational acceleration, $R(q)$ transforms the body thrust direction into the world frame, $e_z$ is the unit vector along the vertical axis, and $T$ is the total thrust.

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

Expanding the translational dynamics gives:

$$
m\ddot{x}=2T(q_0q_2+q_1q_3)
$$

$$
m\ddot{y}=2T(q_2q_3-q_0q_1)
$$

$$
m\ddot{z}=-mg+T(q_0^2-q_1^2-q_2^2+q_3^2)
$$

The translational dynamics are:

$$
m\ddot{\boldsymbol{\xi}}=-mg\mathbf e_z+TR(q)\mathbf e_z
$$

## 8. Nonlinear Rotational Dynamics

The rotational motion of the multicopter is modeled using the rigid-body rotational dynamics:

$$
J\dot{\omega}+\omega\times(J\omega)=\tau
$$

where $J$ is the inertia matrix, $\omega$ is the body angular velocity, and $\tau$ is the control torque.

The angular velocity is defined as

$$
\omega=
\begin{bmatrix}
\omega_x & \omega_y & \omega_z
\end{bmatrix}^{T}
$$

and the control torque is

$$
\tau=
\begin{bmatrix}
\tau_x & \tau_y & \tau_z
\end{bmatrix}^{T}.
$$

For the multicopter model, the inertia matrix is

$$
J=
\begin{bmatrix}
J_x & 0 & 0\\
0 & J_y & 0\\
0 & 0 & J_z
\end{bmatrix}
$$

The rotational dynamics can be expanded into three equations:

$$
\dot{\omega}_x=
\frac{J_y-J_z}{J_x}\omega_y\omega_z+
\frac{\tau_x}{J_x}
$$

$$
\dot{\omega}_y=
\frac{J_z-J_x}{J_y}\omega_z\omega_x+
\frac{\tau_y}{J_y}
$$

$$
\dot{\omega}_z=
\frac{J_x-J_y}{J_z}\omega_x\omega_y+
\frac{\tau_z}{J_z}
$$

These equations describe the nonlinear rotational behavior of the multicopter. The angular-velocity coupling terms are compensated by the computed-torque controller in the attitude-control layer.

## 9. Differential Flatness

The translational dynamics of the multicopter are given by

```math
m\ddot{\boldsymbol{\xi}}=m\mathbf{g}+Re_zT
```

The flat output is selected as

```math
\mathbf{y}_f=
\begin{bmatrix}
x & y & z & q_3
\end{bmatrix}^{T}
```

Using the quaternion representation of the rotation matrix, the translational dynamics can be expressed as

**(A.1a)**

```math
\frac{m\ddot{x}}{T}=2(q_0q_2+q_1q_3)
```

**(A.1b)**

```math
\frac{m\ddot{y}}{T}=2(q_2q_3-q_0q_1)
```

**(A.1c)**

```math
\frac{m(\ddot{z}+g)}{T}=q_0^2-q_1^2-q_2^2+q_3^2
```

Together with the unit-quaternion constraint,

**(A.1d)**

```math
q_0^2+q_1^2+q_2^2+q_3^2=1
```

These equations are used to obtain the thrust and the remaining quaternion components from the flat output and its derivatives.
### 9.1 Derivation of the Required Thrust

Squaring Eqs. (A.1a)--(A.1c) and adding them gives

```math
\frac{m^2}{T^2}
\left[
\ddot{x}^2+\ddot{y}^2+(\ddot{z}+g)^2
\right]
=
\left(q_0^2+q_1^2+q_2^2+q_3^2\right)^2
```

Using the unit-quaternion constraint,

```math
q_0^2+q_1^2+q_2^2+q_3^2=1
```

we obtain

```math
\frac{m^2}{T^2}
\left[
\ddot{x}^2+\ddot{y}^2+(\ddot{z}+g)^2
\right]
=1
```

Therefore,

```math
T^2
=
m^2
\left[
\ddot{x}^2+\ddot{y}^2+(\ddot{z}+g)^2
\right]
```
Since the thrust is positive,

$$
T=m\sqrt{\ddot{x}^{\,2}+\ddot{y}^{\,2}+(\ddot{z}+g)^2} $$



For convenience, define

```math
D=
\sqrt{
\ddot{x}^2+\ddot{y}^2+(\ddot{z}+g)^2
}
```

Thus,

```math
T=mD
```

### 9.2 Derivation of $q_0$

Adding Eqs. (A.1c) and (A.1d) gives

```math
2(q_0^2+q_3^2)
=
1+\frac{m(\ddot{z}+g)}{T}
```

Using $T=mD$,

```math
q_0^2+q_3^2
=
\frac{1}{2}
\left(
1+\frac{\ddot{z}+g}{D}
\right)
```

Hence,

```math
q_0^2
=
\frac{1}{2}
\left(
1+\frac{\ddot{z}+g}{D}
\right)
-q_3^2
```

Since $q_0\geq0$,

```math
q_0=
\frac{1}{\sqrt{2}}
\sqrt{
\frac{\ddot{z}+g}{D}-2q_3^2+1
}

```

### 9.3 Derivation of $q_1$ and $q_2$

From Eqs. (A.1a) and (A.1b),

```math
q_3q_1+q_0q_2
=
\frac{m\ddot{x}}{2T}
```

and

```math
-q_0q_1+q_3q_2
=
\frac{m\ddot{y}}{2T}
```

Solving these two equations for $q_1$ and $q_2$ gives

```math
q_1=
\frac{
m(q_3\ddot{x}-q_0\ddot{y})
}{
2T(q_0^2+q_3^2)
}
```

and

```math
q_2=
\frac{
m(q_0\ddot{x}+q_3\ddot{y})
}{
2T(q_0^2+q_3^2)
}
```

Thus, the thrust and quaternion components are obtained algebraically from the flat output and its derivatives.
## 10. Feedback-Linearization Position Controller

For the position controller at the high level, the flatness representations are used to design the feedback-linearization controller.

The reference quaternion components and thrust are given by

**(19a)**

```math
q_{ir}=\Gamma_{q_i}(\ddot{\xi}^{*},q_{3r}), \qquad i\in\{0,1,2\}
```

**(19b)**

```math
T=\Gamma_T(\ddot{\xi}^{*})
```

where $q_{3r}$ is the reference value of $q_3$ defined by the user, and the corrective term is

```math
\ddot{\xi}^{*}
=
\ddot{\xi}_r
+
K_{p\xi}\tilde{\xi}
+
K_{d\xi}\dot{\tilde{\xi}}
+
K_{i\xi}\int\tilde{\xi}\,dt
```

**(20)**

where

```math
\tilde{\xi}=\xi_r-\xi
```

and $K_{p\xi}$, $K_{d\xi}$, and $K_{i\xi}$ are diagonal positive-definite matrices in $\mathbb{R}^3$.

### 10.1 Closed-Loop Position Dynamics

From (19), with $\ddot{z}^{*}\geq-g$, the following relations are obtained:

**(22a)**

```math
\frac{2(q_{0r}q_{2r}+q_{1r}q_{3r})T}{m}
=
\ddot{x}^{*}
```

**(22b)**

```math
\frac{2(q_{2r}q_{3r}-q_{0r}q_{1r})T}{m}
=
\ddot{y}^{*}
```

**(22c)**

```math
\frac{(q_{0r}^{2}-q_{1r}^{2}-q_{2r}^{2}+q_{3r}^{2})T}{m}
-g
=
\ddot{z}^{*}
```

**(22d)**

```math
q_{0r}^{2}+q_{1r}^{2}+q_{2r}^{2}+q_{3r}^{2}=1
```

From (22a)--(22c), the control law (19) drives the translational dynamics to

**(23)**

```math
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
\end{bmatrix}
```

or equivalently,

**(24)**

```math
\dot{x}=A_xx+B_x\ddot{\xi}^{*}
```

where

```math
x=
\begin{bmatrix}
\xi^{T} & \dot{\xi}^{T}
\end{bmatrix}^{T}
=
\begin{bmatrix}
x & y & z & \dot{x} & \dot{y} & \dot{z}
\end{bmatrix}^{T}
```

and

```math
\ddot{\xi}^{*}
=
\begin{bmatrix}
\ddot{x}^{*} &
\ddot{y}^{*} &
\ddot{z}^{*}
\end{bmatrix}^{T}.
```

The system matrices are

**(25)**

```math
A_x=
\begin{bmatrix}
0_{3\times3} & I_{3\times3}\\
0_{3\times3} & 0_{3\times3}
\end{bmatrix},
\qquad
B_x=
\begin{bmatrix}
0_{3\times3}\\
I_{3\times3}
\end{bmatrix}.
```

Substituting the corrective acceleration from (20) gives the closed-loop error dynamics

**(21)**

```math
\ddot{\tilde{\xi}}
+
K_{p\xi}\tilde{\xi}
+
K_{d\xi}\dot{\tilde{\xi}}
+
K_{i\xi}\int\tilde{\xi}\,dt
=
0
```

Thus, the feedback-linearization control law transforms the nonlinear translational dynamics into the linear closed-loop error system given by (21).
## 11. Computed-Torque Attitude Controller

The low-level attitude controller uses the Computed Torque Control (CTC) method to compensate for the nonlinear rotational dynamics of the multicopter.

The input torque is calculated as

$$
\tau=JQ(q)\tilde{\tau}+JD_q[Q(q)\dot{q}]\dot{q}+[Q(q)\dot{q}]\times[JQ(q)\dot{q}]
$$

where $J$ is the inertia tensor, $Q(q)$ is the quaternion transformation matrix, and $D_q$ represents the Jacobian with respect to the quaternion.

The corrective term is

$$
\tilde{\tau}=\ddot{q}_r+K_{pq}\epsilon_q+K_{dq}\dot{\epsilon}_q+K_{iq}\int\epsilon_q\,dt
$$

The quaternion tracking error is

$$
\epsilon_q=q_r-q
$$

where $q_r$ is the reference quaternion generated by the position controller and $q$ is the measured quaternion.

The matrices $K_{pq}$, $K_{dq}$, and $K_{iq}$ are diagonal positive definite gain matrices.

The attitude controller tracks the last three components of the reference quaternion:

$$
q_r=
\begin{bmatrix}
q_{0r}\\
q_{1r}\\
q_{2r}\\
q_{3r}
\end{bmatrix}
$$

The controller directly tracks $q_{1r}$, $q_{2r}$, and $q_{3r}$. The first quaternion component $q_0$ converges to its reference value through the unit-quaternion constraint.

The unit-quaternion constraint is

$$
q_0^2+q_1^2+q_2^2+q_3^2=1
$$

and similarly for the reference quaternion,

$$
q_{0r}^2+q_{1r}^2+q_{2r}^2+q_{3r}^2=1
$$

Therefore, convergence of the three tracked quaternion components also ensures convergence of the remaining quaternion component.


## Results

# 12. MATLAB/Simulink Simulation Results

The implemented quaternion-based quadrotor controller was evaluated in MATLAB/Simulink to verify the complete control architecture and trajectory-tracking performance.

## 12.1 Complete Simulink Model

The complete MATLAB/Simulink model integrates the reference trajectory, position controller, differential-flatness mapping, attitude controller, and quadrotor dynamics.

<img src= Images/Simulink1.jpg alt="Complete Simulink Model" width="100%">

*Figure 1. Complete MATLAB/Simulink model for quaternion-based quadrotor control.*

## 12.2 Reference Trajectory Generation

The reference trajectory block generates the desired position, velocity, acceleration, and reference attitude required by the controller.

<img src= Images/Reference_Traj.jpeg alt="Reference Trajectory" width="100%">

*Figure 2. Reference trajectory generation block implemented in MATLAB/Simulink.*

## 12.3 Differential-Flatness Mapping

The differential-flatness block converts the corrected desired acceleration into the required thrust and reference quaternion.

<img src= Images/FlatnessMap.jpeg alt="Differential Flatness Mapping" width="100%">

*Figure 3. Differential-flatness transformation for thrust and reference attitude generation.*

## 12.4 Quadrotor Dynamics

The quadrotor plant models the translational dynamics, rotational dynamics, and quaternion kinematics of the vehicle.

<img src=Images/Rotation_Translation.jpeg alt="Quadrotor Dynamics" width="100%">

*Figure 4. Complete quadrotor rotational, translational, and quaternion dynamics model.*

## 12.5 Attitude Controller

The quaternion-based attitude controller generates the control torque required to track the reference attitude.

<img src= Images/Torque.jpeg alt="Attitude Controller" width="100%">

*Figure 5. Quaternion-based attitude controller for control-torque generation.*

## 12.6 Complete Closed-Loop Simulation

The complete closed-loop implementation connects the reference trajectory, position controller, attitude controller, and quadrotor plant with feedback signals.

<img src="Images/Simulink_Result1.jpeg" alt="3D Trajectory Tracking" width="100%">

*Figure 6. Complete closed-loop quadrotor control architecture implemented in MATLAB/Simulink.*

## 12.7 3D Trajectory Tracking

The resulting three-dimensional trajectory is compared with the desired reference trajectory to evaluate tracking performance.

<img src="Images/Simulink_Result.jpeg" alt="Complete Closed Loop Simulation" width="100%">

*Figure 7. Three-dimensional reference and actual quadrotor trajectories obtained from MATLAB/Simulink simulation.*

# 13. MuJoCo Simulation Results

The implemented controller was further evaluated in MuJoCo to verify the quadrotor response in a physics-based simulation environment.

## 13.1 Initial Quadrotor State

<img src=Images/Drone_P0_Before_Avoidance.png alt="MuJoCo Initial Quadrotor State" width="100%">

*Figure 8. Initial quadrotor state in the MuJoCo simulation environment.*

## 13.2 Obstacle Avoidance of Drone 

<img src=Images/Drone_Detected_Obstacle.png alt="MuJoCo Intermediate Flight State" width="100%">

*Figure 9. Quadrotor response during the MuJoCo simulation.*

## 13.3 Final Flight State

<img src=Images/Drone_Avoiding_Reaching_Goal.png alt="MuJoCo Final Flight State" width="100%">

*Figure 10. Quadrotor response at a later stage of the MuJoCo simulation.*

## 13.4 Graph of Mujoco

<img src= Images/Drone_Plot.png alt="MuJoCo Graph" width="100%">

*Figure 11. The plots of Drone Dynamics. *

## 13.5 Graph of Mujoco

<img src= Images/Drone_Path_Plot.png alt="MuJoCo Graph" width="100%">

*Figure 12. The plots of Drone Path. *

# 14 Ardupilot Simulation Results

## 14.1 Starting point of drone 

<img src = Images/Ardupilot_Drone.jpeg alt = "Drone Started from initial point" width="100%">

*Figure 13. Quadrotor Starting Point.*

## 14.2 Drone Following Desired Trajectory

<img src = Images/Drone_Obstacle.jpeg alt = "Drone Started from initial point" width="100%">

*Figure 14. Quadrotor Following the Trajectory

# 15 PyBullet Results 

<img src = Images/Pybullet_result.png alt = "Drone Started from initial point" width="100%">

*Figure 15. Pybullet Result. *


<img src = Images/Pybullet_result1.png alt = "Drone moving from initial point" width="100%">

*Figure 16 . Pybullet Result. *

<img src =  Images/reference_path_generation_graph.png alt = "Trajectory Generation" width="100%">

*Figure 17. Trajectory Generation. *

<img src =  Images/position_tracking_graph.png alt ="Position Tracking" width="100%">

*FIgure 18. *

<img src =  Images/3-D_tracdking_graph.png alt = "Tracking graph" width="100%">

*FIgure 19. *

<img src =  Images/thrust_and_control_torque_graph.png alt = "Thrust and Torque Graph" width="100%">

*FIgure 20. *

<img src =  Images/angular_velocity_graph.png alt = "Angular Velocity" width="100%">

*FIgure 21. *

<img src =  Images/quaternion_and_eulerangles_graph.png alt = "Quaternion and Euler" width="100%">

*FIgure 22. *

<img src =  Images/reference_path_generation_graph.png alt = "Trajectory Generation" width="100%">

*FIgure 23. *

<img src =  Images/reference_path_generation_graph.png alt = "Trajectory Generation" width="100%">

*FIgure 24. *


## Conclusion

This project successfully implemented and evaluated a quaternion-based multicopter trajectory-tracking control framework based on the methodology presented in the reference paper. The developed system integrates *differential flatness, feedback-linearization-based position control, quaternion representation, and computed-torque attitude control* to address the coupled nonlinear translational and rotational dynamics of the multicopter.

The *position controller* generates the required corrective acceleration from the trajectory tracking errors, while the *differential-flatness transformation* converts this acceleration into the required thrust and reference quaternion. The *quaternion-based attitude controller* then generates the required control torque to track the reference orientation while accounting for the nonlinear rotational dynamics. The complete closed-loop model was implemented in *MATLAB/Simulink*, including reference trajectory generation, position control, flatness mapping, attitude control, and multicopter dynamics.

The controller was additionally evaluated using *MuJoCo simulation* to verify the behavior of the implemented multicopter dynamic model in a separate simulation environment. The obtained results demonstrate the complete control pipeline from trajectory generation to position and attitude response. Overall, the implementation provides a systematic quaternion-based approach for multicopter trajectory tracking while avoiding the singularities associated with Euler-angle representations.

