import numpy as np


# ============================================================
# MULTICOPTER PARAMETERS
# Taken from the paper's simulation model
# ============================================================

MASS = 0.025                 # kg

JX = 4.856e-3                # kg m^2
JY = 4.856e-3                # kg m^2
JZ = 8.801e-3                # kg m^2

CD = 0.8                     # drag coefficient
RHO = 1.225                  # air density kg/m^3
AREA = 0.01425               # cross-sectional area m^2

G = 9.81                     # gravitational acceleration


# Inertia tensor
J = np.diag([JX, JY, JZ])


# Body z-axis
EZ = np.array([0.0, 0.0, 1.0])


def drag_force(velocity):
    """
    Aerodynamic drag force used in the paper's
    simulation model.

    FD = -1/2 * rho * CD * A * |v| * v
    """

    velocity = np.asarray(velocity, dtype=float)

    speed = np.linalg.norm(velocity)

    return -0.5 * RHO * CD * AREA * speed * velocity


def translational_acceleration(mass, gravity, R, thrust, velocity):
    """
    Computes translational acceleration:

        m * xi_ddot = m*g + R*e_z*T + FD
    """

    gravity_vector = np.array([0.0, 0.0, -gravity])

    FD = drag_force(velocity)

    acceleration = (
        gravity_vector
        + (thrust / mass) * (R @ EZ)
        + FD / mass
    )

    return acceleration


def rotational_acceleration(omega, torque):
    """
    Computes angular acceleration from:

        J * omega_dot + omega x (J omega) = tau

    Therefore:

        omega_dot = J^-1 [tau - omega x (J omega)]
    """

    omega = np.asarray(omega, dtype=float)
    torque = np.asarray(torque, dtype=float)

    angular_momentum = J @ omega

    gyroscopic_term = np.cross(omega, angular_momentum)

    omega_dot = np.linalg.inv(J) @ (
        torque - gyroscopic_term
    )

    return omega_dot