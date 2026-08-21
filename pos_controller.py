import numpy as np


# ============================================================
# PAPER POSITION CONTROLLER GAINS
# Equation (20)
# ============================================================

KP = np.diag([60.0, 60.0, 60.0])
KD = np.diag([6.0, 6.0, 6.0])
KI = np.diag([0.1, 0.1, 0.1])


class PositionController:
    """
    High-level position controller from the paper.

    Equation (20):

        xi_ddot* =
            xi_ddot_r
            + Kp * e_xi
            + Kd * e_dot_xi
            + Ki * integral(e_xi)

    where:

        e_xi = xi_r - xi
    """

    def __init__(self):
        # Integral of position error
        self.integral_error = np.zeros(3)

    def reset(self):
        """Reset the integral error."""
        self.integral_error = np.zeros(3)

    def update(
        self,
        position_reference,
        velocity_reference,
        acceleration_reference,
        position,
        velocity,
        dt
    ):
        """
        Calculate the desired acceleration xi_ddot*.

        Parameters
        ----------
        position_reference : [x_r, y_r, z_r]
        velocity_reference : [x_dot_r, y_dot_r, z_dot_r]
        acceleration_reference : [x_ddot_r, y_ddot_r, z_ddot_r]

        position : actual [x, y, z]
        velocity : actual [x_dot, y_dot, z_dot]

        dt : simulation time step

        Returns
        -------
        desired_acceleration : xi_ddot*
        """

        position_reference = np.asarray(
            position_reference,
            dtype=float
        )

        velocity_reference = np.asarray(
            velocity_reference,
            dtype=float
        )

        acceleration_reference = np.asarray(
            acceleration_reference,
            dtype=float
        )

        position = np.asarray(
            position,
            dtype=float
        )

        velocity = np.asarray(
            velocity,
            dtype=float
        )

        # ----------------------------------------------------
        # Position error
        #
        # e_xi = xi_r - xi
        # ----------------------------------------------------

        position_error = (
            position_reference - position
        )

        # ----------------------------------------------------
        # Velocity error
        #
        # e_dot_xi = xi_dot_r - xi_dot
        # ----------------------------------------------------

        velocity_error = (
            velocity_reference - velocity
        )

        # ----------------------------------------------------
        # Integral of position error
        #
        # integral(e_xi) dt
        # ----------------------------------------------------

        self.integral_error += (
            position_error * dt
        )

        # ----------------------------------------------------
        # Equation (20)
        # ----------------------------------------------------

        desired_acceleration = (
            acceleration_reference
            + KP @ position_error
            + KD @ velocity_error
            + KI @ self.integral_error
        )

        return desired_acceleration