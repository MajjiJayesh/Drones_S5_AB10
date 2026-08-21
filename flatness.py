import numpy as np

from dynamics import MASS, G


def thrust_from_acceleration(acceleration):
    """
    Paper Eq. (17) / Appendix Eq. (A.3)

        T = m * sqrt(
                x_ddot^2
                + y_ddot^2
                + (z_ddot + g)^2
            )
    """

    acceleration = np.asarray(acceleration, dtype=float)

    x_ddot = acceleration[0]
    y_ddot = acceleration[1]
    z_ddot = acceleration[2]

    D = np.sqrt(
        x_ddot**2
        + y_ddot**2
        + (z_ddot + G)**2
    )

    thrust = MASS * D

    return thrust


def quaternion_from_acceleration(acceleration, q3=0.0):
    """
    Reconstruct q0, q1, q2 from the desired acceleration.

    Based directly on Appendix A:

        Eq. (A.5)  -> q0
        Eq. (A.8a) -> q1
        Eq. (A.8b) -> q2

    q3 is selected by the user.

    For the paper's simulation:
        q3r = 0
    """

    acceleration = np.asarray(acceleration, dtype=float)

    x_ddot = acceleration[0]
    y_ddot = acceleration[1]
    z_ddot = acceleration[2]

    # --------------------------------------------------------
    # Common term:
    #
    # D = sqrt(x_ddot^2 + y_ddot^2 + (z_ddot + g)^2)
    #
    # This is the quantity appearing in Eq. (A.3), (A.5),
    # (A.8a), and (A.8b).
    # --------------------------------------------------------

    D = np.sqrt(
        x_ddot**2
        + y_ddot**2
        + (z_ddot + G)**2
    )

    # --------------------------------------------------------
    # Square-root term appearing in Eq. (A.5), (A.8a),
    # and (A.8b)
    #
    # sqrt((z_ddot + g)/D - 2*q3^2 + 1)
    # --------------------------------------------------------

    S = np.sqrt(
        (z_ddot + G) / D
        - 2.0 * q3**2
        + 1.0
    )

    # --------------------------------------------------------
    # Equation (A.5)
    #
    # q0 = 1/sqrt(2) * S
    # --------------------------------------------------------

    q0 = S / np.sqrt(2.0)

    # Common denominator in Eq. (A.8a) and (A.8b)

    denominator = (z_ddot + G) + D

    # --------------------------------------------------------
    # Equation (A.8a)
    #
    # q1 =
    # [x_ddot*q3
    #  - (1/sqrt(2))*y_ddot*S]
    # /
    # [(z_ddot + g) + D]
    # --------------------------------------------------------

    q1 = (
        x_ddot * q3
        - (y_ddot * S) / np.sqrt(2.0)
    ) / denominator

    # --------------------------------------------------------
    # Equation (A.8b)
    #
    # q2 =
    # [y_ddot*q3
    #  + (1/sqrt(2))*x_ddot*S]
    # /
    # [(z_ddot + g) + D]
    # --------------------------------------------------------

    q2 = (
        y_ddot * q3
        + (x_ddot * S) / np.sqrt(2.0)
    ) / denominator

    # Complete quaternion
    q = np.array([
        q0,
        q1,
        q2,
        q3
    ])

    return q