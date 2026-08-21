import numpy as np


# ============================================================
# PAPER ATTITUDE CONTROLLER GAINS
# ============================================================

KPQ = np.diag([
    2000.0,
    2000.0,
    2000.0
])

KDQ = np.diag([
    10.0,
    10.0,
    10.0
])

KIQ = np.diag([
    10.0,
    10.0,
    5.0
])


class AttitudeController:
    """
    Attitude controller - Equation (27) of the paper.

    The controller tracks only:

        q1, q2, q3

    Therefore all input vectors here have exactly
    three elements.
    """

    def __init__(self):

        # Integral of quaternion error
        self.integral_error = np.zeros(3)

    def reset(self):

        self.integral_error = np.zeros(3)

    def update(
        self,
        q_reference,
        q_actual,
        q_dot_reference,
        q_dot_actual,
        q_ddot_reference,
        dt
    ):
        """
        Equation (27):

        tau_tilde =
            q_ddot_reference
            + Kpq * Delta_q
            + Kdq * Delta_q_dot
            + Kiq * integral(Delta_q)

        where:

            Delta_q = q_reference - q_actual
        """

        q_reference = np.asarray(
            q_reference,
            dtype=float
        )

        q_actual = np.asarray(
            q_actual,
            dtype=float
        )

        q_dot_reference = np.asarray(
            q_dot_reference,
            dtype=float
        )

        q_dot_actual = np.asarray(
            q_dot_actual,
            dtype=float
        )

        q_ddot_reference = np.asarray(
            q_ddot_reference,
            dtype=float
        )

        # ----------------------------------------------------
        # Make sure we are using q1, q2, q3 only
        # ----------------------------------------------------

        if q_reference.shape != (3,):
            raise ValueError(
                "q_reference must contain [q1, q2, q3]"
            )

        if q_actual.shape != (3,):
            raise ValueError(
                "q_actual must contain [q1, q2, q3]"
            )

        if q_dot_reference.shape != (3,):
            raise ValueError(
                "q_dot_reference must contain 3 elements"
            )

        if q_dot_actual.shape != (3,):
            raise ValueError(
                "q_dot_actual must contain 3 elements"
            )

        if q_ddot_reference.shape != (3,):
            raise ValueError(
                "q_ddot_reference must contain 3 elements"
            )

        # ----------------------------------------------------
        # Quaternion error
        #
        # Delta_q = q_r - q
        # ----------------------------------------------------

        error = (
            q_reference
            - q_actual
        )

        # ----------------------------------------------------
        # Quaternion derivative error
        #
        # Delta_q_dot = q_dot_r - q_dot
        # ----------------------------------------------------

        error_dot = (
            q_dot_reference
            - q_dot_actual
        )

        # ----------------------------------------------------
        # Integral error
        #
        # integral(Delta_q) dt
        # ----------------------------------------------------

        self.integral_error += (
            error * dt
        )

        # ----------------------------------------------------
        # Equation (27)
        # ----------------------------------------------------

        tau_tilde = (
            q_ddot_reference
            + KPQ @ error
            + KDQ @ error_dot
            + KIQ @ self.integral_error
        )

        return tau_tilde



    # ============================================================
# EQUATION (5) - Q(q)
# ============================================================
def Q_matrix(q):
    """
    Equation (5) of the paper:

        omega = Q(q) q_dot

    Here:

        q = [q1, q2, q3]

    q0 is recovered from the unit-quaternion constraint:

        q0 = sqrt(1 - q1^2 - q2^2 - q3^2)

    Therefore Q(q) is a 3 x 3 matrix.
    """

    q = np.asarray(q, dtype=float)

    q1, q2, q3 = q

    # Recover q0 from unit quaternion condition
    q0_squared = 1.0 - (
        q1**2 + q2**2 + q3**2
    )

    if q0_squared <= 0.0:
        raise ValueError(
            "Invalid quaternion: q0 cannot be recovered."
        )

    q0 = np.sqrt(q0_squared)

    Q = (2.0 / q0) * np.array([
        [
            q0**2 + q1**2,
            q1*q2 + q0*q3,
            q1*q3 - q0*q2
        ],

        [
            q1*q2 - q0*q3,
            q0**2 + q2**2,
            q2*q3 + q0*q1
        ],

        [
            q1*q3 + q0*q2,
            q2*q3 - q0*q1,
            q0**2 + q3**2
        ]
    ])

    return Q


# ============================================================
# JACOBIAN TERM FOR EQUATION (26)
# D_q [ Q(q) q_dot ]
# ============================================================

def jacobian_Q_qdot(q, q_dot):
    """
    Calculate:

        D_q [ Q(q) q_dot ]

    for the 3-variable formulation used in Equation (5).

    q     = [q1, q2, q3]
    q_dot = [q1_dot, q2_dot, q3_dot]

    Returns a 3 x 3 Jacobian matrix.
    """

    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)

    q1, q2, q3 = q
    dq1, dq2, dq3 = q_dot

    # --------------------------------------------------------
    # Recover q0
    # --------------------------------------------------------

    q0_squared = 1.0 - (
        q1**2
        + q2**2
        + q3**2
    )

    if q0_squared <= 0.0:
        raise ValueError(
            "Invalid quaternion: q0 cannot be recovered."
        )

    q0 = np.sqrt(q0_squared)

    # --------------------------------------------------------
    # q0 derivatives
    #
    # q0 = sqrt(1 - q1^2 - q2^2 - q3^2)
    #
    # ∂q0/∂q1 = -q1/q0
    # ∂q0/∂q2 = -q2/q0
    # ∂q0/∂q3 = -q3/q0
    # --------------------------------------------------------

    dq0_dq1 = -q1 / q0
    dq0_dq2 = -q2 / q0
    dq0_dq3 = -q3 / q0

    # --------------------------------------------------------
    # Q(q) q_dot
    #
    # omega = Q(q) q_dot
    #
    # We calculate the Jacobian numerically using
    # central finite differences.
    #
    # This gives:
    #
    # D_q [Q(q) q_dot]
    # --------------------------------------------------------

    def omega_from_q(q_local):
        """
        Calculate omega for a given q=[q1,q2,q3].
        """

        q1_l, q2_l, q3_l = q_local

        q0_l_squared = 1.0 - (
            q1_l**2
            + q2_l**2
            + q3_l**2
        )

        if q0_l_squared <= 0.0:
            raise ValueError(
                "Invalid quaternion during Jacobian calculation."
            )

        q0_l = np.sqrt(q0_l_squared)

        Q_local = (2.0 / q0_l) * np.array([
            [
                q0_l**2 + q1_l**2,
                q1_l*q2_l + q0_l*q3_l,
                q1_l*q3_l - q0_l*q2_l
            ],

            [
                q1_l*q2_l - q0_l*q3_l,
                q0_l**2 + q2_l**2,
                q2_l*q3_l + q0_l*q1_l
            ],

            [
                q1_l*q3_l + q0_l*q2_l,
                q2_l*q3_l - q0_l*q1_l,
                q0_l**2 + q3_l**2
            ]
        ])

        return Q_local @ q_dot

    # --------------------------------------------------------
    # Central-difference Jacobian
    # --------------------------------------------------------

    h = 1e-6

    J = np.zeros((3, 3))

    for i in range(3):

        q_plus = q.copy()
        q_minus = q.copy()

        q_plus[i] += h
        q_minus[i] -= h

        omega_plus = omega_from_q(q_plus)
        omega_minus = omega_from_q(q_minus)

        J[:, i] = (
            omega_plus
            - omega_minus
        ) / (2.0 * h)

    return J



# ============================================================
# EQUATION (26) - COMPUTED TORQUE
# ============================================================

def computed_torque(
    q,
    q_dot,
    tau_tilde
):
    """
    Equation (26) of the paper:

        tau =
        J [ Q(q) tau_tilde
            + D_q[Q(q) q_dot] q_dot ]
        + [Q(q) q_dot] x [J Q(q) q_dot]

    q         = [q1, q2, q3]
    q_dot     = [q1_dot, q2_dot, q3_dot]
    tau_tilde = corrective term from Equation (27)

    Returns:

        tau = [tau_x, tau_y, tau_z]
    """

    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)
    tau_tilde = np.asarray(tau_tilde, dtype=float)

    # --------------------------------------------------------
    # Check dimensions
    # --------------------------------------------------------

    if q.shape != (3,):
        raise ValueError(
            "q must contain [q1, q2, q3]"
        )

    if q_dot.shape != (3,):
        raise ValueError(
            "q_dot must contain [q1_dot, q2_dot, q3_dot]"
        )

    if tau_tilde.shape != (3,):
        raise ValueError(
            "tau_tilde must contain 3 elements"
        )

    # --------------------------------------------------------
    # Import inertia matrix from dynamics.py
    # --------------------------------------------------------

    from dynamics import J

    # --------------------------------------------------------
    # Q(q)
    #
    # omega = Q(q) q_dot
    # --------------------------------------------------------

    Q = Q_matrix(q)

    # --------------------------------------------------------
    # Angular velocity
    # --------------------------------------------------------

    omega = Q @ q_dot

    # --------------------------------------------------------
    # Jacobian
    #
    # D_q [ Q(q) q_dot ]
    # --------------------------------------------------------

    D = jacobian_Q_qdot(
        q,
        q_dot
    )

    # --------------------------------------------------------
    # First term:
    #
    # J Q(q) tau_tilde
    # --------------------------------------------------------

    term_1 = J @ Q @ tau_tilde

    # --------------------------------------------------------
    # Second term:
    #
    # J D_q[Q(q) q_dot] q_dot
    # --------------------------------------------------------

    term_2 = J @ D @ q_dot

    # --------------------------------------------------------
    # Third term:
    #
    # [Q(q) q_dot] x [J Q(q) q_dot]
    #
    # omega = Q(q) q_dot
    # --------------------------------------------------------

    term_3 = np.cross(
        omega,
        J @ omega
    )

    # --------------------------------------------------------
    # Equation (26)
    # --------------------------------------------------------

    tau = (
        term_1
        + term_2
        + term_3
    )

    return tau