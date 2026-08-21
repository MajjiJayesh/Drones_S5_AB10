import numpy as np

from dynamics import J
from controller.attitude_controller import (
    Q_matrix,
    jacobian_Q_qdot
)


def computed_torque(
    q,
    q_dot,
    tau_tilde
):
    """
    Equation (26) of the paper.

    q        = [q1, q2, q3]
    q_dot    = [q1_dot, q2_dot, q3_dot]
    tau_tilde = corrective term from Equation (27)
    """

    q = np.asarray(q, dtype=float)
    q_dot = np.asarray(q_dot, dtype=float)
    tau_tilde = np.asarray(tau_tilde, dtype=float)

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
    # Q(q)
    # --------------------------------------------------------

    Q = Q_matrix(q)

    # --------------------------------------------------------
    # Angular velocity
    # omega = Q(q) q_dot
    # --------------------------------------------------------

    omega = Q @ q_dot

    # --------------------------------------------------------
    # D_q[Q(q) q_dot]
    # --------------------------------------------------------

    D = jacobian_Q_qdot(
        q,
        q_dot
    )

    # --------------------------------------------------------
    # Equation (26)
    # --------------------------------------------------------

    term_1 = J @ Q @ tau_tilde

    term_2 = J @ D @ q_dot

    term_3 = np.cross(
        omega,
        J @ omega
    )

    tau = (
        term_1
        + term_2
        + term_3
    )

    return tau