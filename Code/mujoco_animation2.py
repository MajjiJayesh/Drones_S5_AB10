import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer


# ============================================================
# PAPER MULTICOPTER — MUJOCO VISUALIZATION
# ============================================================
#
# IMPORTANT:
#
# This file is ONLY the visualization layer.
#
# It does NOT change:
#   - controller
#   - computed torque
#   - dynamics
#   - thrust
#   - torque
#   - actual simulated position
#   - actual simulated attitude
#   - trajectory generation
#
# The drone position remains:
#
#       actual_positions[i]
#
# The reference path is drawn from:
#
#       reference_positions
#
# Therefore:
#
#   STATIC REFERENCE PATH
#          +
#   ACTUAL DRONE MOTION
#
# ============================================================


BASE_DIR = Path(__file__).resolve().parent

MODEL_PATH = BASE_DIR / "model" / "multicopter.xml"
RESULTS_PATH = BASE_DIR / "simulation_results.npz"


# ============================================================
# VISUAL SETTINGS
# ============================================================

# 1.0 = real-time playback
PLAYBACK_SPEED = 1.0

# Purely visual rotor rotation.
ROTOR_SPEED = 90.0

# Existing XML trail update rate.
TRAIL_UPDATE_RATE = 2

# Camera
CAMERA_AZIMUTH = 135.0
CAMERA_ELEVATION = -22.0
CAMERA_DISTANCE = 5.8


# ============================================================
# NEW REFERENCE PATH SETTINGS
# ============================================================

# Thickness of the static reference path.
#
# This is deliberately much thicker than the old trail so that
# P0 -> P1 -> P2 is clearly visible during presentation.
REFERENCE_PATH_RADIUS = 0.035

# Maximum number of capsule segments used for the path.
# Keeping this below MuJoCo's user-scene capacity makes the
# visualization robust even with 2001 simulation samples.
REFERENCE_PATH_MAX_SEGMENTS = 450

# Reference-path colour.
#
# Blue = reference, matching the conventional plot notation.
REFERENCE_PATH_RGBA = np.array(
    [0.05, 0.45, 1.00, 0.95],
    dtype=np.float32
)


# ============================================================
# LOAD RESULTS
# ============================================================

def load_results():

    if not RESULTS_PATH.exists():
        raise FileNotFoundError(
            f"simulation_results.npz not found:\n"
            f"{RESULTS_PATH}"
        )

    data = np.load(
        RESULTS_PATH,
        allow_pickle=True
    )

    print()
    print("Available result arrays:")

    for key in data.files:

        print(
            f"  {key}: "
            f"shape={np.asarray(data[key]).shape}"
        )

    return data


# ============================================================
# ARRAY HELPER
# ============================================================

def get_array(data, names):

    lower_map = {
        key.lower(): key
        for key in data.files
    }

    for name in names:

        if name.lower() in lower_map:

            return np.asarray(
                data[lower_map[name.lower()]],
                dtype=float
            )

    return None


# ============================================================
# REQUIRED ARRAY
# ============================================================

def require_array(
    data,
    names,
    description
):

    value = get_array(
        data,
        names
    )

    if value is None:

        raise ValueError(
            f"Could not find {description}.\n"
            f"Expected one of: {names}"
        )

    return value


# ============================================================
# QUATERNION NORMALIZATION
# ============================================================

def normalize_quaternions(q):

    q = np.asarray(
        q,
        dtype=float
    )

    if q.ndim != 2 or q.shape[1] != 4:

        raise ValueError(
            f"Quaternion array must have shape (N,4), "
            f"got {q.shape}"
        )

    norms = np.linalg.norm(
        q,
        axis=1
    )

    if np.any(~np.isfinite(norms)):

        raise ValueError(
            "Quaternion data contains NaN/Inf."
        )

    if np.any(norms < 1e-12):

        raise ValueError(
            "Quaternion data contains zero-norm samples."
        )

    q = q / norms[:, None]

    # --------------------------------------------------------
    # Quaternion sign continuity
    #
    # q and -q represent the same orientation.
    #
    # This prevents artificial sign flips.
    # --------------------------------------------------------

    for i in range(1, len(q)):

        if np.dot(
            q[i - 1],
            q[i]
        ) < 0.0:

            q[i] *= -1.0

    return q


# ============================================================
# QUATERNION SLERP
# ============================================================

def slerp(
    q0,
    q1,
    alpha
):

    q0 = np.asarray(
        q0,
        dtype=float
    )

    q1 = np.asarray(
        q1,
        dtype=float
    )

    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    dot = np.dot(
        q0,
        q1
    )

    if dot < 0.0:

        q1 = -q1
        dot = -dot

    dot = np.clip(
        dot,
        -1.0,
        1.0
    )

    if dot > 0.9995:

        result = (
            q0
            + alpha * (q1 - q0)
        )

        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)

    sin_theta_0 = np.sin(
        theta_0
    )

    theta = (
        theta_0 * alpha
    )

    sin_theta = np.sin(
        theta
    )

    s0 = (
        np.cos(theta)
        - dot
        * sin_theta
        / sin_theta_0
    )

    s1 = (
        sin_theta
        / sin_theta_0
    )

    result = (
        s0 * q0
        + s1 * q1
    )

    return result / np.linalg.norm(result)


# ============================================================
# FIND FREE JOINT
# ============================================================

def find_free_joint(model):

    for jid in range(
        model.njnt
    ):

        if (
            model.jnt_type[jid]
            == mujoco.mjtJoint.mjJNT_FREE
        ):

            return jid

    raise RuntimeError(
        "No free joint found in multicopter.xml."
    )


# ============================================================
# FIND ROTOR JOINTS
# ============================================================

def find_rotor_joints(model):

    names = [
        "rotor_fl_joint",
        "rotor_fr_joint",
        "rotor_rl_joint",
        "rotor_rr_joint"
    ]

    result = []

    for name in names:

        jid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name
        )

        if jid >= 0:

            result.append(jid)

    return result


# ============================================================
# FIND TRAIL SITES
# ============================================================

def find_sites(
    model,
    prefix
):

    result = []

    for sid in range(
        model.nsite
    ):

        name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            sid
        )

        if (
            name is not None
            and name.startswith(prefix)
        ):

            result.append(sid)

    # Sort numerically:
    #
    # actual_trail_0
    # actual_trail_1
    # actual_trail_2
    # ...
    #

    result.sort(
        key=lambda sid:
        int(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_SITE,
                sid
            ).split("_")[-1]
        )
    )

    return result


# ============================================================
# UPDATE EXISTING TRAIL
# ============================================================

def update_trail(
    model,
    site_ids,
    positions,
    current_index
):

    if len(site_ids) == 0:

        return

    count = len(site_ids)

    start = max(
        0,
        current_index
        - count * TRAIL_UPDATE_RATE
    )

    available = positions[
        start:
        current_index + 1:
        TRAIL_UPDATE_RATE
    ]

    if len(available) == 0:

        return

    indices = np.linspace(
        0,
        len(available) - 1,
        count
    ).astype(int)

    for sid, idx in zip(
        site_ids,
        indices
    ):

        model.site_pos[sid] = (
            available[idx]
        )


# ============================================================
# ROTATION MATRIX
#
# Creates a rotation matrix whose local Z-axis points along
# the supplied vector.
# ============================================================

def rotation_matrix_from_z(
    vector
):

    vector = np.asarray(
        vector,
        dtype=float
    )

    norm = np.linalg.norm(
        vector
    )

    if norm < 1e-12:

        return np.eye(3)

    z = vector / norm

    # Choose a reference vector that is not parallel to z.

    if abs(z[2]) < 0.9:

        reference = np.array(
            [0.0, 0.0, 1.0]
        )

    else:

        reference = np.array(
            [1.0, 0.0, 0.0]
        )

    x = np.cross(
        reference,
        z
    )

    x_norm = np.linalg.norm(
        x
    )

    if x_norm < 1e-12:

        return np.eye(3)

    x = x / x_norm

    y = np.cross(
        z,
        x
    )

    y = y / np.linalg.norm(y)

    # Columns are local x, y, z axes.
    R = np.column_stack(
        [x, y, z]
    )

    return R


# ============================================================
# DRAW ONE CAPSULE
#
# A capsule is used as a thick 3-D line segment.
# ============================================================

def add_path_segment(
    viewer,
    p0,
    p1,
    radius
):

    user_scene = viewer.user_scn

    if (
        user_scene.ngeom
        >= len(user_scene.geoms)
    ):

        return False

    p0 = np.asarray(
        p0,
        dtype=float
    )

    p1 = np.asarray(
        p1,
        dtype=float
    )

    direction = p1 - p0

    length = np.linalg.norm(
        direction
    )

    if length < 1e-9:

        return True

    midpoint = (
        p0 + p1
    ) * 0.5

    rotation = (
        rotation_matrix_from_z(
            direction
        )
    )

    geom = user_scene.geoms[
        user_scene.ngeom
    ]

    # Capsule size:
    #
    # size[0] = radius
    # size[1] = half length
    # size[2] unused
    #

    size = np.array(
        [
            radius,
            length * 0.5,
            0.0
        ],
        dtype=np.float64
    )

    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        size,
        midpoint,
        rotation.flatten(),
        REFERENCE_PATH_RGBA
    )

    user_scene.ngeom += 1

    return True


# ============================================================
# DRAW STATIC REFERENCE PATH
# ============================================================
#
# IMPORTANT:
#
# This function uses ONLY reference_positions.
#
# It does NOT modify:
#
#     actual_positions
#
# Therefore the drone animation itself is unchanged.
#
# The reference path is STATIC.
# ============================================================

def draw_reference_path(
    viewer,
    reference_positions
):

    reference_positions = np.asarray(
        reference_positions,
        dtype=float
    )

    if (
        reference_positions.ndim != 2
        or reference_positions.shape[1] != 3
    ):

        raise ValueError(
            "reference_positions must have shape (N,3)"
        )

    # --------------------------------------------------------
    # Remove duplicate consecutive points.
    # --------------------------------------------------------

    differences = np.linalg.norm(
        np.diff(
            reference_positions,
            axis=0
        ),
        axis=1
    )

    keep = np.concatenate(
        [
            np.array([True]),
            differences > 1e-9
        ]
    )

    path = reference_positions[
        keep
    ]

    if len(path) < 2:

        return

    # --------------------------------------------------------
    # Downsample only the VISUAL path.
    #
    # The actual simulation data is NOT changed.
    # --------------------------------------------------------

    if (
        len(path)
        > REFERENCE_PATH_MAX_SEGMENTS + 1
    ):

        indices = np.linspace(
            0,
            len(path) - 1,
            REFERENCE_PATH_MAX_SEGMENTS + 1
        ).astype(int)

        path = path[
            indices
        ]

    # --------------------------------------------------------
    # Draw each segment.
    # --------------------------------------------------------

    viewer.user_scn.ngeom = 0

    for i in range(
        len(path) - 1
    ):

        success = add_path_segment(
            viewer,
            path[i],
            path[i + 1],
            REFERENCE_PATH_RADIUS
        )

        if not success:

            break


# ============================================================
# CAMERA SETUP
# ============================================================

def setup_camera(
    viewer,
    actual_positions,
    reference_positions
):

    all_positions = np.vstack(
        [
            actual_positions,
            reference_positions
        ]
    )

    center = np.mean(
        all_positions,
        axis=0
    )

    viewer.cam.azimuth = (
        CAMERA_AZIMUTH
    )

    viewer.cam.elevation = (
        CAMERA_ELEVATION
    )

    viewer.cam.distance = (
        CAMERA_DISTANCE
    )

    viewer.cam.lookat[:] = (
        center
    )


# ============================================================
# CAMERA FOLLOW
# ============================================================

def update_camera(
    viewer,
    drone_position,
    trajectory_center
):

    # Keep the drone as the main visual focus while retaining
    # enough of the reference path in view.

    target = (
        0.78 * drone_position
        + 0.22 * trajectory_center
    )

    viewer.cam.lookat[:] = (
        0.97 * viewer.cam.lookat
        + 0.03 * target
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 64)

    print(
        "PAPER MULTICOPTER — MUJOCO ANIMATION"
    )

    print("=" * 64)


    # ========================================================
    # LOAD RESULTS
    # ========================================================

    data = load_results()


    # ========================================================
    # ACTUAL POSITION
    # ========================================================

    actual_positions = require_array(
        data,
        [
            "actual_positions",
            "actual_position",
            "position",
            "positions"
        ],
        "actual positions"
    )


    # ========================================================
    # REFERENCE POSITION
    # ========================================================

    reference_positions = require_array(
        data,
        [
            "reference_positions",
            "reference_position"
        ],
        "reference positions"
    )


    # ========================================================
    # ACTUAL QUATERNIONS
    # ========================================================

    actual_quaternions = require_array(
        data,
        [
            "actual_quaternions",
            "actual_quaternion"
        ],
        "actual quaternions"
    )


    # ========================================================
    # REFERENCE QUATERNIONS
    # ========================================================

    reference_quaternions = get_array(
        data,
        [
            "reference_quaternions",
            "reference_quaternion"
        ]
    )


    # ========================================================
    # TIME
    # ========================================================

    times = get_array(
        data,
        [
            "times",
            "time",
            "t"
        ]
    )

    if times is None:

        times = (
            np.arange(
                len(actual_positions)
            )
            * 0.01
        )

    times = np.asarray(
        times,
        dtype=float
    ).reshape(-1)


    # ========================================================
    # MATCH ARRAY LENGTHS
    # ========================================================

    lengths = [
        len(actual_positions),
        len(reference_positions),
        len(actual_quaternions),
        len(times)
    ]

    if reference_quaternions is not None:

        lengths.append(
            len(reference_quaternions)
        )

    n = min(
        lengths
    )

    actual_positions = (
        actual_positions[:n]
    )

    reference_positions = (
        reference_positions[:n]
    )

    actual_quaternions = (
        actual_quaternions[:n]
    )

    times = (
        times[:n]
    )

    if reference_quaternions is not None:

        reference_quaternions = (
            reference_quaternions[:n]
        )


    # ========================================================
    # VALIDATION
    # ========================================================

    if actual_positions.ndim != 2:

        raise ValueError(
            "actual_positions must be 2-D."
        )

    if actual_positions.shape[1] != 3:

        raise ValueError(
            "actual_positions must have shape (N,3)."
        )

    if reference_positions.shape[1] != 3:

        raise ValueError(
            "reference_positions must have shape (N,3)."
        )

    if actual_quaternions.shape[1] != 4:

        raise ValueError(
            "actual_quaternions must have shape (N,4)."
        )

    if not (
        np.all(
            np.isfinite(
                actual_positions
            )
        )
        and
        np.all(
            np.isfinite(
                reference_positions
            )
        )
        and
        np.all(
            np.isfinite(
                actual_quaternions
            )
        )
        and
        np.all(
            np.isfinite(
                times
            )
        )
    ):

        raise ValueError(
            "Simulation results contain NaN/Inf values."
        )


    # ========================================================
    # QUATERNION NORMALIZATION
    # ========================================================

    actual_quaternions = (
        normalize_quaternions(
            actual_quaternions
        )
    )

    if reference_quaternions is not None:

        reference_quaternions = (
            normalize_quaternions(
                reference_quaternions
            )
        )


    # ========================================================
    # LOAD MUJOCO MODEL
    # ========================================================

    if not MODEL_PATH.exists():

        raise FileNotFoundError(
            f"MuJoCo model not found:\n"
            f"{MODEL_PATH}"
        )

    print()

    print(
        "Loading MuJoCo model:"
    )

    print(
        MODEL_PATH
    )

    model = (
        mujoco.MjModel.from_xml_path(
            str(MODEL_PATH)
        )
    )

    mj_data = (
        mujoco.MjData(model)
    )


    # ========================================================
    # FIND FREE JOINT
    # ========================================================

    free_joint = (
        find_free_joint(
            model
        )
    )

    free_qpos = (
        model.jnt_qposadr[
            free_joint
        ]
    )


    # ========================================================
    # FIND ROTOR JOINTS
    # ========================================================

    rotor_joints = (
        find_rotor_joints(
            model
        )
    )

    rotor_qpos = [

        model.jnt_qposadr[jid]

        for jid in rotor_joints

    ]


    # ========================================================
    # FIND EXISTING TRAIL SITES
    # ========================================================

    actual_sites = find_sites(
        model,
        "actual_trail_"
    )

    reference_sites = find_sites(
        model,
        "reference_trail_"
    )


    # ========================================================
    # INFORMATION
    # ========================================================

    duration = (
        times[-1]
        - times[0]
    )

    if len(times) > 1:

        dt = np.median(
            np.diff(times)
        )

    else:

        dt = 0.01


    print()

    print("=" * 64)

    print(
        "ANIMATION INFORMATION"
    )

    print("=" * 64)

    print(
        f"Frames          : {n}"
    )

    print(
        f"Duration        : {duration:.2f} s"
    )

    print(
        f"Sampling time   : {dt:.4f} s"
    )

    print(
        f"Rotor joints    : {len(rotor_joints)}"
    )

    print(
        f"Actual trail    : {len(actual_sites)}"
    )

    print(
        f"Reference trail : {len(reference_sites)}"
    )


    # ========================================================
    # REFERENCE POINTS
    # ========================================================
    #
    # These are taken directly from the reference trajectory.
    #
    # No new trajectory is created here.
    # ========================================================

    P0 = reference_positions[0]

    P2 = reference_positions[-1]

    # The middle point is selected as the point of maximum
    # change in the reference path's Z direction.
    #
    # This is ONLY for reporting.
    # It does NOT modify the trajectory.

    z_change = np.abs(
        np.diff(
            reference_positions[:, 2]
        )
    )

    if len(z_change) > 0:

        middle_index = (
            int(
                np.argmax(
                    z_change
                )
            )
            + 1
        )

    else:

        middle_index = (
            len(reference_positions) // 2
        )

    P1 = reference_positions[
        middle_index
    ]


    print()

    print(
        "REFERENCE PATH"
    )

    print(
        f"P0 = {P0}"
    )

    print(
        f"P1 = {P1}"
    )

    print(
        f"P2 = {P2}"
    )


    # ========================================================
    # CAMERA CENTER
    # ========================================================

    trajectory_center = np.mean(
        np.vstack(
            [
                actual_positions,
                reference_positions
            ]
        ),
        axis=0
    )


    # ========================================================
    # START MUJOCO
    # ========================================================

    print()

    print(
        "Starting MuJoCo viewer..."
    )


    with mujoco.viewer.launch_passive(
        model,
        mj_data
    ) as viewer:


        # ====================================================
        # CAMERA
        # ====================================================

        setup_camera(
            viewer,
            actual_positions,
            reference_positions
        )


        # ====================================================
        # INITIAL DRONE POSITION
        #
        # IMPORTANT:
        #
        # KEEP ACTUAL POSITION.
        #
        # This is the existing animation behaviour.
        # ====================================================

        mj_data.qpos[
            free_qpos:
            free_qpos + 3
        ] = actual_positions[0]


        # ====================================================
        # INITIAL ATTITUDE
        # ====================================================

        if reference_quaternions is not None:

            display_quaternion = (
                reference_quaternions[0]
            )

        else:

            display_quaternion = (
                actual_quaternions[0]
            )


        mj_data.qpos[
            free_qpos + 3:
            free_qpos + 7
        ] = display_quaternion


        # ====================================================
        # INITIAL TRAILS
        # ====================================================

        update_trail(
            model,
            actual_sites,
            actual_positions,
            0
        )

        update_trail(
            model,
            reference_sites,
            reference_positions,
            0
        )


        # ====================================================
        # STATIC REFERENCE PATH
        #
        # THIS IS THE ONLY MAJOR ADDITION.
        #
        # The path is drawn from reference_positions.
        #
        # It does NOT follow the drone.
        # ====================================================

        draw_reference_path(
            viewer,
            reference_positions
        )


        # ====================================================
        # FORWARD KINEMATICS
        # ====================================================

        mujoco.mj_forward(
            model,
            mj_data
        )

        viewer.sync()


        # ====================================================
        # REAL-TIME PLAYBACK
        # ====================================================

        wall_start = (
            time.perf_counter()
        )

        simulation_start = (
            times[0]
        )


        # ====================================================
        # FRAME LOOP
        # ====================================================

        for i in range(n):

            if not viewer.is_running():

                break


            # =================================================
            # POSITION
            #
            # DO NOT CHANGE THIS.
            #
            # The drone uses the actual numerical simulation.
            # =================================================

            mj_data.qpos[
                free_qpos:
                free_qpos + 3
            ] = actual_positions[i]


            # =================================================
            # ATTITUDE
            #
            # Existing behaviour preserved.
            # =================================================

            if reference_quaternions is not None:

                if i < n - 1:

                    q_display = slerp(
                        reference_quaternions[i],
                        reference_quaternions[
                            min(
                                i + 1,
                                n - 1
                            )
                        ],
                        0.5
                    )

                else:

                    q_display = (
                        reference_quaternions[i]
                    )

            else:

                if i < n - 1:

                    q_display = slerp(
                        actual_quaternions[i],
                        actual_quaternions[
                            min(
                                i + 1,
                                n - 1
                            )
                        ],
                        0.5
                    )

                else:

                    q_display = (
                        actual_quaternions[i]
                    )


            mj_data.qpos[
                free_qpos + 3:
                free_qpos + 7
            ] = q_display


            # =================================================
            # ROTOR ANIMATION
            #
            # Visual only.
            # =================================================

            rotor_angle = (
                ROTOR_SPEED
                * (
                    times[i]
                    - simulation_start
                )
            )


            for r, qadr in enumerate(
                rotor_qpos
            ):

                direction = (
                    1.0
                    if r % 2 == 0
                    else -1.0
                )

                mj_data.qpos[
                    qadr
                ] = (
                    direction
                    * rotor_angle
                )


            # =================================================
            # EXISTING TRAILS
            #
            # Leave these exactly as before.
            # =================================================

            if (
                i % TRAIL_UPDATE_RATE == 0
                or i == n - 1
            ):

                update_trail(
                    model,
                    actual_sites,
                    actual_positions,
                    i
                )

                update_trail(
                    model,
                    reference_sites,
                    reference_positions,
                    i
                )


            # =================================================
            # FORWARD KINEMATICS
            # =================================================

            mujoco.mj_forward(
                model,
                mj_data
            )


            # =================================================
            # CAMERA FOLLOW
            # =================================================

            update_camera(
                viewer,
                actual_positions[i],
                trajectory_center
            )


            # =================================================
            # DISPLAY
            # =================================================

            viewer.sync()


            # =================================================
            # REAL-TIME TIMING
            # =================================================

            target_wall_time = (
                wall_start
                +
                (
                    times[i]
                    - simulation_start
                )
                / PLAYBACK_SPEED
            )

            remaining = (
                target_wall_time
                -
                time.perf_counter()
            )

            if remaining > 0:

                time.sleep(
                    remaining
                )


    # ========================================================
    # COMPLETE
    # ========================================================

    print()

    print("=" * 64)

    print(
        "MUJOCO ANIMATION FINISHED"
    )

    print("=" * 64)


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()