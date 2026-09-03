import marimo

__generated_with = "0.24.0"
app = marimo.App(width="medium")


@app.cell
def _():
    import marimo as mo
    import os, sys
    import threading
    import mujoco
    import imageio, numpy as np


    project_root = str(mo.notebook_dir().parent)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    return imageio, mo, mujoco, np


@app.cell
def _():
    # ROBOT = "unitree_g1"
    # ROBOT_SCENE = "../unitree_mujoco/unitree_robots/" + ROBOT + "/scene_with_hands.xml" # Robot scene

    # FPS = 30

    # # compiled physics model of robot environment (static) (read-only reference)
    # # mass, geometry, joints, gravity
    # mj_model = mujoco.MjModel.from_xml_path(ROBOT_SCENE)

    # # live dynamic states of environment and robot
    # mj_data = mujoco.MjData(mj_model)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Interaction Demo
    """)
    return


@app.cell
def _():
    # # map actuators
    # def actuator(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    # RIGHT_ARM = [actuator(mj_model, f"right_{j}_joint") for j in
    #              ["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow",
    #               "wrist_roll","wrist_pitch","wrist_yaw"]]

    # LEFT_ARM = [actuator(mj_model, f"left_{j}_joint") for j in
    #              ["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow",
    #               "wrist_roll","wrist_pitch","wrist_yaw"]]

    # LEFT_HAND = [actuator(mj_model, n) for n in [
    #     "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    #     "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    #     "left_hand_index_0_joint", "left_hand_index_1_joint",
    # ]]

    # # waist pitch joint (1)
    # WAIST_PITCH = actuator(mj_model, "waist_pitch_joint")
    # LS_PITCH = actuator(mj_model, "left_shoulder_pitch_joint")

    # def get_joint_qps_idx(m, actuator_id):
    #     jid = m.actuator_trnid[actuator_id, 0]
    #     return m.jnt_qposadr[jid]

    # WP_Q = get_joint_qps_idx(mj_model, WAIST_PITCH)
    # LSP_Q = get_joint_qps_idx(mj_model, LEFT_ARM[0])
    # LSY_Q = get_joint_qps_idx(mj_model, LEFT_ARM[2])
    # LE_Q = get_joint_qps_idx(mj_model, LEFT_ARM[3])
    # LWP_Q = get_joint_qps_idx(mj_model, LEFT_ARM[5])


    # WP_THRESH = 0.5
    # LSP_THRESH = -0.7 # -0.67
    # LSY_THRESH = 0.24
    # LE_THRESH = 0.8 # 0.55
    # LWP_THRESH = -0.3 # -0.3

    # mj_data.ctrl[WAIST_PITCH] = WP_THRESH
    # mj_data.ctrl[LEFT_ARM[0]] = LSP_THRESH
    # mj_data.ctrl[LEFT_ARM[2]] = LSY_THRESH
    # mj_data.ctrl[LEFT_ARM[3]] = LE_THRESH
    # mj_data.ctrl[LEFT_ARM[5]] = LWP_THRESH

    # def controller(d, t):
    #     # placeholder: sweep the shoulder
    #     d.ctrl[LEFT_ARM[6]] = 1.0 * np.sin(t)

    # # rendering for mp4
    # renderer = mujoco.Renderer(mj_model, height=480, width=640)
    # # FPS = 30
    # frames = []

    # while mj_data.time < 5.0: # 5 seconds
    #     mujoco.mj_step(mj_model, mj_data)

    #     if len(frames) < mj_data.time * FPS:
    #         renderer.update_scene(mj_data)
    #         frames.append(renderer.render())

    #     # conditions
    #     wp_done = abs(mj_data.qpos[WP_Q] - WP_THRESH) < 0.02
    #     lsp_done = abs(mj_data.qpos[LSP_Q] - LSP_THRESH) < 0.02
    #     le_done = abs(mj_data.qpos[LE_Q] - LE_THRESH) < 0.02
    #     if wp_done and lsp_done and le_done:
    #         # print(f"both converged at t={mj_data.time}")
    #         # break
    #         controller(mj_data, mj_data.time)

    # imageio.mimsave("slap_cube.mp4", frames, fps=FPS)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Grip Cube
    """)
    return


@app.cell
def _():
    # """
    # Differential inverse kinematics for the Unitree G1 left arm in MuJoCo.

    # Replaces hand-tuned joint targets with Cartesian pose commands:
    #     move_to(target_pos, target_quat)  ->  the solver figures out the joints.

    # Uses damped least squares (DLS) on the body Jacobian. This is the same
    # problem mink solves as a QP; this version is dependency-free so you can
    # see the mechanism.

    # Run:  mjpython g1_diff_ik.py     (mjpython is required on macOS)
    # """


    # def name_id(objtype, name):
    #     i = mujoco.mj_name2id(mj_model, objtype, name)
    #     if i < 0:
    #         raise ValueError(f"no {objtype} named {name!r}")
    #     return i


    # def list_candidates(substrings=("hand", "wrist", "palm")):
    #     """Print body/site names so you can pick the right end-effector frame."""
    #     for objtype, label, count in (
    #         (mujoco.mjtObj.mjOBJ_BODY, "body", mj_model.nbody),
    #         (mujoco.mjtObj.mjOBJ_SITE, "site", mj_model.nsite),
    #     ):
    #         for i in range(count):
    #             n = mujoco.mj_id2name(mj_model, objtype, i)
    #             if n and any(s in n for s in substrings):
    #                 print(f"{label:5s} {i:3d}  {n}")


    # # ----------------------------------------------------------------------------
    # # IK chain
    # # ----------------------------------------------------------------------------
    # # waist_pitch is included deliberately: it lets the solver lean the torso in
    # # for reach instead of you pinning it at a guessed constant.
    # CHAIN_JOINTS = [
    #     "waist_pitch_joint",
    #     "left_shoulder_pitch_joint",
    #     "left_shoulder_roll_joint",
    #     "left_shoulder_yaw_joint",
    #     "left_elbow_joint",
    #     "left_wrist_roll_joint",
    #     "left_wrist_pitch_joint",
    #     "left_wrist_yaw_joint",
    # ]

    # CHAIN_JIDS = [name_id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in CHAIN_JOINTS]
    # CHAIN_ACT = np.array([name_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in CHAIN_JOINTS])

    # # NOTE: two different index spaces. dofadr indexes Jacobian columns (length nv),
    # # qposadr indexes joint angles (length nq). They diverge because the pelvis
    # # freejoint is 7 qpos but 6 dof. Using qposadr on the Jacobian is a silent bug.
    # CHAIN_DOF = np.array([mj_model.jnt_dofadr[j] for j in CHAIN_JIDS])
    # CHAIN_QPOS = np.array([mj_model.jnt_qposadr[j] for j in CHAIN_JIDS])

    # N = len(CHAIN_JOINTS)

    # # End-effector frame. Run list_candidates() once and set this to the real name.
    # # A <site> placed at the grasp centre between the fingers is better than a body
    # # origin -- mj_jacBody gives you the Jacobian at the body frame, which for a
    # # palm link usually sits at the wrist, not where the object will be held.
    # EE_BODY = "left_wrist_yaw_link"
    # EE_BID = name_id(mujoco.mjtObj.mjOBJ_BODY, EE_BODY)

    # _jacp = np.zeros((3, mj_model.nv))
    # _jacr = np.zeros((3, mj_model.nv))


    # # ----------------------------------------------------------------------------
    # # Solver
    # # ----------------------------------------------------------------------------
    # def pose_error(target_pos, target_quat):
    #     """Cartesian error as (3,) translation + (3,) rotation vector."""
    #     e_pos = target_pos - mj_data.xpos[EE_BID]

    #     negq = np.zeros(4)
    #     errq = np.zeros(4)
    #     e_rot = np.zeros(3)
    #     mujoco.mju_negQuat(negq, mj_data.xquat[EE_BID])
    #     mujoco.mju_mulQuat(errq, target_quat, negq)
    #     mujoco.mju_quat2Vel(e_rot, errq, 1.0)
    #     return e_pos, e_rot


    # def ik_velocity(target_pos, target_quat, w_rot=0.5, damping=1e-2, kp=3.0):
    #     """Damped least squares:  qdot = J^T (J J^T + lam^2 I)^-1 * kp * e"""
    #     mujoco.mj_jacBody(mj_model, mj_data, _jacp, _jacr, EE_BID)

    #     J = np.vstack([_jacp[:, CHAIN_DOF], w_rot * _jacr[:, CHAIN_DOF]])  # 6 x N
    #     e_pos, e_rot = pose_error(target_pos, target_quat)
    #     e = np.concatenate([e_pos, w_rot * e_rot])

    #     qdot = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), kp * e)
    #     return qdot, np.linalg.norm(e_pos), np.linalg.norm(e_rot)


    # def clamp_to_limits(q):
    #     for k, jid in enumerate(CHAIN_JIDS):
    #         if mj_model.jnt_limited[jid]:
    #             lo, hi = mj_model.jnt_range[jid]
    #             q[k] = np.clip(q[k], lo, hi)
    #     return q


    # # ----------------------------------------------------------------------------
    # # Controller
    # # ----------------------------------------------------------------------------
    # QDOT_MAX = 2.0          # rad/s, per joint
    # POS_TOL = 0.005         # 5 mm
    # ROT_TOL = 0.10          # rad

    # mujoco.mj_forward(mj_model, mj_data)
    # q_des = mj_data.qpos[CHAIN_QPOS].copy()

    # frames = []
    # renderer = mujoco.Renderer(mj_model, height=480, width=640)


    # def _record():
    #     if len(frames) < mj_data.time * FPS:
    #         renderer.update_scene(mj_data)
    #         frames.append(renderer.render())


    # def move_to(target_pos, target_quat, timeout=3.0, hold=None):
    #     """Servo the end-effector to a pose. `hold` optionally sets other ctrl
    #     entries (e.g. finger targets) each step. Returns True if converged."""
    #     global q_des
    #     dt = mj_model.opt.timestep
    #     t_end = mj_data.time + timeout

    #     while mj_data.time < t_end:
    #         qdot, e_pos, e_rot = ik_velocity(target_pos, target_quat)
    #         qdot = np.clip(qdot, -QDOT_MAX, QDOT_MAX)

    #         q_des = clamp_to_limits(q_des + qdot * dt)
    #         mj_data.ctrl[CHAIN_ACT] = q_des
    #         if hold is not None:
    #             for act_id, val in hold.items():
    #                 mj_data.ctrl[act_id] = val

    #         mujoco.mj_step(mj_model, mj_data)
    #         _record()

    #         if e_pos < POS_TOL and e_rot < ROT_TOL:
    #             return True
    #     return False


    # # ----------------------------------------------------------------------------
    # # Example: a two-waypoint reach. This is the skeleton your pick-and-place
    # # state machine slots into -- each phase is one move_to() plus a finger command.
    # # ----------------------------------------------------------------------------
    # list_candidates()   # comment out once EE_BODY is confirmed

    # cube_bid = name_id(mujoco.mjtObj.mjOBJ_BODY, "cube")
    # cube_pos = mj_data.xpos[cube_bid].copy()

    # # Palm pointing down at the cube. Tune this quaternion for your hand's
    # # actual frame convention -- print mj_data.xquat[EE_BID] at rest to see it.
    # down = np.array([0.0, 1.0, 0.0, 0.0])

    # ok = move_to(cube_pos + np.array([0.0, 0.0, 0.15]), down)   # pre-grasp
    # print("pre-grasp:", ok)

    # ok = move_to(cube_pos + np.array([0.0, 0.0, 0.02]), down)   # descend
    # print("descend:  ", ok)

    # imageio.mimsave("ik_reach.mp4", frames, fps=FPS)
    return


@app.cell
def _(imageio, mujoco, np):
    """
    Differential inverse kinematics for the Unitree G1 left arm in MuJoCo.

    Replaces hand-tuned joint targets with Cartesian pose commands:
        move_to(target_pos, target_quat)  ->  the solver figures out the joints.

    Uses damped least squares (DLS) on a point Jacobian. This is the same problem
    mink solves as a QP; this version is dependency-free so you can see the
    mechanism.

    Expects these from upstream marimo cells:
        np, mujoco, imageio, mj_model, mj_data, FPS
    """

    ROBOT = "unitree_g1"
    ROBOT_SCENE = "../unitree_mujoco/unitree_robots/" + ROBOT + "/scene_with_hands.xml" # Robot scene

    FPS = 30

    # compiled physics model of robot environment (static) (read-only reference)
    # mass, geometry, joints, gravity
    mj_model = mujoco.MjModel.from_xml_path(ROBOT_SCENE)

    # live dynamic states of environment and robot
    mj_data = mujoco.MjData(mj_model)


    def name_id(objtype, name):
        i = mujoco.mj_name2id(mj_model, objtype, name)
        if i < 0:
            raise ValueError(f"no {objtype} named {name!r}")
        return i


    def list_candidates(substrings=("hand", "wrist", "palm")):
        """Print body/site names so you can pick the right end-effector frame."""
        for objtype, label, count in (
            (mujoco.mjtObj.mjOBJ_BODY, "body", mj_model.nbody),
            (mujoco.mjtObj.mjOBJ_SITE, "site", mj_model.nsite),
        ):
            for i in range(count):
                n = mujoco.mj_id2name(mj_model, objtype, i)
                if n and any(s in n for s in substrings):
                    print(f"{label:5s} {i:3d}  {n}")


    # ----------------------------------------------------------------------------
    # IK chain
    # ----------------------------------------------------------------------------
    # waist_pitch is included deliberately: it lets the solver lean the torso in
    # for reach instead of you pinning it at a guessed constant.
    CHAIN_JOINTS = [
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ]

    CHAIN_JIDS = [name_id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in CHAIN_JOINTS]
    CHAIN_ACT = np.array([name_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in CHAIN_JOINTS])

    # NOTE: two different index spaces. dofadr indexes Jacobian columns (length nv),
    # qposadr indexes joint angles (length nq). They diverge because the pelvis
    # freejoint is 7 qpos but 6 dof. Using qposadr on the Jacobian is a silent bug.
    CHAIN_DOF = np.array([mj_model.jnt_dofadr[j] for j in CHAIN_JIDS])
    CHAIN_QPOS = np.array([mj_model.jnt_qposadr[j] for j in CHAIN_JIDS])

    N = len(CHAIN_JOINTS)

    # ----------------------------------------------------------------------------
    # End-effector frame
    # ----------------------------------------------------------------------------
    # This model has no palm body -- the Dex3 finger links hang directly off
    # left_wrist_yaw_link. So we use the wrist as the kinematic frame and carry a
    # fixed offset out to the point where the fingers actually close. mj_jac (as
    # opposed to mj_jacBody) computes the Jacobian at an arbitrary point attached
    # to a body, which is exactly what that needs.
    EE_BODY = "left_wrist_yaw_link"
    EE_BID = name_id(mujoco.mjtObj.mjOBJ_BODY, EE_BODY)

    # Calibrate the grasp point from the model rather than guessing it: take the
    # centroid of the three fingertip links and express it in the wrist frame.
    _TIP_BIDS = [
        name_id(mujoco.mjtObj.mjOBJ_BODY, n)
        for n in ("left_hand_thumb_2_link", "left_hand_index_1_link",
                  "left_hand_middle_1_link")
    ]

    mujoco.mj_forward(mj_model, mj_data)

    _centroid = np.mean([mj_data.xpos[b] for b in _TIP_BIDS], axis=0)
    _R_rest = mj_data.xmat[EE_BID].reshape(3, 3)
    GRASP_OFFSET = _R_rest.T @ (_centroid - mj_data.xpos[EE_BID])

    # The fingertip *link origins* sit at their joints, not at the actual pad
    # surface, so the centroid lands a bit shallow. Push it further along its own
    # direction; tune this by eye in the viewer.
    GRASP_EXTRA = 0.02  # metres
    _n = np.linalg.norm(GRASP_OFFSET)
    if _n > 1e-9:
        GRASP_OFFSET = GRASP_OFFSET * (1.0 + GRASP_EXTRA / _n)

    print("grasp offset (wrist frame):", GRASP_OFFSET, " |offset| =", np.linalg.norm(GRASP_OFFSET))
    print("wrist quat at rest:        ", mj_data.xquat[EE_BID])

    _jacp = np.zeros((3, mj_model.nv))
    _jacr = np.zeros((3, mj_model.nv))


    def grasp_point():
        """World-frame position of the grasp point. Recomputed every call because
        the offset is fixed in the hand frame but moves as the arm swings."""
        R = mj_data.xmat[EE_BID].reshape(3, 3)
        return mj_data.xpos[EE_BID] + R @ GRASP_OFFSET


    # ----------------------------------------------------------------------------
    # Solver
    # ----------------------------------------------------------------------------
    def pose_error(target_pos, target_quat):
        """Cartesian error as (3,) translation + (3,) rotation vector."""
        e_pos = target_pos - grasp_point()

        negq = np.zeros(4)
        errq = np.zeros(4)
        e_rot = np.zeros(3)
        mujoco.mju_negQuat(negq, mj_data.xquat[EE_BID])
        mujoco.mju_mulQuat(errq, target_quat, negq)
        mujoco.mju_quat2Vel(e_rot, errq, 1.0)
        return e_pos, e_rot


    def ik_velocity(target_pos, target_quat, w_rot=0.5, damping=1e-2, kp=3.0):
        """Damped least squares:  qdot = J^T (J J^T + lam^2 I)^-1 * kp * e

        Set w_rot=0.0 to solve position only -- useful for isolating whether a
        failure is in the IK itself or in your target orientation.
        """
        mujoco.mj_jac(mj_model, mj_data, _jacp, _jacr, grasp_point(), EE_BID)

        J = np.vstack([_jacp[:, CHAIN_DOF], w_rot * _jacr[:, CHAIN_DOF]])  # 6 x N
        e_pos, e_rot = pose_error(target_pos, target_quat)
        e = np.concatenate([e_pos, w_rot * e_rot])

        qdot = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), kp * e)
        return qdot, np.linalg.norm(e_pos), np.linalg.norm(e_rot)


    def clamp_to_limits(q):
        for k, jid in enumerate(CHAIN_JIDS):
            if mj_model.jnt_limited[jid]:
                lo, hi = mj_model.jnt_range[jid]
                q[k] = np.clip(q[k], lo, hi)
        return q

    # grip
    HAND_JOINTS = [
        "left_hand_thumb_0_joint", 
        "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", "left_hand_middle_1_joint",
        "left_hand_index_0_joint", "left_hand_index_1_joint",
    ]
    HAND_ACT  = np.array([name_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in HAND_JOINTS])
    HAND_JIDS = [name_id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in HAND_JOINTS]

    LO = mj_model.jnt_range[HAND_JIDS, 0]
    HI = mj_model.jnt_range[HAND_JIDS, 1]

    # Which way each joint curls. Flip an entry to -1 if that finger opens when it
    # should close. This is the one thing you'll likely need to tune.
    CLOSE_DIR = np.ones(len(HAND_JOINTS))

    OPEN_POSE   = np.where(CLOSE_DIR > 0, LO, HI)
    CLOSED_POSE = np.where(CLOSE_DIR > 0, HI, LO)

    finger_coef = 5.0
    coefs = np.asarray([1, 1, 1, finger_coef, finger_coef, finger_coef, finger_coef])


    def set_fingers(angles, duration=0.8):
        """Move the finger joints to `angles` (7 values, radians) over `duration`
        seconds, ramping smoothly. Holds the arm still meanwhile."""
        angles = np.asarray(angles, dtype=float)
        start = mj_data.ctrl[HAND_ACT].copy()
        dt = mj_model.opt.timestep
        n = max(1, int(duration / dt))

        for i in range(n):
            alpha = (i + 1) / n
            mj_data.ctrl[HAND_ACT] = start + coefs * alpha * (angles - start)
            mj_data.ctrl[CHAIN_ACT] = q_des          # keep the arm where it is
            mujoco.mj_step(mj_model, mj_data)
            _record()


    def grip(fraction, duration=0.8):
        """fraction=0 fully open, 1 fully closed. Try 0.6-0.8 for a cube."""
        set_fingers(OPEN_POSE + fraction * (CLOSED_POSE - OPEN_POSE), duration)


    # ----------------------------------------------------------------------------
    # Controller
    # ----------------------------------------------------------------------------
    QDOT_MAX = 2.0          # rad/s, per joint
    POS_TOL = 0.005         # 5 mm
    ROT_TOL = 0.10          # rad

    q_des = mj_data.qpos[CHAIN_QPOS].copy()

    frames = []
    renderer = mujoco.Renderer(mj_model, height=480, width=640)


    def _record():
        if len(frames) < mj_data.time * FPS:
            renderer.update_scene(mj_data)
            frames.append(renderer.render())


    def move_to(target_pos, target_quat, timeout=3.0, hold=None, w_rot=0.5):
        """Servo the grasp point to a pose. `hold` optionally sets other ctrl
        entries (e.g. finger targets) each step. Returns True if converged."""
        global q_des
        dt = mj_model.opt.timestep
        t_end = mj_data.time + timeout

        while mj_data.time < t_end:
            qdot, e_pos, e_rot = ik_velocity(target_pos, target_quat, w_rot=w_rot)
            qdot = np.clip(qdot, -QDOT_MAX, QDOT_MAX)

            q_des = clamp_to_limits(q_des + qdot * dt)
            mj_data.ctrl[CHAIN_ACT] = q_des
            if hold is not None:
                for act_id, val in hold.items():
                    mj_data.ctrl[act_id] = val

            mujoco.mj_step(mj_model, mj_data)
            _record()

            if e_pos < POS_TOL and (w_rot == 0.0 or e_rot < ROT_TOL):
                return True
        return False


    # ----------------------------------------------------------------------------
    # Example: a two-waypoint reach. This is the skeleton your pick-and-place
    # state machine slots into -- each phase is one move_to() plus a finger command.
    # ----------------------------------------------------------------------------
    cube_bid = name_id(mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_pos = mj_data.xpos[cube_bid].copy()

    # Replace this with the real thing: pose the wrist by hand in the viewer until
    # the fingers point down at the table, then read mj_data.xquat[EE_BID] off and
    # paste it here. The printout above gives you the rest pose for reference.
    down = np.array([0.0, 1.0, 0.0, 0.0])

    # Start position-only (w_rot=0.0) to confirm the arm reaches the right point at
    # all, then turn orientation back on once that works.
    W_ROT = 0.0

    ok = move_to(cube_pos + np.array([0.1, 0.0, 0.15]), down, w_rot=W_ROT)
    print("pre-grasp:", ok)

    ok = move_to(cube_pos + np.array([0.05, 0.02, 0.00]), down, w_rot=W_ROT)
    print("descend:  ", ok)

    grip(0.9)
    print("grip: check video")

    imageio.mimsave("ik_reach_v1.mp4", frames, fps=FPS)
    return HAND_JOINTS, HI, LO


@app.cell
def _(HAND_JOINTS, HI, LO):
    for n, lo, hi in zip(HAND_JOINTS, LO, HI):
        print(f"{n:32s} [{lo:+.3f}, {hi:+.3f}]  zero at {(0-lo)/(hi-lo)*100:5.1f}% of range")
    return


if __name__ == "__main__":
    app.run()
