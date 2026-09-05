# ./src/assembly.py
import numpy as np
import mujoco


def assemble_robot(m, d, config):
    robot = {}
    arm_config = config.arm_config
    robot["LeftArm"] = assemble_arm(
        m, d, arm_config.chain_joints, arm_config.end_effector, np.asarray(arm_config.end_effector_offset)
    )
    hand_config = config.hand_config
    robot["LeftHand"] = assemble_hand(
        m, d, hand_config.hand_joints, close_dir=hand_config.close_dir, finger_coefs=hand_config.finger_coefs
    )
    return robot


def assemble_arm(m, d, chain_joints, ee_body, grasp_offset=None):
    """
    chain_joints: list of joints
    ee_body: end effector body
    """
    jid = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in chain_joints])
    act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in chain_joints])
    lo, hi = m.jnt_range[jid].T
    return {
        "jid": jid,
        "act": act,
        "dof":  m.jnt_dofadr[jid],      # Jacobian columns (nv space)
        "qpos": m.jnt_qposadr[jid],     # joint angles     (nq space)
        "ee_bid": mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ee_body),
        "lo": lo, "hi": hi,
        "limited": m.jnt_limited[jid].astype(bool),
        "grasp_offset": np.zeros(3) if grasp_offset is None else grasp_offset,
        "q_des": d.qpos[m.jnt_qposadr[jid]].copy(),
        "jacp": np.zeros((3, m.nv)),
        "jacr": np.zeros((3, m.nv)),
    }


def assemble_hand(m, d, hand_joints, close_dir=None, finger_coefs=None):
    jid = np.array([name_id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in hand_joints])
    act = np.array([name_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in hand_joints])
    lo, hi = m.jnt_range[jid].T

    if close_dir is None:
        close_dir = np.ones(len(hand_joints))
    if finger_coefs is None:
        finger_coefs = np.ones(len(hand_joints))

    return {
        "jid": jid, "act": act,
        "qpos": m.jnt_qposadr[jid],
        "open_pose":   np.where(close_dir > 0, lo, hi),
        "closed_pose": np.where(close_dir > 0, hi, lo),
        "finger_coefs": np.asarray(finger_coefs, dtype=float),
        "q_des": d.qpos[m.jnt_qposadr[jid]].copy(),
        # ramp state, owned by set_fingers
        "ramp_from": None,
        "ramp_to": None,
        "ramp_i": 0,
        "ramp_n": 0,
    }


def name_id(m, objtype, name):
    i = mujoco.mj_name2id(m, objtype, name)
    if i < 0:
        raise ValueError(f"no {objtype} named {name!r}")
    return i
