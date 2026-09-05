# ./src/manual_actions.py
import numpy as np
import mujoco

def move_to_ik(m, d, arm, target_pos, target_quat=None,
               kp=3.0, damping=1e-2, qdot_max=2.0, w_rot=0.5):
    """One IK step. Writes d.ctrl, returns (pos_err, rot_err) in m and rad."""
    R = d.xmat[arm["ee_bid"]].reshape(3, 3)
    point = d.xpos[arm["ee_bid"]] + R @ arm["grasp_offset"]
    e_pos = np.asarray(target_pos, float) - point

    if target_quat is None:                      # position-only, as before
        target_quat, w_rot = d.xquat[arm["ee_bid"]].copy(), 0.0

    negq, errq, e_rot = np.zeros(4), np.zeros(4), np.zeros(3)
    mujoco.mju_negQuat(negq, d.xquat[arm["ee_bid"]])
    mujoco.mju_mulQuat(errq, np.asarray(target_quat, float), negq)
    mujoco.mju_quat2Vel(e_rot, errq, 1.0)

    mujoco.mj_jac(m, d, arm["jacp"], arm["jacr"], point, arm["ee_bid"])
    J = np.vstack([arm["jacp"][:, arm["dof"]],
                   w_rot * arm["jacr"][:, arm["dof"]]])          # 6 x N
    e = np.concatenate([e_pos, w_rot * e_rot])

    qdot = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), kp * e)

    q = arm["q_des"] + np.clip(qdot, -qdot_max, qdot_max) * m.opt.timestep
    arm["q_des"] = np.where(arm["limited"], np.clip(q, arm["lo"], arm["hi"]), q)
    d.ctrl[arm["act"]] = arm["q_des"]
    return float(np.linalg.norm(e_pos)), float(np.linalg.norm(e_rot))



def hold(d, part):
    """Re-issue the existing command. Safe to call every step."""
    d.ctrl[part["act"]] = part["q_des"]


def freeze(d, part):
    """Re-seed the command from the measured pose. Call ONCE, on a transition."""
    part["q_des"] = d.qpos[part["qpos"]].copy()
    d.ctrl[part["act"]] = part["q_des"]


def set_fingers(m, d, hand, angles, duration=0.8):
    """One step of a finger ramp. Returns progress 0..1; 1.0 means finished."""
    angles = np.asarray(angles, dtype=float)

    # (re)start the ramp if this is a new destination
    if hand["ramp_to"] is None or not np.allclose(hand["ramp_to"], angles):
        hand["ramp_from"] = hand["q_des"].copy()
        hand["ramp_to"] = angles
        hand["ramp_i"] = 0
        hand["ramp_n"] = max(1, int(duration / m.opt.timestep))

    hand["ramp_i"] = min(hand["ramp_i"] + 1, hand["ramp_n"])
    alpha = hand["ramp_i"] / hand["ramp_n"]

    hand["q_des"] = (hand["ramp_from"]
                     + hand["finger_coefs"] * alpha * (angles - hand["ramp_from"]))
    d.ctrl[hand["act"]] = hand["q_des"]
    return alpha


def grip(m, d, hand, fraction, duration=0.8):
    """fraction=0 fully open, 1 fully closed. Try 0.6-0.9 for a cube."""
    target = hand["open_pose"] + fraction * (hand["closed_pose"] - hand["open_pose"])
    return set_fingers(m, d, hand, target, duration)
