# ./src/episode.py
import numpy as np
import mujoco
from src.assembly import assemble_robot
from src.task import task_move_cylinder
from src.sequencer import Sequencer

def run_episode(m, d, config, seed, record_every=25):
    rng = np.random.default_rng(seed)
    robot = assemble_robot(m, d, config)
    left_arm = robot["LeftArm"]
    home = d.xpos[left_arm["ee_bid"]].copy()
    down = d.xquat[left_arm["ee_bid"]].copy()

    actions, meta = task_move_cylinder(m, d, home, config.ws_config, rng=rng)
    seq = Sequencer(robot, actions, home, down=down)

    r = mujoco.Renderer(m, height=224, width=224)
    log = {k: [] for k in ("head", "wrist", "qpos", "hand_qpos", "ee_pos", "ee_quat", "t")}

    step = 0
    while not seq.done and step < 40_000:
        seq.step(m, d)
        mujoco.mj_step(m, d)
        if step % record_every == 0:
            record(r, m, d, robot, log)
        step += 1

    r.close()
    return log, meta, in_bowl(m, d), step


def record(r, m, d, robot, log):
    arm = robot["LeftArm"]
    for cam, key in (("head_cam", "head"), ("left_wrist_cam", "wrist")):
        r.update_scene(d, camera=cam)
        log[key].append(r.render())
    log["qpos"].append(d.qpos[arm["qpos"]].copy())
    log["hand_qpos"].append(d.qpos[robot["LeftHand"]["qpos"]].copy())
    R = d.xmat[arm["ee_bid"]].reshape(3, 3)
    log["ee_pos"].append((d.xpos[arm["ee_bid"]] + R @ arm["grasp_offset"]).copy())
    log["ee_quat"].append(d.xquat[arm["ee_bid"]].copy())
    log["t"].append(d.time)


def in_bowl(m, d, obj="cylinder", bowl="bowl", xy_tol=0.05, max_h=0.12):
    """True when the object is resting inside the bowl's footprint."""
    ob = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, obj)
    bb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bowl)
    o, b = d.xpos[ob], d.xpos[bb]
    over  = np.linalg.norm(o[:2] - b[:2]) < xy_tol
    above = 0.0 < (o[2] - b[2]) < max_h
    # not still moving / still in the hand
    adr = m.jnt_dofadr[m.body_jntadr[ob]]
    at_rest = np.linalg.norm(d.qvel[adr:adr + 3]) < 0.01
    return bool(over and above and at_rest)
