# ./run_sim_headless.py
import numpy as np, mujoco
from src.assembly import assemble_robot
from src.sequencer import Sequencer
from src.task import task_move_cylinder
from config import SimulationConfig

CAMERAS = ("head_cam", "left_wrist_cam")


def run_episode(m, d, config, renderer, seed):
    rng = np.random.default_rng(seed)
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)

    robot = assemble_robot(m, d, config)
    arm = robot["LeftArm"]
    home = d.xpos[arm["ee_bid"]].copy()
    down = d.xquat[arm["ee_bid"]].copy()

    actions = task_move_cylinder(m, d, home, config.ws_config, rng=rng)
    seq = Sequencer(robot, actions, home, down, hold_when_done=True)

    log = {c: [] for c in CAMERAS}
    log.update(qpos=[], ee_pos=[], t=[])

    step = 0
    while not seq.done and step < 40_000:
        seq.step(m, d)
        mujoco.mj_step(m, d)
        if step % 25 == 0:
            for c in CAMERAS:
                renderer.update_scene(d, camera=c)
                log[c].append(renderer.render())
            R = d.xmat[arm["ee_bid"]].reshape(3, 3)
            log["ee_pos"].append((d.xpos[arm["ee_bid"]] + R @ arm["grasp_offset"]).copy())
            log["qpos"].append(d.qpos[arm["qpos"]].copy())
            log["t"].append(d.time)
        step += 1

    return log


def main():
    config = SimulationConfig()
    m = mujoco.MjModel.from_xml_path(config.robot_scene)
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, height=224, width=224)

    log = run_episode(m, d, config, renderer, seed=0)
    np.savez_compressed("./outputs/data/ep_0000.npz", **{k: np.array(v) for k, v in log.items()})
    print("frames:", len(log["t"]))


if __name__ == "__main__":
    main()
