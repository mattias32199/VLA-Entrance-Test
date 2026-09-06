# ./run_sim_headless.py
import numpy as np
from pathlib import Path
import json
import mujoco
from src.assembly import assemble_robot
from src.sequencer import Sequencer
from src.task import task_move_cylinder, body_pos, in_bowl
from config import SimulationConfig

CAMERAS = ("head_cam", "left_wrist_cam")


def save_episode(log, meta, out_dir="./outputs/data"):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stem = f"ep_{meta['seed']:04d}"

    np.savez_compressed(out / f"{stem}.npz",
                        **{k: np.asarray(v) for k, v in log.items()})
    with open(out / f"{stem}.json", "w") as f:
        json.dump(meta, f, indent=2)

    return out / f"{stem}.npz"


def run_episode(m, d, config, renderer, seed):
    rng = np.random.default_rng(seed)
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)

    robot = assemble_robot(m, d, config)
    arm = robot["LeftArm"]
    home = d.xpos[arm["ee_bid"]].copy()
    down = d.xquat[arm["ee_bid"]].copy()

    actions, task_meta = task_move_cylinder(m, d, home, config.ws_config, rng=rng)
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
    meta = {
        "seed": int(seed),
        "success": in_bowl(m, d),
        "steps": int(step),
        "frames": len(log["t"]),
        "hz": round(1.0 / (m.opt.timestep * 25), 2),
        "cylinder_pos_final": body_pos(m, d, "cylinder").tolist(),
        "bowl_pos_final": body_pos(m, d, "bowl").tolist(),
        **task_meta,
    }
    return log, meta


# def main():
#     config = SimulationConfig()
#     m = mujoco.MjModel.from_xml_path(config.robot_scene)
#     d = mujoco.MjData(m)
#     renderer = mujoco.Renderer(m, height=224, width=224)

#     log, meta = run_episode(m, d, config, renderer, seed=0)
#     save_episode(log, meta)
#     # np.savez_compressed("./outputs/data/ep_0000.npz", **{k: np.array(v) for k, v in log.items()})
#     # print("frames:", len(log["t"]))

def main():
    config = SimulationConfig()
    m = mujoco.MjModel.from_xml_path(config.robot_scene)
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, height=224, width=224)
    # m.vis.map.znear = 0.001

    results = []
    for seed in range(6):
        log, meta = run_episode(m, d, config, renderer, seed)
        save_episode(log, meta)
        results.append(meta)
        print(f"seed={seed:3d} {'OK ' if meta['success'] else 'FAIL'} "
              f"frames={meta['frames']:4d} steps={meta['steps']}")

    ok = sum(r["success"] for r in results)
    print(f"\n{ok}/{len(results)} succeeded ({100*ok/len(results):.0f}%)")


if __name__ == "__main__":
    main()
