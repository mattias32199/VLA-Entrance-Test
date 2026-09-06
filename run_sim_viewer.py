# ./run_sim.py
import time
import numpy as np
import imageio
import mujoco
import mujoco.viewer
from src.assembly import assemble_robot
from src.sequencer import Sequencer
from src.util import get_markers, draw_markers, print_contacts
from src.task import task_move_cylinder
from config import SimulationConfig


def simulation_thread(m, d, config):
    """Simulation / viewer thread."""
    # Initialize simulation variables
    robot = assemble_robot(m, d, config)
    left_arm = robot["LeftArm"]
    home = d.xpos[left_arm["ee_bid"]].copy()
    down = d.xquat[left_arm["ee_bid"]].copy()

    actions, meta = task_move_cylinder(m, d, home, config.ws_config)
    sequencer = Sequencer(robot, actions, home, down, hold_when_done=config.hold_when_done)

    # get markers
    if config.draw_markers:
        markers = get_markers(actions, home)

    # launch viewer
    with mujoco.viewer.launch_passive(
        m, d, show_left_ui=config.ui_config.show_left_ui, show_right_ui=config.ui_config.show_right_ui
    ) as v:
        sync_period = 1.0 / config.ui_config.sync_hz    # e.g. 60 -> 0.0167 s
        next_sync = d.time
        sim_t0, wall_t0 = d.time, time.perf_counter()

        step_count = 0
        while v.is_running():
            # hold(d, left_arm)
            # step robot simulation
            sequencer.step(m, d)
            mujoco.mj_step(m, d)

            # if step_count % 250 == 0:
            #     print_contacts(m, d)
            # step_count += 1
            if d.time >= next_sync:
                if config.draw_markers:
                    ee_pos = d.xpos[left_arm["ee_bid"]] + config.arm_config.end_effector_offset
                    draw_markers(v, markers + [(ee_pos, (0.1, 0.9, 0.3, 0.9), 0.015)])
                v.sync()
                next_sync = max(next_sync + sync_period, d.time)
                lag = (d.time - sim_t0) - (time.perf_counter() - wall_t0)
                if lag > 0:
                    time.sleep(lag)


def main():
    config = SimulationConfig()
    m = mujoco.MjModel.from_xml_path(config.robot_scene)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    print("ncam =", m.ncam)
    for i in range(m.ncam):
        print(" ", i, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i))
    r = mujoco.Renderer(m, height=224, width=224)
    for cam in ("tracking", "head_cam", "left_wrist_cam"):
        r.update_scene(d, camera=cam)
        imageio.imwrite(f"./outputs/cam_{cam}.png", r.render())

    simulation_thread(m, d, config) # launch simulation + viewer
    print("Safely Killed...")


if __name__ == "__main__":
    main()
