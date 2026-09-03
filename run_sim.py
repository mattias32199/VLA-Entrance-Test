import time
import mujoco
import mujoco.viewer
from config import SimulationConfig, UIConfig


def simulation_thread(m, d, config):
    with mujoco.viewer.launch_passive(
            m, d,
            show_left_ui=config.ui_config.show_left_ui,
            show_right_ui=config.ui_config.show_right_ui
    ) as v:
        sync_period = 1.0 / config.ui_config.sync_hz    # e.g. 60 -> 0.0167 s
        next_sync = d.time
        sim_t0, wall_t0 = d.time, time.perf_counter()

        while v.is_running():
            mujoco.mj_step(m, d)

            # actions

            if d.time >= next_sync:
                v.sync()
                next_sync += sync_period

                lag = (d.time - sim_t0) - (time.perf_counter() - wall_t0)
                if lag > 0:
                    time.sleep(lag)


def main():

    config = SimulationConfig()
    m = mujoco.MjModel.from_xml_path(config.robot_scene)
    d = mujoco.MjData(m)

    simulation_thread(m, d, config) # launch simulation + viewer
    print("Safely Killed...")


if __name__ == "__main__":

    main()
