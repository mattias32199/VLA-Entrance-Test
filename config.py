from dataclasses import dataclass, field

"""
STRUCTURE:
SimulationConfig
|--- UIConfig
"""

@dataclass
class UIConfig:
    show_left_ui: bool = True # can hide panels for clean recording
    show_right_ui: bool = True
    sync_hz: int = 60


@dataclass
class SimulationConfig:
    ui_config: UIConfig = field(default_factory=UIConfig)
    robot: str = "unitree_g1"
    robot_scene: str = "../unitree_mujoco/unitree_robots/" + robot + "/scene_with_hands.xml" # Robot scene
