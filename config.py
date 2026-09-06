from dataclasses import dataclass, field
import numpy as np

"""
SimulationConfig
|--- UIConfig
|--- ArmConfig
|--- HandConfig
|--- WorkspaceConfig
"""

@dataclass
class UIConfig:
    show_left_ui: bool = False # can hide panels for clean recording
    show_right_ui: bool = True
    sync_hz: int = 60

@dataclass
class ArmConfig:
    chain_joints: list[str] = field(default_factory=lambda: [
        "waist_yaw_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint",
        "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    ])
    end_effector: str = "left_wrist_yaw_link"
    end_effector_offset: np.ndarray | None = None
    def __post_init__(self):
        if self.end_effector_offset is None:
            self.end_effector_offset = np.array(
                [0.1, -0.04, 0.0]
            )

@dataclass
class HandConfig:
    hand_joints: list[str] = field(default_factory=lambda: [
        "left_hand_thumb_0_joint",
        "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", "left_hand_middle_1_joint",
        "left_hand_index_0_joint", "left_hand_index_1_joint",
    ])
    close_dir: np.ndarray | None = None
    finger_coef: float = 1.5
    thumb_coef: float=1.2
    finger_coefs: np.ndarray | None = None
    def __post_init__(self):
        if self.close_dir is None:
            #        thumb 0,1,2   middle 0,1   index 0,1
            self.close_dir = np.array([1, 1, 1, -1, -1, -1, -1], dtype=float)
        if self.finger_coefs is None:
            # thumb joints at 1.0, the rest overshoot by finger_coef
            self.finger_coefs = np.array(
                [1.0, 1.0, 1.0] + [self.finger_coef] * (len(self.hand_joints) - 3)
            )

@dataclass
class WorkspaceConfig:
    x: tuple[float, float] = (0.32, 0.45)
    y: tuple[float, float] = (0.02, 0.28)
    table_z: float = 0.72
    min_sep: float = 0.15

@dataclass
class SimulationConfig:
    ui_config: UIConfig = field(default_factory=UIConfig)
    arm_config: ArmConfig = field(default_factory=ArmConfig)
    hand_config: HandConfig = field(default_factory=HandConfig)
    ws_config: WorkspaceConfig = field(default_factory=WorkspaceConfig)
    robot: str = "unitree_g1"
    robot_scene: str = "./robots/" + robot + "/scene_with_hands.xml" # Robot scene
    makers: bool = True
    hold_when_done: bool = False
    draw_markers: bool = True
