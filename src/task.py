from dataclasses import dataclass, field
import numpy as np
import mujoco
from src.assembly import name_id
from src.reach import sample_positions, place_object


COLORS = {
    "red":    (0.85, 0.15, 0.15, 1.0),
    "yellow": (0.90, 0.80, 0.10, 1.0),
    "green":  (0.15, 0.70, 0.25, 1.0),
    "blue":   (0.15, 0.35, 0.85, 1.0),
    "purple": (0.55, 0.20, 0.75, 1.0),
    "orange": (0.95, 0.50, 0.10, 1.0),
}


def set_object_color(m, body_name, rgba):
    """Recolor every geom on a body."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        raise ValueError(f"no body named {body_name!r}")
    start = m.body_geomadr[bid]
    for g in range(start, start + m.body_geomnum[bid]):
        m.geom_rgba[g] = rgba

def task_move_cylinder(m, d, home, ws_config, rng=None):
    cyl_xy, bowl_xy = sample_positions(ws_config, 2, rng=rng)
    rng = rng or np.random.default_rng()
    color = rng.choice(list(COLORS))
    set_object_color(m, "cylinder", COLORS[color])
    set_object_color(m, "bowl",     COLORS[color])

    place_object(m, d, "cylinder", [*cyl_xy,  ws_config.table_z + 0.05])
    place_object(m, d, "bowl",     [*bowl_xy, ws_config.table_z])
    mujoco.mj_forward(m, d)                # so xpos reflects the new placement

    cylinder_pos = d.xpos[name_id(m, mujoco.mjtObj.mjOBJ_BODY, "cylinder")].copy()
    bowl_pos     = d.xpos[name_id(m, mujoco.mjtObj.mjOBJ_BODY, "bowl")].copy()
    approach = np.array([0.0, 0.0, 0.15])

    return [
        ("LeftArm",  "move", cylinder_pos + approach - home),
        ("LeftArm",  "move", cylinder_pos - home),
        ("LeftHand", "grip", 0.5),
        ("LeftArm",  "move", cylinder_pos + approach - home),
        ("LeftArm",  "move", bowl_pos + approach - home),
        ("LeftHand", "grip", 0.0),
    ]

# def task_move_cylinder(m, d, home):
#     """
#     Template for moving cylinder to bowl.
#     Object positions are converted to relative coordinates by subtracting home.
#     """

#     # place objects in spots reachable by robot
#     # robot doesn't move, keep z or height constant

#     cylinder_id = name_id(m, mujoco.mjtObj.mjOBJ_BODY, "cylinder")
#     cylinder_pos = d.xpos[cylinder_id].copy()
#     bowl_id = name_id(m, mujoco.mjtObj.mjOBJ_BODY, "bowl")
#     bowl_pos = d.xpos[bowl_id].copy()

#     return [
#         ("LeftArm", "move", cylinder_pos - home),
#         ("LeftHand", "grip", 0.5), # close grip
#         ("LeftArm", "move", bowl_pos - home + np.asarray([0.0, 0.0, 0.2])),
#         ("LeftHand", "grip", 0.0) # open grip
#     ]
