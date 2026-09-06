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

TEMPLATES = [
    "put the {color} cylinder in the {color} bowl",
    "place the {color} cylinder into the {color} bowl",
    "pick up the {color} cylinder and drop it in the {color} bowl",
    "move the {color} cylinder to the {color} bowl",
]


def body_pos(m, d, name):
    return d.xpos[name_id(m, mujoco.mjtObj.mjOBJ_BODY, name)].copy()


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

    actions = [
        ("LeftArm",  "move", cylinder_pos + approach - home),
        ("LeftArm",  "move", cylinder_pos - home),
        ("LeftHand", "grip", 0.5),
        ("LeftArm",  "move", cylinder_pos + approach - home),
        ("LeftArm",  "move", bowl_pos + approach - home),
        ("LeftHand", "grip", 0.0),
    ]
    meta = {
        "color": str(color),
        "instruction": str(rng.choice(TEMPLATES)).format(color=color),
        "cylinder_pos_init": body_pos(m, d, "cylinder").tolist(),
        "bowl_pos_init": body_pos(m, d, "bowl").tolist(),
    }
    return actions, meta


def in_bowl(m, d, obj="cylinder", bowl="bowl", xy_tol=0.05, max_h=0.12):
    """True when the object is resting inside the bowl's footprint."""
    ob = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, obj)
    bb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bowl)
    o, b = d.xpos[ob], d.xpos[bb]
    over  = np.linalg.norm(o[:2] - b[:2]) < xy_tol
    above = 0.0 < (o[2] - b[2]) < max_h
    # not still moving / still in the hand
    adr = m.jnt_dofadr[m.body_jntadr[ob]]
    at_rest = np.linalg.norm(d.qvel[adr:adr + 3]) < 0.05
    return bool(over and above and at_rest)
