import numpy as np
import mujoco

def get_markers(actions, home):
    """Builds markers using actions"""
    markers = [(home, (0.9, 0.9, 0.2, 0.4), 0.02)]
    for action in actions:
        if action[1] != "move":
            continue
        markers.append((home + action[2], (0.9, 0.2, 0.2, 0.5), 0.025))
    return markers

def draw_markers(v, markers):
    """markers: list of (pos, rgba, size). Rebuilds the scratch scene."""
    scn = v.user_scn
    scn.ngeom = 0
    for pos, rgba, size in markers:
        if scn.ngeom >= len(scn.geoms):
            break
        mujoco.mjv_initGeom(
            scn.geoms[scn.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([size, size, size]),
            np.asarray(pos, dtype=np.float64),
            np.eye(3).flatten(),
            np.asarray(rgba, dtype=np.float32),
        )
        scn.ngeom += 1


# def print_contacts(m, d, body_name="cube"):
#     bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
#     print(f"t={d.time:6.2f}  ncon={d.ncon}  z={d.xpos[bid][2]:.4f}")
#     for i in range(d.ncon):
#         c = d.contact[i]
#         g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
#         g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
#         # only contacts involving the object


def print_contacts(m, d, body_name="cube"):
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    print(f"t={d.time:6.2f} ncon={d.ncon} z={d.xpos[bid][2]:.4f}")
    for i in range(d.ncon):
        c = d.contact[i]
        n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1])
        n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2])
        print(f"   [{i}] {n1} <-> {n2}  dist={c.dist:+.6f}")
