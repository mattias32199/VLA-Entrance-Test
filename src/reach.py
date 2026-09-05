import numpy as np
import mujoco


def sample_positions(ws, n, rng=None, tries=200):
    """n reachable (x, y) points in the workspace box, mutually separated."""
    rng = rng or np.random.default_rng()
    pts = []
    for _ in range(n):
        for _ in range(tries):
            xy = np.array([rng.uniform(*ws.x), rng.uniform(*ws.y)])
            if all(np.linalg.norm(xy - p) >= ws.min_sep for p in pts):
                pts.append(xy)
                break
        else:
            raise RuntimeError(f"could not place {n} objects with min_sep={ws.min_sep}")
    return pts


def place_object(m, d, body_name, pos, quat=(1.0, 0.0, 0.0, 0.0)):
    """Write a freejoint body's pose into qpos."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        raise ValueError(f"no body named {body_name!r}")
    jid = m.body_jntadr[bid]
    if jid < 0 or m.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError(f"body {body_name!r} has no freejoint")
    adr = m.jnt_qposadr[jid]
    d.qpos[adr:adr + 3] = pos
    d.qpos[adr + 3:adr + 7] = quat
