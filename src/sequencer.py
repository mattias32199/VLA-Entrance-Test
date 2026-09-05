# ./src/sequencer.py
import numpy as np
from src.manual_actions import move_to_ik, hold, freeze, grip

class Sequencer:
    def __init__(
        self, robot, actions, home, down, err_thr=0.005, rot_thr=0.005, max_steps=3000, hold_when_done=True
    ):
        self.robot, self.actions, self.home, self.down = robot, actions, home, down
        self.err_thr, self.rot_thr, self.max_steps = err_thr, rot_thr, max_steps
        self.hold_when_done = hold_when_done
        self.idx, self.steps = 0, 0

    @property
    def done(self):
        return self.idx >= len(self.actions)

    def act(self, m, d, part_name, action_type, arg):
        part = self.robot[part_name]
        if action_type == "move":
            e_pos, e_rot = move_to_ik(m, d, part,
                                      self.home + np.array(arg, dtype=float),
                                      target_quat=self.down)
            finished = e_pos < self.err_thr and e_rot < self.rot_thr
            detail = f"pos={e_pos:.4f} rot={e_rot:.3f}"
        elif action_type == "grip":
            progress = grip(m, d, part, arg)
            finished = progress >= 1.0
            detail = f"progress={progress:.2f}"
        else:
            raise ValueError(f"unknown action kind {action_type!r}")
        return finished, detail

    def step(self, m, d):
        # early exit
        if self.done:
            if self.hold_when_done:
                for part in self.robot.values():
                    hold(d, part)
            return

        part_name, action_type, arg = self.actions[self.idx]
        finished, detail = self.act(m, d, part_name, action_type, arg)

        # every part that isn't the active one keeps its last command
        for name, other in self.robot.items():
            if name != part_name:
                hold(d, other)

        self.steps += 1
        if finished or self.steps > self.max_steps:
            print(f"action {self.idx} ({part_name}/{action_type}): {detail} steps={self.steps}")
            self.idx, self.steps = self.idx + 1, 0
            if self.done:
                for part in self.robot.values():
                    freeze(d, part)
