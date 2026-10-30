"""Grip guard binding tests on simulated servos, independent of local calibration files."""

import unittest

from so101ik import Arm, Fault, GripLost, default_calibration
from so101ik import _core


class GripGuardTest(unittest.TestCase):
    def setUp(self):
        self.ticks = []
        self.raw = _core.Arm("sim", default_calibration(), on_tick=self.ticks.append)
        self.addCleanup(self.raw.close)
        self.addCleanup(self.raw.torque_off)
        self.raw.check()
        self.raw.torque_on()
        self.raw.unfold()
        # Exercise the public follow wrapper without searching for a machine's calibration.
        self.arm = Arm.__new__(Arm)
        self.arm._arm, self.arm._fast, self.arm._ran = self.raw, False, None
        self.arm.unfold_first = False
        self.raw.move_joints({"gripper": 45}, 90, False)
        self.raw.sim(block={"gripper": (30, None)})
        self.raw.grasp(40, 3, 2.5, 4)
        self.grip_goal = self.raw.goal["gripper"]
        self.ticks.clear()

    def chain(self):
        start = self.raw.goal
        return [start, dict(start, shoulder_pan=start["shoulder_pan"] + 2)]

    def test_guarded_carry_and_unguarded_release_remain_distinct(self):
        self.assertFalse(self.arm.follow(self.chain(), duration=.3, grip_range=(29, 31)))
        start = self.raw.goal
        self.assertFalse(self.arm.follow([start, dict(start, gripper=45)], duration=.3))
        self.assertAlmostEqual(self.raw.goal["gripper"], 45)

    def test_grip_loss_stops_without_releasing_and_allows_explicit_recovery(self):
        self.raw.sim(block={}, pose=dict(self.raw.goal, gripper=27))
        self.raw.lead = [.2] * 4
        with self.assertRaises(GripLost) as stopped:
            self.arm.follow(self.chain(), duration=1, grip_range=(29, 31))
        self.assertIsInstance(stopped.exception, Fault)
        self.assertTrue(self.raw.torque)
        self.assertIsNone(self.raw.pose)
        self.assertEqual(list(self.raw.lead), [0.] * 4)
        self.assertAlmostEqual(self.raw.goal["gripper"], self.grip_goal)
        self.assertEqual(len(self.ticks), 1)
        last = self.ticks[-1]["pos"]
        for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
            self.assertAlmostEqual(self.raw.goal[joint], last[joint])
        self.assertFalse(self.arm.follow(self.chain(), duration=.3))
        self.assertIsNotNone(self.raw.pose)

    def test_invalid_bounds_and_changing_gripper_goal_send_no_ticks(self):
        for bounds in ((31, 29), (-1, 20), (0, 101), (float("nan"), 30), (0, float("inf"))):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                self.arm.follow(self.chain(), grip_range=bounds)
        start = self.raw.goal
        with self.assertRaises(Fault) as stopped:
            self.arm.follow([start, dict(start, gripper=45)], grip_range=(29, 31))
        self.assertNotIsInstance(stopped.exception, GripLost)
        self.assertFalse(self.ticks)
        self.assertAlmostEqual(self.raw.goal["gripper"], self.grip_goal)


if __name__ == "__main__":
    unittest.main()
