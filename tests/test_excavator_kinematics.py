# Copyright VMC Motion Technologies Co., Ltd.
# Licensed under the Apache-2.0 license. See LICENSE.

from pathlib import Path
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "cet200_agxpy_standalone" / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from cet200_agxpy_standalone.excavator_kinematics import (  # noqa: E402
    CylinderDisplacements,
    DEFAULT_DIMENSIONS,
    ELBOW_DOWN,
    JointAngles,
    forward_kinematics_from_cylinders,
    forward_kinematics_from_joints,
    inverse_kinematics_to_cylinders,
    inverse_kinematics_to_joint_angles,
    joint_angles_to_cylinders,
    cylinders_to_joint_angles,
)


class ExcavatorKinematicsTest(unittest.TestCase):
    def assert_tuple_almost_equal(self, actual, expected, places=6):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=places)

    def assert_angles_almost_equal(self, actual, expected, places=6):
        self.assertAlmostEqual(actual.slew, expected.slew, places=places)
        self.assertAlmostEqual(actual.boom, expected.boom, places=places)
        self.assertAlmostEqual(actual.arm, expected.arm, places=places)
        self.assertAlmostEqual(actual.bucket, expected.bucket, places=places)

    def assert_cylinders_almost_equal(self, actual, expected, places=6):
        self.assertAlmostEqual(actual.slew, expected.slew, places=places)
        self.assertAlmostEqual(actual.boom, expected.boom, places=places)
        self.assertAlmostEqual(actual.arm, expected.arm, places=places)
        self.assertAlmostEqual(actual.bucket, expected.bucket, places=places)

    def test_zero_cylinders_match_agx_initial_hinges_and_tip_center(self):
        pose = forward_kinematics_from_cylinders(CylinderDisplacements(0.0, 0.0, 0.0, 0.0))

        self.assert_tuple_almost_equal(pose.position, (4.209238, 0.0, 1.008412), places=5)
        self.assertAlmostEqual(pose.yaw, 0.0)
        self.assertAlmostEqual(pose.pitch, 0.0)

        angles = cylinders_to_joint_angles(CylinderDisplacements(0.0, 0.0, 0.0, 0.0))
        self.assert_angles_almost_equal(angles, JointAngles(0.0, 0.0, 0.0, 0.0))

    def test_joint_fk_round_trips_through_ik(self):
        examples = [
            JointAngles(slew=0.0, boom=0.0, arm=0.0, bucket=0.0),
            JointAngles(slew=0.25, boom=0.10, arm=-0.30, bucket=0.25),
            JointAngles(slew=-0.40, boom=-0.15, arm=0.20, bucket=-0.20),
        ]

        for angles in examples:
            with self.subTest(angles=angles):
                pose = forward_kinematics_from_joints(angles)
                solved = inverse_kinematics_to_joint_angles(pose.position, pose.pitch, slew=pose.yaw, prefer=ELBOW_DOWN)
                self.assert_angles_almost_equal(solved, angles)

    def test_cylinder_joint_round_trip(self):
        examples = [
            JointAngles(slew=0.0, boom=0.0, arm=0.0, bucket=0.0),
            JointAngles(slew=0.2, boom=0.10, arm=-0.25, bucket=0.15),
            JointAngles(slew=-0.3, boom=-0.10, arm=0.15, bucket=-0.10),
        ]

        for angles in examples:
            with self.subTest(angles=angles):
                cylinders = joint_angles_to_cylinders(angles)
                solved = cylinders_to_joint_angles(cylinders)
                self.assert_angles_almost_equal(solved, angles)

    def test_cylinder_fk_and_ik_round_trip(self):
        cylinders = joint_angles_to_cylinders(JointAngles(slew=0.3, boom=0.08, arm=-0.20, bucket=0.12))

        pose = forward_kinematics_from_cylinders(cylinders)
        solved = inverse_kinematics_to_cylinders(pose.position, pose.pitch, slew=pose.yaw)

        self.assert_cylinders_almost_equal(solved, cylinders)

    def test_rejects_out_of_range_cylinder_displacement(self):
        too_far = CylinderDisplacements(
            slew=0.0,
            boom=DEFAULT_DIMENSIONS.boom_cylinder.max_displacement + 1.0,
            arm=0.0,
            bucket=0.0,
        )

        with self.assertRaisesRegex(ValueError, "boom シリンダ変位"):
            cylinders_to_joint_angles(too_far)

    def test_rejects_unreachable_tip_target(self):
        with self.assertRaisesRegex(ValueError, "到達可能範囲外"):
            inverse_kinematics_to_joint_angles((100.0, 0.0, 100.0), pitch=0.0, slew=0.0)


if __name__ == "__main__":
    unittest.main()
