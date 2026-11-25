from __future__ import annotations

import json
import os
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import draccus
from lerobot.robots.rebot_b601_follower import (
    RebotB601Follower,
    RebotB601FollowerRobotConfig,
)
from lerobot.robots.rebot_b601_follower.motor_family import (
    RS_PROFILE,
    MotorFamily,
)
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

from lerobot_teleoperator_so101_b601 import (
    SO101B601Leader,
    SO101B601LeaderConfig,
)
from lerobot_teleoperator_so101_b601.mapping import B601_JOINTS, SO_JOINTS


def _write_dummy_so101_calibration(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    calibration = {
        name: {
            "id": index + 1,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0,
            "range_max": 4095,
        }
        for index, name in enumerate((*SO_JOINTS, "gripper"))
    }
    path.write_text(json.dumps(calibration), encoding="utf-8")


class OfficialFollowerCompatibilityTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.tmp_path = Path(self.temp_dir.name)
        self.leader_config = SO101B601LeaderConfig(
            port="/dev/null",
            id="test_leader",
            calibration_dir=self.tmp_path / "teleop_cal",
            so101_calibration_dir=self.tmp_path / "so_leader",
            mapping_config=None,
        )
        self.follower_config = RebotB601FollowerRobotConfig(
            port="can0",
            id="test_follower",
            motor_family=MotorFamily.RS,
            calibration_dir=self.tmp_path / "robot_cal",
        )

    def test_leader_defaults_to_official_follower_transform(self) -> None:
        self.assertTrue(
            self.leader_config.pretransform_follower_directions
        )

    def test_direction_table_matches_rs_profile(self) -> None:
        self.assertEqual(
            self.leader_config.follower_directions,
            RS_PROFILE.joint_directions,
        )

    def test_limits_match_negated_rs_profile_limits(self) -> None:
        for name in B601_JOINTS:
            phys_low, phys_high = self.leader_config.follower_limits_deg[name]
            rs_low, rs_high = RS_PROFILE.joint_limits[name]
            self.assertEqual((-phys_high, -phys_low), (rs_low, rs_high), msg=name)

    def test_action_features_and_public_to_motor_roundtrip(self) -> None:
        leader = SO101B601Leader(self.leader_config)
        follower = RebotB601Follower(self.follower_config)
        self.assertEqual(
            list(leader.action_features.keys()),
            list(follower.action_features.keys()),
        )

        cal_path = self.tmp_path / "so_leader" / "test_leader.json"
        _write_dummy_so101_calibration(cal_path)

        raw = {
            f"{name}.pos": float(val)
            for name, val in zip(
                SO_JOINTS, self.leader_config.zero_leader_deg, strict=True
            )
        }
        raw["shoulder_lift.pos"] += 30.0
        raw["gripper.pos"] = 50.0

        with patch(
            "lerobot_teleoperator_so101_b601.so101_b601_leader.SO101Leader"
        ) as mock_cls:
            instance = mock_cls.return_value
            instance.is_connected = True
            instance.is_calibrated = True
            instance.get_action.return_value = raw
            leader.connect(calibrate=False)
            mapped = leader.map_leader_action(raw)
            for idx, name in enumerate(B601_JOINTS[:6]):
                motor_deg = follower._public_to_motor_position(
                    name, mapped.action[f"{name}.pos"]
                )
                self.assertAlmostEqual(
                    motor_deg, mapped.b601_target_deg[idx], places=9
                )
            gripper_deg = follower._public_to_motor_position(
                "gripper", mapped.action["gripper.pos"]
            )
            self.assertAlmostEqual(
                gripper_deg, mapped.gripper_target_deg, places=9
            )
            leader.disconnect()

    def test_startup_pose_guard_blocks_off_anchor_and_allows_zero_pose(self) -> None:
        cal_path = self.tmp_path / "so_leader" / "test_leader.json"
        _write_dummy_so101_calibration(cal_path)

        zero_raw = {
            f"{name}.pos": float(val)
            for name, val in zip(
                SO_JOINTS, self.leader_config.zero_leader_deg, strict=True
            )
        }
        zero_raw["gripper.pos"] = 0.0

        off_raw = dict(zero_raw)
        off_raw["shoulder_lift.pos"] += 20.0  # exceeds 15 deg default tolerance

        moved_raw = dict(zero_raw)
        moved_raw["shoulder_lift.pos"] += 40.0  # allowed after startup frame passes

        with patch(
            "lerobot_teleoperator_so101_b601.so101_b601_leader.SO101Leader"
        ) as mock_cls:
            instance = mock_cls.return_value
            instance.is_connected = True
            instance.is_calibrated = True

            # Frame 1 off-anchor raises RuntimeError before returning action
            instance.get_action.return_value = off_raw
            leader = SO101B601Leader(self.leader_config)
            leader.connect(calibrate=False)
            with self.assertRaises(RuntimeError) as ctx:
                leader.get_action()
            self.assertIn("shoulder_lift", str(ctx.exception))
            leader.disconnect()

            # Frame 1 at anchor passes, Frame 2 >15 deg away also passes
            instance.get_action.side_effect = [zero_raw, moved_raw]
            leader2 = SO101B601Leader(self.leader_config)
            leader2.connect(calibrate=False)
            first_action = leader2.get_action()
            self.assertAlmostEqual(first_action["shoulder_lift.pos"], 0.0)
            second_action = leader2.get_action()
            self.assertLess(second_action["shoulder_lift.pos"], -10.0)
            leader2.disconnect()

            # startup_tolerance_deg=0 disables check
            cfg_disabled = SO101B601LeaderConfig(
                port="/dev/null",
                id="test_leader",
                calibration_dir=self.tmp_path / "teleop_cal",
                so101_calibration_dir=self.tmp_path / "so_leader",
                mapping_config=None,
                startup_tolerance_deg=0.0,
            )
            instance.get_action.side_effect = None
            instance.get_action.return_value = off_raw
            leader3 = SO101B601Leader(cfg_disabled)
            leader3.connect(calibrate=False)
            action = leader3.get_action()
            self.assertLess(action["shoulder_lift.pos"], 0.0)
            leader3.disconnect()

    def test_calibration_dir_resolution_primary_and_legacy_fallback(self) -> None:
        so_leader_dir = self.tmp_path / "so_leader"
        legacy_dir = self.tmp_path / "so101_leader"
        _write_dummy_so101_calibration(so_leader_dir / "primary_id.json")
        _write_dummy_so101_calibration(legacy_dir / "legacy_id.json")

        cfg_primary = SO101B601LeaderConfig(
            port="/dev/null",
            id="primary_id",
            calibration_dir=self.tmp_path / "teleop_cal",
            so101_calibration_dir=so_leader_dir,
            mapping_config=None,
        )
        leader_primary = SO101B601Leader(cfg_primary)
        self.assertEqual(leader_primary._leader_calibration_dir, so_leader_dir)

        cfg_fallback = SO101B601LeaderConfig(
            port="/dev/null",
            id="legacy_id",
            calibration_dir=self.tmp_path / "teleop_cal",
            so101_calibration_dir=so_leader_dir,
            mapping_config=None,
        )
        leader_fallback = SO101B601Leader(cfg_fallback)
        self.assertEqual(leader_fallback._leader_calibration_dir, legacy_dir)

    def test_launcher_flags_and_draccus_parse(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        launcher_path = repo_root / "run_teleoperate.sh"
        launcher = launcher_path.read_text(encoding="utf-8")

        self.assertIn("--robot.type=rebot_b601_follower", launcher)
        self.assertIn("--robot.motor_family=rs", launcher)
        self.assertIn("--robot.gripper_hold_torque_limit=", launcher)
        self.assertIn("--teleop.type=so101_b601_leader", launcher)
        self.assertNotIn("seeed_b601_rs_follower", launcher)
        self.assertNotIn("gripper_mit_hold_torque_limit", launcher)

        # Capture exact argv produced by run_teleoperate.sh without calling lerobot-teleoperate
        dumper = self.tmp_path / "dump_argv.py"
        dumper.write_text(
            f"#!{sys.executable}\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n",
            encoding="utf-8",
        )
        os.chmod(dumper, os.stat(dumper).st_mode | stat.S_IXUSR)

        for with_cams in ("0", "1"):
            env = os.environ.copy()
            env["LEROBOT_BIN"] = str(dumper)
            env["WITH_CAMERAS"] = with_cams
            if with_cams == "1":
                env["FRONT_CAM_SERIAL"] = "000000000001"
                env["SIDE_CAM_SERIAL"] = "000000000002"
            proc = subprocess.run(
                ["bash", str(launcher_path)],
                env=env,
                check=True,
                capture_output=True,
                text=True,
            )
            argv = json.loads(proc.stdout)
            cfg = draccus.parse(config_class=TeleoperateConfig, args=argv)
            self.assertEqual(cfg.robot.type, "rebot_b601_follower")
            self.assertEqual(cfg.robot.motor_family, MotorFamily.RS)
            self.assertEqual(cfg.robot.id, "so101_b601_follower")
            self.assertEqual(cfg.robot.gripper_hold_torque_limit, 3.5)
            self.assertIsNone(cfg.robot.max_relative_target)
            self.assertEqual(cfg.teleop.type, "so101_b601_leader")
            self.assertEqual(cfg.teleop.id, "so101_leader")
            self.assertEqual(cfg.teleop.leader_id, "so101_leader")
            self.assertEqual(cfg.fps, 30)
            if with_cams == "1":
                self.assertTrue(cfg.display_data)
                self.assertEqual(set(cfg.robot.cameras.keys()), {"front", "side"})
            else:
                self.assertFalse(cfg.display_data)
                self.assertEqual(cfg.robot.cameras, {})


if __name__ == "__main__":
    unittest.main()
