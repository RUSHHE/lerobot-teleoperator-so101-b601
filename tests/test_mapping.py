from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from lerobot.robots.rebot_b601_follower.motor_family import RS_PROFILE

from lerobot_teleoperator_so101_b601 import SO101B601LeaderConfig
from lerobot_teleoperator_so101_b601.mapping import (
    B601_JOINTS,
    SO_JOINTS,
    build_mapping_state,
    follower_action_to_physical_deg,
    map_action,
)


class MappingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.calibration_path = Path(self.temp_dir.name) / "leader.json"
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
        self.calibration_path.write_text(
            json.dumps(calibration), encoding="utf-8"
        )
        self.config = SO101B601LeaderConfig(
            port="/dev/null",
            id="test_leader",
            leader_id="test_leader",
            mapping_config=None,
        )
        self.state = build_mapping_state(
            self.config, self.calibration_path
        )

    def _raw_action(self, gripper: float = 0.0) -> dict[str, float]:
        action = {
            f"{name}.pos": float(value)
            for name, value in zip(
                SO_JOINTS, self.state.zero_leader_deg, strict=True
            )
        }
        action["gripper.pos"] = gripper
        return action

    def test_bundled_mapping_config_loads(self) -> None:
        bundled = Path(__file__).resolve().parents[1] / "configs" / "so101_b601_mapping.json"
        cfg = SO101B601LeaderConfig(
            port="/dev/null",
            id="test_leader",
            leader_id="test_leader",
            mapping_config=bundled,
        )
        state = build_mapping_state(cfg, self.calibration_path)
        self.assertTrue(state.piecewise_range_map)
        self.assertEqual(state.wrist_yaw_deg, 0.0)

    def test_anchor_maps_to_zero_and_wrist_yaw_is_held(self) -> None:
        mapped = map_action(
            self._raw_action(),
            self.state,
            self.config.follower_directions,
            True,
        )
        expected = np.clip(
            self.state.zero_b601_deg.copy(),
            self.state.follower_low_deg,
            self.state.follower_high_deg,
        )
        expected[4] = self.config.wrist_yaw_deg
        np.testing.assert_allclose(mapped.b601_target_deg, expected, atol=1e-9)
        self.assertAlmostEqual(mapped.action["shoulder_lift.pos"], 0.0)
        self.assertAlmostEqual(mapped.action["gripper.pos"], 0.0)

    def test_wrist_flex_asymmetric_endpoints_reach_both_limits(self) -> None:
        raw_high = self._raw_action()
        raw_high["wrist_flex.pos"] = float(self.state.leader_high_deg[3])
        mapped_high = map_action(
            raw_high,
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertAlmostEqual(mapped_high.b601_target_deg[3], -78.0, places=6)

        raw_low = self._raw_action()
        raw_low["wrist_flex.pos"] = float(self.state.leader_low_deg[3])
        mapped_low = map_action(
            raw_low,
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertAlmostEqual(mapped_low.b601_target_deg[3], 88.0, places=6)

    def test_linear_fallback_mode_remains_available(self) -> None:
        linear_cfg = SO101B601LeaderConfig(
            port="/dev/null",
            id="test_leader",
            leader_id="test_leader",
            mapping_config=None,
            piecewise_range_map=False,
        )
        linear_state = build_mapping_state(linear_cfg, self.calibration_path)
        raw = self._raw_action()
        raw["shoulder_pan.pos"] += 30.0
        mapped = map_action(raw, linear_state, linear_cfg.follower_directions, True)
        self.assertGreater(mapped.b601_target_deg[0], 20.0)

    def test_wrist_roll_is_one_to_one_with_default_multiplier(self) -> None:
        raw = self._raw_action()
        raw["wrist_roll.pos"] += 45.0
        mapped = map_action(
            raw,
            self.state,
            self.config.follower_directions,
            True,
        )
        delta_b601 = mapped.b601_target_deg[5] - self.state.zero_b601_deg[5]
        self.assertGreater(delta_b601, 40.0)
        self.assertLessEqual(delta_b601, 45.0)

    def test_lifting_shoulder_maps_to_positive_physical_and_negative_public(self) -> None:
        raw = self._raw_action()
        raw["shoulder_lift.pos"] += 30.0
        mapped = map_action(
            raw,
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertGreater(mapped.b601_target_deg[1], 0.0)
        self.assertLess(mapped.action["shoulder_lift.pos"], 0.0)
        self.assertAlmostEqual(
            mapped.action["shoulder_lift.pos"],
            -mapped.b601_target_deg[1],
            places=9,
        )

    def test_gripper_full_scale_and_follower_pretransform(self) -> None:
        mapped = map_action(
            self._raw_action(gripper=100.0),
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertAlmostEqual(mapped.gripper_target_deg, 270.0)
        self.assertAlmostEqual(mapped.action["gripper.pos"], -270.0)

        physical = follower_action_to_physical_deg(
            mapped.action, self.config.follower_directions
        )
        self.assertAlmostEqual(physical[-1], 270.0)

    def test_arm_action_is_inverse_of_follower_directions(self) -> None:
        raw = self._raw_action()
        raw["elbow_flex.pos"] -= 20.0
        mapped = map_action(
            raw,
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertGreater(mapped.b601_target_deg[2], 10.0)
        physical = follower_action_to_physical_deg(
            mapped.action, self.config.follower_directions
        )
        np.testing.assert_allclose(
            physical[:6], mapped.b601_target_deg, atol=1e-9
        )
        self.assertAlmostEqual(
            physical[-1], mapped.gripper_target_deg, places=9
        )

    def test_full_stroke_stays_within_rs_profile_limits(self) -> None:
        for bound_selector in (0, 1, 2):
            if bound_selector == 0:
                arm_vals = self.state.leader_low_deg
                gripper_val = 0.0
            elif bound_selector == 1:
                arm_vals = self.state.zero_leader_deg
                gripper_val = 50.0
            else:
                arm_vals = self.state.leader_high_deg
                gripper_val = 100.0
            raw = {
                f"{name}.pos": float(val)
                for name, val in zip(SO_JOINTS, arm_vals, strict=True)
            }
            raw["gripper.pos"] = gripper_val
            mapped = map_action(
                raw,
                self.state,
                self.config.follower_directions,
                True,
            )
            for name in B601_JOINTS:
                low, high = RS_PROFILE.joint_limits[name]
                val = mapped.action[f"{name}.pos"]
                self.assertGreaterEqual(val, low - 1e-9, msg=name)
                self.assertLessEqual(val, high + 1e-9, msg=name)

    def test_inverted_gripper(self) -> None:
        self.state = build_mapping_state(
            SO101B601LeaderConfig(
                port="/dev/null",
                id="test_leader",
                leader_id="test_leader",
                mapping_config=None,
                invert_gripper=True,
            ),
            self.calibration_path,
        )
        mapped = map_action(
            self._raw_action(gripper=100.0),
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertAlmostEqual(mapped.gripper_target_deg, 0.0)
        self.assertAlmostEqual(mapped.action["gripper.pos"], 0.0)

    def test_action_has_exact_follower_keys(self) -> None:
        mapped = map_action(
            self._raw_action(),
            self.state,
            self.config.follower_directions,
            True,
        )
        self.assertEqual(
            tuple(mapped.action),
            tuple(f"{name}.pos" for name in B601_JOINTS),
        )

    def test_direct_follower_mode_keeps_physical_targets(self) -> None:
        raw = self._raw_action(gripper=100.0)
        raw["elbow_flex.pos"] -= 20.0
        mapped = map_action(
            raw,
            self.state,
            self.config.follower_directions,
            False,
        )
        physical = np.concatenate(
            [mapped.b601_target_deg, [mapped.gripper_target_deg]]
        )
        np.testing.assert_allclose(
            np.asarray(list(mapped.action.values())),
            physical,
            atol=1e-9,
        )


if __name__ == "__main__":
    unittest.main()
