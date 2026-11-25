from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig


def _default_so101_calibration_dir() -> Path:
    """Return LeRobot v0.7.0 default calibration directory for SO-101 leaders."""
    return (
        Path.home()
        / ".cache"
        / "huggingface"
        / "lerobot"
        / "calibration"
        / "teleoperators"
        / "so_leader"
    )


@TeleoperatorConfig.register_subclass("so101_b601_leader")
@dataclass
class SO101B601LeaderConfig(TeleoperatorConfig):
    """Configuration for adapting an SO-101 leader to LeRobot v0.7.0 `rebot_b601_follower`."""

    port: str

    # SO-101 calibration identity. Defaults to `id` when omitted.
    leader_id: str | None = None
    so101_calibration_dir: Path | None = field(
        default_factory=_default_so101_calibration_dir
    )
    num_read_retries: int = 2

    # Optional external mapping JSON overriding the calibrated defaults below.
    mapping_config: Path | None = None
    deadband_deg: float = 0.5
    margin_deg: float = 2.0
    startup_tolerance_deg: float | None = 15.0

    # Map each side of `zero_leader_deg` independently to the corresponding follower limit
    # so asymmetric joints such as `wrist_flex` reach both ends of travel.
    piecewise_range_map: bool = True

    zero_leader_deg: list[float] = field(
        default_factory=lambda: [-6.20, -104.09, 95.78, 56.04, 0.48]
    )
    zero_b601_deg: list[float] = field(
        default_factory=lambda: [
            -1.71142,
            -0.10984,
            0.07816,
            -0.00659,
            0.61524,
            -0.90527,
        ]
    )
    joint_signs: list[float] = field(
        default_factory=lambda: [1.0, 1.0, -1.0, -1.0, 1.0]
    )
    joint_scales: list[float] = field(
        default_factory=lambda: [1.0, 1.0, 1.0, 1.0, 1.0]
    )
    # Multiplied on top of `joint_scales` so `wrist_roll` (360 deg SO-101 vs 180 deg B601)
    # tracks 1:1 while preserving shared mapping JSON compatibility.
    joint_scale_multipliers: list[float] = field(
        default_factory=lambda: [1.0, 1.0, 1.0, 1.0, 2.0]
    )

    # Physical B601-RS motor limits in degrees (negated v0.7.0 RS_PROFILE.joint_limits).
    follower_limits_deg: dict[str, list[float]] = field(
        default_factory=lambda: {
            "shoulder_pan": [-145.0, 145.0],
            "shoulder_lift": [0.0, 170.0],
            "elbow_flex": [0.0, 200.0],
            "wrist_flex": [-80.0, 90.0],
            "wrist_yaw": [-90.0, 90.0],
            "wrist_roll": [-90.0, 90.0],
            "gripper": [0.0, 270.0],
        }
    )

    # Matches LeRobot v0.7.0 `RS_PROFILE.joint_directions` applied in `send_action()`.
    follower_directions: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": -1.0,
            "shoulder_lift": -1.0,
            "elbow_flex": -1.0,
            "wrist_flex": -1.0,
            "wrist_yaw": -1.0,
            "wrist_roll": -1.0,
            "gripper": -1.0,
        }
    )
    pretransform_follower_directions: bool = True
    wrist_yaw_deg: float = 0.0

    gripper_leader_min: float = 0.0
    gripper_leader_max: float = 100.0
    gripper_min_deg: float = 0.0
    gripper_max_deg: float = 270.0
    invert_gripper: bool = False

    log_every_s: float = 5.0
