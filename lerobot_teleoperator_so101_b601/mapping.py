from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .config_so101_b601_leader import SO101B601LeaderConfig

SO_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
B601_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)
B601_ARM_JOINTS = B601_JOINTS[:6]
ARM_SLOTS = np.asarray([0, 1, 2, 3, 5], dtype=np.int64)


@dataclass(frozen=True)
class MappingState:
    """Resolved calibration bounds, anchor pose, and mapping parameters."""

    zero_leader_deg: np.ndarray
    zero_b601_deg: np.ndarray
    leader_low_deg: np.ndarray
    leader_high_deg: np.ndarray
    follower_low_deg: np.ndarray
    follower_high_deg: np.ndarray
    joint_signs: np.ndarray
    joint_scales: np.ndarray
    piecewise_range_map: bool
    deadband_deg: float
    margin_deg: float
    wrist_yaw_deg: float
    gripper_leader_min: float
    gripper_leader_max: float
    gripper_min_deg: float
    gripper_max_deg: float
    invert_gripper: bool


@dataclass(frozen=True)
class MappedAction:
    """Mapped action alongside raw leader and physical B601 targets for diagnostics."""

    raw_leader_deg: np.ndarray
    b601_target_deg: np.ndarray
    gripper_target_deg: float
    action: dict[str, float]


def _parse_values(value: object, count: int, name: str) -> np.ndarray:
    if isinstance(value, str):
        values = tuple(float(part.strip()) for part in value.split(","))
    elif isinstance(value, (list, tuple, np.ndarray)):
        values = tuple(float(part) for part in value)
    else:
        raise ValueError(f"{name} must be a comma-separated string or numeric sequence")
    if len(values) != count:
        raise ValueError(f"{name} must contain {count} values, got {len(values)}")
    return np.asarray(values, dtype=np.float64)


def calibration_ranges_deg(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Compute symmetric degree limits around midpoint from an SO-101 calibration JSON."""
    with path.open(encoding="utf-8") as handle:
        data = json.load(handle)
    lows: list[float] = []
    highs: list[float] = []
    for name in SO_JOINTS:
        if name not in data:
            raise ValueError(f"{path} has no calibration for {name}")
        item = data[name]
        mid = (item["range_min"] + item["range_max"]) / 2.0
        lows.append((item["range_min"] - mid) * 360.0 / 4095.0)
        highs.append((item["range_max"] - mid) * 360.0 / 4095.0)
    return np.asarray(lows, dtype=np.float64), np.asarray(highs, dtype=np.float64)


def _load_mapping_file(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    if not path.is_file():
        raise FileNotFoundError(f"mapping config not found: {path}")
    with path.open(encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def _resolve_zero_b601_deg(
    data: dict[str, Any], config: SO101B601LeaderConfig
) -> np.ndarray:
    if "zero_b601_deg" in data:
        return _parse_values(data["zero_b601_deg"], 6, "zero_b601_deg")
    if "zero_b601_rad" in data:
        return np.degrees(
            _parse_values(data["zero_b601_rad"], 6, "zero_b601_rad")
        )
    return _parse_values(config.zero_b601_deg, 6, "zero_b601_deg")


def build_mapping_state(
    config: SO101B601LeaderConfig,
    calibration_path: Path,
) -> MappingState:
    """Build a validated `MappingState` from config, optional JSON, and SO-101 calibration."""
    data = _load_mapping_file(config.mapping_config)

    zero_leader_deg = _parse_values(
        data.get("zero_leader_deg", config.zero_leader_deg), 5, "zero_leader_deg"
    )
    zero_b601_deg = _resolve_zero_b601_deg(data, config)
    joint_signs = _parse_values(
        data.get("joint_signs", config.joint_signs), 5, "joint_signs"
    )
    base_scales = _parse_values(
        data.get("joint_scales", config.joint_scales), 5, "joint_scales"
    )
    scale_multipliers = _parse_values(
        config.joint_scale_multipliers, 5, "joint_scale_multipliers"
    )
    joint_scales = base_scales * scale_multipliers
    piecewise_range_map = bool(
        data.get("piecewise_range_map", config.piecewise_range_map)
    )
    leader_low_deg, leader_high_deg = calibration_ranges_deg(calibration_path)

    follower_low_deg = np.asarray(
        [config.follower_limits_deg[name][0] for name in B601_ARM_JOINTS],
        dtype=np.float64,
    )
    follower_high_deg = np.asarray(
        [config.follower_limits_deg[name][1] for name in B601_ARM_JOINTS],
        dtype=np.float64,
    )

    deadband_deg = float(data.get("deadband_deg", config.deadband_deg))
    margin_deg = float(data.get("joint_margin_deg", config.margin_deg))
    wrist_yaw_deg = float(data.get("wrist_yaw_deg", config.wrist_yaw_deg))
    gripper_leader_min = float(
        data.get("gripper_leader_min", config.gripper_leader_min)
    )
    gripper_leader_max = float(
        data.get("gripper_leader_max", config.gripper_leader_max)
    )
    gripper_min_deg = float(data.get("gripper_min_deg", config.gripper_min_deg))
    gripper_max_deg = float(data.get("gripper_max_deg", config.gripper_max_deg))
    invert_gripper = bool(
        data.get("real_gripper_invert", config.invert_gripper)
    )

    if np.any(np.abs(np.abs(joint_signs) - 1.0) > 1e-9):
        raise ValueError("joint_signs values must be +1 or -1")
    if np.any(joint_scales <= 0):
        raise ValueError("joint_scales values must be positive")
    if np.any(leader_high_deg <= leader_low_deg):
        raise ValueError("SO-101 calibration contains a zero-width joint range")
    if np.any(follower_high_deg <= follower_low_deg):
        raise ValueError("follower joint limits must have positive width")
    if deadband_deg < 0 or margin_deg < 0:
        raise ValueError("deadband_deg and margin_deg must be non-negative")
    if gripper_leader_min >= gripper_leader_max:
        raise ValueError("gripper_leader_min must be less than gripper_leader_max")
    if gripper_min_deg >= gripper_max_deg:
        raise ValueError("gripper_min_deg must be less than gripper_max_deg")

    anchor_tolerance_deg = 1.0
    if np.any(zero_b601_deg < follower_low_deg - anchor_tolerance_deg) or np.any(
        zero_b601_deg > follower_high_deg + anchor_tolerance_deg
    ):
        raise ValueError("zero_b601_deg is outside the follower joint limits")
    zero_b601_deg = np.clip(zero_b601_deg, follower_low_deg, follower_high_deg)
    if not follower_low_deg[4] <= wrist_yaw_deg <= follower_high_deg[4]:
        raise ValueError("wrist_yaw_deg is outside the follower joint limits")

    return MappingState(
        zero_leader_deg=zero_leader_deg,
        zero_b601_deg=zero_b601_deg,
        leader_low_deg=leader_low_deg,
        leader_high_deg=leader_high_deg,
        follower_low_deg=follower_low_deg,
        follower_high_deg=follower_high_deg,
        joint_signs=joint_signs,
        joint_scales=joint_scales,
        piecewise_range_map=piecewise_range_map,
        deadband_deg=deadband_deg,
        margin_deg=margin_deg,
        wrist_yaw_deg=wrist_yaw_deg,
        gripper_leader_min=gripper_leader_min,
        gripper_leader_max=gripper_leader_max,
        gripper_min_deg=gripper_min_deg,
        gripper_max_deg=gripper_max_deg,
        invert_gripper=invert_gripper,
    )


def _compute_arm_limits(state: MappingState) -> tuple[np.ndarray, np.ndarray]:
    # Keep the configured anchor reachable even when it sits directly on a hard stop.
    low = np.maximum(
        state.follower_low_deg,
        np.minimum(state.follower_low_deg + state.margin_deg, state.zero_b601_deg),
    )
    high = np.minimum(
        state.follower_high_deg,
        np.maximum(state.follower_high_deg - state.margin_deg, state.zero_b601_deg),
    )
    if np.any(low >= high):
        raise ValueError("joint margin is too large for the follower limits")
    return low, high


def _map_piecewise_delta(
    delta: np.ndarray,
    state: MappingState,
    low: np.ndarray,
    high: np.ndarray,
) -> np.ndarray:
    target = state.zero_b601_deg.copy()
    for idx, slot in enumerate(ARM_SLOTS):
        d = float(delta[idx])
        if d == 0.0:
            continue
        sign = float(state.joint_signs[idx])
        scale = float(state.joint_scales[idx])
        if d > 0.0:
            leader_avail = float(state.leader_high_deg[idx] - state.zero_leader_deg[idx])
            follower_avail = float(
                (high[slot] - state.zero_b601_deg[slot])
                if sign > 0.0
                else (state.zero_b601_deg[slot] - low[slot])
            )
        else:
            leader_avail = float(state.zero_leader_deg[idx] - state.leader_low_deg[idx])
            follower_avail = float(
                (state.zero_b601_deg[slot] - low[slot])
                if sign > 0.0
                else (high[slot] - state.zero_b601_deg[slot])
            )
        if leader_avail > 1e-6 and follower_avail > 0.0:
            target[slot] += sign * scale * (d / leader_avail) * follower_avail
    return target


def _map_linear_delta(delta: np.ndarray, state: MappingState) -> np.ndarray:
    leader_span = state.leader_high_deg - state.leader_low_deg
    follower_span = (
        state.follower_high_deg[ARM_SLOTS] - state.follower_low_deg[ARM_SLOTS]
    )
    target = state.zero_b601_deg.copy()
    target[ARM_SLOTS] += (
        state.joint_signs * state.joint_scales * (delta / leader_span) * follower_span
    )
    return target


def map_arm_target(leader_deg: np.ndarray, state: MappingState) -> np.ndarray:
    """Map 5 SO-101 arm joint angles (deg) to 6 physical B601 arm joint angles (deg)."""
    low, high = _compute_arm_limits(state)
    delta = leader_deg - state.zero_leader_deg
    delta[np.abs(delta) < state.deadband_deg] = 0.0

    target = (
        _map_piecewise_delta(delta, state, low, high)
        if state.piecewise_range_map
        else _map_linear_delta(delta, state)
    )
    target = np.clip(target, low, high)
    target[4] = state.wrist_yaw_deg
    return target


def map_gripper_target(leader_value: float, state: MappingState) -> float:
    """Map SO-101 normalized gripper position (0..100) to B601 physical gripper deg."""
    ratio = float(
        np.clip(
            (leader_value - state.gripper_leader_min)
            / (state.gripper_leader_max - state.gripper_leader_min),
            0.0,
            1.0,
        )
    )
    if state.invert_gripper:
        ratio = 1.0 - ratio
    return state.gripper_min_deg + ratio * (
        state.gripper_max_deg - state.gripper_min_deg
    )


def map_action(
    raw_action: dict[str, Any],
    state: MappingState,
    follower_directions: dict[str, float],
    pretransform_follower_directions: bool,
) -> MappedAction:
    """Convert raw SO-101 leader reading into a `rebot_b601_follower` action dict."""
    leader_deg = np.asarray(
        [float(raw_action[f"{name}.pos"]) for name in SO_JOINTS],
        dtype=np.float64,
    )
    b601_target_deg = map_arm_target(leader_deg, state)
    gripper_target_deg = map_gripper_target(
        float(raw_action["gripper.pos"]), state
    )
    physical_targets = np.concatenate(
        [b601_target_deg, np.asarray([gripper_target_deg], dtype=np.float64)]
    )

    if pretransform_follower_directions:
        directions = np.asarray(
            [float(follower_directions[name]) for name in B601_JOINTS],
            dtype=np.float64,
        )
        if np.any(np.abs(directions) < 1e-9):
            raise ValueError("follower_directions values must be non-zero")
        action_values = physical_targets / directions
    else:
        action_values = physical_targets

    return MappedAction(
        raw_leader_deg=leader_deg,
        b601_target_deg=b601_target_deg,
        gripper_target_deg=gripper_target_deg,
        action={
            f"{name}.pos": float(action_values[index])
            for index, name in enumerate(B601_JOINTS)
        },
    )


def follower_action_to_physical_deg(
    action: dict[str, float],
    follower_directions: dict[str, float],
) -> np.ndarray:
    """Apply the follower `joint_directions` transform for diagnostics and tests."""
    return np.asarray(
        [
            float(action[f"{name}.pos"]) * float(follower_directions[name])
            for name in B601_JOINTS
        ],
        dtype=np.float64,
    )
