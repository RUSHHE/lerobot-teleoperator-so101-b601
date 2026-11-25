from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

import numpy as np

try:
    from lerobot.lerobot_types import RobotAction
except ImportError:
    from lerobot.processor import RobotAction

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import (
    DeviceAlreadyConnectedError,
    DeviceNotConnectedError,
)

try:
    from lerobot.teleoperators.so101_leader import (
        SO101Leader,
        SO101LeaderConfig,
    )
except ImportError:
    from lerobot.teleoperators.so_leader import (
        SO101Leader,
        SO101LeaderConfig,
    )

from .config_so101_b601_leader import (
    SO101B601LeaderConfig,
    _default_so101_calibration_dir,
)
from .mapping import (
    B601_JOINTS,
    SO_JOINTS,
    MappingState,
    build_mapping_state,
    map_action,
)

logger = logging.getLogger(__name__)


def _resolve_leader_calibration_dir(
    candidate_dir: Path, leader_id: str
) -> Path:
    """Resolve v0.7.0 `so_leader` calibration dir with fallback to legacy `so101_leader`."""
    primary = candidate_dir / f"{leader_id}.json"
    if not primary.is_file() and candidate_dir.name == "so_leader":
        fallback_dir = candidate_dir.parent / "so101_leader"
        if (fallback_dir / f"{leader_id}.json").is_file():
            return fallback_dir
    return candidate_dir


class SO101B601Leader(Teleoperator):
    """LeRobot teleoperator that adapts an SO-101 leader to a B601 follower."""

    config_class = SO101B601LeaderConfig
    name = "so101_b601_leader"

    def __init__(self, config: SO101B601LeaderConfig):
        super().__init__(config)
        self.config = config
        self._leader: SO101Leader | None = None
        self._mapping_state: MappingState | None = None
        self._last_log_time = 0.0
        self._startup_checked = False

        self._leader_id = config.leader_id or config.id
        if not self._leader_id:
            raise ValueError(
                "SO-101 calibration id is required; set --teleop.id or "
                "--teleop.leader_id"
            )
        candidate_dir = (
            config.so101_calibration_dir
            or config.calibration_dir
            or _default_so101_calibration_dir()
        )
        self._leader_calibration_dir = _resolve_leader_calibration_dir(
            candidate_dir, self._leader_id
        )
        self._leader_calibration_path = (
            self._leader_calibration_dir / f"{self._leader_id}.json"
        )
        self._validate_config()

    def _validate_config(self) -> None:
        missing_limits = [
            name
            for name in B601_JOINTS
            if name not in self.config.follower_limits_deg
        ]
        missing_directions = [
            name
            for name in B601_JOINTS
            if name not in self.config.follower_directions
        ]
        if missing_limits:
            raise ValueError(f"follower_limits_deg is missing {missing_limits}")
        if missing_directions:
            raise ValueError(
                f"follower_directions is missing {missing_directions}"
            )
        if self.config.num_read_retries < 0:
            raise ValueError("num_read_retries must be non-negative")
        if self.config.log_every_s < 0:
            raise ValueError("log_every_s must be non-negative")
        if (
            self.config.startup_tolerance_deg is not None
            and self.config.startup_tolerance_deg < 0
        ):
            raise ValueError("startup_tolerance_deg must be non-negative")

    @property
    def leader(self) -> SO101Leader | None:
        return self._leader

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in B601_JOINTS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._leader is not None and self._leader.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info("Connecting SO-101 leader on %s", self.config.port)
        leader_config = SO101LeaderConfig(
            port=self.config.port,
            id=self._leader_id,
            use_degrees=True,
            num_read_retries=self.config.num_read_retries,
            calibration_dir=self._leader_calibration_dir,
        )
        leader = SO101Leader(leader_config)
        try:
            leader.connect(calibrate=calibrate)
            if not self._leader_calibration_path.is_file():
                raise FileNotFoundError(
                    "SO-101 calibration file not found after connect: "
                    f"{self._leader_calibration_path}"
                )
            mapping_state = build_mapping_state(
                self.config, self._leader_calibration_path
            )
        except Exception:
            if leader.is_connected:
                leader.disconnect()
            raise

        self._leader = leader
        self._mapping_state = mapping_state
        self._last_log_time = 0.0
        self._startup_checked = False
        logger.info(
            "SO-101 adapter connected: leader=%s calibration=%s mapping=%s",
            self._leader_id,
            self._leader_calibration_path,
            self.config.mapping_config or "built-in defaults",
        )

    @property
    def is_calibrated(self) -> bool:
        if self._leader is not None:
            return self._leader.is_calibrated
        return self._leader_calibration_path.is_file()

    def calibrate(self) -> None:
        if self._leader is None:
            raise DeviceNotConnectedError(
                "connect the SO-101 leader before calibration"
            )
        self._leader.calibrate()
        self._mapping_state = build_mapping_state(
            self.config, self._leader_calibration_path
        )
        self._startup_checked = False

    def configure(self) -> None:
        if self._leader is None:
            raise DeviceNotConnectedError("SO-101 leader is not connected")
        self._leader.configure()

    def map_leader_action(self, raw_action: RobotAction):
        if self._mapping_state is None:
            raise DeviceNotConnectedError("SO-101 leader is not connected")
        return map_action(
            raw_action,
            self._mapping_state,
            self.config.follower_directions,
            self.config.pretransform_follower_directions,
        )

    def _check_startup_pose(self, raw_leader_deg: np.ndarray) -> None:
        tolerance = self.config.startup_tolerance_deg
        if tolerance is None or tolerance <= 0 or self._mapping_state is None:
            return
        zero_deg = self._mapping_state.zero_leader_deg
        diffs = np.abs(raw_leader_deg - zero_deg)
        if np.any(diffs > tolerance):
            details = ", ".join(
                f"{name}: current={cur:+.1f}deg vs zero={ref:+.1f}deg (diff={diff:.1f}deg)"
                for name, cur, ref, diff in zip(
                    SO_JOINTS, raw_leader_deg, zero_deg, diffs, strict=True
                )
                if diff > tolerance
            )
            raise RuntimeError(
                "SO-101 leader is not in the sit-down zero pose at startup "
                f"(tolerance={tolerance:.1f}deg): {details}. "
                "Place the SO-101 leader in the sit-down zero pose before starting, "
                "or set --teleop.startup_tolerance_deg=0 to disable this check."
            )

    def get_action(self) -> RobotAction:
        if self._leader is None or self._mapping_state is None:
            raise DeviceNotConnectedError("SO-101 leader is not connected")

        raw_action = self._leader.get_action()
        mapped = self.map_leader_action(raw_action)
        if not self._startup_checked:
            self._check_startup_pose(mapped.raw_leader_deg)
            self._startup_checked = True

        now = time.monotonic()
        if (
            self.config.log_every_s == 0
            or now - self._last_log_time >= self.config.log_every_s
        ):
            logger.info(
                "SO-101 raw=%s | B601 target=%s deg | gripper=%.1f deg",
                " ".join(f"{value:+.1f}" for value in mapped.raw_leader_deg),
                " ".join(f"{value:+.1f}" for value in mapped.b601_target_deg),
                mapped.gripper_target_deg,
            )
            self._last_log_time = now

        return mapped.action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if feedback:
            raise NotImplementedError(
                "SO-101 B601 adapter does not implement feedback"
            )

    def disconnect(self) -> None:
        if self._leader is not None and self._leader.is_connected:
            self._leader.disconnect()
        self._leader = None
        self._mapping_state = None
        self._startup_checked = False
