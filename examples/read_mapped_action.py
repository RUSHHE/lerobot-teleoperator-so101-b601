#!/usr/bin/env python3
"""Read SO-101 leader angles and print mapped B601-RS targets without powering the follower."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from lerobot_teleoperator_so101_b601 import (
    SO101B601Leader,
    SO101B601LeaderConfig,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read SO-101 angles and print the mapped B601 follower action."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--leader-id", default="so101_leader")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=None,
        help="Override SO-101 calibration directory (defaults to so_leader with so101_leader fallback).",
    )
    parser.add_argument(
        "--mapping-config",
        type=Path,
        default=None,
        help="Optional mapping JSON path (defaults to built-in calibrated parameters).",
    )
    parser.add_argument("--hz", type=float, default=10.0)
    args = parser.parse_args()

    config_kwargs: dict[str, object] = {
        "port": args.port,
        "id": args.leader_id,
        "leader_id": args.leader_id,
        "mapping_config": args.mapping_config,
    }
    if args.calibration_dir is not None:
        config_kwargs["so101_calibration_dir"] = args.calibration_dir

    config = SO101B601LeaderConfig(**config_kwargs)
    teleop = SO101B601Leader(config)
    teleop.connect(calibrate=False)
    tol = config.startup_tolerance_deg
    try:
        while True:
            raw_action = teleop.leader.get_action()
            mapped = teleop.map_leader_action(raw_action)
            raw = " ".join(f"{value:+.1f}" for value in mapped.raw_leader_deg)
            target = " ".join(
                f"{value:+.1f}" for value in mapped.b601_target_deg
            )
            sent = " ".join(
                f"{value:+.1f}" for value in mapped.action.values()
            )
            startup_tag = ""
            if tol and tol > 0 and teleop._mapping_state is not None:
                max_diff = float(
                    np.max(
                        np.abs(
                            mapped.raw_leader_deg
                            - teleop._mapping_state.zero_leader_deg
                        )
                    )
                )
                startup_tag = (
                    " | STARTUP=OK"
                    if max_diff <= tol
                    else f" | STARTUP=OFF({max_diff:.1f}deg)"
                )
            print(
                f"raw=[{raw}] gripper={raw_action['gripper.pos']:+.1f} | "
                f"B601 target=[{target}] gripper={mapped.gripper_target_deg:+.1f} | "
                f"follower action=[{sent}]{startup_tag}",
                flush=True,
            )
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.disconnect()


if __name__ == "__main__":
    main()
