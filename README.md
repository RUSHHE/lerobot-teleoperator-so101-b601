# lerobot-teleoperator-so101-b601

[中文文档](./README.zh-CN.md)

A LeRobot (`>= 0.7.0`) teleoperator plugin that maps a **5-DOF + gripper SO-101 leader arm** (`so101_b601_leader`) onto the built-in **6-DOF + gripper Seeed reBot B601-RS follower arm** (`rebot_b601_follower` with `motor_family=rs`).

## Features

- **Cross-morphology joint mapping**: Maps SO-101 `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, and `wrist_roll` to B601 joints `1, 2, 3, 4, 6`, while holding B601 joint 5 (`wrist_yaw`) at a configurable fixed angle (default `0°`).
- **LeRobot v0.7.0 `RS_PROFILE` alignment**: Pre-transforms targets by `RS_PROFILE.joint_directions` (`-1.0` across all 7 joints) so built-in `RebotB601Follower` restores exact physical RobStride motor targets without sign inversion or clipping surprises.
- **Piecewise endpoint range mapping**: Independently maps each side of `zero_leader_deg` to the corresponding follower limit (`piecewise_range_map=True`), ensuring asymmetric joints such as `wrist_flex` reach both ends of B601 travel.
- **1:1 `wrist_roll` feel**: Applies a `2.0x` scale multiplier on `wrist_roll` so `±90°` leader rotation spans the full `±90°` B601 `wrist_roll` range.
- **Startup zero-pose safety guard**: Checks on the first `get_action()` frame that all 5 SO-101 arm joints are within `±15°` (`startup_tolerance_deg`) of the sit-down zero pose before returning any motor target.
- **Calibration directory compatibility**: Defaults to LeRobot v0.7.0 `teleoperators/so_leader/<id>.json` and falls back to legacy `teleoperators/so101_leader/<id>.json` if present.

## Joint Mapping

| SO-101 Joint | B601-RS Joint | Physical B601 Limits (deg) | Notes |
| --- | --- | --- | --- |
| `shoulder_pan` | `shoulder_pan` (J1) | `[-145, 145]` | Piecewise endpoint map |
| `shoulder_lift` | `shoulder_lift` (J2) | `[0, 170]` | Sit-down zero at `0°` |
| `elbow_flex` | `elbow_flex` (J3) | `[0, 200]` | Sit-down zero at `0°` |
| `wrist_flex` | `wrist_flex` (J4) | `[-80, 90]` | Piecewise map aligns asymmetric zero (`+56°`) |
| — | `wrist_yaw` (J5) | `[-90, 90]` | Held at `wrist_yaw_deg` (default `0°`) |
| `wrist_roll` | `wrist_roll` (J6) | `[-90, 90]` | `2.0x` multiplier (`±90°` → `±88°` w/ margin) |
| `gripper` | `gripper` (J7) | `[0, 270]` | SO-101 `0..100` → B601 `0..270°` |

## Installation

Activate your LeRobot `>= 0.7.0` environment and install in editable mode:

```bash
conda activate lerobot
git clone https://github.com/RUSHHE/lerobot-teleoperator-so101-b601.git
cd lerobot-teleoperator-so101-b601
pip install -e . --no-deps --no-build-isolation
```

Verify that LeRobot discovers the `so101_b601_leader` plugin:

```bash
lerobot-teleoperate --help | grep -E "so101_b601_leader|rebot_b601_follower"
```

## Calibration

Calibrate the follower and leader separately in LeRobot v0.7.0:

```bash
# Bring up SocketCAN (1 Mbps) and serial permissions
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo chmod 666 /dev/ttyACM*

# 1. Calibrate B601-RS follower (place in sit-down zero pose, gripper closed)
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.motor_family=rs \
  --robot.port=can0 \
  --robot.id=so101_b601_follower

# 2. Calibrate SO-101 leader (midpoint + full-range sweep)
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=so101_leader
```

## Pre-Power-On Check (Leader Only)

Before powering on the B601-RS follower, connect only the SO-101 leader, place it in the sit-down zero pose, and verify that `STARTUP=OK` and `B601 target` values are near zero:

```bash
python examples/read_mapped_action.py \
  --port /dev/ttyACM0 \
  --leader-id so101_leader \
  --mapping-config ./configs/so101_b601_mapping.json
```

## Teleoperation

Place both arms in the sit-down zero pose and launch:

```bash
./run_teleoperate.sh
```

Or invoke `lerobot-teleoperate` directly:

```bash
lerobot-teleoperate \
  --robot.type=rebot_b601_follower \
  --robot.motor_family=rs \
  --robot.port=can0 \
  --robot.id=so101_b601_follower \
  --robot.gripper_hold_torque_limit=3.5 \
  --teleop.type=so101_b601_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=so101_leader \
  --teleop.leader_id=so101_leader \
  --teleop.mapping_config=./configs/so101_b601_mapping.json \
  --fps=30
```

### With RealSense Cameras

Copy `.env.example` to `.env` (gitignored), fill in your camera serial numbers (`FRONT_CAM_SERIAL`, `SIDE_CAM_SERIAL`), and run:

```bash
WITH_CAMERAS=1 ./run_teleoperate.sh
```

If Rerun visualization lags over remote/high-load sessions, append `--display_compressed_images=true` or `--display_mode=foxglove`.

### Environment Overrides (`run_teleoperate.sh`)

- `CAN_PORT` (default `can0`)
- `LEADER_PORT` (default `/dev/ttyACM0`)
- `LEADER_ID` (default `so101_leader`)
- `FOLLOWER_ID` (default `so101_b601_follower`)
- `MAPPING_CONFIG` (default `./configs/so101_b601_mapping.json`)
- `GRIPPER_HOLD_TORQUE` (default `3.5`)
- `MAX_RELATIVE_TARGET` (unset by default; set e.g. `5.0` if needed)
- `FPS` (default `30`)
- `WITH_CAMERAS` (`0` or `1`, default `0`)

## Testing

Run the offline unit test suite (no hardware required):

```bash
python -m unittest discover -s tests -v
```

## Hardware & Safety Notes

- **Do not use raw `--teleop.type=so101_leader` directly with `rebot_b601_follower`**: Their zero poses (`-104°` vs `0°` on `shoulder_lift`) and joint signs differ, which can command an immediate full-speed lift on power-up. Always use `so101_b601_leader`.
- **`max_relative_target` in MIT mode**: Setting a tight per-step limit (e.g. `2°–3°`) caps MIT position error and significantly reduces joint torque, making wrists and gripper feel sluggish. Keep it unset (or `>= 10°`) and rely on the startup zero-pose guard.
- **`wrist_yaw` (J5)**: Held fixed at `wrist_yaw_deg` (`0°`), so datasets recorded with SO-101 should not be mixed directly with 7-DOF leader datasets (`rebot_102_leader`).

## Repository Layout

```text
lerobot-teleoperator-so101-b601/
├── configs/
│   └── so101_b601_mapping.json        # Reference mapping configuration
├── examples/
│   ├── probe_motor.py                 # Single RobStride motor MIT probe utility
│   └── read_mapped_action.py          # Leader-only read-only mapping check
├── lerobot_teleoperator_so101_b601/
│   ├── __init__.py
│   ├── config_so101_b601_leader.py    # TeleoperatorConfig registration & defaults
│   ├── mapping.py                     # Joint/gripper mapping & direction pre-transform
│   └── so101_b601_leader.py           # SO101B601Leader implementation & startup guard
├── tests/
│   ├── test_mapping.py
│   └── test_official_follower.py
├── .env.example
├── .gitignore
├── LICENSE
├── pyproject.toml
├── README.md
├── README.zh-CN.md
└── run_teleoperate.sh
```

## License

[MIT](./LICENSE)
