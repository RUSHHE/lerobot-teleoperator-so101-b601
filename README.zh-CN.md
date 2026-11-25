# lerobot-teleoperator-so101-b601

[English](./README.md)

面向 LeRobot（`>= 0.7.0`）的第三方遥操作插件：将 **5 自由度 + 夹爪的 SO-101 主臂**（`so101_b601_leader`）适配到 LeRobot 内置的 **6 自由度 + 夹爪 Seeed reBot B601-RS 从臂**（`rebot_b601_follower`，`motor_family=rs`）。

## 主要特性

- **跨构型关节映射**：将 SO-101 的 `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll` 映射到 B601 的 `1, 2, 3, 4, 6` 关节；B601 缺失对应的 5 号关节 `wrist_yaw` 固定在配置零位（默认 `0°`）。
- **对齐 LeRobot v0.7.0 `RS_PROFILE`**：输出前自动按 `RS_PROFILE.joint_directions`（全 `-1.0`）做预变换，内置 `RebotB601Follower` 接收后还原出准确的物理 RobStride 电机目标角度。
- **分段端点对齐映射**：以 `zero_leader_deg` 为锚点，分别对齐左右两侧行程到从臂安全限位（`piecewise_range_map=True`），解决 `wrist_flex` 等零位偏心关节一侧行程不足的问题。
- **`wrist_roll` 1:1 手感**：对 `wrist_roll` 应用 `2.0x` 倍率，使主臂转 `±90°` 即覆盖从臂完整 `±90°` 行程。
- **启动零位防呆保护**：首帧 `get_action()` 自动校验 SO-101 五个手臂关节是否在趴下零位 `±15°`（`startup_tolerance_deg`）内，未摆好零位会直接拦截并报错，避免上电猛抬或跳变。
- **标定目录兼容**：默认读取 v0.7.0 的 `teleoperators/so_leader/<id>.json`，若不存在则自动回退兼容旧版 `teleoperators/so101_leader/<id>.json`。

## 关节映射表

| SO-101 关节 | B601-RS 关节 | B601 物理限位 (deg) | 说明 |
| --- | --- | --- | --- |
| `shoulder_pan` | `shoulder_pan` (J1) | `[-145, 145]` | 分段端点映射 |
| `shoulder_lift` | `shoulder_lift` (J2) | `[0, 170]` | 趴下零位在 `0°` |
| `elbow_flex` | `elbow_flex` (J3) | `[0, 200]` | 趴下零位在 `0°` |
| `wrist_flex` | `wrist_flex` (J4) | `[-80, 90]` | 分段映射对齐偏心零位（`+56°`） |
| — | `wrist_yaw` (J5) | `[-90, 90]` | 保持固定值 `wrist_yaw_deg`（默认 `0°`） |
| `wrist_roll` | `wrist_roll` (J6) | `[-90, 90]` | `2.0x` 倍率（`±90°` → 含裕度 `±88°`） |
| `gripper` | `gripper` (J7) | `[0, 270]` | SO-101 `0..100` → B601 `0..270°` |

## 安装

激活 LeRobot（`>= 0.7.0`）环境后以可编辑模式安装：

```bash
conda activate lerobot
git clone https://github.com/RUSHHE/lerobot-teleoperator-so101-b601.git
cd lerobot-teleoperator-so101-b601
pip install -e . --no-deps --no-build-isolation
```

确认 LeRobot 已自动发现并注册 `so101_b601_leader`：

```bash
lerobot-teleoperate --help | grep -E "so101_b601_leader|rebot_b601_follower"
```

## 标定

在 LeRobot v0.7.0 下分别对从臂和主臂标定：

```bash
# 启用 SocketCAN (1 Mbps) 并放开串口权限
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo chmod 666 /dev/ttyACM*

# 1. 标定 B601-RS 从臂（摆至官方 sit-down 零位，夹爪完全闭合）
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.motor_family=rs \
  --robot.port=can0 \
  --robot.id=so101_b601_follower

# 2. 标定 SO-101 主臂（先中位按 Enter，再转过各关节完整行程）
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=so101_leader
```

## 上电前只读检查（仅插主臂）

给 B601-RS 上电前，建议先只连接 SO-101 主臂，摆回趴下零位后确认 `STARTUP=OK` 且 `B601 target` 各关节接近 `0°`：

```bash
python examples/read_mapped_action.py \
  --port /dev/ttyACM0 \
  --leader-id so101_leader \
  --mapping-config ./configs/so101_b601_mapping.json
```

## 遥操作

将两臂摆回趴下零位后运行：

```bash
./run_teleoperate.sh
```

或直接调用 `lerobot-teleoperate`：

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

### 开启 RealSense 相机画面

将 `.env.example` 复制为 `.env`（已被 `.gitignore` 忽略），填入相机序列号（`FRONT_CAM_SERIAL`、`SIDE_CAM_SERIAL`）后运行：

```bash
WITH_CAMERAS=1 ./run_teleoperate.sh
```

若画面带宽压力大或远程显示卡顿，可追加 `--display_compressed_images=true` 或 `--display_mode=foxglove`。

### 环境变量覆盖（`run_teleoperate.sh`）

- `CAN_PORT`（默认 `can0`）
- `LEADER_PORT`（默认 `/dev/ttyACM0`）
- `LEADER_ID`（默认 `so101_leader`）
- `FOLLOWER_ID`（默认 `so101_b601_follower`）
- `MAPPING_CONFIG`（默认 `./configs/so101_b601_mapping.json`）
- `GRIPPER_HOLD_TORQUE`（默认 `3.5`）
- `MAX_RELATIVE_TARGET`（默认不限制；如需限步可设为如 `5.0`）
- `FPS`（默认 `30`）
- `WITH_CAMERAS`（`0` 或 `1`，默认 `0`）

## 运行测试

纯离线运行全部单元测试（无需连接任何硬件）：

```bash
python -m unittest discover -s tests -v
```

## 安全与硬件注意事项

- **不要直接用原生 `--teleop.type=so101_leader` 驱动 `rebot_b601_follower`**：两者零位（2 号关节 `-104°` vs `0°`）与正负方向相反，上电会直接猛抬大臂。务必使用本插件 `so101_b601_leader`。
- **MIT 模式下的 `max_relative_target`**：设置过小的单步限幅（如 `2°~3°`）会限制位置误差，从而大幅压低 MIT 输出力矩，导致手腕和夹爪发钝。默认不开启该限制，依靠启动零位防呆保护即可。
- **`wrist_yaw`（J5）锁定**：由于 SO-101 无对应自由度，B601 第 5 关节固定在 `wrist_yaw_deg`（`0°`），本组合采集的数据不建议与 7 自由度主臂（如 `rebot_102_leader`）的数据混训。

## 仓库结构

```text
lerobot-teleoperator-so101-b601/
├── configs/
│   └── so101_b601_mapping.json        # 参考映射配置文件
├── examples/
│   ├── probe_motor.py                 # 单电机 RobStride MIT 调试脚本
│   └── read_mapped_action.py          # 仅接主臂的只读映射检查工具
├── lerobot_teleoperator_so101_b601/
│   ├── __init__.py
│   ├── config_so101_b601_leader.py    # TeleoperatorConfig 注册与默认参数
│   ├── mapping.py                     # 关节/夹爪映射与方向预变换
│   └── so101_b601_leader.py           # SO101B601Leader 实现与启动防呆
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

## 开源协议

[MIT](./LICENSE)
