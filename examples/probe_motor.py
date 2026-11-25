#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

from motorbridge import Controller, Mode


MODELS = {
    3: "rs-06",
    4: "rs-00",
    7: "rs-00",
}


def read_position(motor) -> float:
    state = motor.get_state()
    if state is not None:
        return float(state.pos)
    return float(motor.robstride_get_param_f32(0x7019, 300))


def poll_position(ctrl: Controller, motor) -> float:
    motor.request_feedback()
    try:
        ctrl.poll_feedback_once()
    except Exception:
        pass
    return read_position(motor)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send a small direct MIT target to one B601 motor."
    )
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--motor-id", type=int, choices=sorted(MODELS), required=True)
    parser.add_argument("--delta-deg", type=float, default=5.0)
    parser.add_argument("--absolute-deg", type=float)
    parser.add_argument("--kp", type=float)
    parser.add_argument("--kd", type=float)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument(
        "--no-return",
        action="store_true",
        help="Leave the motor at the probe target instead of returning to start",
    )
    args = parser.parse_args()

    if args.absolute_deg is None and abs(args.delta_deg) > 20.0:
        raise ValueError("--delta-deg must be in [-20, 20] for this probe")
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")

    motor_id = args.motor_id
    kp = args.kp
    kd = args.kd
    if kp is None:
        kp = 12.0 if motor_id == 7 else 50.0
    if kd is None:
        kd = 0.05 if motor_id == 7 else 5.0

    ctrl = Controller(channel=args.channel)
    motor = None
    start = None
    try:
        motor = ctrl.add_robstride_motor(motor_id, 0xFD, MODELS[motor_id])
        motor.robstride_set_active_report(1)
        ctrl.disable_all()
        motor.ensure_mode(Mode.MIT, 1000)
        ctrl.enable_all()

        start = poll_position(ctrl, motor)
        target = (
            math.radians(args.absolute_deg)
            if args.absolute_deg is not None
            else start + math.radians(args.delta_deg)
        )
        print(
            f"[probe] id={motor_id} start={math.degrees(start):+.3f}deg "
            f"target={math.degrees(target):+.3f}deg "
            f"delta={args.delta_deg:+.1f}deg kp={kp:.2f} kd={kd:.2f}",
            flush=True,
        )

        end = time.monotonic() + args.seconds
        last_report = 0.0
        while time.monotonic() < end:
            motor.send_mit(target, 0.0, kp, kd, 0.0)
            actual = poll_position(ctrl, motor)
            now = time.monotonic()
            if now - last_report >= 0.25:
                print(
                    f"[probe] actual={math.degrees(actual):+.3f}deg "
                    f"error={math.degrees(target - actual):+.3f}deg",
                    flush=True,
                )
                last_report = now
            time.sleep(0.02)
    finally:
        if motor is not None:
            try:
                if start is not None and not args.no_return:
                    print(
                        f"[probe] returning to start={math.degrees(start):+.3f}deg",
                        flush=True,
                    )
                    for _ in range(25):
                        motor.send_mit(start, 0.0, kp, kd, 0.0)
                        poll_position(ctrl, motor)
                        time.sleep(0.02)
                    motor.send_mit(start, 0.0, 0.0, 0.0, 0.0)
                else:
                    actual = poll_position(ctrl, motor)
                    motor.send_mit(actual, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                pass
            try:
                ctrl.disable_all()
            except Exception:
                pass
            motor.close()
        try:
            ctrl.shutdown()
        except Exception:
            pass
        ctrl.close()


if __name__ == "__main__":
    main()
