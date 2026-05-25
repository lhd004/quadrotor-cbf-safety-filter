"""
PX4 SITL offboard smoke test for the UAV CBF project.

Run this on the Ubuntu server while PX4 SITL + Gazebo is running:

    cd ~/PX4-Autopilot
    HEADLESS=1 make px4_sitl gz_x500

Then in another terminal:

    cd ~/Thesis_2
    python3 scripts/px4_cbf_offboard.py

The controller uses MAVLink offboard setpoints. It commands a constant altitude
and horizontal velocity. A simple CBF velocity filter keeps the commanded
horizontal velocity from entering a circular obstacle in local NED coordinates.
This is an integration test, not the final paper controller.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "output" / "px4_offboard"
UINT32_MAX = 2**32


@dataclass
class LocalState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    updated: bool = False


@dataclass
class VehicleStatus:
    mode: str = "UNKNOWN"
    armed: bool = False
    last_statustext: str = ""
    last_command_ack: str = ""


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udpin:0.0.0.0:14540")
    parser.add_argument("--altitude", type=float, default=2.0, help="Target altitude in meters.")
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--goal-x", type=float, default=6.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--obstacle-x", type=float, default=3.0)
    parser.add_argument("--obstacle-y", type=float, default=0.0)
    parser.add_argument("--safe-radius", type=float, default=1.15)
    parser.add_argument(
        "--obstacles",
        default="",
        help="Semicolon-separated obstacle list 'x,y,r;x,y,r'. Overrides --obstacle-x/y/safe-radius.",
    )
    parser.add_argument("--speed", type=float, default=0.65)
    parser.add_argument(
        "--max-speed",
        type=float,
        default=None,
        help="Maximum horizontal command speed after CBF correction. Defaults to --speed.",
    )
    parser.add_argument("--alpha", type=float, default=1.2)
    parser.add_argument("--margin", type=float, default=0.08)
    parser.add_argument("--wind-vx", type=float, default=0.0, help="Wind-equivalent velocity bias in local NED x [m/s].")
    parser.add_argument("--wind-vy", type=float, default=0.0, help="Wind-equivalent velocity bias in local NED y [m/s].")
    parser.add_argument("--output", default=str(OUT_DIR / "offboard_cbf_log.csv"))
    parser.add_argument("--scenario", default="single_obstacle")
    parser.add_argument("--controller", default="", help="Optional label written to the CSV log.")
    parser.add_argument("--no-gcs-heartbeat", action="store_true")
    parser.add_argument("--disable-cbf", action="store_true", help="Send nominal velocity without CBF filtering.")
    parser.add_argument("--diagnostics", action="store_true", help="Print MAVLink heartbeat, ACK, and STATUSTEXT details.")
    return parser.parse_args()


def parse_obstacles(args: argparse.Namespace) -> list[Obstacle]:
    if not args.obstacles.strip():
        return [Obstacle(args.obstacle_x, args.obstacle_y, args.safe_radius)]
    obstacles: list[Obstacle] = []
    for raw_item in args.obstacles.split(";"):
        item = raw_item.strip()
        if not item:
            continue
        parts = [float(value.strip()) for value in item.split(",")]
        if len(parts) != 3:
            raise ValueError(f"Invalid obstacle specification {item!r}; expected x,y,r")
        obstacles.append(Obstacle(parts[0], parts[1], parts[2]))
    if not obstacles:
        raise ValueError("--obstacles was provided but no valid obstacle was parsed")
    return obstacles


def target_component(master: mavutil.mavfile) -> int:
    return int(master.target_component) if int(master.target_component) > 0 else 1


def send_gcs_heartbeat(master: mavutil.mavfile) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def decode_px4_mode(msg: object) -> str:
    mode = mavutil.mode_string_v10(msg)
    if mode != "UNKNOWN":
        return mode
    main_mode = (int(msg.custom_mode) >> 16) & 0xFF
    px4_modes = {
        1: "MANUAL",
        2: "ALTCTL",
        3: "POSCTL",
        4: "AUTO",
        5: "ACRO",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE",
    }
    return px4_modes.get(main_mode, f"UNKNOWN({int(msg.custom_mode)})")


def request_message_interval(master: mavutil.mavfile, message_id: int, hz: float) -> None:
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system,
        target_component(master),
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def update_from_messages(
    master: mavutil.mavfile,
    state: LocalState,
    status: VehicleStatus,
    max_reads: int = 20,
    verbose: bool = False,
) -> None:
    for _ in range(max_reads):
        msg = master.recv_match(blocking=False)
        if msg is None:
            return
        msg_type = msg.get_type()
        if msg_type == "LOCAL_POSITION_NED":
            state.x = float(msg.x)
            state.y = float(msg.y)
            state.z = float(msg.z)
            state.vx = float(msg.vx)
            state.vy = float(msg.vy)
            state.vz = float(msg.vz)
            state.updated = True
        elif msg_type == "HEARTBEAT":
            status.mode = decode_px4_mode(msg)
            status.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if verbose:
                print(f"HEARTBEAT mode={status.mode}, armed={status.armed}, custom_mode={msg.custom_mode}")
        elif msg_type == "STATUSTEXT":
            status.last_statustext = str(msg.text)
            if verbose:
                print(f"STATUSTEXT severity={msg.severity}: {msg.text}")
        elif msg_type == "COMMAND_ACK":
            command = mavutil.mavlink.enums["MAV_CMD"].get(msg.command)
            command_name = command.name if command else str(msg.command)
            result = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result)
            result_name = result.name if result else str(msg.result)
            status.last_command_ack = f"{command_name}: {result_name}"
            if verbose:
                print(f"COMMAND_ACK {status.last_command_ack}")


def send_position_setpoint(master: mavutil.mavfile, x: float, y: float, z: float, yaw: float = 0.0) -> None:
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) % UINT32_MAX,
        master.target_system,
        target_component(master),
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        x,
        y,
        z,
        0,
        0,
        0,
        0,
        0,
        0,
        yaw,
        0,
    )


def send_velocity_altitude_setpoint(
    master: mavutil.mavfile,
    z: float,
    vx: float,
    vy: float,
    vz: float,
    yaw: float = 0.0,
) -> None:
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) % UINT32_MAX,
        master.target_system,
        target_component(master),
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        0,
        0,
        z,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        yaw,
        0,
    )


def set_mode(master: mavutil.mavfile, mode_name: str) -> None:
    px4_main_modes = {
        "MANUAL": 1,
        "ALTCTL": 2,
        "POSCTL": 3,
        "AUTO": 4,
        "ACRO": 5,
        "OFFBOARD": 6,
        "STABILIZED": 7,
        "RATTITUDE": 8,
    }
    if mode_name not in px4_main_modes:
        raise RuntimeError(f"Unsupported PX4 mode {mode_name!r}")
    custom_mode = px4_main_modes[mode_name] << 16
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
    )


def arm(master: mavutil.mavfile) -> None:
    master.mav.command_long_send(
        master.target_system,
        target_component(master),
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def land(master: mavutil.mavfile) -> None:
    master.mav.command_long_send(
        master.target_system,
        target_component(master),
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def cbf_velocity_single(
    p_xy: np.ndarray,
    v_des: np.ndarray,
    obstacle: Obstacle,
    alpha: float,
    margin: float,
) -> tuple[np.ndarray, float, float]:
    rel = p_xy - np.array([obstacle.x, obstacle.y], dtype=float)
    h = float(rel @ rel - obstacle.radius**2)
    a = 2.0 * rel
    b = -alpha * h + margin
    lhs = float(a @ v_des)
    if lhs >= b or float(a @ a) < 1e-9:
        return v_des, h, 0.0
    correction = (b - lhs) / float(a @ a) * a
    return v_des + correction, h, float(np.linalg.norm(correction))


def cbf_velocity_multi(
    p_xy: np.ndarray,
    v_des: np.ndarray,
    obstacles: list[Obstacle],
    alpha: float,
    margin: float,
    passes: int = 3,
) -> tuple[np.ndarray, float, float]:
    v_safe = v_des.copy()
    total_correction = 0.0
    min_h = math.inf
    for _ in range(passes):
        changed = False
        for obstacle in obstacles:
            before = v_safe.copy()
            v_safe, h, _ = cbf_velocity_single(p_xy, v_safe, obstacle, alpha, margin)
            min_h = min(min_h, h)
            step_correction = float(np.linalg.norm(v_safe - before))
            total_correction += step_correction
            changed = changed or step_correction > 1e-9
        if not changed:
            break
    if not math.isfinite(min_h):
        min_h = 0.0
    return v_safe, min_h, total_correction


def main() -> None:
    args = parse_args()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    master = mavutil.mavlink_connection(args.connection)
    print(f"Waiting for PX4 heartbeat on {args.connection} ...")
    master.wait_heartbeat(timeout=30)
    print(f"Heartbeat from system={master.target_system}, component={master.target_component}")

    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, args.rate)
    state = LocalState()
    status = VehicleStatus()

    deadline = time.time() + 15
    while time.time() < deadline and not state.updated:
        if not args.no_gcs_heartbeat:
            send_gcs_heartbeat(master)
        update_from_messages(master, state, status, max_reads=50, verbose=args.diagnostics)
        time.sleep(0.05)
    if not state.updated:
        raise RuntimeError("No LOCAL_POSITION_NED received from PX4.")

    target_z = -abs(args.altitude)
    dt = 1.0 / args.rate
    command_speed_limit = args.speed if args.max_speed is None else args.max_speed

    print("Priming offboard setpoints ...")
    for _ in range(int(2.0 * args.rate)):
        if not args.no_gcs_heartbeat:
            send_gcs_heartbeat(master)
        update_from_messages(master, state, status)
        send_position_setpoint(master, state.x, state.y, target_z)
        time.sleep(dt)

    print("Switching to OFFBOARD and arming ...")
    set_mode(master, "OFFBOARD")
    arm(master)
    deadline = time.time() + 8
    while time.time() < deadline and not (status.mode == "OFFBOARD" and status.armed):
        if not args.no_gcs_heartbeat:
            send_gcs_heartbeat(master)
        update_from_messages(master, state, status, max_reads=100, verbose=True)
        send_position_setpoint(master, state.x, state.y, target_z)
        time.sleep(dt)
    print(f"Vehicle status after arm request: mode={status.mode}, armed={status.armed}")
    if status.mode != "OFFBOARD" or not status.armed:
        detail = status.last_statustext or status.last_command_ack or "no PX4 diagnostic message received"
        raise RuntimeError(f"PX4 did not enter armed OFFBOARD state: {detail}")

    obstacles = parse_obstacles(args)
    controller_name = args.controller or ("baseline_no_cbf" if args.disable_cbf else "cbf_velocity_filter")
    goal = np.array([args.goal_x, args.goal_y], dtype=float)

    rows = []
    start = time.time()
    last_print = start
    try:
        while time.time() - start < args.duration:
            loop_start = time.time()
            if not args.no_gcs_heartbeat:
                send_gcs_heartbeat(master)
            update_from_messages(master, state, status, verbose=args.diagnostics)

            p_xy = np.array([state.x, state.y], dtype=float)
            to_goal = goal - p_xy
            distance = float(np.linalg.norm(to_goal))
            if distance > 0.2:
                v_des_xy = args.speed * to_goal / max(distance, 1e-6)
            else:
                v_des_xy = np.zeros(2)
            wind_xy = np.array([args.wind_vx, args.wind_vy], dtype=float)
            v_disturbed_xy = v_des_xy + wind_xy

            v_safe_xy, h, correction = cbf_velocity_multi(
                p_xy,
                v_disturbed_xy,
                obstacles,
                args.alpha,
                args.margin,
            )
            if args.disable_cbf:
                v_safe_xy = v_disturbed_xy
                correction = 0.0
            v_norm = float(np.linalg.norm(v_safe_xy))
            if v_norm > command_speed_limit:
                v_safe_xy = command_speed_limit * v_safe_xy / v_norm

            vz = 0.7 * (target_z - state.z)
            vz = float(np.clip(vz, -0.5, 0.5))

            send_velocity_altitude_setpoint(master, target_z, float(v_safe_xy[0]), float(v_safe_xy[1]), vz)

            t = time.time() - start
            rows.append(
                {
                    "t": t,
                    "scenario": args.scenario,
                    "controller": controller_name,
                    "obstacles": ";".join(f"{obs.x:.3f},{obs.y:.3f},{obs.radius:.3f}" for obs in obstacles),
                    "x": state.x,
                    "y": state.y,
                    "z": state.z,
                    "vx": state.vx,
                    "vy": state.vy,
                    "vz": state.vz,
                    "v_cmd_x": float(v_safe_xy[0]),
                    "v_cmd_y": float(v_safe_xy[1]),
                    "v_des_x": float(v_des_xy[0]),
                    "v_des_y": float(v_des_xy[1]),
                    "wind_vx": float(wind_xy[0]),
                    "wind_vy": float(wind_xy[1]),
                    "v_disturbed_x": float(v_disturbed_xy[0]),
                    "v_disturbed_y": float(v_disturbed_xy[1]),
                    "h": h,
                    "correction": correction,
                    "goal_distance": distance,
                    "cbf_enabled": not args.disable_cbf,
                }
            )

            if time.time() - last_print > 2.0:
                print(
                    f"t={t:5.1f}s p=({state.x: .2f},{state.y: .2f},{state.z: .2f}) "
                    f"h={h: .3f} corr={correction:.3f}"
                )
                last_print = time.time()

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))
    finally:
        print("Sending LAND command ...")
        land(master)

        with output_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["t"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"Wrote {len(rows)} rows to {output_path}")

        if rows:
            min_h = min(row["h"] for row in rows)
            final = rows[-1]
            print(f"min_h={min_h:.6f}, final=({final['x']:.3f}, {final['y']:.3f}, {final['z']:.3f})")


if __name__ == "__main__":
    main()
