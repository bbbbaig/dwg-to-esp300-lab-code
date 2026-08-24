from __future__ import annotations

import argparse
import json
import math
import re
import time

import serial


AXIS_X = 1
AXIS_Y = 2
AXIS_Z = 3

DEFAULT_PORT = 'COM5'
DEFAULT_BAUDRATE = 19200
DRAW_SPEED = 0.4
MOVE_SPEED = 1.0
Z_SPEED = 0.3
PASSES = 1
POSITION_TOLERANCE = 0.003
SEND_SETTLE = 0.05
READ_POLLS = 20
READ_DELAY = 0.03
TP_READ_RETRIES = 5
TP_RETRY_DELAY = 0.08
POST_MOVE_SETTLE = 0.05
Y_AXIS_EXTRA_SETTLE = 0.05
DRY_RUN_DEFAULT = True
WAIT_MODE = 'position'
Z_DEFOCUS_MM = 1.0
Z_DEFOCUS_DIRECTION = -1.0
Z_STEP_BACK_PASSES = 10
Z_STEP_BACK_MM = 0.001
POSITION_PATTERN = re.compile(r"[-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?")

# Fill these in only if the ESP300 or another serial device controls laser TTL.
LASER_ON_CMD = None
LASER_OFF_CMD = None

METADATA = json.loads('{"source":"x","unit_scale":1.0,"origin":[12.5,12.5],"glass_size_mm":[25,25],"start_xy_mm":[12.5,12.5],"usable_diameter_mm":25.0,"usable_center_xy_mm":[12.5,12.5],"usable_margin_mm":0.05,"feed_rate":0.4,"travel_rate":1.0,"focus_z_mm":0.0,"z_defocus_mm":1.0,"z_speed_mm_s":0.3,"position_tolerance_mm":0.003,"passes":1,"path_count":1,"point_count":2,"total_length_mm":1.0}')
PATHS = json.loads('[{"layer":"L","entity_type":"LINE","closed":false,"points":[[0.0,0.0],[1.0,1.0]]}]')


def metadata_pair(name: str, default: tuple[float, float]) -> tuple[float, float]:
    raw = METADATA.get(name, default)
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return float(raw[0]), float(raw[1])
    return float(default[0]), float(default[1])


START_XY = metadata_pair("start_xy_mm", (0.0, 0.0))
ser = None
DRY_RUN = DRY_RUN_DEFAULT
VERBOSE = True
CURRENT_XY = START_XY
CURRENT_Z = None
FOCUS_Z = None
Z_IS_DEFOCUSED = False
CURRENT_PASS_RETREAT = 0.0
LASER_IS_ON = None


def connect(port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE, dry_run: bool = DRY_RUN_DEFAULT):
    """Same connection style as the controller notebook, but safe to dry-run."""
    global ser, DRY_RUN
    DRY_RUN = dry_run
    if DRY_RUN:
        print("Dry run: serial connection not opened")
        return None
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=2,
        rtscts=True,
    )
    print("Connected to:", ser.name)
    return ser


def close():
    global ser
    if ser and ser.is_open:
        ser.close()


def write_command(cmd: str, settle: float = SEND_SETTLE):
    global ser
    if VERBOSE:
        print(f">>> Sending: {cmd}")
    if DRY_RUN:
        return
    if ser is None or not ser.is_open:
        raise RuntimeError("Serial port is not open. Call connect(..., dry_run=False) first.")

    ser.write((cmd + "\r\n").encode("ascii"))
    if settle > 0:
        time.sleep(settle)


def query(cmd: str):
    global ser
    if VERBOSE:
        print(f">>> Sending: {cmd}")
    if DRY_RUN:
        return "0"
    if ser is None or not ser.is_open:
        raise RuntimeError("Serial port is not open. Call connect(..., dry_run=False) first.")

    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    time.sleep(SEND_SETTLE)

    # Wait for a line terminator before treating the reply as complete. A
    # reply spanning multiple USB-serial read chunks can otherwise be read
    # as complete after only a partial chunk, leaving trailing bytes in the
    # buffer that corrupt the next query's response (seen as axis position
    # spikes/jumps on the controller).
    response = b""
    for _ in range(READ_POLLS):
        if ser.in_waiting > 0:
            response += ser.read(ser.in_waiting)
            if b"\r" in response or b"\n" in response:
                time.sleep(0.005)
                if ser.in_waiting > 0:
                    response += ser.read(ser.in_waiting)
                break
        time.sleep(READ_DELAY)

    reply = response.decode("ascii", errors="ignore").strip()
    if VERBOSE:
        print(f"<<< Received: '{reply}'")
    return reply


def send(cmd: str):
    return query(cmd)


def parse_position_reply(reply: str, axis: int) -> float | None:
    text = (reply or "").strip()
    if not text:
        return None

    text = re.sub(rf"^{axis}\s*TP", "", text, flags=re.IGNORECASE).strip()
    tokens = [token.strip() for token in re.split(r"[\s,;]+", text) if token.strip()]
    numeric_tokens = [token for token in tokens if POSITION_PATTERN.fullmatch(token)]
    if numeric_tokens:
        return float(numeric_tokens[-1])

    if "TP" not in text.upper():
        matches = POSITION_PATTERN.findall(text)
        if matches:
            return float(matches[-1])
    return None


def motor_on(axis: int):
    write_command(f"{axis}MO")


def motor_off(axis: int):
    write_command(f"{axis}MF")


def get_position(axis: int, retries: int = TP_READ_RETRIES, retry_delay: float = TP_RETRY_DELAY) -> float:
    last_reply = ""
    for attempt in range(1, max(1, int(retries)) + 1):
        last_reply = query(f"{axis}TP")
        parsed = parse_position_reply(last_reply, axis)
        # Discard out-of-range values (parsing/framing artifacts) rather
        # than accepting them; the configured travel range is well under
        # 200 mm on every axis.
        if parsed is not None and abs(parsed) <= 250.0:
            return parsed
        if VERBOSE:
            print(f"TP read retry {attempt}/{retries} for axis {axis}; reply={last_reply!r}")
        if attempt < retries:
            time.sleep(max(0.0, retry_delay))
    raise RuntimeError(f"Could not read axis {axis} position after {retries} TP attempts; last reply={last_reply!r}")


def move_abs(axis: int, position: float, settle: float = SEND_SETTLE):
    write_command(f"{axis}PA{position:.6f}", settle=settle)


def move_rel(axis: int, distance: float, settle: float = SEND_SETTLE):
    write_command(f"{axis}PR{distance:.6f}", settle=settle)


def set_velocity(axis: int, velocity: float, settle: float = SEND_SETTLE):
    write_command(f"{axis}VA{max(float(velocity), 0.001):.6f}", settle=settle)


def wait_axis_position(axis: int, target: float, timeout_s: float):
    if DRY_RUN:
        return
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            if abs(get_position(axis) - target) <= POSITION_TOLERANCE:
                return
        except RuntimeError as exc:
            last_error = exc
        time.sleep(0.05)
    detail = f"; last TP error: {last_error}" if last_error else ""
    raise TimeoutError(f"Timed out waiting for axis {axis} at {target:.4f}{detail}")


def wait_xy_position(x: float, y: float, timeout_s: float):
    if DRY_RUN:
        return
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            current_x = get_position(AXIS_X)
            current_y = get_position(AXIS_Y)
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.05)
            continue
        if abs(current_x - x) <= POSITION_TOLERANCE and abs(current_y - y) <= POSITION_TOLERANCE:
            return
        time.sleep(0.05)
    detail = f"; last TP error: {last_error}" if last_error else ""
    raise TimeoutError(f"Timed out waiting for X={x:.4f}, Y={y:.4f}{detail}")


def laser_on():
    global LASER_IS_ON
    if LASER_IS_ON is True:
        return
    if LASER_ON_CMD:
        write_command(LASER_ON_CMD)
    else:
        print("Laser ON")
    LASER_IS_ON = True


def laser_off(force: bool = False):
    global LASER_IS_ON
    if LASER_IS_ON is False and not force:
        return
    if LASER_OFF_CMD:
        write_command(LASER_OFF_CMD)
    else:
        print("Laser OFF")
    LASER_IS_ON = False


def point_distance(a, b) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def is_closed_path(laser_path: dict, points: list[tuple[float, float]]) -> bool:
    return bool(laser_path.get("closed")) or (len(points) > 2 and point_distance(points[0], points[-1]) <= 1e-6)


def path_pass_count(laser_path: dict, passes_override: int | None) -> int:
    if passes_override is not None:
        return max(0, int(passes_override))
    if PASSES is not None:
        return max(0, int(PASSES))
    return max(0, int(laser_path.get("passes", 1)))


def set_focus_reference(z: float | None = None):
    global FOCUS_Z, CURRENT_Z, Z_IS_DEFOCUSED
    if z is None:
        z = 0.0 if DRY_RUN else get_position(AXIS_Z)
    FOCUS_Z = float(z)
    CURRENT_Z = FOCUS_Z
    Z_IS_DEFOCUSED = False
    print(f"Stage focus Z reference: {FOCUS_Z:.6f} mm")


def focus_target_z() -> float:
    global FOCUS_Z
    if FOCUS_Z is None:
        set_focus_reference()
    # Applies a cumulative Z retreat (defocus direction) at
    # Z_STEP_BACK_PASSES-pass intervals to offset material recession over
    # repeated passes.
    return float(FOCUS_Z) + float(Z_DEFOCUS_DIRECTION) * float(CURRENT_PASS_RETREAT)


def defocus_target_z() -> float:
    return focus_target_z() + (float(Z_DEFOCUS_DIRECTION) * abs(float(Z_DEFOCUS_MM)))


def move_z_abs(position: float, speed: float = Z_SPEED):
    global CURRENT_Z
    target = float(position)
    current = target if CURRENT_Z is None else float(CURRENT_Z)
    distance_z = abs(target - current)
    if distance_z <= POSITION_TOLERANCE:
        CURRENT_Z = target
        return target

    move_time = distance_z / max(float(speed), 0.001)
    set_velocity(AXIS_Z, speed)
    started_at = time.monotonic()
    move_abs(AXIS_Z, target)
    if WAIT_MODE == "position":
        wait_axis_position(AXIS_Z, target, timeout_s=max(5.0, move_time + 2.0))
        if not DRY_RUN:
            time.sleep(max(0.0, move_time - (time.monotonic() - started_at)))
    elif not DRY_RUN:
        time.sleep(move_time + SEND_SETTLE)
    if not DRY_RUN:
        time.sleep(POST_MOVE_SETTLE)
    CURRENT_Z = target
    return target


def defocus_z():
    global Z_IS_DEFOCUSED
    if Z_DEFOCUS_MM <= 0:
        return focus_target_z()
    target = defocus_target_z()
    print(f"Stage Z defocus: {target:.6f} mm")
    move_z_abs(target, Z_SPEED)
    Z_IS_DEFOCUSED = True
    return target


def focus_z():
    global Z_IS_DEFOCUSED
    target = focus_target_z()
    print(f"Stage Z focus: {target:.6f} mm")
    move_z_abs(target, Z_SPEED)
    Z_IS_DEFOCUSED = False
    return target


def move_xy_abs(x: float, y: float, speed: float, settle_after: bool = False):
    """Move X/Y with synchronized axis velocities for straight vector motion."""
    global CURRENT_XY
    target = (float(x), float(y))
    dist = point_distance(CURRENT_XY, target)
    if dist <= POSITION_TOLERANCE:
        CURRENT_XY = target
        return target

    dx = target[0] - CURRENT_XY[0]
    dy = target[1] - CURRENT_XY[1]
    move_x = abs(dx) > POSITION_TOLERANCE
    move_y = abs(dy) > POSITION_TOLERANCE
    move_time = dist / max(float(speed), 0.001)

    if move_x:
        set_velocity(AXIS_X, abs(dx) / move_time, settle=0.0)
    if move_y:
        set_velocity(AXIS_Y, abs(dy) / move_time, settle=0.0)

    started_at = time.monotonic()
    if move_x:
        move_abs(AXIS_X, target[0], settle=0.0)
    if move_y:
        move_abs(AXIS_Y, target[1], settle=0.0)

    if WAIT_MODE == "position":
        wait_xy_position(target[0], target[1], timeout_s=max(5.0, move_time + 2.0))
        if not DRY_RUN:
            time.sleep(max(0.0, move_time - (time.monotonic() - started_at)))
    elif not DRY_RUN:
        time.sleep(max(move_time - (time.monotonic() - started_at), 0.0) + SEND_SETTLE)
    if settle_after and not DRY_RUN:
        time.sleep(POST_MOVE_SETTLE + (Y_AXIS_EXTRA_SETTLE if move_y else 0.0))
    CURRENT_XY = target
    return target


def prepare_cut_start(x: float, y: float, move_speed: float):
    target = (float(x), float(y))
    needs_travel = point_distance(CURRENT_XY, target) > POSITION_TOLERANCE
    if needs_travel:
        laser_off()
        defocus_z()
        move_xy_abs(target[0], target[1], move_speed, settle_after=True)
        focus_z()
        laser_on()
        return

    if Z_IS_DEFOCUSED or CURRENT_Z is None or abs(float(CURRENT_Z) - focus_target_z()) > POSITION_TOLERANCE:
        focus_z()
    laser_on()


def run_paths(passes: int | None = None, move_speed: float = MOVE_SPEED, draw_speed: float = DRAW_SPEED):
    global CURRENT_XY, CURRENT_PASS_RETREAT
    for path_index, laser_path in enumerate(PATHS):
        points = [(float(x), float(y)) for x, y in laser_path["points"]]
        if len(points) < 2:
            continue

        pass_count = path_pass_count(laser_path, passes)
        closed = is_closed_path(laser_path, points)
        print(f"\n=== Path {path_index + 1}/{len(PATHS)} passes={pass_count} layer={laser_path['layer']} ===")

        for pass_index in range(pass_count):
            pass_points = points if pass_index % 2 == 0 or closed else list(reversed(points))
            CURRENT_PASS_RETREAT = (pass_index // Z_STEP_BACK_PASSES) * Z_STEP_BACK_MM
            print(f"  Pass {pass_index + 1}/{pass_count} (Z retreat {CURRENT_PASS_RETREAT:.6f} mm)")
            prepare_cut_start(pass_points[0][0], pass_points[0][1], move_speed)
            for x, y in pass_points[1:]:
                move_xy_abs(x, y, draw_speed)

    print("Final expected X/Y:", CURRENT_XY)


def run(
    port: str = DEFAULT_PORT,
    baudrate: int = DEFAULT_BAUDRATE,
    dry_run: bool = DRY_RUN_DEFAULT,
    passes: int | None = None,
    move_speed: float = MOVE_SPEED,
    draw_speed: float = DRAW_SPEED,
    wait_mode: str = WAIT_MODE,
    z_defocus_mm: float = Z_DEFOCUS_MM,
    z_speed: float = Z_SPEED,
):
    global WAIT_MODE, Z_DEFOCUS_MM, Z_SPEED, CURRENT_XY, CURRENT_Z, FOCUS_Z, Z_IS_DEFOCUSED, LASER_IS_ON
    WAIT_MODE = wait_mode
    Z_DEFOCUS_MM = max(0.0, float(z_defocus_mm))
    Z_SPEED = max(0.001, float(z_speed))
    CURRENT_XY = START_XY
    CURRENT_Z = None
    FOCUS_Z = None
    Z_IS_DEFOCUSED = False
    LASER_IS_ON = None
    connect(port=port, baudrate=baudrate, dry_run=dry_run)
    try:
        motor_on(AXIS_X)
        motor_on(AXIS_Y)
        motor_on(AXIS_Z)
        set_focus_reference()
        laser_off(force=True)
        run_paths(passes=passes, move_speed=move_speed, draw_speed=draw_speed)
    finally:
        laser_off(force=True)
        try:
            defocus_z()
        except Exception as exc:
            print(f"Could not move stage Z to defocus position during shutdown: {exc}")
        close()


def main():
    parser = argparse.ArgumentParser(description="Run generated ESP300 laser path.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--passes", type=int, default=None, help="Override every path's pass count.")
    parser.add_argument("--move-speed", type=float, default=MOVE_SPEED)
    parser.add_argument("--draw-speed", type=float, default=DRAW_SPEED)
    parser.add_argument("--wait-mode", choices=["position", "sleep"], default=WAIT_MODE)
    parser.add_argument("--z-defocus-mm", type=float, default=Z_DEFOCUS_MM)
    parser.add_argument("--z-speed", type=float, default=Z_SPEED)
    parser.add_argument("--no-z-defocus", action="store_true")
    parser.add_argument("--dry-run", action="store_true", default=DRY_RUN_DEFAULT)
    parser.add_argument("--live", action="store_true", help="Force serial execution even if dry-run is default.")
    args = parser.parse_args()

    dry_run = False if args.live else args.dry_run
    print("Metadata:", json.dumps(METADATA, indent=2))
    run(
        port=args.port,
        baudrate=args.baudrate,
        dry_run=dry_run,
        passes=args.passes,
        move_speed=args.move_speed,
        draw_speed=args.draw_speed,
        wait_mode=args.wait_mode,
        z_defocus_mm=0.0 if args.no_z_defocus else args.z_defocus_mm,
        z_speed=args.z_speed,
    )


if __name__ == "__main__":
    main()
