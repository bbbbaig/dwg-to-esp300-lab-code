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
PASSES = 2
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
POSITION_PATTERN = re.compile(r"[-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?")

# Fill these in only if the ESP300 or another serial device controls laser TTL.
LASER_ON_CMD = None
LASER_OFF_CMD = None

METADATA = json.loads('{"source":"C:\\\\Users\\\\QCQN\\\\Downloads\\\\laser_shield\\\\adjust25design.dwg","dxf":"C:\\\\Users\\\\QCQN\\\\Desktop\\\\dwg to code\\\\adjust25design_2passes_esp300.converted.dxf","unit_scale":23.10348189688244,"origin":[12.5,12.5],"anchor":"center","flip_x":false,"flip_y":false,"glass_size_mm":[25.0,25.0],"start_xy_mm":[12.5,12.5],"usable_area_shape":"circle","usable_diameter_mm":25.0,"usable_center_xy_mm":[12.5,12.5],"usable_margin_mm":0.05,"feed_rate":0.4,"travel_rate":1.0,"focus_z_mm":0.0,"z_defocus_mm":1.0,"z_speed_mm_s":0.3,"position_tolerance_mm":0.003,"passes":2,"trace_mode":"design","fit_width_mm":null,"fit_height_mm":null,"fit_longest_mm":25.0,"path_count":35,"point_count":510,"total_length_mm":146.406537}')
PATHS = json.loads('[{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[14.762141,11.821358],[14.762141,11.459415]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[14.762141,11.459415],[12.613107,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.613107,7.297075],[12.386893,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.386893,7.297075],[10.237859,11.459415]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[10.237859,11.459415],[10.237859,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[10.237859,11.821358],[10.192616,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[10.192616,11.821358],[10.192616,11.459415]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[10.192616,11.459415],[12.34165,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.34165,7.297075],[12.115436,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.115436,7.297075],[5.668333,10.509316]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[5.668333,10.509316],[5.668333,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[5.668333,11.821358],[5.623091,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[5.623091,11.821358],[5.623091,10.509316]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[5.623091,10.509316],[12.070193,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.070193,7.297075],[3.017104,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[3.017104,7.297075],[1.189294,9.452512]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[1.189294,9.452512],[1.189294,17.702925]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[1.189294,17.702925],[23.810706,17.702925]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[23.810706,17.702925],[23.810706,9.452512]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[23.810706,9.452512],[21.982896,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[21.982896,7.297075],[12.929807,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.929807,7.297075],[19.376909,10.509316]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[19.376909,10.509316],[19.376909,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[19.376909,11.821358],[19.331667,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[19.331667,11.821358],[19.331667,10.509316]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[19.331667,10.509316],[12.884564,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.884564,7.297075],[12.65835,7.297075]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[12.65835,7.297075],[14.807384,11.459415]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[14.807384,11.459415],[14.807384,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":false,"points":[[14.807384,11.821358],[14.762141,11.821358]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":true,"points":[[16.51244,13.958619],[16.563536,13.921496],[16.617097,13.888027],[16.672862,13.858376],[16.73056,13.832687],[16.789909,13.811086],[16.850621,13.793677],[16.912399,13.780546],[16.974942,13.771756],[17.037946,13.76735],[17.101104,13.76735],[17.164109,13.771756],[17.226652,13.780546],[17.28843,13.793677],[17.349141,13.811086],[17.40849,13.832687],[17.466188,13.858376],[17.521954,13.888027],[17.575515,13.921496],[17.626611,13.958619],[17.674992,13.999216],[17.720425,14.043089],[17.762686,14.090025],[17.80157,14.139794],[17.836887,14.192155],[17.868466,14.246851],[17.896153,14.303617],[17.919812,14.362176],[17.939329,14.422243],[17.954609,14.483525],[17.965576,14.545724],[17.974382,14.671656],[17.972178,14.734775],[17.965576,14.797587],[17.954609,14.859786],[17.939329,14.921068],[17.919812,14.981135],[17.896153,15.039694],[17.868466,15.09646],[17.836887,15.151156],[17.80157,15.203517],[17.762686,15.253286],[17.720425,15.300222],[17.674992,15.344095],[17.626611,15.384692],[17.575515,15.421816],[17.521954,15.455284],[17.466188,15.484935],[17.40849,15.510624],[17.349141,15.532225],[17.28843,15.549634],[17.226652,15.562765],[17.164109,15.571555],[17.101104,15.575961],[17.037946,15.575961],[16.974942,15.571555],[16.912399,15.562765],[16.850621,15.549634],[16.789909,15.532225],[16.73056,15.510624],[16.672862,15.484935],[16.617097,15.455284],[16.563536,15.421816],[16.51244,15.384692],[16.464058,15.344095],[16.418626,15.300222],[16.376365,15.253286],[16.337481,15.203517],[16.302163,15.151156],[16.270584,15.09646],[16.242898,15.039694],[16.219238,14.981135],[16.199721,14.921068],[16.184442,14.859786],[16.173475,14.797587],[16.166873,14.734775],[16.164669,14.671656],[16.166873,14.608536],[16.173475,14.545724],[16.184442,14.483525],[16.199721,14.422243],[16.219238,14.362176],[16.242898,14.303617],[16.270584,14.246851],[16.302163,14.192155],[16.337481,14.139794],[16.376365,14.090025],[16.418626,14.043089],[16.464058,13.999216],[16.51244,13.958619]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":true,"points":[[20.743,14.545724],[20.753967,14.483525],[20.769247,14.422243],[20.788764,14.362176],[20.812423,14.303617],[20.84011,14.246851],[20.871689,14.192155],[20.907006,14.139794],[20.94589,14.090025],[20.988151,14.043089],[21.033583,13.999216],[21.081965,13.958619],[21.133061,13.921496],[21.186622,13.888027],[21.242388,13.858376],[21.300085,13.832687],[21.359435,13.811086],[21.420146,13.793677],[21.481924,13.780546],[21.544467,13.771756],[21.607472,13.76735],[21.67063,13.76735],[21.733634,13.771756],[21.796177,13.780546],[21.857955,13.793677],[21.918667,13.811086],[21.978016,13.832687],[22.035714,13.858376],[22.091479,13.888027],[22.14504,13.921496],[22.196136,13.958619],[22.244518,13.999216],[22.28995,14.043089],[22.332211,14.090025],[22.371095,14.139794],[22.406412,14.192155],[22.437991,14.246851],[22.465678,14.303617],[22.489338,14.362176],[22.508854,14.422243],[22.524134,14.483525],[22.535101,14.545724],[22.543907,14.671656],[22.541703,14.734775],[22.535101,14.797587],[22.524134,14.859786],[22.508854,14.921068],[22.489338,14.981135],[22.465678,15.039694],[22.437991,15.09646],[22.406412,15.151156],[22.371095,15.203517],[22.332211,15.253286],[22.28995,15.300222],[22.244518,15.344095],[22.196136,15.384692],[22.14504,15.421816],[22.091479,15.455284],[22.035714,15.484935],[21.978016,15.510624],[21.918667,15.532225],[21.857955,15.549634],[21.796177,15.562765],[21.733634,15.571555],[21.67063,15.575961],[21.607472,15.575961],[21.544467,15.571555],[21.481924,15.562765],[21.420146,15.549634],[21.359435,15.532225],[21.300085,15.510624],[21.242388,15.484935],[21.186622,15.455284],[21.133061,15.421816],[21.081965,15.384692],[21.033583,15.344095],[20.988151,15.300222],[20.94589,15.253286],[20.907006,15.203517],[20.871689,15.151156],[20.84011,15.09646],[20.812423,15.039694],[20.788764,14.981135],[20.769247,14.921068],[20.753967,14.859786],[20.743,14.797587],[20.736398,14.734775],[20.734194,14.671656],[20.736398,14.608536],[20.743,14.545724]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":true,"points":[[13.404856,14.671656],[13.402652,14.734775],[13.39605,14.797587],[13.385083,14.859786],[13.369804,14.921068],[13.350287,14.981135],[13.326628,15.039694],[13.298941,15.09646],[13.267362,15.151156],[13.232044,15.203517],[13.19316,15.253286],[13.150899,15.300222],[13.105467,15.344095],[13.057085,15.384692],[13.005989,15.421816],[12.952428,15.455284],[12.896663,15.484935],[12.838965,15.510624],[12.779616,15.532225],[12.718905,15.549634],[12.657127,15.562765],[12.594583,15.571555],[12.531579,15.575961],[12.468421,15.575961],[12.405417,15.571555],[12.342873,15.562765],[12.281095,15.549634],[12.220384,15.532225],[12.161035,15.510624],[12.103337,15.484935],[12.047572,15.455284],[11.994011,15.421816],[11.942915,15.384692],[11.894533,15.344095],[11.849101,15.300222],[11.80684,15.253286],[11.767956,15.203517],[11.732638,15.151156],[11.701059,15.09646],[11.673372,15.039694],[11.649713,14.981135],[11.630196,14.921068],[11.614917,14.859786],[11.60395,14.797587],[11.597348,14.734775],[11.595144,14.671656],[11.597348,14.608536],[11.60395,14.545724],[11.614917,14.483525],[11.630196,14.422243],[11.649713,14.362176],[11.673372,14.303617],[11.701059,14.246851],[11.732638,14.192155],[11.767956,14.139794],[11.80684,14.090025],[11.849101,14.043089],[11.894533,13.999216],[11.942915,13.958619],[11.994011,13.921496],[12.047572,13.888027],[12.103337,13.858376],[12.161035,13.832687],[12.220384,13.811086],[12.281095,13.793677],[12.342873,13.780546],[12.405417,13.771756],[12.468421,13.76735],[12.531579,13.76735],[12.594583,13.771756],[12.657127,13.780546],[12.718905,13.793677],[12.779616,13.811086],[12.838965,13.832687],[12.896663,13.858376],[12.952428,13.888027],[13.005989,13.921496],[13.057085,13.958619],[13.105467,13.999216],[13.150899,14.043089],[13.19316,14.090025],[13.232044,14.139794],[13.267362,14.192155],[13.298941,14.246851],[13.326628,14.303617],[13.350287,14.362176],[13.369804,14.422243],[13.385083,14.483525],[13.39605,14.545724],[13.404856,14.671656]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":true,"points":[[8.835331,14.671656],[8.833127,14.734775],[8.826525,14.797587],[8.815558,14.859786],[8.800279,14.921068],[8.780762,14.981135],[8.757102,15.039694],[8.729416,15.09646],[8.697837,15.151156],[8.662519,15.203517],[8.623635,15.253286],[8.581374,15.300222],[8.535942,15.344095],[8.48756,15.384692],[8.436464,15.421816],[8.382903,15.455284],[8.327138,15.484935],[8.26944,15.510624],[8.210091,15.532225],[8.149379,15.549634],[8.087601,15.562765],[8.025058,15.571555],[7.962054,15.575961],[7.898896,15.575961],[7.835891,15.571555],[7.773348,15.562765],[7.71157,15.549634],[7.650859,15.532225],[7.59151,15.510624],[7.533812,15.484935],[7.478046,15.455284],[7.424485,15.421816],[7.373389,15.384692],[7.325008,15.344095],[7.279575,15.300222],[7.237314,15.253286],[7.19843,15.203517],[7.163113,15.151156],[7.131534,15.09646],[7.103847,15.039694],[7.080188,14.981135],[7.060671,14.921068],[7.045391,14.859786],[7.034424,14.797587],[7.027822,14.734775],[7.025618,14.671656],[7.027822,14.608536],[7.034424,14.545724],[7.045391,14.483525],[7.060671,14.422243],[7.080188,14.362176],[7.103847,14.303617],[7.131534,14.246851],[7.163113,14.192155],[7.19843,14.139794],[7.237314,14.090025],[7.279575,14.043089],[7.325008,13.999216],[7.373389,13.958619],[7.424485,13.921496],[7.478046,13.888027],[7.533812,13.858376],[7.59151,13.832687],[7.650859,13.811086],[7.71157,13.793677],[7.773348,13.780546],[7.835891,13.771756],[7.898896,13.76735],[7.962054,13.76735],[8.025058,13.771756],[8.087601,13.780546],[8.149379,13.793677],[8.210091,13.811086],[8.26944,13.832687],[8.327138,13.858376],[8.382903,13.888027],[8.436464,13.921496],[8.48756,13.958619],[8.535942,13.999216],[8.581374,14.043089],[8.623635,14.090025],[8.662519,14.139794],[8.697837,14.192155],[8.729416,14.246851],[8.757102,14.303617],[8.780762,14.362176],[8.800279,14.422243],[8.815558,14.483525],[8.826525,14.545724],[8.835331,14.671656]]},{"layer":"\\\\U+C678\\\\U+D615\\\\U+C120(ISO)","entity_type":"POLYLINE","closed":true,"points":[[4.265806,14.671656],[4.263602,14.734775],[4.257,14.797587],[4.246033,14.859786],[4.230753,14.921068],[4.211236,14.981135],[4.187577,15.039694],[4.15989,15.09646],[4.128311,15.151156],[4.092994,15.203517],[4.05411,15.253286],[4.011849,15.300222],[3.966417,15.344095],[3.918035,15.384692],[3.866939,15.421816],[3.813378,15.455284],[3.757612,15.484935],[3.699915,15.510624],[3.640565,15.532225],[3.579854,15.549634],[3.518076,15.562765],[3.455533,15.571555],[3.392528,15.575961],[3.32937,15.575961],[3.266366,15.571555],[3.203823,15.562765],[3.142045,15.549634],[3.081333,15.532225],[3.021984,15.510624],[2.964286,15.484935],[2.908521,15.455284],[2.85496,15.421816],[2.803864,15.384692],[2.755482,15.344095],[2.71005,15.300222],[2.667789,15.253286],[2.628905,15.203517],[2.593588,15.151156],[2.562009,15.09646],[2.534322,15.039694],[2.510662,14.981135],[2.491146,14.921068],[2.475866,14.859786],[2.464899,14.797587],[2.458297,14.734775],[2.456093,14.671656],[2.458297,14.608536],[2.464899,14.545724],[2.475866,14.483525],[2.491146,14.422243],[2.510662,14.362176],[2.534322,14.303617],[2.562009,14.246851],[2.593588,14.192155],[2.628905,14.139794],[2.667789,14.090025],[2.71005,14.043089],[2.755482,13.999216],[2.803864,13.958619],[2.85496,13.921496],[2.908521,13.888027],[2.964286,13.858376],[3.021984,13.832687],[3.081333,13.811086],[3.142045,13.793677],[3.203823,13.780546],[3.266366,13.771756],[3.32937,13.76735],[3.392528,13.76735],[3.455533,13.771756],[3.518076,13.780546],[3.579854,13.793677],[3.640565,13.811086],[3.699915,13.832687],[3.757612,13.858376],[3.813378,13.888027],[3.866939,13.921496],[3.918035,13.958619],[3.966417,13.999216],[4.011849,14.043089],[4.05411,14.090025],[4.092994,14.139794],[4.128311,14.192155],[4.15989,14.246851],[4.187577,14.303617],[4.211236,14.362176],[4.230753,14.422243],[4.246033,14.483525],[4.257,14.545724],[4.265806,14.671656]]}]')


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
    return float(FOCUS_Z)


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
    global CURRENT_XY
    for path_index, laser_path in enumerate(PATHS):
        points = [(float(x), float(y)) for x, y in laser_path["points"]]
        if len(points) < 2:
            continue

        pass_count = path_pass_count(laser_path, passes)
        closed = is_closed_path(laser_path, points)
        print(f"\n=== Path {path_index + 1}/{len(PATHS)} passes={pass_count} layer={laser_path['layer']} ===")

        for pass_index in range(pass_count):
            pass_points = points if pass_index % 2 == 0 or closed else list(reversed(points))
            print(f"  Pass {pass_index + 1}/{pass_count}")
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
