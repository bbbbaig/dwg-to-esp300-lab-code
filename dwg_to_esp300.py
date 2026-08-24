from __future__ import annotations

import argparse
import fnmatch
import json
import math
import re
import shutil
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import ezdxf
from ezdxf import bbox, disassemble, path
from ezdxf.addons import odafc


Point = tuple[float, float]


DEFAULT_FEED_RATE_MM_S = 0.4
DEFAULT_TRAVEL_RATE_MM_S = 1.0
DEFAULT_Z_DEFOCUS_MM = 1.0
DEFAULT_Z_SPEED_MM_S = 0.3
DEFAULT_COMMAND_SETTLE_S = 0.05
DEFAULT_POSITION_TOLERANCE_MM = 0.003


INSUNITS_TO_MM = {
    1: 25.4,  # inches
    2: 304.8,  # feet
    4: 1.0,  # millimeters
    5: 10.0,  # centimeters
    6: 1000.0,  # meters
}


@dataclass
class LaserPath:
    points: list[Point]
    layer: str
    entity_type: str
    closed: bool = False

    @property
    def start(self) -> Point:
        return self.points[0]

    @property
    def end(self) -> Point:
        return self.points[-1]

    @property
    def length(self) -> float:
        return polyline_length(self.points)


@dataclass
class DrawingData:
    paths: list[LaserPath]
    source_path: Path
    dxf_path: Path
    unit_scale: float
    raw_bbox: tuple[Point, Point]
    machine_bbox: tuple[Point, Point]
    skipped: dict[str, int]


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points: Sequence[Point]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def dedupe_points(points: Iterable[Point], tolerance: float = 1e-9) -> list[Point]:
    clean: list[Point] = []
    for point in points:
        if not clean or distance(clean[-1], point) > tolerance:
            clean.append(point)
    return clean


def split_long_segments(points: Sequence[Point], max_segment: float) -> list[Point]:
    if max_segment <= 0 or len(points) < 2:
        return list(points)

    result = [points[0]]
    for start, end in zip(points, points[1:]):
        segment_length = distance(start, end)
        if segment_length <= max_segment:
            result.append(end)
            continue

        steps = max(1, math.ceil(segment_length / max_segment))
        for step in range(1, steps + 1):
            t = step / steps
            result.append(
                (
                    start[0] + (end[0] - start[0]) * t,
                    start[1] + (end[1] - start[1]) * t,
                )
            )
    return result


def close_if_needed(points: list[Point], closed: bool, tolerance: float = 1e-7) -> list[Point]:
    if closed and len(points) > 2 and distance(points[0], points[-1]) > tolerance:
        points = [*points, points[0]]
    return points


def decode_dxf_unicode_escapes(text: str) -> str:
    return re.sub(r"\\U\+([0-9A-Fa-f]{4})", lambda match: chr(int(match.group(1), 16)), text)


def matches_layer(layer: str, include: Sequence[str], exclude: Sequence[str]) -> bool:
    candidates = {layer, decode_dxf_unicode_escapes(layer)}
    if include and not any(fnmatch.fnmatchcase(candidate, pattern) for candidate in candidates for pattern in include):
        return False
    if exclude and any(fnmatch.fnmatchcase(candidate, pattern) for candidate in candidates for pattern in exclude):
        return False
    return True


def layer_name(laser_path: LaserPath) -> str:
    return decode_dxf_unicode_escapes(laser_path.layer)


ANNOTATION_LAYER_KEYWORDS = (
    "annotation",
    "anno",
    "dim",
    "dimension",
    "text",
    "eval",
    "evaluation",
    "watermark",
    "title",
    "note",
    "center",
    "hidden",
    "construction",
    "치수",
    "문자",
    "주석",
    "평가",
)


def looks_like_annotation_layer(name: str) -> bool:
    lowered = name.lower()
    return any(keyword in lowered for keyword in ANNOTATION_LAYER_KEYWORDS)


def looks_like_global_diagonal(laser_path: LaserPath, drawing_bbox: tuple[Point, Point]) -> bool:
    (min_x, min_y), (max_x, max_y) = drawing_bbox
    width = max(max_x - min_x, 1e-9)
    height = max(max_y - min_y, 1e-9)
    xs = [point[0] for point in laser_path.points]
    ys = [point[1] for point in laser_path.points]
    path_width = max(xs) - min(xs)
    path_height = max(ys) - min(ys)
    return len(laser_path.points) == 2 and path_width >= width * 0.9 and path_height >= height * 0.9


def select_design_paths(paths: Sequence[LaserPath], skipped: dict[str, int]) -> list[LaserPath]:
    layers = {layer_name(laser_path) for laser_path in paths}
    if not paths:
        return []

    drawing_bbox = raw_bbox(paths)

    def without_global_diagonals(candidates: Sequence[LaserPath]) -> list[LaserPath]:
        selected: list[LaserPath] = []
        for laser_path in candidates:
            if looks_like_global_diagonal(laser_path, drawing_bbox):
                skipped["auto_trace_diagonal"] = skipped.get("auto_trace_diagonal", 0) + 1
                continue
            selected.append(laser_path)
        return selected

    nonzero_paths = [laser_path for laser_path in paths if layer_name(laser_path) not in {"0", "Defpoints"}]
    design_named_paths = [
        laser_path
        for laser_path in nonzero_paths
        if not looks_like_annotation_layer(layer_name(laser_path))
    ]

    if design_named_paths:
        skipped["auto_trace_layer_0"] = len(paths) - len(nonzero_paths)
        skipped["auto_trace_annotation_layer"] = len(nonzero_paths) - len(design_named_paths)
        return without_global_diagonals(design_named_paths)

    if nonzero_paths:
        skipped["auto_trace_layer_0"] = len(paths) - len(nonzero_paths)
        return without_global_diagonals(nonzero_paths)

    selected = []
    for laser_path in paths:
        if looks_like_global_diagonal(laser_path, drawing_bbox):
            skipped["auto_trace_diagonal"] = skipped.get("auto_trace_diagonal", 0) + 1
            continue
        selected.append(laser_path)

    skipped["auto_trace_layers"] = len(layers)
    return selected


def convert_dwg_with_aspose(source: Path, dxf_path: Path) -> None:
    try:
        import aspose.cad as cad
    except Exception as exc:  # pragma: no cover - depends on optional package
        raise RuntimeError("aspose-cad is not installed") from exc

    with tempfile.TemporaryDirectory(prefix="dwg_to_esp300_") as temp_dir:
        temp_source = Path(temp_dir) / source.name
        shutil.copy2(source, temp_source)
        with cad.Image.load(str(temp_source)) as image:
            options = cad.imageoptions.DxfOptions()
            image.save(str(dxf_path), options)


def convert_dwg_to_dxf(source: Path, dxf_path: Path) -> Path:
    dxf_path.parent.mkdir(parents=True, exist_ok=True)
    if dxf_path.exists():
        dxf_path.unlink()

    if odafc.is_installed():
        odafc.convert(source, dxf_path, replace=True)
        if dxf_path.exists():
            return dxf_path

    try:
        convert_dwg_with_aspose(source, dxf_path)
    except Exception as exc:
        raise RuntimeError(
            "DWG input needs a converter. Install ODA File Converter or "
            "install the optional Python package with `py -m pip install aspose-cad`."
        ) from exc

    if not dxf_path.exists():
        raise RuntimeError(f"DWG conversion did not create {dxf_path}")
    return dxf_path


def ensure_dxf(source: Path, output: Path | None) -> Path:
    suffix = source.suffix.lower()
    if suffix == ".dxf":
        return source
    if suffix != ".dwg":
        raise ValueError(f"Unsupported drawing type: {source.suffix}. Use .dwg or .dxf.")

    dxf_path = output.with_suffix(".converted.dxf") if output else source.with_suffix(".converted.dxf")
    return convert_dwg_to_dxf(source, dxf_path)


def infer_unit_scale(doc: ezdxf.document.Drawing, explicit_scale: float | None) -> float:
    if explicit_scale is not None:
        return explicit_scale
    insunits = doc.header.get("$INSUNITS", 0)
    return INSUNITS_TO_MM.get(insunits, 1.0)


def entity_closed(entity) -> bool:
    value = getattr(entity, "is_closed", False)
    if callable(value):
        try:
            return bool(value())
        except TypeError:
            return False
    return bool(value)


def entity_to_points(entity, flatness_drawing_units: float) -> list[Point]:
    entity_path = path.make_path(entity)
    vertices = entity_path.flattening(flatness_drawing_units)
    return dedupe_points((float(v.x), float(v.y)) for v in vertices)


def extract_paths(
    dxf_path: Path,
    *,
    include_layers: Sequence[str],
    exclude_layers: Sequence[str],
    flatness_mm: float,
    unit_scale: float | None,
    min_length_mm: float,
) -> tuple[list[LaserPath], ezdxf.document.Drawing, dict[str, int]]:
    doc = ezdxf.readfile(dxf_path)
    scale = infer_unit_scale(doc, unit_scale)
    flatness_drawing_units = flatness_mm / scale if scale else flatness_mm

    paths: list[LaserPath] = []
    skipped: dict[str, int] = {}

    entities = disassemble.recursive_decompose(doc.modelspace())
    for entity in entities:
        entity_type = entity.dxftype()
        layer = getattr(entity.dxf, "layer", "0")
        if not matches_layer(layer, include_layers, exclude_layers):
            skipped["layer_filter"] = skipped.get("layer_filter", 0) + 1
            continue

        try:
            points = entity_to_points(entity, flatness_drawing_units)
        except Exception:
            skipped[entity_type] = skipped.get(entity_type, 0) + 1
            continue

        closed = entity_closed(entity)
        points = close_if_needed(points, closed)
        length_drawing_units = polyline_length(points)
        if len(points) < 2 or length_drawing_units * scale < min_length_mm:
            skipped["too_short"] = skipped.get("too_short", 0) + 1
            continue

        paths.append(LaserPath(points=points, layer=layer, entity_type=entity_type, closed=closed))

    return paths, doc, skipped


def raw_bbox(paths: Sequence[LaserPath]) -> tuple[Point, Point]:
    xs = [point[0] for laser_path in paths for point in laser_path.points]
    ys = [point[1] for laser_path in paths for point in laser_path.points]
    return (min(xs), min(ys)), (max(xs), max(ys))


def rotate_points(points: Sequence[Point], angle_deg: float, center: Point) -> list[Point]:
    if abs(angle_deg) < 1e-9:
        return list(points)
    theta = math.radians(angle_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    cx, cy = center
    rotated: list[Point] = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        rotated.append((cx + dx * cos_t - dy * sin_t, cy + dx * sin_t + dy * cos_t))
    return rotated


def rotate_paths(paths: Sequence[LaserPath], angle_deg: float) -> list[LaserPath]:
    """Rotate the drawing about its own bounding-box center prior to fit
    scaling. For a non-square layout, this typically increases the scale
    factor produced by apply_fit_scale before cap_scale_to_circle applies
    the circular working-area limit, yielding additional usable margin."""
    if abs(angle_deg) < 1e-9:
        return list(paths)
    (min_x, min_y), (max_x, max_y) = raw_bbox(paths)
    center = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
    return [
        LaserPath(
            points=rotate_points(laser_path.points, angle_deg, center),
            layer=laser_path.layer,
            entity_type=laser_path.entity_type,
            closed=laser_path.closed,
        )
        for laser_path in paths
    ]


def apply_fit_scale(
    paths: Sequence[LaserPath],
    unit_scale: float,
    fit_width_mm: float | None,
    fit_height_mm: float | None,
    fit_longest_mm: float | None = None,
) -> float:
    if fit_width_mm is None and fit_height_mm is None and fit_longest_mm is None:
        return unit_scale

    (min_x, min_y), (max_x, max_y) = raw_bbox(paths)
    width = max(max_x - min_x, 1e-12)
    height = max(max_y - min_y, 1e-12)
    scales: list[float] = []
    if fit_longest_mm is not None:
        scales.append(fit_longest_mm / max(width, height))
    if fit_width_mm is not None:
        scales.append(fit_width_mm / width)
    if fit_height_mm is not None:
        scales.append(fit_height_mm / height)
    return min(scales)


def radial_extent_from_center(paths: Sequence[LaserPath]) -> float:
    (min_x, min_y), (max_x, max_y) = raw_bbox(paths)
    center = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
    return max(
        distance(point, center)
        for laser_path in paths
        for point in laser_path.points
    )


def cap_scale_to_circle(
    paths: Sequence[LaserPath],
    scale: float,
    *,
    diameter_mm: float,
    margin_mm: float = 0.0,
) -> float:
    radius = max((float(diameter_mm) / 2.0) - float(margin_mm), 0.001)
    extent = max(radial_extent_from_center(paths), 1e-12)
    return min(float(scale), radius / extent)


def transform_paths(
    paths: Sequence[LaserPath],
    *,
    unit_scale: float,
    origin: Point,
    anchor: str,
    flip_x: bool,
    flip_y: bool,
    max_segment_mm: float,
) -> tuple[list[LaserPath], tuple[Point, Point], tuple[Point, Point]]:
    (min_x, min_y), (max_x, max_y) = raw_bbox(paths)
    if anchor == "lower-left":
        anchor_point = (min_x, min_y)
    elif anchor == "center":
        anchor_point = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
    elif anchor == "drawing-origin":
        anchor_point = (0.0, 0.0)
    else:
        raise ValueError(f"Unsupported anchor: {anchor}")

    transformed: list[LaserPath] = []
    for laser_path in paths:
        machine_points: list[Point] = []
        for x, y in laser_path.points:
            local_x = x - anchor_point[0]
            local_y = y - anchor_point[1]
            if flip_x:
                local_x = -local_x
            if flip_y:
                local_y = -local_y
            machine_points.append(
                (
                    origin[0] + local_x * unit_scale,
                    origin[1] + local_y * unit_scale,
                )
            )

        machine_points = split_long_segments(machine_points, max_segment_mm)
        machine_points = dedupe_points(machine_points, tolerance=1e-6)
        transformed.append(
            LaserPath(
                points=machine_points,
                layer=laser_path.layer,
                entity_type=laser_path.entity_type,
                closed=laser_path.closed,
            )
        )

    machine_min, machine_max = raw_bbox(transformed)
    return transformed, ((min_x, min_y), (max_x, max_y)), (machine_min, machine_max)


def rotate_closed_path_to_nearest(points: Sequence[Point], current: Point) -> list[Point]:
    if len(points) <= 3:
        return list(points)
    core = list(points[:-1]) if distance(points[0], points[-1]) < 1e-6 else list(points)
    nearest_index = min(range(len(core)), key=lambda index: distance(current, core[index]))
    rotated = core[nearest_index:] + core[:nearest_index]
    return [*rotated, rotated[0]]


def is_effectively_closed(laser_path: LaserPath) -> bool:
    return laser_path.closed or distance(laser_path.points[0], laser_path.points[-1]) < 1e-6


def orient_path_from_current(laser_path: LaserPath, current: Point) -> LaserPath:
    if is_effectively_closed(laser_path):
        points = rotate_closed_path_to_nearest(laser_path.points, current)
        return LaserPath(points=points, layer=laser_path.layer, entity_type=laser_path.entity_type, closed=True)

    forward_distance = distance(current, laser_path.points[0])
    backward_distance = distance(current, laser_path.points[-1])
    points = laser_path.points if forward_distance <= backward_distance else list(reversed(laser_path.points))
    return LaserPath(points=list(points), layer=laser_path.layer, entity_type=laser_path.entity_type, closed=False)


def travel_cost_for_order(paths: Sequence[LaserPath], start: Point) -> float:
    current = start
    cost = 0.0
    for laser_path in paths:
        if is_effectively_closed(laser_path):
            core = laser_path.points[:-1] if distance(laser_path.points[0], laser_path.points[-1]) < 1e-6 else laser_path.points
            nearest = min(core, key=lambda point: distance(current, point))
            cost += distance(current, nearest)
            current = nearest
            continue

        start_distance = distance(current, laser_path.points[0])
        end_distance = distance(current, laser_path.points[-1])
        if start_distance <= end_distance:
            cost += start_distance
            current = laser_path.points[-1]
        else:
            cost += end_distance
            current = laser_path.points[0]
    return cost


def orient_order(paths: Sequence[LaserPath], start: Point) -> list[LaserPath]:
    oriented: list[LaserPath] = []
    current = start
    for laser_path in paths:
        oriented_path = orient_path_from_current(laser_path, current)
        oriented.append(oriented_path)
        current = oriented_path.points[-1]
    return oriented


def improve_order_by_local_search(paths: Sequence[LaserPath], start: Point, passes: int = 2) -> list[LaserPath]:
    order = list(paths)
    best_cost = travel_cost_for_order(order, start)
    if len(order) < 4:
        return order

    for _ in range(max(0, passes)):
        improved = False

        for i in range(0, len(order) - 2):
            for j in range(i + 2, len(order)):
                candidate = order[:i] + list(reversed(order[i : j + 1])) + order[j + 1 :]
                candidate_cost = travel_cost_for_order(candidate, start)
                if candidate_cost + 1e-6 < best_cost:
                    order = candidate
                    best_cost = candidate_cost
                    improved = True
                    break
            if improved:
                break
        if improved:
            continue

        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                candidate = list(order)
                candidate[i], candidate[j] = candidate[j], candidate[i]
                candidate_cost = travel_cost_for_order(candidate, start)
                if candidate_cost + 1e-6 < best_cost:
                    order = candidate
                    best_cost = candidate_cost
                    improved = True
                    break
            if improved:
                break

        if not improved:
            break

    return order


def optimize_path_order(paths: Sequence[LaserPath], start: Point) -> list[LaserPath]:
    remaining = list(paths)
    ordered: list[LaserPath] = []
    current = start

    while remaining:
        best_index = 0
        best_points = remaining[0].points
        best_distance = float("inf")

        for index, laser_path in enumerate(remaining):
            candidates: list[list[Point]]
            if laser_path.closed:
                candidates = [rotate_closed_path_to_nearest(laser_path.points, current)]
            else:
                forward = laser_path.points
                backward = list(reversed(laser_path.points))
                candidates = [forward, backward]

            for candidate in candidates:
                candidate_distance = distance(current, candidate[0])
                if candidate_distance < best_distance:
                    best_index = index
                    best_points = list(candidate)
                    best_distance = candidate_distance

        chosen = remaining.pop(best_index)
        ordered.append(LaserPath(points=best_points, layer=chosen.layer, entity_type=chosen.entity_type, closed=chosen.closed))
        current = best_points[-1]

    improved_order = improve_order_by_local_search(ordered, start=start, passes=2)
    return orient_order(improved_order, start=start)


def travel_length_for_paths(paths: Sequence[LaserPath], start: Point) -> float:
    current = start
    travel_length = 0.0
    for laser_path in paths:
        travel_length += distance(current, laser_path.points[0])
        current = laser_path.points[-1]
    return travel_length


def format_paths_for_python(paths: Sequence[LaserPath], path_passes: Sequence[int] | None = None) -> str:
    payload = []
    for index, laser_path in enumerate(paths):
        item = {
            "layer": laser_path.layer,
            "entity_type": laser_path.entity_type,
            "closed": laser_path.closed,
            "points": [[round(x, 6), round(y, 6)] for x, y in laser_path.points],
        }
        if path_passes is not None:
            item["passes"] = max(0, int(path_passes[index]))
        payload.append(item)
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))


def generate_esp300_script(
    paths: Sequence[LaserPath],
    *,
    output: Path,
    metadata: dict[str, object],
    port: str,
    baudrate: int,
    feed_rate: float,
    travel_rate: float,
    passes: int | None,
    position_tolerance: float,
    command_settle: float,
    dry_run_default: bool,
    path_passes: Sequence[int] | None = None,
    z_defocus_mm: float = DEFAULT_Z_DEFOCUS_MM,
    z_defocus_direction: float = -1.0,
    z_speed: float = DEFAULT_Z_SPEED_MM_S,
    wait_mode: str = "position",
) -> str:
    path_payload = format_paths_for_python(paths, path_passes=path_passes)
    metadata_payload = json.dumps(metadata, ensure_ascii=True, separators=(",", ":"))
    passes_payload = "None" if passes is None else str(max(0, int(passes)))
    script = f'''from __future__ import annotations

import argparse
import json
import math
import re
import time

import serial


AXIS_X = 1
AXIS_Y = 2
AXIS_Z = 3

DEFAULT_PORT = {port!r}
DEFAULT_BAUDRATE = {baudrate}
DRAW_SPEED = {feed_rate}
MOVE_SPEED = {travel_rate}
Z_SPEED = {z_speed}
PASSES = {passes_payload}
POSITION_TOLERANCE = {position_tolerance}
SEND_SETTLE = {command_settle}
READ_POLLS = 20
READ_DELAY = 0.03
TP_READ_RETRIES = 5
TP_RETRY_DELAY = 0.08
POST_MOVE_SETTLE = 0.05
Y_AXIS_EXTRA_SETTLE = 0.05
DRY_RUN_DEFAULT = {dry_run_default!r}
WAIT_MODE = {wait_mode!r}
Z_DEFOCUS_MM = {z_defocus_mm}
Z_DEFOCUS_DIRECTION = {z_defocus_direction}
Z_STEP_BACK_PASSES = 10
Z_STEP_BACK_MM = 0.001
POSITION_PATTERN = re.compile(r"[-+]?\\d*\\.?\\d+(?:[Ee][-+]?\\d+)?")

# Fill these in only if the ESP300 or another serial device controls laser TTL.
LASER_ON_CMD = None
LASER_OFF_CMD = None

METADATA = json.loads({metadata_payload!r})
PATHS = json.loads({path_payload!r})


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
        print(f">>> Sending: {{cmd}}")
    if DRY_RUN:
        return
    if ser is None or not ser.is_open:
        raise RuntimeError("Serial port is not open. Call connect(..., dry_run=False) first.")

    ser.write((cmd + "\\r\\n").encode("ascii"))
    if settle > 0:
        time.sleep(settle)


def query(cmd: str):
    global ser
    if VERBOSE:
        print(f">>> Sending: {{cmd}}")
    if DRY_RUN:
        return "0"
    if ser is None or not ser.is_open:
        raise RuntimeError("Serial port is not open. Call connect(..., dry_run=False) first.")

    ser.reset_input_buffer()
    ser.write((cmd + "\\r\\n").encode("ascii"))
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
            if b"\\r" in response or b"\\n" in response:
                time.sleep(0.005)
                if ser.in_waiting > 0:
                    response += ser.read(ser.in_waiting)
                break
        time.sleep(READ_DELAY)

    reply = response.decode("ascii", errors="ignore").strip()
    if VERBOSE:
        print(f"<<< Received: '{{reply}}'")
    return reply


def send(cmd: str):
    return query(cmd)


def parse_position_reply(reply: str, axis: int) -> float | None:
    text = (reply or "").strip()
    if not text:
        return None

    text = re.sub(rf"^{{axis}}\\s*TP", "", text, flags=re.IGNORECASE).strip()
    tokens = [token.strip() for token in re.split(r"[\\s,;]+", text) if token.strip()]
    numeric_tokens = [token for token in tokens if POSITION_PATTERN.fullmatch(token)]
    if numeric_tokens:
        return float(numeric_tokens[-1])

    if "TP" not in text.upper():
        matches = POSITION_PATTERN.findall(text)
        if matches:
            return float(matches[-1])
    return None


def motor_on(axis: int):
    write_command(f"{{axis}}MO")


def motor_off(axis: int):
    write_command(f"{{axis}}MF")


def get_position(axis: int, retries: int = TP_READ_RETRIES, retry_delay: float = TP_RETRY_DELAY) -> float:
    last_reply = ""
    for attempt in range(1, max(1, int(retries)) + 1):
        last_reply = query(f"{{axis}}TP")
        parsed = parse_position_reply(last_reply, axis)
        # Discard out-of-range values (parsing/framing artifacts) rather
        # than accepting them; the configured travel range is well under
        # 200 mm on every axis.
        if parsed is not None and abs(parsed) <= 250.0:
            return parsed
        if VERBOSE:
            print(f"TP read retry {{attempt}}/{{retries}} for axis {{axis}}; reply={{last_reply!r}}")
        if attempt < retries:
            time.sleep(max(0.0, retry_delay))
    raise RuntimeError(f"Could not read axis {{axis}} position after {{retries}} TP attempts; last reply={{last_reply!r}}")


def move_abs(axis: int, position: float, settle: float = SEND_SETTLE):
    write_command(f"{{axis}}PA{{position:.6f}}", settle=settle)


def move_rel(axis: int, distance: float, settle: float = SEND_SETTLE):
    write_command(f"{{axis}}PR{{distance:.6f}}", settle=settle)


def set_velocity(axis: int, velocity: float, settle: float = SEND_SETTLE):
    write_command(f"{{axis}}VA{{max(float(velocity), 0.001):.6f}}", settle=settle)


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
    detail = f"; last TP error: {{last_error}}" if last_error else ""
    raise TimeoutError(f"Timed out waiting for axis {{axis}} at {{target:.4f}}{{detail}}")


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
    detail = f"; last TP error: {{last_error}}" if last_error else ""
    raise TimeoutError(f"Timed out waiting for X={{x:.4f}}, Y={{y:.4f}}{{detail}}")


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
    print(f"Stage focus Z reference: {{FOCUS_Z:.6f}} mm")


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
    print(f"Stage Z defocus: {{target:.6f}} mm")
    move_z_abs(target, Z_SPEED)
    Z_IS_DEFOCUSED = True
    return target


def focus_z():
    global Z_IS_DEFOCUSED
    target = focus_target_z()
    print(f"Stage Z focus: {{target:.6f}} mm")
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
        print(f"\\n=== Path {{path_index + 1}}/{{len(PATHS)}} passes={{pass_count}} layer={{laser_path['layer']}} ===")

        for pass_index in range(pass_count):
            pass_points = points if pass_index % 2 == 0 or closed else list(reversed(points))
            CURRENT_PASS_RETREAT = (pass_index // Z_STEP_BACK_PASSES) * Z_STEP_BACK_MM
            print(f"  Pass {{pass_index + 1}}/{{pass_count}} (Z retreat {{CURRENT_PASS_RETREAT:.6f}} mm)")
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
            print(f"Could not move stage Z to defocus position during shutdown: {{exc}}")
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
'''
    output.write_text(script, encoding="utf-8")
    return script


def _notebook_source(text: str) -> list[str]:
    return text.splitlines(keepends=True) or [""]


def _notebook_cell(cell_type: str, source: str) -> dict[str, object]:
    cell: dict[str, object] = {
        "cell_type": cell_type,
        "metadata": {},
        "source": _notebook_source(source),
    }
    if cell_type == "code":
        cell["execution_count"] = None
        cell["outputs"] = []
    return cell


def _disable_script_main_call(script: str) -> str:
    patterns = (
        '\nif __name__ == "__main__":\n    main()\n',
        '\nif __name__ == "__main__":\n    raise SystemExit(main())\n',
    )
    for pattern in patterns:
        if pattern in script:
            return script.replace(
                pattern,
                "\n# CLI entry point disabled for notebook use.\n# Call run(...) from a later cell instead.\n",
            )
    return script


def build_script_notebook(script: str, *, title: str, source_script: Path | None = None) -> dict[str, object]:
    safe_script = _disable_script_main_call(script)
    source_line = f"\nSource script: `{source_script}`\n" if source_script else ""
    return {
        "cells": [
            _notebook_cell(
                "markdown",
                f"# {title}\n"
                f"{source_line}\n"
                "This notebook contains the generated ESP300 motion code in a Jupyter-friendly form. "
                "The command-line `main()` call is disabled so Run All will not accidentally start a laser job.\n",
            ),
            _notebook_cell(
                "markdown",
                "## Setup\n"
                "Run the next cell to load the generated paths, ESP300 helpers, and `run(...)` function. "
                "Serial hardware is only opened when you explicitly call `run(..., dry_run=False)`.\n",
            ),
            _notebook_cell("code", safe_script),
            _notebook_cell(
                "markdown",
                "## Metadata\n"
                "Use this cell to inspect the generated path count, scale, source file, and motion settings.\n",
            ),
            _notebook_cell(
                "code",
                "import json\n"
                "print(json.dumps(METADATA, indent=2))\n"
                "print(f\"paths={len(PATHS)}\")\n",
            ),
            _notebook_cell(
                "markdown",
                "## Dry Run\n"
                "Set `RUN_DRY_PREVIEW = True` to print the ESP300 commands without opening the serial port.\n",
            ),
            _notebook_cell(
                "code",
                "RUN_DRY_PREVIEW = False\n"
                "if RUN_DRY_PREVIEW:\n"
                "    try:\n"
                "        run(dry_run=True, passes=PASSES)\n"
                "    except TypeError:\n"
                "        run(DEFAULT_PORT, DEFAULT_BAUDRATE, True, \"sleep\", PASSES)\n",
            ),
            _notebook_cell(
                "markdown",
                "## Live Run\n"
                "Set `LIVE_RUN = True` only after checking the path preview, port, speed, pass count, and laser trigger commands.\n",
            ),
            _notebook_cell(
                "code",
                "LIVE_RUN = False\n"
                "if LIVE_RUN:\n"
                "    try:\n"
                "        run(port=DEFAULT_PORT, dry_run=False, passes=PASSES)\n"
                "    except TypeError:\n"
                "        run(DEFAULT_PORT, DEFAULT_BAUDRATE, False, \"sleep\", PASSES)\n",
            ),
        ],
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3",
            },
            "language_info": {
                "name": "python",
                "pygments_lexer": "ipython3",
            },
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def write_script_notebook(
    script: str,
    output: Path,
    *,
    title: str = "ESP300 Laser Path",
    source_script: Path | None = None,
) -> None:
    notebook = build_script_notebook(script, title=title, source_script=source_script)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(notebook, ensure_ascii=False, indent=2), encoding="utf-8")


def write_preview_svg(paths: Sequence[LaserPath], output: Path, margin: float = 2.0) -> None:
    (min_x, min_y), (max_x, max_y) = raw_bbox(paths)
    width = max(max_x - min_x, 1e-6)
    height = max(max_y - min_y, 1e-6)
    view_box = f"{min_x - margin:.6f} {-max_y - margin:.6f} {width + margin * 2:.6f} {height + margin * 2:.6f}"

    colors = ["#005f73", "#9b2226", "#0a9396", "#bb3e03", "#3a0ca3", "#2b9348"]
    body: list[str] = []
    for index, laser_path in enumerate(paths):
        color = colors[index % len(colors)]
        commands = []
        for point_index, (x, y) in enumerate(laser_path.points):
            prefix = "M" if point_index == 0 else "L"
            commands.append(f"{prefix} {x:.6f} {-y:.6f}")
        body.append(
            f'<path d="{" ".join(commands)}" fill="none" stroke="{color}" '
            f'stroke-width="0.03" vector-effect="non-scaling-stroke" />'
        )

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="{view_box}">
  <rect x="{min_x - margin:.6f}" y="{-max_y - margin:.6f}" width="{width + margin * 2:.6f}" height="{height + margin * 2:.6f}" fill="white" />
  {"".join(body)}
</svg>
'''
    output.write_text(svg, encoding="utf-8")


def summarize(paths: Sequence[LaserPath], drawing_data: DrawingData) -> str:
    total_length = sum(laser_path.length for laser_path in paths)
    total_points = sum(len(laser_path.points) for laser_path in paths)
    (raw_min, raw_max) = drawing_data.raw_bbox
    (machine_min, machine_max) = drawing_data.machine_bbox
    layer_stats: dict[str, tuple[int, float]] = {}
    for laser_path in paths:
        layer = decode_dxf_unicode_escapes(laser_path.layer)
        count, length = layer_stats.get(layer, (0, 0.0))
        layer_stats[layer] = (count + 1, length + laser_path.length)
    layer_summary = "; ".join(
        f"{layer}: {count} paths, {length:.3f} mm"
        for layer, (count, length) in sorted(layer_stats.items())
    )
    return "\n".join(
        [
            f"Source: {drawing_data.source_path}",
            f"DXF: {drawing_data.dxf_path}",
            f"Paths: {len(paths)}",
            f"Points: {total_points}",
            f"Total cut length: {total_length:.3f} mm",
            f"Layers: {layer_summary}",
            f"Raw bbox: ({raw_min[0]:.6f}, {raw_min[1]:.6f}) to ({raw_max[0]:.6f}, {raw_max[1]:.6f})",
            f"Machine bbox: ({machine_min[0]:.3f}, {machine_min[1]:.3f}) to ({machine_max[0]:.3f}, {machine_max[1]:.3f}) mm",
            f"Unit scale: {drawing_data.unit_scale:g} mm per drawing unit",
            f"Skipped: {drawing_data.skipped}",
        ]
    )


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert DWG/DXF geometry to ESP300 laser path code.")
    parser.add_argument("input", type=Path, help="Input .dwg or .dxf file.")
    parser.add_argument("--output", "-o", type=Path, help="Generated ESP300 Python script.")
    parser.add_argument("--ipynb", action="store_true", help="Also write a Jupyter notebook next to the generated script.")
    parser.add_argument("--notebook-output", type=Path, help="Generated Jupyter notebook path. Implies --ipynb.")
    parser.add_argument("--preview", type=Path, help="SVG preview path.")
    parser.add_argument("--include-layer", action="append", default=[], help="Only include matching layer. Wildcards are allowed.")
    parser.add_argument("--exclude-layer", action="append", default=[], help="Exclude matching layer. Wildcards are allowed.")
    parser.add_argument("--trace-mode", choices=["design", "all"], default="design", help="`design` excludes obvious watermark/support layers.")
    parser.add_argument("--unit-scale", type=float, help="Millimeters per drawing unit. Example: 25.4 for inch-based drawings.")
    parser.add_argument("--fit-width-mm", type=float, help="Uniformly scale selected geometry to this machine width.")
    parser.add_argument("--fit-height-mm", type=float, help="Uniformly scale selected geometry to this machine height.")
    parser.add_argument("--fit-longest-mm", type=float, help="Uniformly scale selected geometry so the longest side has this size.")
    parser.add_argument(
        "--rotate-deg",
        type=float,
        default=45.0,
        help="Rotate the drawing about its own center before fitting. For a non-square layout this can increase the achievable scale within the circular working area. Set to 0 to disable.",
    )
    parser.add_argument("--origin-x", type=float, default=0.0, help="Machine X coordinate for the selected anchor.")
    parser.add_argument("--origin-y", type=float, default=0.0, help="Machine Y coordinate for the selected anchor.")
    parser.add_argument("--anchor", choices=["lower-left", "center", "drawing-origin"], default="lower-left")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--flatness-mm", type=float, default=0.02, help="Curve flattening tolerance in machine millimeters.")
    parser.add_argument("--max-segment-mm", type=float, default=0.0, help="Optionally split long moves into smaller segments.")
    parser.add_argument("--min-length-mm", type=float, default=0.001, help="Skip paths shorter than this.")
    parser.add_argument("--feed-rate", type=float, default=DEFAULT_FEED_RATE_MM_S, help="Laser-on path speed in mm/s.")
    parser.add_argument("--travel-rate", type=float, default=DEFAULT_TRAVEL_RATE_MM_S, help="Laser-off travel speed in mm/s.")
    parser.add_argument("--passes", type=int, default=1)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--position-tolerance", type=float, default=DEFAULT_POSITION_TOLERANCE_MM)
    parser.add_argument("--command-settle", type=float, default=DEFAULT_COMMAND_SETTLE_S)
    parser.add_argument("--z-defocus-mm", type=float, default=DEFAULT_Z_DEFOCUS_MM)
    parser.add_argument("--z-speed", type=float, default=DEFAULT_Z_SPEED_MM_S, help="Stage Z velocity in mm/s.")
    parser.add_argument("--no-optimize", action="store_true", help="Keep CAD entity order instead of nearest-neighbor path ordering.")
    parser.add_argument("--live-default", action="store_true", help="Generated script defaults to live serial execution instead of dry-run.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    source = args.input.resolve()
    if not source.exists():
        raise FileNotFoundError(source)

    output = (args.output or source.with_name(f"{source.stem}_esp300.py")).resolve()
    preview = (args.preview or output.with_suffix(".svg")).resolve()
    dxf_path = ensure_dxf(source, output)

    raw_paths, doc, skipped = extract_paths(
        dxf_path,
        include_layers=args.include_layer,
        exclude_layers=args.exclude_layer,
        flatness_mm=args.flatness_mm,
        unit_scale=args.unit_scale,
        min_length_mm=args.min_length_mm,
    )
    if not raw_paths:
        raise RuntimeError("No drawable paths were found. Check layers and CAD entities.")

    if args.trace_mode == "design" and not args.include_layer:
        raw_paths = select_design_paths(raw_paths, skipped)
        if not raw_paths:
            raise RuntimeError("Auto trace removed every path. Use --trace-mode all or --include-layer.")

    raw_paths = rotate_paths(raw_paths, args.rotate_deg)

    usable_diameter_mm = 25.0
    usable_margin_mm = 0.05
    usable_center_xy_mm = [usable_diameter_mm / 2.0, usable_diameter_mm / 2.0]
    glass_size_mm = [usable_diameter_mm, usable_diameter_mm]
    start_xy_mm = usable_center_xy_mm[:]

    scale = apply_fit_scale(
        raw_paths,
        infer_unit_scale(doc, args.unit_scale),
        args.fit_width_mm,
        args.fit_height_mm,
        args.fit_longest_mm,
    )
    scale = cap_scale_to_circle(raw_paths, scale, diameter_mm=usable_diameter_mm, margin_mm=usable_margin_mm)
    machine_paths, source_bbox, machine_bbox = transform_paths(
        raw_paths,
        unit_scale=scale,
        origin=(usable_center_xy_mm[0], usable_center_xy_mm[1]),
        anchor="center",
        flip_x=args.flip_x,
        flip_y=args.flip_y,
        max_segment_mm=args.max_segment_mm,
    )

    if not args.no_optimize:
        machine_paths = optimize_path_order(machine_paths, start=(start_xy_mm[0], start_xy_mm[1]))

    drawing_data = DrawingData(
        paths=machine_paths,
        source_path=source,
        dxf_path=dxf_path,
        unit_scale=scale,
        raw_bbox=source_bbox,
        machine_bbox=machine_bbox,
        skipped=skipped,
    )

    metadata = {
        "source": str(source),
        "dxf": str(dxf_path),
        "unit_scale": scale,
        "origin": usable_center_xy_mm,
        "anchor": "center",
        "flip_x": args.flip_x,
        "flip_y": args.flip_y,
        "glass_size_mm": glass_size_mm,
        "start_xy_mm": start_xy_mm,
        "usable_area_shape": "circle",
        "usable_diameter_mm": usable_diameter_mm,
        "usable_center_xy_mm": usable_center_xy_mm,
        "usable_margin_mm": usable_margin_mm,
        "feed_rate": args.feed_rate,
        "travel_rate": args.travel_rate,
        "focus_z_mm": 0.0,
        "z_defocus_mm": args.z_defocus_mm,
        "z_speed_mm_s": args.z_speed,
        "position_tolerance_mm": args.position_tolerance,
        "passes": args.passes,
        "trace_mode": args.trace_mode,
        "fit_width_mm": args.fit_width_mm,
        "fit_height_mm": args.fit_height_mm,
        "fit_longest_mm": args.fit_longest_mm,
        "path_count": len(machine_paths),
        "point_count": sum(len(laser_path.points) for laser_path in machine_paths),
        "total_length_mm": round(sum(laser_path.length for laser_path in machine_paths), 6),
    }

    script = generate_esp300_script(
        machine_paths,
        output=output,
        metadata=metadata,
        port=args.port,
        baudrate=args.baudrate,
        feed_rate=args.feed_rate,
        travel_rate=args.travel_rate,
        passes=args.passes,
        position_tolerance=args.position_tolerance,
        command_settle=args.command_settle,
        dry_run_default=not args.live_default,
        z_defocus_mm=args.z_defocus_mm,
        z_speed=args.z_speed,
    )
    notebook = None
    if args.ipynb or args.notebook_output:
        notebook = (args.notebook_output or output.with_suffix(".ipynb")).resolve()
        write_script_notebook(
            script,
            notebook,
            title=f"{source.stem} ESP300 laser path",
            source_script=output,
        )
    write_preview_svg(machine_paths, preview)

    print(summarize(machine_paths, drawing_data))
    print(f"Generated script: {output}")
    if notebook:
        print(f"Generated notebook: {notebook}")
    print(f"Preview SVG: {preview}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
