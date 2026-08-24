# DWG to ESP300 Code Context

This repository/package is a local tool for converting DWG/DXF CAD geometry into
ESP300 stage-control Python code for laser engraving on a small glass sample.

The key physical assumption is that the laser beam is fixed and the glass/sample
stage moves in X, Y, and Z. During laser-off travel moves, the glass stage is
defocused in Z so the laser is not focused on the sample. During cutting moves,
the stage returns to the focus Z and the X/Y stage moves along the drawing path.
Default live speeds are set to conservative field-code values: cutting
`0.4 mm/s`, travel `1.0 mm/s`, Z motion `0.3 mm/s`, and serial command settle
`0.05 s`.

## Main Files

| File | Role |
| --- | --- |
| `dwg_to_esp300.py` | Command-line converter and ESP300 script generator. |
| `visualizer_server.py` | Local HTTP server/API for trace, plan, export, and download. |
| `visualizer/index.html` | Browser UI shell. |
| `visualizer/styles.css` | Browser UI styling. |
| `visualizer/app.js` | Frontend state, 2D canvas preview, and Three.js 3D visualization. |
| `visualizer/vendor/three.module.js` | Local Three.js dependency for the 3D view. |
| `adjust25design_2passes_esp300.py` | Example generated ESP300 script using two passes. |
| `adjust25design_2passes_esp300.ipynb` | Notebook form of the generated script. |
| `adjust25design_2passes_preview.svg` | Preview of the generated two-pass path. |
| `launch_dwg_to_esp300.ps1` | Windows launcher for the local app. |
| `requirements.txt` | Python dependencies. |
| `DWG_to_ESP300_Code_Overview_Lab_Report.pptx` | Plain lab-report slide deck explaining the code. |

## Conversion Pipeline

1. `dwg_to_esp300.py` accepts DWG or DXF input.
2. DWG input is converted to DXF when needed.
3. `ezdxf` extracts drawable geometry and flattens curves into point sequences.
4. The code filters likely annotation/helper geometry in design trace mode.
5. Geometry is scaled to millimeters and centered on the glass.
6. Scaling is capped so the path fits inside the centered 25 mm usable circle.
7. Optional path ordering optimization reduces non-cutting travel distance.
8. The generator writes an ESP300 Python script, optional notebook, and SVG preview.

## Motion Planning Logic

The server-side visualizer builds motion segments from selected paths:

- `z-defocus`: move glass Z away from focus before laser-off travel.
- `travel`: move X/Y with laser off and Z defocused.
- `z-focus`: return glass Z to focus before cutting.
- `cut`: move X/Y with laser on at focus Z.

The generated ESP300 script follows the same logic.

## Multi-Pass Logic

Each path can have its own pass count.

- Open paths alternate direction on repeated passes: forward, backward, forward,
  and so on.
- Closed paths repeat from an equivalent closed-loop start.
- The optimizer considers the expected endpoint after the requested pass count.

The optimizer is a heuristic, not a mathematical proof of the global shortest
route. It uses nearest-neighbor ordering and limited local search to reduce
travel.

## ESP300 Command Pattern

The generated script uses basic ESP300-style serial commands:

- `MO`: motor on
- `VA`: set axis velocity
- `PA`: absolute position move
- `TP`: read current position

X/Y synchronization is implemented by calculating a common move time for each
line segment:

```text
move_time = vector_distance / command_speed
vx = abs(dx) / move_time
vy = abs(dy) / move_time
```

Each axis receives its own velocity so both axes reach the endpoint together.

## Robustness Details

- `TP_READ_RETRIES = 5`
- Empty or unparseable `TP` replies are retried rather than treated as zero.
- Position wait loops tolerate intermittent TP failures.
- Extra settle time is applied after laser-off travel and Z focus/defocus moves.
- Z travel duration is computed from Z distance and configured Z speed.

## UI / Visualizer Details

The browser app provides:

- 2D drawing preview with the centered usable circle.
- 3D visualization showing the glass plate moving under a fixed laser tip.
- Path selection and per-path pass controls.
- Feed speed, travel speed, and Z speed controls.
- Export button that writes a generated ESP300 script into `generated/`.

The current UI uses side-by-side 2D and 3D panels on desktop. On small screens,
the panels stack vertically.

## Safety Notes

Before live use, confirm:

- The COM port and baudrate match the ESP300.
- Mechanical zero corresponds to the assumed glass center.
- The usable area matches the mounted sample and tape placement.
- The generated path stays inside the 25 mm circular usable area.
- Laser control is manual unless `LASER_ON_CMD` and `LASER_OFF_CMD` are filled
  with the correct device commands.

## Suggested Review Starting Points

1. Read `dwg_to_esp300.py` around `generate_esp300_script()`.
2. Read the generated `adjust25design_2passes_esp300.py` around `run_paths()`.
3. Read `visualizer_server.py` around `build_segments()`, `expand_passes()`, and
   `optimize_items()`.
4. Read `visualizer/app.js` around `updatePlan()`, `rebuildStage3d()`, and
   `updateStage3d()`.
