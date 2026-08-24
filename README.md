# DWG/DXF to ESP300 laser path

This project turns CAD geometry into an ESP300 stage-control Python script.
It follows the style of `ESP300_Controller.ipynb`: serial commands such as
`1PA...`, `2PA...`, `1VA...`, and `2VA...` are generated for the X/Y stage.

## Generate code from the sample DWG

The generated stage coordinates are always in millimeters. For the sample DWG,
the longest side is calibrated to 25 mm, which also makes the circular holes
about 2 mm in diameter.

```powershell
py dwg_to_esp300.py `
  "C:\Users\QCQN\Downloads\laser_shield\adjust25design.dwg" `
  --output adjust25design_esp300.py `
  --preview adjust25design_preview.svg `
  --fit-longest-mm 25 `
  --origin-x 0 `
  --origin-y 0 `
  --feed-rate 0.4 `
  --travel-rate 1.0 `
  --passes 1 `
  --ipynb
```

Generated files:

- `adjust25design_esp300.py`: executable ESP300 motion script
- `adjust25design_esp300.ipynb`: Jupyter notebook version when `--ipynb` is used
- `adjust25design_preview.svg`: path preview in machine millimeters
- `adjust25design.converted.dxf`: intermediate DXF when the input is DWG

Use `--notebook-output path\to\job.ipynb` if you want to choose the notebook
path explicitly.

## Run the generated machine script

Open and inspect the generated script first. The generated file now follows the
notebook style: `send()`, `move_abs()`, `move_rel()`, and `set_velocity()` are
top-level functions. Repeated cuts are stored as `PASSES = 5` when all selected
paths share the same count, or as per-path `passes` when they differ. The laser
hooks are intentionally no-ops unless you fill in laser trigger commands.
Default live speeds follow the field notebooks conservatively: `0.4 mm/s` for
cutting, `1.0 mm/s` for travel, and `0.3 mm/s` for Z motion.

```powershell
py adjust25design_esp300.py --dry-run
py adjust25design_esp300.py --live --port COM5
py adjust25design_esp300.py --live --port COM5 --passes 2
```

In Jupyter, open the generated `.ipynb`, run the setup cells, inspect
`METADATA`, and only set `LIVE_RUN = True` after checking the preview, COM port,
pass count, speed, and laser trigger commands.

## Visualize the generated movement

```powershell
py visualizer_server.py --host 127.0.0.1 --port 8765
```

Then open:

```text
http://127.0.0.1:8765
```

The visualizer reads the generated ESP300 scripts and animates the exact move
sequence. The full drawing view shows the 25.000 mm side; the outline view uses
only the `외형선(ISO)` layer.

The browser interface can now trace a new DWG/DXF directly:

- Choose a `.dwg` or `.dxf` file and press `Trace File`, or paste a local
  drawing path and press `Trace Path`.
- Use `직접 조합` to build paths without CAD. `빈 작업` starts an empty mm
  workspace, then add `직선`, `사각형`, or a fixed 2 mm `원`.
- `긴 변(mm)` sets the real-world size of the longest side. Use `25` for the
  current sample.
- Coordinates are generated in mm, and the circular-hole check is fixed at
  2 mm so it cannot be changed accidentally.
- Select paths from the canvas or the path list.
- Set pass counts per path, or use the global pass box and `Apply`.
- Keep `Optimize` enabled to reorder selected paths and repeated passes to
  reduce laser-off travel.
- Press `코드 다운로드` to write the selected plan under `generated/` and
  download the generated ESP300 Python file in the browser.

By default, generated path order is optimized to reduce laser-off travel moves.
Use `--no-optimize` only when you need to preserve the original CAD entity order.
By default, `--trace-mode design` excludes obvious watermark/support geometry
such as `layer 0` evaluation text and full-page diagonal strokes. Use
`--trace-mode all` only when you intentionally want every CAD entity.

## Notes

- DWG input is converted to DXF automatically with ODA File Converter when it is
  installed. If ODA is not found, the script tries `aspose-cad`.
- DXF input is parsed directly with `ezdxf`.
- The converter supports common curve geometry through `ezdxf.path.make_path`,
  including lines, polylines, arcs, circles, ellipses, and splines when `ezdxf`
  can flatten them.
- `--include-layer` and `--exclude-layer` accept shell-style wildcards.
