DWG to ESP300 Lab Code Package
Date: 2026-08-16

Purpose
This package converts DWG/DXF drawing geometry into ESP300 stage-control Python
code for laser engraving on a 25 mm x 25 mm glass sample. The intended physical
setup is a fixed laser beam and a moving glass/sample stage.

Main entry points
- launch_dwg_to_esp300.ps1
  Starts the local visualizer server and opens the browser UI.

- visualizer_server.py
  Local HTTP server and API for tracing drawings, planning motion, and exporting
  ESP300 Python code.

- dwg_to_esp300.py
  Command-line converter from DWG/DXF geometry to ESP300 motion code, SVG
  preview, and optional Jupyter notebook.

- visualizer/
  Browser UI for 2D drawing preview and 3D moving-glass visualization.

Included generated example
- adjust25design_2passes_esp300.py
  Example generated ESP300 script with passes=2.

- adjust25design_2passes_esp300.ipynb
  Notebook version of the same generated script.

- adjust25design_2passes_preview.svg
  Static preview of the generated path.

Current motion assumptions
- Glass size: 25 mm x 25 mm.
- Usable area: centered 25 mm diameter circle with 0.05 mm margin.
- Start point: center of the glass, (12.5 mm, 12.5 mm).
- Laser is treated as fixed; X/Y/Z motion belongs to the glass/sample stage.
- Laser-off travel defocuses the glass stage in Z by 1 mm by default.
- Default live speeds are conservative field-code defaults:
  cut 0.4 mm/s, travel 1.0 mm/s, and Z 0.3 mm/s.
- Serial command settle time defaults to 0.05 s.
- Z motion waits according to the configured Z speed and settle time.
- Continuous cutting segments do not unnecessarily change Z.
- X/Y synchronization is handled by assigning component velocities so both axes
  arrive at the segment endpoint at the same time.
- TP position reads retry up to 5 times to tolerate empty ESP300 replies.

Before live use
1. Run a dry run first.
2. Confirm mechanical zero and the actual sample mounting.
3. Confirm the path stays inside the usable circular area.
4. Confirm COM port and baudrate.
5. Confirm whether laser TTL is manual or connected to ESP300/another serial
   device; LASER_ON_CMD and LASER_OFF_CMD are placeholders by default.

Not included
- Email drafts or .eml files.
- Temporary screenshots.
- Old review package folders.
- Python cache files.
