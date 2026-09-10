import * as THREE from "./vendor/three.module.js";

const canvas = document.getElementById("previewCanvas");
const ctx = canvas.getContext("2d");
const stageRoot = document.getElementById("stage3d");
const laserState = document.getElementById("laserState");
const zLabel = document.getElementById("zLabel");
const segmentLabel = document.getElementById("segmentLabel");
const fileInput = document.getElementById("fileInput");
const sourcePath = document.getElementById("sourcePath");
const traceFileButton = document.getElementById("traceFileButton");
const tracePathButton = document.getElementById("tracePathButton");
const clearFileButton = document.getElementById("clearFileButton");
const sampleButton = document.getElementById("sampleButton");
const traceMode = document.getElementById("traceMode");
const fitLongest = document.getElementById("fitLongest");
const scaleFactor = document.getElementById("scaleFactor");
const applyScaleButton = document.getElementById("applyScaleButton");
const rotationDeg = document.getElementById("rotationDeg");
const feedRate = document.getElementById("feedRate");
const travelRate = document.getElementById("travelRate");
const zSpeed = document.getElementById("zSpeed");
const optimizeToggle = document.getElementById("optimizeToggle");
const exportButton = document.getElementById("exportButton");
const restartButton = document.getElementById("restartButton");
const playButton = document.getElementById("playButton");
const scrubber = document.getElementById("scrubber");
const timeLabel = document.getElementById("timeLabel");
const coordLabel = document.getElementById("coordLabel");
const statusText = document.getElementById("statusText");
const bboxMetric = document.getElementById("bboxMetric");
const pathMetric = document.getElementById("pathMetric");
const cutMetric = document.getElementById("cutMetric");
const travelMetric = document.getElementById("travelMetric");
const pathList = document.getElementById("pathList");
const selectAllButton = document.getElementById("selectAllButton");
const selectNoneButton = document.getElementById("selectNoneButton");
const globalPasses = document.getElementById("globalPasses");
const applyPassesButton = document.getElementById("applyPassesButton");
const shapeTabs = document.querySelectorAll(".shape-tab");
const shapeFields = document.querySelectorAll(".shape-fields");
const panelTabs = document.querySelectorAll(".panel-tab");
const tabPanes = document.querySelectorAll(".tab-pane");
// Jog tab active -> relocate the live .camera-view into the drawing preview
// (large, watchable while jogging); any other tab -> put it back. Same DOM
// node both places (not a second <img>), so the one open MJPEG connection
// and the click-to-target handler (bound to #cameraStream, coordinates read
// from getBoundingClientRect at click time) keep working unchanged either way.
const cameraViewBox = document.querySelector(".camera-view");
const cameraViewHomeParent = cameraViewBox.parentNode;
const cameraViewHomeNext = cameraViewBox.nextElementSibling;
const canvasWrap = document.querySelector(".canvas-wrap");
const cameraIndex = document.getElementById("cameraIndex");
const cameraConnectButton = document.getElementById("cameraConnectButton");
const cameraDisconnectButton = document.getElementById("cameraDisconnectButton");
const cameraStatusText = document.getElementById("cameraStatusText");
const cameraStream = document.getElementById("cameraStream");
const cameraPlaceholder = document.getElementById("cameraPlaceholder");
const cameraBrightnessText = document.getElementById("cameraBrightnessText");
const cameraSpotText = document.getElementById("cameraSpotText");
const focusScanButton = document.getElementById("focusScanButton");
const focusScanWideButton = document.getElementById("focusScanWideButton");
const focusScanNarrowButton = document.getElementById("focusScanNarrowButton");
const focusSurveyNarrowToggle = document.getElementById("focusSurveyNarrowToggle");
const focusCustomDirection = document.getElementById("focusCustomDirection");
const focusCustomRange = document.getElementById("focusCustomRange");
const focusCustomStep = document.getElementById("focusCustomStep");
const focusCustomSettle = document.getElementById("focusCustomSettle");
const focusCustomButton = document.getElementById("focusCustomButton");
const focusChart = document.getElementById("focusChart");
const focusSelectionText = document.getElementById("focusSelectionText");
const focusActiveToggle = document.getElementById("focusActiveToggle");
const focusStatusText = document.getElementById("focusStatusText");
const focusResultText = document.getElementById("focusResultText");
const focusLatencyButton = document.getElementById("focusLatencyButton");
const focusLatencyText = document.getElementById("focusLatencyText");
const focusSurveyButton = document.getElementById("focusSurveyButton");
const focusSurveyStopButton = document.getElementById("focusSurveyStopButton");
const focusSurveyStatusText = document.getElementById("focusSurveyStatusText");
const focusSurveyMap = document.getElementById("focusSurveyMap");
const focusSurveyMapTip = document.getElementById("focusSurveyMapTip");
const jogPort = document.getElementById("jogPort");
const jogBaud = document.getElementById("jogBaud");
const jogDryRun = document.getElementById("jogDryRun");
const jogRangeXY = document.getElementById("jogRangeXY");
const jogRangeZ = document.getElementById("jogRangeZ");
const jogConnectButton = document.getElementById("jogConnectButton");
const jogDisconnectButton = document.getElementById("jogDisconnectButton");
const jogStatusText = document.getElementById("jogStatusText");
const jogStepOptions = document.getElementById("jogStepOptions");
const jogPad = document.querySelector(".jog-pad");
const jogZRow = document.querySelector(".jog-z");
const jogEstopButton = document.getElementById("jogEstop");
const jogHomeButton = document.getElementById("jogHomeButton");
const jogSetLocalHomeButton = document.getElementById("jogSetLocalHomeButton");
const jogGoLocalHomeButton = document.getElementById("jogGoLocalHomeButton");
const jogGotoX = document.getElementById("jogGotoX");
const jogGotoY = document.getElementById("jogGotoY");
const jogGotoZ = document.getElementById("jogGotoZ");
const jogGotoButton = document.getElementById("jogGotoButton");
const jogRunButton = document.getElementById("jogRunButton");
const jogRunStopButton = document.getElementById("jogRunStopButton");
const jogRunStatusText = document.getElementById("jogRunStatusText");
const jogPositionText = document.getElementById("jogPositionText");
const jogLimitText = document.getElementById("jogLimitText");
const jogBlockedText = document.getElementById("jogBlockedText");

const newManualButton = document.getElementById("newManualButton");
const addShapeButton = document.getElementById("addShapeButton");
const deleteSelectedButton = document.getElementById("deleteSelectedButton");
const lineX1 = document.getElementById("lineX1");
const lineY1 = document.getElementById("lineY1");
const lineX2 = document.getElementById("lineX2");
const lineY2 = document.getElementById("lineY2");
const rectX = document.getElementById("rectX");
const rectY = document.getElementById("rectY");
const rectW = document.getElementById("rectW");
const rectH = document.getElementById("rectH");
const circleX = document.getElementById("circleX");
const circleY = document.getElementById("circleY");
const circleDiameter = document.getElementById("circleDiameter");
const spiralX = document.getElementById("spiralX");
const spiralY = document.getElementById("spiralY");
const spiralRadius = document.getElementById("spiralRadius");
const spiralTurns = document.getElementById("spiralTurns");

let session = null;
let motion = null;
let lastTraceMode = "path";
let jogIsLive = false;
let jogLivePoint3 = null;
let selected = new Set();
let passCounts = new Map();
let hoverPath = null;
let playhead = 0;
let playing = false;
let lastFrame = performance.now();
let cumulative = [];
let planTimer = null;
let shapeMode = "line";
let stageRenderer = null;
let stageScene = null;
let stageCamera = null;
let stageGroup = null;
let sampleGroup = null;
let staticRouteGroup = null;
let progressRouteGroup = null;
let toolGroup = null;
let motionCueGroup = null;
let beamLine = null;
let focusRing = null;
let laserSpot = null;
let contactRing = null;
let zGapLine = null;
let stageNeedsRebuild = true;
let stageDrag = null;
let stageYaw = -0.72;
let stagePitch = 0.58;
let stageDistance = 58;

const Z_VISUAL_SCALE = 8;
const GLASS_THICKNESS = 0.14;
const GLASS_CENTER_Y = -0.035;
const GLASS_SURFACE_Y = 0.055;
const FOCUS_PLANE_Y = GLASS_SURFACE_Y;
const PLAYBACK_RATE = 1;
const STAGE_COLORS = {
  cut: 0xd84335,
  travel: 0x6f7c84,
  z: 0x1d7fd1,
  future: 0xcfd7dd,
  head: 0x07847e,
  glass: 0xbfefff,
  tape: 0xe8c987,
  carrier: 0x303438,
};

const mm = (value, digits = 2) => `${Number(value).toFixed(digits)} mm`;
const sec = (value) => `${Number(value).toFixed(1)} s`;

function setStatus(text) {
  statusText.textContent = text;
}

function requestPayload() {
  return {
    sessionId: session?.id,
    feedRate: Number(feedRate.value || 0.4),
    travelRate: Number(travelRate.value || 1.0),
    focusZMm: 0,
    zDefocusMm: 1,
    zSpeedMmS: Number(zSpeed?.value || 0.3),
    positionTolerance: 0.003,
    optimize: optimizeToggle.checked,
    paths: [...selected].map((id) => ({ id, passes: Math.max(1, Number(passCounts.get(id) || 1)) })),
  };
}

function traceParams() {
  return {
    sourcePath: sourcePath.value.trim(),
    traceMode: traceMode.value,
    fitLongestMm: fitLongest.value,
    rotationDeg: Number(rotationDeg.value || 0),
    feedRate: Number(feedRate.value || 0.4),
    travelRate: Number(travelRate.value || 1.0),
  };
}

function readNumber(input, label) {
  const value = Number(input.value);
  if (!Number.isFinite(value)) throw new Error(`Check ${label} value.`);
  return value;
}
function makeCirclePoints(cx, cy, diameter, segments = 96) {
  const radius = diameter / 2;
  const points = [];
  for (let i = 0; i <= segments; i += 1) {
    const angle = (Math.PI * 2 * i) / segments;
    points.push([cx + Math.cos(angle) * radius, cy + Math.sin(angle) * radius]);
  }
  return points;
}

function makeSpiralPoints(cx, cy, maxR, turns, pointsPerTurn = 48) {
  const totalSegments = Math.max(2, Math.round(turns * pointsPerTurn));
  const points = [];
  for (let i = 0; i <= totalSegments; i += 1) {
    const t = i / totalSegments; // 0 (center) -> 1 (outer edge)
    const angle = t * turns * Math.PI * 2;
    const r = t * maxR;
    points.push([cx + Math.cos(angle) * r, cy + Math.sin(angle) * r]);
  }
  return points;
}

function buildSpiralShapePath() {
  const cx = readNumber(spiralX, "CX");
  const cy = readNumber(spiralY, "CY");
  const maxR = readNumber(spiralRadius, "Max radius");
  const turns = readNumber(spiralTurns, "Turns");
  if (maxR <= 0) throw new Error("Max radius must be positive.");
  if (turns <= 0) throw new Error("Turns must be positive.");
  return {
    layer: `Manual spiral R${maxR}mm x${turns}`,
    entityType: "MANUAL_SPIRAL",
    closed: false,
    points: makeSpiralPoints(cx, cy, maxR, turns),
  };
}

function makeShapePath() {
  if (shapeMode === "line") {
    const x1 = readNumber(lineX1, "X1");
    const y1 = readNumber(lineY1, "Y1");
    const x2 = readNumber(lineX2, "X2");
    const y2 = readNumber(lineY2, "Y2");
    if (Math.hypot(x2 - x1, y2 - y1) <= 0.000001) throw new Error("Line length is zero.");
    return {
      layer: "Manual line",
      entityType: "MANUAL_LINE",
      closed: false,
      points: [
        [x1, y1],
        [x2, y2],
      ],
    };
  }

  if (shapeMode === "rect") {
    const x = readNumber(rectX, "X");
    const y = readNumber(rectY, "Y");
    const width = readNumber(rectW, "W");
    const height = readNumber(rectH, "H");
    if (width <= 0 || height <= 0) throw new Error("Rectangle width and height must be positive.");
    return {
      layer: "Manual rectangle",
      entityType: "MANUAL_RECTANGLE",
      closed: true,
      points: [
        [x, y],
        [x + width, y],
        [x + width, y + height],
        [x, y + height],
        [x, y],
      ],
    };
  }

  if (shapeMode === "spiral") return buildSpiralShapePath();

  const cx = readNumber(circleX, "CX");
  const cy = readNumber(circleY, "CY");
  const diameter = readNumber(circleDiameter, "Diameter");
  return {
    layer: `Manual circle ${diameter}mm`,
    entityType: "MANUAL_CIRCLE",
    closed: true,
    points: makeCirclePoints(cx, cy, diameter),
  };
}

function currentPathsForManual(extraPath = null) {
  const paths = session
    ? session.paths.map((path) => ({
        layer: path.rawLayer || path.layer,
        entityType: path.entityType,
        closed: path.closed,
        points: path.points,
        passes: Math.max(1, Number(passCounts.get(path.id) || path.passes || 1)),
        selected: selected.has(path.id),
      }))
    : [];

  if (extraPath) {
    paths.push({
      ...extraPath,
      passes: 1,
      selected: true,
    });
  }
  return paths;
}

async function createManualSession(paths, status) {
  const data = await fetchJson("/api/manual", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      name: "Manual design",
      feedRate: Number(feedRate.value || 0.4),
      travelRate: Number(travelRate.value || 1.0),
      paths,
    }),
  });
  applySession(data);
  setStatus(status);
}

async function addShape() {
  const shapePath = makeShapePath();
  await createManualSession(currentPathsForManual(shapePath), `${shapePath.layer} added`);
}

async function deleteSelectedPaths() {
  if (!session || selected.size === 0) {
    setStatus("Select paths to delete.");
    return;
  }
  const remaining = currentPathsForManual().filter((_, index) => !selected.has(session.paths[index].id));
  await createManualSession(remaining.map((path) => ({ ...path, selected: true })), "Selected paths deleted.");
}

function setPanelTab(tab) {
  for (const button of panelTabs) button.classList.toggle("active", button.dataset.tab === tab);
  for (const pane of tabPanes) pane.classList.toggle("hidden", pane.dataset.tabPane !== tab);
  updateCameraBigView(tab);
}

function updateCameraBigView(tab) {
  if (tab === "jog") {
    canvasWrap.appendChild(cameraViewBox);
    cameraViewBox.classList.add("camera-view--big");
  } else {
    cameraViewBox.classList.remove("camera-view--big");
    cameraViewHomeParent.insertBefore(cameraViewBox, cameraViewHomeNext);
  }
}

function setShapeMode(mode) {
  shapeMode = mode;
  for (const tab of shapeTabs) tab.classList.toggle("active", tab.dataset.shape === mode);
  for (const fields of shapeFields) fields.classList.toggle("hidden", fields.dataset.fields !== mode);
}

async function fetchJson(url, options = {}) {
  const response = await fetch(url, options);
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || response.statusText);
  return payload;
}

let cameraConnected = false;
let cameraPollTimer = null;

function cameraApplyStatus(data) {
  cameraConnected = Boolean(data.connected);
  if (!cameraConnected) {
    cameraStatusText.textContent = "Disconnected";
    cameraStatusText.classList.remove("live");
    cameraStream.removeAttribute("src");
    cameraPlaceholder.style.display = "flex";
    cameraBrightnessText.textContent = "Brightness: -";
    cameraSpotText.textContent = "Spot: -";
    return;
  }
  cameraStatusText.textContent = `Connected: index ${data.index}`;
  cameraStatusText.classList.add("live");
  cameraPlaceholder.style.display = "none";
  if (!cameraStream.getAttribute("src")) {
    // Cache-bust so a reconnect after a stream drop starts a fresh MJPEG response.
    cameraStream.src = `/api/camera/stream?t=${Date.now()}`;
  }
  const brightness = data.brightness || {};
  if (brightness.mean != null && brightness.max != null) {
    cameraBrightnessText.textContent = `Brightness: mean ${brightness.mean.toFixed(1)} / max ${brightness.max.toFixed(1)}`;
  } else {
    cameraBrightnessText.textContent = "Brightness: -";
  }
  const peak = data.peak;
  if (peak && peak.n_blobs > 0) {
    const pos = peak.x != null ? `x ${peak.x.toFixed(0)}, y ${peak.y.toFixed(0)}` : "no centroid";
    cameraSpotText.textContent = `Peak: score ${peak.score.toFixed(0)} / ${peak.n_blobs} blob(s) / ${pos}`;
  } else {
    cameraSpotText.textContent = "Peak: not found";
  }
}

async function cameraRefreshStatus() {
  try {
    const data = await fetchJson("/api/camera/status");
    cameraApplyStatus(data);
  } catch (error) {
    // Transient poll failures are ignored; connect/disconnect actions surface real errors.
  }
}

async function cameraConnect() {
  try {
    const data = await fetchJson("/api/camera/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ index: Number(cameraIndex.value || 0) }),
    });
    cameraApplyStatus(data);
    setStatus(`Camera connected on index ${data.index}.`);
  } catch (error) {
    setStatus(error.message);
  }
}

async function cameraDisconnect() {
  try {
    const data = await fetchJson("/api/camera/disconnect", { method: "POST" });
    cameraApplyStatus(data);
    setStatus("Camera disconnected.");
  } catch (error) {
    setStatus(error.message);
  }
}

// Click the blob you actually want on the live feed: several red+core
// candidates can be onscreen at once (reflections, stray hot spots), and
// only the user knows which one is the real target. This sets the
// continuity anchor server-side so frame-to-frame tracking locks onto
// whichever candidate lands nearest the click, instead of guessing by size.
async function cameraSelectTarget(evt) {
  if (!cameraConnected || !cameraStream.naturalWidth) return;
  const rect = cameraStream.getBoundingClientRect();
  const scaleX = cameraStream.naturalWidth / rect.width;
  const scaleY = cameraStream.naturalHeight / rect.height;
  const x = (evt.clientX - rect.left) * scaleX;
  const y = (evt.clientY - rect.top) * scaleY;
  try {
    await fetchJson("/api/camera/select-target", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x, y }),
    });
    setStatus(`Target anchor set at (${x.toFixed(0)}, ${y.toFixed(0)}).`);
  } catch (error) {
    setStatus(error.message);
  }
}

let focusPollTimer = null;
let focusSuppressToggle = false;

function focusApplyStatus(data) {
  if (data.focusTiming) focusTimingConfig = data.focusTiming;
  if (data.scanning) {
    focusStatusText.textContent = `Scanning (${data.stage || "..."})`;
    focusStatusText.classList.add("live");
  } else if (data.error) {
    focusStatusText.textContent = `Error: ${data.error}`;
    focusStatusText.classList.remove("live");
  } else if (data.active) {
    focusStatusText.textContent = "Active - watching for X/Y moves";
    focusStatusText.classList.add("live");
  } else {
    focusStatusText.textContent = data.hasBaseline ? "Idle" : "Idle (no scan yet)";
    focusStatusText.classList.remove("live");
  }

  const result = data.lastResult;
  if (result) {
    const when = new Date(result.timestamp * 1000).toLocaleTimeString();
    const peak = result.peakBrightness != null ? result.peakBrightness.toFixed(1) : "-";
    const kind = result.label || (result.usedWide ? "wide+narrow" : "narrow");
    const offset = result.bestOffsetMm >= 0 ? `+${result.bestOffsetMm.toFixed(4)}` : result.bestOffsetMm.toFixed(4);
    const trend = data.planeSamples >= 3 ? ` / trend fit (${data.planeSamples} pts)` : ` / trend: need ${3 - data.planeSamples} more pt(s)`;
    focusResultText.textContent = `Last (${kind}): moved ${offset} mm, peak blob score ${peak} @ ${when}${trend}`;
  } else {
    focusResultText.textContent = "No scan yet.";
  }
  if (data.scanning && data.liveSamples && data.liveSamples.length > 1) {
    renderFocusChartLive(data.liveStage, data.liveSamples);
  } else {
    renderFocusChart(result);
  }

  if (!focusSuppressToggle) focusActiveToggle.checked = Boolean(data.active);

  const lc = data.latencyCheck;
  if (lc) {
    const when = new Date(lc.timestamp * 1000).toLocaleTimeString();
    const deltaUm = (lc.deltaMm * 1000).toFixed(1);
    const lagMs = (lc.impliedLagS * 1000).toFixed(0);
    focusLatencyText.textContent =
      `slow ${lc.slowBestOffsetMm >= 0 ? "+" : ""}${lc.slowBestOffsetMm.toFixed(4)}mm vs fast ${lc.fastBestOffsetMm >= 0 ? "+" : ""}${lc.fastBestOffsetMm.toFixed(4)}mm ` +
      `-> gap ${deltaUm}µm (~${lagMs}ms implied lag) @ ${when}`;
  } else if (!data.scanning) {
    focusLatencyText.textContent = "No timing check yet.";
  }

  const sv = data.survey;
  if (data.surveying) {
    focusSurveyStatusText.textContent = `Surveying: point ${sv.index}/${sv.total}${data.scanning ? ` (${data.stage || "scanning"})` : ""}`;
    focusSurveyStatusText.classList.add("live");
  } else if (sv) {
    const failed = sv.results.filter((r) => r.error).length;
    focusSurveyStatusText.textContent =
      `Survey done: ${sv.results.length}/${sv.total} points` + (failed ? `, ${failed} failed` : "") + (sv.error ? ` - stopped: ${sv.error}` : "");
    focusSurveyStatusText.classList.remove("live");
  } else {
    focusSurveyStatusText.textContent = "No survey yet.";
    focusSurveyStatusText.classList.remove("live");
  }
  renderSurveyMap(sv);
}

// In-panel Z-offset vs blob-score chart for the last completed scan, with
// drag-to-select: dragging a range on the graph computes a center offset +
// half-width relative to the CURRENT stage position (samples are offsets
// from wherever that particular scan started, not absolute Z -- see
// _run_stage server-side), pre-fills the custom-scan Range/Step fields, and
// stashes the center offset for focusRunCustomScan() to send along. This is
// what lets picking a peak on the graph immediately chain into a finer scan
// right there, and picking again on THAT scan's fresh graph chain further.
const FOCUS_CHART_PLOT = { padL: 42, padR: 8, padT: 8, padB: 20, w: 480, h: 200 };
let focusChartSamples = null;
let focusChartBestZ = null;
let focusChartTimestamp = null;
let focusSelection = null; // {min, max} in offset-mm units, in the currently-rendered scan's own frame
let focusSelectionCenterOffsetMm = 0;
let focusTimingConfig = null; // {speedMmS, mechSettleS, exposureMarginS, narrowRangeMm, narrowStepMm}, from status polls

function focusChartClear() {
  focusChart.innerHTML = "";
  focusChartSamples = null;
  focusChartBestZ = null;
  focusSelection = null;
  focusSelectionCenterOffsetMm = 0;
  focusSelectionText.textContent = "No range selected.";
}

function drawFocusChartCore(samples, peakZ, opts) {
  const { padL, padR, padT, padB, w, h } = FOCUS_CHART_PLOT;
  const innerW = w - padL - padR;
  const innerH = h - padT - padB;
  const xs = samples.map((s) => s[0]);
  const ys = samples.map((s) => s[1]);
  const xMin = Math.min(...xs);
  const xMax = Math.max(...xs);
  const yMin = Math.min(...ys);
  const yMax = Math.max(...ys);
  const yPad = (yMax - yMin) * 0.08 || 1;
  const yLo = yMin - yPad;
  const yHi = yMax + yPad;

  const xPix = (x) => padL + ((x - xMin) / (xMax - xMin || 1)) * innerW;
  const yPix = (y) => padT + innerH - ((y - yLo) / (yHi - yLo || 1)) * innerH;
  const xData = (px) => xMin + ((px - padL) / innerW) * (xMax - xMin);

  const ns = "http://www.w3.org/2000/svg";
  function el(tag, attrs) {
    const e = document.createElementNS(ns, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    return e;
  }

  focusChart.innerHTML = "";
  focusChart.setAttribute("viewBox", `0 0 ${w} ${h}`);

  const yTicks = 3;
  for (let i = 0; i <= yTicks; i += 1) {
    const py = yPix(yLo + (i / yTicks) * (yHi - yLo));
    focusChart.appendChild(el("line", { class: "focus-chart-grid", x1: padL, x2: w - padR, y1: py, y2: py }));
  }
  for (const xv of [xMin, (xMin + xMax) / 2, xMax]) {
    const label = el("text", { class: "focus-chart-axis", x: xPix(xv), y: h - 4, "text-anchor": "middle" });
    label.textContent = xv.toFixed(3);
    focusChart.appendChild(label);
  }
  focusChart.appendChild(el("line", { class: "focus-chart-axis", x1: padL, x2: w - padR, y1: h - padB, y2: h - padB }));

  const d = samples.map((s, i) => `${i === 0 ? "M" : "L"} ${xPix(s[0]).toFixed(2)} ${yPix(s[1]).toFixed(2)}`).join(" ");
  focusChart.appendChild(el("path", { class: opts.interactive ? "focus-chart-line" : "focus-chart-line live", d }));

  if (peakZ != null) {
    let nearestIdx = 0;
    for (let i = 1; i < samples.length; i += 1) {
      if (Math.abs(samples[i][0] - peakZ) < Math.abs(samples[nearestIdx][0] - peakZ)) nearestIdx = i;
    }
    focusChart.appendChild(el("circle", { class: "focus-chart-peak", cx: xPix(peakZ), cy: yPix(samples[nearestIdx][1]), r: 4 }));
  }

  if (!opts.interactive) return; // live/in-progress: read-only, no drag-select UI

  const selRect = el("rect", { class: "focus-chart-selection", y: padT, height: innerH, x: 0, width: 0, visibility: "hidden" });
  focusChart.appendChild(selRect);
  if (focusSelection) {
    const x0 = xPix(focusSelection.min);
    const x1 = xPix(focusSelection.max);
    selRect.setAttribute("x", Math.min(x0, x1));
    selRect.setAttribute("width", Math.max(1, Math.abs(x1 - x0)));
    selRect.setAttribute("visibility", "visible");
  }

  const hit = el("rect", { x: padL, y: padT, width: innerW, height: innerH, fill: "transparent" });
  focusChart.appendChild(hit);

  let dragStartData = null;
  function pointToData(evt) {
    const rect = focusChart.getBoundingClientRect();
    const px = (evt.clientX - rect.left) * (w / rect.width);
    return Math.min(xMax, Math.max(xMin, xData(px)));
  }
  hit.addEventListener("pointerdown", (evt) => {
    dragStartData = pointToData(evt);
    hit.setPointerCapture(evt.pointerId);
  });
  hit.addEventListener("pointermove", (evt) => {
    if (dragStartData == null) return;
    const cur = pointToData(evt);
    const lo = Math.min(dragStartData, cur);
    const hi = Math.max(dragStartData, cur);
    selRect.setAttribute("x", xPix(lo));
    selRect.setAttribute("width", Math.max(1, xPix(hi) - xPix(lo)));
    selRect.setAttribute("visibility", "visible");
  });
  hit.addEventListener("pointerup", (evt) => {
    if (dragStartData == null) return;
    const cur = pointToData(evt);
    const lo = Math.min(dragStartData, cur);
    const hi = Math.max(dragStartData, cur);
    dragStartData = null;
    if (hi - lo < (xMax - xMin) * 0.01) {
      selRect.setAttribute("visibility", focusSelection ? "visible" : "hidden");
      return; // too small to be a deliberate drag -- treat as a stray click
    }
    focusSelection = { min: lo, max: hi };
    applyFocusSelection();
  });
}

function renderFocusChart(result) {
  if (!result || !result.samples) {
    focusChartClear();
    return;
  }
  const samples = result.samples.narrow && result.samples.narrow.length > 1 ? result.samples.narrow : result.samples.wide || [];
  if (samples.length < 2) {
    focusChartClear();
    return;
  }
  if (result.timestamp === focusChartTimestamp) {
    // Same scan as last render (just another 400ms status poll finding
    // nothing new) -- skip the redraw. drawFocusChartCore rebuilds the
    // whole SVG from scratch, which would tear out the hit-rect a
    // drag-select is mid-flight on and silently drop the gesture.
    return;
  }
  // A fresh scan landed -- any prior selection was drawn against the old
  // graph and no longer corresponds to anything real.
  focusChartTimestamp = result.timestamp;
  focusSelection = null;
  focusSelectionCenterOffsetMm = 0;
  focusSelectionText.textContent = "No range selected.";
  focusChartSamples = samples;
  focusChartBestZ = result.bestOffsetMm;
  focusChartHint.textContent = "Drag on the graph to select a range around a peak.";
  drawFocusChartCore(samples, focusChartBestZ, { interactive: true });
}

// Live, read-only view of the scan currently in progress -- polled from
// data.liveSamples every 400ms while data.scanning is true, so the curve
// visibly grows point-by-point instead of the panel sitting blank/stale
// for the 1-50s a scan can take. Swaps back to the interactive final chart
// (renderFocusChart) the moment the scan completes and lastResult updates.
function renderFocusChartLive(stage, samples) {
  if (!samples || samples.length < 2) return;
  let peakIdx = 0;
  for (let i = 1; i < samples.length; i += 1) {
    if (samples[i][1] > samples[peakIdx][1]) peakIdx = i;
  }
  focusChartHint.textContent = `Live: ${stage || "scanning"} - ${samples.length} pt(s), running peak ${samples[peakIdx][0].toFixed(4)}mm`;
  drawFocusChartCore(samples, samples[peakIdx][0], { interactive: false });
}

function applyFocusSelection() {
  if (!focusSelection || focusChartBestZ == null) return;
  const mid = (focusSelection.min + focusSelection.max) / 2;
  const halfWidth = Math.max(0.0005, (focusSelection.max - focusSelection.min) / 2);
  focusSelectionCenterOffsetMm = mid - focusChartBestZ;
  focusCustomRange.value = halfWidth.toFixed(5);
  focusCustomStep.value = Math.max(0.0002, halfWidth / 20).toFixed(5);
  const sign = focusSelectionCenterOffsetMm >= 0 ? "+" : "";
  focusSelectionText.textContent =
    `Selected ${focusSelection.min.toFixed(4)} to ${focusSelection.max.toFixed(4)} mm -> ` +
    `center ${sign}${focusSelectionCenterOffsetMm.toFixed(4)}mm from current position, half-width ${halfWidth.toFixed(4)}mm. ` +
    "Adjust Step/Settle below, then Run custom scan.";
}

// X/Y survey map: fills in live from data.survey.results while a survey
// runs (polled every 400ms same as everything else here), and stays put
// showing the finished map afterward. focusSurveyWaypoints (set by
// focusStartSurvey) is client-side only -- it's what lets not-yet-visited
// stops show as faint pending dots; the server only echoes completed
// results, so a page reload mid-survey just loses the pending dots, not
// the real data.
let focusSurveyWaypoints = [];

function renderSurveyMap(survey) {
  const svg = focusSurveyMap;
  const hasPending = focusSurveyWaypoints.length > 0;
  const hasResults = survey && survey.results.length > 0;
  if (!hasPending && !hasResults) {
    svg.innerHTML = "";
    focusSurveyMapTip.textContent = "Hover a point for detail.";
    return;
  }

  const w = 480, h = 260, padL = 36, padR = 8, padT = 8, padB = 18;
  const innerW = w - padL - padR, innerH = h - padT - padB;

  const xs = [], ys = [];
  for (const [x, y] of focusSurveyWaypoints) { xs.push(x); ys.push(y); }
  if (survey) for (const r of survey.results) { xs.push(r.x); ys.push(r.y); }
  let xMin = Math.min(...xs), xMax = Math.max(...xs), yMin = Math.min(...ys), yMax = Math.max(...ys);
  const xSpan = Math.max(xMax - xMin, 0.5), ySpan = Math.max(yMax - yMin, 0.5);
  xMin -= xSpan * 0.1; xMax += xSpan * 0.1; yMin -= ySpan * 0.1; yMax += ySpan * 0.1;

  const xPix = (x) => padL + ((x - xMin) / (xMax - xMin)) * innerW;
  const yPix = (y) => padT + innerH - ((y - yMin) / (yMax - yMin)) * innerH; // Y grows upward on the glass

  const ns = "http://www.w3.org/2000/svg";
  function el(tag, attrs) {
    const e = document.createElementNS(ns, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    return e;
  }

  svg.innerHTML = "";
  svg.setAttribute("viewBox", `0 0 ${w} ${h}`);

  for (const xv of [xMin, (xMin + xMax) / 2, xMax]) {
    svg.appendChild(el("line", { class: "focus-chart-grid", x1: xPix(xv), x2: xPix(xv), y1: padT, y2: h - padB }));
  }
  for (const yv of [yMin, (yMin + yMax) / 2, yMax]) {
    svg.appendChild(el("line", { class: "focus-chart-grid", x1: padL, x2: w - padR, y1: yPix(yv), y2: yPix(yv) }));
  }
  svg.appendChild(el("line", { class: "focus-chart-axis", x1: padL, x2: w - padR, y1: h - padB, y2: h - padB }));
  svg.appendChild(el("line", { class: "focus-chart-axis", x1: padL, x2: padL, y1: padT, y2: h - padB }));

  // Usable glass circle, if a design session's metadata has one, for spatial reference.
  const meta = session && session.metadata;
  if (meta && meta.usable_center_xy_mm && meta.usable_diameter_mm) {
    const [cx, cy] = meta.usable_center_xy_mm;
    const rMm = meta.usable_diameter_mm / 2;
    const rPix = (rMm / (xMax - xMin)) * innerW;
    svg.appendChild(el("circle", { class: "focus-map-glass", cx: xPix(cx), cy: yPix(cy), r: rPix }));
  }

  const visited = new Set((survey ? survey.results : []).map((r) => `${r.x},${r.y}`));
  for (const [x, y] of focusSurveyWaypoints) {
    if (visited.has(`${x},${y}`)) continue; // will be drawn as a real result below
    svg.appendChild(el("circle", { class: "focus-map-dot pending", cx: xPix(x), cy: yPix(y), r: 4 }));
  }

  if (survey && survey.results.length) {
    const scores = survey.results.filter((r) => r.result && !r.error).map((r) => r.result.peakBrightness || 0);
    const sMin = Math.min(...scores, 0), sMax = Math.max(...scores, 1);
    for (const r of survey.results) {
      const px = xPix(r.x), py = yPix(r.y);
      let dot;
      if (r.error) {
        dot = el("circle", { class: "focus-map-dot error", cx: px, cy: py, r: 6 });
      } else {
        const score = r.result ? r.result.peakBrightness || 0 : 0;
        const t = sMax > sMin ? (score - sMin) / (sMax - sMin) : 0.5;
        dot = el("circle", {
          class: "focus-map-dot",
          cx: px, cy: py, r: 5 + t * 7,
          fill: "var(--primary)",
          "fill-opacity": (0.35 + t * 0.65).toFixed(2),
        });
      }
      dot.addEventListener("pointerenter", () => {
        if (r.error) {
          focusSurveyMapTip.textContent = `#${r.index + 1} X ${r.x.toFixed(3)} / Y ${r.y.toFixed(3)} -- failed: ${r.error}`;
        } else {
          const res = r.result;
          const offset = res.bestOffsetMm >= 0 ? `+${res.bestOffsetMm.toFixed(4)}` : res.bestOffsetMm.toFixed(4);
          focusSurveyMapTip.textContent =
            `#${r.index + 1} X ${r.x.toFixed(3)} / Y ${r.y.toFixed(3)} -- moved ${offset}mm, score ${(res.peakBrightness || 0).toFixed(0)}`;
        }
      });
      svg.appendChild(dot);
    }
  }
}

async function focusRefreshStatus() {
  try {
    const data = await fetchJson("/api/focus/status");
    focusApplyStatus(data);
  } catch (error) {
    // Transient poll failures are ignored, same as jog/camera status polling.
  }
}

async function focusScan(wide, narrowOnly = false) {
  try {
    const data = await fetchJson("/api/focus/scan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ wide: Boolean(wide), narrowOnly: Boolean(narrowOnly) }),
    });
    focusApplyStatus(data);
  } catch (error) {
    setStatus(error.message);
  }
}

async function focusSetActive(enabled) {
  focusSuppressToggle = true;
  try {
    const data = await fetchJson("/api/focus/active", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled }),
    });
    focusApplyStatus(data);
  } catch (error) {
    setStatus(error.message);
    focusActiveToggle.checked = !enabled;
  } finally {
    focusSuppressToggle = false;
  }
}

const SURVEY_WAYPOINT_SPACING_MM = 8.0; // ~25s/point at production narrow-scan settings -- keep this generous or a full spiral survey takes an hour+
const SURVEY_SECONDS_PER_POINT = 25; // rough -- matches production narrow scan duration

function findSpiralPathPoints() {
  if (!session) return null;
  const spiral = session.paths.find((p) => p.entityType === "MANUAL_SPIRAL");
  return spiral ? spiral.points : null;
}

function resamplePolylineBySpacing(points, spacing) {
  if (points.length === 0) return [];
  const waypoints = [points[0]];
  let accum = 0;
  for (let i = 1; i < points.length; i += 1) {
    const [x0, y0] = points[i - 1];
    const [x1, y1] = points[i];
    accum += Math.hypot(x1 - x0, y1 - y0);
    if (accum >= spacing) {
      waypoints.push(points[i]);
      accum = 0;
    }
  }
  const last = points[points.length - 1];
  const lastWp = waypoints[waypoints.length - 1];
  if (Math.hypot(last[0] - lastWp[0], last[1] - lastWp[1]) > 1e-6) waypoints.push(last);
  return waypoints;
}

async function focusStartSurvey() {
  let points = findSpiralPathPoints();
  if (!points) {
    // No spiral on the canvas -- use the default spiral (current Spiral-tab
    // field values) for survey waypoints WITHOUT merging it into the loaded
    // design session. Survey is a diagnostic path, not a shape to cut, and
    // persisting it into session.paths would draw it on top of (and get
    // exported alongside) whatever drawing the user already has loaded.
    try {
      points = buildSpiralShapePath().points;
    } catch (error) {
      setStatus(error.message);
      return;
    }
  }
  const waypoints = resamplePolylineBySpacing(points, SURVEY_WAYPOINT_SPACING_MM);
  const etaMin = Math.round((waypoints.length * SURVEY_SECONDS_PER_POINT) / 60);
  if (!window.confirm(`Survey ${waypoints.length} points along the spiral (~${etaMin} min at this scan speed). Start?`)) return;
  try {
    const data = await fetchJson("/api/focus/survey/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ waypoints, narrowOnly: focusSurveyNarrowToggle.checked }),
    });
    focusSurveyWaypoints = waypoints;
    focusApplyStatus(data);
    setStatus(`Survey started: ${waypoints.length} waypoints.`);
  } catch (error) {
    setStatus(error.message);
  }
}

async function focusStopSurvey() {
  try {
    const data = await fetchJson("/api/focus/survey/stop", { method: "POST" });
    focusApplyStatus(data);
    setStatus("Survey stop requested.");
  } catch (error) {
    setStatus(error.message);
  }
}

async function focusRunCustomScan() {
  const rangeMm = Number(focusCustomRange.value);
  const stepMm = Number(focusCustomStep.value);
  const settleS = Number(focusCustomSettle.value);
  if (!(rangeMm > 0) || !(stepMm > 0) || !(settleS >= 0)) {
    setStatus("Enter a positive range/step and a non-negative settle time.");
    return;
  }
  try {
    const data = await fetchJson("/api/focus/scan/custom", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        direction: focusCustomDirection.value,
        rangeMm,
        stepMm,
        settleS,
        centerOffsetMm: focusSelectionCenterOffsetMm,
      }),
    });
    focusApplyStatus(data);
  } catch (error) {
    setStatus(error.message);
  }
}

async function focusRunLatencyCheck() {
  try {
    const data = await fetchJson("/api/focus/latency-check", { method: "POST" });
    focusApplyStatus(data);
    setStatus("Timing check running (slow reference pass, then fast pass) - watch the Autofocus panel.");
  } catch (error) {
    setStatus(error.message);
  }
}

let jogConnected = false;
let jogStepMm = 0.1;
let jogBusy = false;
let jogHoldTimer = null;
let jogPollTimer = null;

function jogApplyStatus(data) {
  jogConnected = Boolean(data.connected);
  const pos = data.position || { x: 0, y: 0, z: 0 };
  const focusTrim = Number(data.focusTrimMm || 0);
  const trimText = Math.abs(focusTrim) > 0.0001 ? ` (focus trim ${focusTrim >= 0 ? "+" : ""}${focusTrim.toFixed(3)})` : "";
  jogPositionText.textContent = `X ${pos.x.toFixed(3)} / Y ${pos.y.toFixed(3)} / Z ${pos.z.toFixed(3)}${trimText}`;

  const limit = data.limit;
  if (limit) {
    const distToEdge = limit.radiusMm - Math.hypot(pos.x - limit.centerX, pos.y - limit.centerY);
    const zHeadroom = limit.zRangeMm - Math.abs(focusTrim);
    const nearXY = distToEdge < Math.max(0.5, limit.radiusMm * 0.05);
    const nearZ = zHeadroom < Math.max(0.2, limit.zRangeMm * 0.1);
    jogLimitText.textContent = `${distToEdge.toFixed(2)}mm to XY limit / ${zHeadroom.toFixed(2)}mm of Z headroom`;
    jogLimitText.classList.toggle("near-limit", nearXY || nearZ);
  } else {
    jogLimitText.textContent = "";
    jogLimitText.classList.remove("near-limit");
  }

  jogGotoX.placeholder = pos.x.toFixed(3);
  jogGotoY.placeholder = pos.y.toFixed(3);
  jogGotoZ.placeholder = pos.z.toFixed(3);
  if (!jogConnected) {
    jogStatusText.textContent = "Disconnected";
    jogStatusText.classList.remove("live");
  } else if (data.dryRun) {
    jogStatusText.textContent = `Connected (dry run, ${data.port})`;
    jogStatusText.classList.remove("live");
  } else {
    jogStatusText.textContent = `Connected: ${data.port} @ ${data.baudrate}`;
    jogStatusText.classList.add("live");
  }
  // Propagate the current hardware position to the preview/3D view,
  // independent of the source of motion (jog controls or an external
  // pendant).
  jogIsLive = jogConnected && !data.dryRun;
  jogLivePoint3 = jogIsLive ? [pos.x, pos.y, pos.z] : null;
  if (session) draw();
}

async function jogRefreshStatus() {
  try {
    const data = await fetchJson("/api/jog/status");
    jogApplyStatus(data);
  } catch (error) {
    // Transient poll failures are ignored; subsequent actions surface real errors.
  }
}

async function jogConnect() {
  try {
    const data = await fetchJson("/api/jog/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        port: jogPort.value.trim() || "COM5",
        baudrate: Number(jogBaud.value || 19200),
        dryRun: jogDryRun.checked,
        rangeXYMm: Number(jogRangeXY.value || 75),
        rangeZMm: Number(jogRangeZ.value || 20),
      }),
    });
    jogApplyStatus(data);
    setStatus(data.dryRun ? "Jog connected (dry run, no hardware move)." : `Jog connected on ${data.port}.`);
  } catch (error) {
    setStatus(error.message);
  }
}

async function jogDisconnect() {
  stopJogHold();
  try {
    const data = await fetchJson("/api/jog/disconnect", { method: "POST" });
    jogApplyStatus(data);
    setStatus("Jog disconnected.");
  } catch (error) {
    setStatus(error.message);
  }
}

let jogBlockedFlashTimer = null;

function jogShowBlocked(message) {
  // Blocked-move feedback lands right at the pad (where the operator's eyes
  // already are while jogging), not just the page-top status line -- a
  // move rejected near a limit was otherwise silent unless you happened to
  // be looking at the far-away status text.
  jogBlockedText.textContent = message;
  jogPad.classList.add("blocked");
  clearTimeout(jogBlockedFlashTimer);
  jogBlockedFlashTimer = setTimeout(() => jogPad.classList.remove("blocked"), 400);
}

async function jogSendMove(axis, dir) {
  if (jogBusy || !jogConnected) return;
  jogBusy = true;
  try {
    // Speed scales with step size to hold a roughly constant move duration.
    const speedMmS = Math.max(0.05, Math.min(10.0, jogStepMm / 0.08));
    const data = await fetchJson("/api/jog/move", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ axis, deltaMm: dir * jogStepMm, speedMmS }),
    });
    jogApplyStatus(data);
    jogBlockedText.textContent = "";
  } catch (error) {
    setStatus(error.message);
    jogShowBlocked(error.message);
    stopJogHold();
  } finally {
    jogBusy = false;
  }
}

const JOG_KEY_ELEMENTS = new Map(); // "axis:dir" -> button, filled once the pad buttons are wired up below
let jogPressedButton = null;

function jogSetPressed(axis, dir) {
  const button = JOG_KEY_ELEMENTS.get(`${axis}:${dir}`);
  if (jogPressedButton && jogPressedButton !== button) jogPressedButton.classList.remove("pressed");
  if (button) button.classList.add("pressed");
  jogPressedButton = button || null;
}

function jogClearPressed() {
  if (jogPressedButton) jogPressedButton.classList.remove("pressed");
  jogPressedButton = null;
}

function startJogHold(axis, dir) {
  if (!jogConnected) {
    setStatus("Connect jog control first.");
    return;
  }
  stopJogHold();
  jogSetPressed(axis, dir);
  jogSendMove(axis, dir);
  jogHoldTimer = setInterval(() => jogSendMove(axis, dir), 120);
}

function stopJogHold() {
  if (jogHoldTimer) {
    clearInterval(jogHoldTimer);
    jogHoldTimer = null;
  }
  jogClearPressed();
}

async function jogHome() {
  if (!jogConnected) {
    setStatus("Connect jog control first.");
    return;
  }
  if (!window.confirm("Move stage to machine zero (0, 0, 0)? This can be a large move.")) return;
  stopJogHold();
  setStatus("Homing...");
  try {
    const data = await fetchJson("/api/jog/home", { method: "POST" });
    jogApplyStatus(data);
    setStatus("Homed to (0, 0, 0).");
  } catch (error) {
    setStatus(error.message);
  }
}

async function jogSetLocalHome() {
  if (!jogConnected) {
    setStatus("Connect jog control first.");
    return;
  }
  const center = session?.metadata?.usable_center_xy_mm || [12.5, 12.5];
  const focusZ = session?.metadata?.focus_z_mm ?? 0;
  const pos = jogPositionText.textContent;
  if (
    !window.confirm(
      `Label current position as the design center (X ${center[0]}, Y ${center[1]}, Z ${focusZ})?\n` +
        `Current: ${pos}\nNo motion, no change to the real machine home — this only sets a software offset so displayed/limit/Run-on-machine coordinates line up with the drawing.`,
    )
  )
    return;
  stopJogHold();
  try {
    const data = await fetchJson("/api/jog/set-local-home", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: center[0], y: center[1], z: focusZ }),
    });
    jogApplyStatus(data);
    setStatus("Local home set (software offset only, real machine home unchanged).");
  } catch (error) {
    setStatus(error.message);
  }
}

async function jogGoLocalHome() {
  if (!jogConnected) {
    setStatus("Connect jog control first.");
    return;
  }
  if (!window.confirm("Move stage to the labeled local home position?")) return;
  stopJogHold();
  setStatus("Moving to local home...");
  try {
    const data = await fetchJson("/api/jog/go-local-home", { method: "POST" });
    jogApplyStatus(data);
    setStatus("At local home.");
  } catch (error) {
    setStatus(error.message);
  }
}

async function jogGoto() {
  if (!jogConnected) {
    setStatus("Connect jog control first.");
    return;
  }
  const x = jogGotoX.value === "" ? Number(jogGotoX.placeholder) : Number(jogGotoX.value);
  const y = jogGotoY.value === "" ? Number(jogGotoY.placeholder) : Number(jogGotoY.value);
  const z = jogGotoZ.value === "" ? Number(jogGotoZ.placeholder) : Number(jogGotoZ.value);
  if ([x, y, z].some((value) => Number.isNaN(value))) {
    setStatus("Enter valid X/Y/Z numbers.");
    return;
  }
  if (!window.confirm(`Move stage to X ${x} / Y ${y} / Z ${z} mm?`)) return;
  stopJogHold();
  try {
    const data = await fetchJson("/api/jog/goto", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x, y, z }),
    });
    jogApplyStatus(data);
    setStatus(`Moving to X ${x} / Y ${y} / Z ${z} mm.`);
  } catch (error) {
    setStatus(error.message);
  }
}

let jogRunPollTimer = null;
let jogRunActive = false;

function jogRunApplyState(data) {
  jogRunActive = Boolean(data.running);
  if (data.running) {
    jogRunStatusText.textContent = `Running segment ${data.index}/${data.total}`;
  } else if (data.error) {
    jogRunStatusText.textContent = `Stopped: ${data.error}`;
  } else if (data.done) {
    jogRunStatusText.textContent = "Run complete.";
  } else {
    jogRunStatusText.textContent = "Idle";
  }
  // Advance the playback playhead using segments confirmed complete on the
  // physical stage, rather than elapsed time.
  if (motion && motion.segments && data.index > 0) {
    let elapsed = 0;
    for (let i = 0; i < Math.min(data.index, motion.segments.length); i += 1) {
      elapsed += motion.segments[i].duration;
    }
    playhead = Math.min(elapsed, motion.stats.duration || elapsed);
    playing = false;
    playButton.textContent = ">";
    draw();
  }
  if (!data.running) {
    if (jogRunPollTimer) {
      clearInterval(jogRunPollTimer);
      jogRunPollTimer = null;
    }
  }
}

async function jogRunPlan() {
  if (!jogConnected) {
    setStatus("Connect jog control first.");
    return;
  }
  if (!motion || !motion.segments || motion.segments.length === 0) {
    setStatus("No planned path to run.");
    return;
  }
  if (jogRunActive) {
    setStatus("A run is already in progress.");
    return;
  }
  const count = motion.segments.length;
  const seconds = (motion.stats.duration || 0).toFixed(1);
  if (!window.confirm(`Run the full planned path on the machine?\n${count} segments, ~${seconds}s.`)) return;
  stopJogHold();
  try {
    const data = await fetchJson("/api/run/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ segments: motion.segments }),
    });
    jogRunApplyState(data);
    clearInterval(jogRunPollTimer);
    jogRunPollTimer = setInterval(async () => {
      try {
        const status = await fetchJson("/api/run/status");
        jogRunApplyState(status);
      } catch (error) {
        setStatus(error.message);
        clearInterval(jogRunPollTimer);
        jogRunPollTimer = null;
      }
    }, 200);
  } catch (error) {
    setStatus(error.message);
  }
}

async function jogRunStopFn() {
  try {
    const data = await fetchJson("/api/run/stop", { method: "POST" });
    jogRunApplyState(data);
  } catch (error) {
    setStatus(error.message);
  }
}

async function jogEstop() {
  stopJogHold();
  if (jogRunPollTimer) {
    clearInterval(jogRunPollTimer);
    jogRunPollTimer = null;
  }
  jogRunActive = false;
  try {
    const data = await fetchJson("/api/jog/estop", { method: "POST" });
    jogApplyStatus(data);
    setStatus("E-STOP sent. Reconnect before jogging again.");
  } catch (error) {
    setStatus(error.message);
  }
}

async function loadDefault() {
  setStatus("Tracing sample drawing...");
  const data = await fetchJson("/api/default");
  applySession(data);
}

async function traceCurrent(mode = "path") {
  setStatus("Tracing drawing...");
  lastTraceMode = mode;
  let data;
  if (mode === "file") {
    if (!fileInput.files.length) {
      setStatus("Choose a DWG/DXF file first.");
      return;
    }
    const form = new FormData();
    form.append("file", fileInput.files[0]);
    for (const [key, value] of Object.entries(traceParams())) form.append(key, value);
    data = await fetchJson("/api/trace", { method: "POST", body: form });
  } else {
    if (!sourcePath.value.trim()) {
      setStatus("Enter a DWG/DXF path first.");
      return;
    }
    data = await fetchJson("/api/trace", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(traceParams()),
    });
  }
  applySession(data);
}

async function applyScale() {
  if (!session) {
    setStatus("Open a file first.");
    return;
  }
  const factor = Number(scaleFactor.value);
  if (!Number.isFinite(factor) || factor <= 0) {
    setStatus("Scale must be a positive number.");
    return;
  }
  const currentFit = Number(fitLongest.value) || 25;
  fitLongest.value = (currentFit * factor).toFixed(3);
  scaleFactor.value = "1";
  await traceCurrent(lastTraceMode);
}

function applySession(data) {
  session = data;
  motion = null;
  selected = new Set(data.paths.filter((path) => path.selected !== false).map((path) => path.id));
  passCounts = new Map(data.paths.map((path) => [path.id, path.passes || 1]));
  playhead = 0;
  playing = false;
  playButton.textContent = ">";
  stageNeedsRebuild = true;
  renderPathList();
  updateMetrics();
  resizeCanvas();
  updatePlan();
  const circle = data.metadata?.circle_check;
  if (circle?.target_diameter_mm) {
    setStatus(`${data.sourceName} traced - circle ${circle.detected_diameter_mm.toFixed(3)} mm`);
  } else if (data.paths.length === 0) {
    setStatus(`${data.sourceName} ready`);
  } else {
    setStatus(`${data.sourceName} traced`);
  }
}

function schedulePlan() {
  if (!session) return;
  clearTimeout(planTimer);
  planTimer = setTimeout(updatePlan, 180);
}

async function updatePlan() {
  if (!session) return;
  if (selected.size === 0) {
    motion = null;
    stageNeedsRebuild = true;
    cumulative = [];
    playhead = 0;
    updateMetrics();
    draw();
    setStatus("No selected paths.");
    return;
  }
  try {
    const plan = await fetchJson("/api/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(requestPayload()),
    });
    motion = plan;
    stageNeedsRebuild = true;
    cumulative = [];
    let cursor = 0;
    for (const segment of motion.segments) {
      cumulative.push(cursor);
      cursor += segment.duration;
    }
    playhead = Math.min(playhead, motion.stats.duration || 0);
    updateMetrics();
    draw();
  } catch (error) {
    setStatus(error.message);
  }
}

async function exportCode() {
  if (!session || selected.size === 0) {
    setStatus("Select paths before export.");
    return;
  }
  setStatus("Generating code...");
  const payload = await fetchJson("/api/export", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestPayload()),
  });
  if (payload.downloadUrl) {
    const link = document.createElement("a");
    link.href = payload.downloadUrl;
    link.download = payload.fileName || "";
    document.body.appendChild(link);
    link.click();
    link.remove();
  }
  setStatus(`Download ready: ${payload.fileName || payload.scriptPath}`);
}

function updateMetrics() {
  if (!session) {
    bboxMetric.textContent = "-";
    pathMetric.textContent = "-";
    cutMetric.textContent = "-";
    travelMetric.textContent = "-";
    return;
  }
  const box = motion?.bbox || session.bbox;
  bboxMetric.textContent = `${box.width.toFixed(3)} x ${box.height.toFixed(3)}`;
  pathMetric.textContent = `${selected.size} / ${session.paths.length}`;
  cutMetric.textContent = motion ? mm(motion.stats.cutLength, 1) : "-";
  travelMetric.textContent = motion ? mm(motion.stats.travelLength, 1) : "-";
}

function renderPathList() {
  if (!session) {
    pathList.innerHTML = "";
    return;
  }
  pathList.innerHTML = session.paths
    .map((path) => {
      const checked = selected.has(path.id) ? "checked" : "";
      const passes = passCounts.get(path.id) || 1;
      return `
        <div class="path-row ${checked ? "selected" : ""}" data-path="${path.id}">
          <input type="checkbox" ${checked} />
          <button type="button" class="path-focus">#${path.id + 1}</button>
          <span class="path-name">${path.layer}</span>
          <span class="path-length">${mm(path.length, 2)}</span>
          <input class="pass-input" type="number" min="1" step="1" value="${passes}" />
        </div>
      `;
    })
    .join("");
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  resizeStage3d();
  draw();
}

function viewBox() {
  if (!session) return { minX: 0, minY: 0, width: 1, height: 1 };
  const box = session.bbox;
  return {
    minX: Math.min(0, box.minX),
    minY: Math.min(0, box.minY),
    width: Math.max(box.maxX, 0) - Math.min(box.minX, 0),
    height: Math.max(box.maxY, 0) - Math.min(box.minY, 0),
  };
}

function project(point) {
  const box = viewBox();
  const rect = canvas.getBoundingClientRect();
  const pad = 42;
  const scale = Math.min((rect.width - pad * 2) / Math.max(box.width, 0.001), (rect.height - pad * 2) / Math.max(box.height, 0.001));
  const ox = (rect.width - box.width * scale) / 2;
  const oy = (rect.height - box.height * scale) / 2;
  return {
    x: ox + (point[0] - box.minX) * scale,
    y: rect.height - (oy + (point[1] - box.minY) * scale),
  };
}

function drawPolyline(points, color, width, dash = []) {
  if (points.length < 2) return;
  const first = project(points[0]);
  ctx.beginPath();
  ctx.moveTo(first.x, first.y);
  for (const point of points.slice(1)) {
    const p = project(point);
    ctx.lineTo(p.x, p.y);
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.stroke();
  ctx.setLineDash([]);
}

function segmentAt(time) {
  if (!motion || motion.segments.length === 0) return null;
  for (let i = 0; i < cumulative.length; i += 1) {
    const start = cumulative[i];
    const end = start + motion.segments[i].duration;
    if (time <= end) return { index: i, t: (time - start) / Math.max(motion.segments[i].duration, 0.0001) };
  }
  return { index: motion.segments.length - 1, t: 1 };
}

function pointOnSegment(segment, t) {
  return [
    segment.from[0] + (segment.to[0] - segment.from[0]) * t,
    segment.from[1] + (segment.to[1] - segment.from[1]) * t,
  ];
}

function drawSegment(segment, progress, color, width, dash = []) {
  const end = pointOnSegment(segment, progress);
  drawPolyline([segment.from, end], color, width, dash);
}

function drawUsableArea() {
  const usable = usableCircle();
  if (!usable) return;
  const center = project(usable.center);
  const edge = project([usable.center[0] + usable.radius, usable.center[1]]);
  const radius = Math.abs(edge.x - center.x);
  ctx.save();
  ctx.beginPath();
  ctx.arc(center.x, center.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(29, 127, 209, 0.045)";
  ctx.fill();
  ctx.strokeStyle = "rgba(29, 127, 209, 0.72)";
  ctx.lineWidth = 1.4;
  ctx.setLineDash([7, 5]);
  ctx.stroke();
  ctx.restore();
}

function clearThreeGroup(group) {
  if (!group) return;
  while (group.children.length) {
    const child = group.children[0];
    group.remove(child);
    disposeThreeObject(child);
  }
}

function stagePoint(point3) {
  return new THREE.Vector3(Number(point3[0] || 0), Number(point3[2] || 0) * Z_VISUAL_SCALE, -Number(point3[1] || 0));
}

function sampleLocalPoint(point, y = 0.04) {
  return new THREE.Vector3(Number(point[0] || 0), y, -Number(point[1] || 0));
}

function samplePositionForStage(point3) {
  return new THREE.Vector3(-Number(point3[0] || 0), Number(point3[2] || 0) * Z_VISUAL_SCALE, Number(point3[1] || 0));
}

function disposeThreeObject(object) {
  object.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach((material) => material.dispose());
      else child.material.dispose();
    }
  });
}

function segmentPoint3(segment, t) {
  const from = segment.from3 || [segment.from[0], segment.from[1], 0];
  const to = segment.to3 || [segment.to[0], segment.to[1], 0];
  return [
    from[0] + (to[0] - from[0]) * t,
    from[1] + (to[1] - from[1]) * t,
    from[2] + (to[2] - from[2]) * t,
  ];
}

function isLaserOnSegment(segment) {
  return segment?.laserOn === true || segment?.kind === "cut";
}

function segmentColor(segment) {
  if (isLaserOnSegment(segment)) return STAGE_COLORS.cut;
  if (segment?.zOnly || segment?.kind?.startsWith("z-")) return STAGE_COLORS.z;
  return STAGE_COLORS.travel;
}

function makeLine3(points, color, opacity = 1) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map(stagePoint));
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
  });
  return new THREE.Line(geometry, material);
}

function makeWorldLine(points, color, opacity = 1) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map((point) => new THREE.Vector3(point[0], point[1], point[2])));
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
  });
  return new THREE.Line(geometry, material);
}

function updateWorldLine(line, points) {
  if (!line) return;
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points.map((point) => new THREE.Vector3(point[0], point[1], point[2])));
}

function makeSampleLine(points, color, opacity = 1, y = 0.04) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map((point) => sampleLocalPoint(point, y)));
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
  });
  return new THREE.Line(geometry, material);
}

function makeSamplePlane(centerX, centerY, width, height, color, opacity = 1, y = GLASS_SURFACE_Y + 0.012) {
  const geometry = new THREE.PlaneGeometry(width, height);
  const material = new THREE.MeshStandardMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    side: THREE.DoubleSide,
    roughness: 0.55,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.rotation.x = -Math.PI / 2;
  mesh.position.set(centerX, y, -centerY);
  return mesh;
}

function makeSampleCircle(center, radius, color, opacity = 1, y = 0.09) {
  const points = [];
  const segments = 144;
  for (let i = 0; i <= segments; i += 1) {
    const angle = (Math.PI * 2 * i) / segments;
    points.push([center[0] + Math.cos(angle) * radius, center[1] + Math.sin(angle) * radius]);
  }
  return makeSampleLine(points, color, opacity, y);
}

function activeMetadata() {
  return motion?.metadata || session?.metadata || {};
}

function glassSize() {
  const raw = activeMetadata().glass_size_mm;
  if (Array.isArray(raw) && raw.length >= 2) return [Math.max(Number(raw[0]) || 25, 0.001), Math.max(Number(raw[1]) || 25, 0.001)];
  const size = Math.max(Number(raw) || 25, 0.001);
  return [size, size];
}

function startPoint3() {
  const [width, height] = glassSize();
  const raw = activeMetadata().start_xy_mm;
  const x = Array.isArray(raw) && raw.length >= 2 ? Number(raw[0]) : width / 2;
  const y = Array.isArray(raw) && raw.length >= 2 ? Number(raw[1]) : height / 2;
  const z = Number(activeMetadata().focus_z_mm || 0);
  return [Number.isFinite(x) ? x : width / 2, Number.isFinite(y) ? y : height / 2, Number.isFinite(z) ? z : 0];
}

function usableCircle() {
  const metadata = activeMetadata();
  if (metadata.usable_area_shape !== "circle") return null;
  const [width, height] = glassSize();
  const centerRaw = metadata.usable_center_xy_mm;
  const diameter = Math.max(Number(metadata.usable_diameter_mm) || Math.min(width, height), 0.001);
  const center = Array.isArray(centerRaw) && centerRaw.length >= 2
    ? [Number(centerRaw[0]), Number(centerRaw[1])]
    : [width / 2, height / 2];
  return {
    center: [
      Number.isFinite(center[0]) ? center[0] : width / 2,
      Number.isFinite(center[1]) ? center[1] : height / 2,
    ],
    diameter,
    radius: diameter / 2,
  };
}

function stageBounds() {
  const [glassWidth, glassHeight] = glassSize();
  const box = motion?.bbox || session?.bbox || { minX: 0, minY: 0, maxX: glassWidth, maxY: glassHeight, width: glassWidth, height: glassHeight };
  const minX = Math.min(0, box.minX);
  const minY = Math.min(0, box.minY);
  const maxX = Math.max(glassWidth, box.maxX);
  const maxY = Math.max(glassHeight, box.maxY);
  return {
    minX,
    minY,
    maxX,
    maxY,
    width: Math.max(maxX - minX, 1),
    height: Math.max(maxY - minY, 1),
    centerX: (minX + maxX) / 2,
    centerY: (minY + maxY) / 2,
  };
}

function updateStageCamera() {
  if (!stageCamera) return;
  const box = stageBounds();
  const target = new THREE.Vector3(0, -1, 0);
  const radius = stageDistance;
  const horizontal = Math.cos(stagePitch) * radius;
  stageCamera.position.set(
    target.x + Math.sin(stageYaw) * horizontal,
    target.y + Math.sin(stagePitch) * radius,
    target.z + Math.cos(stageYaw) * horizontal,
  );
  stageCamera.lookAt(target);
}

function resizeStage3d() {
  if (!stageRenderer || !stageCamera || !stageRoot) return;
  const rect = stageRoot.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width));
  const height = Math.max(1, Math.round(rect.height));
  stageRenderer.setSize(width, height, false);
  stageCamera.aspect = width / height;
  stageCamera.updateProjectionMatrix();
  updateStageCamera();
  stageRenderer.render(stageScene, stageCamera);
}

function initStage3d() {
  if (!stageRoot || stageRenderer) return;
  stageScene = new THREE.Scene();
  stageScene.background = new THREE.Color(0x272729);

  stageCamera = new THREE.PerspectiveCamera(42, 1, 0.1, 2000);
  stageRenderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  stageRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  stageRoot.appendChild(stageRenderer.domElement);

  const ambient = new THREE.AmbientLight(0xffffff, 0.92);
  const key = new THREE.DirectionalLight(0xffffff, 0.84);
  key.position.set(20, 35, 18);
  stageScene.add(ambient, key);

  stageGroup = new THREE.Group();
  sampleGroup = new THREE.Group();
  staticRouteGroup = new THREE.Group();
  progressRouteGroup = new THREE.Group();
  toolGroup = new THREE.Group();
  motionCueGroup = new THREE.Group();
  sampleGroup.add(staticRouteGroup, progressRouteGroup);
  stageScene.add(stageGroup, sampleGroup, motionCueGroup, toolGroup);

  const headRadius = 0.52;
  const head = new THREE.Mesh(
    new THREE.SphereGeometry(headRadius, 24, 16),
    new THREE.MeshStandardMaterial({ color: STAGE_COLORS.head, metalness: 0.2, roughness: 0.45 }),
  );
  head.position.set(0, 1.48, 0);
  const nozzle = new THREE.Mesh(
    new THREE.ConeGeometry(0.34, 0.9, 24),
    new THREE.MeshStandardMaterial({ color: 0x25313a, roughness: 0.35 }),
  );
  nozzle.position.set(0, 0.78, 0);
  nozzle.rotation.x = Math.PI;
  const alignmentGuide = makeWorldLine([[0, 0.7, 0], [0, FOCUS_PLANE_Y, 0]], 0xd8dde2, 0.18);
  beamLine = makeWorldLine([[0, 0.7, 0], [0, FOCUS_PLANE_Y + 0.02, 0]], STAGE_COLORS.cut, 0.95);
  beamLine.visible = false;

  focusRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.32, 0.01, 8, 48),
    new THREE.MeshBasicMaterial({ color: 0xd8dde2, transparent: true, opacity: 0.58 }),
  );
  focusRing.rotation.x = Math.PI / 2;
  focusRing.position.set(0, FOCUS_PLANE_Y, 0);

  zGapLine = makeWorldLine([[0, FOCUS_PLANE_Y, 0], [0, FOCUS_PLANE_Y, 0]], STAGE_COLORS.z, 0.92);
  zGapLine.visible = false;

  contactRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.5, 0.016, 8, 56),
    new THREE.MeshBasicMaterial({ color: STAGE_COLORS.z, transparent: true, opacity: 0.72 }),
  );
  contactRing.rotation.x = Math.PI / 2;
  contactRing.position.set(0, FOCUS_PLANE_Y, 0);

  laserSpot = new THREE.Mesh(
    new THREE.SphereGeometry(0.13, 20, 14),
    new THREE.MeshBasicMaterial({ color: STAGE_COLORS.cut, transparent: true, opacity: 0.95 }),
  );
  laserSpot.position.set(0, FOCUS_PLANE_Y + 0.04, 0);
  laserSpot.visible = false;

  toolGroup.add(head, nozzle, alignmentGuide, beamLine, focusRing);
  motionCueGroup.add(zGapLine, contactRing, laserSpot);

  stageRoot.addEventListener("pointerdown", (event) => {
    stageDrag = { x: event.clientX, y: event.clientY, yaw: stageYaw, pitch: stagePitch };
    stageRoot.setPointerCapture(event.pointerId);
  });
  stageRoot.addEventListener("pointermove", (event) => {
    if (!stageDrag) return;
    stageYaw = stageDrag.yaw - (event.clientX - stageDrag.x) * 0.008;
    stagePitch = Math.max(0.18, Math.min(1.2, stageDrag.pitch + (event.clientY - stageDrag.y) * 0.006));
    updateStageCamera();
    stageRenderer.render(stageScene, stageCamera);
  });
  stageRoot.addEventListener("pointerup", () => {
    stageDrag = null;
  });
  stageRoot.addEventListener("wheel", (event) => {
    event.preventDefault();
    stageDistance = Math.max(12, stageDistance * (event.deltaY > 0 ? 1.08 : 0.92));
    updateStageCamera();
    stageRenderer.render(stageScene, stageCamera);
  });

  resizeStage3d();
}

function rebuildStage3d() {
  if (!stageScene || !stageGroup || !sampleGroup || !staticRouteGroup || !progressRouteGroup) return;
  clearThreeGroup(stageGroup);
  clearThreeGroup(staticRouteGroup);
  clearThreeGroup(progressRouteGroup);
  sampleGroup.position.copy(samplePositionForStage(startPoint3()));

  if (!session) {
    sampleGroup.visible = false;
    stageNeedsRebuild = false;
    return;
  }

  sampleGroup.visible = true;
  const box = stageBounds();
  const [glassWidth, glassHeight] = glassSize();
  const glassCenterX = glassWidth / 2;
  const glassCenterY = glassHeight / 2;
  const span = Math.max(box.width, box.height, 8);
  stageDistance = Math.max(stageDistance, span * 2.8);
  const grid = new THREE.GridHelper(span * 2.35, Math.max(8, Math.min(36, Math.round(span / 2))), 0x5b6166, 0x3b3f42);
  grid.position.set(0, GLASS_CENTER_Y - 0.6, 0);
  stageGroup.add(grid);

  const railMaterialOptions = { color: 0x151719, roughness: 0.48, metalness: 0.18 };
  const railSpan = span * 1.55;
  const xRailA = new THREE.Mesh(new THREE.BoxGeometry(railSpan, 0.12, 0.22), new THREE.MeshStandardMaterial(railMaterialOptions));
  const xRailB = new THREE.Mesh(new THREE.BoxGeometry(railSpan, 0.12, 0.22), new THREE.MeshStandardMaterial(railMaterialOptions));
  xRailA.position.set(0, GLASS_CENTER_Y - 0.5, -span * 0.42);
  xRailB.position.set(0, GLASS_CENTER_Y - 0.5, span * 0.42);
  const yRailA = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.12, railSpan), new THREE.MeshStandardMaterial(railMaterialOptions));
  const yRailB = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.12, railSpan), new THREE.MeshStandardMaterial(railMaterialOptions));
  yRailA.position.set(-span * 0.42, GLASS_CENTER_Y - 0.48, 0);
  yRailB.position.set(span * 0.42, GLASS_CENTER_Y - 0.48, 0);
  stageGroup.add(xRailA, xRailB, yRailA, yRailB);

  const carrier = new THREE.Mesh(
    new THREE.BoxGeometry(glassWidth + 4.2, 0.26, glassHeight + 4.2),
    new THREE.MeshStandardMaterial({
      color: STAGE_COLORS.carrier,
      transparent: true,
      opacity: 0.86,
      roughness: 0.62,
      metalness: 0.12,
    }),
  );
  carrier.position.set(glassCenterX, GLASS_CENTER_Y - 0.31, -glassCenterY);
  staticRouteGroup.add(carrier);

  const glassGeometry = new THREE.BoxGeometry(glassWidth, GLASS_THICKNESS, glassHeight);
  const glass = new THREE.Mesh(
    glassGeometry,
    new THREE.MeshStandardMaterial({
      color: STAGE_COLORS.glass,
      transparent: true,
      opacity: 0.38,
      side: THREE.DoubleSide,
      roughness: 0.08,
      metalness: 0.02,
    }),
  );
  glass.position.set(glassCenterX, GLASS_CENTER_Y, -glassCenterY);
  staticRouteGroup.add(glass);

  const glassEdges = new THREE.LineSegments(
    new THREE.EdgesGeometry(glassGeometry),
    new THREE.LineBasicMaterial({ color: 0xd8fbff, transparent: true, opacity: 0.7 }),
  );
  glassEdges.position.copy(glass.position);
  staticRouteGroup.add(glassEdges);

  const border = [
    [0, 0],
    [glassWidth, 0],
    [glassWidth, glassHeight],
    [0, glassHeight],
    [0, 0],
  ];
  staticRouteGroup.add(makeSampleLine(border, 0xd8fbff, 0.92, GLASS_SURFACE_Y + 0.018));

  for (let x = 5; x < glassWidth; x += 5) {
    staticRouteGroup.add(makeSampleLine([[x, 0], [x, glassHeight]], 0xffffff, 0.18, GLASS_SURFACE_Y + 0.012));
  }
  for (let y = 5; y < glassHeight; y += 5) {
    staticRouteGroup.add(makeSampleLine([[0, y], [glassWidth, y]], 0xffffff, 0.18, GLASS_SURFACE_Y + 0.012));
  }

  const tapeSize = Math.min(3.4, glassWidth * 0.16, glassHeight * 0.16);
  const tapeInset = tapeSize / 2 + 0.45;
  staticRouteGroup.add(makeSamplePlane(tapeInset, tapeInset, tapeSize, tapeSize, STAGE_COLORS.tape, 0.78, GLASS_SURFACE_Y + 0.025));
  staticRouteGroup.add(makeSamplePlane(glassWidth - tapeInset, tapeInset, tapeSize, tapeSize, STAGE_COLORS.tape, 0.78, GLASS_SURFACE_Y + 0.025));
  staticRouteGroup.add(makeSamplePlane(tapeInset, glassHeight - tapeInset, tapeSize, tapeSize, STAGE_COLORS.tape, 0.78, GLASS_SURFACE_Y + 0.025));
  staticRouteGroup.add(makeSamplePlane(glassWidth - tapeInset, glassHeight - tapeInset, tapeSize, tapeSize, STAGE_COLORS.tape, 0.78, GLASS_SURFACE_Y + 0.025));

  staticRouteGroup.add(makeSampleLine([[glassCenterX - 1.25, glassCenterY], [glassCenterX + 1.25, glassCenterY]], 0x2997ff, 0.88, GLASS_SURFACE_Y + 0.034));
  staticRouteGroup.add(makeSampleLine([[glassCenterX, glassCenterY - 1.25], [glassCenterX, glassCenterY + 1.25]], 0x2997ff, 0.88, GLASS_SURFACE_Y + 0.034));

  const usable = usableCircle();
  if (usable) {
    staticRouteGroup.add(makeSampleCircle(usable.center, usable.radius, 0x2997ff, 0.95, GLASS_SURFACE_Y + 0.04));
  }

  for (const path of session.paths) {
    if (path.points.length < 2) continue;
    const color = selected.has(path.id) ? 0x1d1d1f : 0x7f8f98;
    const opacity = selected.has(path.id) ? 0.72 : 0.26;
    staticRouteGroup.add(makeSampleLine(path.points, color, opacity, GLASS_SURFACE_Y + 0.03));
  }

  stageNeedsRebuild = false;
  updateStageCamera();
}

function updateStageMotionCues(stagePosition, laserOn) {
  if (!zGapLine || !contactRing || !laserSpot || !beamLine) return;
  const sampleWorldPosition = samplePositionForStage(stagePosition);
  const sampleSurfaceY = sampleWorldPosition.y + GLASS_SURFACE_Y;
  const gap = sampleSurfaceY - FOCUS_PLANE_Y;
  updateWorldLine(zGapLine, [[0, FOCUS_PLANE_Y, 0], [0, sampleSurfaceY, 0]]);
  zGapLine.visible = Math.abs(gap) > 0.025;

  contactRing.position.set(0, sampleSurfaceY + 0.014, 0);
  contactRing.material.color.setHex(laserOn ? STAGE_COLORS.cut : STAGE_COLORS.z);
  contactRing.material.opacity = laserOn ? 0.86 : 0.66;
  contactRing.scale.setScalar(laserOn ? 0.78 : 1.0);

  laserSpot.position.set(0, sampleSurfaceY + 0.04, 0);
  laserSpot.visible = laserOn;
  beamLine.visible = laserOn;
  if (focusRing) focusRing.visible = true;
}

function updateStage3d(active, livePoint3) {
  if (!stageRenderer || !stageScene || !stageCamera || !sampleGroup || !toolGroup) return;
  if (stageNeedsRebuild) rebuildStage3d();
  clearThreeGroup(progressRouteGroup);

  if (!session) {
    sampleGroup.visible = false;
    toolGroup.visible = true;
    toolGroup.position.set(0, 0, 0);
    const start = startPoint3();
    updateStageMotionCues(start, false);
    laserState.textContent = "Laser OFF";
    zLabel.textContent = `Glass Z ${start[2].toFixed(3)} mm`;
    segmentLabel.textContent = "Idle";
    stageRenderer.render(stageScene, stageCamera);
    return;
  }

  // The glass position always prefers real polled hardware telemetry
  // (livePoint3) over the simulated playhead, so the 3D view tracks the
  // actual machine in real time during a live "Run on machine" too, not
  // just idle jogging. The progress route overlay and laser/segment
  // labels still come from the planned segment when one is active.
  const runningSegment = motion && active ? motion.segments[active.index] : null;
  const laserOn = runningSegment ? isLaserOnSegment(runningSegment) : false;
  const stagePosition = livePoint3 || (runningSegment ? segmentPoint3(runningSegment, active.t) : startPoint3());

  sampleGroup.visible = true;
  sampleGroup.position.copy(samplePositionForStage(stagePosition));
  toolGroup.visible = true;
  toolGroup.position.set(0, 0, 0);
  updateStageMotionCues(stagePosition, laserOn);
  zLabel.textContent = `Glass Z ${stagePosition[2].toFixed(3)} mm`;

  if (runningSegment) {
    laserState.textContent = laserOn ? "Laser ON" : "Laser OFF";
    segmentLabel.textContent = runningSegment.kind;
    for (let i = 0; i <= active.index; i += 1) {
      const cutSegment = motion.segments[i];
      if (!isLaserOnSegment(cutSegment)) continue;
      const progress = i < active.index ? 1 : active.t;
      if (progress <= 0) continue;
      progressRouteGroup.add(makeSampleLine([cutSegment.from, pointOnSegment(cutSegment, progress)], STAGE_COLORS.cut, 1, GLASS_SURFACE_Y + 0.06));
    }
  } else if (livePoint3) {
    laserState.textContent = "Live jog (laser manual)";
    segmentLabel.textContent = "Live jog";
  } else {
    laserState.textContent = "Laser OFF";
    segmentLabel.textContent = "Idle";
  }

  stageRenderer.render(stageScene, stageCamera);
}

function draw() {
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  if (!session) {
    updateStage3d(null);
    return;
  }

  drawUsableArea();

  for (const path of session.paths) {
    const isSelected = selected.has(path.id);
    const isHover = hoverPath === path.id;
    drawPolyline(
      path.points,
      isHover ? "#1d4e89" : isSelected ? "#c93a2d" : "#b8c2c8",
      isHover ? 3 : isSelected ? 2 : 1,
      isSelected ? [] : [4, 5],
    );
  }

  const active = segmentAt(playhead);

  // Progress overlay (segments completed so far) always follows the
  // planned timeline, independent of the position dot below, so the
  // completed/in-progress path coloring keeps working during a live run.
  if (motion && active) {
    for (let i = 0; i < motion.segments.length; i += 1) {
      const segment = motion.segments[i];
      const complete = i < active.index ? 1 : i === active.index ? active.t : 0;
      if (complete <= 0) continue;
      if (!isLaserOnSegment(segment)) {
        drawSegment(segment, complete, "#6f7c84", 1.2, [6, 6]);
      } else {
        drawSegment(segment, complete, "#da3b2f", 2.6);
      }
    }
  }

  // Position dot + coordinate readout: whenever live hardware is
  // connected (idle jog or an active "Run on machine" alike), track the
  // actual polled machine position instead of the simulated playhead, so
  // both the 2D preview and the 3D stage view stay matched to the real
  // machine in real time rather than an estimated playback.
  const liveOverride = jogIsLive && jogLivePoint3;
  if (liveOverride) {
    const head = project([jogLivePoint3[0], jogLivePoint3[1]]);
    ctx.beginPath();
    ctx.arc(head.x, head.y, 6, 0, Math.PI * 2);
    ctx.fillStyle = "#0066cc";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#fff";
    ctx.stroke();
    coordLabel.textContent = `Live X ${jogLivePoint3[0].toFixed(3)} / Y ${jogLivePoint3[1].toFixed(3)} / Z ${jogLivePoint3[2].toFixed(3)}`;
  } else if (motion && active) {
    const current = motion.segments[active.index];
    const headPoint = pointOnSegment(current, active.t);
    const headPoint3 = segmentPoint3(current, active.t);
    const head = project(headPoint);
    ctx.beginPath();
    ctx.arc(head.x, head.y, 6, 0, Math.PI * 2);
    ctx.fillStyle = "#07847e";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#fff";
    ctx.stroke();
    coordLabel.textContent = `Glass X ${headPoint3[0].toFixed(3)} / Y ${headPoint3[1].toFixed(3)} / Z ${headPoint3[2].toFixed(3)}`;
  } else {
    const start = startPoint3();
    coordLabel.textContent = `Glass X ${start[0].toFixed(3)} / Y ${start[1].toFixed(3)} / Z ${start[2].toFixed(3)}`;
  }

  timeLabel.textContent = motion ? `${sec(playhead)} / ${sec(motion.stats.duration)}` : "0.0 s";
  scrubber.value = motion ? String(Math.round((playhead / Math.max(motion.stats.duration, 0.001)) * 1000)) : "0";
  updateStage3d(active, liveOverride ? jogLivePoint3 : null);
}

function distanceToSegment(point, a, b) {
  const ax = a.x;
  const ay = a.y;
  const bx = b.x;
  const by = b.y;
  const dx = bx - ax;
  const dy = by - ay;
  const denom = dx * dx + dy * dy || 1;
  const t = Math.max(0, Math.min(1, ((point.x - ax) * dx + (point.y - ay) * dy) / denom));
  const x = ax + dx * t;
  const y = ay + dy * t;
  return Math.hypot(point.x - x, point.y - y);
}

function nearestPath(event) {
  if (!session) return null;
  const rect = canvas.getBoundingClientRect();
  const point = { x: event.clientX - rect.left, y: event.clientY - rect.top };
  let best = null;
  let bestDistance = 12;
  for (const path of session.paths) {
    for (let i = 1; i < path.points.length; i += 1) {
      const d = distanceToSegment(point, project(path.points[i - 1]), project(path.points[i]));
      if (d < bestDistance) {
        best = path.id;
        bestDistance = d;
      }
    }
  }
  return best;
}

canvas.addEventListener("mousemove", (event) => {
  hoverPath = nearestPath(event);
  draw();
});

canvas.addEventListener("mouseleave", () => {
  hoverPath = null;
  draw();
});

canvas.addEventListener("click", (event) => {
  const id = nearestPath(event);
  if (id === null) return;
  if (selected.has(id)) selected.delete(id);
  else selected.add(id);
  renderPathList();
  updateMetrics();
  schedulePlan();
});

pathList.addEventListener("change", (event) => {
  const row = event.target.closest(".path-row");
  if (!row) return;
  const id = Number(row.dataset.path);
  if (event.target.type === "checkbox") {
    if (event.target.checked) selected.add(id);
    else selected.delete(id);
  }
  if (event.target.classList.contains("pass-input")) {
    passCounts.set(id, Math.max(1, Number(event.target.value || 1)));
    selected.add(id);
  }
  renderPathList();
  updateMetrics();
  schedulePlan();
});

pathList.addEventListener("mouseover", (event) => {
  const row = event.target.closest(".path-row");
  hoverPath = row ? Number(row.dataset.path) : null;
  draw();
});

pathList.addEventListener("mouseleave", () => {
  hoverPath = null;
  draw();
});

traceFileButton.addEventListener("click", () => traceCurrent("file").catch((error) => setStatus(error.message)));
tracePathButton.addEventListener("click", () => traceCurrent("path").catch((error) => setStatus(error.message)));
applyScaleButton.addEventListener("click", () => applyScale().catch((error) => setStatus(error.message)));
sourcePath.addEventListener("keydown", (event) => {
  if (event.key === "Enter") traceCurrent("path").catch((error) => setStatus(error.message));
});
clearFileButton.addEventListener("click", () => {
  fileInput.value = "";
  setStatus("File selection cleared.");
});
sampleButton.addEventListener("click", () => loadDefault().catch((error) => setStatus(error.message)));
exportButton.addEventListener("click", () => exportCode().catch((error) => setStatus(error.message)));
optimizeToggle.addEventListener("change", schedulePlan);
feedRate.addEventListener("change", schedulePlan);
travelRate.addEventListener("change", schedulePlan);
zSpeed?.addEventListener("change", schedulePlan);
for (const tab of shapeTabs) {
  tab.addEventListener("click", () => setShapeMode(tab.dataset.shape));
}
for (const tab of panelTabs) {
  tab.addEventListener("click", () => setPanelTab(tab.dataset.tab));
}
// Match whichever tab the static HTML already marks active (Jog, by
// default) -- setPanelTab itself only runs on a later click.
updateCameraBigView(document.querySelector(".panel-tab.active")?.dataset.tab);
newManualButton.addEventListener("click", () => createManualSession([], "Blank manual job ready.").catch((error) => setStatus(error.message)));
addShapeButton.addEventListener("click", () => addShape().catch((error) => setStatus(error.message)));
deleteSelectedButton.addEventListener("click", () => deleteSelectedPaths().catch((error) => setStatus(error.message)));

selectAllButton.addEventListener("click", () => {
  if (!session) return;
  selected = new Set(session.paths.map((path) => path.id));
  renderPathList();
  updateMetrics();
  schedulePlan();
});

selectNoneButton.addEventListener("click", () => {
  selected.clear();
  renderPathList();
  updateMetrics();
  schedulePlan();
});

applyPassesButton.addEventListener("click", () => {
  const passes = Math.max(1, Number(globalPasses.value || 1));
  for (const id of selected) passCounts.set(id, passes);
  renderPathList();
  schedulePlan();
});

restartButton.addEventListener("click", () => {
  playhead = 0;
  playing = true;
  playButton.textContent = "||";
  draw();
});

playButton.addEventListener("click", () => {
  playing = !playing;
  playButton.textContent = playing ? "||" : ">";
  lastFrame = performance.now();
});

scrubber.addEventListener("input", () => {
  if (!motion) return;
  playing = false;
  playButton.textContent = ">";
  playhead = (Number(scrubber.value) / 1000) * motion.stats.duration;
  draw();
});

function tick(now) {
  const elapsed = (now - lastFrame) / 1000;
  lastFrame = now;
  if (playing && motion) {
    playhead += elapsed * PLAYBACK_RATE;
    if (playhead >= motion.stats.duration) {
      playhead = motion.stats.duration;
      playing = false;
      playButton.textContent = ">";
    }
    draw();
  }
  requestAnimationFrame(tick);
}

jogConnectButton.addEventListener("click", () => jogConnect());
jogDisconnectButton.addEventListener("click", () => jogDisconnect());
jogEstopButton.addEventListener("click", () => jogEstop());
jogHomeButton.addEventListener("click", () => jogHome());
jogSetLocalHomeButton.addEventListener("click", () => jogSetLocalHome());
jogGoLocalHomeButton.addEventListener("click", () => jogGoLocalHome());
jogGotoButton.addEventListener("click", () => jogGoto());
jogRunButton.addEventListener("click", () => jogRunPlan());
jogRunStopButton.addEventListener("click", () => jogRunStopFn());

for (const button of jogStepOptions.querySelectorAll("button")) {
  button.addEventListener("click", () => {
    jogStepMm = Number(button.dataset.step);
    for (const other of jogStepOptions.querySelectorAll("button")) other.classList.toggle("active", other === button);
  });
}

for (const button of [...jogPad.querySelectorAll(".jog-key"), ...jogZRow.querySelectorAll(".jog-key")]) {
  const axis = button.dataset.axis;
  const dir = Number(button.dataset.dir);
  JOG_KEY_ELEMENTS.set(`${axis}:${dir}`, button);
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    startJogHold(axis, dir);
  });
  button.addEventListener("pointerup", stopJogHold);
  button.addEventListener("pointerleave", stopJogHold);
  button.addEventListener("pointercancel", stopJogHold);
}
// Redundant release handlers at the window level ensure the jog hold stops
// regardless of where the pointer/mouse is released.
window.addEventListener("pointerup", stopJogHold);
window.addEventListener("mouseup", stopJogHold);
window.addEventListener("blur", stopJogHold);

const JOG_KEY_MAP = {
  ArrowUp: ["y", 1],
  ArrowDown: ["y", -1],
  ArrowLeft: ["x", -1],
  ArrowRight: ["x", 1],
  PageUp: ["z", 1],
  PageDown: ["z", -1],
};
const jogKeysHeld = new Set();

window.addEventListener("keydown", (event) => {
  const tag = event.target.tagName;
  if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
  if (event.key === " " || event.code === "Space") {
    event.preventDefault();
    jogEstop();
    return;
  }
  const mapped = JOG_KEY_MAP[event.key];
  if (!mapped || event.repeat || jogKeysHeld.has(event.key)) return;
  event.preventDefault();
  jogKeysHeld.add(event.key);
  startJogHold(mapped[0], mapped[1]);
});

window.addEventListener("keyup", (event) => {
  if (jogKeysHeld.has(event.key)) {
    jogKeysHeld.delete(event.key);
    if (jogKeysHeld.size === 0) stopJogHold();
  }
});

jogRefreshStatus();
jogPollTimer = setInterval(jogRefreshStatus, 150);

cameraConnectButton.addEventListener("click", () => cameraConnect());
cameraDisconnectButton.addEventListener("click", () => cameraDisconnect());
cameraStream.addEventListener("click", (evt) => cameraSelectTarget(evt));
cameraRefreshStatus();
cameraPollTimer = setInterval(cameraRefreshStatus, 500);

focusScanButton.addEventListener("click", () => focusScan(false));
focusScanWideButton.addEventListener("click", () => focusScan(true));
focusScanNarrowButton.addEventListener("click", () => focusScan(false, true));
focusCustomButton.addEventListener("click", () => focusRunCustomScan());
focusActiveToggle.addEventListener("change", () => focusSetActive(focusActiveToggle.checked));
focusLatencyButton.addEventListener("click", () => focusRunLatencyCheck());
focusSurveyButton.addEventListener("click", () => focusStartSurvey());
focusSurveyStopButton.addEventListener("click", () => focusStopSurvey());
focusRefreshStatus();
focusPollTimer = setInterval(focusRefreshStatus, 400);

window.addEventListener("resize", resizeCanvas);
initStage3d();
loadDefault().catch((error) => setStatus(error.message));
requestAnimationFrame(tick);

