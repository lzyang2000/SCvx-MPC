import { createSimulationAudio } from "./simulation-audio.js";
import { initializeLanguage } from "./i18n.js";
import { createPlanningStatus } from "./planning-status.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ROCKET } from "./compute/rocket-dynamics.js";
import { createRocket } from "./rocket-model.js";
import { createRocketCamera } from "./rocket-camera.js";
import { createRocketInspector } from "./rocket-inspector.js";
import { computeEvents } from "./compute/client.js";
import { createViewportResize } from "./viewport.js";
initializeLanguage();
const planningStatus = createPlanningStatus(document.querySelector(".viewport"));
const audio = createSimulationAudio("rocket");
const $ = (id) => document.getElementById(id),
  point = (p) => new THREE.Vector3(p[0], p[2], -p[1]),
  quat = (q) => new THREE.Quaternion(q[1], q[3], -q[2], q[0]);
const scene = new THREE.Scene();
scene.background = new THREE.Color("#080d15");
// A centimetre-scale pad and a kilometre-scale flight share this depth buffer.
const renderer = new THREE.WebGLRenderer({
  antialias: true,
  logarithmicDepthBuffer: true,
});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.3;
$("scene").appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(42, 1, 0.3, 2e6),
  controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = 15;
controls.maxDistance = 600000;
// Keep orbit views above the ground; the terrain is an upward-facing surface.
controls.maxPolarAngle = Math.PI / 2 - 0.02;
scene.add(new THREE.HemisphereLight(0xeaf5ff, 0x273344, 2.8));
const sun = new THREE.DirectionalLight(0xf5f7ff, 3.5);
sun.position.set(-100, 200, 120);
scene.add(sun);
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(1e6, 1e6, 64, 64),
  new THREE.MeshStandardMaterial({ color: 0x171f2b, roughness: 1 }),
);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.025;
scene.add(ground);
const grid = new THREE.GridHelper(2000, 100, 0x526780, 0x29384a);
grid.position.y = 0.02;
scene.add(grid);
function cylinder(r, h, mat, x = 0, y = 0) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 64), mat);
  m.position.set(x, y, 0);
  scene.add(m);
  return m;
}
const concrete = new THREE.MeshStandardMaterial({
  color: 0x394859,
  roughness: 0.9,
});
cylinder(20, 0.16, concrete, ROCKET.padX, 0.08);
for (const [inner, outer] of [
  [18.4, 19],
  [7.2, 8.8],
]) {
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(inner, outer, 64),
    new THREE.MeshBasicMaterial({
      color: 0xd4dce7,
      side: THREE.DoubleSide,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -1,
      polygonOffsetUnits: -1,
    }),
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.set(ROCKET.padX, 0.18, 0);
  scene.add(ring);
}
const tower = new THREE.Group();
for (const x of [-12.4, -9.6])
  for (const z of [-1.4, 1.4]) {
    const m = new THREE.Mesh(
      new THREE.BoxGeometry(0.35, 72, 0.35),
      new THREE.MeshStandardMaterial({
        color: 0x8a9aae,
        metalness: 0.7,
        roughness: 0.5,
      }),
    );
    m.position.set(x, 36, z);
    tower.add(m);
  }
for (let y = 6; y <= 66; y += 12) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(3.5, 0.25, 3.5), concrete);
  m.position.set(-11, y, 0);
  tower.add(m);
}
scene.add(tower);
const rocket = createRocket();
scene.add(rocket.booster, rocket.upper);
const trail = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0xd4dce7 }),
);
trail.frustumCulled = false;
scene.add(trail);
const prediction = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineDashedMaterial({
    color: 0x4da3ff,
    dashSize: 0.8,
    gapSize: 0.4,
    depthTest: false,
  }),
);
prediction.renderOrder = 2;
scene.add(prediction);
const missionPath = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({
    color: 0x50c8bd,
    transparent: true,
    opacity: 0.5,
    depthTest: false,
  }),
);
missionPath.renderOrder = 1;
scene.add(missionPath);
let drawnMissionPlan = null;
function setMissionPath(plan) {
  if (drawnMissionPlan === plan) return;
  drawnMissionPlan = plan;
  missionPath.geometry.dispose();
  missionPath.geometry = new THREE.BufferGeometry().setFromPoints(
    (plan?.prediction ?? []).map(point),
  );
}
const beacon = new THREE.Mesh(
  new THREE.SphereGeometry(1, 12, 8),
  new THREE.MeshBasicMaterial({ color: 0xd4dce7, depthTest: false }),
);
beacon.renderOrder = 3;
scene.add(beacon);
let run = null,
  current = 0,
  index = 0,
  playing = false,
  rate = 7,
  mode = "follow",
  live = false,
  abort = null,
  last = null,
  lastSolve = null,
  frame = null,
  rateLive = 7;
const trackingCamera = createRocketCamera(camera, controls);
const updateInspector = createRocketInspector();
function cameraAt(snap = false, dt = 0.016) {
  if (!frame || !["follow", "upper"].includes(mode)) return;
  const part = frame.attached
    ? "stack"
    : mode === "upper"
      ? "upper"
      : "booster";
  trackingCamera.update(
    part === "upper" ? rocket.upper : rocket.booster,
    part,
    { snap, dt },
  );
}
function setView(view) {
  mode = view;
  document
    .querySelectorAll("[data-view]")
    .forEach((b) => b.classList.toggle("selected", b.dataset.view === view));
  controls.enabled = ["overview", "pad"].includes(view);
  if (view === "pad") {
    camera.position.set(ROCKET.padX + 60, 45, 100);
    controls.target.set(ROCKET.padX, 24, 0);
    controls.update();
  }
  if (view === "overview" && run) {
    const fs = run.frames,
      top = Math.max(150, ...fs.map((f) => f.p[2])),
      xmin = Math.min(-100, ...fs.map((f) => f.p[0])),
      xmax = Math.max(100, ...fs.map((f) => f.p[0])),
      centerX = (xmin + xmax) / 2;
    camera.position.set(centerX + top * 0.3, top * 0.48, top * 1.65);
    controls.target.set(centerX, top * 0.48, 0);
    controls.update();
  }
  cameraAt(true);
}
function appendPath(frame) {
  let attribute = trail.geometry.getAttribute("position");
  const count = run.frames.length;
  if (!attribute || attribute.count < count) {
    const capacity = Math.max(1024, 2 * (attribute?.count ?? 0)),
      array = new Float32Array(capacity * 3);
    if (attribute) array.set(attribute.array);
    attribute = new THREE.BufferAttribute(array, 3);
    attribute.setUsage(THREE.DynamicDrawUsage);
    trail.geometry.setAttribute("position", attribute);
  }
  attribute.setXYZ(count - 1, frame.p[0], frame.p[2], -frame.p[1]);
  attribute.addUpdateRange((count - 1) * 3, 3);
  attribute.needsUpdate = true;
}
function setPath() {
  trail.geometry.dispose();
  trail.geometry = new THREE.BufferGeometry().setFromPoints(
    run.frames.map((f) => point(f.p)),
  );
}
function setRun(
  data,
  { time = 0, autoplay = false, preserveCamera = false } = {},
) {
  run = data;
  current = Math.min(time, data.frames.at(-1).t);
  index = 0;
  playing = autoplay;
  $("play").textContent = playing ? "Ⅱ" : "▶";
  $("timeline").max = data.frames.at(-1).t;
  $("duration").textContent = "/ " + data.frames.at(-1).t.toFixed(1) + " s";
  setPath();
  setMissionPath(data.flight_plan);
  $("phase-jumps").replaceChildren();
  $("phase-ticks").replaceChildren();
  for (const event of data.events) {
    const b = document.createElement("button");
    b.className = "outline-button";
    b.textContent = event.name;
    b.onclick = () => {
      current = event.t;
      playing = false;
      update();
      cameraAt(true);
    };
    $("phase-jumps").appendChild(b);
    const s = document.createElement("span");
    s.textContent = event.name;
    s.style.left = (100 * event.t) / data.frames.at(-1).t + "%";
    $("phase-ticks").appendChild(s);
  }
  const summary = data.summary;
  $("compute-status").textContent = summary.success
    ? "Booster landed · " + summary.pad_error.toFixed(2) + " m from pad center"
    : (summary.failure ?? "Mission incomplete");
  $("outcome").classList.toggle("failed", !summary.success);
  $("outcome").textContent = summary.success
    ? `Booster landed · ${summary.pad_error.toFixed(2)} m pad error · ${summary.fuel_remaining.toFixed(0)} kg propellant left`
    : (summary.failure ?? "Mission incomplete");
  lastSolve = null;
  update();
  if (!preserveCamera) cameraAt(true);
  renderer.render(scene, camera);
}
function update() {
  if (!run?.frames.length) return;
  const fs = run.frames;
  while (index < fs.length - 2 && fs[index + 1].t <= current) index++;
  while (index > 0 && fs[index].t > current) index--;
  const a = fs[index],
    b = fs[Math.min(index + 1, fs.length - 1)],
    u = THREE.MathUtils.clamp((current - a.t) / (b.t - a.t || 1), 0, 1);
  frame = a;
  document.querySelector('[data-view="follow"]').textContent = a.attached
    ? "Follow rocket"
    : "Follow booster";
  rocket.booster.position.lerpVectors(point(a.p), point(b.p), u);
  rocket.booster.quaternion.slerpQuaternions(quat(a.q), quat(b.q), u);
  const ap = a.attached ? a : a.upper,
    bp = b.attached ? b : b.upper;
  rocket.upper.position.lerpVectors(point(ap.p), point(bp.p), u);
  rocket.upper.quaternion.slerpQuaternions(quat(ap.q), quat(bp.q), u);
  rocket.update(a, $("jets").checked, current);
  const daylight = THREE.MathUtils.clamp(1 - a.p[2] / 110000, 0, 1);
  scene.background.setRGB(
    0.025 + 0.61 * daylight,
    0.04 + 0.68 * daylight,
    0.07 + 0.7 * daylight,
  );
  $("phase").textContent = a.phase;
  $("owner").textContent = a.controller;
  $("altitude").textContent = ((a.p[2] - 20.76) / 1000).toFixed(3);
  $("vz").textContent = a.v[2].toFixed(1);
  $("vx").textContent = Math.hypot(...a.v.slice(0, 2)).toFixed(1);
  $("fuel").textContent = a.fuel.toFixed(0);
  $("fuel-fill").style.width = (100 * a.fuel) / 407500 + "%";
  $("thrust").textContent = (
    Math.hypot(...a.control.slice(0, 3)) / 1e6
  ).toFixed(2);
  $("engines").textContent = "MN / " + a.engines;
  $("pad-error").textContent = Math.hypot(a.p[0] - ROCKET.padX, a.p[1]).toFixed(
    1,
  );
  $("time").textContent = current.toFixed(1);
  $("timeline").value = current;
  trail.visible = $("trail").checked;
  missionPath.visible = $("mission-plan").checked;
  const plans = run.flight_plans ?? [run.flight_plan];
  setMissionPath(plans.findLast((p) => p && (p.start ?? 0) <= current));
  trail.geometry.setDrawRange(0, index + 1);
  beacon.position.copy(rocket.booster.position);
  beacon.visible = mode === "overview";
  beacon.scale.setScalar(
    Math.max(1, camera.position.distanceTo(beacon.position) * 0.002),
  );
  const solve = updateInspector(run, a, current, live);
  prediction.visible = $("prediction").checked && !!solve;
  if (solve !== lastSolve) {
    lastSolve = solve;
    prediction.geometry.dispose();
    prediction.geometry = new THREE.BufferGeometry().setFromPoints(
      (solve?.prediction ?? []).map(point),
    );
    prediction.computeLineDistances();
    prediction.material.color.setHex(solve?.success ? 0x4da3ff : 0xff7b86);
  }
  $("outcome").hidden = live || current < fs.at(-1).t - 0.1;
}
function controlsLive(value) {
  live = value;
  for (const id of [
    "compute",
    "separation-altitude",
    "live-speed",
    "play",
    "restart",
    "rate",
    "timeline",
    "export",
    "inspect-mpc",
    "next-solve",
  ])
    $(id).disabled = value;
  $("live").textContent = value ? "Stop live ■" : "Run live ●";
  $("transport").textContent = value
    ? "LIVE / BROWSER WASM"
    : "RECORDED / WASM";
}
async function start(liveMode) {
  if (live) {
    abort.abort();
    return;
  }
  const previous = run;
  abort = new AbortController();
  controlsLive(true);
  playing = false;
  rateLive = Number($("live-speed").value);
  let received = false,
    finished = false;
  try {
    for await (const event of computeEvents(
      "rocket",
      { separationAltitude: Number($("separation-altitude").value) * 1000 },
      { live: liveMode, speed: rateLive, signal: abort.signal },
    )) {
      planningStatus.update(event);
      if (event.type === "status")
        $("compute-status").textContent = event.data.message;
      if (event.type === "start") {
        run = {
          config: event.data.config,
          frames: [],
          events: [],
          summary: { mpc_solves: [], mpc_accepted: 0, mpc_rejected: 0 },
        };
        index = 0;
        current = 0;
        lastSolve = null;
        received = true;
        $("outcome").hidden = true;
        $("phase-jumps").replaceChildren();
        $("phase-ticks").replaceChildren();
      }
      if (event.type === "plan") {
        run.flight_plan ??= event.data;
        (run.flight_plans ??= []).push(event.data);
        setMissionPath(event.data);
      }
      if (event.type === "solve") {
        run.summary.mpc_solves.push(event.data);
        run.summary[event.data.success ? "mpc_accepted" : "mpc_rejected"]++;
      }
      if (event.type === "frame") {
        run.frames.push(event.data);
        appendPath(event.data);
        $("compute-status").textContent =
          `Computing ${event.data.phase.toLowerCase()} · t=${event.data.t.toFixed(1)} s`;
        $("timeline").max = event.data.t;
        $("duration").textContent = "/ LIVE";
        if (run.frames.length === 1) {
          update();
          // Reset the selected view around the launch site, including orbit views
          // left at a different position by the previous flight. Paint before the
          // worker begins its long initial solve instead of awaiting the next RAF.
          setView(mode);
          renderer.render(scene, camera);
        }
      }
      if (event.type === "complete") {
        finished = true;
        controlsLive(false);
        setRun(event.data, {
          time: liveMode ? current : 0,
          autoplay: liveMode,
          preserveCamera: true,
        });
      }
    }
  } catch (error) {
    controlsLive(false);
    if (received && run.frames.length) {
      run.summary = {
        ...run.summary,
        success: false,
        failure: "Mission stopped before completion",
      };
      setRun(run, { time: current });
    } else if (previous) setRun(previous);
    $("compute-status").textContent =
      error.name === "AbortError"
        ? "Mission stopped. You can replay the flight so far."
        : error.message;
  } finally {
    planningStatus.clear();
    controlsLive(false);
    abort = null;
    if (!finished) playing = false;
  }
}
$("live").onclick = () => start(true);
$("compute").onclick = () => start(false);
$("play").onclick = () => {
  if (!run) return;
  if (current >= run.frames.at(-1).t) current = 0;
  playing = !playing;
  $("play").textContent = playing ? "Ⅱ" : "▶";
};
$("restart").onclick = () => {
  current = 0;
  update();
  cameraAt(true);
};
$("timeline").oninput = () => {
  current = Number($("timeline").value);
  update();
  cameraAt(true);
};
$("inspect-mpc").onclick = () => {
  if (!run || live) return;
  playing = false;
  $("play").textContent = "▶";
  setView("follow");
};
$("next-solve").onclick = () => {
  if (!run || live) return;
  const next = run.summary.mpc_solves.find(
    (s) => s.t > current + 0.001 && s.t <= run.frames.at(-1).t,
  );
  if (!next) return;
  const nextFrame = run.frames.find(
    (f) => f.t >= next.t && (f.mpc_id == null || f.mpc_id >= next.id),
  );
  if (!nextFrame) return;
  current = nextFrame.t;
  playing = false;
  $("play").textContent = "▶";
  update();
  cameraAt(true);
};
$("rate").onchange = () => (rate = Number($("rate").value));
$("separation-altitude").oninput = () =>
  ($("separation-altitude-out").textContent =
    $("separation-altitude").value + " km");
for (const b of document.querySelectorAll("[data-view]"))
  b.onclick = () => setView(b.dataset.view);
for (const id of ["prediction", "trail", "jets", "mission-plan"])
  $(id).onchange = update;
$("about").onclick = () => $("notes").showModal();
$("close-notes").onclick = () => $("notes").close();
$("export").onclick = () => {
  if (!run) return;
  const url = URL.createObjectURL(
      new Blob([JSON.stringify(run)], { type: "application/json" }),
    ),
    a = document.createElement("a");
  a.href = url;
  a.download = "rocket-scvx-mission.json";
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
};
window.addEventListener("message", (e) => {
  if (e.origin === location.origin && e.data?.type === "pause-demo") {
    abort?.abort();
    playing = false;
    $("play").textContent = "▶";
  }
});
window.addEventListener("pagehide", () => abort?.abort());
const resizeViewport = createViewportResize(renderer, camera, () => {
  cameraAt(true);
  renderer.render(scene, camera);
});
new ResizeObserver(() => {
  resizeViewport($("scene").getBoundingClientRect());
}).observe($("scene"));
renderer.setAnimationLoop((timestamp) => {
  const dt = last === null ? 0 : Math.min(0.07, (timestamp - last) / 1000);
  last = timestamp;
  if (live && run?.frames.length) {
    const latest = run.frames.at(-1).t;
    if (latest - current > 0.5 * rateLive || current > 0)
      current = Math.min(latest, current + dt * rateLive);
    update();
  } else if (playing && run) {
    current = Math.min(run.frames.at(-1).t, current + dt * rate);
    if (current >= run.frames.at(-1).t) {
      playing = false;
      $("play").textContent = "▶";
    }
    update();
  }
  if (controls.enabled) controls.update();
  else cameraAt(false, dt);
  audio.update(frame, { time: current, playing: playing || live });
  renderer.render(scene, camera);
});
try {
  const response = await fetch("./rocket-nominal.json");
  if (!response.ok)
    throw Error(
      "The rocket recording could not be loaded. Choose Run live to compute a new mission.",
    );
  setRun(await response.json());
  setView("follow");
} catch (error) {
  $("compute-status").textContent = error.message;
  rocket.booster.position.y = 20.76;
  rocket.upper.position.y = 20.76;
  camera.position.set(50, 50, 85);
  controls.target.set(0, 35, 0);
  controls.update();
}
