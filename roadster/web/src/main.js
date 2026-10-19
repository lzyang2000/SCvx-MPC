import { createSimulationAudio } from "./simulation-audio.js";
import { initializeLanguage } from "./i18n.js";
import { createPlanningStatus } from "./planning-status.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { createMpcInspector } from "./mpc-inspector.js";
import { computeEvents } from "./compute/client.js";
import { createTrackingCamera } from "./tracking-camera.js";
import { createLivePlaybackClock } from "./live-playback.js";
import { bodyRollDegrees } from "./attitude-display.js";
import { rollingDistance } from "./wheel-motion.js";
import { createStageVehicles } from "./stage-models.js";
import { CAR_LAYOUT, STAGE_VEHICLES } from "./car-stage.js";
import { carSceneView } from "./car-camera.js";
import {
  createGroundTargetPicker,
  landingBounds,
} from "./car-target-picker.js";
import { createViewportResize } from "./viewport.js";

initializeLanguage();
const planningStatus = createPlanningStatus(document.querySelector(".viewport"));
const audio = createSimulationAudio("car");
const $ = (id) => document.getElementById(id);
const stage = $("scene");
const scene = new THREE.Scene();
scene.background = new THREE.Color("#080d15");
scene.fog = new THREE.Fog("#080d15", 90, 180);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.18;
stage.appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 300);
const controls = new OrbitControls(camera, renderer.domElement);
const trackingCamera = createTrackingCamera(camera, controls);
controls.enableDamping = true;
controls.minDistance = 5;
controls.maxDistance = 230;
controls.maxPolarAngle = Math.PI * 0.49;
const hemi = new THREE.HemisphereLight("#e5efff", "#273344", 2.6);
scene.add(hemi);
const sun = new THREE.DirectionalLight("#f5f7ff", 3.2);
sun.position.set(-12, 38, 20);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, {
  left: -100,
  right: 100,
  top: 100,
  bottom: -100,
  near: 0.5,
  far: 220,
});
sun.shadow.bias = -0.0003;
sun.shadow.normalBias = 0.04;
scene.add(sun);
const pmrem = new THREE.PMREMGenerator(renderer);
const room = new RoomEnvironment();
const env = pmrem.fromScene(room, 0.04);
scene.environment = env.texture;
room.dispose();
pmrem.dispose();
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(300, 300),
  new THREE.MeshStandardMaterial({ color: "#171f2b", roughness: 0.92 }),
);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.025;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(160, 80, "#526780", "#29384a");
grid.position.y = 0.005;
grid.material.transparent = true;
grid.material.opacity = 0.4;
scene.add(grid);

function line(points, color, opacity = 1) {
  const g = new THREE.BufferGeometry().setFromPoints(points);
  const m = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
  });
  return new THREE.Line(g, m);
}
function bar(a, b, r, mat) {
  const d = new THREE.Vector3().subVectors(b, a);
  const m = new THREE.Mesh(
    new THREE.CylinderGeometry(r, r, d.length(), 8),
    mat,
  );
  m.position.copy(a).add(b).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  m.castShadow = true;
  return m;
}
function textSprite(text, color = "#acbbce", size = 1) {
  const c = document.createElement("canvas");
  c.width = 512;
  c.height = 96;
  const ctx = c.getContext("2d");
  ctx.fillStyle = color;
  ctx.font = "500 36px monospace";
  ctx.textAlign = "center";
  ctx.fillText(text, 256, 58);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: t, transparent: true, depthWrite: false }),
  );
  s.scale.set(size * 5.3, size, 1);
  return s;
}
function floorLabel(text, x, z, size = 1) {
  const c = document.createElement("canvas");
  c.width = 1024;
  c.height = 128;
  const ctx = c.getContext("2d");
  ctx.font = "500 44px monospace";
  ctx.textAlign = "center";
  ctx.fillStyle = "#8295ad";
  ctx.fillText(text, 512, 78);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  const p = new THREE.Mesh(
    new THREE.PlaneGeometry(size * 8, size),
    new THREE.MeshBasicMaterial({
      map: t,
      transparent: true,
      depthWrite: false,
    }),
  );
  p.rotation.x = -Math.PI / 2;
  p.position.set(x, 0.012, z);
  scene.add(p);
  return p;
}
floorLabel("FLIGHT TEST / 001", -12, 7, 1.35);
const rampEntryLabel = floorLabel("RAMP ENTRY →", -7, 3.8, 0.55);
const stageVehicles = createStageVehicles();
stageVehicles.visible = false;
scene.add(stageVehicles);
for (const v of STAGE_VEHICLES) {
  const label = textSprite(v.name.toUpperCase(), "#a7b3c4", 0.4);
  label.position.set(v.x, 0.4, 5.5);
  stageVehicles.add(label);
}

const centerline = [];
for (let x = -31; x < 0; x += 2) {
  centerline.push(
    new THREE.Vector3(x, 0.018, 0),
    new THREE.Vector3(x + 1, 0.018, 0),
  );
}
scene.add(
  new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(centerline),
    new THREE.LineBasicMaterial({ color: "#657b96" }),
  ),
);
const pad = new THREE.Group();
const padMat = new THREE.MeshStandardMaterial({
  color: "#273445",
  roughness: 0.85,
});
const padMesh = new THREE.Mesh(new THREE.BoxGeometry(9, 0.03, 6), padMat);
padMesh.receiveShadow = true;
padMesh.position.y = -0.003;
pad.add(padMesh);
for (const z of [-2.9, 2.9])
  pad.add(
    line(
      [new THREE.Vector3(-4.4, 0.025, z), new THREE.Vector3(4.4, 0.025, z)],
      "#8da1ba",
    ),
  );
const padRing = new THREE.Mesh(
  new THREE.RingGeometry(1.6, 1.64, 64),
  new THREE.MeshBasicMaterial({ color: "#b4c5da", side: THREE.DoubleSide }),
);
padRing.rotation.x = -Math.PI / 2;
padRing.position.y = 0.025;
pad.add(padRing);
scene.add(pad);
const landingZone = new THREE.Mesh(
  new THREE.PlaneGeometry(1, 7),
  new THREE.MeshBasicMaterial({
    color: "#4aafff",
    transparent: true,
    opacity: 0.18,
    depthWrite: false,
    side: THREE.DoubleSide,
  }),
);
landingZone.rotation.x = -Math.PI / 2;
landingZone.position.y = 0.015;
scene.add(landingZone);
let ramp = new THREE.Group();
scene.add(ramp);
const steel = new THREE.MeshStandardMaterial({
  color: "#889aaf",
  metalness: 0.6,
  roughness: 0.45,
});
const darkSteel = new THREE.MeshStandardMaterial({
  color: "#3e5066",
  metalness: 0.65,
  roughness: 0.46,
});
function buildRamp(cfg) {
  scene.remove(ramp);
  ramp.traverse((o) => {
    o.geometry?.dispose();
    if (o.material && ![steel, darkSteel].includes(o.material))
      o.material.dispose();
  });
  ramp = new THREE.Group();
  scene.add(ramp);
  scene.fog.near = cfg.maneuver === "barrel-roll" ? 180 : 90;
  scene.fog.far = cfg.maneuver === "barrel-roll" ? 400 : 180;
  stageVehicles.visible = cfg.maneuver === "barrel-roll";
  rampEntryLabel.visible = !stageVehicles.visible;
  if (stageVehicles.visible) return;
  const r = cfg.radius,
    end = THREE.MathUtils.degToRad(cfg.ramp_degrees),
    width = 4.3,
    steps = 120;
  const positions = [],
    indices = [];
  for (let i = 0; i <= steps; i++) {
    const a = (end * i) / steps;
    for (const z of [-width / 2, width / 2])
      positions.push(r * Math.sin(a), r * (1 - Math.cos(a)), z);
    if (i < steps) {
      const k = i * 2;
      indices.push(k, k + 1, k + 2, k + 1, k + 3, k + 2);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  const deck = new THREE.Mesh(
    geo,
    new THREE.MeshStandardMaterial({
      color: "#75879d",
      metalness: 0.35,
      roughness: 0.76,
      side: THREE.DoubleSide,
    }),
  );
  deck.castShadow = true;
  deck.receiveShadow = true;
  ramp.add(deck);
  for (const z of [-width / 2, width / 2]) {
    const edge = [],
      stripe = [];
    for (let i = 0; i <= steps; i++) {
      const a = (end * i) / steps;
      edge.push(new THREE.Vector3(r * Math.sin(a), r * (1 - Math.cos(a)), z));
      stripe.push(
        new THREE.Vector3(
          (r - 0.03) * Math.sin(a),
          r - (r - 0.03) * Math.cos(a),
          z * 0.88,
        ),
      );
    }
    ramp.add(
      new THREE.Mesh(
        new THREE.TubeGeometry(
          new THREE.CatmullRomCurve3(edge),
          120,
          0.065,
          6,
          false,
        ),
        darkSteel,
      ),
    );
    ramp.add(line(stripe, "#4da3ff"));
    const outside = r + 0.75;
    const out = [];
    for (let i = 0; i <= steps; i++) {
      const a = (end * i) / steps;
      out.push(
        new THREE.Vector3(outside * Math.sin(a), r - outside * Math.cos(a), z),
      );
    }
    ramp.add(
      new THREE.Mesh(
        new THREE.TubeGeometry(
          new THREE.CatmullRomCurve3(out),
          120,
          0.05,
          6,
          false,
        ),
        steel,
      ),
    );
    for (let i = 4; i <= 24; i++) {
      const a = (end * i) / 24,
        prev = (end * (i - 1)) / 24;
      const inner = new THREE.Vector3(r * Math.sin(a), r - r * Math.cos(a), z),
        outer = new THREE.Vector3(
          outside * Math.sin(a),
          r - outside * Math.cos(a),
          z,
        ),
        past = new THREE.Vector3(
          outside * Math.sin(prev),
          r - outside * Math.cos(prev),
          z,
        );
      ramp.add(bar(inner, outer, 0.035, steel));
      ramp.add(bar(inner, past, 0.027, steel));
    }
    // Towers stand outside the contact surface, with triangular ground bracing.
    for (const a of [end * 0.3, end * 0.55, end * 0.8, end]) {
      const x = outside * Math.sin(a),
        y = r - outside * Math.cos(a);
      const top = new THREE.Vector3(x, y, z),
        foot = new THREE.Vector3(x, 0.05, z * 1.4);
      ramp.add(bar(foot, top, 0.055, steel));
      ramp.add(
        bar(new THREE.Vector3(x + 2.3, 0.05, z * 1.4), top, 0.035, steel),
      );
    }
  }
  for (let i = 1; i < 24; i++) {
    const a = (end * i) / 24;
    const rr = r + 0.12;
    ramp.add(
      bar(
        new THREE.Vector3(rr * Math.sin(a), r - rr * Math.cos(a), -width / 2),
        new THREE.Vector3(rr * Math.sin(a), r - rr * Math.cos(a), width / 2),
        0.04,
        steel,
      ),
    );
  }
  for (const z of [-1, 1]) {
    const pts = [];
    for (let i = 0; i <= steps; i++) {
      const a = (end * i) / steps;
      pts.push(
        new THREE.Vector3(
          (r - 0.015) * Math.sin(a),
          r - (r - 0.015) * Math.cos(a),
          z,
        ),
      );
    }
    ramp.add(line(pts, "#a7b9ce", 0.8));
  }
  const label = textSprite("C / " + r.toFixed(0) + " M", "#b6c7dd", 0.8);
  label.position.set(r + 1.5, r * 0.6, 3.8);
  ramp.add(label);
  const lip = textSprite("RELEASE", "#4da3ff", 0.45);
  lip.position.set(r * Math.sin(end) - 1, r * (1 - Math.cos(end)) + 1.4, 0);
  ramp.add(lip);
}

const car = new THREE.Group();
scene.add(car);
const modelFrame = new THREE.Group();
modelFrame.rotation.x = -Math.PI / 2;
car.add(modelFrame);
const wheelAxles = [];
let wheelDistance = [];
const carReady = new GLTFLoader()
  .loadAsync("./assets/roadster.gltf")
  .then((g) => {
    g.scene.traverse((o) => {
      if (o.name.startsWith("Wheel_axle") || o.name.startsWith("Wheel axle")) {
        o.userData.physicsWheel =
          (o.position.x > 0 ? 2 : 0) + (o.position.y > 0 ? 1 : 0);
        wheelAxles.push(o);
      }
      if (o.isMesh) {
        o.castShadow = true;
        o.receiveShadow = true;
      }
    });
    modelFrame.add(g.scene);
    $("load-status").textContent = "glTF / 6 JETS";
    return g.scene;
  })
  .catch((e) => {
    showError("The car model could not be loaded. " + e.message);
    throw e;
  });
// Cold-air exhaust is a pale expanding cloud. No flame or combustion colors.
const plumeGroup = new THREE.Group();
modelFrame.add(plumeGroup);
const plumeMeshes = [];
const plumeMat = new THREE.MeshBasicMaterial({
  color: "#d8efff",
  transparent: true,
  opacity: 0.2,
  depthWrite: false,
  side: THREE.DoubleSide,
});
const puffGeometry = new THREE.SphereGeometry(1, 8, 6);
function buildPlumes(data) {
  plumeGroup.traverse((o) => {
    if (o.isMesh) o.material.dispose();
  });
  plumeGroup.clear();
  plumeMeshes.length = 0;
  for (let i = 0; i < data.nozzles.positions.length; i++) {
    const jet = new THREE.Group();
    const pos = new THREE.Vector3(...data.nozzles.positions[i]),
      dir = new THREE.Vector3(...data.nozzles.directions[i]).negate();
    jet.position.copy(pos);
    jet.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    for (let k = 0; k < 9; k++) {
      const m = new THREE.Mesh(puffGeometry, plumeMat.clone());
      jet.add(m);
    }
    plumeGroup.add(jet);
    plumeMeshes.push(jet);
  }
}
const trailMat = new THREE.LineBasicMaterial({
  color: "#d4dce7",
  transparent: true,
  opacity: 0.7,
});
let flightLine = new THREE.Line(new THREE.BufferGeometry(), trailMat);
scene.add(flightLine);
const plannedLine = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineDashedMaterial({
    color: "#cfae7d",
    transparent: true,
    opacity: 0.7,
    dashSize: 0.6,
    gapSize: 0.4,
  }),
);
scene.add(plannedLine);
function showFlightPlan(plan) {
  plannedLine.geometry.dispose();
  plannedLine.geometry = new THREE.BufferGeometry().setFromPoints(
    (plan?.prediction ?? []).map(mapPosition),
  );
  plannedLine.computeLineDistances();
  plannedLine.visible = !!plan;
  $("planner-status").textContent = plan
    ? `Full-flight plan · ${plan.duration.toFixed(1)} s flight · ${plan.seconds.toFixed(2)} s planning`
    : "Choose a target, then compute a new flight.";
}
const updateMpc = createMpcInspector(scene);
let liveAbort = null,
  liveRunning = false;
let liveDistance = 0;
let liveTrailCount = 0;
const liveClock = createLivePlaybackClock();
let run = null,
  playing = false,
  current = 0,
  rate = 1,
  mode = "overview",
  lastTimestamp = null,
  activeFrame = null;
let selectedManeuver = "c-ramp",
  loadingManeuver = false;
let index = 0;
const leds = [];
function buildNozzleLeds(data) {
  $("nozzle-grid").replaceChildren();
  leds.length = 0;
  for (let i = 0; i < data.nozzles.positions.length; i++) {
    const led = document.createElement("i");
    led.className = "jet-led";
    const [x, y] = data.nozzles.positions[i];
    led.style.left = `${50 - y * (i < 4 ? 17 : 40)}%`;
    led.style.top = `${50 - x * 18}%`;
    led.title = (i < 4 ? "Bottom" : "Front side") + " nozzle " + ((i % 4) + 1);
    $("nozzle-grid").appendChild(led);
    leds.push(led);
  }
}
const mapPosition = (p) => new THREE.Vector3(p[0], p[2], -p[1]);
function mapQuaternion(q) {
  return new THREE.Quaternion(q[1], q[3], -q[2], q[0]);
}
function setRun(
  data,
  { time = 0, autoplay = false, preserveCamera = false } = {},
) {
  $("transport-badge").textContent = "REPLAY";
  $("transport-badge").classList.remove("live");
  run = data;
  showFlightPlan(data.flight_plan);
  current = Math.min(time, data.frames.at(-1).t);
  playing = autoplay && current < data.frames.at(-1).t;
  index = 0;
  $("play").textContent = playing ? "Ⅱ" : "▶";
  $("play").setAttribute(
    "aria-label",
    playing ? "Pause simulation" : "Play simulation",
  );
  $("timeline").max = data.frames.at(-1).t;
  $("timeline").value = current;
  $("duration").textContent = "/ " + data.frames.at(-1).t.toFixed(2) + " s";
  selectedManeuver = data.config.maneuver ?? "c-ramp";
  updateManeuverUI();
  buildRamp(data.config);
  buildPlumes(data);
  buildNozzleLeds(data);
  wheelDistance = [0];
  for (let i = 1; i < data.frames.length; i++) {
    const f = data.frames[i],
      prev = data.frames[i - 1];
    const ds = rollingDistance(f, prev);
    wheelDistance.push(wheelDistance[i - 1] + ds);
  }
  const flight = data.frames.filter(
    (f) => !["APPROACH", "RAMP"].includes(f.phase),
  );
  flightLine.geometry.dispose();
  flightLine.geometry = new THREE.BufferGeometry().setFromPoints(
    flight.map((f) => mapPosition(f.p)),
  );
  flightLine.geometry.setDrawRange(0, 0);
  const td = data.summary.touchdown;
  pad.position.x =
    data.landing_target?.[0] ??
    data.config.landing_x ??
    CAR_LAYOUT[selectedManeuver].landing;
  $("landing-x").value = pad.position.x;
  $("landing-x-out").textContent = pad.position.x + " m";
  pad.scale.x = selectedManeuver === "barrel-roll" ? 2.5 : 1;
  const max = Math.max(...data.frames.map((f) => f.p[2]));
  const path = data.frames
    .map(
      (f, i) =>
        `${i ? "L" : "M"} ${((200 * i) / (data.frames.length - 1)).toFixed(2)} ${(46 - (42 * f.p[2]) / max).toFixed(2)}`,
    )
    .join(" ");
  $("alt-path").setAttribute("d", path);
  $("solver-summary").textContent =
    `SCvx · ${data.summary.mpc_accepted} accepted / ${data.summary.mpc_rejected} rejected`;
  $("compute-status").textContent =
    (data.summary.success ? "Landing successful" : "Landing unsuccessful") +
    ` · ${data.summary.compute_seconds.toFixed(2)} s compute`;
  $("outcome").classList.toggle("failed", !data.summary.success);
  $("outcome").innerHTML = data.summary.success
    ? `<b>Upright landing</b><span>${td.vertical_speed.toFixed(2)} m/s touchdown · ${td.tilt_degrees.toFixed(2)}° tilt<br>${data.summary.gas_remaining.toFixed(1)} kg of air remaining</span>`
    : `<b>Landing unsuccessful.</b><span>${data.summary.failure}${td ? `<br>${td.vertical_speed.toFixed(2)} m/s contact · ${td.tilt_degrees.toFixed(1)}° tilt` : ""}</span>`;
  $("phase-ticks").replaceChildren();
  for (const [label, t] of selectedManeuver === "barrel-roll"
    ? [
        ["DRIVE", 0],
        ["LIFT", data.events[0]?.t],
        [
          data.config.require_roll === false ? "CLEAR" : "ROLL",
          data.events[1]?.t,
        ],
        ["LAND", td?.t],
      ]
    : [
        ["DRIVE", 0],
        ["RAMP", data.events[0]?.t],
        ["FLIP", data.events[1]?.t],
        ["LAND", td?.t],
      ]) {
    if (t == null) continue;
    const tick = document.createElement("span");
    tick.textContent = label;
    tick.style.left = (100 * t) / data.frames.at(-1).t + "%";
    $("phase-ticks").appendChild(tick);
  }
  updateFrame();
  if (!preserveCamera) setView(mode);
}
function setView(view) {
  mode = view;
  document
    .querySelectorAll("[data-view]")
    .forEach((b) => b.classList.toggle("selected", b.dataset.view === view));
  const framing = carSceneView(run, camera.aspect, camera.fov, view);
  camera.far = Math.max(300, framing.distance * 3);
  camera.updateProjectionMatrix();
  controls.maxDistance = Math.max(230, framing.distance * 1.8);
  controls.enabled = !["follow", "mpc"].includes(view);
  const framed = view === "overview" || view === "side";
  scene.fog.near = framed ? framing.distance + 60 : 180;
  scene.fog.far = scene.fog.near + 220;
  if (framed) {
    camera.position.set(...framing.position);
    controls.target.set(...framing.target);
  }
  if (["follow", "mpc"].includes(view)) trackingCamera.snap(car.position, view);
  else controls.update();
}
function showError(message) {
  $("compute-status").textContent = message;
  $("compute-status").style.color = "#ff7b86";
  $("load-status").textContent = "LOAD ERROR";
}
function updateFrame() {
  if (!run) return;
  const fs = run.frames;
  while (index < fs.length - 2 && fs[index + 1].t <= current) index++;
  while (index > 0 && fs[index].t > current) index--;
  const a = fs[index],
    b = fs[Math.min(index + 1, fs.length - 1)],
    mix = Math.max(0, Math.min(1, (current - a.t) / (b.t - a.t || 1)));
  activeFrame = a;
  car.position.lerpVectors(mapPosition(a.p), mapPosition(b.p), mix);
  car.quaternion.slerpQuaternions(mapQuaternion(a.q), mapQuaternion(b.q), mix);
  for (const axle of wheelAxles) {
    const i = axle.userData.physicsWheel;
    axle.rotation.y = a.wheel_angles
      ? a.wheel_angles[i] +
        mix * ((b.wheel_angles?.[i] ?? a.wheel_angles[i]) - a.wheel_angles[i])
      : wheelDistance[index] / 0.37;
  }
  $("phase-label").textContent =
    a.phase === "APPROACH"
      ? "GROUND APPROACH"
      : a.phase === "FLIP"
        ? "AIRBORNE / ROLL"
        : a.phase === "SETTLED"
          ? "WHEEL CONTACT"
          : a.phase === "CRASH"
            ? "BODY IMPACT"
            : a.phase;
  $("controller-label").textContent = a.controller;
  $("altitude").textContent = a.p[2].toFixed(2);
  $("velocity").textContent = Math.hypot(...a.v).toFixed(2);
  $("roll-value").textContent = bodyRollDegrees(a.q).toFixed(1);
  $("air-value").textContent = a.gas.toFixed(1);
  $("gas-fill").style.width =
    (a.gas / Math.max(run.config.gas_kg, 1)) * 100 + "%";
  $("time").textContent = current.toFixed(2).padStart(5, "0");
  $("timeline").value = current;
  $("fan-value").textContent = "FAN " + a.fan_g.toFixed(2) + " g";
  $("contact-value").textContent = ["APPROACH", "RAMP", "SETTLED"].includes(
    a.phase,
  )
    ? "● WHEEL CONTACT"
    : "○ AIRBORNE";
  const showPlume = $("jets").checked;
  for (let i = 0; i < a.jets.length; i++) {
    const f = a.jets[i] / Math.max(run.config.nozzle_n, 1);
    leds[i].style.background =
      f > 0.01 ? `rgb(${77 + f * 80},${163 + f * 50},255)` : "";
    leds[i].title =
      `${i < 4 ? "Bottom" : "Front side"} nozzle ${(i % 4) + 1}: ${(a.jets[i] / 1000).toFixed(2)} kN`;
    const jet = plumeMeshes[i];
    if (!jet) continue;
    jet.visible = showPlume && f > 0.015;
    for (let k = 0; k < 9; k++) {
      const p = jet.children[k];
      const cycle = (current * 2.3 + k / 9) % 1;
      p.position.set(
        Math.sin(k * 3 + current * 12) * cycle * 0.12,
        0.13 + cycle * (0.8 + f * 3),
        Math.cos(k * 2 + current * 10) * cycle * 0.12,
      );
      const scale = 0.04 + cycle * 0.3;
      p.scale.set(scale, scale * 1.9, scale);
      p.material.opacity = (1 - cycle) * (0.15 + f * 0.18);
    }
  }
  const trailCount = fs
    .slice(0, index + 1)
    .filter((f) => !["APPROACH", "RAMP"].includes(f.phase)).length;
  flightLine.visible = $("trail").checked;
  flightLine.geometry.setDrawRange(0, trailCount);
  updateMpc(run, a, current);
  $("chart-cursor").setAttribute(
    "x1",
    (200 * current) / Math.max(fs.at(-1).t, 0.01),
  );
  $("chart-cursor").setAttribute(
    "x2",
    (200 * current) / Math.max(fs.at(-1).t, 0.01),
  );
  $("outcome").hidden =
    liveRunning ||
    !(current >= (run.summary.touchdown?.t ?? fs.at(-1).t) - 0.01);
}
const resizeViewport = createViewportResize(renderer, camera, () => {
  renderer.render(scene, camera);
});
function resize() {
  resizeViewport(stage.getBoundingClientRect());
}
new ResizeObserver(resize).observe(stage);
resize();
renderer.setAnimationLoop((timestamp) => {
  const elapsed =
    lastTimestamp === null
      ? 0
      : Math.min(0.07, (timestamp - lastTimestamp) / 1000);
  lastTimestamp = timestamp;
  if (liveRunning && run?.frames.length) {
    current = liveClock.advance(elapsed, run.frames.at(-1).t);
    updateFrame();
  } else if (playing && run) {
    current = Math.min(current + elapsed * rate, run.frames.at(-1).t);
    if (current >= run.frames.at(-1).t) {
      playing = false;
      $("play").textContent = "▶";
      $("play").setAttribute("aria-label", "Play simulation");
    }
    updateFrame();
  }
  if (["follow", "mpc"].includes(mode) && activeFrame)
    trackingCamera.update(car.position, mode, elapsed);
  else controls.update();
  plannedLine.visible = !!run?.flight_plan && $("prediction").checked;
  audio.update(activeFrame, { time: current, playing: playing || liveRunning });
  renderer.render(scene, camera);
});
function snapTracking() {
  if (["follow", "mpc"].includes(mode)) trackingCamera.snap(car.position, mode);
}
function togglePlay() {
  if (!run || liveRunning) return;
  if (current >= run.frames.at(-1).t) current = 0;
  playing = !playing;
  $("play").textContent = playing ? "Ⅱ" : "▶";
  $("play").setAttribute(
    "aria-label",
    playing ? "Pause simulation" : "Play simulation",
  );
}
$("play").onclick = togglePlay;
$("restart").onclick = () => {
  current = 0;
  index = 0;
  updateFrame();
  snapTracking();
};
$("timeline").oninput = (e) => {
  current = Number(e.target.value);
  updateFrame();
  snapTracking();
};
$("rate").onchange = (e) => {
  rate = Number(e.target.value);
};
document
  .querySelectorAll("[data-view]")
  .forEach((b) => (b.onclick = () => setView(b.dataset.view)));
for (const id of ["trail", "jets", "prediction"]) $(id).onchange = updateFrame;
for (const [id, unit] of [
  ["speed", "m/s"],
  ["radius", "m"],
  ["gas", "kg"],
])
  $(id).oninput = () => {
    $(id + "-out").textContent = $(id).value + " " + unit;
    $("compute-status").textContent =
      "Settings changed. Compute a new flight to apply them.";
  };
function selectLanding(x) {
  $("landing-x").value = x;
  $("landing-x-out").textContent = x + " m";
  pad.position.x = x;
  $("goal-planning").checked = true;
  $("mpc").checked = true;
  showFlightPlan(null);
  $("compute-status").textContent =
    "Landing target changed. Compute a new flight to apply it.";
}
const targetPicker = createGroundTargetPicker({
  canvas: renderer.domElement,
  camera,
  bounds: () => landingBounds(selectedManeuver),
  enabled: () => !!run && !liveRunning && !loadingManeuver,
  onSelect: selectLanding,
});
$("landing-x").oninput = () => {
  const value = Number($("landing-x").value);
  const [min, max] = landingBounds(selectedManeuver);
  if (
    $("landing-x").value !== "" &&
    Number.isFinite(value) &&
    value >= min &&
    value <= max
  )
    selectLanding(value);
};
$("landing-x").onchange = () => {
  const [min, max] = landingBounds(selectedManeuver);
  const value = Number($("landing-x").value);
  selectLanding(
    Math.max(
      min,
      Math.min(max, Number.isFinite(value) ? value : pad.position.x),
    ),
  );
};
for (const id of ["goal-planning", "require-roll"])
  $(id).onchange = () => {
    $("compute-status").textContent =
      "Settings changed. Compute a new flight to apply them.";
  };
$("mpc").onchange = () => {
  $("compute-status").textContent =
    "Controller changed. Compute a new flight to apply it.";
};
$("compute").onclick = () => runBrowser(false);
$("export").onclick = () => {
  if (!run) return;
  const blob = new Blob([JSON.stringify(run, null, 2)], {
    type: "application/json",
  });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = selectedManeuver + "-flight.json";
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
};
for (const id of ["about", "source-link"])
  $(id).onclick = () => $("notes").showModal();
$("close-notes").onclick = () => $("notes").close();
document.addEventListener("keydown", (e) => {
  if (
    e.code === "Space" &&
    !["INPUT", "BUTTON", "SELECT", "TEXTAREA"].includes(
      document.activeElement.tagName,
    ) &&
    !$("notes").open
  ) {
    e.preventDefault();
    togglePlay();
  }
});
try {
  const response = await fetch("./car-nominal.json");
  if (!response.ok)
    throw Error(
      "The default flight recording could not be loaded. Reload the page to try again.",
    );
  const data = await response.json();
  await carReady;
  setRun(data);
} catch (e) {
  showError(e.message);
}

function configuration() {
  return {
    maneuver: selectedManeuver,
    speed: Number($("speed").value),
    radius: Number($("radius").value),
    gas_kg: Number($("gas").value),
    landing_x: Number($("landing-x").value),
    goal_planning: $("goal-planning").checked,
    require_roll: $("require-roll").checked,
    use_mpc: $("mpc").checked,
  };
}
function setLiveControls(running) {
  liveRunning = running;
  landingZone.visible = !running;
  targetPicker.cancel();
  for (const id of [
    "compute",
    "play",
    "restart",
    "timeline",
    "rate",
    "inspect-mpc",
    "next-solve",
    "export",
    "speed",
    "radius",
    "gas",
    "landing-x",
    "goal-planning",
    "require-roll",
    "mpc",
    "maneuver-ramp",
    "maneuver-barrel",
  ])
    $(id).disabled = running;
  $("live").textContent = running ? "Stop live ■" : "Run live ●";
  $("live").classList.toggle("running", running);
  $("transport-badge").textContent = running ? "LIVE / WASM" : "REPLAY";
  $("transport-badge").classList.toggle("live", running);
}
function startLiveRun(config) {
  const empty = {
    config,
    frames: [],
    events: [],
    nozzles: run.nozzles,
    summary: { mpc_solves: [], mpc_accepted: 0, mpc_rejected: 0 },
  };
  run = empty;
  updateManeuverUI();
  showFlightPlan(null);
  current = 0;
  index = 0;
  playing = false;
  liveDistance = 0;
  liveClock.reset();
  wheelDistance = [];
  rate = 1;
  $("rate").value = "1";
  $("play").textContent = "▶";
  $("play").setAttribute("aria-label", "Play simulation");
  $("compute-status").textContent = "Preparing live run…";
  $("timeline").max = 24;
  $("duration").textContent = "/ LIVE";
  $("phase-ticks").replaceChildren();
  $("outcome").hidden = true;
  $("alt-path").setAttribute("d", "");
  pad.position.x = config.landing_x ?? CAR_LAYOUT[config.maneuver].landing;
  pad.scale.x = config.maneuver === "barrel-roll" ? 2.5 : 1;
  buildRamp(config);
  buildPlumes(empty);
  setView(mode);
  flightLine.geometry.dispose();
  flightLine.geometry = new THREE.BufferGeometry();
  flightLine.geometry.setAttribute(
    "position",
    new THREE.BufferAttribute(new Float32Array(3072), 3).setUsage(
      THREE.DynamicDrawUsage,
    ),
  );
  flightLine.geometry.setDrawRange(0, 0);
  liveTrailCount = 0;
}
function acceptLiveFrame(frame) {
  const prev = run.frames.at(-1);
  if (prev) liveDistance += rollingDistance(frame, prev);
  wheelDistance.push(liveDistance);
  run.frames.push(frame);
  if (!["APPROACH", "RAMP"].includes(frame.phase)) {
    let attribute = flightLine.geometry.getAttribute("position");
    if (liveTrailCount >= attribute.count) {
      const replacement = new THREE.BufferAttribute(
        new Float32Array(attribute.array.length * 2),
        3,
      ).setUsage(THREE.DynamicDrawUsage);
      replacement.array.set(attribute.array);
      flightLine.geometry.dispose();
      flightLine.geometry = new THREE.BufferGeometry();
      flightLine.geometry.setAttribute("position", replacement);
      attribute = replacement;
    }
    attribute.setXYZ(liveTrailCount, frame.p[0], frame.p[2], -frame.p[1]);
    attribute.addUpdateRange(liveTrailCount * 3, 3);
    attribute.needsUpdate = true;
    liveTrailCount++;
  }
  $("compute-status").textContent =
    `Live physics · t=${frame.t.toFixed(2)} s · ${frame.controller}`;
  if (!prev) {
    liveClock.reset(frame.t);
    current = frame.t;
    index = 0;
    updateFrame();
    snapTracking();
  }
}
async function runBrowser(live = true) {
  if (liveRunning) {
    liveAbort.abort();
    return;
  }
  if (!run || loadingManeuver) return;
  const previous = run;
  let completed = false;
  liveAbort = new AbortController();
  setLiveControls(true);
  playing = false;
  try {
    for await (const event of computeEvents("car", configuration(), {
      live,
      signal: liveAbort.signal,
    })) {
      planningStatus.update(event);
      if (event.type === "status")
        $("compute-status").textContent = event.data.message;
      if (event.type === "start") startLiveRun(event.data.config);
      if (event.type === "plan") {
        run.flight_plan = event.data;
        showFlightPlan(event.data);
      }
      if (event.type === "solve") {
        run.summary.mpc_solves.push(event.data);
        run.summary[event.data.success ? "mpc_accepted" : "mpc_rejected"]++;
        $("solver-summary").textContent =
          `SCvx · ${run.summary.mpc_accepted} accepted / ${run.summary.mpc_rejected} rejected`;
      }
      if (event.type === "frame") acceptLiveFrame(event.data);
      if (event.type === "complete") {
        completed = true;
        setLiveControls(false);
        setRun(event.data, {
          time: live ? current : 0,
          autoplay: live,
          preserveCamera: true,
        });
        $("compute-status").textContent = event.data.summary.success
          ? "Flight complete. Replay or drag the timeline to inspect it."
          : event.data.summary.failure;
      }
    }
  } catch (error) {
    setLiveControls(false);
    if (run !== previous && run.frames.length > 1) {
      run.summary = {
        ...run.summary,
        success: false,
        failure: "Run stopped before completion",
        touchdown: null,
        compute_seconds: 0,
        gas_remaining: run.frames.at(-1).gas,
      };
      setRun(run, { time: current, preserveCamera: true });
    } else if (!completed) setRun(previous);
    $("compute-status").textContent =
      error.name === "AbortError"
        ? "Run stopped. You can replay the flight so far."
        : "Could not compute flight: " + error.message;
  } finally {
    planningStatus.clear();
    setLiveControls(false);
    liveAbort = null;
  }
}
$("live").onclick = () => {
  if (!loadingManeuver) runBrowser(true);
};
window.addEventListener("message", (e) => {
  if (e.origin === location.origin && e.data?.type === "pause-demo") {
    liveAbort?.abort();
    playing = false;
    $("play").textContent = "▶";
  }
});
window.addEventListener("pagehide", () => liveAbort?.abort());
$("inspect-mpc").onclick = () => {
  if (!run) return;
  const first =
    run.summary.mpc_solves?.find((s) => s.phase === "FLIGHT") ??
    run.summary.mpc_solves?.[0];
  if (first) {
    current = Math.max(0, first.t - 0.2);
    index = 0;
    updateFrame();
    if (!playing) togglePlay();
  }
  setView("mpc");
};
$("next-solve").onclick = () => {
  const next = run?.summary.mpc_solves?.find((s) => s.t > current + 0.001);
  if (!next) return;
  playing = false;
  $("play").textContent = "▶";
  $("play").setAttribute("aria-label", "Play simulation");
  const frame = run.frames.find((f) => f.t >= next.t);
  current = frame?.t ?? next.t;
  updateFrame();
  snapTracking();
};

function updateManeuverUI() {
  const barrel = selectedManeuver === "barrel-roll";
  const roll = run?.config.require_roll !== false;
  $("maneuver-ramp").setAttribute("aria-pressed", String(!barrel));
  $("maneuver-barrel").setAttribute("aria-pressed", String(barrel));
  $("radius-control").hidden = barrel;
  $("require-roll-control").hidden = !barrel;
  const [minLanding, maxLanding] = landingBounds(selectedManeuver);
  $("landing-x").min = minLanding;
  $("landing-x").max = maxLanding;
  landingZone.position.x = (minLanding + maxLanding) / 2;
  landingZone.scale.x = maxLanding - minLanding;
  $("fan-spec").hidden = barrel;
  $("car-heading").textContent = barrel
    ? roll
      ? "Roadster barrel roll"
      : "Roadster jump"
    : "Roadster ramp recovery";
  $("car-description").textContent = barrel
    ? roll
      ? "Lift over five parked Teslas and roll 360°. Start beside the row and stop just beyond it."
      : "Lift over five parked Teslas and land upright at the selected target."
    : "Drive up the ramp, launch upside down and roll upright. Land at the selected target, facing the approach.";
  $("launch-spec-label").textContent = barrel ? "Takeoff" : "Ramp exit";
  $("launch-spec-value").textContent = barrel
    ? roll
      ? "Flat ground / 360° roll"
      : "Flat ground / upright jump"
    : "180° / inverted";
  $("speed-label").textContent = barrel ? "Approach speed" : "Ramp speed";
}
async function switchManeuver(maneuver) {
  if (liveRunning || loadingManeuver || selectedManeuver === maneuver) return;
  const previous = run;
  loadingManeuver = true;
  playing = false;
  setLiveControls(true);
  $("compute-status").textContent = "Loading maneuver…";
  try {
    const response = await fetch(
      maneuver === "barrel-roll"
        ? "./barrel-nominal.json"
        : "./car-nominal.json",
    );
    if (!response.ok) throw Error("Recording could not be loaded");
    const data = await response.json();
    for (const id of ["speed", "radius", "gas"]) {
      const key = id === "gas" ? "gas_kg" : id;
      $(id).value = data.config[key];
      $(id + "-out").textContent =
        data.config[key] +
        " " +
        (id === "speed" ? "m/s" : id === "gas" ? "kg" : "m");
    }
    $("mpc").checked = data.config.use_mpc;
    $("landing-x").min = maneuver === "barrel-roll" ? 50 : -28;
    $("landing-x").max = maneuver === "barrel-roll" ? 64 : -12;
    $("landing-x").value =
      data.config.landing_x ?? CAR_LAYOUT[maneuver].landing;
    $("landing-x-out").textContent = $("landing-x").value + " m";
    $("require-roll").checked = data.config.require_roll !== false;
    setRun(data);
  } catch (error) {
    setRun(previous);
    showError(error.message);
  } finally {
    loadingManeuver = false;
    setLiveControls(false);
  }
}
$("maneuver-ramp").onclick = () => switchManeuver("c-ramp");
$("maneuver-barrel").onclick = () => switchManeuver("barrel-roll");
