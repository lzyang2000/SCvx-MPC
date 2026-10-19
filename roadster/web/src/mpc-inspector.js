import * as THREE from "three";
import { mpcAtTime } from "./mpc-data.js";

const $ = (id) => document.getElementById(id);
const point = (p) => new THREE.Vector3(p[0], p[2], -p[1]);
const BLUE = 0x4da3ff,
  RED = 0xff7b86;

export function createMpcInspector(scene) {
  const group = new THREE.Group();
  scene.add(group);
  const path = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineDashedMaterial({
      color: BLUE,
      dashSize: 0.14,
      gapSize: 0.08,
      transparent: true,
    }),
  );
  group.add(path);
  const nodes = [];
  const geometry = new THREE.SphereGeometry(0.075, 10, 8);
  for (let i = 0; i < 13; i++) {
    const node = new THREE.Mesh(
      geometry,
      new THREE.MeshBasicMaterial({ color: BLUE, transparent: true }),
    );
    group.add(node);
    nodes.push(node);
  }
  const target = new THREE.Group();
  group.add(target);
  for (const direction of [
    [0.3, 0, 0],
    [0, 0.3, 0],
    [0, 0, 0.3],
  ]) {
    const d = new THREE.Vector3(...direction);
    target.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([d.clone().negate(), d]),
        new THREE.LineBasicMaterial({ color: BLUE }),
      ),
    );
  }
  let lastSolve = null;

  return function update(run, frame, time) {
    const view = mpcAtTime(run, frame, time),
      s = view.solve;
    const barrel = run.config.maneuver === "barrel-roll";
    const labels = {
      waiting: "Waiting for first plan",
      disabled: "MPC disabled",
      rejected: "Plan rejected; run stopped",
      active: "MPC controlling flight",
      handoff: "Wheels down · MPC complete",
    };
    document.querySelector(".mpc-inspector").dataset.state = view.state;
    $("mpc-state").textContent = labels[view.state];
    $("mpc-explanation").textContent = !s
      ? run.config.use_mpc
        ? barrel
          ? "The wheels drive the approach. MPC controls six jets through liftoff, the full roll and descent. The wheels brake after landing."
          : "The wheels drive the ramp, and fan suction keeps the tires in contact. MPC controls all six jets from ramp exit to wheel contact."
        : barrel
          ? "MPC is off. The car cannot lift from flat ground."
          : "MPC is off. The car coasts after ramp release."
      : !s.success
        ? "The solver rejected this plan and stopped the run. The red path stays visible for inspection."
        : view.active
          ? barrel
            ? "MPC predicts motion and orientation 1.2 seconds ahead through the climb, full roll and descent. It chooses the six jet forces. The wheels brake after landing."
            : "MPC predicts motion and orientation 1.2 seconds ahead and chooses all six jet forces. It applies the first command, then replans. The car tilts to brake."
          : "The wheels now support and brake the car. The last flight plan remains visible.";
    $("mpc-solve").textContent = s
      ? `#${s.id} / ${s.iterations} iterations`
      : "—";
    $("mpc-horizon").textContent = s
      ? `${s.prediction.length - 1} × ${s.prediction_dt.toFixed(2)} s`
      : "Phase-specific horizon";
    $("mpc-latency").textContent = s
      ? `${(s.seconds * 1000).toFixed(0)} ms`
      : "—";
    $("mpc-age").textContent = s ? `${view.age.toFixed(2)} s` : "—";
    $("mpc-force").textContent = s
      ? `${s.requested_force ? (Math.hypot(...s.requested_force) / 1000).toFixed(1) : "—"} / ${(Math.hypot(...view.delivered) / 1000).toFixed(1)} kN`
      : "—";
    $("mpc-defect").textContent = s?.defect == null ? "—" : s.defect.toFixed(4);
    $("mpc-solver-status").textContent = s
      ? `${s.status} · t=${s.t.toFixed(2)} s · ${s.model}`
      : "Airborne flight MPC";
    group.visible = $("prediction").checked && !!s;
    if (!s) {
      for (const id of ["mpc-plot-predicted", "mpc-plot-actual"])
        $(id).setAttribute("d", "");
      $("mpc-plot-now").setAttribute("visibility", "hidden");
      for (const id of [
        "mpc-x-min",
        "mpc-x-mid",
        "mpc-x-max",
        "mpc-z-min",
        "mpc-z-max",
      ])
        $(id).textContent = "—";
      lastSolve = null;
      return;
    }
    if (lastSolve !== s) {
      lastSolve = s;
      path.geometry.dispose();
      path.geometry = new THREE.BufferGeometry().setFromPoints(
        s.prediction.map(point),
      );
      path.computeLineDistances();
    }
    const color = s.success ? BLUE : RED;
    path.material.color.setHex(color);
    path.material.opacity = view.active || !s.success ? 1 : 0.45;
    nodes.forEach((node, i) => {
      node.visible = !!s.prediction[i];
      if (!node.visible) return;
      node.position.copy(point(s.prediction[i]));
      node.material.color.setHex(color);
      node.material.opacity = i * s.prediction_dt >= view.age ? 0.95 : 0.22;
      node.scale.setScalar(i === 0 ? 1.35 : 1);
    });
    target.position.copy(point(s.target));
    const actual = view.actual.map((f) => f.p),
      all = [...s.prediction, ...actual];
    const xmin = Math.min(...all.map((p) => p[0])),
      xmax = Math.max(...all.map((p) => p[0]));
    const zmin = Math.min(...all.map((p) => p[2])),
      zmax = Math.max(...all.map((p) => p[2]));
    const xspan = Math.max(0.5, xmax - xmin),
      zspan = Math.max(0.5, zmax - zmin);
    const xlo = xmin - 0.05 * xspan,
      xhi = xmin + 1.05 * xspan;
    const zlo = zmin - 0.05 * zspan,
      zhi = zmin + 1.05 * zspan;
    const label = (value, span) => value.toFixed(span < 1 ? 2 : 1);
    $("mpc-x-min").textContent = label(xlo, xspan);
    $("mpc-x-mid").textContent = label((xlo + xhi) / 2, xspan);
    $("mpc-x-max").textContent = label(xhi, xspan);
    $("mpc-z-min").textContent = label(zlo, zspan);
    $("mpc-z-max").textContent = label(zhi, zspan);
    const xy = (p) => [
      48 + (254 * (p[0] - xlo)) / (xhi - xlo),
      108 - (90 * (p[2] - zlo)) / (zhi - zlo),
    ];
    const svgPath = (points) =>
      points
        .map(
          (p, i) =>
            `${i ? "L" : "M"} ${xy(p)
              .map((n) => n.toFixed(2))
              .join(" ")}`,
        )
        .join(" ");
    $("mpc-plot-predicted").setAttribute("d", svgPath(s.prediction));
    $("mpc-plot-predicted").setAttribute(
      "stroke",
      s.success ? "#4da3ff" : "#ff7b86",
    );
    $("mpc-plot-actual").setAttribute("d", svgPath(actual));
    const now = actual.at(-1);
    $("mpc-plot-now").setAttribute("visibility", now ? "visible" : "hidden");
    if (now) {
      const [x, y] = xy(now);
      $("mpc-plot-now").setAttribute("cx", x);
      $("mpc-plot-now").setAttribute("cy", y);
    }
  };
}
