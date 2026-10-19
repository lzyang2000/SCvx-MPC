import { properties } from "./compute/rocket-dynamics.js";
import { add, rotate } from "./compute/math.js";

// The optimizer predicts the changing center of mass, while recorded poses
// locate the model's body origin. Compare the same point in both traces.
export function rocketCenterOfMass(frame) {
  const attached = frame.attached ? 111500 : 0;
  const mass = frame.mass ?? 25600 + frame.fuel + attached;
  return add(frame.p, rotate(frame.q, [0, 0, properties(mass, attached).z]));
}

export function rocketMpcAtTime(run, frame, time) {
  const solve = run.summary.mpc_solves.findLast(
    (s) => s.t <= time && (frame.mpc_id == null || s.id <= frame.mpc_id),
  );
  const state = !solve
    ? "waiting"
    : !solve.success
      ? "rejected"
      : frame.phase === "CONTACT"
        ? "handoff"
        : "active";
  return {
    solve,
    state,
    age: solve ? Math.max(0, time - solve.t) : 0,
    actual: solve
      ? [
          solve.origin ?? solve.prediction[0],
          ...run.frames
            .filter(
              (f) =>
                f.t >= solve.t && f.t <= time && f.attached === frame.attached,
            )
            .map(rocketCenterOfMass),
        ]
      : [],
  };
}

function plotData(predicted, actual) {
  const all = [...predicted, ...actual];
  const xmin = Math.min(...all.map((p) => p[0]));
  const xmax = Math.max(...all.map((p) => p[0]));
  const zmin = Math.min(...all.map((p) => p[2]));
  const zmax = Math.max(...all.map((p) => p[2]));
  const xspan = Math.max(0.5, xmax - xmin),
    zspan = Math.max(0.5, zmax - zmin);
  const xlo = xmin - 0.05 * xspan,
    xhi = xmin + 1.05 * xspan;
  const zlo = zmin - 0.05 * zspan,
    zhi = zmin + 1.05 * zspan;
  const xy = (p) => [
    48 + (254 * (p[0] - xlo)) / (xhi - xlo),
    108 - (90 * (p[2] - zlo)) / (zhi - zlo),
  ];
  const path = (ps) =>
    ps
      .map(
        (p, i) =>
          `${i ? "L" : "M"} ${xy(p)
            .map((n) => n.toFixed(2))
            .join(" ")}`,
      )
      .join(" ");
  const label = (n, span) => n.toFixed(span < 1 ? 2 : 1);
  return {
    predicted: path(predicted),
    actual: path(actual),
    now: actual.length ? xy(actual.at(-1)) : null,
    labels: [
      label(xlo, xspan),
      label((xlo + xhi) / 2, xspan),
      label(xhi, xspan),
      label(zlo, zspan),
      label(zhi, zspan),
    ],
  };
}

export function createRocketInspector(root = document) {
  const $ = (id) => root.getElementById(id);
  return function update(run, frame, time, live) {
    const view = rocketMpcAtTime(run, frame, time),
      s = view.solve;
    root.querySelector(".mpc-inspector").dataset.state = view.state;
    $("transport-badge").textContent = live ? "LIVE / WASM" : "REPLAY / WASM";
    $("transport-badge").classList.toggle("live", live);
    $("mpc-state").textContent = !s
      ? "Waiting for first plan"
      : !s.success
        ? "Plan rejected; run stopped"
        : view.state === "handoff"
          ? "Foot contact · engines off"
          : frame.landing_mode === "direct-landing"
            ? "MPC braking directly to touchdown"
            : frame.landing_mode === "slow-descent"
              ? "MPC lowering the booster gently"
              : frame.landing_hold
                ? frame.landing_mode === "hover"
                  ? "MPC hovering above the pad"
                  : "MPC approaching the hover point"
                : frame.phase === "MECO"
                  ? "Main engines off before separation"
                  : frame.phase === "SEPARATION"
                    ? "MPC turning the booster · engines off"
                    : "MPC controlling " +
                      frame.phase.toLowerCase().replaceAll("_", " ");
    $("mpc-explanation").textContent = !s
      ? "MPC controls booster thrust, gimbal and RCS from liftoff to landing."
      : !s.success
        ? "The red path shows the rejected plan. A plan that fails the acceptance checks stops the run."
        : view.state === "handoff"
          ? "The booster has landed with its engines off. The last plan remains visible."
          : frame.landing_mode === "direct-landing"
            ? "Minimum engine thrust exceeds the hover margin at this mass. MPC keeps braking toward foot contact without a hover."
            : frame.landing_mode === "slow-descent"
              ? "After the brief hover, MPC lowers the booster at up to 0.7 m/s and slows toward 0.2 m/s before foot contact."
              : frame.landing_hold
                ? "MPC settles the booster with its feet 2 m above the pad, holds for one second, then lowers it to the pad. The center engine stays lit until contact."
                : frame.phase === "MECO"
                  ? "The main engines are off. The stages coast together until the separation impulse. MPC uses RCS to hold their orientation."
                  : frame.phase === "SEPARATION"
                    ? "MPC turns the booster with RCS while its engines stay off. The upper stage ignites once clear. The booster waits for enough clearance and alignment for return before relighting."
                    : frame.phase === "BOOSTBACK"
                      ? "MPC controls the boostback burn to return toward the launch pad. Engines shut down before the entry burn."
                      : frame.phase === "DESCENT_COAST"
                        ? "The entry burn is complete. Engines stay off while MPC holds attitude with RCS until landing ignition."
                        : `MPC predicts motion and orientation ${s.horizon_seconds.toFixed(1)} seconds ahead. It applies the first thrust and RCS command, then replans. The chart tracks the center of mass.`;
    $("mpc-solve").textContent = s
      ? `#${s.id} / ${s.iterations} iterations`
      : "—";
    $("mpc-horizon").textContent = s
      ? `${s.prediction.length - 1} × ${s.prediction_dt.toFixed(2)} s`
      : "—";
    $("mpc-latency").textContent = s
      ? `${(s.seconds * 1000).toFixed(0)} ms`
      : "—";
    $("mpc-age").textContent = s ? `${view.age.toFixed(2)} s` : "—";
    const request = s?.control_plan?.[0]?.slice(0, 3);
    $("mpc-force").textContent = s
      ? `${request ? (Math.hypot(...request) / 1e6).toFixed(2) : "—"} / ${(Math.hypot(...frame.control.slice(0, 3)) / 1e6).toFixed(2)} MN`
      : "—";
    $("mpc-defect").textContent =
      s?.defect == null ? "—" : s.defect.toExponential(2);
    $("mpc-solver-status").textContent = s
      ? `${s.status} · t=${s.t.toFixed(2)} s · virtual control ${s.virtual_control?.toExponential(2) ?? "—"} · ${s.model}`
      : "Booster flight predictions";
    const next = run.summary.mpc_solves.find(
      (p) => p.t > time + 0.001 && p.t <= run.frames.at(-1).t,
    );
    $("next-solve").disabled = live || !next;
    $("inspect-mpc").disabled = live || !s;
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
      return null;
    }
    const plot = plotData(s.prediction, view.actual);
    for (const [i, id] of [
      "mpc-x-min",
      "mpc-x-mid",
      "mpc-x-max",
      "mpc-z-min",
      "mpc-z-max",
    ].entries())
      $(id).textContent = plot.labels[i];
    $("mpc-plot-predicted").setAttribute("d", plot.predicted);
    $("mpc-plot-predicted").setAttribute(
      "stroke",
      s.success ? "#4da3ff" : "#ff7b86",
    );
    $("mpc-plot-actual").setAttribute("d", plot.actual);
    $("mpc-plot-now").setAttribute(
      "visibility",
      plot.now ? "visible" : "hidden",
    );
    if (plot.now) {
      $("mpc-plot-now").setAttribute("cx", plot.now[0]);
      $("mpc-plot-now").setAttribute("cy", plot.now[1]);
    }
    return s;
  };
}
