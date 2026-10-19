import { Scvx } from "./scvx.js";
import { carModel } from "./car-mission.js";
import { STAGE_VEHICLES, stageClearance } from "../car-stage.js";
import { dot, norm, rotate, unit } from "./math.js";

const DT = 0.12;
const SCALE = [1678, 30, 5, 20, 12, 5, 6, 1, 1, 1, 1, 3, 3, 3];
const END_WEIGHTS = [
  0, 120, 100, 200, 60, 60, 80, 120, 120, 120, 120, 30, 30, 30,
];
const bodyPoints = [];
for (const x of [-2.1, 2.1])
  for (const y of [-0.9, 0.9])
    for (const z of [-0.25, 0.6]) bodyPoints.push([x, y, z]);

for (const x of [-1.4, 1.4])
  for (const y of [-0.92, 0.92]) bodyPoints.push([x, y, -0.62]);
const lowest = (x) =>
  Math.min(...bodyPoints.map((p) => x[3] + rotate(x.slice(7, 11), p)[2]));
function heightConstraint(qp, k, x, height, nx, sx) {
  const q = x.slice(7, 11);
  for (const p of bodyPoints) {
    const h = rotate(q, p)[2];
    const dh = q.map((_, i) => {
      const plus = q.slice();
      plus[i] += 1e-5;
      return (rotate(unit(plus), p)[2] - h) / 1e-5;
    });
    qp.le(
      [[k * nx + 3, -1], ...dh.map((v, i) => [k * nx + 7 + i, -v / sx[3]])],
      (h - dot(dh, q) - height) / sx[3],
    );
  }
}
export function planClearance(x) {
  return stageClearance(
    bodyPoints.map((p) =>
      rotate(x.slice(7, 11), p).map((v, i) => v + x[i + 1]),
    ),
  ).clearance;
}

// The initial guess may use guidance. Only the endpoint and sparse maneuver
// constraints enter the full-flight objective; intermediate seed positions do not.
export async function prepareCarPlan(mission, status = () => {}) {
  if (!mission.config.goal_planning || !mission.config.use_mpc) return null;
  const began = performance.now();
  const seed =
    mission.createPlanningSeed?.() ??
    new mission.constructor({
      ...mission.config,
      goal_planning: false,
    });
  status("Building a feasible initial guess…");
  try {
    while (seed.release === null && !seed.failure && seed.t < 20) seed.step();
  } finally {
    // Only the ground seed needs MuJoCo data; flight guidance uses copied state.
    seed.dispose?.();
  }
  if (seed.release === null) throw Error("The car could not reach takeoff");
  const state = seed.x.slice(),
    start = seed.t;
  const model = carModel(seed.predictionConfig?.() ?? seed.config, state[0]);
  const controls = [],
    states = [state];
  for (let k = 0; k < 95; k++) {
    seed.plan();
    if (seed.failure)
      throw Error("Could not initialize a flight to this target");
    controls.push(seed.u.slice());
    seed.x = model.step(seed.x, seed.u, DT);
    seed.t += DT;
    states.push(seed.x.slice());
    if (k > 25 && seed.x[3] < 0.85 && seed.x[6] < 0) break;
    if (k % 10 === 0) await new Promise((resolve) => setTimeout(resolve, 0));
  }
  if (states.at(-1)[3] > 1.5)
    throw Error("This target needs a longer flight than the planner supports");
  const n = controls.length;
  const solver = new Scvx({
    nu: 6,
    n,
    scaleX: SCALE,
    scaleU: Array(6).fill(seed.config.nozzle_n),
  });
  const goal = states.at(-1).slice();
  goal.splice(1, 3, mission.target, 0, 0.8);
  goal.splice(4, 3, 0, 0, -0.18);
  goal.splice(
    7,
    4,
    ...(seed.barrel
      ? seed.config.require_roll
        ? [-1, 0, 0, 0]
        : [1, 0, 0, 0]
      : [0, 0, 0, 1]),
  );
  goal.splice(11, 3, 0, 0, 0);
  const refs = Array.from({ length: n + 1 }, () => goal.slice());
  // A signed quaternion branch plus three orientation gates specifies the roll
  // without prescribing the intervening position or altitude trajectory.
  const gates =
    seed.barrel && seed.config.require_roll
      ? [2.3, 2.8, 3.3].map((t) => {
          const k = Math.min(
            n - 1,
            Math.round((t + seed.release - start) / DT),
          );
          return { k, q: states[k].slice(7, 11) };
        })
      : [];
  const merit = (xs, us) => {
    const last = xs.at(-1);
    let cost = last.reduce(
      (sum, v, i) => sum + ((END_WEIGHTS[i] * (v - goal[i])) / SCALE[i]) ** 2,
      0,
    );
    for (const u of us)
      for (const thrust of u) cost += (0.015 * thrust) / seed.config.nozzle_n;
    for (const { k, q } of gates)
      cost += 1e5 * Math.max(0, 0.97 - dot(xs[k].slice(7, 11), q)) ** 2;
    for (const x of xs) {
      cost += 1e5 * Math.max(0, 0.01 - lowest(x)) ** 2;
      if (seed.barrel) cost += 1e5 * Math.max(0, 0.2 - planClearance(x)) ** 2;
    }
    return cost;
  };
  status("Optimizing the complete flight…");
  await new Promise((resolve) => setTimeout(resolve, 0));
  const out = solver.solve(state, refs, model, {
    dt: DT,
    weights: Array(14).fill(0),
    terminalWeights: END_WEIGHTS,
    seed: (k) => controls[k],
    iterations: 35,
    minimumIterations: 35,
    trust: 0.12,
    dry: 1428,
    ground: 0.3,
    controlLinear: 0.015,
    maxDefect: 0.03,
    includeStates: true,
    merit,
    extraConstraints(qp, { nx, sx, xs }) {
      for (const { k, q } of gates)
        qp.le(
          q.map((v, i) => [k * nx + 7 + i, -v]),
          -0.97,
        );
      for (let k = 1; k <= n; k++) {
        heightConstraint(qp, k, xs[k], 0.01, nx, sx);
        if (!seed.barrel) continue;
        for (const v of STAGE_VEHICLES) {
          const left = v.x - v.size[0] / 2,
            right = v.x + v.size[0] / 2;
          const gaps = [
            left - xs[k][1] - 2.5,
            xs[k][1] - 2.5 - right,
            lowest(xs[k]) - v.size[2] - 0.2,
          ];
          const side = gaps.indexOf(Math.max(...gaps));
          if (side === 0) qp.le([[k * nx + 1, 1]], (left - 2.5) / sx[1]);
          else if (side === 1)
            qp.le([[k * nx + 1, -1]], -(right + 2.5) / sx[1]);
          else heightConstraint(qp, k, xs[k], v.size[2] + 0.2, nx, sx);
        }
      }
    },
  });
  const end = out.states.at(-1);
  const error = norm(end.slice(1, 4).map((v, i) => v - goal[i + 1]));
  const tilt =
    (Math.acos(
      Math.min(1, Math.max(-1, rotate(end.slice(7, 11), [0, 0, 1])[2])),
    ) *
      180) /
    Math.PI;
  if (
    !out.snapshot.success ||
    error > 0.8 ||
    norm(end.slice(4, 7)) > 0.6 ||
    tilt > 4
  )
    throw Error(
      `No validated full-flight plan for this target (${out.snapshot.status}, position error ${error.toFixed(2)} m)`,
    );
  if (gates.some(({ k, q }) => dot(out.states[k].slice(7, 11), q) < 0.965))
    throw Error("The planned trajectory did not satisfy the roll gates");
  if (
    2 *
      Math.acos(
        Math.min(1, Math.abs(dot(end.slice(7, 11), goal.slice(7, 11)))),
      ) >
    (5 * Math.PI) / 180
  )
    throw Error("The planned landing heading is outside tolerance");
  // Independently check the swept trajectory at physics-scale increments.
  let x = state,
    roll = 0,
    previousRoll = 0;
  for (const u of out.snapshot.control_plan)
    for (let j = 0; j < 12; j++) {
      x = model.step(x, u, DT / 12);
      if (lowest(x) < -0.005)
        throw Error(
          "Planned flight contacts the ground before its landing goal",
        );
      if (seed.barrel) {
        const q = x.slice(7, 11),
          ry = rotate(q, [0, 1, 0]),
          rz = rotate(q, [0, 0, 1]);
        const angle = Math.atan2(ry[2], rz[2]);
        roll += Math.atan2(
          Math.sin(angle - previousRoll),
          Math.cos(angle - previousRoll),
        );
        previousRoll = angle;
      }
      if (seed.barrel && planClearance(x) < 0.15)
        throw Error("Planned flight is too close to a display vehicle");
      if (x[0] < 1428 || x.some((v) => !Number.isFinite(v)))
        throw Error("Planned flight exceeds its air budget");
    }
  if (
    seed.barrel &&
    Math.abs(roll - (seed.config.require_roll ? 2 * Math.PI : 0)) >
      (10 * Math.PI) / 180
  )
    throw Error("The planned flight did not complete the selected maneuver");
  const plan = {
    states: out.states,
    controls: out.snapshot.control_plan,
    dt: DT,
    start,
    duration: n * DT,
    target: goal,
    require_roll: seed.barrel && seed.config.require_roll,
    seconds: (performance.now() - began) / 1000,
    optimization_seconds: out.snapshot.seconds,
    iterations: out.snapshot.iterations,
    defect: out.snapshot.defect,
    terminal_error: error,
    costs: out.snapshot.costs,
    prediction: out.states.map((x) => x.slice(1, 4)),
  };
  mission.flightPlan = plan;
  return plan;
}
