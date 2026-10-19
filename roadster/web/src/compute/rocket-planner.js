import { Scvx } from "./scvx.js";
import { ROCKET, aerodynamicForce } from "./rocket-dynamics.js";
import { norm, dot, rotate, rk4, clamp, qblend } from "./math.js";

const SX = [550000, 10000, 10000, 10000, 2000, 2000, 2000];
const SU = Array(3).fill(8e6);
const END = [0, 100000, 100000, 100000, 2000, 2000, 2000];
const pause = () => new Promise((resolve) => setTimeout(resolve, 0));

// The schedule is discrete. Continuous position, velocity, mass and world force
// are optimized jointly across all phases; seed positions are not tracking goals.
function planningModel(segments, integrationStep = 0.2) {
  const bounds = (k) => {
    const max = segments[k].engines * ROCKET.engineMax;
    return { min: max * ROCKET.minThrottle, max };
  };
  return {
    step(x, force, _, k) {
      const s = segments[k],
        steps = Math.max(1, Math.ceil(Math.abs(s.dt) / integrationStep));
      const dynamics = (state, f) => {
        const mass = state[0] + (s.attached ? 111500 : 0);
        const flight = [mass, ...state.slice(1), ...s.q, 0, 0, 0];
        const drag = aerodynamicForce(flight, s.attached);
        return [
          -norm(f) / ROCKET.isp,
          ...state.slice(4),
          (f[0] + drag[0]) / mass,
          (f[1] + drag[1]) / mass,
          (f[2] + drag[2]) / mass - 9.81,
        ];
      };
      let next = x;
      for (let j = 0; j < steps; j++)
        next = rk4(next, force, s.dt / steps, dynamics);
      return next;
    },
    limit(force, k) {
      const { min, max } = bounds(k);
      if (!max) return [0, 0, 0];
      const axis = segments[k].axis;
      let f = force.slice();
      const axial = dot(f, axis);
      if (axial < min) f = f.map((v, i) => v + (min - axial) * axis[i]);
      if (norm(f) > max) f = f.map((v) => (v * max) / norm(f));
      return f;
    },
    feasible(force, k) {
      const { min, max } = bounds(k);
      return (
        force.every(Number.isFinite) &&
        norm(force) <= max + 2 &&
        dot(force, segments[k].axis) >= min - 2
      );
    },
    constraints(qp, i, k, su) {
      const { min, max } = bounds(k),
        axis = segments[k].axis;
      if (!max) {
        for (let j = 0; j < 3; j++) qp.bound(i + j, 0, 0);
        return;
      }
      qp.le(
        axis.map((v, j) => [i + j, (-v * su[j]) / su[0]]),
        -min / su[0],
      );
      qp.cone(
        [[], ...su.map((v, j) => [[i + j, -v / su[0]]])],
        [max / su[0], 0, 0, 0],
      );
      // Keep force directions in the attitude corridor that the 6-DoF MPC can
      // follow. This is a schedule/attitude constraint, not a position reference.
      const margin = segments[k].phase === "ASCENT" ? 0.04 : 0.08;
      const seed = segments[k].force;
      for (let j = 0; j < 3; j++)
        qp.bound(
          i + j,
          (seed[j] - margin * max) / su[j],
          (seed[j] + margin * max) / su[j],
        );
    },
  };
}

export async function prepareRocketPlan(mission, status = () => {}) {
  const began = performance.now();
  status("Building a feasible rocket mission…");
  const seed = new mission.constructor(mission.mj, mission.xml, {
    separationAltitude: mission.separationAltitude,
    initialFuel: mission.initialFuel,
  });
  const segments = [],
    initial = [
      ROCKET.dry + seed.fuel,
      ...Array.from(seed.data.qpos.slice(0, 3)),
      ...Array.from(seed.data.qvel.slice(0, 3)),
    ];
  let block = null,
    lastYield = performance.now();
  const finish = () => {
    if (!block || block.dt < 1e-8) return;
    block.force = block.impulse.map((v) => v / block.dt);
    block.nominalForce = block.force.slice();
    delete block.impulse;
    segments.push(block);
    block = null;
  };
  let seedSummary;
  try {
    for (let i = 0; i < 150000; i++) {
      const q = Array.from(seed.data.qpos.slice(3, 7));
      const t = seed.data.time;
      const out = seed.step();
      if (seed.phase === "CONTACT" || out.done) {
        finish();
        break;
      }
      const frame = seed.frame();
      const interval =
        frame.phase === "LANDING"
          ? seed.landingHeight() < 100
            ? 0.1
            : 0.6
          : ["SEPARATION", "BOOSTBACK", "ENTRY"].includes(frame.phase)
            ? 0.6
            : ["ASCENT", "MECO", "DESCENT_COAST"].includes(frame.phase)
              ? 2
              : 10;
      if (
        block &&
        (block.phase !== frame.phase ||
          block.engines !== frame.engines ||
          block.dt >= interval - 1e-8)
      )
        finish();
      if (!block)
        block = {
          start: t,
          dt: 0,
          phase: frame.phase,
          engines: frame.engines,
          attached: frame.attached,
          q,
          axis: rotate(q, [0, 0, 1]),
          impulse: [0, 0, 0],
        };
      const ds = seed.data.time - t,
        force = rotate(q, frame.control.slice(0, 3));
      block.impulse = block.impulse.map((v, j) => v + force[j] * ds);
      block.dt += ds;
      if (performance.now() - lastYield > 20) {
        await pause();
        lastYield = performance.now();
      }
    }
    seedSummary = seed.summary();
    // Contact is enough to seed the continuous optimizer; settling is validated
    // separately by the actual mission, not inferred from this approximation.
    if (seed.phase !== "CONTACT" || seed.failure)
      throw Error(
        `Could not initialize rocket mission: ${seedSummary.failure ?? seed.failure}`,
      );
  } finally {
    seed.dispose();
  }
  const plan = optimizeRocketPlan(mission, segments, initial, status, began);
  mission.flightPlanHistory = [plan];
  return plan;
}

export async function replanRocketPlan(mission, status = () => {}) {
  const previous = mission.flightPlan;
  const count = mission.phase === "LANDING" ? mission.landingEngineCount : null;
  const first = previous.segments.findIndex(
    (s) => s.phase === mission.phase && (count === null || s.engines === count),
  );
  if (first < 0)
    throw Error("The mission schedule has no remaining phase for this state");
  const start = mission.data.time,
    offset = start - previous.segments[first].start;
  const segments = previous.segments.slice(first).map((s, i) => ({
    ...s,
    start: s.start + offset,
    nominalForce: s.nominalForce ?? s.force,
    force: previous.controls[first + i].slice(),
  }));
  const initial = [
    ROCKET.dry + mission.fuel,
    ...Array.from(mission.data.qpos.slice(0, 3)),
    ...Array.from(mission.data.qvel.slice(0, 3)),
  ];
  status("Replanning the remaining rocket trajectory…");
  await pause();
  const plan = optimizeRocketPlan(
    mission,
    segments,
    initial,
    status,
    performance.now(),
  );
  mission.flightPlanHistory.push(plan);
  mission.pendingReplan = null;
  return plan;
}

function optimizeRocketPlan(mission, segments, initial, status, began) {
  const model = planningModel(segments),
    controls = segments.map((s, k) => model.limit(s.force, k));
  const goal = [initial[0], ROCKET.padX, 0, ROCKET.landedZ, 0, 0, -0.2];
  const refs = Array.from({ length: segments.length + 1 }, () => goal.slice());
  const solver = new Scvx({
    nx: 7,
    nu: 3,
    n: segments.length,
    scaleX: SX,
    scaleU: SU,
  });
  const sep = segments.findIndex((s) => s.phase === "SEPARATION");
  const merit = (states) => {
    const end = states.at(-1);
    let cost = end.reduce(
      (sum, v, j) => sum + ((END[j] * (v - goal[j])) / SX[j]) ** 2,
      0,
    );
    if (sep >= 0)
      cost +=
        1e6 *
        ((states[sep][3] - ROCKET.landedZ - mission.separationAltitude) /
          SX[3]) **
          2;
    cost += (0.5 * (initial[0] - end[0])) / SX[0];
    return cost;
  };
  status("Optimizing the complete rocket trajectory…");
  const result = solver.solve(initial, refs, model, {
    dt: 1,
    weights: Array(7).fill(0),
    terminalWeights: END,
    seed: (k) => controls[k],
    iterations: 30,
    minimumIterations: 12,
    trust: segments[0].start > 0 ? 0.5 : 0.12,
    ground: ROCKET.landedZ - 0.5,
    dry: ROCKET.dry + 500,
    includeStates: true,
    merit,
    extraConstraints(qp, { nx, sx }) {
      qp.q[segments.length * nx] -= 0.5;
      if (sep >= 0)
        qp.bound(
          sep * nx + 3,
          (mission.separationAltitude + ROCKET.landedZ - 25) / sx[3],
          (mission.separationAltitude + ROCKET.landedZ + 25) / sx[3],
        );
    },
  });
  const end = result.states.at(-1),
    error = norm(end.slice(1, 4).map((v, i) => v - goal[i + 1]));
  const speed = norm(end.slice(4));
  if (
    !result.snapshot.success ||
    error > 2 ||
    speed > 0.7 ||
    end[0] < ROCKET.dry + 500
  )
    throw Error(
      `No validated rocket goal plan (${result.snapshot.status}, terminal error ${error.toFixed(2)} m, speed ${speed.toFixed(2)} m/s)`,
    );
  const fine = planningModel(segments, 0.05);
  let verified = initial.slice(),
    validationError = 0;
  for (let k = 0; k < segments.length; k++) {
    verified = fine.step(verified, result.snapshot.control_plan[k], 1, k);
    validationError = Math.max(
      validationError,
      norm(verified.slice(1, 4).map((v, j) => v - result.states[k + 1][j + 1])),
    );
    if (
      verified[0] < ROCKET.dry + 500 - 0.01 ||
      verified[3] < ROCKET.landedZ - 0.5 ||
      verified.some((v) => !Number.isFinite(v))
    )
      throw Error("Rocket plan failed the independent integration check");
  }
  if (
    validationError > 0.1 ||
    norm(verified.slice(1, 4).map((v, j) => v - goal[j + 1])) > 2 ||
    norm(verified.slice(4)) > 0.7
  )
    throw Error("Rocket plan failed the refined terminal check");
  const plan = {
    states: result.states,
    controls: result.snapshot.control_plan,
    segments,
    times: [segments[0].start, ...segments.map((s) => s.start + s.dt)],
    goal: {
      separation_altitude: mission.separationAltitude,
      landing_position: goal.slice(1, 4),
      landing_velocity: goal.slice(4),
      minimum_propellant: 500,
      upright: true,
    },
    duration: segments.at(-1).start + segments.at(-1).dt,
    seconds: (performance.now() - began) / 1000,
    optimization_seconds: result.snapshot.seconds,
    iterations: result.snapshot.iterations,
    defect: result.snapshot.defect,
    terminal_error: error,
    integration_error: validationError,
    terminal_speed: speed,
    costs: result.snapshot.costs,
    prediction: result.states.map((x) => x.slice(1, 4)),
    start: segments[0].start,
    model: "3-DoF mission SCvx / 6-DoF tracking MPC",
  };
  mission.flightPlan = plan;
  return plan;
}

export function sampleRocketPlan(plan, time) {
  let lo = 0,
    hi = plan.segments.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (plan.times[mid + 1] <= time) lo = mid + 1;
    else hi = mid;
  }
  const k = lo,
    s = plan.segments[k],
    f = clamp((time - s.start) / s.dt, 0, 1);
  return {
    state: plan.states[k].map((v, j) => v + f * (plan.states[k + 1][j] - v)),
    force: plan.controls[k],
    nominalForce: s.nominalForce ?? s.force,
    q: qblend(
      s.q,
      plan.segments[Math.min(k + 1, plan.segments.length - 1)].q,
      f,
    ),
    phase: s.phase,
    engines: s.engines,
  };
}

// A local MPC solve keeps one actuator mode throughout its short horizon.
// Extend that mode at a boundary rather than asking a lit engine to coast (or
// an unlit one to accelerate) before the discrete transition actually occurs.
export function sampleRocketPhasePlan(plan, time, phase, engines) {
  const indices = [];
  for (let k = 0; k < plan.segments.length; k++)
    if (
      plan.segments[k].phase === phase &&
      plan.segments[k].engines === engines
    )
      indices.push(k);
  if (!indices.length)
    throw Error("No trajectory for the active rocket actuator mode");
  const first = indices[0],
    last = indices.at(-1),
    start = plan.segments[first].start;
  const end = plan.times[last + 1];
  if (time >= start && time < end) return sampleRocketPlan(plan, time);
  const k = time < start ? first : last;
  const state = plan.states[time < start ? first : last + 1];
  const dt = time - (time < start ? start : end);
  const segment = { ...plan.segments[k], dt };
  return {
    state: planningModel([segment]).step(state, plan.controls[k], 1, 0),
    force: plan.controls[k],
    nominalForce: segment.nominalForce ?? segment.force,
    q: segment.q,
    phase,
    engines,
  };
}
