import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { initSolver } from "../src/compute/conic.js";
import { Scvx } from "../src/compute/scvx.js";
import { sampleRocketPhasePlan } from "../src/compute/rocket-planner.js";

await initSolver(
  await readFile(
    new URL(
      "../src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
      import.meta.url,
    ),
  ),
);

test("indexed SCvx dynamics and actuator constraints agree throughout a rollout", () => {
  const solver = new Scvx({
    nx: 7,
    nu: 1,
    n: 3,
    scaleX: [100, 1, 1, 1, 1, 1, 1],
    scaleU: [1],
  });
  const initial = [100, 0, 0, 1, 0, 0, 0],
    goal = initial.slice();
  goal[1] = 3;
  const model = {
    step(x, u, _, k) {
      const next = x.slice();
      next[1] += (k + 1) * u[0];
      return next;
    },
    limit(u, k) {
      return [k === 1 ? 0 : Math.max(0, Math.min(1, u[0]))];
    },
    feasible(u, k) {
      return u[0] >= -1e-6 && u[0] <= (k === 1 ? 0 : 1) + 1e-6;
    },
    constraints(qp, i, k) {
      qp.bound(i, 0, k === 1 ? 0 : 1);
    },
  };
  const out = solver.solve(
    initial,
    Array.from({ length: 4 }, () => goal),
    model,
    {
      dt: 1,
      weights: Array(7).fill(0),
      terminalWeights: [0, 100, 0, 0, 0, 0, 0],
      seed: () => [0.5],
      trust: 4,
      includeStates: true,
      iterations: 5,
    },
  );
  assert.equal(out.snapshot.success, true);
  assert.ok(Math.abs(out.states.at(-1)[1] - 3) < 0.001);
  assert.ok(Math.abs(out.snapshot.control_plan[1][0]) < 1e-7);
});

test("MPC phase extension retains a lit engine at a planned coast boundary", () => {
  const q = [1, 0, 0, 0],
    force = [0, 0, 1500000];
  const plan = {
    times: [0, 1, 2],
    states: [
      [60000, 0, 0, 1000, 0, 0, -100],
      [59457, 0, 0, 908, 0, 0, -84],
      [59457, 0, 0, 819, 0, 0, -94],
    ],
    controls: [force, [0, 0, 0]],
    segments: [
      {
        start: 0,
        dt: 1,
        phase: "ENTRY",
        engines: 3,
        attached: false,
        q,
        force,
      },
      {
        start: 1,
        dt: 1,
        phase: "DESCENT_COAST",
        engines: 0,
        attached: false,
        q,
        force: [0, 0, 0],
      },
    ],
  };
  const sample = sampleRocketPhasePlan(plan, 1.5, "ENTRY", 3);
  assert.deepEqual(sample.force, force);
  assert.equal(sample.phase, "ENTRY");
  assert.equal(sample.engines, 3);
  assert.ok(
    sample.state[6] > -84,
    "continues powered deceleration during the local horizon",
  );
  assert.ok(sample.state[0] < plan.states[1][0]);
});
