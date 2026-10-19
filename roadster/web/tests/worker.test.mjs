import test from "node:test";
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { spawnSync } from "node:child_process";
import { rotate } from "../src/compute/math.js";
// Build to exercise the exact import and WASM layout shipped to Pages.
const build = spawnSync(process.execPath, ["build.mjs"], {
  cwd: new URL("..", import.meta.url),
  encoding: "utf8",
});
assert.equal(build.status, 0, build.stderr);
function start(demo, config, options = {}) {
  const w = new Worker(new URL("./worker-harness.mjs", import.meta.url));
  w.postMessage({ demo, config, live: false, ...options });
  return w;
}
for (const maneuver of ["c-ramp", "barrel-roll"])
  test(`${maneuver} worker initializes WASM and streams causal solves and frames`, async () => {
    const w = start("car", { maneuver }),
      events = [];
    try {
      const result = await new Promise((resolve, reject) => {
        w.on("error", reject);
        w.on("message", (event) => {
          events.push(event);
          if (event.type === "error") reject(Error(event.data.error));
          if (event.type === "complete") resolve(event.data);
        });
      });
      assert.equal(result.summary.success, true);
      assert.equal(result.config.maneuver, maneuver);
      assert.equal(result.config.physics, "mujoco");
      assert.equal(result.summary.physics, "mujoco");
      assert.match(result.model, /MuJoCo WASM/);
      assert.ok(
        result.frames.every(
          (f) => f.physics === "mujoco" && f.wheel_angles.length === 4,
        ),
      );
      assert.equal(result.nozzles.positions.length, 6);
      assert.ok(result.frames.every((f) => f.jets.length === 6));
      assert.ok(
        result.summary.mpc_solves.every((s) =>
          s.nozzle_plan.every((u) => u.length === 6),
        ),
      );
      if (maneuver === "barrel-roll")
        assert.ok(result.summary.stage_clearance > 0);
      assert.equal(
        events.find((e) => e.type === "start").data.config.radius,
        10,
      );
      const known = new Set();
      for (const e of events) {
        if (e.type === "solve") known.add(e.data.id);
        if (e.type === "frame" && e.data.mpc_id !== null)
          assert.ok(known.has(e.data.mpc_id));
      }
      assert.ok(known.size > 50);
    } finally {
      await w.terminate();
    }
  });
test("terminating a live rocket worker cancels computation and releases it", async () => {
  const w = start("rocket", {}, { live: true, speed: 1 });
  let lastTime = 0;
  try {
    await new Promise((resolve, reject) => {
      w.on("error", reject);
      w.on("message", (e) => {
        if (e.type === "error") reject(Error(e.data.error));
        if (e.type === "frame") {
          lastTime = e.data.t;
          resolve();
        }
      });
    });
    const startTime = performance.now();
    await w.terminate();
    assert.ok(performance.now() - startTime < 2000);
    assert.ok(lastTime < 1);
  } finally {
    await w.terminate();
  }
});

test("car worker publishes a complete goal plan before applying airborne MPC", async () => {
  const w = start("car", {
    maneuver: "barrel-roll",
    goal_planning: true,
    require_roll: false,
    landing_x: 56,
  });
  const events = [];
  try {
    const result = await new Promise((resolve, reject) => {
      w.on("error", reject);
      w.on("message", (event) => {
        events.push(event);
        if (event.type === "error") reject(Error(event.data.error));
        if (event.type === "complete") resolve(event.data);
      });
    });
    const planIndex = events.findIndex((e) => e.type === "plan");
    const firstSolve = events.findIndex((e) => e.type === "solve");
    assert.ok(planIndex > 0 && firstSolve > planIndex);
    assert.equal(result.summary.success, true, JSON.stringify(result.summary));
    assert.equal(result.flight_plan.require_roll, false);
    assert.equal(result.landing_target[0], 56);
    assert.ok(result.summary.landing_error < 0.6);
    assert.ok(Math.abs(result.summary.maneuver_roll_degrees) < 2);
  } finally {
    await w.terminate();
  }
});

test("rocket worker plans each mission and replans the remaining flight before MPC", async () => {
  const w = start("rocket", { separationAltitude: 75000 });
  const events = [];
  try {
    const result = await new Promise((resolve, reject) => {
      w.on("error", reject);
      w.on("message", (event) => {
        events.push(event);
        if (event.type === "error") reject(Error(event.data.error));
        if (event.type === "complete") resolve(event.data);
      });
    });
    assert.ok(
      events.findIndex((e) => e.type === "plan") <
        events.findIndex((e) => e.type === "solve"),
    );
    assert.equal(result.summary.success, true, JSON.stringify(result.summary));
    assert.equal(result.summary.post_separation_ignitions, 3);
    assert.equal(result.summary.mpc_rejected, 0);
    assert.ok(Math.abs(result.summary.separation.altitude - 75000) < 50);
    assert.ok(result.summary.touchdown.vertical_speed < 0.3);
    assert.ok(result.summary.touchdown.horizontal_speed < 0.6);
    assert.ok(result.summary.touchdown.tilt_degrees < 1);
    assert.ok(result.summary.pad_error < 8);
    assert.ok(result.summary.separation.velocity[0] < 0);
    let towerChecks = 0;
    for (const f of result.frames) {
      if (!["LANDING", "CONTACT"].includes(f.phase)) continue;
      // Conservative oriented box enclosing the booster and fully deployed feet.
      // The rendered tower's platforms span x=[-12.75,-9.25], z=[0,72].
      const corners = [];
      for (const x of [-9.22, 9.22])
        for (const y of [-9.22, 9.22])
          for (const z of [-20.6, 21])
            corners.push(rotate(f.q, [x, y, z]).map((v, i) => v + f.p[i]));
      if (Math.min(...corners.map((p) => p[2])) > 72) continue;
      towerChecks++;
      assert.ok(
        Math.min(...corners.map((p) => p[0])) > -9.25,
        `booster and deployed feet must clear the launch tower at t=${f.t}`,
      );
    }
    assert.ok(towerChecks > 10);

    assert.equal(result.flight_plans.length, 5);
    assert.equal(events.filter((e) => e.type === "plan").length, 5);
    assert.equal(result.summary.planning.replans, 4);
    assert.equal(result.flight_plan.start, 0);
    for (const plan of result.flight_plans) {
      assert.match(plan.model, /3-DoF.*6-DoF/);
      assert.equal(plan.states.length, plan.controls.length + 1);
      assert.ok(plan.terminal_error < 2);
      assert.ok(plan.terminal_speed < 0.7);
      assert.ok(plan.integration_error < 0.1);
      assert.ok(plan.costs.at(-1) <= plan.costs[0]);
      for (let k = 0; k < plan.controls.length; k++) {
        const max = plan.segments[k].engines * 845000;
        assert.ok(Math.hypot(...plan.controls[k]) <= max + 2);
        assert.ok(plan.states[k][0] > 25600);
      }
    }
    assert.ok(
      result.summary.mpc_solves.some(
        (s) =>
          s.phase === "LANDING" &&
          s.engine_count === 3 &&
          s.prediction.length === 33,
      ),
    );
  } finally {
    await w.terminate();
  }
});
