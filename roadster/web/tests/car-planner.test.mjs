import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { initSolver } from "../src/compute/conic.js";
import load from "../node_modules/@mujoco/mujoco/mujoco.js";
import { MuJoCoCarMission } from "../src/compute/car-mujoco.js";
import { CarMission } from "../src/compute/car-mission.js";
import { prepareCarPlan, planClearance } from "../src/compute/car-planner.js";
import { norm } from "../src/compute/math.js";
await initSolver(
  await readFile(
    new URL(
      "../src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
      import.meta.url,
    ),
  ),
);

const mj = await load();

for (const config of [
  { maneuver: "c-ramp" },
  { maneuver: "c-ramp", landing_x: -16 },
  { maneuver: "c-ramp", landing_x: -12 },
  { maneuver: "barrel-roll" },
  { maneuver: "barrel-roll", require_roll: false },
  { maneuver: "barrel-roll", landing_x: 56 },
  { maneuver: "barrel-roll", landing_x: 64, require_roll: false },
])
  test(`full-flight planning and feedback landing: ${JSON.stringify(config)}`, async () => {
    const mission = new MuJoCoCarMission(mj, {
      ...config,
      goal_planning: true,
    });
    try {
      const status = [];
      const plan = await prepareCarPlan(mission, (message) =>
        status.push(message),
      );
      assert.equal(
        mission.t,
        0,
        "planning does not advance the displayed plant",
      );
      assert.equal(plan.states.length, plan.controls.length + 1);
      assert.ok(plan.duration > 4 && plan.duration < 12);
      assert.ok(plan.defect < 0.03);
      assert.ok(
        plan.costs.at(-1) < plan.costs[0] * 0.5,
        "optimize the endpoint instead of replaying the guide seed",
      );
      for (let i = 1; i < plan.costs.length; i++)
        assert.ok(plan.costs[i] <= plan.costs[i - 1] + 1e-8);
      assert.equal(status.at(-1), "Optimizing the complete flight…");
      if (mission.barrel)
        assert.ok(plan.states.every((x) => planClearance(x) >= 0.15));
      const phases = new Set();
      let ended = false;
      for (let i = 0; i < 2500; i++) {
        const out = mission.step(),
          frame = mission.frame();
        phases.add(frame.phase);
        assert.ok(frame.gas >= 0);
        assert.ok(frame.jets.every((u) => u >= -1e-6 && u <= 8500 + 0.02));
        if (["APPROACH", "RAMP", "SETTLED"].includes(frame.phase))
          assert.equal(norm(frame.jets), 0);
        if (out.done) {
          ended = true;
          break;
        }
      }
      const summary = mission.summary();
      assert.ok(ended);
      assert.equal(summary.success, true, JSON.stringify(summary));
      assert.equal(summary.physics, "mujoco");
      assert.ok(summary.landing_error < 0.6);
      assert.ok(summary.stop_error < 0.65);
      assert.ok(summary.touchdown.vertical_speed < 0.3);
      assert.ok(summary.touchdown.tilt_degrees < 3);
      assert.equal(summary.mpc_rejected, 0);
      if (mission.barrel) {
        assert.ok(summary.stage_clearance > 0.15);
        if (config.require_roll === false) {
          assert.ok(Math.abs(summary.maneuver_roll_degrees) < 2);
          assert.ok(!phases.has("BARREL_ROLL"));
        } else assert.ok(Math.abs(summary.maneuver_roll_degrees - 360) < 2);
      }
    } finally {
      mission.dispose();
    }
  });

test("invalid targets and failed seeds cannot silently start a guidance flight", async () => {
  assert.throws(() => new CarMission({ landing_x: NaN }), /Landing target/);
  assert.throws(
    () => new CarMission({ maneuver: "barrel-roll", landing_x: 35 }),
    /Landing target/,
  );
  const mission = new CarMission({ goal_planning: true, gas_kg: 0 });
  await assert.rejects(prepareCarPlan(mission), /initialize|longer|validated/);
  assert.equal(mission.flightPlan, undefined);
  while (mission.release === null) mission.step();
  assert.throws(() => mission.step(), /Prepare a full-flight plan/);
  assert.equal(await prepareCarPlan(new CarMission()), null);
});
