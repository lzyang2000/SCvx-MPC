import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import * as THREE from "three";
import { initSolver } from "../src/compute/conic.js";
import { CarMission } from "../src/compute/car-mission.js";
import { rotate, norm } from "../src/compute/math.js";
import { STAGE_VEHICLES, stageClearance } from "../src/car-stage.js";
import { createStageVehicles } from "../src/stage-models.js";
await initSolver(
  await readFile(
    new URL(
      "../src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
      import.meta.url,
    ),
  ),
);

test("barrel roll physically clears the display row, lands forward and brakes on wheels", () => {
  const m = new CarMission({ maneuver: "barrel-roll" }),
    phases = new Set();
  let ended = false,
    previousGas = 250,
    inverted = false;
  for (let i = 0; i < 2500; i++) {
    const out = m.step(),
      f = m.frame();
    phases.add(f.phase);
    assert.ok(f.gas >= 0 && f.gas <= previousGas + 1e-7);
    previousGas = f.gas;
    assert.equal(f.jets.length, 6);
    assert.ok(f.jets.every((u) => u >= -1e-8 && u <= 8500 + 0.01));
    assert.equal(f.fan_g, 0);
    if (rotate(f.q, [0, 0, 1])[2] < -0.99) inverted = true;
    if (f.phase === "APPROACH" || f.phase === "SETTLED" || !m.solves.length)
      assert.equal(norm(f.jets), 0);
    else {
      assert.match(f.controller, /SCvx MPC/);
      assert.ok(m.solves.at(-1).success);
    }
    if (out.done) {
      ended = true;
      break;
    }
  }
  const s = m.summary();
  assert.ok(ended);
  assert.ok(inverted);
  assert.equal(s.success, true, JSON.stringify(s));
  assert.deepEqual(
    [...phases],
    ["APPROACH", "LIFTOFF", "BARREL_ROLL", "DESCENT", "SETTLED"],
  );
  assert.ok(s.maneuver_roll_degrees > 350 && s.maneuver_roll_degrees < 370);
  assert.ok(s.touchdown.vertical_speed < 0.5 && s.touchdown.tilt_degrees < 1);
  assert.ok(s.stage_clearance > 1);
  assert.ok(s.gas_used <= 250 && s.gas_remaining > 30);
  assert.ok(s.settled_speed < 0.15);
  assert.ok(s.start_position[0] >= 5, "start beside the display row");
  assert.ok(s.touchdown.position[0] > 47 && s.touchdown.position[0] < 56);
  assert.ok(m.frame().p[0] < 56, "stop just beyond the final parked car");
  assert.equal(s.mpc_rejected, 0);
  assert.ok(rotate(m.frame().q, [1, 0, 0])[0] > 0.99);
});

test("parked vehicle overlap is detected and stops the physical mission", () => {
  assert.ok(
    stageClearance([
      [34, 0, 3.9],
      [34.2, 0, 4.5],
    ]).clearance < 0,
  );
  assert.equal(
    stageClearance([
      [34, 0, 5],
      [34.2, 0, 6],
    ]).clearance,
    1,
  );
  const m = new CarMission({ maneuver: "barrel-roll" });
  m.phase = "DESCENT";
  m.release = -4;
  m.t = 1;
  m.next = 100;
  m.owner = "test";
  m.x.splice(1, 3, 34, 0, 3);
  m.x.splice(4, 3, 0, 0, 0);
  const out = m.step();
  assert.equal(out.done, true);
  assert.match(m.failure, /Display vehicle impact: Semi/);
});

test("the five display meshes stay inside the physical obstacle bounds", () => {
  const row = createStageVehicles();
  assert.deepEqual(
    row.children.map((c) => c.name),
    STAGE_VEHICLES.map((v) => v.name),
  );
  row.updateMatrixWorld(true);
  for (let i = 0; i < row.children.length; i++) {
    const box = new THREE.Box3().setFromObject(row.children[i]),
      v = STAGE_VEHICLES[i];
    assert.ok(
      box.min.x >= v.x - v.size[0] / 2 - 0.001,
      `${v.name}: left ${box.min.x}`,
    );
    assert.ok(
      box.max.x <= v.x + v.size[0] / 2 + 0.001,
      `${v.name}: right ${box.max.x}`,
    );
    assert.ok(
      box.min.y >= -0.001 && box.max.y <= v.size[2] + 0.001,
      `${v.name}: height ${box.max.y}`,
    );
    assert.ok(
      box.min.z >= -v.size[1] / 2 - 0.001 && box.max.z <= v.size[1] / 2 + 0.001,
      `${v.name}: length`,
    );
  }
});

test("barrel example is a complete recording from the same six-jet mission", async () => {
  const data = JSON.parse(
    await readFile(new URL("../public/barrel-nominal.json", import.meta.url)),
  );
  assert.equal(data.config.maneuver, "barrel-roll");
  assert.ok(data.summary.success);
  assert.equal(data.nozzles.positions.length, 6);
  assert.ok(data.summary.stage_clearance > 0);
  assert.ok(data.frames.some((f) => f.phase === "BARREL_ROLL"));
  assert.ok(data.frames.at(-1).phase === "SETTLED");
});
