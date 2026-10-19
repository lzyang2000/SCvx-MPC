import test from "node:test";
import assert from "node:assert/strict";
import load from "../node_modules/@mujoco/mujoco/mujoco.js";
import { MuJoCoCarMission } from "../src/compute/car-mujoco.js";
import { carModel } from "../src/compute/car-mission.js";
import { norm, rotate, dot } from "../src/compute/math.js";
const mj = await load();

function airborne(m, height = 10) {
  m.data.qpos.set([80, 0, height, 1, 0, 0, 0]);
  m.data.qvel.fill(0);
  m.release = 0;
  m.phase = "DESCENT";
  mj.mj_forward(m.model, m.data);
  m.syncState();
}

test("MuJoCo wheels drive the C-ramp through contact and leave its upper edge inverted", () => {
  const m = new MuJoCoCarMission(mj, { use_mpc: false });
  try {
    let rampContact = false;
    for (let k = 0; k < 1500 && m.release === null && !m.failure; k++) {
      m.step();
      if (m.contactState().ramp > 0) rampContact = true;
      assert.equal(norm(m.u), 0);
      assert.deepEqual(m.x.slice(1, 4), Array.from(m.data.qpos.slice(0, 3)));
    }
    assert.equal(m.failure, null);
    assert.ok(rampContact);
    assert.ok(m.release > 4 && m.release < 8);
    assert.ok(rotate(m.x.slice(7, 11), [0, 0, 1])[2] < -0.99);
    assert.equal(m.contactState().ramp, 0);
    assert.ok(m.x[1] < -1.3 && m.x[3] > 19);
    assert.ok(m.axles.every((a) => Math.abs(m.data.qvel[a.v]) > 20));
    const p = m.x.slice(1, 4);
    for (let i = 0; i < 10; i++) m.step();
    assert.ok(m.x[3] < p[2] - 0.05, "with jets off MuJoCo supplies freefall");
  } finally {
    m.dispose();
  }
});

test("six nozzle point forces produce physical translation and torque with the expected directions", () => {
  for (const jet of [0, 1, 4, 5]) {
    const m = new MuJoCoCarMission(mj, {
      maneuver: "barrel-roll",
      use_mpc: false,
    });
    try {
      airborne(m);
      m.u[jet] = 1000;
      m.applyForces(false);
      mj.mj_forward(m.model, m.data);
      const a = Array.from(m.data.qacc.slice(0, 6));
      if (jet < 4) {
        assert.ok(Math.abs(a[2] - (1000 / 1678 - 9.81)) < 1e-6);
        assert.ok(Math.abs(a[3]) > 0.8);
        assert.ok(Math.abs(a[4]) > 0.5);
      } else {
        const sign = jet === 4 ? 1 : -1;
        assert.ok(a[1] * sign > 0.59);
        assert.ok(a[5] * sign > 0.68, "front side jets also yaw the chassis");
        assert.ok(Math.abs(a[0]) < 1e-8);
      }
    } finally {
      m.dispose();
    }
  }
});

test("RK4 prediction stays close to a separate MuJoCo powered rollout", () => {
  const m = new MuJoCoCarMission(mj, { maneuver: "barrel-roll" });
  try {
    airborne(m);
    m.u = [1800, 2200, 2100, 1700, 300, 200];
    const initial = m.x.slice(),
      dt = 0.12;
    const predicted = carModel(m.config, m.mass).step(initial, m.u, dt);
    for (let i = 0; i < 60; i++) {
      m.massUpdate();
      m.applyForces(false);
      mj.mj_step(m.model, m.data);
      m.mass -= (m.u.reduce((a, b) => a + b, 0) / m.config.exhaust_mps) * 0.002;
      m.syncState();
    }
    assert.ok(
      norm(m.x.slice(1, 4).map((v, i) => v - predicted[i + 1])) < 0.003,
    );
    assert.ok(
      norm(m.x.slice(4, 7).map((v, i) => v - predicted[i + 4])) < 0.003,
    );
    assert.ok(
      Math.abs(dot(m.x.slice(7, 11), predicted.slice(7, 11))) > 0.999999,
    );
  } finally {
    m.dispose();
  }
});

test("MuJoCo detects tire touchdown, cuts jets and brakes on real wheel contacts", () => {
  const m = new MuJoCoCarMission(mj, {
    maneuver: "barrel-roll",
    use_mpc: false,
  });
  try {
    airborne(m, 0.7);
    m.data.qvel[0] = 0.5;
    m.data.qvel[2] = -0.2;
    mj.mj_forward(m.model, m.data);
    m.hasLeftGround = true;
    for (let i = 0; i < 300 && m.land === null; i++) m.step();
    assert.ok(m.touch);
    assert.ok(m.contacts > 0);
    assert.equal(norm(m.u), 0);
    for (let i = 0; i < 200; i++) m.step();
    assert.ok(norm(m.x.slice(4, 7)) < 0.01);
    assert.ok(m.x[3] > 0.6 && m.x[3] < 0.63);
  } finally {
    m.dispose();
  }
});

test("MuJoCo collisions with display vehicles and the chassis stop the mission", () => {
  for (const [position, expected] of [
    [[34, 0, 3], /Display vehicle impact/],
    [[80, 0, 0.2], /Body impact/],
  ]) {
    const m = new MuJoCoCarMission(mj, {
      maneuver: "barrel-roll",
      use_mpc: false,
    });
    try {
      airborne(m);
      m.data.qpos.set(position);
      mj.mj_forward(m.model, m.data);
      assert.equal(m.step().done, true);
      assert.match(m.failure, expected);
    } finally {
      m.dispose();
    }
  }
});

test("actuator gas limits hold in MuJoCo even if commands exceed the available gas", () => {
  const m = new MuJoCoCarMission(mj, {
    maneuver: "barrel-roll",
    gas_kg: 0.001,
  });
  try {
    airborne(m);
    m.mpc.plan = Array.from({ length: 10 }, () => Array(6).fill(8500));
    m.next = Infinity;
    for (let i = 0; i < 5; i++) m.step();
    assert.ok(m.mass >= 1428);
    assert.ok(m.u.every((v) => v < 1));
  } finally {
    m.dispose();
  }
});
