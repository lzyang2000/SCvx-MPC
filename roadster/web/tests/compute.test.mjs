import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import load from "../node_modules/@mujoco/mujoco/mujoco.js";
import { initSolver, Conic } from "../src/compute/conic.js";
import {
  CarMission,
  NOZZLES,
  Ramp,
  carModel,
} from "../src/compute/car-mission.js";
import {
  ROCKET,
  properties,
  rocketModel,
  aerodynamicForce,
} from "../src/compute/rocket-dynamics.js";
import { RocketMission } from "../src/compute/rocket-mission.js";
import { norm, rotate, dot } from "../src/compute/math.js";
await initSolver(
  await readFile(
    new URL(
      "../src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
      import.meta.url,
    ),
  ),
);
const mj = await load(),
  xml = await readFile(
    new URL("../public/assets/rocket.xml", import.meta.url),
    "utf8",
  );

test("Clarabel WASM solves a constrained quadratic with an active bound", () => {
  const qp = new Conic(1);
  qp.square(0, 1, 3);
  qp.bound(0, 0, 2);
  const out = qp.solve();
  assert.equal(out.status, "optimal");
  assert.ok(Math.abs(out.x[0] - 2) < 1e-4);
});
test("rocket mass, COM and inertia match the native tank calibration", () => {
  const p = properties(433100);
  assert.equal(p.mass, 433100);
  assert.ok(Math.abs(p.z) < 1e-12);
  assert.ok(Math.abs(p.I[0] - 60000000) < 1e-6);
  assert.ok(Math.abs(p.I[2] - 1000000) < 1e-6);
  const dry = properties(25600);
  assert.ok(dry.I.every((v) => v > 0));
  const stack = properties(544600, 111500);
  assert.ok(Math.abs(stack.z - (111500 * 34) / 544600) < 1e-12);
  assert.ok(stack.I[0] > p.I[0]);
});
test("MuJoCo and the rocket prediction agree on a powered interval", () => {
  const m = new RocketMission(mj, xml);
  try {
    m.data.qpos[2] = 1000;
    m.attached = false;
    m.fuel = 20000;
    m.massUpdate(true);
    m.model.geom_contype[m.mount] = 0;
    m.model.geom_conaffinity[m.mount] = 0;
    const x = m.state(),
      u = [15000, -10000, 520000, 0, 0, 1000],
      model = rocketModel({
        max: ROCKET.engineMax,
        min: ROCKET.minThrottle * ROCKET.engineMax,
      }),
      dt = 0.2;
    const predicted = model.step(x, u, dt);
    m.control = u;
    m.phase = "LANDING";
    m.landingEngineCount = 1;
    m.lastSolve = m.data.time;
    m.interval = 99;
    for (let i = 0; i < 40; i++) m.step();
    const actual = m.state();
    assert.ok(
      norm(actual.slice(1, 4).map((v, i) => v - predicted[1 + i])) < 0.03,
    );
    assert.ok(
      norm(actual.slice(4, 7).map((v, i) => v - predicted[4 + i])) < 0.02,
    );
    assert.ok(dot(actual.slice(7, 11), predicted.slice(7, 11)) > 0.999999);
  } finally {
    m.dispose();
  }
});
test("terminal clearance follows the lowest tilted foot and MuJoCo pad contact", () => {
  const m = new RocketMission(mj, xml);
  try {
    m.phase = "LANDING";
    m.legs = 1;
    m.updateLegs();
    for (const [axis, angle] of [
      [0, 0],
      [0, 0.06],
      [1, -0.08],
    ]) {
      const q = [Math.cos(angle / 2), 0, 0, 0];
      q[axis + 1] = Math.sin(angle / 2);
      m.data.qpos.set(q, 3);
      m.data.qpos[2] = 30;
      const clearance = m.landingHeight();
      m.data.qpos[2] += 0.01 - clearance;
      assert.ok(Math.abs(m.landingHeight() - 0.01) < 1e-10);
      m.data.qpos[2] -= 0.02;
      mj.mj_forward(m.model, m.data);
      const contacts = m.data.contact;
      let nearest = Infinity;
      for (let i = 0; i < contacts.size(); i++) {
        const c = contacts.get(i);
        if (m.feet.includes(c.geom1) || m.feet.includes(c.geom2))
          nearest = Math.min(nearest, c.dist);
        c.delete();
      }
      contacts.delete();
      assert.ok(Math.abs(nearest + 0.01) < 1e-5, `Actual foot gap: ${nearest}`);
    }
  } finally {
    m.dispose();
  }
});
test("separation preserves body pose and transfers upper stage velocity", () => {
  const m = new RocketMission(mj, xml, { initialFuel: ROCKET.fuel });
  try {
    m.data.qpos.set([500, 2, 54000]);
    m.data.qvel.set([270, 0, 1200, 0.01, 0.02, 0]);
    const p = Array.from(m.data.qpos.slice(0, 7)),
      v = Array.from(m.data.qvel.slice(0, 6));
    m.control = [0, 0, 7500000, 0, 0, 0];
    m.activeCount = 9;
    m.separate();
    assert.equal(m.activeCount, 0);
    assert.equal(m.attached, false);
    assert.equal(m.phase, "SEPARATION");
    assert.equal(norm(m.control.slice(0, 3)), 0);
    assert.deepEqual(Array.from(m.data.qpos.slice(0, 7)), p);
    assert.deepEqual(Array.from(m.data.qpos.slice(7, 14)), p);
    assert.deepEqual(Array.from(m.data.qvel.slice(3, 6)), v.slice(3));
    const boosterVelocity = Array.from(m.data.qvel.slice(0, 3));
    const upperVelocity = Array.from(m.data.qvel.slice(6, 9));
    assert.deepEqual(
      upperVelocity.map((u, i) => u - boosterVelocity[i]),
      [0, 0, 3],
    );
    for (let i = 0; i < 3; i++)
      assert.ok(
        Math.abs(
          433100 * boosterVelocity[i] +
            111500 * upperVelocity[i] -
            544600 * v[i],
        ) < 1e-6,
      );
    assert.equal(m.model.body_mass[m.bid], 433100);
  } finally {
    m.dispose();
  }
});
test("rejected rocket optimization stops before applying a fallback command", () => {
  const m = new RocketMission(mj, xml);
  try {
    m.mpc.solve = () => ({
      command: null,
      snapshot: { success: false, status: "test_rejection", defect: 1, id: 1 },
    });
    const before = m.data.time,
      out = m.step();
    assert.equal(out.done, true);
    assert.equal(m.data.time, before);
    assert.match(m.failure, /test_rejection/);
    assert.deepEqual(
      Array.from(m.data.xfrc_applied),
      Array(m.data.xfrc_applied.length).fill(0),
    );
  } finally {
    m.dispose();
  }
});
test("rocket actuator projection enforces gimbal, throttle and RCS bounds", () => {
  for (const coast of [false, true]) {
    const model = rocketModel({
      max: coast ? 0 : ROCKET.engineMax,
      min: coast ? 0 : ROCKET.minThrottle * ROCKET.engineMax,
      coast,
    });
    for (const u of [
      [1e9, 1e9, -1e8, 1e9, -1e9, 1e9],
      [-1e8, 3e8, 1e9, -1e9, 1e9, -1e9],
    ]) {
      const a = model.limit(u);
      assert.ok(model.feasible(a));
      if (!coast) {
        assert.ok(a[3] === 0);
        assert.ok(a[4] === 0);
        assert.ok(norm(a.slice(0, 2)) <= Math.tan(Math.PI / 30) * a[2] + 1e-6);
      } else assert.equal(norm(a.slice(0, 3)), 0);
    }
  }
});
test("browser car preserves the native wheelbase contact geometry", () => {
  const r = new Ramp(10);
  assert.ok(Math.abs(r.at(r.release).p[2] - 19.277691220857236) < 1e-8);
  assert.equal(r.at(r.release).phi, Math.PI);
  assert.equal(NOZZLES.positions.length, 6);
  assert.deepEqual(NOZZLES.positions.slice(4), [
    [2, -0.8, -0.08],
    [2, 0.8, -0.08],
  ]);
});
test("only the front side pair contributes lateral thrust and coupled yaw torque", () => {
  const model = carModel({ nozzle_n: 8500, exhaust_mps: 650 }, 1678);
  for (const jet of [4, 5]) {
    const u = Array(6).fill(0);
    u[jet] = 1000;
    const force = model.wrench(u);
    assert.equal(force[0], 0);
    assert.equal(Math.abs(force[1]), 1000);
    assert.equal(force[5], 2 * force[1]);
  }
});
test("live car solves new SCvx plans and lands without ramp or ground jets", () => {
  const m = new CarMission();
  let minimumFlightX = Infinity,
    maximumReturnSpeed = -Infinity;
  let ended = false;
  for (let i = 0; i < 2500; i++) {
    const out = m.step(),
      f = m.frame();
    if (m.release !== null && !m.touch) {
      minimumFlightX = Math.min(minimumFlightX, f.p[0]);
      maximumReturnSpeed = Math.max(maximumReturnSpeed, f.v[0]);
    }
    assert.equal(f.jets.length, 6);
    assert.ok(f.jets.every((v) => v >= -1e-8 && v <= 8500 + 0.01));
    assert.ok(f.gas >= 0);
    if (["APPROACH", "RAMP", "SETTLED"].includes(f.phase))
      assert.equal(norm(f.jets), 0);
    if (["APPROACH", "RAMP"].includes(f.phase))
      assert.equal(m.solves.length, 0);
    if (out.done) {
      ended = true;
      break;
    }
  }
  const summary = m.summary();
  assert.ok(ended);
  assert.equal(summary.success, true, JSON.stringify(summary));
  assert.equal(summary.mpc_rejected, 0);
  assert.ok(summary.touchdown.vertical_speed < 0.2);
  assert.ok(summary.touchdown.tilt_degrees < 3);
  assert.ok(summary.gas_remaining > 30);
  assert.ok(summary.maneuver_roll_degrees > 179);
  assert.deepEqual(summary.start_position, summary.landing_target);
  assert.ok(
    minimumFlightX >= m.target - 0.15,
    "reach the marker without a flyback",
  );
  assert.ok(maximumReturnSpeed < 0.2, "allow only small settling corrections");
  assert.ok(
    summary.landing_error < 0.5,
    "land directly at the starting marker",
  );
  assert.ok(
    summary.stop_error < 0.25,
    "wheel braking stays at the starting marker",
  );
});
test("rocket separates at 75 km and returns with three separate burns", () => {
  const m = new RocketMission(mj, xml),
    phases = new Set();
  let ended = false,
    postSeparationBurns = 0,
    wasPowered = false,
    hoverSamples = 0,
    slowDescentSamples = 0,
    nearHoverSamples = 0,
    cutoffChecked = false;
  try {
    for (let i = 0; i < 150000; i++) {
      const out = m.step(),
        f = m.frame();
      phases.add(f.phase);
      assert.ok(f.fuel >= 0);
      assert.ok(
        norm(f.control.slice(0, 3)) <= f.engines * ROCKET.engineMax + 2,
      );
      if (f.phase !== "CONTACT" && !out.done)
        assert.equal(f.controller, "SCvx MPC");
      if (["MECO", "SEPARATION", "COAST", "DESCENT_COAST"].includes(f.phase)) {
        assert.equal(norm(f.control.slice(0, 3)), 0);
        assert.equal(f.engines, 0);
      }
      if (f.phase === "MECO") {
        if (!cutoffChecked) {
          const delay = ROCKET.separationDelay;
          const predicted =
            m.landingHeight() +
            Math.max(m.state()[6], 0) * delay -
            0.5 * 9.81 * delay ** 2;
          assert.ok(Math.abs(predicted - m.separationAltitude) < 50);
          cutoffChecked = true;
        }
        assert.equal(f.attached, true);
        assert.equal(f.upper.thrust, 0);
      }
      if (f.phase === "SEPARATION") assert.equal(f.attached, false);
      if (f.upper.thrust > 0) {
        assert.ok(
          m.upperIgnitionTime - m.separationTime >=
            ROCKET.upperIgnitionDelay - 1e-8,
        );
        if (m.data.time - m.upperIgnitionTime < 0.01)
          assert.ok(m.stageClearance() >= 5);
      }
      if (!f.attached && m.data.time - m.separationTime < 15) {
        assert.ok(m.stageClearance() > 0);
        assert.ok(
          dot(
            f.upper.v.map((v, i) => v - f.v[i]),
            m.separationAxis,
          ) > 0,
        );
      }
      if (!f.attached) {
        const powered = norm(f.control.slice(0, 3)) > 1000;
        if (powered && !wasPowered) postSeparationBurns++;
        if (powered) {
          assert.ok(
            f.control[2] >=
              ROCKET.minThrottle * f.engines * ROCKET.engineMax - 2,
          );
        }
        wasPowered = powered;
      }
      // Count actual near-zero motion, including both transitions, so changing
      // the phase label cannot hide a prolonged hover.
      if (
        f.phase === "LANDING" &&
        m.landingHeight() < 6 &&
        Math.abs(f.v[2]) < 0.15
      )
        nearHoverSamples++;
      if (f.landing_mode === "hover") {
        hoverSamples++;
        assert.ok(Math.abs(m.landingHeight() - ROCKET.hoverHeight) < 0.3);
        assert.ok(
          Math.abs(f.v[2]) < 0.16,
          "physically settle before counting the dwell",
        );
        assert.ok(norm(f.v.slice(0, 2)) < 0.31);
        assert.ok(Math.hypot(f.p[0] - ROCKET.padX, f.p[1]) < 2.1);
        assert.equal(f.engines, 1);
      }
      if (f.landing_mode === "slow-descent") {
        slowDescentSamples++;
        assert.ok(
          f.v[2] > -0.8 && f.v[2] < 0.02,
          "lower gently after hovering",
        );
      }
      if (out.solve)
        assert.equal(out.solve.success, true, JSON.stringify(out.solve));
      if (out.done) {
        ended = true;
        break;
      }
    }
    const s = m.summary();
    assert.ok(ended);
    assert.equal(s.success, true, JSON.stringify(s));
    assert.deepEqual(
      [...phases],
      [
        "ASCENT",
        "MECO",
        "SEPARATION",
        "BOOSTBACK",
        "COAST",
        "ENTRY",
        "DESCENT_COAST",
        "LANDING",
        "CONTACT",
      ],
    );
    assert.equal(s.mpc_rejected, 0);
    assert.ok(
      s.separation_time - s.cutoff_time >= ROCKET.separationDelay - 1e-8,
    );
    assert.ok(s.upper_ignition_time > s.separation_time);
    assert.ok(Math.abs(s.separation.altitude - ROCKET.separationAltitude) < 50);
    assert.ok(s.separation.speed > 1400 && s.separation.speed < 1900);
    assert.equal(s.separation.booster_mass, ROCKET.dry + s.separation.fuel);
    assert.ok(s.fuel_remaining > 0);
    if (s.landing_profile === "hover") {
      assert.ok(
        Math.abs(
          s.slow_descent_started_at - s.hover_started_at - ROCKET.hoverSeconds,
        ) < 0.01,
      );
      assert.ok(
        hoverSamples * m.model.opt.timestep >= ROCKET.hoverSeconds - 0.01,
      );
      assert.ok(slowDescentSamples * m.model.opt.timestep > 2);
      assert.ok(slowDescentSamples * m.model.opt.timestep < 7);
    } else {
      assert.equal(s.hover_started_at, null);
      assert.equal(s.slow_descent_started_at, null);
    }
    assert.ok(nearHoverSamples * m.model.opt.timestep <= 3);
    assert.ok(cutoffChecked);
    assert.equal(postSeparationBurns, 3);
    assert.equal(s.ignitions, 4);
    assert.equal(s.post_separation_ignitions, 3);
    assert.deepEqual(s.landing_target, s.start_position);
    assert.ok(s.settled_speed < 0.15);
    assert.ok(s.pad_error < 8);
    assert.ok(
      s.touchdown.vertical_speed < 0.25,
      "touch down with near-zero descent speed",
    );
    assert.ok(s.touchdown.tilt_degrees < 6);
  } finally {
    m.dispose();
  }
});

test("atmospheric drag opposes motion and follows altitude and projected area", () => {
  const x = [50000, 0, 0, 1000, 0, 0, -500, 1, 0, 0, 0, 0, 0, 0];
  const vertical = aerodynamicForce(x);
  assert.ok(dot(vertical, x.slice(4, 7)) < 0);
  const high = x.slice();
  high[3] = 50000;
  assert.ok(norm(aerodynamicForce(high)) < norm(vertical) / 100);
  const sideways = x.slice();
  sideways.splice(7, 4, Math.SQRT1_2, 0, Math.SQRT1_2, 0);
  assert.ok(norm(aerodynamicForce(sideways)) > norm(vertical) * 5);
  const still = x.slice();
  still.splice(4, 3, 0, 0, 0);
  assert.deepEqual(aerodynamicForce(still), [0, 0, 0]);
});
