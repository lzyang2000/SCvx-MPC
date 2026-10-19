import { CAR_LAYOUT, stageClearance } from "../car-stage.js";
import { Scvx } from "./scvx.js";
import {
  add,
  sub,
  scale,
  dot,
  norm,
  cross,
  unit,
  rotate,
  invRotate,
  qmul,
  clamp,
  rk4,
} from "./math.js";
export const NOZZLES = { positions: [], directions: [] };
for (const x of [-1.45, 1.45])
  for (const y of [-0.78, 0.78]) {
    NOZZLES.positions.push([x, y, -0.32]);
    NOZZLES.directions.push([0, 0, 1]);
  }
// One side jet at each front corner; no rear side jets.
for (const y of [-0.8, 0.8]) {
  NOZZLES.positions.push([2, y, -0.08]);
  NOZZLES.directions.push([0, -Math.sign(y), 0]);
}
const NOZZLE_COUNT = NOZZLES.positions.length;
const torques = NOZZLES.positions.map((p, i) =>
  cross(p, NOZZLES.directions[i]),
);
const wheels = [-1.4, 1.4].flatMap((x) =>
  [-0.85, 0.85].map((y) => [x, y, -0.62]),
);
const inertia = (m) => scale([800, 2600, 2900], m / 1678);
const wrench = (u) =>
  [0, 1, 2, 3, 4, 5].map((i) =>
    u.reduce(
      (a, v, j) =>
        a + v * (i < 3 ? NOZZLES.directions[j][i] : torques[j][i - 3]),
      0,
    ),
  );
export function attitude(t, angle = Math.PI) {
  const u = clamp(t / 2, 0, 1),
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5,
    p = angle + (Math.PI - angle) * s;
  return qmul(
    [Math.cos(p / 2), 0, -Math.sin(p / 2), 0],
    [Math.cos((Math.PI * s) / 2), Math.sin((Math.PI * s) / 2), 0, 0],
  );
}
// A full longitudinal roll after a short thruster-powered climb.
const BARREL_LIFT = 1.8;
const BARREL_ROLL = 2;
export function barrelAttitude(t) {
  const u = clamp((t - BARREL_LIFT) / BARREL_ROLL, 0, 1);
  const s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5;
  return [Math.cos(Math.PI * s), Math.sin(Math.PI * s), 0, 0];
}
function barrelHeight(t) {
  const lift = Math.min(t, BARREL_LIFT),
    coast = Math.max(0, t - BARREL_LIFT);
  return {
    z:
      0.62 +
      0.5 * 5 * lift ** 2 +
      5 * BARREL_LIFT * coast -
      0.5 * 9.81 * coast ** 2,
    vz: 5 * lift - 9.81 * coast,
  };
}
export function carModel(cfg, mass) {
  const cap = cfg.nozzle_n * Math.min(1, Math.max(0, mass - 1428) / 12);
  const dyn = (x, u) => {
    const m = Math.max(x[0], 1428),
      q = unit(x.slice(7, 11)),
      w = x.slice(11),
      I = inertia(m),
      f = wrench(u),
      v = x.slice(4, 7),
      drag = scale(v, -0.42 * norm(v));
    return [
      -u.reduce((a, b) => a + b, 0) / cfg.exhaust_mps,
      ...v,
      ...add(scale(add(rotate(q, f.slice(0, 3)), drag), 1 / m), [0, 0, -9.81]),
      ...scale(qmul(q, [0, ...w]), 0.5),
      ...sub(
        f.slice(3),
        cross(
          w,
          w.map((v, i) => v * I[i] + (cfg.wheel_momentum?.[i] ?? 0)),
        ),
      ).map((v, i) => v / I[i] - 0.025 * w[i]),
    ];
  };
  return {
    step: (x, u, dt) => rk4(x, u, dt, dyn),
    limit: (u) => u.map((v) => clamp(v, 0, cap)),
    feasible: (u) => u.every((v) => v >= -1e-5 && v <= cap + 0.02),
    constraints(qp, i, k, s) {
      for (let j = 0; j < NOZZLE_COUNT; j++) qp.bound(i + j, 0, cap / s[j]);
    },
    trajectoryConstraints(qp, { uoff, nu, n, su, dt, nx, sx }) {
      const row = [];
      for (let k = 0; k < n; k++) {
        for (let j = 0; j < nu; j++)
          row.push([uoff + k * nu + j, (su[j] * dt) / cfg.exhaust_mps / 250]);
        qp.le(row.slice(), (mass - 1428) / 250);
        for (let j = 0; j < nu; j++)
          qp.le(
            [
              [uoff + k * nu + j, 1],
              [k * nx, ((-cfg.nozzle_n / 12) * sx[0]) / su[j]],
            ],
            ((-cfg.nozzle_n / 12) * 1428) / su[j],
          );
      }
    },
    rolloutFeasible(xs, us, dt) {
      return us.every(
        (u, k) =>
          u.every(
            (v) =>
              v <=
              cfg.nozzle_n * Math.min(1, Math.max(0, xs[k][0] - 1428) / 12) +
                0.05,
          ) &&
          (u.reduce((a, b) => a + b, 0) * dt) / cfg.exhaust_mps <=
            Math.max(0, xs[k][0] - 1428) + 0.01,
      );
    },
    wrench,
  };
}
// Rigid wheelbase on the circular deck, including mixed flat/circle axle contact.
export class Ramp {
  constructor(r) {
    this.r = r;
    this.entry = -1.4;
    const rh = r - 0.37;
    this.angle = Math.asin(1.4 / rh);
    this.orbit = Math.sqrt(rh * rh - 1.4 ** 2) - 0.25;
    this.rows = [];
    let length = 0;
    for (let i = 0; i <= 256; i++) {
      const a = (2 * this.angle * i) / 256,
        fx = rh * Math.sin(a),
        fz = r - rh * Math.cos(a),
        dz = fz - 0.37,
        dx = Math.sqrt(2.8 ** 2 - dz * dz),
        p = Math.atan2(dz, dx),
        x = fx - dx / 2 - 0.25 * Math.sin(p),
        z = (fz + 0.37) / 2 + 0.25 * Math.cos(p);
      if (i)
        length += Math.hypot(x - this.rows.at(-1)[1], z - this.rows.at(-1)[2]);
      this.rows.push([length, x, z, p]);
    }
    this.length = length;
    this.release = this.entry + length + this.orbit * (Math.PI - this.angle);
    const deriv = (field, i) => {
      const a = this.rows[Math.max(0, i - 1)],
        b = this.rows[Math.min(256, i + 1)];
      return (b[field] - a[field]) / (b[0] - a[0]);
    };
    this.rows.forEach((row, i) => {
      const tx = deriv(1, i),
        tz = deriv(2, i),
        n = Math.hypot(tx, tz);
      row.push(tx / n, tz / n, deriv(3, i));
    });
    this.rows.forEach((row, i) => {
      const a = this.rows[Math.max(0, i - 1)],
        b = this.rows[Math.min(256, i + 1)];
      row.push(
        (Math.atan2(b[5], b[4]) - Math.atan2(a[5], a[4])) / (b[0] - a[0]),
      );
    });
  }
  at(s) {
    if (s < this.entry)
      return { p: [s, 0, 0.62], t: [1, 0, 0], phi: 0, rate: 0, c: 0 };
    const d = s - this.entry;
    if (d < this.length) {
      let i = 1;
      while (this.rows[i][0] < d) i++;
      const a = this.rows[i - 1],
        b = this.rows[i],
        u = (d - a[0]) / (b[0] - a[0]),
        r = a.map((v, j) => v + u * (b[j] - v));
      return {
        p: [r[1], 0, r[2]],
        t: unit([r[4], 0, r[5]]),
        phi: r[3],
        rate: r[6],
        c: r[7],
      };
    }
    const p = Math.min(Math.PI, this.angle + (d - this.length) / this.orbit);
    return {
      p: [this.orbit * Math.sin(p), 0, this.r - this.orbit * Math.cos(p)],
      t: [Math.cos(p), 0, Math.sin(p)],
      phi: p,
      rate: 1 / this.orbit,
      c: 1 / this.orbit,
    };
  }
}
export class CarMission {
  constructor(config = {}) {
    this.config = {
      maneuver: "c-ramp",
      radius: 10,
      ramp_degrees: 180,
      speed: 12,
      gas_kg: 250,
      nozzle_n: 8500,
      exhaust_mps: 650,
      fan_g: 2,
      use_mpc: true,
      goal_planning: false,
      require_roll: true,
      dt: 0.01,
      ...config,
    };
    const c = this.config;
    if (
      !["c-ramp", "barrel-roll"].includes(c.maneuver) ||
      ![c.radius, c.speed, c.gas_kg].every(Number.isFinite) ||
      c.radius < 8 ||
      c.radius > 16 ||
      c.speed < 9 ||
      c.speed > 16 ||
      c.gas_kg < 0 ||
      c.gas_kg > 250
    )
      throw Error("Invalid car configuration");
    this.barrel = c.maneuver === "barrel-roll";
    this.layout = CAR_LAYOUT[c.maneuver];
    this.hasLeftGround = false;
    this.minStageClearance = Infinity;
    this.ramp = new Ramp(c.radius);
    this.x = [
      1428 + c.gas_kg,
      this.layout.start,
      0,
      0.62,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      0,
      0,
    ];
    this.t = 0;
    this.path = this.layout.start;
    this.speed = 0;
    this.phase = "APPROACH";
    this.owner = "WHEEL DRIVE";
    this.u = Array(NOZZLE_COUNT).fill(0);
    this.solves = [];
    this.events = [];
    this.release = null;
    this.land = null;
    this.failure = null;
    this.roll = 0;
    this.maneuver = 0;
    this.previousRoll = 0;
    this.fan = 0;
    this.contacts = 0;
    this.next = 0;
    this.target = c.landing_x ?? this.layout.landing;
    if (
      !Number.isFinite(this.target) ||
      this.target < (this.barrel ? 48 : -35) ||
      this.target > (this.barrel ? 80 : -5)
    )
      throw Error("Landing target is outside the supported range");
    this.maxHeight = 0;
    this.maxFan = 0;
    this.touch = null;
    this.mpc = new Scvx({
      nu: NOZZLE_COUNT,
      n: 10,
      scaleX: [1678, 15, 5, 10, 12, 5, 6, 1, 1, 1, 1, 3, 3, 3],
      scaleU: Array(NOZZLE_COUNT).fill(c.nozzle_n),
    });
  }
  plannedSample(time) {
    const plan = this.flightPlan;
    const node = Math.max(0, (time - plan.start) / plan.dt);
    if (node >= plan.controls.length) {
      const x = plan.target.slice();
      x[3] = Math.max(0.56, x[3] - 0.18 * (time - plan.start - plan.duration));
      return { x, u: plan.controls.at(-1).slice() };
    }
    const k = Math.floor(node),
      fraction = node - k;
    const x = plan.states[k].map(
      (v, i) => v + fraction * (plan.states[k + 1][i] - v),
    );
    x.splice(7, 4, ...unit(x.slice(7, 11)));
    return { x, u: plan.controls[k].slice() };
  }
  references() {
    if (this.flightPlan)
      return Array.from(
        { length: 11 },
        (_, k) => this.plannedSample(this.t + k * 0.12).x,
      );
    const x = this.x,
      dt = 0.12,
      h = Math.max(x[3] - 0.62, 0),
      // Descend promptly at altitude, then soften the approach near the tires.
      // The front-only side pair couples lateral thrust to yaw; bank to correct drift.
      vz = -Math.min(4, 0.45 + 0.9 * h),
      ax = this.barrel
        ? clamp(-1.3 * x[4] + 0.4 * (this.target - x[1]), -4.5, 4.5)
        : clamp(-2 * x[4] + 0.8 * (this.target - x[1]), -8, 8),
      ay = clamp(-2 * x[5] - x[2], -3, 3),
      refs = [];
    for (let k = 0; k <= 10; k++) {
      const tau = k * dt,
        tt = this.t - this.release + tau,
        r = x.slice(),
        end = this.barrel ? BARREL_LIFT + BARREL_ROLL : 2,
        // Brake from release on the C-ramp so the car reaches the marker
        // in one pass. The barrel maneuver retains its lift and roll sequence.
        u = clamp(
          (tt - (this.barrel ? end : 0)) / (this.barrel ? 0.7 : 0.4),
          0,
          1,
        ),
        blend = u * u * (3 - 2 * u),
        tilt =
          (this.barrel ? 1 : -1) *
          Math.atan2(ax, 9.81) *
          blend *
          clamp(h / (this.barrel ? 3 : 2), 0, 1);
      let q = qmul(
        this.barrel
          ? this.config.require_roll
            ? barrelAttitude(tt)
            : [1, 0, 0, 0]
          : attitude(tt, this.releaseAngle),
        [Math.cos(tilt / 2), 0, Math.sin(tilt / 2), 0],
      );
      const sideTilt =
        (this.barrel ? -1 : 1) *
        Math.atan2(ay, 9.81) *
        blend *
        clamp(h / 3, 0, 1);
      q = qmul(q, [Math.cos(sideTilt / 2), Math.sin(sideTilt / 2), 0, 0]);
      if (dot(q, k ? refs[k - 1].slice(7, 11) : x.slice(7, 11)) < 0)
        q = scale(q, -1);
      r.splice(7, 4, ...q);
      r.splice(
        1,
        3,
        x[1] + x[4] * tau + 0.5 * ax * tau * tau,
        x[2] + x[5] * tau + 0.5 * ay * tau * tau,
        Math.max(this.barrel ? 0.3 : 0.58, x[3] + vz * tau),
      );
      r.splice(4, 3, x[4] + ax * tau, x[5] + ay * tau, vz);
      if (this.barrel && tt <= BARREL_LIFT + BARREL_ROLL) {
        const height = barrelHeight(tt);
        r[3] = height.z;
        r[6] = height.vz;
      }
      refs.push(r);
    }
    for (let k = 0; k < 10; k++) {
      const q = refs[k].slice(7, 11),
        dq = qmul([q[0], -q[1], -q[2], -q[3]], refs[k + 1].slice(7, 11));
      refs[k].splice(11, 3, ...scale(dq.slice(1), 2 / 0.12));
    }
    refs[10].splice(11, 3, ...refs[9].slice(11));
    return refs;
  }
  seed(refs, model) {
    let x = this.x.slice();
    const us = [];
    for (let k = 0; k < 10; k++) {
      const q = x.slice(7, 11),
        w = x.slice(11),
        I = inertia(x[0]);
      let e = qmul([q[0], -q[1], -q[2], -q[3]], refs[k + 1].slice(7, 11));
      if (e[0] < 0) e = scale(e, -1);
      const tau = add(
          I.map(
            (v, i) => v * (36 * e[i + 1] + 7 * (refs[k + 1][11 + i] - w[i])),
          ),
          cross(
            w,
            w.map((v, i) => v * I[i]),
          ),
        ),
        acc = refs[k + 1]
          .slice(4, 7)
          .map((v, i) =>
            clamp(
              3 * (v - x[4 + i]),
              [this.barrel ? -4 : -10, -3, -6][i],
              [this.barrel ? 4 : 10, 3, 13][i],
            ),
          ),
        desired = [
          ...invRotate(q, scale(add(acc, [0, 0, 9.81]), x[0])),
          ...tau,
        ];
      // Projected least squares seeds the convex iteration; it never drives the plant.
      let u = Array(NOZZLE_COUNT).fill(0);
      for (let it = 0; it < 120; it++)
        for (let j = 0; j < NOZZLE_COUNT; j++) {
          const f = wrench(u),
            column = [...NOZZLES.directions[j], ...torques[j]],
            weights = [1, 1, 1, 9, 9, 9];
          let num = 0,
            den = 0.035 ** 2;
          for (let i = 0; i < 6; i++) {
            num += column[i] * weights[i] * (desired[i] - f[i]);
            den += column[i] * weights[i] * column[i];
          }
          u[j] = clamp(u[j] + num / den, 0, this.config.nozzle_n);
        }
      u = model.limit(u);
      us.push(u);
      x = model.step(x, u, 0.12);
    }
    return us;
  }
  plan() {
    const model = carModel(this.predictionConfig?.() ?? this.config, this.x[0]),
      refs = this.references(),
      seed = this.flightPlan
        ? Array.from(
            { length: 10 },
            (_, k) => this.plannedSample(this.t + k * 0.12).u,
          )
        : this.seed(refs, model);
    const out = this.mpc.solve(this.x, refs, model, {
      dt: 0.12,
      weights: [0, 2, 4, 5, 3, 3, 5, 15, 15, 15, 15, 4, 4, 4],
      seed: (k) => seed[k],
      iterations: 3,
      trust: 0.65,
      // The flight predictor excludes tire forces. Allow its terminal COM below
      // wheel height so it reaches contact; the plant cuts jets at actual contact.
      ground: this.barrel ? 0.3 : 0.56,
      dry: 1428,
      controlLinear: 0.5,
      controlCost: 0.05,
      maxDefect: 0.12,
    });
    const s = {
      ...out.snapshot,
      t: this.t,
      phase: "FLIGHT",
      model: "six fixed jets / browser SCvx",
      nozzle_plan: out.snapshot.control_plan,
      requested_force: out.command
        ? rotate(this.x.slice(7, 11), wrench(out.command).slice(0, 3))
        : null,
    };
    this.solves.push(s);
    if (out.command) this.u = out.command;
    else this.failure = "SCvx plan rejected; simulation stopped";
    return s;
  }
  step() {
    if (this.failure) return { done: true };
    if (
      this.config.goal_planning &&
      this.config.use_mpc &&
      this.release !== null &&
      !this.flightPlan
    )
      throw Error("Prepare a full-flight plan before takeoff");
    const c = this.config,
      dt = c.dt;
    let solve = null;
    this.u.fill(0);
    this.fan = 0;
    if (["APPROACH", "RAMP"].includes(this.phase)) {
      const flat = (s) => ({
        p: [s, 0, 0.62],
        t: [1, 0, 0],
        phi: 0,
        rate: 0,
        c: 0,
      });
      const a = this.barrel ? flat(this.path) : this.ramp.at(this.path),
        mass = this.x[0],
        desired = clamp(2.8 * (c.speed - this.speed), -5, 5);
      this.fan =
        !this.barrel && this.path >= this.ramp.entry
          ? Math.min(
              Math.max(0, mass * (-(this.speed ** 2) * a.c - 9.81 * a.t[0])) +
                0.65 * mass * 9.81,
              c.fan_g * mass * 9.81,
            )
          : 0;
      const normal =
          mass * this.speed ** 2 * a.c + this.fan + mass * 9.81 * a.t[0],
        traction = Math.max(normal, 0) * 1.25,
        drive = clamp(mass * (desired + 9.81 * a.t[2]), -traction, traction);
      this.speed = Math.max(
        0,
        this.speed + (drive / mass - 9.81 * a.t[2]) * dt,
      );
      this.path += this.speed * dt;
      const b = this.barrel ? flat(this.path) : this.ramp.at(this.path);
      this.x.splice(1, 3, ...b.p);
      this.x.splice(4, 3, ...scale(b.t, this.speed));
      this.x.splice(7, 4, Math.cos(b.phi / 2), 0, -Math.sin(b.phi / 2), 0);
      this.x.splice(11, 3, 0, -this.speed * b.rate, 0);
      if (this.barrel && this.path >= this.layout.takeoff) {
        this.phase = "LIFTOFF";
        this.release = this.t;
        this.releaseAngle = 0;
        this.events.push({ t: this.t, name: "Thruster liftoff / flight MPC" });
      }
      if (!this.barrel && this.path >= this.ramp.entry) {
        if (this.phase === "APPROACH")
          this.events.push({ t: this.t, name: "Fan seal / C-ramp entry" });
        this.phase = "RAMP";
        this.owner = "FAN + WHEEL DRIVE";
        this.maxFan = Math.max(this.maxFan, this.fan / mass / 9.81);
        if (normal < 0 || this.path >= this.ramp.release) {
          this.phase = "FLIP";
          this.release = this.t;
          this.releaseAngle = b.phi;
          this.events.push({ t: this.t, name: "Ramp release / flight MPC" });
        }
      }
    } else {
      const air = this.t - this.release,
        x = this.x,
        q = x.slice(7, 11),
        w = x.slice(11),
        I = inertia(x[0]);
      const u = clamp(air / 2, 0, 1),
        p = this.barrel
          ? 0
          : this.releaseAngle +
            (Math.PI - this.releaseAngle) *
              (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5),
        rel = qmul([Math.cos(p / 2), 0, Math.sin(p / 2), 0], q),
        ry = rotate(rel, [0, 1, 0]),
        rz = rotate(rel, [0, 0, 1]),
        roll = Math.atan2(ry[2], rz[2]);
      if (this.land === null) {
        this.maneuver += Math.atan2(
          Math.sin(roll - this.previousRoll),
          Math.cos(roll - this.previousRoll),
        );
        this.previousRoll = roll;
      }
      const previousPhase = this.phase;
      if (this.phase !== "SETTLED") {
        this.phase = this.barrel
          ? air < BARREL_LIFT
            ? "LIFTOFF"
            : air < BARREL_LIFT + BARREL_ROLL
              ? c.require_roll
                ? "BARREL_ROLL"
                : "CLEARANCE"
              : "DESCENT"
          : air < 2
            ? "FLIP"
            : "DESCENT";
        if (this.barrel && previousPhase !== this.phase)
          this.events.push({
            t: this.t,
            name:
              this.phase === "BARREL_ROLL"
                ? "Full roll begins"
                : this.phase === "CLEARANCE"
                  ? "Vehicle clearance"
                  : "Descent begins",
          });
        if (c.use_mpc) {
          if (this.t + 1e-8 >= this.next) {
            solve = this.plan();
            this.next = this.t + 0.12;
          }
          this.u = this.mpc.plan?.[0].slice() ?? Array(NOZZLE_COUNT).fill(0);
          this.owner =
            "ROADSTER SCvx MPC / " +
            (this.barrel
              ? this.phase.replace("_", " ")
              : air < 2
                ? "HALF-ROLL"
                : "DESCENT");
        } else {
          this.owner = "BALLISTIC / MPC OFF";
        }
        if (this.failure) return { done: true, solve };
      } else this.owner = "WHEEL CONTACT";
      let fg = [0, 0, 0],
        tg = [0, 0, 0];
      this.contacts = 0;
      for (const wheel of wheels) {
        const r = rotate(q, wheel),
          wp = add(x.slice(1, 4), r),
          wv = add(x.slice(4, 7), rotate(q, cross(w, wheel)));
        if (wp[2] < 0) {
          this.contacts++;
          const n = Math.max(0, -180000 * wp[2] - 11000 * wv[2]);
          let tang = [-wv[0] * x[0] * 2, -wv[1] * x[0] * 2, 0];
          tang = scale(
            tang,
            Math.min(1, (0.9 * n) / Math.max(norm(tang), 1e-9)),
          );
          const f = add(tang, [0, 0, n]);
          fg = add(fg, f);
          tg = add(tg, cross(wheel, invRotate(q, f)));
        }
      }
      let lowest = Infinity;
      for (const xx of [-2.1, 2.1])
        for (const yy of [-0.9, 0.9])
          for (const zz of [-0.25, 0.6])
            lowest = Math.min(lowest, rotate(q, [xx, yy, zz])[2] + x[3]);
      if (this.barrel) {
        const points = [...wheels];
        for (const xx of [-2.1, 2.1])
          for (const yy of [-0.9, 0.9])
            for (const zz of [-0.25, 0.6]) points.push([xx, yy, zz]);
        const gap = stageClearance(
          points.map((p) => add(x.slice(1, 4), rotate(q, p))),
        );
        this.minStageClearance = Math.min(
          this.minStageClearance,
          gap.clearance,
        );
        if (gap.clearance < 0) {
          this.failure = "Display vehicle impact: " + gap.vehicle;
          this.phase = "CRASH";
        }
      }
      if (lowest < -0.08) {
        this.failure = "Body impact";
        this.phase = "CRASH";
      }
      if (!this.contacts && x[3] > 0.72) this.hasLeftGround = true;
      if (
        this.contacts &&
        this.touch === null &&
        (!this.barrel || this.hasLeftGround)
      ) {
        this.touch = {
          t: this.t,
          position: x.slice(1, 4),
          vertical_speed: Math.abs(x[6]),
          horizontal_speed: norm(x.slice(4, 6)),
          tilt_degrees:
            (Math.acos(clamp(rotate(q, [0, 0, 1])[2], -1, 1)) * 180) / Math.PI,
          angular_rate: norm(w),
          roll_degrees: this.roll,
        };
        this.land = this.t;
        this.phase = "SETTLED";
        this.u.fill(0);
        this.events.push({ t: this.t, name: "First wheel contact" });
        if (this.touch.vertical_speed > 2 || this.touch.tilt_degrees > 10)
          this.failure = "Hard or tilted touchdown";
      }
      const f = wrench(this.u),
        v = x.slice(4, 7),
        drag = scale(v, -0.42 * norm(v)),
        a = add(
          [0, 0, -9.81],
          scale(add(add(rotate(q, f.slice(0, 3)), drag), fg), 1 / x[0]),
        ),
        alpha = sub(
          add(f.slice(3), tg),
          cross(
            w,
            w.map((v, i) => v * I[i]),
          ),
        ).map((v, i) => v / I[i]);
      const nv = add(v, scale(a, dt)),
        nw = scale(add(w, scale(alpha, dt)), Math.exp(-0.025 * dt)),
        angle = norm(nw) * dt,
        dq = [
          Math.cos(angle / 2),
          ...scale(
            nw,
            dt * (angle > 1e-12 ? Math.sin(angle / 2) / angle : 0.5),
          ),
        ];
      x.splice(4, 3, ...nv);
      x.splice(1, 3, ...add(x.slice(1, 4), scale(nv, dt)));
      x.splice(11, 3, ...nw);
      x.splice(7, 4, ...unit(qmul(q, dq)));
      x[0] = Math.max(
        1428,
        x[0] - (this.u.reduce((a, b) => a + b, 0) * dt) / c.exhaust_mps,
      );
      this.roll += (nw[0] * dt * 180) / Math.PI;
    }
    this.maxHeight = Math.max(this.maxHeight, this.x[3]);
    this.t += dt;
    return {
      done:
        !!this.failure ||
        this.t >= 24 ||
        (this.land !== null && this.t - this.land > (this.barrel ? 4 : 2)),
      solve,
    };
  }
  frame() {
    const x = this.x;
    return {
      t: this.t,
      p: x.slice(1, 4),
      v: x.slice(4, 7),
      q: x.slice(7, 11),
      omega: x.slice(11),
      mass: x[0],
      gas: x[0] - 1428,
      phase: this.phase,
      controller: this.owner,
      jets: this.u.slice(),
      fan_g: this.fan / x[0] / 9.81,
      roll: this.roll,
      contact: this.contacts,
      residual: 0,
      mpc_id: this.solves.at(-1)?.id ?? null,
      delivered_force: rotate(x.slice(7, 11), wrench(this.u).slice(0, 3)),
    };
  }
  summary() {
    const success =
      !!this.touch &&
      !this.failure &&
      this.touch.vertical_speed < 1 &&
      this.touch.tilt_degrees < 5 &&
      this.touch.horizontal_speed < (this.barrel ? this.config.speed + 1 : 4) &&
      (this.barrel && !this.config.require_roll
        ? Math.abs((this.maneuver * 180) / Math.PI) < 10
        : (this.maneuver * 180) / Math.PI > (this.barrel ? 350 : 170)) &&
      (!this.barrel || (this.maneuver * 180) / Math.PI < 370) &&
      norm(this.x.slice(4, 7)) < 0.15 &&
      ((this.barrel && !this.config.goal_planning) ||
        Math.abs(this.touch.position[0] - this.target) < 1);
    return {
      success,
      planning: this.flightPlan
        ? {
            mode: "terminal-goal SCvx",
            seconds: this.flightPlan.seconds,
            duration: this.flightPlan.duration,
            terminal_error: this.flightPlan.terminal_error,
            require_roll: this.flightPlan.require_roll,
          }
        : null,
      failure: this.failure ?? (success ? null : "Acceptance criteria not met"),
      touchdown: this.touch,
      start_position: [this.layout.start, 0, 0.62],
      landing_target: [this.target, 0, 0.62],
      landing_error: this.touch
        ? Math.hypot(
            this.touch.position[0] - this.target,
            this.touch.position[1],
          )
        : null,
      stop_error: Math.hypot(this.x[1] - this.target, this.x[2]),
      max_height: this.maxHeight,
      roll_degrees: this.roll,
      maneuver_roll_degrees: (this.maneuver * 180) / Math.PI,
      gas_remaining: this.x[0] - 1428,
      gas_used: 1428 + this.config.gas_kg - this.x[0],
      mpc_accepted: this.solves.filter((s) => s.success).length,
      mpc_rejected: this.solves.filter((s) => !s.success).length,
      settled_speed: norm(this.x.slice(4, 7)),
      max_fan_g: this.maxFan,
      stage_clearance: Number.isFinite(this.minStageClearance)
        ? this.minStageClearance
        : null,
    };
  }
}
