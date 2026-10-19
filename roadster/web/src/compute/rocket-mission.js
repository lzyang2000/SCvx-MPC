import { Scvx } from "./scvx.js";
import { sampleRocketPhasePlan } from "./rocket-planner.js";
import {
  ROCKET,
  properties,
  rocketModel,
  aerodynamicForce,
} from "./rocket-dynamics.js";
import {
  add,
  sub,
  scale,
  norm,
  dot,
  cross,
  rotate,
  qmul,
  unit,
  qblend,
  clamp,
} from "./math.js";

// Vertical reference rollout includes propellant depletion and atmospheric drag.
// Find where a constant-thrust burn stops, without changing the plant state.
function stoppingHeight(mass, velocity, height, thrust) {
  const flow = thrust / ROCKET.isp;
  const area = ROCKET.axialCd * Math.PI * 1.83 ** 2;
  const acceleration = (h, v, m) =>
    thrust / m -
    9.81 +
    (0.5 *
      ROCKET.airDensity *
      Math.exp(-Math.max(h, 0) / ROCKET.atmosphereHeight) *
      area *
      v *
      v) /
      m;
  for (let i = 0; i < 2000; i++) {
    if (velocity >= 0) return height;
    if (mass - flow * 0.1 <= ROCKET.dry) return -Infinity;
    const dt = 0.1,
      a = acceleration(height, velocity, mass);
    const v2 = velocity + (a * dt) / 2,
      h2 = height + (velocity * dt) / 2,
      m2 = mass - (flow * dt) / 2;
    const a2 = acceleration(h2, v2, m2),
      v3 = velocity + (a2 * dt) / 2;
    const a3 = acceleration(height + (v2 * dt) / 2, v3, m2),
      v4 = velocity + a3 * dt;
    const a4 = acceleration(height + v3 * dt, v4, mass - flow * dt);
    const next = velocity + (dt * (a + 2 * a2 + 2 * a3 + a4)) / 6;
    if (next >= 0)
      return height - velocity ** 2 / (2 * ((next - velocity) / dt));
    height += (dt * (velocity + 2 * v2 + 2 * v3 + v4)) / 6;
    velocity = next;
    mass -= flow * dt;
  }
  return -Infinity;
}
function brakingThrust(mass, velocity, height, min, max, targetHeight = 0) {
  let lo = min,
    hi = max;
  for (let i = 0; i < 20; i++) {
    const thrust = (lo + hi) / 2;
    if (stoppingHeight(mass, velocity, height, thrust) < targetHeight)
      lo = thrust;
    else hi = thrust;
  }
  return (lo + hi) / 2;
}
// A height-dependent reference lowers briskly, then eases toward foot contact.
// The same reference acceleration seeds the horizon; SCvx supplies plant forces.
function finalDescentAcceleration(height, velocity) {
  const speed = Math.min(
    ROCKET.descentSpeed,
    Math.sqrt(
      ROCKET.landingSpeed ** 2 +
        2 * ROCKET.descentDeceleration * Math.max(0, height),
    ),
  );
  const braking =
    speed < ROCKET.descentSpeed
      ? (-ROCKET.descentDeceleration * velocity) / speed
      : 0;
  return clamp(braking + (-speed - velocity) / ROCKET.descentResponse, -2, 2);
}
/** MuJoCo contacts + SCvx commands. No PD controller or terminal PD handoff. */
export class RocketMission {
  constructor(
    mj,
    xml,
    {
      separationAltitude = ROCKET.separationAltitude,
      initialFuel = ROCKET.initialFuel,
    } = {},
  ) {
    if (
      !Number.isFinite(separationAltitude) ||
      separationAltitude < 70000 ||
      separationAltitude > 80000
    )
      throw Error("Separation altitude must be between 70 and 80 km");
    if (
      !Number.isFinite(initialFuel) ||
      initialFuel <= 0 ||
      initialFuel > ROCKET.fuel
    )
      throw Error(
        "Initial propellant must be positive and within tank capacity",
      );
    this.initialFuel = initialFuel;
    this.mj = mj;
    this.xml = xml;
    this.model = mj.MjModel.from_xml_string(xml);
    this.data = new mj.MjData(this.model);
    const id = (type, name) => {
      const handle = this.model[type](name),
        value = handle.id;
      handle.delete();
      return value;
    };
    this.bid = id("body", "rocket");
    this.uid = id("body", "separated_upper_stage");
    this.separationAltitude = separationAltitude;
    this.separationState = null;
    this.fuel = initialFuel;
    this.upperFuel = ROCKET.upperFuel;
    this.attached = true;
    this.phase = "ASCENT";
    this.separationTime = null;
    this.cutoffTime = null;
    this.upperIgnitionTime = null;
    this.control = [0, 0, 0, 0, 0, 0];
    this.solves = [];
    this.lastSolve = -Infinity;
    this.lastMass = -Infinity;
    this.lastFrame = -Infinity;
    this.events = [];
    this.legs = 0;
    this.ignitions = 1;
    this.failure = null;
    this.settledAt = null;
    this.landingCleared = false;
    this.landingEngineCount = 3;
    this.hoverStartedAt = null;
    this.hoverEnabled = null;
    this.descentStartedAt = null;
    this.mpc = new Scvx({
      nu: 6,
      n: 10,
      scaleX: [550000, 1000, 1000, 1000, 200, 200, 200, 1, 1, 1, 1, 1, 1, 1],
      scaleU: [8e6, 8e6, 8e6, 2e6, 2e6, 17500],
    });
    this.mount = id("geom", "launch_mount");
    this.feet = ["xp", "xm", "yp", "ym"].map((s) => id("geom", "foot_" + s));
    for (const s of ["xp", "xm", "yp", "ym"])
      for (const part of ["leg_", "strut_"]) {
        const g = id("geom", part + s);
        this.model.geom_contype[g] = 0;
        this.model.geom_conaffinity[g] = 0;
      }
    this.model.geom_rgba[this.mount * 4 + 3] = 0;
    this.massUpdate(true);
    mj.mj_forward(this.model, this.data);
  }
  dispose() {
    this.data.delete();
    this.model.delete();
  }
  landingHeight() {
    const raw = this.data.qpos[2] - ROCKET.landedZ;
    if (this.phase !== "LANDING" || raw > 30) return Math.max(0, raw);
    // Lowest ellipsoid support point along world-up, including body tilt.
    const q = Array.from(this.data.qpos.slice(3, 7)),
      up = rotate([q[0], -q[1], -q[2], -q[3]], [0, 0, 1]);
    let bottom = Infinity;
    for (const id of this.feet) {
      const p = Array.from(this.model.geom_pos.slice(id * 3, id * 3 + 3)),
        size = Array.from(this.model.geom_size.slice(id * 3, id * 3 + 3));
      bottom = Math.min(
        bottom,
        dot(up, p) - norm(size.map((v, i) => v * up[i])),
      );
    }
    return Math.max(0, this.data.qpos[2] + bottom - ROCKET.padHeight);
  }
  state() {
    const q = Array.from(this.data.qpos.slice(3, 7)),
      p = properties(
        ROCKET.dry + this.fuel + (this.attached ? 111500 : 0),
        this.attached ? 111500 : 0,
      ),
      w = Array.from(this.data.qvel.slice(3, 6));
    return [
      p.mass,
      ...add(Array.from(this.data.qpos.slice(0, 3)), rotate(q, [0, 0, p.z])),
      ...add(
        Array.from(this.data.qvel.slice(0, 3)),
        rotate(q, cross(w, [0, 0, p.z])),
      ),
      ...q,
      ...w,
    ];
  }
  massUpdate(force = false) {
    const { mj, model: m, data: d } = this;
    if (!force && d.time - this.lastMass < 0.1) return;
    const p = properties(
      ROCKET.dry + this.fuel + (this.attached ? 111500 : 0),
      this.attached ? 111500 : 0,
    );
    m.body_mass[this.bid] = p.mass;
    m.body_ipos[this.bid * 3 + 2] = p.z;
    m.body_inertia.set(p.I, this.bid * 3);
    m.body_mass[this.uid] = 4000 + this.upperFuel;
    m.body_inertia.set(
      [7767663, 7767663, 192010].map(
        (v) => (v * (4000 + this.upperFuel)) / 111500,
      ),
      this.uid * 3,
    );
    const pos = d.qpos.slice(),
      vel = d.qvel.slice(),
      t = d.time;
    mj.mj_setConst(m, d);
    d.qpos.set(pos);
    d.qvel.set(vel);
    d.time = t;
    mj.mj_forward(m, d);
    this.lastMass = t;
  }
  transition(phase) {
    this.phase = phase;
    this.phaseStart = this.data.time;
    this.mpc.reset();
    this.lastSolve = -Infinity;
    this.events.push({ t: this.data.time, name: phase });
    if (this.flightPlan && ["BOOSTBACK", "ENTRY", "LANDING"].includes(phase))
      this.pendingReplan = phase;
  }

  separate() {
    const d = this.data,
      q = Array.from(d.qpos.slice(3, 7));
    const velocity = Array.from(d.qvel.slice(0, 3));
    const boosterMass = ROCKET.dry + this.fuel;
    const upperMass = ROCKET.upperDry + this.upperFuel;
    this.separationState = {
      altitude: this.landingHeight(),
      velocity: velocity.slice(),
      speed: norm(velocity),
      fuel: this.fuel,
      booster_mass: boosterMass,
      ballistic_apogee:
        this.landingHeight() + Math.max(velocity[2], 0) ** 2 / (2 * 9.81),
    };
    this.separationAxis = rotate(q, [0, 0, 1]);
    // Opposite axial impulses give 3 m/s relative separation and conserve momentum.
    const impulse = 3 / (1 / boosterMass + 1 / upperMass);
    d.qpos.set(d.qpos.slice(0, 7), 7);
    d.qvel.set(
      add(velocity, scale(this.separationAxis, impulse / upperMass)),
      6,
    );
    d.qvel.set(
      sub(velocity, scale(this.separationAxis, impulse / boosterMass)),
      0,
    );
    d.qvel.set(d.qvel.slice(3, 6), 9);
    this.attached = false;
    this.separationTime = d.time;
    this.massUpdate(true);
    this.control.fill(0);
    this.activeCount = 0;
    this.transition("SEPARATION");
  }
  stageClearance() {
    return dot(
      sub(
        Array.from(this.data.qpos.slice(7, 10)),
        Array.from(this.data.qpos.slice(0, 3)),
      ),
      this.separationAxis,
    );
  }
  guidance() {
    const x = this.state(),
      h = this.landingHeight(),
      vz = x[6],
      g = 9.81;
    if (
      this.phase === "ASCENT" &&
      (h +
        Math.max(vz, 0) * ROCKET.separationDelay -
        0.5 * g * ROCKET.separationDelay ** 2 >=
        this.separationAltitude ||
        this.fuel < 15000)
    ) {
      this.cutoffTime = this.data.time;
      this.control.fill(0);
      this.activeCount = 0;
      this.transition("MECO");
    }
    if (
      this.phase === "MECO" &&
      this.data.time - this.cutoffTime >= ROCKET.separationDelay
    )
      this.separate();
    if (!this.attached && this.separationTime !== null) {
      const elapsed = this.data.time - this.separationTime;
      const clearance = this.stageClearance();
      // Cold staging: upper ignition follows a coast and positive stage clearance.
      if (
        this.upperIgnitionTime === null &&
        elapsed >= ROCKET.upperIgnitionDelay &&
        clearance >= 5
      ) {
        this.upperIgnitionTime = this.data.time;
        this.events.push({ t: this.data.time, name: "UPPER IGNITION" });
      }
      const axis = rotate(x.slice(7, 11), [0, 0, 1]),
        boostAxis = [
          -ROCKET.downrangeSign * Math.sin((75 * Math.PI) / 180),
          0,
          Math.cos((75 * Math.PI) / 180),
        ];
      if (
        this.phase === "SEPARATION" &&
        elapsed >= 6 &&
        clearance >= 30 &&
        dot(axis, boostAxis) > Math.cos((8 * Math.PI) / 180)
      ) {
        this.ignitions++;
        this.transition("BOOSTBACK");
      }
      if (this.phase === "SEPARATION" && elapsed > 60) {
        this.failure = "Stage clearance or boostback alignment not achieved";
        return false;
      }
    }
    if (this.phase === "BOOSTBACK") {
      const fall = (vz + Math.sqrt(vz * vz + 2 * g * h)) / g,
        impact = x[1] + x[4] * fall;
      if (ROCKET.downrangeSign * (impact - ROCKET.padX) < 20)
        this.transition("COAST");
    }
    if (this.phase === "COAST" && vz < 0 && h < 60000) {
      this.ignitions++;
      this.transition("ENTRY");
    }
    if (this.phase === "ENTRY" && this.data.time - this.phaseStart >= 12) {
      this.transition("DESCENT_COAST");
    }
    if (
      this.phase === "DESCENT_COAST" &&
      this.data.time - this.phaseStart >= 3 &&
      vz < 0
    ) {
      const stop = stoppingHeight(x[0], vz, h, 0.95 * 3 * ROCKET.engineMax);
      if (stop < 1500) {
        this.ignitions++;
        this.transition("LANDING");
      }
    }
    if (this.phase === "LANDING" && this.landingEngineCount === 3) {
      const stop = stoppingHeight(x[0], vz, h, 0.9 * ROCKET.engineMax);
      if (stop > 30) {
        this.landingEngineCount = 1;
        this.mpc.reset();
        this.lastSolve = -Infinity;
        this.events.push({ t: this.data.time, name: "CENTER ENGINE" });
        if (this.flightPlan) this.pendingReplan = "CENTER ENGINE";
      }
    }
    if (this.phase === "LANDING" && h < 80 && this.hoverEnabled === null) {
      // Reserve covers the approach, short dwell and lowering. Below this mass,
      // minimum engine thrust exceeds weight, so follow the braking corridor
      // all the way to contact instead of asking MPC for an impossible hover.
      const hoverFuel =
        (ROCKET.minThrottle * ROCKET.engineMax) / g - ROCKET.dry;
      this.hoverEnabled = this.fuel >= hoverFuel + 4000;
      if (!this.hoverEnabled)
        this.events.push({ t: this.data.time, name: "DIRECT LANDING" });
    }
    // Start the dwell clock only after the simulated booster has actually
    // reached the hover point and settled, rather than on entry to LANDING.
    const hoverReady =
      this.hoverEnabled === true &&
      this.phase === "LANDING" &&
      Math.abs(h - ROCKET.hoverHeight) < 0.25 &&
      Math.hypot(x[1] - ROCKET.padX, x[2]) < 2 &&
      norm(x.slice(4, 6)) < 0.3 &&
      Math.abs(vz) < 0.15;
    if (!this.landingCleared && !hoverReady) this.hoverStartedAt = null;
    if (hoverReady && this.hoverStartedAt === null) {
      this.hoverStartedAt = this.data.time;
      this.events.push({ t: this.data.time, name: "HOVER" });
    }
    if (
      this.hoverStartedAt !== null &&
      !this.landingCleared &&
      this.data.time - this.hoverStartedAt >= ROCKET.hoverSeconds - 1e-8
    ) {
      this.landingCleared = true;
      this.descentStartedAt = this.data.time;
      this.events.push({ t: this.data.time, name: "SLOW DESCENT" });
    }
    if (["ENTRY", "LANDING"].includes(this.phase) && h < 1000)
      this.legs = clamp(this.legs + this.model.opt.timestep / 1.25, 0, 1);
    if (["ENTRY", "LANDING"].includes(this.phase) && this.data.ncon) {
      const contacts = this.data.contact;
      let feetContact = false;
      for (let i = 0; i < contacts.size(); i++) {
        const c = contacts.get(i);
        if (this.feet.includes(c.geom1) || this.feet.includes(c.geom2))
          feetContact = true;
        c.delete();
      }
      contacts.delete();
      if (feetContact) {
        const tilt =
          (Math.acos(clamp(rotate(x.slice(7, 11), [0, 0, 1])[2], -1, 1)) *
            180) /
          Math.PI;
        this.touchdown = {
          t: this.data.time,
          vertical_speed: Math.abs((this.preStepVelocity ?? x.slice(4, 7))[2]),
          horizontal_speed: norm(
            (this.preStepVelocity ?? x.slice(4, 7)).slice(0, 2),
          ),
          tilt_degrees: tilt,
        };
        this.control.fill(0);
        this.activeCount = 0;
        this.settledAt = this.data.time;
        this.transition("CONTACT");
        if (
          this.touchdown.vertical_speed > 2 ||
          this.touchdown.horizontal_speed > 3 ||
          tilt > 6
        )
          this.failure = "Hard or tilted foot contact";
      }
    }
    if (this.fuel <= 0) {
      this.failure = "Booster propellant depleted";
      return false;
    }
    if (this.data.qpos[2] < 18 && this.phase !== "CONTACT") {
      this.failure = "Booster impact";
      return false;
    }
    return true;
  }
  plan() {
    const x = this.state(),
      h = this.landingHeight(),
      phase = this.phase,
      coast = ["MECO", "SEPARATION", "COAST", "DESCENT_COAST"].includes(phase);
    const count = coast
      ? 0
      : phase === "ASCENT"
        ? 9
        : phase === "LANDING"
          ? this.landingEngineCount
          : 3;
    const max = coast ? 0 : count * ROCKET.engineMax,
      min = ROCKET.minThrottle * max;
    const descent = phase === "LANDING";
    const hold =
      descent && h < 80 && this.hoverEnabled === true && !this.landingCleared;
    const slowDescent = descent && this.landingCleared;
    // Keep the terminal horizon within contact time and available propellant.
    const dt = descent
      ? Math.min(
          this.landingEngineCount === 3 ? 0.06 : hold || h > 80 ? 0.25 : 0.12,
          Math.max(
            0.002,
            ((2 * h) / Math.max(Math.abs(x[6]), 0.05) + 0.02) / 16,
          ),
          Math.max(0.02, (this.fuel - 60) / (min / ROCKET.isp) / 17),
        )
      : coast
        ? 0.15
        : phase === "ENTRY"
          ? 0.12
          : 0.4;
    this.mpc.n =
      descent && this.flightPlan && count === 3
        ? 32
        : descent || coast
          ? 16
          : 10;
    const model = rocketModel({
      attached: this.attached ? 111500 : 0,
      min,
      max,
      coast,
      isp: ROCKET.isp,
    });
    let descentTarget = 0,
      descentAccel = 0,
      lateralY = 0,
      angle = 0,
      acc = [0, 0, 0],
      directionOverride = null,
      thrust = max,
      weights = [0, 5, 5, 5, 10, 10, 15, 40, 40, 40, 40, 8, 8, 8];
    if (phase === "ASCENT") {
      let u = clamp((h - 1000) / 44000, 0, 1);
      u = u * u * (3 - 2 * u);
      angle =
        ((ROCKET.downrangeSign * ROCKET.ascentPitch * Math.PI) / 180) * u;
      acc = add(
        scale([Math.sin(angle), 0, Math.cos(angle)], max / x[0]),
        add([0, 0, -9.81], scale(aerodynamicForce(x, true), 1 / x[0])),
      );
    } else if (phase === "BOOSTBACK") {
      angle = (-ROCKET.downrangeSign * 75 * Math.PI) / 180;
      acc = add(
        scale([Math.sin(angle), 0, Math.cos(angle)], max / x[0]),
        [0, 0, -9.81],
      );
    } else if (phase === "ENTRY") {
      const az = Math.min(18, (0.95 * max) / x[0] - 9.81);
      descentAccel = az;
      acc = [0, 0, az];
      thrust = clamp(x[0] * (9.81 + az), min, max);
    } else if (coast) {
      angle =
        phase === "SEPARATION"
          ? (-ROCKET.downrangeSign * 75 * Math.PI) / 180
          : phase === "MECO"
            ? Math.atan2(
                rotate(x.slice(7, 11), [0, 0, 1])[0],
                rotate(x.slice(7, 11), [0, 0, 1])[2],
              )
            : 0;
      acc = add(
        [0, 0, -9.81],
        scale(aerodynamicForce(x, this.attached), 1 / x[0]),
      );
      thrust = 0;
    } else {
      // Reference guidance supplies an energy corridor; optimization chooses all forces.
      const vz = x[6];
      const dragAcceleration = scale(aerodynamicForce(x, false), 1 / x[0]);
      const brake = brakingThrust(
        x[0],
        vz,
        h,
        min,
        0.95 * max,
        this.landingEngineCount === 3 ? 1500 : 0,
      );
      const target = vz;
      const targetAccel = brake / x[0] - 9.81 + dragAcceleration[2];
      const az = hold
        ? clamp(
            -2 * ROCKET.hoverResponse * vz -
              ROCKET.hoverResponse ** 2 * (h - ROCKET.hoverHeight),
            -6,
            25,
          )
        : slowDescent
          ? finalDescentAcceleration(h, vz)
          : clamp(targetAccel + (target - vz) / 2, -6, 25);
      descentTarget = target;
      descentAccel = hold || slowDescent ? az : targetAccel;
      const flightTime = Math.max(2, (2 * h) / Math.max(Math.abs(vz), 0.5));
      let ax = clamp(
        ((6 * (ROCKET.padX - x[1])) / flightTime - 4 * x[4]) / flightTime,
        -12,
        12,
      );
      let ay = clamp(
        (2 * (-x[2] - x[5] * flightTime)) / (flightTime * flightTime),
        -3,
        3,
      );
      lateralY = ay;
      acc = [ax, ay, az];
      const required = sub([ax, ay, 9.81 + az], dragAcceleration);
      directionOverride = unit(required);
      angle = Math.atan2(required[0], required[2]);
      thrust = clamp(x[0] * norm(required), min, max);
      const proximity = clamp(100 / Math.max(h, 1), 1, 4);
      weights = [
        0,
        5 * proximity,
        5 * proximity,
        12 * proximity,
        30,
        30,
        45,
        45,
        45,
        45,
        45,
        12,
        12,
        12,
      ];
    }
    const direction =
        directionOverride ??
        unit([
          Math.sin(angle),
          lateralY / Math.max(9.81 + acc[2], 1),
          Math.cos(angle),
        ]),
      qw = Math.sqrt((1 + direction[2]) / 2);
    const desired = [qw, -direction[1] / (2 * qw), direction[0] / (2 * qw), 0],
      refs = [];
    const lateralTime = Math.max(2, (2 * h) / Math.max(Math.abs(x[6]), 0.5));
    const horizontalAcceleration = (p, v, t) => {
      const remaining = Math.max(2, lateralTime - t);
      return p.map((value, i) =>
        clamp(
          (6 * ((i === 0 ? ROCKET.padX : 0) - value)) / remaining ** 2 -
            (4 * v[i]) / remaining,
          -8,
          8,
        ),
      );
    };
    let finalHeight = h,
      finalVelocity = x[6],
      hp = x.slice(1, 3),
      hv = x.slice(4, 6),
      ha = [0, 0];
    for (let k = 0; k <= this.mpc.n; k++) {
      const t = k * dt,
        r = x.slice();
      const p = add(
        add(x.slice(1, 4), scale(x.slice(4, 7), t)),
        scale(acc, 0.5 * t * t),
      );
      r.splice(1, 3, ...p);
      r.splice(4, 3, ...add(x.slice(4, 7), scale(acc, t)));
      if (hold) {
        // This is an optimization reference; only accepted SCvx forces are
        // applied to MuJoCo. Actual height/velocity gate the hover dwell.
        const rate = ROCKET.hoverResponse,
          decay = Math.exp(-rate * t),
          e = h - ROCKET.hoverHeight,
          c = x[6] + rate * e;
        r[3] = x[3] - h + ROCKET.hoverHeight + (e + c * t) * decay;
        r[6] = (x[6] - rate * c * t) * decay;
      } else if (slowDescent) {
        if (k) {
          const steps = Math.max(1, Math.ceil(dt / 0.01)),
            ds = dt / steps;
          for (let j = 0; j < steps; j++) {
            const a = finalDescentAcceleration(finalHeight, finalVelocity);
            finalHeight += finalVelocity * ds + 0.5 * a * ds * ds;
            finalVelocity += a * ds;
          }
        }
        r[3] = x[3] - h + finalHeight;
        r[6] = finalVelocity;
      } else if (descent) {
        const stop = Math.min(
            t,
            Math.abs(descentTarget) / Math.max(descentAccel, 0.1),
          ),
          height = Math.max(
            0,
            h + descentTarget * stop + 0.5 * descentAccel * stop * stop,
          );
        r[3] = x[3] - h + height;
        r[6] = Math.min(-0.15, descentTarget + descentAccel * t);
      }
      let futureDesired = desired;
      if ((descent || phase === "ENTRY") && h < 100000) {
        if (k)
          for (let j = 0; j < 20; j++) {
            const ds = dt / 20;
            ha = horizontalAcceleration(hp, hv, t - dt + j * ds);
            hp = hp.map((v, i) => v + hv[i] * ds + 0.5 * ha[i] * ds * ds);
            hv = hv.map((v, i) => v + ha[i] * ds);
          }
        ha = horizontalAcceleration(hp, hv, t);
        r[1] = hp[0];
        r[2] = hp[1];
        r[4] = hv[0];
        r[5] = hv[1];
        const drag =
          phase === "LANDING"
            ? scale(aerodynamicForce(x, false), 1 / x[0])
            : [0, 0, 0];
        const d =
            descent && h < 300
              ? unit([
                  clamp(ha[0], -0.8, 0.8) * clamp((h - 50) / 250, 0, 1),
                  clamp(ha[1], -0.8, 0.8) * clamp((h - 50) / 250, 0, 1),
                  9.81 + descentAccel,
                ])
              : unit(sub([ha[0], ha[1], 9.81 + descentAccel], drag)),
          w = Math.sqrt((1 + d[2]) / 2);
        futureDesired = [w, -d[1] / (2 * w), d[0] / (2 * w), 0];
      }
      const q = qblend(
        x.slice(7, 11),
        futureDesired,
        clamp(
          t / (descent && h < 300 ? 1 : phase === "BOOSTBACK" ? 2 : 4),
          0,
          1,
        ),
      );
      r.splice(7, 4, ...q);
      r.splice(11, 3, 0, 0, 0);
      refs.push(r);
    }
    // Follow the optimized mission in flight. Close to contact, retain a
    // goal-based braking horizon from the measured foot clearance; the same
    // six-DoF MPC still computes the commands, including terminal corrections.
    if (this.flightPlan && !(descent && h < 300)) {
      if (descent)
        weights = [0, 15, 15, 15, 45, 45, 45, 35, 35, 35, 35, 12, 12, 12];
      const now = this.data.time;
      for (let k = 0; k <= this.mpc.n; k++) {
        const sample = sampleRocketPhasePlan(
          this.flightPlan,
          now + k * dt,
          phase,
          count,
        );
        const mass = sample.state[0] + (this.attached ? 111500 : 0);
        const prop = properties(mass, this.attached ? 111500 : 0);
        let targetQ = sample.q;
        if (norm(sample.force) > 1 && norm(sample.nominalForce) > 1) {
          const from = unit(sample.nominalForce),
            to = unit(sample.force);
          const delta = unit([1 + dot(from, to), ...cross(from, to)]);
          targetQ = unit(qmul(delta, sample.q));
        }
        const q = targetQ;
        refs[k] = [
          mass,
          ...add(sample.state.slice(1, 4), rotate(q, [0, 0, prop.z])),
          ...sample.state.slice(4),
          ...q,
          0,
          0,
          0,
        ];
      }
      const force = sampleRocketPhasePlan(
        this.flightPlan,
        now,
        phase,
        count,
      ).force;
      thrust = clamp(norm(force), min, max);
    }
    // Differentiate the attitude reference instead of asking for zero rate during a turn.
    for (let k = 0; k < refs.length - 1; k++) {
      const q = refs[k].slice(7, 11),
        next = refs[k + 1].slice(7, 11),
        dq = qmul([q[0], -q[1], -q[2], -q[3]], next);
      refs[k].splice(11, 3, ...scale(dq.slice(1), 2 / dt));
    }
    const out = this.mpc.solve(x, refs, model, {
      dt,
      weights,
      seed: () => [0, 0, thrust, 0, 0, 0],
      iterations: 16,
      trust: 0.12,
      shift: (this.interval ?? 0.4) / dt,
      ground: -1e9,
      dry: ROCKET.dry + (this.attached ? 111500 : 0),
    });
    const s = {
      ...out.snapshot,
      t: this.data.time,
      phase,
      model: "MuJoCo WASM / SCvx booster",
      engine_count: count,
    };
    this.solves.push(s);
    this.lastSolve = this.data.time;
    this.interval =
      phase === "LANDING" && this.landingEngineCount === 3
        ? 0.1
        : h < 5 && phase !== "ASCENT"
          ? 0.01
          : h < 30 && phase !== "ASCENT"
            ? 0.04
            : h < 300 && phase !== "ASCENT"
              ? 0.2
              : 0.4;
    if (!out.command) {
      this.failure = `SCvx ${s.status}; nonlinear defect ${s.defect}; no PD fallback`;
      return s;
    }
    this.control = out.command;
    this.activeCount = coast ? 0 : count;
    return s;
  }
  updateLegs() {
    const m = this.model,
      d = this.legs;
    for (const [suffix, axis, sign] of [
      ["xp", 0, 1],
      ["xm", 0, -1],
      ["yp", 1, 1],
      ["ym", 1, -1],
    ]) {
      const foot = [0, 0, -3.1 + (-20.38 + 3.1) * d];
      foot[axis] = sign * (2.05 + (8.5 - 2.05) * d);
      const gid = this.feet[["xp", "xm", "yp", "ym"].indexOf(suffix)];
      m.geom_pos.set(foot, gid * 3);
      // MuJoCo physical foot contact follows the same animated endpoint as Three.js.
    }
  }
  step() {
    if (this.failure) return { done: true };
    if (this.phase === "CONTACT") {
      this.control.fill(0);
      if (this.data.time - this.settledAt > 12) return { done: true };
    } else if (!this.guidance()) return { done: true };
    if (this.pendingReplan) return { planning: true, done: false };
    let solve = null;
    if (
      this.phase !== "CONTACT" &&
      this.data.time - this.lastSolve >= (this.interval ?? 0)
    )
      solve = this.plan();
    if (this.failure) return { done: true, solve };
    const { mj, model: m, data: d } = this,
      u = this.control,
      q = Array.from(d.qpos.slice(3, 7));
    if (d.qpos[2] > 20.82) {
      m.geom_contype[this.mount] = 0;
      m.geom_conaffinity[this.mount] = 0;
    }
    this.updateLegs();
    this.massUpdate();
    d.xfrc_applied.fill(0);
    const prop = properties(
      ROCKET.dry + this.fuel + (this.attached ? 111500 : 0),
      this.attached ? 111500 : 0,
    );
    d.xfrc_applied.set(
      [
        ...add(
          rotate(q, u.slice(0, 3)),
          aerodynamicForce(this.state(), this.attached),
        ),
        ...rotate(
          q,
          add(cross([0, 0, -20.1 - prop.z], u.slice(0, 3)), u.slice(3, 6)),
        ),
      ],
      this.bid * 6,
    );
    if (this.attached)
      d.xfrc_applied.set(
        [0, 0, (4000 + this.upperFuel) * 9.81, 0, 0, 0],
        this.uid * 6,
      );
    else if (this.upperIgnitionTime !== null && this.upperFuel > 0) {
      const uq = Array.from(d.qpos.slice(10, 14));
      d.xfrc_applied.set(rotate(uq, [0, 0, 981000]), this.uid * 6);
      this.upperFuel = Math.max(
        0,
        this.upperFuel - (981000 / (348 * 9.80665)) * m.opt.timestep,
      );
    }
    this.fuel = Math.max(
      0,
      this.fuel - (norm(u.slice(0, 3)) / ROCKET.isp) * m.opt.timestep,
    );
    this.preStepVelocity = Array.from(d.qvel.slice(0, 3));
    mj.mj_step(m, d);
    if (d.time > 700 && this.phase !== "CONTACT")
      this.failure = "Mission time limit reached";
    return { done: !!this.failure || d.time > 700, solve };
  }
  frame() {
    const d = this.data;
    return {
      t: d.time,
      p: Array.from(d.qpos.slice(0, 3)),
      q: Array.from(d.qpos.slice(3, 7)),
      v: Array.from(d.qvel.slice(0, 3)),
      omega: Array.from(d.qvel.slice(3, 6)),
      fuel: this.fuel,
      mass: ROCKET.dry + this.fuel + (this.attached ? 111500 : 0),
      phase: this.phase,
      controller:
        this.phase === "CONTACT" ? "CONTACT / ENGINES OFF" : "SCvx MPC",
      control: this.control.slice(),
      engines: this.activeCount ?? 9,
      legs: this.legs,
      landing_hold:
        this.phase === "LANDING" &&
        this.data.qpos[2] - ROCKET.landedZ < 80 &&
        this.hoverEnabled === true &&
        !this.landingCleared,
      landing_mode:
        this.phase === "LANDING"
          ? this.hoverEnabled === false
            ? "direct-landing"
            : this.landingCleared
              ? "slow-descent"
              : this.hoverStartedAt !== null
                ? "hover"
                : this.landingHeight() < 80
                  ? "hover-approach"
                  : "braking"
          : null,
      attached: this.attached,
      mpc_id: this.solves.at(-1)?.id ?? null,
      upper: {
        p: Array.from(d.qpos.slice(7, 10)),
        q: Array.from(d.qpos.slice(10, 14)),
        v: Array.from(d.qvel.slice(6, 9)),
        fuel: this.upperFuel,
        thrust:
          !this.attached &&
          this.upperIgnitionTime !== null &&
          this.upperFuel > 0
            ? 981000
            : 0,
      },
    };
  }
  summary() {
    const settledSpeed = norm(Array.from(this.data.qvel.slice(0, 3))),
      q = Array.from(this.data.qpos.slice(3, 7)),
      tilt = (Math.acos(clamp(rotate(q, [0, 0, 1])[2], -1, 1)) * 180) / Math.PI,
      padError = Math.hypot(this.data.qpos[0] - ROCKET.padX, this.data.qpos[1]);
    const success =
      this.phase === "CONTACT" &&
      !this.failure &&
      settledSpeed < 0.15 &&
      tilt < 5 &&
      padError < 8;
    return {
      success,
      planning: this.flightPlan
        ? {
            seconds: (this.flightPlanHistory ?? [this.flightPlan]).reduce(
              (sum, plan) => sum + plan.seconds,
              0,
            ),
            duration: this.flightPlan.duration,
            terminal_error: this.flightPlan.terminal_error,
            model: this.flightPlan.model,
            replans: (this.flightPlanHistory?.length ?? 1) - 1,
          }
        : null,
      touchdown: this.touchdown ?? null,
      failure:
        this.failure ??
        (success ? null : "Landing acceptance criteria not met"),
      phase: this.phase,
      fuel_remaining: this.fuel,
      landing_profile: this.hoverEnabled === false ? "direct" : "hover",
      hover_started_at: this.hoverStartedAt,
      slow_descent_started_at: this.descentStartedAt,
      ignitions: this.ignitions,
      post_separation_ignitions: this.ignitions - 1,
      cutoff_time: this.cutoffTime,
      separation_time: this.separationTime,
      separation: this.separationState,
      upper_ignition_time: this.upperIgnitionTime,
      mpc_accepted: this.solves.filter((s) => s.success).length,
      mpc_rejected: this.solves.filter((s) => !s.success).length,
      start_position: [0, 0],
      landing_target: [ROCKET.padX, 0],
      pad_error: padError,
      settled_speed: settledSpeed,
      settled_tilt_degrees: tilt,
    };
  }
}
