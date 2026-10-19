import { CarMission, NOZZLES } from "./car-mission.js";
import { STAGE_VEHICLES, stageClearance } from "../car-stage.js";
import { add, scale, norm, rotate, invRotate, qmul, clamp } from "./math.js";

const WHEELS = [-1.4, 1.4].flatMap((x) =>
  [-0.92, 0.92].map((y) => [x, y, -0.25]),
);
const WHEEL_MASS = 18;
const WHEEL_I = [0.55, 0.99, 0.55];
const STEP = 0.002;
const fmt = (a) => a.join(" ");

export function carXml(config, start = -20) {
  const ramp = [];
  if (config.maneuver !== "barrel-roll") {
    const n = 160,
      r = config.radius,
      da = Math.PI / n;
    for (let i = 0; i < n; i++) {
      const a = (i + 0.5) * da;
      ramp.push(
        `<geom name="ramp_${i}" type="box" pos="${(r + 0.06) * Math.sin(a)} 0 ${r - (r + 0.06) * Math.cos(a)}" quat="${Math.cos(a / 2)} 0 ${-Math.sin(a / 2)} 0" size="${r * Math.tan(da / 2) + 0.001} 2.15 0.06"/>`,
      );
    }
  }
  const obstacles =
    config.maneuver === "barrel-roll"
      ? STAGE_VEHICLES.map(
          (v, i) =>
            `<geom name="vehicle_${i}" type="box" pos="${v.x} 0 ${v.size[2] / 2}" size="${fmt(v.size.map((s) => s / 2))}"/>`,
        ).join("\n")
      : "";
  return `<mujoco model="Six-jet Roadster">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="${STEP}" gravity="0 0 -9.81" integrator="implicitfast" solver="Newton" iterations="50"/>
  <default><geom friction="1.25 0.005 0.0001" condim="3" solref="0.012 1" solimp="0.95 0.99 0.001" contype="1" conaffinity="2"/></default>
  <worldbody>
    <geom name="ground" type="plane" size="200 200 0.1"/>
    ${ramp.join("\n")}
    ${obstacles}
    <body name="car" pos="${start} 0 0.62">
      <freejoint name="car_free"/>
      <inertial pos="0 0 0" mass="1606" diaginertia="700 2400 2700"/>
      <geom name="chassis" type="box" pos="0 0 0.15" size="2.1 0.9 0.4" contype="2" conaffinity="1"/>
      ${WHEELS.map((p, i) => `<body name="wheel_${i}" pos="${fmt(p)}"><inertial pos="0 0 0" mass="${WHEEL_MASS}" diaginertia="${fmt(WHEEL_I)}"/><joint name="axle_${i}" type="hinge" axis="0 1 0" damping="0.08"/><geom name="tire_${i}" type="ellipsoid" size="0.37 0.12 0.37" contype="2" conaffinity="1"/></body>`).join("\n")}
    </body>
  </worldbody>
</mujoco>`;
}

/** A MuJoCo plant. Only qpos/qvel readbacks become the controller's state. */
export class MuJoCoCarMission extends CarMission {
  constructor(mj, config = {}) {
    super(config);
    if (
      !Number.isFinite(this.config.dt) ||
      !(this.config.dt > 0) ||
      Math.abs(this.config.dt / STEP - Math.round(this.config.dt / STEP)) > 1e-8
    )
      throw Error("Car timestep must be a positive multiple of 0.002 s");
    this.config.physics = "mujoco";
    this.mj = mj;
    this.model = mj.MjModel.from_xml_string(
      carXml(this.config, this.layout.start),
    );
    this.data = new mj.MjData(this.model);
    const id = (kind, name) => {
      const handle = this.model[kind](name),
        result = handle.id;
      handle.delete();
      return result;
    };
    this.bodyId = id("body", "car");
    this.chassisGeom = id("geom", "chassis");
    this.groundGeom = id("geom", "ground");
    this.wheelGeoms = WHEELS.map((_, i) => id("geom", `tire_${i}`));
    this.axles = WHEELS.map((_, i) => {
      const j = id("jnt", `axle_${i}`);
      return { q: this.model.jnt_qposadr[j], v: this.model.jnt_dofadr[j] };
    });
    this.rampGeoms = new Set(
      this.barrel
        ? []
        : Array.from({ length: 160 }, (_, i) => id("geom", `ramp_${i}`)),
    );
    this.obstacleGeoms = new Map(
      this.barrel
        ? STAGE_VEHICLES.map((v, i) => [id("geom", `vehicle_${i}`), v.name])
        : [],
    );
    this.mass = this.x[0];
    this.lastMass = NaN;
    this.massUpdate();
    mj.mj_forward(this.model, this.data);
    this.syncState();
  }
  predictionConfig() {
    return { ...this.config, wheel_momentum: this.wheelMomentum };
  }
  createPlanningSeed() {
    return new MuJoCoCarMission(this.mj, {
      ...this.config,
      goal_planning: false,
    });
  }
  dispose() {
    this.data.delete();
    this.model.delete();
  }
  massUpdate() {
    if (Math.abs(this.mass - this.lastMass) < 0.1) return;
    const m = this.model,
      bid = this.bodyId;
    const chassisMass = this.mass - 4 * WHEEL_MASS;
    const z = WHEEL_MASS / chassisMass; // balances 4 wheels at z = -0.25 m
    m.body_mass[bid] = chassisMass;
    m.body_ipos[bid * 3 + 2] = z;
    const I = [800, 2600, 2900].map((value, axis) => {
      const wheel = WHEELS.reduce(
        (s, p) =>
          s +
          WHEEL_I[axis] +
          WHEEL_MASS * p.reduce((t, v, i) => t + (i === axis ? 0 : v * v), 0),
        0,
      );
      return (
        (value * this.mass) / 1678 -
        wheel -
        (axis < 2 ? chassisMass * z * z : 0)
      );
    });
    m.body_inertia.set(I, bid * 3);
    const p = this.data.qpos.slice(),
      v = this.data.qvel.slice(),
      time = this.data.time;
    this.mj.mj_setConst(m, this.data);
    this.data.qpos.set(p);
    this.data.qvel.set(v);
    this.data.time = time;
    this.mj.mj_forward(m, this.data);
    this.lastMass = this.mass;
  }
  syncState() {
    this.x = [
      this.mass,
      ...this.data.qpos.slice(0, 3),
      ...this.data.qvel.slice(0, 3),
      ...this.data.qpos.slice(3, 7),
      ...this.data.qvel.slice(3, 6),
    ];
    this.wheelMomentum = [
      0,
      this.axles.reduce((sum, a) => sum + WHEEL_I[1] * this.data.qvel[a.v], 0),
      0,
    ];
    this.t = this.data.time;
    this.maxHeight = Math.max(this.maxHeight, this.x[3]);
  }
  contactState() {
    const contacts = this.data.contact;
    let tires = 0,
      ramp = 0,
      ground = 0,
      impact = null;
    const touching = new Set();
    for (let i = 0; i < contacts.size(); i++) {
      const c = contacts.get(i);
      if (c.dist <= 0) {
        const a = c.geom1,
          b = c.geom2;
        const tire = this.wheelGeoms.includes(a)
          ? a
          : this.wheelGeoms.includes(b)
            ? b
            : null;
        const other = tire === a ? b : a;
        if (tire !== null) {
          touching.add(tire);
          if (this.rampGeoms.has(other)) ramp++;
          if (other === this.groundGeom) ground++;
        }
        if (a === this.chassisGeom || b === this.chassisGeom)
          impact = "Body impact";
        const vehicle = this.obstacleGeoms.get(a) ?? this.obstacleGeoms.get(b);
        if (vehicle) impact = "Display vehicle impact: " + vehicle;
      }
      c.delete();
    }
    contacts.delete();
    tires = touching.size;
    return { tires, ramp, ground, impact };
  }
  force(force, point, torque = [0, 0, 0]) {
    this.mj.mj_applyFT(
      this.model,
      this.data,
      force,
      torque,
      point,
      this.bodyId,
      this.data.qfrc_applied,
    );
  }
  applyForces(driving, braking = false) {
    const x = this.x,
      q = x.slice(7, 11),
      w = x.slice(11),
      p = x.slice(1, 4),
      v = x.slice(4, 7);
    this.data.qfrc_applied.fill(0);
    this.data.xfrc_applied.fill(0);
    for (let i = 0; i < NOZZLES.positions.length; i++) {
      this.force(
        rotate(q, scale(NOZZLES.directions[i], this.u[i])),
        add(p, rotate(q, NOZZLES.positions[i])),
      );
    }
    this.force(
      scale(v, -0.42 * norm(v)),
      p,
      rotate(
        q,
        w.map(
          (v, i) => ((-0.025 * [800, 2600, 2900][i] * this.mass) / 1678) * v,
        ),
      ),
    );
    if (this.fan) this.force(rotate(q, [0, 0, -this.fan]), p);
    if (driving || braking) {
      const local = invRotate(q, v),
        forward = rotate(q, [1, 0, 0]);
      const acceleration = clamp(
        2.8 * ((braking ? 0 : this.config.speed) - local[0]),
        -5,
        5,
      );
      const torque =
        (this.mass * (acceleration + 9.81 * forward[2]) * 0.37) / 4;
      for (const axle of this.axles) {
        const slip = local[0] / 0.37 - this.data.qvel[axle.v];
        this.data.qfrc_applied[axle.v] = clamp(torque + 60 * slip, -1800, 1800);
      }
    }
  }
  releaseFlight() {
    this.release = this.t;
    this.releaseAngle = this.barrel
      ? 0
      : Math.acos(clamp(rotate(this.x.slice(7, 11), [1, 0, 0])[0], -1, 1));
    this.phase = this.barrel ? "LIFTOFF" : "FLIP";
    this.events.push({
      t: this.t,
      name: this.barrel
        ? "Thruster liftoff / flight MPC"
        : "Ramp release / flight MPC",
    });
    this.fan = 0;
  }
  observeFlight() {
    const x = this.x,
      air = this.t - this.release,
      q = x.slice(7, 11);
    const u = clamp(air / 2, 0, 1),
      pitch = this.barrel
        ? 0
        : this.releaseAngle +
          (Math.PI - this.releaseAngle) *
            (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5);
    const rel = qmul([Math.cos(pitch / 2), 0, Math.sin(pitch / 2), 0], q);
    const ry = rotate(rel, [0, 1, 0]),
      rz = rotate(rel, [0, 0, 1]);
    const roll = Math.atan2(ry[2], rz[2]);
    if (this.land === null) {
      this.maneuver += Math.atan2(
        Math.sin(roll - this.previousRoll),
        Math.cos(roll - this.previousRoll),
      );
      this.previousRoll = roll;
    }
    this.roll += (this.x[11] * this.config.dt * 180) / Math.PI;
    if (this.barrel) {
      const points = [];
      for (const xx of [-2.1, 2.1])
        for (const yy of [-0.9, 0.9])
          for (const zz of [-0.25, 0.6])
            points.push(add(x.slice(1, 4), rotate(q, [xx, yy, zz])));
      for (const wheel of WHEELS)
        points.push(add(x.slice(1, 4), rotate(q, add(wheel, [0, 0, -0.37]))));
      this.minStageClearance = Math.min(
        this.minStageClearance,
        stageClearance(points).clearance,
      );
    }
  }
  touchdown(state) {
    const x = state;
    this.touch = {
      t: this.t,
      position: x.slice(1, 4),
      vertical_speed: Math.abs(x[6]),
      horizontal_speed: norm(x.slice(4, 6)),
      tilt_degrees:
        (Math.acos(clamp(rotate(x.slice(7, 11), [0, 0, 1])[2], -1, 1)) * 180) /
        Math.PI,
      angular_rate: norm(x.slice(11)),
      roll_degrees: this.roll,
    };
    this.land = this.t;
    this.phase = "SETTLED";
    this.u.fill(0);
    this.events.push({ t: this.t, name: "First wheel contact" });
    if (this.touch.vertical_speed > 2 || this.touch.tilt_degrees > 10)
      this.failure = "Hard or tilted touchdown";
  }
  step() {
    if (this.failure) return { done: true };
    this.syncState();
    const driving = this.release === null;
    if (
      !driving &&
      this.config.goal_planning &&
      this.config.use_mpc &&
      !this.flightPlan
    )
      throw Error("Prepare a full-flight plan before takeoff");
    this.u.fill(0);
    this.fan = 0;
    let solve = null;
    if (driving) {
      const pitch = Math.atan2(
        rotate(this.x.slice(7, 11), [1, 0, 0])[2],
        rotate(this.x.slice(7, 11), [1, 0, 0])[0],
      );
      const angle = Math.atan2(this.x[1], this.config.radius - this.x[3]);
      if (!this.barrel && angle > 0.03) {
        if (this.phase !== "RAMP")
          this.events.push({ t: this.t, name: "Fan seal / C-ramp entry" });
        this.phase = "RAMP";
        const speed = norm(this.x.slice(4, 7));
        this.fan = Math.min(
          Math.max(
            0,
            this.mass *
              ((-speed * speed) / Math.max(1, this.config.radius - 0.62) -
                9.81 * Math.cos(pitch)),
          ) +
            0.65 * this.mass * 9.81,
          this.config.fan_g * this.mass * 9.81,
        );
        this.maxFan = Math.max(this.maxFan, this.fan / this.mass / 9.81);
      }
      this.owner =
        this.phase === "RAMP"
          ? "FAN + WHEEL DRIVE / MUJOCO"
          : "WHEEL DRIVE / MUJOCO";
    } else if (this.land === null) {
      const air = this.t - this.release;
      const phase = this.barrel
        ? air < 1.8
          ? "LIFTOFF"
          : air < 3.8
            ? this.config.require_roll
              ? "BARREL_ROLL"
              : "CLEARANCE"
            : "DESCENT"
        : air < 2
          ? "FLIP"
          : "DESCENT";
      if (phase !== this.phase && this.barrel)
        this.events.push({
          t: this.t,
          name:
            phase === "BARREL_ROLL"
              ? "Full roll begins"
              : phase === "CLEARANCE"
                ? "Vehicle clearance"
                : "Descent begins",
        });
      this.phase = phase;
      if (this.config.use_mpc) {
        if (this.t + 1e-8 >= this.next) {
          solve = this.plan();
          this.next = this.t + 0.12;
        }
        this.u = this.mpc.plan?.[0].slice() ?? Array(6).fill(0);
      }
      this.owner = this.config.use_mpc
        ? "ROADSTER SCvx MPC / MUJOCO"
        : "BALLISTIC / MPC OFF / MUJOCO";
    } else this.owner = "WHEEL CONTACT / MUJOCO";
    if (this.failure) return { done: true, solve };
    const steps = Math.round(this.config.dt / STEP);
    for (let i = 0; i < steps; i++) {
      const previous = this.x.slice();
      // mj_setConst can reset force arrays, so update mass before applying loads.
      this.massUpdate();
      const remaining = Math.max(0, this.mass - 1428);
      const cap = this.config.nozzle_n * Math.min(1, remaining / 12);
      this.u = this.u.map((v) => clamp(v, 0, cap));
      const needed =
        (this.u.reduce((a, b) => a + b, 0) * STEP) / this.config.exhaust_mps;
      if (needed > remaining) this.u = scale(this.u, remaining / needed);
      this.applyForces(driving, this.land !== null);
      const thrust = this.u.reduce((a, b) => a + b, 0);
      this.mass = Math.max(
        1428,
        this.mass - (thrust / this.config.exhaust_mps) * STEP,
      );
      this.mj.mj_step(this.model, this.data);
      this.syncState();
      const contacts = this.contactState();
      this.contacts = contacts.tires;
      if (contacts.impact) {
        this.failure = contacts.impact;
        this.phase = "CRASH";
        break;
      }
      if (driving) {
        if (this.barrel && this.x[1] >= this.layout.takeoff) {
          this.releaseFlight();
          break;
        }
        if (
          !this.barrel &&
          this.phase === "RAMP" &&
          contacts.ramp === 0 &&
          this.x[3] > this.config.radius * 1.8 &&
          this.x[1] < 0
        ) {
          this.releaseFlight();
          break;
        }
      } else {
        if (!contacts.tires && this.x[3] > 0.72) this.hasLeftGround = true;
        if (this.land === null && this.hasLeftGround && contacts.ground)
          this.touchdown(previous);
      }
    }
    if (this.release !== null) this.observeFlight();
    return {
      done:
        !!this.failure ||
        this.t >= 24 ||
        (this.land !== null && this.t - this.land > (this.barrel ? 4 : 2)),
      solve,
    };
  }
  frame() {
    return {
      ...super.frame(),
      physics: "mujoco",
      wheel_angles: this.axles.map((a) => this.data.qpos[a.q]),
      wheel_rates: this.axles.map((a) => this.data.qvel[a.v]),
    };
  }
  summary() {
    return { ...super.summary(), physics: "mujoco" };
  }
}
