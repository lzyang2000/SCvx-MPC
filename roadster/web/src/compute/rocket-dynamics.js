import {
  add,
  scale,
  norm,
  cross,
  unit,
  qmul,
  rotate,
  clamp,
  rk4,
} from "./math.js";
export const ROCKET = {
  dry: 25600, // Estimated booster dry mass; not an official SpaceX specification.
  fuel: 407500, // Tank capacity; mass properties use the full tank geometry.
  initialFuel: 407500, // Estimated full first-stage tank at liftoff.
  engineMax: 845000, // SpaceX 2025: 190,000 lbf per M1D at sea level (rounded).
  minThrottle: 108300 / 190000, // Published sea-level throttle range.
  separationAltitude: 75000, // Target actual separation altitude, metres above launch.
  separationDelay: 3, // MECO coast; Falcon user guide LEO example.
  upperIgnitionDelay: 8, // Coast after separation; same LEO timeline example.
  downrangeSign: -1, // Mirror ascent and return so deployed legs approach clear of the -X tower.
  ascentPitch: 20, // Simplified pitch profile, degrees from vertical.
  hoverHeight: 2, // Lowest foot clearance above the pad, metres.
  hoverSeconds: 1, // Brief measured dwell before resuming descent.
  descentResponse: 0.6, // Smoothly resume descent without extending the hover.
  landingSpeed: 0.2, // Terminal descent speed, m/s.
  descentSpeed: 0.7, // Lower from the hover point, then brake near contact.
  descentDeceleration: 0.3, // Braking corridor above the terminal speed, m/s².
  hoverResponse: 0.7, // Critically damped approach rate, 1/s.
  upperDry: 4000,
  upperFuel: 107500,
  upperCom: 34,
  engineZ: -20.1,
  padHeight: 0.16, // Top of the MuJoCo landing-pad cylinder.
  padX: 0, // Return to the launch position.
  landedZ: 20.76,
  isp: 282 * 9.80665,
  airDensity: 1.225, // Exponential atmosphere; no winds or aerodynamic moments.
  atmosphereHeight: 8500,
  axialCd: 0.8,
  sideCd: 0.8,
};
const tanks = (fill) => [
  [0.72 * 407500 * fill, -4.9 + 8 * fill, 16 * fill],
  [0.28 * 407500 * fill, -14 + 6 * fill, 12 * fill],
];
const tankI = (mass, length) => [
  (mass * (3 * 1.65 ** 2 + length ** 2)) / 12,
  (mass * (3 * 1.65 ** 2 + length ** 2)) / 12,
  0.5 * mass * 1.65 ** 2,
];
const dryCom = -tanks(1).reduce((s, [m, z]) => s + m * z, 0) / ROCKET.dry;
const dryI = [60000000, 60000000, 1000000];
for (const [m, z, l] of tanks(1)) {
  const I = tankI(m, l);
  for (let j = 0; j < 3; j++) dryI[j] -= I[j] + (j < 2 ? m * z * z : 0);
}
for (let j = 0; j < 2; j++) dryI[j] -= ROCKET.dry * dryCom ** 2;
export function properties(totalMass, attached = 0) {
  const m = clamp(totalMass - attached, ROCKET.dry, ROCKET.dry + ROCKET.fuel),
    ts = tanks((m - ROCKET.dry) / ROCKET.fuel);
  const z = (ROCKET.dry * dryCom + ts.reduce((s, [m, z]) => s + m * z, 0)) / m;
  const I = dryI.map(
    (v, i) => v + (i < 2 ? ROCKET.dry * (dryCom - z) ** 2 : 0),
  );
  for (const [tm, tz, l] of ts) {
    const ti = tankI(tm, l);
    for (let j = 0; j < 3; j++)
      I[j] += ti[j] + (j < 2 ? tm * (tz - z) ** 2 : 0);
  }
  if (!attached) return { mass: m, z, I };
  const center = (m * z + attached * 34) / (m + attached);
  return {
    mass: m + attached,
    z: center,
    I: I.map(
      (v, j) =>
        v +
        [7767663, 7767663, 192010][j] +
        (j < 2 ? m * (z - center) ** 2 + attached * (34 - center) ** 2 : 0),
    ),
  };
}
// Translational drag at the COM; projected area follows the rocket axis.
// Coefficients are simple model assumptions, not a Falcon aerodynamic database.
export function aerodynamicForce(x, attached = false) {
  const v = x.slice(4, 7),
    speed = norm(v);
  if (speed < 1e-8) return [0, 0, 0];
  const axis = rotate(unit(x.slice(7, 11)), [0, 0, 1]);
  const along = clamp(Math.abs(dotProduct(axis, v)) / speed, 0, 1);
  const area =
    ROCKET.axialCd * Math.PI * 1.83 ** 2 * along ** 2 +
    ROCKET.sideCd * 3.66 * (attached ? 70 : 42) * Math.max(0, 1 - along ** 2);
  const density =
    ROCKET.airDensity * Math.exp(-Math.max(0, x[3]) / ROCKET.atmosphereHeight);
  return scale(v, -0.5 * density * area * speed);
}
const dotProduct = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

export function rocketModel({
  attached = 0,
  min = 0,
  max = 0,
  coast = false,
  isp = ROCKET.isp,
}) {
  const gimbal = Math.tan((6 * Math.PI) / 180),
    torque = coast ? [2e6, 2e6, 17500] : [0, 0, 17500];
  const dynamics = (x, u) => {
    const m = Math.max(x[0], ROCKET.dry + attached),
      p = properties(m, attached),
      q = unit(x.slice(7, 11)),
      w = x.slice(11, 14),
      f = u.slice(0, 3);
    const torqueBody = add(
      cross([0, 0, ROCKET.engineZ - p.z], f),
      u.slice(3, 6),
    );
    return [
      -norm(f) / isp,
      ...x.slice(4, 7),
      ...add(
        scale(add(rotate(q, f), aerodynamicForce(x, attached > 0)), 1 / m),
        [0, 0, -9.81],
      ),
      ...scale(qmul(q, [0, ...w]), 0.5),
      ...torqueBody.map(
        (v, i) =>
          (v -
            cross(
              w,
              w.map((v, j) => v * p.I[j]),
            )[i]) /
          p.I[i],
      ),
    ];
  };
  return {
    step: (x, u, dt) => rk4(x, u, dt, dynamics),
    limit(u) {
      let f = u.slice(0, 3);
      f[2] = clamp(f[2], min, max);
      const xy = norm(f.slice(0, 2)),
        cap = gimbal * f[2];
      if (xy > cap) {
        f[0] *= cap / xy;
        f[1] *= cap / xy;
      }
      if (norm(f) > max) f = scale(f, max / norm(f));
      return [
        ...f,
        ...u.slice(3, 6).map((v, i) => clamp(v, -torque[i], torque[i])),
      ];
    },
    feasible: (u) =>
      u.every(Number.isFinite) &&
      u[2] >= min - 2 &&
      norm(u.slice(0, 3)) <= max + 2 &&
      norm(u.slice(0, 2)) <= gimbal * u[2] + 2 &&
      u.slice(3, 6).every((v, i) => Math.abs(v) <= torque[i] + 2),
    constraints(qp, i, k, s) {
      qp.bound(i + 2, min / s[2], max / s[2]);
      qp.cone(
        [
          [[i + 2, (-gimbal * s[2]) / s[0]]],
          [[i, -1]],
          [[i + 1, -s[1] / s[0]]],
        ],
        [0, 0, 0],
      );
      qp.cone(
        [[], [[i, -s[0] / s[2]]], [[i + 1, -s[1] / s[2]]], [[i + 2, -1]]],
        [max / s[2], 0, 0, 0],
      );
      for (let j = 0; j < 3; j++)
        qp.bound(i + 3 + j, -torque[j] / s[3 + j], torque[j] / s[3 + j]);
    },
  };
}
