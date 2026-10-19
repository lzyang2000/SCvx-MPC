export const add = (a, b) => a.map((v, i) => v + b[i]);
export const sub = (a, b) => a.map((v, i) => v - b[i]);
export const scale = (a, s) => a.map((v) => v * s);
export const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
export const norm = (a) => Math.hypot(...a);
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export const unit = (a) => scale(a, 1 / Math.max(norm(a), 1e-12));
export const qmul = (a, b) => [
  a[0] * b[0] - dot(a.slice(1), b.slice(1)),
  ...add(
    add(scale(b.slice(1), a[0]), scale(a.slice(1), b[0])),
    cross(a.slice(1), b.slice(1)),
  ),
];
export const rotate = (q, v) => {
  const t = scale(cross(q.slice(1), v), 2);
  return add(v, add(scale(t, q[0]), cross(q.slice(1), t)));
};
export const invRotate = (q, v) => rotate([q[0], -q[1], -q[2], -q[3]], v);
export function qblend(a, b, t) {
  if (dot(a, b) < 0) b = scale(b, -1);
  return unit(add(scale(a, 1 - t), scale(b, t)));
}
export function rk4(x, u, dt, dynamics) {
  const a = dynamics(x, u),
    b = dynamics(add(x, scale(a, dt / 2)), u),
    c = dynamics(add(x, scale(b, dt / 2)), u),
    d = dynamics(add(x, scale(c, dt)), u);
  const y = x.map((v, i) => v + (dt * (a[i] + 2 * b[i] + 2 * c[i] + d[i])) / 6);
  const q = unit(y.slice(7, 11));
  if (dot(q, x.slice(7, 11)) < 0) q.forEach((v, i) => (y[i + 7] = -v));
  else q.forEach((v, i) => (y[i + 7] = v));
  return y;
}
