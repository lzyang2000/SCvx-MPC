import { Conic } from "./conic.js";

/** Scaled, multiple-shooting SCvx. Only validated nonlinear plans are applied. */
export class Scvx {
  constructor({ nx = 14, nu, n = 8, scaleX, scaleU }) {
    Object.assign(this, { nx, nu, n, sx: scaleX, su: scaleU });
    this.plan = null;
    this.id = 0;
  }
  reset() {
    this.plan = null;
  }
  rollout(x, us, model, dt) {
    const xs = [x];
    for (let k = 0; k < us.length; k++)
      xs.push(model.step(xs.at(-1), us[k], dt, k));
    return xs;
  }
  solve(
    state,
    refs,
    model,
    {
      dt,
      weights,
      seed,
      iterations = 3,
      trust = 0.6,
      ground = -1e20,
      dry = 0,
      controlLinear = 0,
      controlCost = 0.03,
      maxDefect = 0.04,
      shift = 1,
      terminalWeights = null,
      extraConstraints = null,
      minimumIterations = 2,
      includeStates = false,
      merit = null,
    },
  ) {
    const began = performance.now(),
      { nx, nu, n, sx, su } = this;
    let us = this.plan
      ? Array.from({ length: n }, (_, k) => {
          const a = Math.min(n - 1, k + Math.floor(shift)),
            b = Math.min(n - 1, a + 1),
            f = shift - Math.floor(shift);
          return this.plan[a].map((v, j) => v + f * (this.plan[b][j] - v));
        })
      : Array.from({ length: n }, (_, k) => seed(k));
    us = us.map((u, k) => model.limit(u, k));
    let xs = this.rollout(state, us, model, dt);
    const uoff = nx * (n + 1),
      voff = uoff + nu * n,
      soff = voff + nx * n;
    let result,
      actual,
      virtual = Infinity,
      defect = Infinity,
      candidate,
      passes = 0;
    const costs = merit ? [merit(xs, us)] : [];
    for (let pass = 0; pass < iterations; pass++) {
      const qp = new Conic(soff + nx * n);
      for (let i = 0; i < nx; i++) qp.eq([[i, 1]], state[i] / sx[i]);
      for (let k = 0; k <= n; k++) {
        for (let i = 0; i < nx; i++) {
          const xi = k * nx + i,
            w =
              k === n && terminalWeights
                ? terminalWeights[i]
                : weights[i] * (k === n ? 2 : 1);
          qp.square(xi, w * w, refs[k][i] / sx[i]);
          if (k)
            qp.bound(xi, xs[k][i] / sx[i] - trust, xs[k][i] / sx[i] + trust);
        }
        qp.le([[k * nx, -1]], -dry / sx[0]);
        qp.le([[k * nx + 3, -1]], -ground / sx[3]);
      }
      for (let k = 0; k < n; k++) {
        const base = model.step(xs[k], us[k], dt, k),
          A = [],
          B = [];
        for (let j = 0; j < nx; j++) {
          const dx = j === 0 ? 1 : j < 7 ? 0.002 : 0.00002,
            plus = xs[k].slice();
          plus[j] += dx;
          A.push(
            model.step(plus, us[k], dt, k).map((v, i) => (v - base[i]) / dx),
          );
        }
        for (let j = 0; j < nu; j++) {
          const du = su[j] * 0.00002,
            plus = us[k].slice();
          plus[j] += du;
          B.push(
            model.step(xs[k], plus, dt, k).map((v, i) => (v - base[i]) / du),
          );
        }
        for (let i = 0; i < nx; i++) {
          const row = [
            [(k + 1) * nx + i, 1],
            [voff + k * nx + i, -1],
          ];
          let c = base[i];
          for (let j = 0; j < nx; j++) {
            row.push([k * nx + j, (-A[j][i] * sx[j]) / sx[i]]);
            c -= A[j][i] * xs[k][j];
          }
          for (let j = 0; j < nu; j++) {
            row.push([uoff + k * nu + j, (-B[j][i] * su[j]) / sx[i]]);
            c -= B[j][i] * us[k][j];
          }
          qp.eq(row, c / sx[i]);
          const vi = voff + k * nx + i,
            si = soff + k * nx + i;
          qp.le(
            [
              [vi, 1],
              [si, -1],
            ],
            0,
          );
          qp.le(
            [
              [vi, -1],
              [si, -1],
            ],
            0,
          );
          qp.q[si] = 30000;
        }
        const ui = uoff + k * nu;
        for (let j = 0; j < nu; j++) {
          qp.square(ui + j, controlCost, us[k][j] / su[j]);
          qp.q[ui + j] += controlLinear;
          qp.bound(ui + j, us[k][j] / su[j] - trust, us[k][j] / su[j] + trust);
        }
        model.constraints(qp, ui, k, su);
      }
      model.trajectoryConstraints?.(qp, { uoff, nu, n, su, dt, nx, sx });
      extraConstraints?.(qp, { nx, nu, n, sx, su, xs, us, dt, uoff });
      result = qp.solve();
      passes++;
      if (result.status !== "optimal" || !result.x) break;
      const z = result.x;
      candidate = Array.from({ length: n }, (_, k) =>
        Array.from({ length: nu }, (_, j) => z[uoff + k * nu + j] * su[j]),
      );
      let alpha = 1;
      actual = this.rollout(state, candidate, model, dt);
      if (merit) {
        const full = candidate;
        const previousCost = merit(xs, us);
        let accepted = false;
        for (let backtrack = 0; backtrack < 12; backtrack++) {
          candidate = us.map((u, k) =>
            u.map((v, j) => v + alpha * (full[k][j] - v)),
          );
          actual = this.rollout(state, candidate, model, dt);
          const cost = merit(actual, candidate);
          if (Number.isFinite(cost) && cost <= previousCost + 1e-9) {
            accepted = true;
            costs.push(cost);
            break;
          }
          alpha *= 0.5;
        }
        if (!accepted) {
          candidate = us;
          actual = xs;
          alpha = 0;
          trust *= 0.5;
        }
      }
      virtual = 0;
      defect = 0;
      for (let k = 0; k < n; k++)
        for (let i = 0; i < nx; i++) {
          virtual = Math.max(virtual, Math.abs(alpha * z[voff + k * nx + i]));
          defect = Math.max(
            defect,
            Math.abs(
              actual[k + 1][i] / sx[i] -
                (((1 - alpha) * xs[k + 1][i]) / sx[i] +
                  alpha * z[(k + 1) * nx + i]),
            ),
          );
        }
      us = candidate;
      xs = actual;
      if (defect < 0.002 && virtual < 0.0001 && pass >= minimumIterations - 1)
        break;
    }
    const success =
      result?.status === "optimal" &&
      defect < maxDefect &&
      virtual < 0.01 &&
      actual.every((x) => x.every(Number.isFinite)) &&
      candidate.every((u, k) => model.feasible(u, k)) &&
      actual.every((x) => x[0] >= dry - 0.01) &&
      (model.rolloutFeasible?.(actual, candidate, dt) ?? true);
    if (success) this.plan = candidate.map((u, k) => model.limit(u, k));
    const snapshot = {
      id: ++this.id,
      success: !!success,
      status: result?.status ?? "error",
      seconds: (performance.now() - began) / 1000,
      iterations: passes,
      defect: Number.isFinite(defect) ? defect : null,
      virtual_control: Number.isFinite(virtual) ? virtual : null,
      prediction: (actual ?? xs).map((x) => x.slice(1, 4)),
      attitude_prediction: (actual ?? xs).map((x) => x.slice(7, 11)),
      prediction_dt: dt,
      horizon_seconds: n * dt,
      origin: state.slice(1, 4),
      target: refs.at(-1).slice(1, 4),
      control_plan: candidate ?? us,
      ...(merit ? { costs } : {}),
    };
    return {
      command: success ? this.plan[0].slice() : null,
      snapshot,
      ...(includeStates ? { states: actual ?? xs } : {}),
    };
  }
}
