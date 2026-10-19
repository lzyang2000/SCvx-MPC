import init, {
  solve as clarabelSolve,
} from "./vendor/clarabel/clarabel_wasm.js";
export async function initSolver(bytes) {
  if (bytes) await init({ module_or_path: bytes });
  else await init();
}
function csc(n, rows) {
  const cols = Array.from({ length: n }, () => []);
  rows.forEach((row, r) => {
    for (const [c, v] of row) if (Math.abs(v) > 1e-15) cols[c].push([r, v]);
  });
  const ptr = [0],
    idx = [],
    val = [];
  for (const col of cols) {
    for (const [r, v] of col) {
      idx.push(r);
      val.push(v);
    }
    ptr.push(idx.length);
  }
  return [new Uint32Array(ptr), new Uint32Array(idx), new Float64Array(val)];
}
export class Conic {
  constructor(n) {
    this.n = n;
    this.p = new Float64Array(n);
    this.q = new Float64Array(n);
    this.zero = [];
    this.positive = [];
    this.soc = [];
  }
  square(i, weight, target) {
    this.p[i] += 2 * weight;
    this.q[i] -= 2 * weight * target;
  }
  eq(row, b) {
    this.zero.push([row, b]);
  }
  le(row, b) {
    this.positive.push([row, b]);
  }
  bound(i, lo, hi) {
    this.le([[i, 1]], hi);
    this.le([[i, -1]], -lo);
  }
  cone(rows, rhs) {
    this.soc.push(rows.map((r, i) => [r, rhs[i]]));
  }
  solve() {
    const all = [...this.zero, ...this.positive, ...this.soc.flat()];
    const P = csc(
      this.n,
      Array.from(this.p, (v, i) => (v ? [[i, v]] : [])),
    );
    const A = csc(
      this.n,
      all.map((v) => v[0]),
    );
    return clarabelSolve(
      ...P,
      this.q,
      ...A,
      new Float64Array(all.map((v) => v[1])),
      this.n,
      all.length,
      JSON.stringify({
        zero: this.zero.length,
        nonneg: this.positive.length,
        soc: this.soc.map((v) => v.length),
        exp: 0,
        power: [],
      }),
      JSON.stringify({
        verbose: false,
        max_iter: 180,
        tol_gap_abs: 1e-6,
        tol_gap_rel: 1e-6,
      }),
    );
  }
}
