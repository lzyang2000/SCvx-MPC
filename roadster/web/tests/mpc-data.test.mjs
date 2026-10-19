import test from "node:test";
import assert from "node:assert/strict";
import { mpcAtTime } from "../src/mpc-data.js";

const solves = [
  { id: 1, t: 1, success: false },
  { id: 2, t: 1.4, success: true },
  { id: 3, t: 1.8, success: true },
];
const run = {
  config: { use_mpc: true },
  summary: { mpc_solves: solves },
  frames: [
    { t: 1.39, p: [1, 2, 3] },
    { t: 1.4, p: [2, 3, 4] },
    { t: 1.5, p: [3, 4, 5] },
    { t: 2, p: [4, 5, 6] },
  ],
};
test("scrubbing selects causal solve and never leaks a future prediction", () => {
  assert.equal(
    mpcAtTime(run, { controller: "WHEEL DRIVE" }, 0.9).state,
    "waiting",
  );
  assert.equal(
    mpcAtTime(run, { controller: "WHEEL DRIVE", mpc_id: 1 }, 1.2).state,
    "rejected",
  );
  const view = mpcAtTime(
    run,
    { controller: "ROADSTER SCvx MPC / HALF-ROLL", mpc_id: 2 },
    1.5,
  );
  assert.equal(view.solve.id, 2);
  assert.equal(view.state, "active");
  assert.deepEqual(
    view.actual.map((f) => f.t),
    [1.4, 1.5],
  );
  assert.equal(
    mpcAtTime(run, { controller: "WHEEL DRIVE", mpc_id: 1 }, 1.1).solve.id,
    1,
  );
});
test("last accepted prediction remains inspectable after wheel contact", () => {
  const view = mpcAtTime(
    run,
    {
      controller: "WHEEL CONTACT",
      mpc_id: 2,
      delivered_force: [1, 2, 3],
    },
    1.6,
  );
  assert.equal(view.state, "handoff");
  assert.equal(view.active, false);
  assert.deepEqual(view.delivered, [1, 2, 3]);
  assert.equal(
    mpcAtTime(
      { ...run, config: { use_mpc: false } },
      { controller: "WHEEL DRIVE" },
      0,
    ).state,
    "disabled",
  );
});
