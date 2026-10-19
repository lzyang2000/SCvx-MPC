import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import {
  createRocketInspector,
  rocketMpcAtTime,
  rocketCenterOfMass,
} from "../src/rocket-inspector.js";

const recording = JSON.parse(
  await readFile(new URL("../public/rocket-nominal.json", import.meta.url)),
);

test("rocket inspector follows causal plans and compares center of mass positions", () => {
  const frame = recording.frames.find((f) => f.t > 60);
  const view = rocketMpcAtTime(recording, frame, frame.t);
  assert.equal(view.solve.id, frame.mpc_id);
  assert.equal(view.state, "active");
  assert.ok(view.solve.t <= frame.t);
  assert.deepEqual(view.actual.at(-1), rocketCenterOfMass(frame));
  const index = recording.frames.findIndex((f) => f.t >= view.solve.t);
  const before = recording.frames[index - 1],
    after = recording.frames[index];
  const u = (view.solve.t - before.t) / (after.t - before.t);
  const start = rocketCenterOfMass(before),
    end = rocketCenterOfMass(after);
  const measured = start.map((v, i) => v + u * (end[i] - v));
  // Compare at solve time; the next saved frame can be over 100 m away in ascent.
  assert.ok(Math.hypot(...view.actual[0].map((v, i) => v - measured[i])) < 0.2);
  const earlier = rocketMpcAtTime(recording, { ...frame, mpc_id: 1 }, frame.t);
  assert.equal(earlier.solve.id, 1);
  assert.equal(rocketMpcAtTime(recording, frame, -1).state, "waiting");
  const contact = recording.frames.at(-1);
  assert.equal(rocketMpcAtTime(recording, contact, contact.t).state, "handoff");
});

test("COM comparison rotates the body offset and accounts for separation", () => {
  const frame = {
    p: [1, 2, 30],
    q: [Math.SQRT1_2, 0, Math.SQRT1_2, 0],
    fuel: 200000,
    mass: 337100,
    attached: true,
  };
  const p = rocketCenterOfMass(frame);
  assert.ok(Math.abs(p[2] - 30) < 1e-12);
  assert.equal(p[1], 2);
  assert.ok(p[0] > 1);
  const detached = rocketCenterOfMass({
    ...frame,
    mass: frame.mass - 111500,
    attached: false,
  });
  assert.notEqual(detached[0], p[0]);
});

function inspectorDom(html) {
  const elements = new Map(
    [...html.matchAll(/id="([^"]+)"/g)].map(([, id]) => [
      id,
      {
        textContent: "",
        attributes: {},
        classList: { toggle() {} },
        setAttribute(name, value) {
          this.attributes[name] = String(value);
        },
      },
    ]),
  );
  const section = { dataset: {} };
  return {
    section,
    getElementById(id) {
      assert.ok(elements.has(id), `Missing inspector element ${id}`);
      return elements.get(id);
    },
    querySelector() {
      return section;
    },
  };
}

test("rocket panel renders live solves, landing horizons, replay, rejection and empty states", async () => {
  const root = inspectorDom(
    await readFile(new URL("../rocket.html", import.meta.url), "utf8"),
  );
  const update = createRocketInspector(root),
    $ = root.getElementById;
  for (const phase of [
    "ASCENT",
    "MECO",
    "SEPARATION",
    "BOOSTBACK",
    "COAST",
    "ENTRY",
    "DESCENT_COAST",
    "LANDING",
    "CONTACT",
  ]) {
    const frame = recording.frames.find((f) => f.phase === phase);
    assert.ok(frame, phase);
    const solve = update(recording, frame, frame.t, true);
    assert.match($("mpc-solve").textContent, new RegExp(`#${frame.mpc_id} /`));
    assert.equal(
      $("mpc-horizon").textContent,
      `${solve.prediction.length - 1} × ${solve.prediction_dt.toFixed(2)} s`,
    );
    assert.equal($("transport-badge").textContent, "LIVE / WASM");
    assert.equal($("next-solve").disabled, true);
    assert.equal($("mpc-plot-now").attributes.visibility, "visible");
    for (const id of ["mpc-plot-actual", "mpc-plot-predicted"]) {
      const path = $(id).attributes.d;
      assert.match(path, /^M /);
      assert.ok(!/NaN|Infinity/.test(path));
      const coords = path.match(/-?\d+(?:\.\d+)?/g).map(Number);
      coords.forEach((v, i) =>
        assert.ok(v >= (i % 2 ? 18 : 48) && v <= (i % 2 ? 108 : 302)),
      );
    }
    update(recording, frame, frame.t, false);
    assert.equal($("transport-badge").textContent, "REPLAY / WASM");
    if (phase === "CONTACT") {
      assert.equal($("next-solve").disabled, true);
      assert.match($("mpc-state").textContent, /engines off/);
      assert.match($("mpc-force").textContent, /\/ 0.00 MN/);
    } else {
      assert.equal($("next-solve").disabled, false);
    }
  }
  const landing = recording.frames.find((f) => f.phase === "LANDING");
  const hold = { ...landing, landing_mode: "hover", landing_hold: true };
  update(recording, hold, hold.t, false);
  assert.match($("mpc-state").textContent, /hovering above/);
  assert.match($("mpc-explanation").textContent, /2 m/);
  const descent = {
    ...landing,
    landing_mode: "slow-descent",
    landing_hold: false,
  };
  update(recording, descent, descent.t, false);
  assert.match($("mpc-state").textContent, /lowering/);
  assert.match($("mpc-explanation").textContent, /0.2 m\/s/);
  const direct = {
    ...descent,
    landing_mode: "direct-landing",
    landing_hold: false,
  };
  update(recording, direct, direct.t, false);
  assert.match($("mpc-state").textContent, /directly to touchdown/);
  assert.match($("mpc-explanation").textContent, /without a hover/);
  const frame = recording.frames[0];
  const failed = {
    ...recording,
    summary: {
      mpc_solves: [{ ...recording.summary.mpc_solves[0], success: false }],
    },
  };
  update(failed, frame, frame.t, false);
  assert.equal(root.section.dataset.state, "rejected");
  assert.equal($("mpc-plot-predicted").attributes.stroke, "#ff7b86");
  update(recording, frame, -1, false);
  assert.equal(root.section.dataset.state, "waiting");
  assert.equal($("mpc-plot-predicted").attributes.d, "");
  assert.equal($("mpc-plot-now").attributes.visibility, "hidden");
});
