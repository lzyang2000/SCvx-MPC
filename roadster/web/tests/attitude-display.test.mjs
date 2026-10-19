import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { bodyRollDegrees } from "../src/attitude-display.js";

test("signed attitude distinguishes bank direction and wraps a full rotation", () => {
  for (const [travel, expected] of [[0,0],[90,90],[-90,-90],[180,180],[270,-90],[360,0],[450,90]]) {
    const a = travel * Math.PI / 360;
    assert.ok(Math.abs(bodyRollDegrees([Math.cos(a),Math.sin(a),0,0]) - expected) < 1e-8);
  }
  assert.equal(bodyRollDegrees([0,0,1,0]), 180); // Inverted C-ramp release.
  assert.equal(bodyRollDegrees([0,0,0,1]), 0); // Upright, facing the approach.
});
test("both recorded cars show upright attitude after touchdown, within signed bounds", async () => {
  for (const name of ["car", "barrel"]) {
    const run = JSON.parse(await readFile(new URL(`../public/${name}-nominal.json`, import.meta.url)));
    for (const frame of run.frames) assert.ok(Math.abs(bodyRollDegrees(frame.q)) <= 180);
    assert.ok(Math.abs(bodyRollDegrees(run.frames.at(-1).q)) < 3);
  }
});
