import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { createNozzleSteering } from "../src/rocket-gimbal.js";
const neutral = [0, 0, 500000], turned = [45000, 0, 500000];
const target = new THREE.Quaternion().setFromUnitVectors(
  new THREE.Vector3(0, 0, 1), new THREE.Vector3(...turned).normalize(),
);

test("nozzle direction eases without overshoot and matches across render rates", () => {
  const results = [];
  for (const fps of [30, 60, 120]) {
    const steering = createNozzleSteering();
    steering.update(0, neutral);
    let error = target.angleTo(new THREE.Quaternion());
    for (let i = 1; i <= fps; i++) {
      const pose = steering.update(i / fps, turned);
      const next = target.angleTo(pose);
      assert.ok(next <= error + 1e-10);
      if (i === 1) assert.ok(next > 0.01, "avoid an instantaneous direction jump");
      error = next;
      assert.ok(Math.abs(pose.length() - 1) < 1e-12);
    }
    results.push(steering.update(1, turned).clone());
  }
  assert.ok(results[0].angleTo(results[1]) < 1e-7);
  assert.ok(results[1].angleTo(results[2]) < 1e-7);
});
test("pause preserves nozzle pose; seeks and a new burn reset stale direction", () => {
  const steering = createNozzleSteering();
  steering.update(0, neutral);
  const pose = steering.update(0.1, turned).clone();
  assert.deepEqual(steering.update(0.1, turned).toArray(), pose.toArray());
  assert.ok(steering.update(0.05, turned).angleTo(target) < 1e-7);
  assert.ok(steering.update(10, neutral).angleTo(new THREE.Quaternion()) < 1e-7);
  steering.update(10.1, [0,0,0]);
  assert.ok(steering.update(10.2, turned).angleTo(target) < 1e-7);
  assert.deepEqual(turned, [45000, 0, 500000]);
});
