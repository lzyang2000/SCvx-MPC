import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { createTrackingCamera } from "../src/tracking-camera.js";

for (const mode of ["follow", "mpc"]) {
  test(`${mode} preserves viewing direction and distance while tracking`, () => {
    const camera = new THREE.PerspectiveCamera();
    const controls = { target: new THREE.Vector3() };
    const tracking = createTrackingCamera(camera, controls);
    tracking.snap(new THREE.Vector3(0, 0.62, 0), mode);
    const offset = camera.position.clone().sub(controls.target);
    const direction = camera.getWorldDirection(new THREE.Vector3());
    for (const [dt, point] of [
      [1 / 120, [1, 2, 0]],
      [1 / 30, [2, 4, 1]],
      [0.07, [-8, 19, 0]],
      [1 / 60, [-39, 0.62, 0]],
    ]) {
      tracking.update(new THREE.Vector3(...point), mode, dt);
      assert.ok(
        camera.position.clone().sub(controls.target).distanceTo(offset) < 1e-12,
      );
      assert.ok(
        camera.getWorldDirection(new THREE.Vector3()).distanceTo(direction) <
          1e-12,
      );
    }
  });
}

test("tracking settles identically at 30, 60 and 120 frames per second", () => {
  const positions = [];
  for (const fps of [30, 60, 120]) {
    const camera = new THREE.PerspectiveCamera();
    const controls = { target: new THREE.Vector3() };
    const tracking = createTrackingCamera(camera, controls);
    tracking.snap(new THREE.Vector3(), "follow");
    for (let frame = 0; frame < fps; frame++)
      tracking.update(new THREE.Vector3(10, 4, 0), "follow", 1 / fps);
    positions.push(camera.position.clone());
  }
  assert.ok(positions[0].distanceTo(positions[1]) < 1e-12);
  assert.ok(positions[1].distanceTo(positions[2]) < 1e-12);
});
