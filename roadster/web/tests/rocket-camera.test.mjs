import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { createRocketCamera } from "../src/rocket-camera.js";
import { createRocket } from "../src/rocket-model.js";

// Textures need a canvas handle; these checks only use mesh geometry and matrices.
globalThis.document = {
  createElement: () => ({ getContext: () => ({ fillText() {} }) }),
};
const rocket = createRocket();
function checkModelFits(camera, groups) {
  camera.updateMatrixWorld(true);
  for (const group of groups) {
    group.updateMatrixWorld(true);
    group.traverseVisible((mesh) => {
      if (!mesh.isMesh) return;
      const positions = mesh.geometry.getAttribute("position");
      for (let i = 0; i < positions.count; i++) {
        const p = new THREE.Vector3()
          .fromBufferAttribute(positions, i)
          .applyMatrix4(mesh.matrixWorld)
          .project(camera);
        assert.ok(
          Math.abs(p.x) <= 0.72 + 1e-8 && Math.abs(p.y) <= 0.72 + 1e-8,
          `Mesh cropped at ${p.toArray()}`,
        );
        assert.ok(p.z >= -1 && p.z <= 1);
      }
    });
  }
}

for (const aspect of [0.35, 1.25, 2.4]) {
  test(`entire rocket, feet and plumes fit at aspect ${aspect}`, () => {
    const camera = new THREE.PerspectiveCamera(42, aspect, 0.3, 2e6);
    const controls = { target: new THREE.Vector3() };
    const tracking = createRocketCamera(camera, controls);
    for (const attached of [true, false]) {
      rocket.update({
        attached,
        legs: attached ? 0 : 1,
        engines: 9,
        control: [0, 0, 7600000],
        t: 1,
        upper: { thrust: attached ? 0 : 981000 },
      });
      for (const angle of [0, Math.PI / 2, Math.PI]) {
        for (const body of [rocket.booster, rocket.upper]) {
          body.position.set(1000, 130000, -500);
          body.quaternion.setFromEuler(new THREE.Euler(angle, 0.7, angle / 2));
        }
        tracking.update(rocket.booster, attached ? "stack" : "booster", {
          snap: true,
        });
        checkModelFits(
          camera,
          attached ? [rocket.booster, rocket.upper] : [rocket.booster],
        );
        if (!attached) {
          tracking.update(rocket.upper, "upper", { snap: true });
          checkModelFits(camera, [rocket.upper]);
        }
      }
    }
  });
}

test("staging tightens smoothly while high-speed tracking has no position lag", () => {
  const camera = new THREE.PerspectiveCamera(42, 1.3, 0.3, 2e6);
  const controls = { target: new THREE.Vector3() };
  const tracking = createRocketCamera(camera, controls);
  const body = new THREE.Object3D();
  tracking.update(body, "stack", { snap: true });
  const before = camera.position.distanceTo(controls.target);
  rocket.update({
    attached: false,
    legs: 1,
    engines: 9,
    control: [0, 0, 7600000],
    t: 1,
  });
  for (let i = 0; i < 120; i++) {
    body.position.y += (1200 * 10) / 60;
    rocket.booster.position.copy(body.position);
    rocket.booster.quaternion.copy(body.quaternion);
    const previous = camera.position.clone();
    tracking.update(body, "booster", { dt: 1 / 60 });
    assert.ok(camera.position.y - previous.y > 199);
    checkModelFits(camera, [rocket.booster]);
  }
  assert.ok(camera.position.distanceTo(controls.target) < before);
  camera.aspect = 0.3;
  camera.updateProjectionMatrix();
  tracking.update(body, "booster");
  checkModelFits(camera, [rocket.booster]);
});
