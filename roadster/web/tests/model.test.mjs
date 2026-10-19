import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

// Node's data-URI fetch has no browser ProgressEvent.
globalThis.ProgressEvent ??= class ProgressEvent {
  constructor(type, values) {
    this.type = type;
    Object.assign(this, values);
  }
};

test("all four loaded wheels spin on fixed independent hubs", async () => {
  const source = await readFile(
    new URL("../public/assets/roadster.gltf", import.meta.url),
    "utf8",
  );
  const { scene } = await new GLTFLoader().parseAsync(source, "");
  const axles = [];
  scene.traverse((node) => {
    if (node.name.startsWith("Wheel_axle")) axles.push(node);
  });
  assert.equal(axles.length, 4);
  assert.equal(new Set(axles.flatMap((axle) => axle.children)).size, 20);
  for (const axle of axles) {
    assert.equal(
      axle.parent,
      scene,
      "an axle must not be inside another wheel",
    );
    assert.equal(axle.children.length, 5);
    const center = axle.position.clone();
    assert.ok(Math.abs(Math.abs(center.x) - 1.4) < 1e-6);
    assert.ok(Math.abs(Math.abs(center.y) - 0.92) < 1e-6);
    assert.equal(center.z, -0.25);
    for (const angle of [0, Math.PI / 2, Math.PI, 1.8 * Math.PI, 24]) {
      axle.rotation.y = angle;
      scene.updateMatrixWorld(true);
      assert.ok(
        axle.getWorldPosition(new THREE.Vector3()).distanceTo(center) < 1e-6,
      );
      const points = [];
      axle.traverse((node) => {
        if (!node.isMesh) return;
        const vertices = node.geometry.attributes.position;
        for (let i = 0; i < vertices.count; i++) {
          const p = node
            .localToWorld(new THREE.Vector3().fromBufferAttribute(vertices, i))
            .sub(center);
          assert.ok(
            Math.hypot(p.x, p.z) <= 0.37001,
            `${node.name} escaped its hub at spin ${angle}`,
          );
          assert.ok(Math.abs(p.y) <= 0.12501);
          points.push(p);
        }
      });
      const bounds = new THREE.Box3().setFromPoints(points);
      assert.ok(bounds.getCenter(new THREE.Vector3()).length() < 0.001);
    }
  }
});

test("asset contains exactly six nozzles matching the simulated mounts", async () => {
  const asset = JSON.parse(
    await readFile(
      new URL("../public/assets/roadster.gltf", import.meta.url),
      "utf8",
    ),
  );
  const jets = asset.nodes.filter((n) => /^Nozzle_\d+$/.test(n.name));
  assert.equal(jets.length, 6);
  assert.deepEqual(
    jets.map((n) => n.name).sort(),
    Array.from({ length: 6 }, (_, i) => `Nozzle_${String(i).padStart(2, "0")}`),
  );
  const run = JSON.parse(
    await readFile(
      new URL("./fixtures/car-python.json", import.meta.url),
      "utf8",
    ),
  );
  const { scene } = await new GLTFLoader().parseAsync(
    JSON.stringify(asset),
    "",
  );
  scene.updateMatrixWorld(true);
  const wheelBounds = [];
  scene.traverse((node) => {
    if (node.name.startsWith("Wheel_axle"))
      wheelBounds.push(new THREE.Box3().setFromObject(node));
  });
  for (let i = 0; i < 6; i++) {
    const node = scene.getObjectByName(`Nozzle_${String(i).padStart(2, "0")}`);
    const bounds = new THREE.Box3().setFromObject(node);
    const mount = new THREE.Vector3(...run.nozzles.positions[i]);
    const direction = new THREE.Vector3(...run.nozzles.directions[i]);
    const expectedCenter = mount.clone().addScaledVector(direction, -0.055);
    assert.ok(
      bounds.getCenter(new THREE.Vector3()).distanceTo(expectedCenter) < 1e-6,
      `nozzle ${i} must match its physical mount and exhaust direction`,
    );
    if (i >= 4) {
      for (const wheel of wheelBounds) {
        // Fore/aft separation remains valid for the full circular tire sweep.
        const gap = Math.max(
          bounds.min.x - wheel.max.x,
          wheel.min.x - bounds.max.x,
        );
        assert.ok(
          gap > 0.1,
          `side nozzle ${i} must clear the wheels by at least 10 cm`,
        );
      }
    }
  }
});

test("rendered tires stay above ground and inside the curved ramp deck", async () => {
  const source = await readFile(
    new URL("../public/assets/roadster.gltf", import.meta.url),
    "utf8",
  );
  const run = JSON.parse(
    await readFile(
      new URL("./fixtures/car-python.json", import.meta.url),
      "utf8",
    ),
  );
  const { scene } = await new GLTFLoader().parseAsync(source, "");
  const tires = [];
  scene.traverse((node) => {
    if (node.name.startsWith("Tire_")) tires.push(node);
  });
  assert.equal(tires.length, 4);
  const radius = run.config.radius;
  const position = new THREE.Vector3();
  for (const frame of run.frames.filter((f) =>
    ["APPROACH", "RAMP"].includes(f.phase),
  )) {
    scene.position.set(...frame.p);
    scene.quaternion.set(frame.q[1], frame.q[2], frame.q[3], frame.q[0]);
    scene.updateMatrixWorld(true);
    for (const tire of tires) {
      const vertices = tire.geometry.attributes.position;
      for (let i = 0; i < vertices.count; i++) {
        position
          .fromBufferAttribute(vertices, i)
          .applyMatrix4(tire.matrixWorld);
        assert.ok(position.z >= -0.002, `tire below ground at t=${frame.t}`);
        if (position.x >= 0) {
          assert.ok(
            Math.hypot(position.x, position.z - radius) <= radius + 0.002,
            `tire intersects ramp at t=${frame.t}`,
          );
        }
      }
    }
  }
});
