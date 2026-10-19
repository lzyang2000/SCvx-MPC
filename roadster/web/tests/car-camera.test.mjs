import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import * as THREE from "three";
import { carSceneView } from "../src/car-camera.js";
import { createStageVehicles } from "../src/stage-models.js";
for (const file of ["car", "barrel"]) {
  const run = JSON.parse(
    await readFile(new URL(`../public/${file}-nominal.json`, import.meta.url)),
  );
  for (const aspect of [0.35, 1.25, 2.4])
    for (const view of ["overview", "side"])
      test(`${file} ${view} fits flight and stage at aspect ${aspect}`, () => {
        const framing = carSceneView(run, aspect, 40, view),
          camera = new THREE.PerspectiveCamera(
            40,
            aspect,
            0.1,
            framing.distance * 3,
          );
        camera.position.set(...framing.position);
        camera.lookAt(new THREE.Vector3(...framing.target));
        camera.updateMatrixWorld();
        const points = [];
        for (const f of run.frames) {
          const c = new THREE.Vector3(f.p[0], f.p[2], -f.p[1]);
          for (const dx of [-2.5, 2.5])
            for (const dy of [-1.2, 1.2])
              for (const dz of [-1.2, 1.2])
                points.push(c.clone().add(new THREE.Vector3(dx, dy, dz)));
        }
        if (file === "barrel") {
          const box = new THREE.Box3().setFromObject(createStageVehicles());
          for (const x of [box.min.x, box.max.x])
            for (const y of [box.min.y, box.max.y])
              for (const z of [box.min.z, box.max.z])
                points.push(new THREE.Vector3(x, y, z));
        }
        for (const p of points) {
          const projected = p.clone().project(camera);
          assert.ok(
            Math.abs(projected.x) < 0.98 &&
              Math.abs(projected.y) < 0.98 &&
              projected.z < 1,
            `${p.toArray()} -> ${projected.toArray()}`,
          );
        }
      });
}
