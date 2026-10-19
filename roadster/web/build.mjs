import { cp, mkdir, rm, copyFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";
const root = fileURLToPath(new URL(".", import.meta.url));
await rm(root + "dist", { recursive: true, force: true });
await mkdir(root + "dist/vendor/three/addons", { recursive: true });
await cp(root + "public", root + "dist", { recursive: true });
await cp(root + "src", root + "dist/src", { recursive: true });
for (const file of [
  "index.html",
  "car.html",
  "rocket.html",
  "formulation.html",
])
  await copyFile(root + file, root + "dist/" + file);
await mkdir(root + "dist/vendor/mujoco", { recursive: true });
for (const file of ["mujoco.js", "mujoco.wasm"])
  await copyFile(
    root + "node_modules/@mujoco/mujoco/" + file,
    root + "dist/vendor/mujoco/" + file,
  );
await copyFile(
  root + "node_modules/@mujoco/mujoco/README.md",
  root + "dist/vendor/mujoco/README.md",
);
for (const file of ["three.module.js", "three.core.js"])
  await copyFile(
    root + "node_modules/three/build/" + file,
    root + "dist/vendor/three/" + file,
  );
for (const file of [
  "loaders/GLTFLoader.js",
  "controls/OrbitControls.js",
  "utils/BufferGeometryUtils.js",
  "environments/RoomEnvironment.js",
]) {
  await mkdir(root + "dist/vendor/three/addons/" + file.split("/")[0], {
    recursive: true,
  });
  await copyFile(
    root + "node_modules/three/examples/jsm/" + file,
    root + "dist/vendor/three/addons/" + file,
  );
}
await copyFile(
  root + "node_modules/three/LICENSE",
  root + "dist/vendor/three/LICENSE",
);
await copyFile(
  root + "public/assets/MUJOCO-LICENSE",
  root + "dist/vendor/mujoco/LICENSE",
);
await copyFile(
  root + "public/assets/MUJOCO-NOTICE.txt",
  root + "dist/vendor/mujoco/NOTICE.txt",
);

console.log("Built local viewer in " + root + "dist");
