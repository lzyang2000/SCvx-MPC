import * as THREE from "three";
import { ROCKET } from "./compute/rocket-dynamics.js";
import { createEnginePlume } from "./rocket-plume.js";
import { createNozzleSteering } from "./rocket-gimbal.js";
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const mats = {
  white: new THREE.MeshStandardMaterial({
    color: 0xf3f1e8,
    roughness: 0.42,
    metalness: 0.25,
  }),
  black: new THREE.MeshStandardMaterial({
    color: 0x171c20,
    roughness: 0.43,
    metalness: 0.55,
  }),
  steel: new THREE.MeshStandardMaterial({
    color: 0x8c969d,
    roughness: 0.28,
    metalness: 0.85,
  }),
  bell: new THREE.MeshStandardMaterial({
    color: 0x43484b,
    roughness: 0.34,
    metalness: 0.85,
  }),
};
function mesh(group, geo, mat, pos = [0, 0, 0]) {
  const m = new THREE.Mesh(geo, mat);
  m.position.set(...pos);
  group.add(m);
  return m;
}
function cylinder(group, r, h, z, mat) {
  const m = mesh(group, new THREE.CylinderGeometry(r, r, h, 48), mat, [
    0,
    0,
    z,
  ]);
  m.rotation.x = Math.PI / 2;
  return m;
}
function lathe(group, points, mat) {
  const m = mesh(
    group,
    new THREE.LatheGeometry(
      points.map(([r, z]) => new THREE.Vector2(r, z)),
      64,
    ),
    mat,
  );
  m.rotation.x = Math.PI / 2;
  return m;
}
function rod(group, a, b, r, mat) {
  const m = mesh(group, new THREE.CylinderGeometry(r, r, 1, 8), mat);
  moveRod(m, a, b);
  return m;
}
function moveRod(m, a, b) {
  const d = b.clone().sub(a);
  m.position.copy(a).add(b).multiplyScalar(0.5);
  m.scale.y = d.length();
  m.quaternion.setFromUnitVectors(V(0, 1, 0), d.normalize());
}
function label(group, text, z) {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 128;
  const c = canvas.getContext("2d");
  c.fillStyle = "#20272a";
  c.font = "600 54px sans-serif";
  c.textAlign = "center";
  c.fillText(text, 256, 82);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const m = mesh(
    group,
    new THREE.PlaneGeometry(3, 0.75),
    new THREE.MeshStandardMaterial({
      map: texture,
      transparent: true,
      depthWrite: false,
    }),
    [0, -1.844, z],
  );
  m.rotation.x = Math.PI / 2;
}
function plume(group, x, y, z, r = 1, seed = 0) {
  const g = createEnginePlume(r, seed);
  g.position.set(x, y, z);
  group.add(g);
  return g;
}
/** Original geometry in the same local frame and dimensions as the MuJoCo body. */
export function createRocket() {
  const booster = new THREE.Group(),
    upper = new THREE.Group(),
    root = new THREE.Group();
  root.rotation.x = -Math.PI / 2;
  booster.add(root);
  cylinder(root, 1.83, 33.8, -0.5, mats.white);
  cylinder(root, 1.83, 4.04, 18.42, mats.black);
  cylinder(root, 1.91, 0.26, 20.31, mats.steel);
  cylinder(root, 1.9, 1.5, -18.15, mats.black);
  cylinder(root, 1.66, 0.64, -19.22, mats.black);
  for (const z of [-16.8, -10, 0, 10, 16.35])
    cylinder(root, 1.835, 0.032, z, mats.steel);
  mesh(
    root,
    new THREE.BoxGeometry(0.15, 0.1, 31),
    mats.black,
    [0, -1.835, -0.2],
  );
  label(root, "F9  /  SCvx", 5);
  label(root, "01", -7);
  const bells = [],
    flames = [];
  for (let i = 0; i < 9; i++) {
    const a = ((i - 1) * Math.PI) / 4,
      x = i ? 1.06 * Math.cos(a) : 0,
      y = i ? 1.06 * Math.sin(a) : 0;
    const g = new THREE.Group();
    g.position.set(x, y, -19.5);
    root.add(g);
    lathe(
      g,
      [
        [0.17, 0],
        [0.18, 0.12],
        [0.23, 0.32],
        [0.38, 0.6],
      ],
      mats.bell,
    ).rotation.x = -Math.PI / 2;
    bells.push(g);
    flames.push(plume(root, x, y, -20.12, 0.43, i * 7.13));
  }
  const fins = new THREE.Group();
  root.add(fins);
  for (let i = 0; i < 4; i++) {
    const g = new THREE.Group();
    g.rotation.z = (i * Math.PI) / 2;
    g.position.z = 15.25;
    fins.add(g);
    for (const y of [-0.7, 0.7])
      rod(g, V(1.84, y, 0), V(3.1, y, 0), 0.06, mats.black);
    for (const x of [1.84, 3.1])
      rod(g, V(x, -0.7, 0), V(x, 0.7, 0), 0.06, mats.black);
    for (let j = 0; j < 7; j++) {
      const a = 1.84 + (j * 1.26) / 6;
      rod(g, V(a, -0.7, 0), V(a, 0.7, 0), 0.025, mats.steel);
      const b = -0.7 + (j * 1.4) / 6;
      rod(g, V(1.84, b, 0), V(3.1, b, 0), 0.025, mats.steel);
    }
  }
  const legs = [];
  for (let i = 0; i < 4; i++) {
    const g = new THREE.Group();
    g.rotation.z = (i * Math.PI) / 2;
    root.add(g);
    const main = rod(g, V(1.63, 0, -13.1), V(2.05, 0, -3.1), 0.15, mats.black),
      strut = rod(g, V(1.72, 0, -17.15), V(2.05, 0, -3.1), 0.09, mats.steel),
      foot = mesh(g, new THREE.SphereGeometry(1, 16, 8), mats.black);
    foot.scale.set(0.72, 0.38, 0.22);
    legs.push({ main, strut, foot });
  }
  const upperRoot = new THREE.Group();
  upperRoot.rotation.x = -Math.PI / 2;
  upper.add(upperRoot);
  cylinder(upperRoot, 1.83, 13.4, 27.15, mats.white);
  cylinder(upperRoot, 1.86, 0.6, 20.72, mats.black);
  label(upperRoot, "02", 28);
  const points = [
    [1.83, 33.5],
    [2.3, 34],
    [2.6, 35],
    [2.6, 41.05],
  ];
  for (let i = 1; i <= 28; i++) {
    const a = ((Math.PI / 2) * i) / 28;
    points.push([2.6 * Math.cos(a), 41.05 + 6.7 * Math.sin(a)]);
  }
  lathe(upperRoot, points, mats.white);
  const uflame = plume(upperRoot, 0, 0, 20.1, 1.2);
  lathe(
    upperRoot,
    [
      [0.35, 20.8],
      [0.35, 20.5],
      [0.5, 20.2],
      [0.9, 19.8],
    ],
    mats.bell,
  );
  const steering = createNozzleSteering(),
    exitOffset = new THREE.Vector3();
  return {
    booster,
    upper,
    update(frame, showPlumes = true, renderTime = frame.t) {
      const deployment = frame.legs ?? 0;
      for (const leg of legs) {
        const end = V(2.05 + 6.45 * deployment, 0, -3.1 - 17.28 * deployment);
        moveRod(leg.main, V(1.63, 0, -13.1), end);
        moveRod(leg.strut, V(1.72, 0, -17.15), end);
        leg.foot.position.copy(end);
      }
      fins.visible = !frame.attached;
      const n = frame.engines ?? 9,
        f = Math.hypot(...frame.control.slice(0, 3)),
        ratio = f / Math.max(1, n * ROCKET.engineMax);
      const nozzlePose = steering.update(renderTime, frame.control);
      flames.forEach((p, i) => {
        p.visible =
          showPlumes &&
          f > 1000 &&
          (n === 9 || i === 0 || (n === 3 && [1, 5].includes(i)));
        if (f > 1000) {
          p.quaternion.copy(nozzlePose);
          bells[i].quaternion.copy(p.quaternion);
          exitOffset.set(0, 0, -.62).applyQuaternion(p.quaternion);
          p.position.copy(bells[i].position).add(exitOffset);
        }
        p.userData.update(renderTime, ratio, frame.p?.[2] ?? booster.position.y);
      });
      uflame.visible = showPlumes && frame.upper?.thrust > 0;
      uflame.userData.update(renderTime, (frame.upper?.thrust ?? 0) / 981000, frame.upper?.p?.[2] ?? upper.position.y);
    },
  };
}
