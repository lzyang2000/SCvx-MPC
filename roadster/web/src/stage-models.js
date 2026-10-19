import * as THREE from "three";
import { STAGE_VEHICLES } from "./car-stage.js";

// Original simplified meshes, with approximate dimensions and recognizable profiles.
export function createStageVehicles() {
  const row = new THREE.Group();
  row.name = "Parked Tesla display";
  const material = (color, metalness = 0.35, roughness = 0.4) =>
    new THREE.MeshStandardMaterial({ color, metalness, roughness });
  const tire = material("#22282a", 0.05, 0.9),
    alloy = material("#929a9d", 0.8, 0.27);
  const glass = material("#172a32", 0.65, 0.19),
    trim = material("#333c40", 0.5, 0.5);
  const lamp = new THREE.MeshStandardMaterial({
    color: "#f6f8ed",
    emissive: "#e1f3f1",
    emissiveIntensity: 0.3,
  });
  const tail = new THREE.MeshStandardMaterial({
    color: "#a12720",
    emissive: "#8f251d",
    emissiveIntensity: 0.3,
  });
  function mesh(group, geometry, mat, position) {
    const m = new THREE.Mesh(geometry, mat);
    m.position.set(...position);
    m.castShadow = true;
    m.receiveShadow = true;
    group.add(m);
    return m;
  }
  function box(g, size, pos, mat) {
    return mesh(g, new THREE.BoxGeometry(...size), mat, pos);
  }
  function profile(g, points, width, mat) {
    const s = new THREE.Shape();
    points.forEach(([z, y], i) => (i ? s.lineTo(z, y) : s.moveTo(z, y)));
    s.closePath();
    const geo = new THREE.ExtrudeGeometry(s, {
      depth: width,
      bevelEnabled: false,
      steps: 1,
    });
    geo.rotateY(-Math.PI / 2);
    geo.translate(width / 2, 0, 0);
    return mesh(g, geo, mat, [0, 0, 0]);
  }
  function wheels(g, width, zs, r) {
    for (const z of zs)
      for (const side of [-1, 1]) {
        const x = side * (width / 2 - 0.15);
        const t = mesh(g, new THREE.CylinderGeometry(r, r, 0.23, 24), tire, [
          x,
          r,
          z,
        ]);
        t.rotation.z = Math.PI / 2;
        const rim = mesh(
          g,
          new THREE.CylinderGeometry(r * 0.67, r * 0.67, 0.245, 12),
          alloy,
          [x, r, z],
        );
        rim.rotation.z = Math.PI / 2;
        const hub = mesh(
          g,
          new THREE.CylinderGeometry(r * 0.22, r * 0.22, 0.255, 12),
          trim,
          [x, r, z],
        );
        hub.rotation.z = Math.PI / 2;
      }
  }
  for (const v of STAGE_VEHICLES) {
    const g = new THREE.Group();
    g.name = v.name;
    g.userData.stageVehicle = v;
    g.position.x = v.x;
    row.add(g);
    const [w, l, h] = v.size,
      body = material(v.color, v.kind === "truck" ? 0.75 : 0.48, 0.31);
    if (v.kind === "semi") {
      // Trailerless tractor: tall aero cab, split windshield and tandem rear axles.
      box(g, [w * 0.85, 0.3, l * 0.84], [0, 0.9, -0.15], trim);
      profile(
        g,
        [
          [-0.9, 1],
          [-1.1, 3.65],
          [-0.7, h],
          [1.4, h],
          [2.1, 3.5],
          [3.45, 1.4],
          [3.65, 0.75],
        ],
        w * 0.91,
        body,
      );
      profile(
        g,
        [
          [1.42, 3.78],
          [2.02, 3.34],
          [2.72, 2.35],
          [1.35, 2.27],
        ],
        w * 0.92,
        glass,
      );
      box(g, [0.045, 1.35, 0.055], [0, 3.02, 1.99], trim).rotation.x = -0.42;
      for (const side of [-1, 1]) {
        box(g, [0.1, 0.5, 0.18], [side * (w / 2 - 0.07), 2.15, 2.15], trim);
        box(g, [0.32, 0.1, 0.08], [side * 0.9, 0.94, 3.55], lamp);
      }
      box(g, [w * 0.72, 0.16, 0.4], [0, 1.16, -2.7], alloy);
      wheels(g, w, [2.35, -1.65, -2.85], 0.52);
    } else if (v.kind === "truck") {
      profile(
        g,
        [
          [-l / 2, 0.55],
          [-l / 2, 1.35],
          [-0.25, h],
          [1.35, 1.43],
          [l / 2, 0.85],
          [l / 2, 0.48],
        ],
        w * 0.93,
        body,
      );
      profile(
        g,
        [
          [-0.25, h - 0.04],
          [1.35, 1.4],
          [1.42, 1.17],
          [-1.45, 1.19],
        ],
        w * 0.94,
        glass,
      );
      // Sloping stainless-steel tonneau and straight light bars.
      box(g, [w * 0.78, 0.07, 1.25], [0, 1.24, -1.65], trim).rotation.x = -0.15;
      box(g, [w * 0.85, 0.055, 0.045], [0, 0.91, l / 2 - 0.03], lamp);
      box(g, [w * 0.85, 0.05, 0.04], [0, 0.92, -l / 2 + 0.03], tail);
      wheels(g, w, [-1.78, 1.82], 0.43);
    } else {
      const cab = v.kind === "cab",
        suv = v.kind === "crossover";
      profile(
        g,
        [
          [-l / 2, 0.43],
          [-l / 2, 0.83],
          [-1.28, 1.03],
          [-0.75, h * 0.97],
          [0.55, h],
          [1.4, 1.02],
          [l / 2, 0.73],
          [l / 2, 0.42],
        ],
        w * 0.94,
        body,
      );
      profile(
        g,
        [
          [-1.28, 1.04],
          [-0.72, h * 0.96],
          [0.55, h * 0.97],
          [1.36, 1.04],
        ],
        w * 0.82,
        glass,
      );
      // Side window pillars, door seams and inset handles.
      for (const side of [-1, 1]) {
        box(
          g,
          [0.022, (h - 1) * 0.75, 0.055],
          [side * w * 0.414, (h + 1.05) / 2, -0.05],
          trim,
        );
        for (const z of [-0.65, 0.8])
          box(g, [0.018, 0.022, 0.16], [side * w * 0.476, 0.97, z], trim);
        for (const z of [-0.75, 0.7])
          box(
            g,
            [0.25, 0.06, 0.05],
            [side * w * 0.32, 0.74, z > 0 ? l / 2 - 0.03 : -l / 2 + 0.03],
            z > 0 ? lamp : tail,
          );
      }
      if (cab) {
        box(g, [w * 0.85, 0.04, 0.04], [0, 0.78, l / 2 - 0.03], lamp);
        box(g, [w * 0.75, 0.04, 0.04], [0, 0.79, -l / 2 + 0.03], tail);
      }
      wheels(g, w, [-l * 0.31, l * 0.31], suv ? 0.36 : 0.34);
    }
  }
  return row;
}
