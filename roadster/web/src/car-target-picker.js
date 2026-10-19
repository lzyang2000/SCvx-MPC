import * as THREE from "three";

// Landing remains on the runway centerline; the shaded strip selects X.
export function landingBounds(maneuver) {
  return maneuver === "barrel-roll" ? [50, 64] : [-28, -12];
}

export function createGroundTargetPicker({
  canvas,
  camera,
  bounds,
  enabled,
  onSelect,
}) {
  const raycaster = new THREE.Raycaster();
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const point = new THREE.Vector3();
  let press = null;
  function pick(event) {
    if (!enabled()) return null;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    const x = (event.clientX - rect.left) / rect.width;
    const y = (event.clientY - rect.top) / rect.height;
    if (x < 0 || x > 1 || y < 0 || y > 1) return null;
    raycaster.setFromCamera(new THREE.Vector2(2 * x - 1, 1 - 2 * y), camera);
    if (!raycaster.ray.intersectPlane(plane, point)) return null;
    const [min, max] = bounds();
    if (point.x < min || point.x > max || Math.abs(point.z) > 3.5) return null;
    return Math.round(point.x * 10) / 10;
  }
  function down(event) {
    if (event.button !== 0 || event.isPrimary === false || press) {
      press = null;
      return;
    }
    if (enabled())
      press = {
        id: event.pointerId,
        x: event.clientX,
        y: event.clientY,
        dragged: false,
      };
  }
  function move(event) {
    if (
      press &&
      event.pointerId === press.id &&
      Math.hypot(event.clientX - press.x, event.clientY - press.y) > 6
    )
      press.dragged = true;
    canvas.style.cursor = pick(event) === null ? "" : "crosshair";
  }
  function up(event) {
    const start = press;
    press = null;
    if (
      !start ||
      event.pointerId !== start.id ||
      event.button !== 0 ||
      start.dragged ||
      Math.hypot(event.clientX - start.x, event.clientY - start.y) > 6
    )
      return;
    const x = pick(event);
    if (x !== null) onSelect(x);
  }
  function cancel() {
    press = null;
    canvas.style.cursor = "";
  }
  for (const [name, listener] of Object.entries({
    pointerdown: down,
    pointermove: move,
    pointerup: up,
    pointercancel: cancel,
    pointerleave: cancel,
  }))
    canvas.addEventListener(name, listener);
  return { cancel };
}
