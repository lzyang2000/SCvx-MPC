import * as THREE from "three";

// Scene-local Y is the rocket axis. Include deployed feet and full plumes so
// throttle changes cannot make the camera pulse in and out.
const bounds = {
  stack: { min: -60, max: 48, radial: 10 },
  booster: { min: -60, max: 21, radial: 10 },
  upper: { min: -14, max: 48, radial: 14 },
};
const viewDirection = new THREE.Vector3(48, 40, 72).normalize();
const frameLimit = 0.72; // Leave room for the view buttons and telemetry overlay.

export function createRocketCamera(camera, controls) {
  const focus = new THREE.Vector3();
  let center = null;
  let distance = null;
  return {
    update(body, part, { snap = false, dt = 1 / 60 } = {}) {
      const b = bounds[part];
      const desiredCenter = (b.min + b.max) / 2 + 10;
      const alpha = snap ? 1 : -Math.expm1(-Math.max(0, dt) / 0.45);
      center =
        center === null
          ? desiredCenter
          : THREE.MathUtils.lerp(center, desiredCenter, alpha);

      // Bound the model around the moving framing center, including while the
      // camera recenters after staging. Fit the sphere in both screen axes.
      const radius = Math.hypot(
        b.radial,
        Math.max(b.max - center, center - b.min),
      );
      const halfFov = Math.atan(
        frameLimit *
          Math.tan(THREE.MathUtils.degToRad(camera.getEffectiveFOV()) / 2) *
          Math.min(1, camera.aspect),
      );
      const required = radius / Math.sin(halfFov);
      distance =
        distance === null
          ? required
          : Math.max(required, THREE.MathUtils.lerp(distance, required, alpha));

      // Track the interpolated pose directly, even at accelerated playback.
      // Smooth only framing changes; position smoothing would lag behind ascent.
      focus
        .set(0, center, 0)
        .applyQuaternion(body.quaternion)
        .add(body.position);
      controls.target.copy(focus);
      camera.position.copy(focus).addScaledVector(viewDirection, distance);
      camera.lookAt(focus);
    },
  };
}
