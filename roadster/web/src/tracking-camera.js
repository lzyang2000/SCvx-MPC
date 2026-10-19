import * as THREE from 'three';

const offsets = {
  follow: new THREE.Vector3(-7, 3.4, 8),
  mpc: new THREE.Vector3(-4, 3.5, 6),
};

/** One damped focus keeps camera distance and viewing direction constant. */
export function createTrackingCamera(camera, controls) {
  const focus = new THREE.Vector3();
  function place(mode) {
    camera.position.copy(focus).add(offsets[mode]);
    controls.target.copy(focus);
    camera.lookAt(focus);
  }
  return {
    snap(target, mode) { focus.copy(target); place(mode); },
    update(target, mode, seconds) {
      // Exponential damping has the same time constant at any render rate.
      focus.lerp(target, -Math.expm1(-Math.max(0, seconds)/.12));
      place(mode);
    },
  };
}
