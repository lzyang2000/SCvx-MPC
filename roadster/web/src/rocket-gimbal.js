import * as THREE from "three";

// Visual nozzle response only. Physics and the displayed MPC command stay exact.
export function createNozzleSteering(response = 0.18) {
  const axis = new THREE.Vector3(0, 0, 1),
    direction = new THREE.Vector3(),
    target = new THREE.Quaternion(),
    pose = new THREE.Quaternion();
  let previous = null, wasPowered = false;
  return {
    update(time, force) {
      const powered = Math.hypot(...force.slice(0, 3)) > 1000,
        dt = previous === null ? 0 : time - previous;
      if (powered) {
        direction.fromArray(force).normalize();
        target.setFromUnitVectors(axis, direction);
        // A new burn or a timeline seek starts at the requested orientation.
        if (!wasPowered || previous === null || dt < 0 || dt > 1)
          pose.copy(target);
        else if (dt > 0)
          pose.slerp(target, -Math.expm1(-dt / response));
      }
      previous = time;
      wasPowered = powered;
      return pose;
    },
  };
}
