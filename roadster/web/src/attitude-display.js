// Signed bank angle from the current [w,x,y,z] attitude, independent of
// accumulated angular travel. Upright is zero; inverted is +/-180 degrees.
export function bodyRollDegrees([w, x, y, z]) {
  const angle = Math.atan2(2 * (w * x + y * z), w * w + z * z - x * x - y * y) * 180 / Math.PI;
  return Math.abs(angle) < 0.05 ? 0 : angle;
}
