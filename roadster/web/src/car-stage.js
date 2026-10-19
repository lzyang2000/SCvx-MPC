// Shared positions for physics, live landing markers and camera framing.
export const CAR_LAYOUT = {
  "c-ramp": { start: -20, landing: -20 },
  "barrel-roll": { start: 8, takeoff: 14, landing: 50 },
};
// Approximate parked display vehicles. Coordinates use the physics Z-up frame.
// They face across the Roadster's path; size = [width, length, height].
export const STAGE_VEHICLES = [
  {
    name: "Model 3",
    kind: "sedan",
    x: 25,
    size: [1.85, 4.72, 1.44],
    color: "#294d85",
  },
  {
    name: "Model Y",
    kind: "crossover",
    x: 29,
    size: [1.98, 4.79, 1.62],
    color: "#9d302e",
  },
  { name: "Semi", kind: "semi", x: 34, size: [2.6, 7.4, 4], color: "#e3e5df" },
  {
    name: "Cybertruck",
    kind: "truck",
    x: 39,
    size: [2.2, 5.68, 1.79],
    color: "#a4adb0",
  },
  {
    name: "Cybercab",
    kind: "cab",
    x: 43,
    size: [1.9, 4.5, 1.5],
    color: "#c6a86a",
  },
];
// A conservative overlap check using the moving body's rotated bounding box.
// The reference points include the tires, so clearance is measured below them.
export function stageClearance(points) {
  const lo = [0, 1, 2].map((i) => Math.min(...points.map((p) => p[i])));
  const hi = [0, 1, 2].map((i) => Math.max(...points.map((p) => p[i])));
  let clearance = Infinity,
    vehicle = null;
  for (const v of STAGE_VEHICLES) {
    if (
      hi[0] < v.x - v.size[0] / 2 ||
      lo[0] > v.x + v.size[0] / 2 ||
      hi[1] < -v.size[1] / 2 ||
      lo[1] > v.size[1] / 2
    )
      continue;
    const gap = lo[2] - v.size[2];
    if (gap < clearance) {
      clearance = gap;
      vehicle = v.name;
    }
  }
  return { clearance, vehicle };
}
