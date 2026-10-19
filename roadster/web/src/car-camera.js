import { CAR_LAYOUT, STAGE_VEHICLES } from "./car-stage.js";
// Fixed world framing keeps roll out of the camera orientation.
export function carSceneView(run, aspect, fov, view = "overview") {
  const barrel = run?.config.maneuver === "barrel-roll",
    r = run?.config.radius ?? 10;
  const xs = run?.frames.map((f) => f.p[0]) ?? [];
  const layout = CAR_LAYOUT[barrel ? "barrel-roll" : "c-ramp"];
  const minX = Math.min(
    layout.start - 3,
    ...xs,
    run?.landing_target?.[0] ?? layout.landing,
  );
  const maxX = barrel
      ? Math.max(
          (run?.landing_target?.[0] ??
            run?.config.landing_x ??
            layout.landing) + 5,
          ...xs,
          ...STAGE_VEHICLES.map((v) => v.x + v.size[0] / 2),
        ) + 3
      : r + 3,
    center = (minX + maxX) / 2;
  const height = barrel ? 7 : r * 0.85;
  // Sideways motion can bring the body closer to the side camera.
  const depth = Math.max(
    0,
    ...(run?.frames.map((f) => Math.abs(f.p[1])) ?? []),
  );
  const distance =
    depth +
    Math.max(
      50,
      ((maxX - minX) / (2 * Math.tan((fov * Math.PI) / 360) * aspect)) * 1.2,
    );
  return {
    target: [center, height, 0],
    position:
      view === "side"
        ? [center, height, distance * 1.05]
        : [center - distance * 0.22, height + distance * 0.32, distance],
    distance,
  };
}
