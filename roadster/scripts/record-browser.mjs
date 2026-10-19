import { readFile, writeFile } from "node:fs/promises";
import load from "../web/node_modules/@mujoco/mujoco/mujoco.js";
import { initSolver } from "../web/src/compute/conic.js";
import { RocketMission } from "../web/src/compute/rocket-mission.js";
import {
  prepareRocketPlan,
  replanRocketPlan,
} from "../web/src/compute/rocket-planner.js";
import { CarMission, NOZZLES } from "../web/src/compute/car-mission.js";
const demo = process.argv[2] ?? "rocket";
if (!["car", "barrel", "rocket"].includes(demo))
  throw Error("Use car, barrel or rocket");
await initSolver(
  await readFile(
    new URL(
      "../web/src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
      import.meta.url,
    ),
  ),
);
const mission =
  demo === "rocket"
    ? new RocketMission(
        await load(),
        await readFile(
          new URL("../web/public/assets/rocket.xml", import.meta.url),
          "utf8",
        ),
      )
    : new CarMission(demo === "barrel" ? { maneuver: "barrel-roll" } : {});
const frames = [],
  began = performance.now();
let last = -Infinity;
try {
  if (demo === "rocket") await prepareRocketPlan(mission, console.log);
  for (let i = 0; i < 150000; i++) {
    const out = mission.step();
    if (out.planning) {
      await replanRocketPlan(mission, console.log);
      continue;
    }
    const t = demo === "rocket" ? mission.data.time : mission.t;
    if (t - last >= (demo === "rocket" ? 0.1 : 0.03) - 1e-8 || out.done) {
      frames.push(mission.frame());
      last = t;
    }
    if (out.done) break;
  }
  const config =
      demo === "rocket"
        ? {
            separationAltitude: mission.separationAltitude,
            initialFuel: mission.initialFuel,
            use_mpc: true,
            goal_planning: true,
          }
        : mission.config,
    result = {
      config,
      frames,
      events: mission.events,
      summary: {
        ...mission.summary(),
        compute_seconds: (performance.now() - began) / 1000,
        mpc_solves: mission.solves,
      },
      model:
        demo === "rocket"
          ? "MuJoCo WASM / SCvx booster"
          : "Browser rigid body / SCvx six fixed jets",
    };
  if (demo === "rocket") {
    result.flight_plan = mission.flightPlanHistory[0];
    result.flight_plans = mission.flightPlanHistory;
  }
  if (demo !== "rocket") {
    result.nozzles = NOZZLES;
    result.landing_target = [mission.target, 0, 0.62];
  }
  if (!result.summary.success)
    throw Error(JSON.stringify({ ...result.summary, mpc_solves: undefined }));
  await writeFile(
    new URL(`../web/public/${demo}-nominal.json`, import.meta.url),
    JSON.stringify(result),
  );
  console.log(
    demo,
    JSON.stringify({ ...result.summary, mpc_solves: undefined }),
  );
} finally {
  mission.dispose?.();
}
