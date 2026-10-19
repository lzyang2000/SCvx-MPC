import { initSolver } from "./conic.js";
import { NOZZLES } from "./car-mission.js";
import { MuJoCoCarMission } from "./car-mujoco.js";
import { prepareCarPlan } from "./car-planner.js";
import { RocketMission } from "./rocket-mission.js";
import { prepareRocketPlan, replanRocketPlan } from "./rocket-planner.js";
const pause = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
self.onmessage = async ({
  data: { demo, config = {}, live = true, speed = 1 },
}) => {
  let mission;
  const send = (type, data) => self.postMessage({ type, data });
  try {
    send("status", { message: "Loading browser solver…" });
    await initSolver();
    send("status", { message: "Loading MuJoCo physics…" });
    const { default: load } = await import("../../vendor/mujoco/mujoco.js");
    const mj = await load();
    if (demo === "rocket") {
      const response = await fetch(
        new URL("../../assets/rocket.xml", import.meta.url),
      );
      if (!response.ok) throw Error("Rocket physics model could not be loaded");
      mission = new RocketMission(mj, await response.text(), config);
    } else mission = new MuJoCoCarMission(mj, config);
    const resolved =
      demo === "rocket"
        ? {
            separationAltitude: mission.separationAltitude,
            initialFuel: mission.initialFuel,
            use_mpc: true,
            goal_planning: true,
          }
        : mission.config;
    send("start", { config: resolved });
    send("frame", mission.frame());
    const plan = await (demo === "rocket" ? prepareRocketPlan : prepareCarPlan)(
      mission,
      (message) => send("status", { message, planning: true }),
    );
    if (plan) send("plan", plan);
    const frames = [],
      started = performance.now();
    let lastFrame = -Infinity,
      lastYield = started;
    while (true) {
      const out = mission.step();
      if (out.planning) {
        const plan = await replanRocketPlan(mission, (message) =>
          send("status", { message }),
        );
        send("plan", plan);
        continue;
      }
      const t = demo === "rocket" ? mission.data.time : mission.t;
      if (out.solve) send("solve", out.solve);
      if (
        t - lastFrame >= (demo === "rocket" ? 0.1 : 0.03) - 1e-8 ||
        out.done
      ) {
        const frame = mission.frame();
        frames.push(frame);
        if (live)
          await pause(
            Math.max(0, started + (t / speed) * 1000 - performance.now()),
          );
        send("frame", frame);
        lastFrame = t;
      }
      if (out.done) break;
      if (performance.now() - lastYield > 10) {
        // Pace newly integrated states. Optimization stays entirely in this worker.
        await pause(0);
        lastYield = performance.now();
      }
    }
    const result = {
      config: resolved,
      frames,
      events: mission.events,
      summary: {
        ...mission.summary(),
        mpc_solves: mission.solves,
        compute_seconds:
          (performance.now() - started) / 1000 +
          (demo === "rocket" ? mission.flightPlanHistory[0].seconds : 0),
      },
      model:
        demo === "rocket"
          ? "MuJoCo WASM / SCvx booster"
          : "MuJoCo WASM / SCvx six fixed jets",
    };
    result.flight_plan =
      mission.flightPlanHistory?.[0] ?? mission.flightPlan ?? null;
    if (demo === "rocket") result.flight_plans = mission.flightPlanHistory;
    if (demo === "car") {
      result.nozzles = NOZZLES;
      result.landing_target = [mission.target, 0, 0.62];
    }
    send("complete", result);
  } catch (error) {
    send("error", { error: error.message });
  } finally {
    mission?.dispose?.();
    self.close();
  }
};
