import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import load from "../node_modules/@mujoco/mujoco/mujoco.js";
import { initSolver } from "../src/compute/conic.js";
import { RocketMission } from "../src/compute/rocket-mission.js";
import { ROCKET } from "../src/compute/rocket-dynamics.js";
import { norm } from "../src/compute/math.js";

await initSolver(
  await readFile(
    new URL(
      "../src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
      import.meta.url,
    ),
  ),
);
const mj = await load();
const xml = await readFile(
  new URL("../public/assets/rocket.xml", import.meta.url),
  "utf8",
);

// The nominal 75 km mission is covered in compute.test.mjs.
// Exercise both ends of the separation-altitude control.
for (const km of [70, 80]) {
  test(`${km} km mission returns gently without a minimum-throttle flyaway`, () => {
    const m = new RocketMission(mj, xml, { separationAltitude: km * 1000 });
    let ended = false,
      sawDirect = false,
      nearGroundSamples = 0;
    try {
      for (let i = 0; i < 150000; i++) {
        const out = m.step(),
          f = m.frame();
        if (out.solve)
          assert.equal(out.solve.success, true, JSON.stringify(out.solve));
        if (f.phase === "LANDING" && m.landingHeight() < 80) {
          nearGroundSamples++;
          assert.ok(
            f.v[2] < 0.2,
            `must keep descending near the pad: ${f.v[2]} m/s`,
          );
          assert.ok(
            norm(f.control.slice(0, 3)) <= f.engines * ROCKET.engineMax + 2,
          );
          assert.ok(
            f.control[2] >=
              ROCKET.minThrottle * f.engines * ROCKET.engineMax - 2,
          );
          if (f.landing_mode === "direct-landing") {
            sawDirect = true;
            assert.equal(f.landing_hold, false);
            assert.equal(m.hoverStartedAt, null);
          }
        }
        if (out.done) {
          ended = true;
          break;
        }
      }
      const s = m.summary();
      assert.ok(ended);
      assert.equal(s.success, true, JSON.stringify(s));
      assert.equal(s.mpc_rejected, 0);
      assert.ok(Math.abs(s.separation.altitude - km * 1000) < 50);
      assert.ok(s.separation.speed > 1400 && s.separation.speed < 1900);
      assert.equal(s.separation.booster_mass, ROCKET.dry + s.separation.fuel);
      assert.ok(s.separation.fuel > s.fuel_remaining);
      assert.equal(s.post_separation_ignitions, 3);
      assert.ok(nearGroundSamples > 10);
      assert.ok(s.touchdown.vertical_speed < 0.3, JSON.stringify(s.touchdown));
      assert.ok(s.touchdown.horizontal_speed < 0.5);
      assert.ok(s.touchdown.tilt_degrees < 1);
      assert.ok(s.pad_error < 2);
      assert.ok(s.fuel_remaining > 0);
      if (s.landing_profile === "direct") {
        assert.ok(
          sawDirect,
          "a light booster cannot hover at minimum throttle",
        );
        assert.equal(s.landing_profile, "direct");
        assert.equal(s.hover_started_at, null);
        assert.equal(s.slow_descent_started_at, null);
      }
    } finally {
      m.dispose();
    }
  });
}
