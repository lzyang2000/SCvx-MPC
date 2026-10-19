import test from "node:test";
import assert from "node:assert/strict";
import { createLivePlaybackClock } from "../src/live-playback.js";

test("live playback builds a queue before moving", () => {
  const clock = createLivePlaybackClock();
  assert.equal(clock.advance(0.016, 0.49), 0);
  assert.equal(clock.advance(0.016, 0.51), 0.016);
});

test("the sample queue covers a 470 ms solver pause at ramp release", () => {
  for (const fps of [30, 60, 120]) {
    const clock = createLivePlaybackClock();
    let previous = 0;
    for (let tick = 0; tick < 8 * fps; tick++) {
      const wall = tick / fps;
      const latest =
        wall >= 5.35 && wall < 5.82 ? 5.34 : Math.floor(wall / 0.03) * 0.03;
      const time = clock.advance(1 / fps, latest);
      assert.ok(time <= latest, "never extrapolate beyond received physics");
      if (wall > 1) {
        assert.ok(
          time - previous >= 0.999 / fps,
          `playback stalled at ${wall}s at ${fps} fps`,
        );
        assert.ok(time - previous <= 1.101 / fps, "catch-up stays gradual");
      }
      previous = time;
    }
  }
});

test("long outages stop at known samples and bursts cannot jump playback", () => {
  const clock = createLivePlaybackClock();
  for (let tick = 0; tick < 100; tick++) clock.advance(0.02, 0.6);
  assert.equal(clock.advance(0.02, 0.6), 0.6);
  const resumed = clock.advance(0.02, 3);
  assert.ok(resumed > 0.6 && resumed <= 0.622001);
  clock.reset(4);
  assert.equal(clock.advance(0.02, 4.49), 4);
  assert.equal(clock.advance(0.02, 4.6), 4.02);
});
