/** Consume a real queue of physical samples while synchronous MPC solves run. */
export function createLivePlaybackClock({ bufferSeconds = .5 } = {}) {
  let time = 0;
  let started = false;
  return {
    reset(initialTime = 0) { time = initialTime; started = false; },
    advance(elapsed, latestTime) {
      if (!started) {
        if (latestTime - time < bufferSeconds) return time;
        started = true;
      }
      // Once started, consume available samples even if no new frame arrives.
      // Catch up gently only when a burst has left more than the target queue.
      const excess = Math.max(0, latestTime - time - bufferSeconds - .15);
      const speed = Math.min(1.1, 1 + excess / 3);
      time = Math.min(latestTime, time + Math.max(0, elapsed) * speed);
      return time;
    },
  };
}
