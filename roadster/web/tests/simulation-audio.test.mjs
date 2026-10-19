import test from "node:test";
import assert from "node:assert/strict";
import { soundState, createSimulationAudio } from "../src/simulation-audio.js";

test("audio follows nozzle forces, ground motion and engine cutoff", () => {
  const car = { phase: "APPROACH", v: [12, 0, 0], jets: [0, 0, 0, 0, 0, 0] };
  assert.ok(soundState("car", car).motor > 0);
  const flight = soundState("car", {
    ...car,
    phase: "FLIP",
    jets: [8500, 8500, 8500, 8500, 0, 8500],
  });
  assert.equal(flight.motor, 0);
  assert.ok(flight.jets > 0.9);
  const burning = { control: [0, 0, 7605000], attached: true };
  assert.equal(soundState("rocket", burning).engine, 1);
  assert.equal(
    soundState("rocket", { ...burning, control: [0, 0, 0] }).engine,
    0,
  );
  assert.equal(
    soundState("rocket", {
      ...burning,
      attached: false,
      upper: { thrust: 981000 },
    }).upper,
    1,
  );
  assert.deepEqual(soundState("car", null), soundState("rocket", null));
});

class Control extends EventTarget {
  value = "0.45";
  checked = false;
  textContent = "";
  setAttribute() {}
  click() {
    this.dispatchEvent(new Event("click"));
  }
}
class Param {
  value = 0;
  setTargetAtTime(value) {
    this.value = value;
  }
  setValueAtTime(value) {
    this.value = value;
  }
  exponentialRampToValueAtTime(value) {
    this.value = value;
  }
}
class Node {
  gain = new Param();
  frequency = new Param();
  Q = new Param();
  threshold = new Param();
  knee = new Param();
  ratio = new Param();
  attack = new Param();
  release = new Param();
  connect(node) {
    return node;
  }
  disconnect() {}
  start() {}
  stop() {
    this.stopped = true;
  }
}
class Context {
  static instances = [];
  currentTime = 0;
  sampleRate = 100;
  state = "suspended";
  destination = new Node();
  gains = [];
  constructor() {
    Context.instances.push(this);
  }
  createGain() {
    const node = new Node();
    this.gains.push(node);
    return node;
  }
  createDynamicsCompressor() {
    return new Node();
  }
  createBiquadFilter() {
    return {
      frequency: new Param(),
      Q: new Param(),
      connect(node) {
        return node;
      },
    };
  }
  createOscillator() {
    return new Node();
  }
  createBufferSource() {
    return new Node();
  }
  createBuffer() {
    return {
      getChannelData() {
        return new Float32Array(10);
      },
    };
  }
  resume() {
    this.state = "running";
    return Promise.resolve();
  }
  close() {
    this.state = "closed";
    return Promise.resolve();
  }
}
function fixture() {
  const root = new EventTarget(),
    win = new EventTarget();
  root.hidden = false;
  const controls = Object.fromEntries(
    ["sound-toggle", "sound-volume", "sound-music"].map((id) => [
      id,
      new Control(),
    ]),
  );
  root.getElementById = (id) => controls[id];
  win.AudioContext = Context;
  win.location = { origin: "https://example.com" };
  win.localStorage = { getItem: () => '{"music":false}', setItem() {} };
  return { root, win, controls };
}

test("website audio defaults on at 30% and silences on pause, live starvation, tab switch and visibility", () => {
  const f = fixture(),
    before = Context.instances.length;
  const audio = createSimulationAudio("car", f);
  assert.equal(
    Context.instances.length,
    before,
    "no audio resources before a gesture",
  );
  assert.equal(f.controls["sound-toggle"].textContent, "Sound on");
  assert.equal(Number(f.controls["sound-volume"].value), 0.3);
  f.root.dispatchEvent(new Event("pointerdown"));
  const ctx = Context.instances.at(-1);
  assert.equal(ctx.state, "running");
  const frame = { phase: "APPROACH", v: [10, 0, 0], jets: [] };
  audio.update(frame, { time: 1, playing: true });
  assert.equal(ctx.gains[0].gain.value, 0.3);
  audio.update(frame, { time: 1, playing: false });
  assert.equal(ctx.gains[0].gain.value, 0);
  ctx.currentTime = 1;
  audio.update(frame, { time: 2, playing: true });
  ctx.currentTime = 1.3;
  audio.update(frame, { time: 2, playing: true });
  assert.equal(
    ctx.gains[0].gain.value,
    0,
    "buffer starvation must not leave an engine sounding",
  );
  f.win.dispatchEvent(
    new MessageEvent("message", {
      origin: f.win.location.origin,
      data: { type: "pause-demo" },
    }),
  );
  audio.update(frame, { time: 3, playing: true });
  assert.equal(ctx.gains[0].gain.value, 0);
  f.win.dispatchEvent(
    new MessageEvent("message", {
      origin: f.win.location.origin,
      data: { type: "resume-demo" },
    }),
  );
  audio.update(frame, { time: 4, playing: true });
  assert.equal(ctx.gains[0].gain.value, 0.3);
  f.root.hidden = true;
  f.root.dispatchEvent(new Event("visibilitychange"));
  assert.equal(ctx.gains[0].gain.value, 0);
  f.root.hidden = false;
  f.controls["sound-toggle"].click();
  f.root.dispatchEvent(new Event("pointerdown"));
  f.root.dispatchEvent(new Event("click"));
  f.root.dispatchEvent(new Event("keydown"));
  audio.update(frame, { time: 5, playing: true });
  assert.equal(
    ctx.gains[0].gain.value,
    0,
    "mute persists during running playback",
  );
  audio.dispose();
  assert.equal(ctx.state, "closed");
  assert.doesNotThrow(() => audio.dispose());
});

test("partial audio initialization is cleaned up and can be retried", () => {
  const f = fixture();
  let fail = true;
  f.win.AudioContext = class extends Context {
    createOscillator() {
      if (fail) throw new Error("Audio device unavailable");
      return super.createOscillator();
    }
  };
  const audio = createSimulationAudio("car", f);
  f.root.dispatchEvent(new Event("pointerdown"));
  assert.equal(Context.instances.at(-1).state, "closed");
  assert.doesNotThrow(() => audio.update({}, { time: 1, playing: true }));
  fail = false;
  f.controls["sound-toggle"].click();
  assert.equal(Context.instances.at(-1).state, "running");
  audio.dispose();
});
