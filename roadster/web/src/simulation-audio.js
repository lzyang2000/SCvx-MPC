const clamp = (n, max = 1) => Math.max(0, Math.min(max, Number(n) || 0));

// Gains and pitch follow physical controls, independently of replay speed.
export function soundState(kind, frame) {
  if (!frame)
    return { motor: 0, motorHz: 100, jets: 0, fan: 0, engine: 0, upper: 0 };
  if (kind === "rocket")
    return {
      motor: 0,
      motorHz: 100,
      jets: 0,
      fan: 0,
      engine: Math.sqrt(
        clamp(Math.hypot(...(frame.control ?? []).slice(0, 3)) / 7605000),
      ),
      upper: frame.attached
        ? 0
        : Math.sqrt(clamp(frame.upper?.thrust / 981000)),
    };
  const speed = Math.hypot(...(frame.v ?? [0, 0, 0]));
  const grounded = ["APPROACH", "RAMP", "SETTLED", "CONTACT"].includes(
    frame.phase,
  );
  return {
    motor: grounded ? clamp(speed / 5) : 0,
    motorHz: 85 + clamp(speed, 40) * 19,
    jets: Math.sqrt(
      clamp((frame.jets ?? []).reduce((a, b) => a + clamp(b, 8500), 0) / 51000),
    ),
    fan: clamp(frame.fan_g / 2),
    engine: 0,
    upper: 0,
  };
}

export function createSimulationAudio(
  kind,
  { root = document, win = window } = {},
) {
  const button = root.getElementById("sound-toggle");
  const volume = root.getElementById("sound-volume");
  const music = root.getElementById("sound-music");
  const AudioContext = win.AudioContext || win.webkitAudioContext;
  let context, master, musicGain, musicSource, musicBuffer, musicLoading;
  let voices = [],
    enabled = true,
    blocked = false,
    running = false;
  let lastTime = null,
    lastMotion = 0,
    lastContact = false,
    alive = true;
  let settings = { volume: 0.3, music: true };
  try {
    Object.assign(
      settings,
      JSON.parse(win.localStorage.getItem("scvx-audio") || "{}"),
    );
  } catch {}
  volume.value = clamp(settings.volume);
  music.checked = settings.music !== false;
  function updateButton() {
    button.disabled = !AudioContext;
    button.setAttribute("aria-pressed", String(enabled && !!AudioContext));
    button.textContent = !AudioContext
      ? "Sound unavailable"
      : enabled
        ? "Sound on"
        : "Sound off";
  }
  updateButton();
  const target = (param, value, response = 0.045) =>
    param.setTargetAtTime(value, context.currentTime, response);

  function save() {
    try {
      win.localStorage.setItem(
        "scvx-audio",
        JSON.stringify({ volume: Number(volume.value), music: music.checked }),
      );
    } catch {}
  }
  function voice(source, level, filterType, frequency) {
    const filter = context.createBiquadFilter();
    filter.type = filterType;
    filter.frequency.value = frequency;
    filter.Q.value = 0.65;
    const gain = context.createGain();
    gain.gain.value = 0;
    source.connect(filter).connect(gain).connect(master);
    source.start?.();
    voices.push({ source, filter, gain, level });
  }
  function noise(low, high, level) {
    const source = context.createBufferSource();
    const buffer = context.createBuffer(
      2,
      context.sampleRate * 3,
      context.sampleRate,
    );
    for (let channel = 0; channel < 2; channel++) {
      const samples = buffer.getChannelData(channel);
      for (let i = 0; i < samples.length; i++)
        samples[i] = Math.random() * 2 - 1;
    }
    source.buffer = buffer;
    source.loop = true;
    const hp = context.createBiquadFilter();
    hp.type = "highpass";
    hp.frequency.value = low;
    source.connect(hp);
    voice(hp, level, "lowpass", high);
    // The filter is the voice input; the underlying source owns start and stop.
    voices.at(-1).source = source;
    source.start();
  }
  function oscillator(frequency, level, type = "sine") {
    const source = context.createOscillator();
    source.type = type;
    source.frequency.value = frequency;
    voice(source, level, "lowpass", 4000);
  }
  function stopMusic() {
    musicSource?.stop();
    musicSource = null;
  }
  function updateMusic() {
    if (!context) return;
    const active =
      enabled && running && !blocked && !root.hidden && music.checked;
    target(musicGain.gain, active ? 0.85 : 0, 0.15);
    if (active && musicBuffer && !musicSource) {
      musicSource = context.createBufferSource();
      musicSource.buffer = musicBuffer;
      musicSource.loop = true;
      musicSource.connect(musicGain);
      musicSource.start();
    } else if (!active && musicSource) {
      // Stop the source after a short fade, without stopping a newly resumed one.
      const old = musicSource;
      musicSource = null;
      old.stop(context.currentTime + 0.2);
    }
  }
  async function loadMusic() {
    if (musicBuffer || musicLoading || !music.checked || !context) return;
    const owner = context;
    musicLoading = (async () => {
      try {
        const response = await fetch(
          new URL("../assets/audio/music.mp3", import.meta.url),
        );
        if (!response.ok) throw Error("Music could not be loaded");
        const buffer = await owner.decodeAudioData(
          await response.arrayBuffer(),
        );
        if (alive) {
          musicBuffer = buffer;
          updateMusic();
        }
      } catch {
        /* Effects remain available if music cannot load. */
      } finally {
        musicLoading = null;
      }
    })();
    await musicLoading;
  }
  function initialize() {
    if (context) return;
    context = new AudioContext();
    master = context.createGain();
    master.gain.value = 0;
    const compressor = context.createDynamicsCompressor();
    compressor.threshold.value = -16;
    compressor.knee.value = 14;
    compressor.ratio.value = 8;
    compressor.attack.value = 0.004;
    compressor.release.value = 0.16;
    master.connect(compressor).connect(context.destination);
    musicGain = context.createGain();
    musicGain.gain.value = 0;
    musicGain.connect(master);
    oscillator(100, 0.09);
    oscillator(300, 0.025);
    noise(500, 11000, 0.32);
    noise(65, 650, 0.14);
    noise(24, 180, 0.72);
    noise(90, 3800, 0.4);
    noise(100, 2600, 0.12);
  }
  function silence() {
    running = false;
    if (context) {
      target(master.gain, 0, 0.015);
      updateMusic();
    }
  }
  function startAudio() {
    if (!enabled || !alive || blocked || !AudioContext) return;
    try {
      initialize();
      if (context.state === "suspended")
        context.resume().catch(() => silence());
      loadMusic();
    } catch {
      enabled = false;
      for (const { source } of voices) {
        try {
          source.stop();
        } catch {}
      }
      voices = [];
      context?.close().catch(() => {});
      context = null;
      updateButton();
    }
  }
  button.addEventListener("click", () => {
    enabled = !enabled;
    updateButton();
    if (enabled) startAudio();
    else silence();
  });
  volume.addEventListener("input", save);
  music.addEventListener("change", () => {
    save();
    loadMusic();
    updateMusic();
  });
  const message = (event) => {
    if (event.origin !== win.location.origin) return;
    if (event.data?.type === "pause-demo") {
      blocked = true;
      silence();
    }
    if (event.data?.type === "resume-demo") blocked = false;
  };
  const visibility = () => {
    if (root.hidden) silence();
  };
  // Browsers unlock audio on a gesture. Sound is enabled before that gesture.
  const resume = () => startAudio();
  win.addEventListener("message", message);
  root.addEventListener("visibilitychange", visibility);
  root.addEventListener("pointerdown", resume);
  root.addEventListener("click", resume);
  root.addEventListener("keydown", resume);
  const api = {
    update(frame, { time, playing } = {}) {
      if (!context || !alive) return;
      const now = context.currentTime;
      if (time !== lastTime) lastMotion = now;
      running = !!playing && !!frame && now - lastMotion < 0.2;
      const audible = enabled && running && !blocked && !root.hidden;
      const state = soundState(kind, frame);
      target(master.gain, audible ? clamp(volume.value) : 0, 0.025);
      const levels = [
        state.motor,
        state.motor * 0.8,
        state.jets,
        state.fan,
        state.engine,
        state.engine,
        state.upper,
      ];
      voices.forEach((v, i) => target(v.gain.gain, levels[i] * v.level));
      target(voices[0].source.frequency, state.motorHz);
      target(voices[1].source.frequency, state.motorHz * 3);
      target(
        musicGain.gain,
        music.checked ? 0.85 - 0.3 * Math.max(state.jets, state.engine) : 0,
        0.15,
      );
      const contact = ["CONTACT", "SETTLED"].includes(frame?.phase);
      const forward =
        lastTime !== null && time > lastTime && time - lastTime < 0.5;
      if (audible && contact && !lastContact && forward) {
        const hit = context.createOscillator(),
          gain = context.createGain();
        hit.frequency.setValueAtTime(90, now);
        hit.frequency.exponentialRampToValueAtTime(40, now + 0.2);
        gain.gain.setValueAtTime(0.08, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.25);
        hit.connect(gain).connect(master);
        hit.start();
        hit.stop(now + 0.3);
        hit.onended = () => {
          hit.disconnect();
          gain.disconnect();
        };
      }
      lastContact = contact;
      lastTime = time;
      // Sources run only during moving playback; no repeated restarts each frame.
      if (
        (audible && music.checked && !musicSource) ||
        ((!audible || !music.checked) && musicSource)
      )
        updateMusic();
    },
    dispose() {
      if (!alive) return;
      alive = false;
      silence();
      stopMusic();
      for (const { source } of voices) source.stop();
      voices = [];
      context?.close();
      win.removeEventListener("message", message);
      root.removeEventListener("visibilitychange", visibility);
      root.removeEventListener("pointerdown", resume);
      root.removeEventListener("click", resume);
      root.removeEventListener("keydown", resume);
    },
  };
  win.addEventListener("pagehide", () => api.dispose(), { once: true });
  return api;
}
