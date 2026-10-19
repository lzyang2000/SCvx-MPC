import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import {
  createGroundTargetPicker,
  landingBounds,
} from "../src/car-target-picker.js";

function setup(maneuver = "c-ramp") {
  const listeners = {},
    selected = [];
  const rect = { left: 80, top: 40, width: 1000, height: 600 };
  const canvas = {
    style: {},
    getBoundingClientRect: () => rect,
    addEventListener: (name, fn) => {
      listeners[name] = fn;
    },
  };
  const camera = new THREE.PerspectiveCamera(
    40,
    rect.width / rect.height,
    0.1,
    300,
  );
  const center = maneuver === "c-ramp" ? -20 : 57;
  camera.position.set(center - 10, 24, 45);
  camera.lookAt(center, 0, 0);
  camera.updateMatrixWorld();
  let active = true;
  const picker = createGroundTargetPicker({
    canvas,
    camera,
    bounds: () => landingBounds(maneuver),
    enabled: () => active,
    onSelect: (x) => selected.push(x),
  });
  function event(x, z = 0, overrides = {}) {
    const p = new THREE.Vector3(x, 0, z).project(camera);
    return {
      pointerId: 1,
      button: 0,
      isPrimary: true,
      clientX: rect.left + ((p.x + 1) * rect.width) / 2,
      clientY: rect.top + ((1 - p.y) * rect.height) / 2,
      ...overrides,
    };
  }
  function click(e) {
    listeners.pointerdown(e);
    listeners.pointerup(e);
  }
  return {
    listeners,
    selected,
    event,
    click,
    canvas,
    picker,
    disable: () => {
      active = false;
    },
  };
}

test("ground raycasting selects runway X in both scenes, independent of viewport offset", () => {
  for (const [maneuver, x] of [
    ["c-ramp", -16.3],
    ["barrel-roll", 56.4],
  ]) {
    const s = setup(maneuver);
    s.click(s.event(x, 2));
    assert.deepEqual(s.selected, [x]);
  }
});
test("camera drags, including dragging away and returning, do not move the landing target", () => {
  const s = setup(),
    e = s.event(-20);
  s.listeners.pointerdown(e);
  s.listeners.pointermove({ ...e, clientX: e.clientX + 30 });
  s.listeners.pointermove(e);
  s.listeners.pointerup(e);
  s.listeners.pointerdown(e);
  s.listeners.pointerup({ ...e, clientX: e.clientX + 30 });
  assert.deepEqual(s.selected, []);
});
test("off-runway, out-of-range, secondary, cancelled and busy clicks cannot select", () => {
  const s = setup();
  s.click(s.event(-10));
  s.click(s.event(-20, 8));
  s.click(s.event(-20, 0, { button: 2 }));
  s.click(s.event(-20, 0, { isPrimary: false }));
  s.listeners.pointerdown(s.event(-20));
  s.listeners.pointercancel();
  s.listeners.pointerup(s.event(-20));
  s.disable();
  s.click(s.event(-20));
  assert.deepEqual(s.selected, []);
});
test("hover shows a crosshair only on selectable ground; cancelling clears it", () => {
  const s = setup();
  s.listeners.pointermove(s.event(-20));
  assert.equal(s.canvas.style.cursor, "crosshair");
  s.listeners.pointermove(s.event(-10));
  assert.equal(s.canvas.style.cursor, "");
  s.listeners.pointermove(s.event(-20));
  s.picker.cancel();
  assert.equal(s.canvas.style.cursor, "");
});
