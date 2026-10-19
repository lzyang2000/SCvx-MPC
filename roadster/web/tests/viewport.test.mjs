import test from 'node:test';
import assert from 'node:assert/strict';
import { createViewportResize } from '../src/viewport.js';

test('repeated layout notifications do not clear a live WebGL canvas', () => {
  const calls = [];
  const camera = { aspect: 0, updateProjectionMatrix() { calls.push('projection'); } };
  const resize = createViewportResize({ setSize(...args) { calls.push(args); } }, camera, () => calls.push('paint'));
  assert.equal(resize({ width: 960, height: 540 }), true);
  for (let i = 0; i < 300; i++) assert.equal(resize({ width: 960.1, height: 539.9 }), false);
  assert.deepEqual(calls, [[960, 540, false], 'projection', 'paint']);
  assert.equal(camera.aspect, 16 / 9);
});

test('a real resize repaints immediately and a hidden tab leaves its buffer intact', () => {
  const calls = [];
  const camera = { updateProjectionMatrix() { calls.push('projection'); } };
  const resize = createViewportResize({ setSize(...args) { calls.push(args); } }, camera, () => calls.push('paint'));
  resize({ width: 960, height: 540 });
  assert.equal(resize({ width: 0, height: 0 }), false);
  assert.equal(resize({ width: 960, height: 540 }), false);
  assert.equal(resize({ width: 480, height: 600 }), true);
  assert.deepEqual(calls.slice(3), [[480, 600, false], 'projection', 'paint']);
  assert.equal(camera.aspect, .8);
});
