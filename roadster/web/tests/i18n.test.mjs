import test from "node:test";
import assert from "node:assert/strict";
import { translateText } from "../src/i18n.js";
import { readFile, stat } from "node:fs/promises";

test("Chinese preserves MPC, numeric telemetry, units and solver details", () => {
  assert.equal(translateText("MPC close-up", "zh"), "MPC 近景");
  assert.equal(translateText("#43 / 3 iterations", "zh"), "#43 / 3 次迭代");
  assert.equal(
    translateText(
      "Booster landed · 0.02 m pad error · 26214 kg propellant left",
      "zh",
    ),
    "一级已着陆 · 位置误差 0.02 m · 剩余推进剂 26214 kg",
  );
  assert.equal(translateText("FAN 0.65 g", "zh"), "风扇 0.65 g");
  assert.equal(translateText("  Sound on \n", "zh"), "  声音开 \n");
  for (const value of [
    "12.34",
    "10 × 0.12 s",
    "SCvx MPC",
    "x = [m, r, v, q, ω]",
    "Clarabel",
  ])
    assert.equal(translateText(value, "zh"), value);
  assert.equal(translateText("Sound on", "en"), "Sound on");
});

test("both demos ship sound controls and all four pages load language controls", async () => {
  const root = new URL("../", import.meta.url);
  for (const name of ["car", "rocket"]) {
    const html = await readFile(new URL(`${name}.html`, root), "utf8");
    for (const id of ["sound-toggle", "sound-music", "sound-volume"])
      assert.match(html, new RegExp(`id="${id}"`));
  }
  for (const name of ["index", "car", "rocket", "formulation"])
    assert.match(
      await readFile(new URL(`${name}.html`, root), "utf8"),
      /src\/language.css/,
    );
  const music = await stat(new URL("public/assets/audio/music.mp3", root));
  assert.ok(music.size > 10000 && music.size < 1000000);
});
