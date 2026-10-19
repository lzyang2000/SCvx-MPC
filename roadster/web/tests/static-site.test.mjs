import test from "node:test";
import assert from "node:assert/strict";
import { readFile, stat, readdir } from "node:fs/promises";
import path from "node:path";
const root = new URL("../", import.meta.url),
  base = new URL("https://example.github.io/SCvx-MPC/");
async function local(spec, from) {
  if (
    !spec ||
    spec.startsWith("data:") ||
    spec.startsWith("http") ||
    spec.startsWith("mailto:")
  )
    return;
  assert.ok(!spec.startsWith("/"), `Root-relative asset: ${spec}`);
  const target = new URL(spec, from);
  assert.ok(
    target.pathname.startsWith(base.pathname),
    `Escaped repository subpath: ${target}`,
  );
  const relative = target.pathname.slice(base.pathname.length),
    file = new URL(
      (relative.startsWith("assets/") || relative === "favicon.svg"
        ? "public/"
        : "") + (relative || "index.html"),
      root,
    );
  if (relative.startsWith("vendor/")) return;
  await stat(file);
  if (target.hash && relative.endsWith(".html")) {
    const html = await readFile(file, "utf8");
    assert.ok(
      html.includes(`id="${target.hash.slice(1)}"`),
      `Missing anchor ${target}`,
    );
  }
}
test("site links and module paths resolve under a GitHub Pages repository prefix", async () => {
  for (const name of [
    "index.html",
    "car.html",
    "rocket.html",
    "formulation.html",
  ]) {
    const html = await readFile(new URL(name, root), "utf8");
    for (const match of html.matchAll(/(?:href|src|data-src)="([^"]+)"/g))
      await local(match[1], new URL(name, base));
    const map = html.match(/<script type="importmap">([\s\S]*?)<\/script>/);
    if (map)
      for (const v of Object.values(JSON.parse(map[1]).imports)) {
        if (v.endsWith("/")) continue;
        await stat(new URL("node_modules/three/build/three.module.js", root));
        assert.ok(v.startsWith("./vendor/"));
      }
  }
  for (const dir of ["src/", "src/compute/"])
    for (const name of await readdir(new URL(dir, root))) {
      if (!name.endsWith(".js")) continue;
      const js = await readFile(new URL(dir + name, root), "utf8");
      for (const match of js.matchAll(
        /(?:from\s*|import\s*\()(["'])([^"']+)\1/g,
      )) {
        if (match[2].startsWith(".") && !match[2].includes("/vendor/mujoco/"))
          await local(match[2], new URL(dir + name, base));
      }
    }
});
test("WASM files and legal notices are present for a static deployment", async () => {
  for (const name of [
    "src/compute/vendor/clarabel/clarabel_wasm_bg.wasm",
    "node_modules/@mujoco/mujoco/mujoco.wasm",
  ]) {
    const bytes = await readFile(new URL(name, root));
    assert.deepEqual([...bytes.slice(0, 4)], [0, 97, 115, 109]);
  }
  for (const name of [
    "public/assets/MUJOCO-LICENSE",
    "src/compute/vendor/clarabel/LICENSE",
    "public/.nojekyll",
  ])
    await stat(new URL(name, root));
});
