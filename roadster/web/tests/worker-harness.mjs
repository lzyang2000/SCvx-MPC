// Node adapter exercises the real module worker and its WASM assets without a DOM.
import { parentPort } from "node:worker_threads";
import { readFile } from "node:fs/promises";
const nativeFetch = globalThis.fetch;
globalThis.fetch = async (url, options) => {
  const u = new URL(url);
  if (u.protocol === "file:")
    return new Response(await readFile(u), {
      headers: {
        "Content-Type": u.pathname.endsWith(".wasm")
          ? "application/wasm"
          : "application/xml",
      },
    });
  return nativeFetch(url, options);
};
globalThis.self = globalThis;
self.postMessage = (data) => parentPort.postMessage(data);
self.close = () => parentPort.close();
await import("../dist/src/compute/worker.js");
parentPort.on("message", (data) => self.onmessage({ data }));
