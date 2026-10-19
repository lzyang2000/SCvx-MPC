/** A separate worker per run makes Stop interrupt even a synchronous WASM solve. */
export async function* computeEvents(
  demo,
  config,
  { live = true, speed = 1, signal } = {},
) {
  const worker = new Worker(new URL("./worker.js", import.meta.url), {
      type: "module",
    }),
    queue = [];
  let wake = null,
    ended = false;
  const push = (value) => {
    queue.push(value);
    wake?.();
    wake = null;
  };
  const abort = () => {
    ended = true;
    worker.terminate();
    push({ type: "aborted" });
  };
  worker.onmessage = ({ data }) => push(data);
  worker.onerror = (event) =>
    push({
      type: "error",
      data: { error: event.message || "Browser worker failed" },
    });
  if (signal?.aborted) {
    worker.terminate();
    throw new DOMException("Run stopped", "AbortError");
  }
  signal?.addEventListener("abort", abort, { once: true });
  worker.postMessage({ demo, config, live, speed });
  try {
    while (!ended) {
      if (!queue.length) await new Promise((resolve) => (wake = resolve));
      const event = queue.shift();
      if (event.type === "aborted")
        throw new DOMException("Run stopped", "AbortError");
      if (event.type === "error") throw Error(event.data.error);
      yield event;
      if (event.type === "complete") ended = true;
    }
    if (signal?.aborted) throw new DOMException("Run stopped", "AbortError");
  } finally {
    signal?.removeEventListener("abort", abort);
    worker.terminate();
  }
}
