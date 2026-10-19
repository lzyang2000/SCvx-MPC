import { chinese, translateDynamic } from "./locale-zh.js";

let language = "en";
export const currentLanguage = () => language;
export function translateText(text, locale = language) {
  if (locale !== "zh" || !text?.trim()) return text;
  const normalized = text.trim().replace(/\s+/g, " ");
  const translated = chinese[normalized] ?? translateDynamic(normalized);
  return translated === normalized
    ? text
    : text.match(/^\s*/)[0] + translated + text.match(/\s*$/)[0];
}

export function initializeLanguage({ root = document, win = window } = {}) {
  const remembered = () => {
    try {
      return win.localStorage.getItem("scvx-language");
    } catch {
      return null;
    }
  };
  const requested = new URLSearchParams(win.location.search).get("lang");
  language = (requested || remembered()) === "zh" ? "zh" : "en";
  const originals = new WeakMap();
  const attributes = new WeakMap();
  const skipped = (element) =>
    !element || element.closest("script,style,pre,code,[data-no-translate]");
  const processText = (node) => {
    if (skipped(node.parentElement)) return;
    const old = originals.get(node);
    const source =
      old && old.rendered === node.nodeValue ? old.source : node.nodeValue;
    const rendered = translateText(source);
    if (rendered !== source || old) originals.set(node, { source, rendered });
    if (node.nodeValue !== rendered) node.nodeValue = rendered;
  };
  function processElement(element) {
    if (skipped(element)) return;
    const records = attributes.get(element) ?? {};
    for (const name of ["title", "aria-label", "alt"]) {
      const value = element.getAttribute(name);
      if (!value) continue;
      const old = records[name];
      const source = old && old.rendered === value ? old.source : value;
      const rendered = translateText(source);
      records[name] = { source, rendered };
      if (value !== rendered) element.setAttribute(name, rendered);
    }
    attributes.set(element, records);
  }
  function scan(node) {
    if (node.nodeType === 3) return processText(node);
    if (node.nodeType !== 1 && node.nodeType !== 9) return;
    if (node.nodeType === 1) processElement(node);
    const walker = root.createTreeWalker(node, 5); // Elements and text nodes.
    while (walker.nextNode()) {
      if (walker.currentNode.nodeType === 3) processText(walker.currentNode);
      else processElement(walker.currentNode);
    }
  }
  const observer = new win.MutationObserver((changes) => {
    observer.disconnect();
    for (const change of changes) {
      if (change.type === "characterData") processText(change.target);
      else if (change.type === "attributes") processElement(change.target);
      else for (const node of change.addedNodes) scan(node);
    }
    watch();
  });
  function watch() {
    observer.observe(root.documentElement, {
      subtree: true,
      childList: true,
      characterData: true,
      attributes: true,
      attributeFilter: ["title", "aria-label", "alt"],
    });
  }
  const controls = root.createElement("div");
  controls.className = "language-switch";
  controls.dataset.noTranslate = "";
  controls.setAttribute("role", "group");
  controls.setAttribute("aria-label", "Language / 语言");
  controls.innerHTML =
    '<button type="button" lang="en" data-language="en">EN</button><button type="button" lang="zh-CN" data-language="zh">中文</button>';
  if (win === win.top)
    (
      root.querySelector(".demo-nav") ??
      root.querySelector("header") ??
      root.body
    ).appendChild(controls);
  function setLanguage(next, persist = true) {
    language = next === "zh" ? "zh" : "en";
    if (persist) {
      try {
        win.localStorage.setItem("scvx-language", language);
      } catch {}
    }
    observer.disconnect();
    root.documentElement.lang = language === "zh" ? "zh-CN" : "en";
    for (const button of controls.querySelectorAll("button"))
      button.setAttribute(
        "aria-pressed",
        String(button.dataset.language === language),
      );
    scan(root.documentElement);
    watch();
    for (const frame of root.querySelectorAll("iframe"))
      frame.contentWindow?.postMessage(
        { type: "language", language },
        win.location.origin,
      );
    win.dispatchEvent(
      new win.CustomEvent("languagechange", { detail: language }),
    );
  }
  for (const button of controls.querySelectorAll("button"))
    button.onclick = () => setLanguage(button.dataset.language);
  win.addEventListener("storage", (e) => {
    if (e.key === "scvx-language") setLanguage(e.newValue, false);
  });
  win.addEventListener("message", (e) => {
    if (e.origin === win.location.origin && e.data?.type === "language")
      setLanguage(e.data.language, false);
  });
  for (const frame of root.querySelectorAll("iframe"))
    frame.addEventListener("load", () =>
      frame.contentWindow?.postMessage(
        { type: "language", language },
        win.location.origin,
      ),
    );
  setLanguage(language, !!requested);
  return { setLanguage };
}
