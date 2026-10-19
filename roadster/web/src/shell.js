import { initializeLanguage } from "./i18n.js";
initializeLanguage();
const buttons = [...document.querySelectorAll("[data-demo]")];
function select(demo) {
  if (!["car", "rocket"].includes(demo)) demo = "car";
  for (const button of buttons) {
    const active = button.dataset.demo === demo,
      panel = document.getElementById("demo-" + button.dataset.demo);
    panel.contentWindow?.postMessage(
      { type: active ? "resume-demo" : "pause-demo" },
      location.origin,
    );
    panel.hidden = !active;
    button.setAttribute("aria-selected", String(active));
    button.tabIndex = active ? 0 : -1;
    if (active && !panel.src) panel.src = panel.dataset.src;
  }
  history.replaceState(null, "", "#" + demo);
}
for (const button of buttons) {
  button.onclick = () => select(button.dataset.demo);
  button.onkeydown = (e) => {
    if (["ArrowLeft", "ArrowRight", "Home", "End"].includes(e.key)) {
      e.preventDefault();
      const target =
        e.key === "Home"
          ? buttons[0]
          : e.key === "End"
            ? buttons[1]
            : buttons[1 - buttons.indexOf(button)];
      target.focus();
      select(target.dataset.demo);
    }
  };
}
window.addEventListener("hashchange", () => select(location.hash.slice(1)));
select(location.hash.slice(1) || "car");
