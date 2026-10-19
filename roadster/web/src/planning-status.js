/** Keep the scene visible while the first trajectory is being planned. */
export function createPlanningStatus(viewport) {
  const status = document.createElement("div");
  status.className = "planning-status";
  status.hidden = true;
  status.setAttribute("role", "status");
  status.setAttribute("aria-live", "polite");
  status.setAttribute("aria-atomic", "true");
  const spinner = document.createElement("span");
  spinner.className = "planning-spinner";
  spinner.setAttribute("aria-hidden", "true");
  const text = document.createElement("span");
  text.textContent = "Planning trajectory…";
  status.append(spinner, text);
  viewport.appendChild(status);
  return {
    update(event) {
      if (event.type === "status" && event.data.planning)
        status.hidden = false;
      else if (["plan", "complete", "error"].includes(event.type))
        status.hidden = true;
    },
    clear() {
      status.hidden = true;
    },
  };
}
