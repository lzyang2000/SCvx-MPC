/** Resize only when the CSS viewport changes, then paint the new buffer immediately. */
export function createViewportResize(renderer, camera, paint) {
  let width = 0, height = 0;
  return (rect) => {
    const w = Math.round(rect.width), h = Math.round(rect.height);
    if (w < 1 || h < 1 || (w === width && h === height)) return false;
    width = w;
    height = h;
    // Keep layout in CSS. Reassigning the canvas size clears its drawing buffer.
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    paint();
    return true;
  };
}
