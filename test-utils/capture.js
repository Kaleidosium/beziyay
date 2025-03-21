globalThis.window.lines = [];

let isDrawing = false;

globalThis.window.addEventListener("mousedown", ({ pageX: x, pageY: y }) => {
  isDrawing = true;
  globalThis.window.lines.push([{ x, y }]);
});

globalThis.window.addEventListener("mousemove", ({ pageX: x, pageY: y }) => {
  if (!isDrawing) return;
  globalThis.window.lines[globalThis.window.lines.length - 1].push({ x, y });
});

globalThis.window.addEventListener("mouseup", () => {
  isDrawing = false;
});
