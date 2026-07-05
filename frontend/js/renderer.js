/* renderer.js — Canvas drawing: front + top views, polylines, markers. */
const Renderer = {
  pendingDraw: false,

  scheduleDraw() {
    if (!this.pendingDraw) {
      this.pendingDraw = true;
      requestAnimationFrame(() => {
        this.pendingDraw = false;
        this.draw();
      });
    }
  },

  draw() {
    this._drawView("canvas-front", AppState.frontView, false);
    this._drawView("canvas-top", AppState.topView, true);
  },

  _drawView(canvasId, points, isTopView) {
    const canvas = document.getElementById(canvasId);
    if (!canvas || !points.length) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width, h = canvas.height;
    const ox = isTopView ? Scale.topOffsetX : Scale.frontOffsetX;
    const oy = isTopView ? Scale.topOffsetY : Scale.frontOffsetY;

    /* Clear. */
    ctx.fillStyle = "#000000";
    ctx.fillRect(0, 0, w, h);

    /* Grid. */
    this._drawGrid(ctx, w, h, ox, oy);

    /* Convert world points to pixel coords. */
    const pixels = points.map(p => {
      const vert = isTopView ? p.y : p.z;
      return {
        px: ox + p.x * Scale.pixelsPerUnit,
        py: oy - vert * Scale.pixelsPerUnit
      };
    });

    /* Arm segments (gold). */
    ctx.strokeStyle = "#FFD700";
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.beginPath();
    ctx.moveTo(pixels[0].px, pixels[0].py);
    for (let i = 1; i < pixels.length; i++) {
      ctx.lineTo(pixels[i].px, pixels[i].py);
    }
    ctx.stroke();

    /* Intermediate joint markers (white circles). */
    ctx.fillStyle = "#FFFFFF";
    for (let i = 0; i < pixels.length - 1; i++) {
      ctx.beginPath();
      ctx.arc(pixels[i].px, pixels[i].py, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    /* End-effector marker (gold fill + white outline). */
    if (pixels.length > 0) {
      const last = pixels[pixels.length - 1];
      ctx.strokeStyle = "#FFFFFF";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(last.px, last.py, 6, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = "#FFD700";
      ctx.fill();
    }
  },

  _drawGrid(ctx, w, h, ox, oy) {
    const spacing = Scale.pixelsPerUnit * Scale.gridSpacing;
    if (spacing < 20) return;

    ctx.strokeStyle = "#7A6A00";
    ctx.lineWidth = 0.5;
    ctx.font = "9px monospace";
    ctx.fillStyle = "#7A6A00";

    /* Vertical grid lines. */
    let x = ((ox % spacing) + spacing) % spacing;
    while (x < w) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
      const worldX = Math.round((x - ox) / Scale.pixelsPerUnit);
      if (worldX !== 0) {
        ctx.fillText(worldX.toString(), x + 2, 12);
      }
      x += spacing;
    }

    /* Horizontal grid lines. */
    let y = ((oy % spacing) + spacing) % spacing;
    while (y < h) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
      const worldY = Math.round((oy - y) / Scale.pixelsPerUnit);
      if (worldY !== 0) {
        ctx.fillText(worldY.toString(), 2, y - 2);
      }
      y += spacing;
    }
  }
};
