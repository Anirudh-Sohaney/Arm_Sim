/* scale.js — World-units-to-pixels with proper workspace fitting.

The arm can reach ±totalReach in X/Y from the base.  We size the
view so totalReach * 2.2 fits in the canvas, giving 10% padding on
each side, and centre the base in both views (lower-third for front
view since the arm extends upward from the base).
*/
const Scale = {
  pixelsPerUnit: 20,
  gridSpacing: 10,

  /* Per-view offsets (pixels) — base joint maps to these canvas coords. */
  frontOffsetX: 0,  frontOffsetY: 0,
  topOffsetX:   0,  topOffsetY:   0,

  /**
   * Initialize from total_reach (sum of link lengths) and the front
   * canvas dimensions (both canvases are assumed equal size).
   */
  init(totalReach, canvasWidth, canvasHeight) {
    const reach = Math.max(totalReach, 1.0);

    /* World span: arm can swing ±reach, plus 10% padding each side. */
    const worldSpan = reach * 2.2;

    /* Fit worldSpan into the smaller canvas dimension. */
    const minDim = Math.min(canvasWidth, canvasHeight);
    this.pixelsPerUnit = minDim / worldSpan;

    /* Pick nice round grid spacing. */
    const approx = reach / 3;
    const mag = Math.pow(10, Math.floor(Math.log10(approx)));
    const r = approx / mag;
    if      (r < 1.5) this.gridSpacing = mag;
    else if (r < 3.5) this.gridSpacing = 2 * mag;
    else if (r < 7.5) this.gridSpacing = 5 * mag;
    else              this.gridSpacing = 10 * mag;

    /* Front view (X-Z): base centred horizontally, lower third vertically. */
    this.frontOffsetX = canvasWidth  / 2;
    this.frontOffsetY = canvasHeight * 0.65;

    /* Top view (X-Y): base centred both axes. */
    this.topOffsetX = canvasWidth  / 2;
    this.topOffsetY = canvasHeight / 2;
  },

  /** Convert world coords to pixel coords for a specific view. */
  toPixel(wx, worldVertical, isTopView) {
    const ox = isTopView ? this.topOffsetX : this.frontOffsetX;
    const oy = isTopView ? this.topOffsetY : this.frontOffsetY;
    return {
      px: ox + wx * this.pixelsPerUnit,
      py: oy - worldVertical * this.pixelsPerUnit
    };
  }
};
