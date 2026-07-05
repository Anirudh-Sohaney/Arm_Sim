/* state.js — Client-side mirror of the latest simulation state. */
const AppState = {
  armName: "",
  tickRateHz: 30,
  jointCount: 0,
  totalReach: 0,

  /* Live data from the most recent "state" message. */
  joints: [],
  endEffector: { x: 0, y: 0, z: 0 },
  frontView: [],
  topView: [],

  /** Process the one-time "config" message on connect. */
  applyConfig(msg) {
    this.armName = msg.arm_name || "demo_arm";
    this.tickRateHz = msg.tick_rate_hz || 30;
    this.totalReach = msg.total_reach || 0;
    this.jointCount = msg.joint_count || 0;
    document.getElementById("arm-name").textContent =
      "ROBOTIC ARM SIMULATOR — " + this.armName;

    /* Initialize joint readout rows. */
    const container = document.getElementById("joint-readouts");
    container.innerHTML = "";
    if (msg.joints) {
      msg.joints.forEach(j => {
        const row = document.createElement("div");
        row.className = "joint-row";
        row.id = "joint-row-" + j.name;
        row.innerHTML =
          `<span class="joint-name">${j.name}</span>` +
          `<span class="joint-angle">0.0°</span>` +
          `<span class="joint-target"></span>` +
          `<span class="joint-tag idle">idle</span>`;
        container.appendChild(row);
      });
    }
  },

  /** Process a `state` message, updating all views. */
  applyState(msg) {
    this.joints = msg.joints || [];
    this.endEffector = msg.end_effector || { x: 0, y: 0, z: 0 };
    this.frontView = msg.front_view || [];
    this.topView = msg.top_view || [];

    /* Update readouts. */
    (msg.joints || []).forEach(j => {
      const row = document.getElementById("joint-row-" + j.name);
      if (!row) return;
      const angleEl = row.querySelector(".joint-angle");
      const targetEl = row.querySelector(".joint-target");
      const tagEl = row.querySelector(".joint-tag");
      if (angleEl) angleEl.textContent = j.angle.toFixed(1) + "°";
      if (targetEl) {
        targetEl.textContent = j.is_moving
          ? " → " + j.target_angle.toFixed(1) + "°"
          : "";
      }
      if (tagEl) {
        tagEl.textContent = j.is_moving ? "moving" : "idle";
        tagEl.className = "joint-tag " + (j.is_moving ? "moving" : "idle");
      }
    });

    /* End-effector readout. */
    const ee = this.endEffector;
    document.getElementById("end-effector-readout").textContent =
      `end-effector   x=${ee.x.toFixed(2)}   y=${ee.y.toFixed(2)}   z=${ee.z.toFixed(2)}`;
  },

  /** Initialize canvases from a config message. */
  initCanvases() {
    const frontCanvas = document.getElementById("canvas-front");
    const topCanvas = document.getElementById("canvas-top");

    /* Size canvases to their containers. */
    const resizeAll = () => {
      [frontCanvas, topCanvas].forEach(c => {
        c.width = c.parentElement.clientWidth;
        c.height = c.parentElement.clientHeight;
      });
      Scale.init(this.totalReach || 10, frontCanvas.width, frontCanvas.height);
      Renderer.draw();
    };

    window.addEventListener("resize", resizeAll);
    resizeAll();
  }
};
