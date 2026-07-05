/* connection_indicator.js — Connection-status dot + aria-live text. */
const ConnectionIndicator = {
  /** Update the indicator to one of: "connected", "connecting", "disconnected". */
  set(state) {
    const dot = document.getElementById("status-dot");
    const text = document.getElementById("status-text");

    dot.className = "dot " + state;
    text.textContent = state;

    /* aria-live announcement. */
    const ariaRegion = document.getElementById("connection-status");
    ariaRegion.setAttribute("aria-label", "Connection status: " + state);
  }
};
