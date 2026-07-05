/* websocket_client.js — WebSocket connection, message parsing, reconnection. */
const WSClient = {
  ws: null,
  reconnectAttempts: 0,
  maxBackoffMs: 10000,
  baseBackoffMs: 500,

  /** Connect (or reconnect) to the server's WebSocket endpoint. */
  connect() {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    const url = protocol + "//" + location.host + "/ws";

    ConnectionIndicator.set("connecting");
    this.ws = new WebSocket(url);

    this.ws.onopen = () => {
      this.reconnectAttempts = 0;
      ConnectionIndicator.set("connected");
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === "config") {
          AppState.applyConfig(msg);
          AppState.initCanvases();
        } else if (msg.type === "state") {
          AppState.applyState(msg);
          Renderer.scheduleDraw();
        } else if (msg.type === "error") {
          ConnectionIndicator.showError(msg);
        }
      } catch (e) {
        /* Ignore malformed messages. */
      }
    };

    this.ws.onerror = () => {
      /* onclose will fire next; handled there. */
    };

    this.ws.onclose = () => {
      ConnectionIndicator.set("disconnected");
      this.scheduleReconnect();
    };
  },

  /** Exponential-backoff reconnection. */
  scheduleReconnect() {
    this.reconnectAttempts++;
    const delay = Math.min(
      this.baseBackoffMs * Math.pow(2, this.reconnectAttempts - 1),
      this.maxBackoffMs
    );
    setTimeout(() => this.connect(), delay);
  },

  /** Show an error banner (dismissible). */
  showError(msg) {
    const banner = document.getElementById("error-banner");
    const text = document.getElementById("error-text");
    banner.classList.remove("hidden");
    text.textContent = msg.message || "Unknown error";

    document.getElementById("error-dismiss").onclick = () => {
      banner.classList.add("hidden");
    };
  }
};
