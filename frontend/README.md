# armsim — Frontend Dashboard

A zero‑dependency, static HTML/JS/CSS real‑time viewer for the armsim
robotic arm simulator. No build step, no bundler, no framework — just
drop the folder and serve it.

## How It Works

1. The browser opens `index.html`
2. `websocket_client.js` connects to `ws://<host>/ws`
3. The server sends a one‑time `config` message (joint names, lengths, total reach)
4. The server streams `state` messages at the configured tick rate (default 30 Hz)
5. `state.js` updates the internal data model; `renderer.js` draws it on two `<canvas>` views
6. On disconnect, the client auto‑reconnects with exponential backoff

## File Map

```
frontend/
├── index.html                  # Entry point
├── css/
│   ├── layout.css              # Grid layout, responsive breakpoint at 700px
│   └── style.css               # Retro terminal theme (black/gold/white)
├── js/
│   ├── scale.js                # World→pixel coordinate mapping, auto‑scales to totalReach
│   ├── state.js                # Client‑side data model (joints, EE, view arrays)
│   ├── connection_indicator.js # Header dot + status text
│   ├── renderer.js             # Canvas drawing (grid, arm segments, joint markers)
│   ├── websocket_client.js     # WebSocket connect/reconnect, JSON parsing
│   └── main.js                 # Bootstrap — wire dismiss button, call WSClient.connect()
└── assets/                     # (reserved for icons, favicons, etc.)
```

## How to Serve

### Built‑in (with the Python backend)

The armsim backend serves the frontend automatically. Just start the arm
and open a browser:

```python
import armsim
arm = armsim.Arm([...])
arm.start(mode="lan", port=8080)
```

Then visit `http://localhost:8080` (local) or `http://<machine-ip>:8080` (LAN).

### Standalone (any static file server)

You can serve the frontend independently — it only needs a WebSocket
endpoint at `/ws`:

```bash
# Python
python -m http.server 3000 --directory robotics_sim/frontend/

# Node (with serve)
npx serve robotics_sim/frontend/

# Nginx, Apache, etc.
```

> The WebSocket URL is constructed from `location.host`, so the HTML page
> and WebSocket server must share the same host.

## Customising

### Theme

Edit `css/style.css` — the colour scheme is defined with CSS variables:

```css
/* Key variables in style.css */
--gold: #FFD700;
--bg: #000000;
--text: #FFFFFF;
--grid: #7A6A00;
```

### Layout

Edit `css/layout.css`. The two view panels sit in a CSS Grid. Below 700px
viewport they stack vertically.

### Canvas Rendering

`js/renderer.js` handles all drawing:
- `_drawGrid()` — grid lines with world‑coordinate labels
- Arm segments drawn as gold (`#FFD700`) polylines
- Joint markers as white circles, end‑effector as gold‑filled circle

### Scaling

`js/scale.js` automatically computes a pixels‑per‑unit ratio from the
`totalReach` value sent in the config message. The canvas fits `totalReach × 2.2`
in the smaller dimension, giving ~10% padding around the workspace.

### Adding UI Controls

The frontend is **read‑only** — all arm control happens through the Python
API. If you want to add sliders/buttons, export a function from `main.js`
and call `arm.set_angles()` via a REST endpoint or WebSocket command
(message type not yet implemented in the protocol).

## WebSocket Protocol

The frontend expects two JSON message types on `/ws`:

### `config` (once, immediately after connect)

```json
{
  "type": "config",
  "protocol_version": 1,
  "arm_name": "six_axis_industrial",
  "tick_rate_hz": 30,
  "joint_count": 6,
  "total_reach": 68.0,
  "joints": [
    {"name": "base", "link_length": 0, "link_offset": 10,
     "angle_min": -180, "angle_max": 180, "axis": "z"}
  ]
}
```

Frontend uses this to:
- Set the header title
- Build joint readout rows
- Initialise canvas scaling via `totalReach`

### `state` (every tick)

```json
{
  "type": "state",
  "protocol_version": 1,
  "tick": 4821,
  "timestamp": 1730822400.123,
  "joints": [
    {"name": "base", "angle": 12.4, "target_angle": 90.0, "is_moving": true}
  ],
  "end_effector": {"x": 33.2, "y": 12.0, "z": 15.4},
  "front_view": [{"x": 0.0, "z": 5.0}, ...],
  "top_view": [{"x": 0.0, "y": 0.0}, ...]
}
```

Frontend uses this to:
- Update the per‑joint angle, target, and idle/moving tags
- Update the end‑effector readout
- Redraw both canvas views

### `error` (on fault)

```json
{
  "type": "error",
  "protocol_version": 1,
  "code": "JOINT_LIMIT",
  "message": "shoulder: 200.0 exceeds max 90.0",
  "recoverable": true
}
```

Frontend displays this in a dismissible red banner at the top.

## Browser Support

Any modern browser with WebSocket and `<canvas>` support — Chrome, Firefox,
Safari, Edge. No polyfills needed.
