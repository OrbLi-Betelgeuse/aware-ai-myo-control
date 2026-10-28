# Bundled Jackal simulator website

This directory contains the prebuilt simulator site, including its JavaScript bundle, MuJoCo WASM, robot assets, courses and the active `wheel-overlay.js` customization. The simulator comes from the instructor's Hackathon environment; the entire site is not a team-authored wheel component.

From the repository root:

```bash
/path/to/jackal-runtime/bin/jackal-bridge --serve-site --site-dir ./jackal-site
```

Open `http://localhost:8765/Hackathon/`. The page loads the wheel overlay automatically. Chase and top views were confirmed working by the project owner; FPV hides the overlay. See [component details](../docs/STEERING_OVERLAY.md) and [full setup](../docs/RUNNING.md).

If rebuilding upstream, preserve the overlay and its script reference. This folder is build output, not the upstream TypeScript source tree.

The optional experimental preparation overlay is enabled only by `?controller=3.2a`. It displays the 3.2a controller's status from localhost port 8766; it does not implement the control gate. See [3.2a details](../docs/SAFETY_3_2A.md). The normal competition URL does not activate it.

## Native keyboard interface preview

Use `?controller=3.2a&input=native` for built-in keyboard controls with the shared experimental interface and active steering overlay. This browser-only adapter disconnects external control, gates native input during preparation and pauses practice scoring during explicit stops. It does not run Python/Myo control or its LiDAR limits. Use `?controller=3.2a&keyboard=1` only for the separate synthetic-Myo testing tool.

At course completion or DNF, the simulator result banner takes priority. Shared preparation/stop overlays and GO remain hidden while results are displayed, then become available again after Restart Course.
