# Bundled Jackal Simulator Site

This is the tested static site served by `jackal-bridge --serve-site`. It includes the steering-wheel overlay in `wheel-overlay.js` and loads it from `index.html`, so the wheel appears over the Jackal chassis in supported camera views.

From the repository root, start the bridge with:

```bash
/path/to/jackal-runtime/bin/jackal-bridge --serve-site --site-dir ./jackal-site
```

Open `http://localhost:8765/Hackathon/`. The overlay is hidden in FPV mode. If rebuilding the web app, copy the rebuilt static output here and keep `wheel-overlay.js` plus its script reference in `index.html`.
