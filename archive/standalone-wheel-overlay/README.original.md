# Simulator steering wheel overlay

`wheel-overlay.js` draws the steering-wheel indicator in the browser on the Jackal's grey chassis. It reads the simulator's `window.__sim`, `window.__view`, and `window.__cams` hooks; it does not open a separate Pygame window.

## Add it to the simulator site

1. Copy `wheel-overlay.js` into the site's public root so it is served at `/Hackathon/wheel-overlay.js`.
2. Load it from the site's `index.html`:

   ```html
   <script defer src="/Hackathon/wheel-overlay.js"></script>
   ```

   For the Vite source site, put the file in `public/wheel-overlay.js` and use:

   ```html
   <script defer src="%BASE_URL%wheel-overlay.js"></script>
   ```

3. Reload the simulator page. The indicator is a screen-space overlay projected onto the chassis, so it follows the car and camera.

## Status

- Chase view: verified working.
- Top view: heading alignment is still under test; do not treat it as resolved yet.
- The 2.5 controller uses the browser overlay and does not import `wheel_display.py`.
