# Steering feedback component

The active implementation is `jackal-site/wheel-overlay.js`, loaded by `jackal-site/index.html`. It is JavaScript with HTML/CSS/SVG, not Java. No separate Pygame process is required for the final controller.

The overlay reads simulator hooks (`window.__sim`, `window.__view`, `window.__cams`), projects the robot's world position through the camera to screen coordinates, and draws a wheel indicator above the chassis. Its rotation reflects the simulator's applied angular command (`sim.applied.w`); it is not a measured physical wheel angle. `requestAnimationFrame` updates position and appearance as the robot/camera moves.

The owner confirmed both chase and top views were debugged and working. Top view includes heading alignment; FPV intentionally hides the overlay. This confirmation concerns the project's prior testing, not a new browser test during repository cleanup.

`jackal-site/` is the whole prebuilt simulator website: JavaScript bundle, MuJoCo WASM, robot assets, courses and the overlay. Only the overlay/customizations should be identified as team additions without further provenance checks.

The historical duplicate at `archive/standalone-wheel-overlay/` is preserved unchanged. Edit the active site file for future overlay work. The older independent Python/Pygame display is retained at `controllers/archive/day2-morning/wheel_display.py` and is not the browser overlay.
