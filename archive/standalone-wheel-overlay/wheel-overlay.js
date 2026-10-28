// A small steering-wheel indicator projected onto the Jackal chassis in the browser view.
// It reads the simulator's applied angular command, so the indicator follows what the car does.
(() => {
  const MAX_W = 2.0; // protocol W_MAX, rad/s
  const MAX_ANGLE = 78;
  const CHASSIS_Z = 0.19; // metres above the ground; puts the overlay on the grey top panel
  const wheel = document.createElement('div');
  wheel.id = 'jackal-wheel-overlay';
  wheel.setAttribute('aria-label', 'Steering direction');
  Object.assign(wheel.style, {
    position: 'fixed', width: '54px', height: '54px', zIndex: '6',
    display: 'none', placeItems: 'center', borderRadius: '50%',
    background: 'rgba(20, 25, 34, 0.32)', border: '1px solid rgba(240, 244, 250, 0.76)',
    boxShadow: '0 2px 8px rgba(0,0,0,.48), inset 0 0 8px rgba(255,255,255,.10)',
    backdropFilter: 'blur(1px)', pointerEvents: 'none', opacity: '0.9',
    transform: 'translate(-50%, -50%)', transition: 'left 50ms linear, top 50ms linear',
  });
  wheel.innerHTML = '<svg viewBox="0 0 100 100" width="48" height="48" aria-hidden="true">' +
    '<circle class="rim" cx="50" cy="50" r="39" fill="none" stroke="#e7edf5" stroke-width="8"/>' +
    '<path d="M50 12 L50 48 M18 35 L44 51 M82 35 L56 51" fill="none" stroke="#dce3ed" stroke-width="7" stroke-linecap="round"/>' +
    '<path d="M50 12 L50 23" fill="none" stroke="#60dc8b" stroke-width="8" stroke-linecap="round"/>' +
    '<circle cx="50" cy="51" r="9" fill="#cbd3df" stroke="#15191f" stroke-width="3"/>' +
    '</svg>';
  document.body.appendChild(wheel);
  const svg = wheel.querySelector('svg');
  const rim = wheel.querySelector('.rim');
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

  function project(camera, x, y, z) {
    camera.updateMatrixWorld(true);
    const v = camera.matrixWorldInverse.elements;
    const p = camera.projectionMatrix.elements;
    const vx = v[0] * x + v[4] * y + v[8] * z + v[12];
    const vy = v[1] * x + v[5] * y + v[9] * z + v[13];
    const vz = v[2] * x + v[6] * y + v[10] * z + v[14];
    const vw = v[3] * x + v[7] * y + v[11] * z + v[15];
    const cx = p[0] * vx + p[4] * vy + p[8] * vz + p[12] * vw;
    const cy = p[1] * vx + p[5] * vy + p[9] * vz + p[13] * vw;
    const cz = p[2] * vx + p[6] * vy + p[10] * vz + p[14] * vw;
    const cw = p[3] * vx + p[7] * vy + p[11] * vz + p[15] * vw;
    if (cw <= 0) return null;
    return { x: cx / cw, y: cy / cw, z: cz / cw };
  }

  function tick() {
    const sim = window.__sim;
    const view = window.__view;
    const cams = window.__cams;
    const canvas = document.getElementById('view');
    if (sim && view && cams && canvas && cams.mode !== 'fpv') {
      const pose = sim.state().pose;
      const ndc = project(cams.camera, pose.x, pose.y, CHASSIS_Z);
      const rect = canvas.getBoundingClientRect();
      if (ndc && ndc.z >= -1 && ndc.z <= 1 && Math.abs(ndc.x) < 1.15 && Math.abs(ndc.y) < 1.15) {
        wheel.style.display = 'grid';
        wheel.style.left = `${rect.left + (ndc.x + 1) * rect.width / 2}px`;
        wheel.style.top = `${rect.top + (1 - ndc.y) * rect.height / 2}px`;
        const w = Number(sim.applied?.w ?? 0);
        const turn = clamp(w / MAX_W, -1, 1);
        // Chase keeps its existing screen-relative steering indicator. In top view,
        // align the wheel's forward marker with the robot's projected heading first,
        // then apply the same steering offset relative to the chassis.
        const heading = cams.mode === 'top' ? 90 - pose.yaw * 180 / Math.PI : 0;
        svg.style.transform = `rotate(${heading - turn * MAX_ANGLE}deg)`;
        rim.setAttribute('stroke', turn > 0.02 ? '#73a9ff' : turn < -0.02 ? '#e47be5' : '#e7edf5');
      } else {
        wheel.style.display = 'none';
      }
    }
    requestAnimationFrame(tick);
  }
  requestAnimationFrame(tick);
})();
