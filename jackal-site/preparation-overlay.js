// Opt-in 3.2a display. The Python controller owns the timer and motion gate.
(() => {
  if (new URLSearchParams(location.search).get('controller') !== '3.2a') return;
  const style = document.createElement('style');
  style.textContent = `
    #myo-prepare { position:fixed; inset:0; z-index:30; display:grid; place-items:center;
      pointer-events:none; font-family:system-ui,sans-serif; }
    #myo-prepare .card { text-align:center; color:#f4f7fb; width:min(420px,85vw); padding:30px;
      background:rgba(11,20,31,.92); border:1px solid #66849c; border-radius:24px;
      box-shadow:0 18px 70px #0008; }
    #myo-prepare .tag { font-size:12px; letter-spacing:.16em; color:#b6cadb; }
    #myo-prepare .number { font-size:100px; line-height:1.1; font-weight:800; margin:16px 0; }
    #myo-prepare .hint { font-size:17px; line-height:1.5; }
    #myo-prepare[data-phase="running"] .number { color:#70e6af; }
    #myo-prepare .pulse { animation:myo-prepare-pulse .35s ease-out; }
    @keyframes myo-prepare-pulse { from {transform:scale(1.18);opacity:.5} to {transform:scale(1);opacity:1} }
    @media(prefers-reduced-motion:reduce) { #myo-prepare .pulse {animation:none;} }
  `;
  document.head.appendChild(style);
  const root = document.createElement('div');
  root.id = 'myo-prepare';
  root.innerHTML = '<div class="card" role="status" aria-live="polite"><div class="tag">MYO · 3.2a EXPERIMENTAL</div><div class="number"></div><div class="hint"></div></div>';
  document.body.appendChild(root);
  const number = root.querySelector('.number');
  const hint = root.querySelector('.hint');
  let lastMessage = -Infinity, lastKey = '', goAt = null, socket;
  function show(phase, label, message) {
    const key = `${phase}:${label}:${message}`;
    if (key === lastKey) return;
    lastKey = key;
    root.style.display = 'grid';
    root.dataset.phase = phase;
    number.textContent = label;
    hint.textContent = message;
    number.classList.remove('pulse');
    void number.offsetWidth;
    number.classList.add('pulse');
  }
  function disconnected() {
    goAt = null;
    show('offline', '—', 'Countdown connection unavailable. Check the controller; use E-stop if needed.');
  }
  function connect() {
    socket = new WebSocket('ws://127.0.0.1:8766');
    socket.onmessage = ({data}) => {
      let msg;
      try { msg = JSON.parse(data); } catch { return; }
      if (msg.type !== 'preparation' || msg.version !== '3.2a-experimental' ||
          !['waiting','countdown','stopped','running'].includes(msg.phase)) return;
      if (msg.phase === 'countdown' && ![1,2,3].includes(msg.remaining)) return;
      lastMessage = performance.now();
      if (msg.phase === 'running') {
        if (goAt === null) { goAt = lastMessage; show('running', 'GO', 'Ready — relaxed hand moves forward'); }
      } else {
        goAt = null;
        show(msg.phase, msg.phase === 'countdown' ? String(msg.remaining) : msg.phase === 'stopped' ? 'STOP' : 'READY',
             msg.phase === 'countdown' ? 'Hold your arm level and relax' :
             msg.phase === 'stopped' ? 'Controller paused. Resolve the fault, then return to neutral.' :
             'Hold your arm level and relax to start the 3-second countdown');
      }
    };
    socket.onclose = () => { disconnected(); setTimeout(connect, 1000); };
    socket.onerror = () => socket.close();
  }
  disconnected();
  connect();
  setInterval(() => {
    const now = performance.now();
    if (now - lastMessage > 750) disconnected();
    else if (goAt !== null && now - goAt > 700) root.style.display = 'none';
  }, 100);
})();
