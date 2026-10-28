// Opt-in 3.2a display. The Python controller owns the timer and motion gate.
(() => {
  const params=new URLSearchParams(location.search);
  if (params.get('controller') !== '3.2a') return;
  const nativeMode=params.get('input')==='native';
  const keyboardMode=params.get('keyboard')==='1' || nativeMode;
  const demo = window.awareDemo = window.awareDemo || {enabled:!keyboardMode,phase:'waiting',lastState:-Infinity};
  const style = document.createElement('style');
  style.textContent = `
    #myo-prepare { position:fixed; inset:0; z-index:30; display:grid; place-items:center;
      pointer-events:none; font-family:system-ui,sans-serif; }
    #myo-prepare .card { text-align:center; color:#f4f7fb; width:min(420px,85vw); padding:30px;
      background:rgba(11,20,31,.92); border:1px solid #66849c; border-radius:24px;
      box-shadow:0 18px 70px #0008; }
    #myo-prepare .tag { font-size:12px; letter-spacing:.16em; color:#b6cadb; }
    #myo-prepare .number { font-size:48px; line-height:1.1; font-weight:800; margin:16px 0; }
    #myo-prepare .hint { font-size:17px; line-height:1.5; }
    #myo-prepare[data-phase="running"] .number { color:#70e6af; }
    #myo-demo-go { position:fixed; inset:0; z-index:31; pointer-events:none;
      display:flex; align-items:center; justify-content:center;
      color:#70e6af; font:800 48px/1.1 system-ui,sans-serif;
      background:none; border:none; padding:0; }
  `;
  document.head.appendChild(style);
  const root = document.createElement('div');
  root.id = 'myo-prepare';
  root.innerHTML = '<div class="card" role="status" aria-live="polite"><div class="tag">KEYBOARD DEMO · EXPERIMENTAL</div><div class="number"></div><div class="hint"></div></div>';
  document.body.appendChild(root);
  const goBadge = document.createElement('div');
  goBadge.id='myo-demo-go';
  goBadge.textContent='GO';
  goBadge.setAttribute('role','status');
  goBadge.style.display='none';
  document.body.appendChild(goBadge);
  root.querySelector('.tag').textContent=nativeMode ? 'KEYBOARD DEMO · INTERFACE PREVIEW' : keyboardMode ? 'KEYBOARD DEMO · EXPERIMENTAL' : 'MYO · 3.2a EXPERIMENTAL';
  const number = root.querySelector('.number');
  const hint = root.querySelector('.hint');
  let lastMessage = -Infinity, lastKey = '', goAt = null, socket;
  function respectResults() {
    // The simulator's result banner owns the end-of-course presentation.
    // Keep its score / DNF details unobstructed, even if controller status is late.
    const banner=document.getElementById('finish-banner');
    const resultsVisible=!!banner && banner.style.display !== 'none' && banner.style.display !== '';
    root.style.visibility=resultsVisible ? 'hidden' : 'visible';
    goBadge.style.visibility=resultsVisible ? 'hidden' : 'visible';
  }

  function show(phase, label, message) {
    respectResults();
    const key = `${phase}:${label}:${message}`;
    if (key === lastKey) return;
    lastKey = key;
    root.style.display = 'grid';
    root.dataset.phase = phase;
    number.textContent = label;
    hint.textContent = message;
  }
  function disconnected() {
    goBadge.style.display='none';
    goAt = null;demo.phase='offline';
    show('offline', '-', 'Countdown connection unavailable. Check the controller; use E-stop if needed.');
  }
  function connect() {
    const receive = ({data}) => {
      let msg;
      try { msg = JSON.parse(data); } catch { return; }
      if (msg.type !== 'preparation' || msg.version !== '3.2a-experimental' ||
          !['waiting','countdown','stopped','running'].includes(msg.phase)) return;
      if (msg.phase === 'countdown' && ![1,2,3].includes(msg.remaining)) return;
      respectResults();
      lastMessage = performance.now();
      demo.phase=msg.phase;demo.lastState=lastMessage;
      if (msg.phase === 'running') {
        // The same controller state permits motion and removes the central card.
        root.style.display='none';
        lastKey='';
        if (goAt === null) { goAt = lastMessage; goBadge.style.display='flex'; }
      } else {
        goBadge.style.display='none';
        goAt = null;
        let label=msg.phase === 'countdown' ? String(msg.remaining) : 'Ready';
        let message=keyboardMode ? 'Release WASD to begin the 3-second countdown.' : 'Hold your arm level and relax to begin.';
        if(msg.phase === 'countdown') message=keyboardMode ? 'Get ready - release WASD.' : 'Get ready - hold your arm level and relax.';
        if(msg.phase === 'stopped') {
          const reason=String(msg.reason || '');
          if(/estop/i.test(reason)) {
            label='Emergency Stop Active';message='Select Clear E-stop in Emergency Controls. Demo timer paused.';
          } else if(/finished|terminal/i.test(reason)) {
            label='Course Ended';message='Select Restart Course to try again.';
          } else if(/keyboard.*paused/i.test(reason)) {
            label='Driving Stopped';message='Select Start / Resume. Demo timer paused.';
          } else if(/stale|missing|connection|stream|old\/future/i.test(reason)) {
            label='Connection / Input Lost';message='Waiting for valid input and simulator state. '+reason;
          } else {
            label='Controller Stopped';message=reason || 'Check the controller before resuming.';
          }
        }
        show(msg.phase,label,message);
      }
    };
    if(nativeMode) {
      window.addEventListener('aware-native-state',event=>receive({data:JSON.stringify(event.detail)}));
      return;
    }
    socket = new WebSocket('ws://127.0.0.1:8766');
    socket.onmessage = receive;
    socket.onclose = () => { disconnected(); setTimeout(connect, 1000); };
    socket.onerror = () => socket.close();
  }
  disconnected();
  connect();
  setInterval(() => {
    respectResults();
    const now = performance.now();
    if (now - lastMessage > 750) disconnected();
    else if (goAt !== null && now - goAt > 450) goBadge.style.display = 'none';
  }, 30);
})();
