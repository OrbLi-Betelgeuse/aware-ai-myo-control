// Optional test input only. Never sends velocity commands or alters the simulator.
(() => {
  const params = new URLSearchParams(location.search);
  if (params.get('controller') !== '3.2a' || params.get('keyboard') !== '1' || params.get('input') === 'native') return;
  const demo = window.awareDemo = window.awareDemo || {enabled:false,phase:'waiting',lastState:-Infinity};
  const panel = document.createElement('div');
  Object.assign(panel.style, {position:'fixed',bottom:'16px',left:'16px',zIndex:'40',
    background:'#101d2ef2',color:'#f5f8ff',padding:'16px 20px',borderRadius:'14px',
    border:'1px solid #64859e',font:'14px/1.6 system-ui',maxWidth:'430px'});
  panel.innerHTML = '<strong>Keyboard Demo · No Myo Required</strong><br>Enter: start / resume · Space: stop driving<br>W: fist / faster · A / D: turn · S: reverse<br><small>Release keys: slow forward · Space: stop</small><br><button type="button" data-start>Start / Resume</button> <button type="button" data-pause>Stop Driving</button><br><span role="status" aria-live="polite"></span>';
  document.body.appendChild(panel);
  const status = panel.querySelector('span');
  const keys = new Set(), allowed = new Set(['KeyW','KeyA','KeyS','KeyD']);
  let enabled = false, socket, seq = 0, lastAck = -Infinity;
  let inputReason = 'initial', cameraEvent = null;
  function send() {
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({type:'keys',seq:seq++,enabled,keys:[...keys],reason:inputReason,camera:cameraEvent}));
    }
  }
  function pause(message='Driving stopped. Demo timer paused. Select Start / Resume.') {
    enabled=false;demo.enabled=false;keys.clear();inputReason=message;status.textContent=message;send();
  }
  function start() {
    if (socket?.readyState !== WebSocket.OPEN) {pause('Not connected - waiting to reconnect');return;}
    keys.clear();enabled=true;demo.enabled=true;inputReason='start';status.textContent='Start requested - release WASD during countdown';send();
  }
  panel.querySelector('[data-start]').addEventListener('click', event=>{event.currentTarget.blur();start();});
  panel.querySelector('[data-pause]').addEventListener('click', ()=>pause());
  function editable(target) {
    return target?.isContentEditable || ['INPUT','TEXTAREA','SELECT','BUTTON'].includes(target?.tagName);
  }
  document.addEventListener('click', event => {
    const button=event.target.closest?.('button[data-cam]');
    if (!button) return;
    cameraEvent={mode:button.dataset.cam,at:performance.now()};
    // Restore driving key handling after a camera button takes focus.
    button.blur();
    send();
  });
  window.addEventListener('keydown', event => {
    // Space is always available as test pause, even when a UI control has focus.
    if (event.code === 'Space') {
      event.preventDefault();event.stopImmediatePropagation();pause();return;
    }
    if (editable(event.target)) return;
    if (!allowed.has(event.code) && event.code !== 'Enter') return;
    event.preventDefault();event.stopImmediatePropagation();
    if (event.code === 'Enter') {
      if (event.repeat) return;
      start();
    } else if (enabled) keys.add(event.code);
    send();
  }, true);
  window.addEventListener('keyup', event => {
    if (!allowed.has(event.code)) return;
    event.preventDefault();event.stopImmediatePropagation();keys.delete(event.code);send();
  }, true);
  window.addEventListener('blur',()=>pause('Paused: page lost focus - press Enter to resume'));
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pause('Paused: tab hidden - press Enter to resume');});
  window.addEventListener('pagehide',()=>pause());
  function connect() {
    socket=new WebSocket('ws://127.0.0.1:8767');
    socket.onopen=()=>{seq=0;lastAck=performance.now();pause('Connected - click simulator, then Enter');};
    socket.onmessage=({data})=>{
      try {
        const reply=JSON.parse(data);
        if(reply.type==='keyboard_status') {
          lastAck=performance.now();
          if(reply.enabled && enabled) status.textContent=reply.fault === 'Waiting for fresh Myo windows' ? 'Preparing input - release WASD.' : reply.fault ? 'Stopped: '+reply.fault : 'Control accepted - W/A/S/D to drive, Space to pause';
          if(reply.busy && enabled) pause('Another page is driving - pause it, then Enter here');
        }
      } catch {}
    };
    socket.onclose=event=>{pause('Keyboard disconnected ('+event.code+') - reconnecting');setTimeout(connect,1000);};
    socket.onerror=()=>socket.close();
  }
  connect();
  setInterval(()=>{
    if(document.hidden || !document.hasFocus()) {if(enabled)pause('Paused: page unfocused - press Enter');}
    if(enabled && performance.now()-lastAck>500)pause('Paused: keyboard connection stalled - press Enter');
    send();
  },50);
})();
