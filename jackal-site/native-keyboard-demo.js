// Interface preview: retain the instructor simulator's built-in keyboard mapping.
// No Python/Myo controller or synthetic sensor input is used.
(() => {
  const params=new URLSearchParams(location.search);
  if(params.get('controller')!=='3.2a' || params.get('input')!=='native') return;
  const demo=window.awareDemo=window.awareDemo || {};
  Object.assign(demo,{enabled:false,phase:'waiting',lastState:-Infinity});
  const panel=document.createElement('div');
  Object.assign(panel.style,{position:'fixed',bottom:'16px',left:'16px',zIndex:40,
    padding:'16px 20px',borderRadius:'14px',background:'#101d2ef2',color:'#f5f8ff',font:'14px/1.6 system-ui'});
  panel.innerHTML='<strong>Keyboard Demo · Interface Preview</strong><br>Built-in keyboard controls · No Myo controller<br>W/S: forward/reverse · A/D: turn · Shift: fine control<br>Release keys to stop · C: camera<br><button data-start>Start / Resume</button> <button data-stop>Stop Driving</button><br><span role="status"></span>';
  panel.id = 'keyboard-demo-panel';
  document.body.appendChild(panel);
  const status=panel.querySelector('span');
  let sim,arbiter,started=null,lastPhase='stopped';
  function stop(reason='Driving stopped. Select Start / Resume.') {
    demo.enabled=false;started=null;status.textContent=reason;
    if(arbiter) arbiter.paused=true;
    if(sim) sim.deliver({type:'cmd',v:0,w:0,source:'interface-preview'});
  }
  function start() {
    if(!sim) {status.textContent='Simulator loading. Please wait.';return;}
    if(sim.gate.isEstopped()) {status.textContent='Clear E-stop before resuming.';return;}
    if(['finished','dnf'].includes(sim.run.phase)) {status.textContent='Select Restart Course first.';return;}
    demo.enabled=true;started=performance.now();status.textContent='Get ready. Release movement keys.';
  }
  panel.querySelector('[data-start]').addEventListener('click',e=>{e.currentTarget.blur();start();});
  panel.querySelector('[data-stop]').addEventListener('click',()=>stop());
  window.addEventListener('keydown',e=>{
    if(e.code==='Space') {e.preventDefault();e.stopImmediatePropagation();stop();}
    else if(e.code==='Enter' && !e.repeat && !['INPUT','TEXTAREA','SELECT','BUTTON'].includes(e.target?.tagName)) {
      e.preventDefault();e.stopImmediatePropagation();start();
    }
  },true);
  window.addEventListener('blur',()=>stop('Window inactive. Select Start / Resume.'));
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stop('Tab hidden. Select Start / Resume.');});
  setInterval(()=>{
    if(!sim && window.__sim && window.__arbiter && window.__bridge) {
      sim=window.__sim;arbiter=window.__arbiter;
      // Remote commands must never compete with native controls in this preview.
      window.__bridge.close();
      window.__bridge.connect=()=>{status.textContent='Bridge control disabled in interface preview.';};
      const reset=sim.reset.bind(sim);
      sim.reset=()=>{stop();return reset();};
      stop();
    }
    if(!sim) return;
    let phase='stopped',remaining=3,reason='Keyboard test paused';
    if(sim.gate.isEstopped()) {stop('Emergency stop active. Clear E-stop before resuming.');reason='Simulator estop';}
    else if(['finished','dnf'].includes(sim.run.phase)) {stop('Course ended. Select Restart Course.');reason='Simulator finished';}
    else if(demo.enabled) {
      const elapsed=(performance.now()-started)/1000;
      phase=elapsed<0.8?'waiting':elapsed<3.8?'countdown':'running';
      remaining=phase==='countdown'?Math.ceil(3.8-elapsed):phase==='running'?0:3;
      reason='';
    }
    arbiter.paused=phase!=='running';
    if(arbiter.paused) sim.deliver({type:'cmd',v:0,w:0,source:'interface-preview'});
    if(phase==='running' && lastPhase!=='running') status.textContent='W/S to drive, A/D to turn. Release keys to stop.';
    lastPhase=phase;
    Object.assign(demo,{phase,lastState:performance.now()});
    window.dispatchEvent(new CustomEvent('aware-native-state',{detail:{type:'preparation',version:'3.2a-experimental',phase,remaining,reason}}));
  },30);
})();
