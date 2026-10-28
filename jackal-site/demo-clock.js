// Practice scoring only: keep physics, state timestamps and safety checks running.
(() => {
  const params=new URLSearchParams(location.search);
  if(params.get('controller')!=='3.2a') return;
  const demo=window.awareDemo=window.awareDemo || {enabled:params.get('keyboard')!=='1',phase:'waiting',lastState:-Infinity};
  const timer=setInterval(()=>{
    const sim=window.__sim;
    if(!sim?.run || sim.run.demoClockInstalled) return;
    const run=sim.run, update=run.update;
    run.demoClockInstalled=true;
    run.update=function(state) {
      const hold=!demo.enabled || demo.phase!=='running' ||
        performance.now()-demo.lastState>750 || sim.gate.isEstopped();
      if(this.phase==='running' && hold) this.startT=state.t-this.elapsed;
      return update.call(this,state);
    };
    clearInterval(timer);
  },50);
})();
