// Experimental simulator controls, shared by real Myo and keyboard input.
(() => {
  if(new URLSearchParams(location.search).get('controller')!=='3.2a') return;
  function organizeControls() {
    const buttons=[...document.querySelectorAll('button')];
    const emergency=buttons.find(b=>b.textContent.trim()==='E-stop');
    const clear=buttons.find(b=>b.textContent.trim()==='Clear E-stop');
    const reset=buttons.find(b=>b.textContent.trim()==='Reset');
    if(reset) reset.textContent='Restart Course';
    if(!emergency || !clear || emergency.closest('[data-demo-emergency]')) return;
    const group=document.createElement('details');
    group.dataset.demoEmergency='';group.open=true;
    const title=document.createElement('summary');title.textContent='Emergency Controls';
    const note=document.createElement('div');note.textContent='Stops the vehicle. Demo timer paused.';
    emergency.parentNode.insertBefore(group,emergency);
    group.append(title,emergency,clear,note);
  }
  organizeControls();
  new MutationObserver(organizeControls).observe(document.body,{childList:true,subtree:true});
})();
