// Offline DOM/socket probes. No browser or network connection is opened.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const code = fs.readFileSync(path.join(__dirname, '../jackal-site/preparation-demo.js'), 'utf8');
function setup(search) {
  let now=0, ws, interval;
  const finish={style:{display:"none"}};
  const number={textContent:'',classList:{remove(){},add(){}},offsetWidth:1};
  const hint={textContent:''};
  const root={style:{},dataset:{},querySelector(s){return s==='.number'?number:hint;}};
  let created=0;
  const badge={style:{},setAttribute(){}};
  class Socket { constructor(){ws=this;} close(){} }
  const context={window:{},URLSearchParams,location:{search},performance:{now:()=>now},
    document:{getElementById:id=>id==='finish-banner'?finish:null,head:{appendChild(){}},body:{appendChild(){}},createElement(tag){created++;return tag==='style'?{}:created===2?root:badge;}},
    WebSocket:Socket,setTimeout(){},setInterval(fn){interval=fn;}};
  vm.runInNewContext(code,context);
  return {root,number,hint,badge,finish,get created(){return created;},send(phase,remaining,reason=''){ws.onmessage({data:JSON.stringify({type:'preparation',version:'3.2a-experimental',phase,remaining,reason})});},tick(t){now=t;interval();}};
}
assert.equal(setup('').created,0,'ordinary competition page is untouched');
const x=setup('?controller=3.2a&keyboard=1');
for(const n of [3,2,1]){x.send('countdown',n);assert.equal(x.number.textContent,String(n));}
x.send('countdown',0);assert.equal(x.number.textContent,'1','invalid countdown cannot show GO');
x.send('running',0);
assert.equal(x.root.style.display,'none','central card disappears immediately');
assert.equal(x.badge.style.display,'flex');
x.tick(180);assert.equal(x.root.style.display,'none');assert.equal(x.badge.style.display,'flex');
x.send('running',0);x.tick(480);assert.equal(x.badge.style.display,'none','repeated state does not restart badge');
x.tick(1000);assert.equal(x.root.style.display,'grid');assert.equal(x.root.dataset.phase,'offline');assert.equal(x.badge.style.display,'none');
for(const [reason,label] of [['Simulator estop','Emergency Stop Active'],['Simulator finished','Course Ended'],['Keyboard test paused/disconnected','Driving Stopped'],['Simulator state missing/stale','Connection / Input Lost'],['Invalid configuration','Controller Stopped']]) {
  x.send('stopped',3,reason);assert.equal(x.number.textContent,label);
}
x.send('waiting',3);assert.equal(x.number.textContent,'Ready');
x.send('running',0);assert.equal(x.badge.style.display,'flex');
x.send('stopped',3,'Simulator estop');assert.equal(x.badge.style.display,'none');assert.equal(x.root.style.display,'grid');
assert.ok(setup('?controller=3.2a').created>0);
assert.equal(setup('?controller=3.2').created,0);
console.log('Demo status checks passed: opt-in, countdown, GO, stale display, specific stop causes.');

for(const query of ['?controller=3.2a','?controller=3.2a&keyboard=1']) {
  const end=setup(query);
  end.finish.style.display='block';
  end.send('stopped',3,'Simulator finished');
  assert.equal(end.root.style.visibility,'hidden');
  assert.equal(end.finish.style.display,'block','native result must stay visible');
  end.tick(1000);
  assert.equal(end.root.style.visibility,'hidden','connection loss must not cover results');
  end.finish.style.display='none';
  end.send('waiting',3);end.tick(1010);
  assert.equal(end.root.style.visibility,'visible');
  assert.equal(end.root.style.display,'grid');
}
console.log('Result banner priority and reset recovery passed.');
