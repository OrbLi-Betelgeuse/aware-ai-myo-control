// Offline DOM/socket probes. No browser or network connection is opened.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const code = fs.readFileSync(path.join(__dirname, '../jackal-site/preparation-overlay.js'), 'utf8');
function setup(search) {
  let now=0, ws, interval;
  const number={textContent:'',classList:{remove(){},add(){}},offsetWidth:1};
  const hint={textContent:''};
  const root={style:{},dataset:{},querySelector(s){return s==='.number'?number:hint;}};
  let created=0;
  class Socket { constructor(){ws=this;} close(){} }
  const context={URLSearchParams,location:{search},performance:{now:()=>now},
    document:{head:{appendChild(){}},body:{appendChild(){}},createElement(tag){created++;return tag==='style'?{}:root;}},
    WebSocket:Socket,setTimeout(){},setInterval(fn){interval=fn;}};
  vm.runInNewContext(code,context);
  return {root,number,hint,get created(){return created;},send(phase,remaining){ws.onmessage({data:JSON.stringify({type:'preparation',version:'3.2a-experimental',phase,remaining})});},tick(t){now=t;interval();}};
}
assert.equal(setup('').created,0,'ordinary competition page is untouched');
const x=setup('?controller=3.2a');
for(const n of [3,2,1]){x.send('countdown',n);assert.equal(x.number.textContent,String(n));}
x.send('countdown',0);assert.equal(x.number.textContent,'1','invalid countdown cannot show GO');
x.send('running',0);assert.equal(x.number.textContent,'GO');
x.tick(710);assert.equal(x.root.style.display,'none');
x.tick(800);assert.equal(x.root.style.display,'grid');assert.equal(x.root.dataset.phase,'offline');
x.send('stopped',3);assert.equal(x.number.textContent,'STOP');
x.send('waiting',3);assert.equal(x.number.textContent,'READY');
console.log('Overlay checks passed: opt-in, 3/2/1, invalid payload, GO, hide, stale connection, STOP, READY.');
