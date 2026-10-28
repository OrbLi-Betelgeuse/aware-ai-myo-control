// Offline event/DOM probes; no browser or network.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const code=fs.readFileSync(require('node:path').join(__dirname,'../jackal-site/keyboard-myo-test.js'),'utf8');
function setup(search){
 let now=0,ws,tick,created=0;const events={},docEvents={},sent=[],status={},buttons={};
 const document={querySelectorAll:()=>[],hidden:false,hasFocus:()=>true,body:{appendChild(){}},createElement(){created++;return {style:{},querySelector:s=>s==='span'?status:(buttons[s]={addEventListener:(type,fn)=>buttons[s].click=fn})};},addEventListener:(k,f)=>docEvents[k]=f};
 class Socket{static OPEN=1;constructor(){ws=this;this.readyState=1;}send(s){sent.push(JSON.parse(s));}close(){}}
 vm.runInNewContext(code,{MutationObserver:class {observe(){}},URLSearchParams,location:{search},document,window:{addEventListener:(k,f)=>events[k]=f},WebSocket:Socket,performance:{now:()=>now},setTimeout(){},setInterval:f=>tick=f});
 return {camera(){docEvents.click({target:{closest:()=>({dataset:{cam:'top'},blur(){}})}});},clickStart(){buttons['[data-start]'].click({currentTarget:{blur(){}}});},get created(){return created;},document,open(){ws.onopen();},last:()=>sent.at(-1),key(code,type='keydown',target={}){events[type]({code,target,preventDefault(){},stopImmediatePropagation(){}});},blur(){events.blur();},hide(){document.hidden=true;docEvents.visibilitychange();},tick(t){now=t;tick();}};
}
assert.equal(setup('?controller=3.2a').created,0);
assert.equal(setup('?keyboard=1').created,0);
const x=setup('?controller=3.2a&keyboard=1');x.open();assert.equal(x.last().enabled,false);
x.key('Enter');assert.equal(x.last().enabled,true);
x.key('KeyW');x.key('KeyA');assert.deepEqual(x.last().keys,['KeyW','KeyA']);
x.key('KeyW','keyup');assert.deepEqual(x.last().keys,['KeyA']);
x.key('Space');assert.equal(x.last().enabled,false);assert.deepEqual(x.last().keys,[]);
x.key('Enter','keydown',{tagName:'INPUT'});assert.equal(x.last().enabled,false);
x.key('Enter');x.blur();assert.equal(x.last().enabled,false);
x.key('Enter');x.tick(600);assert.equal(x.last().enabled,false);
x.key('Enter');x.hide();assert.equal(x.last().enabled,false);
console.log('Keyboard UI checks passed: opt-in, start, combined keys, release, pause, editable focus, blur, stalled ack, hidden tab.');

const y=setup('?controller=3.2a&keyboard=1');y.open();y.clickStart();assert.equal(y.last().enabled,true);console.log('Start button sends enabled input.');

y.camera();assert.equal(y.last().enabled,true);assert.equal(y.last().camera.mode,'top');console.log('Camera click preserves enabled input.');
