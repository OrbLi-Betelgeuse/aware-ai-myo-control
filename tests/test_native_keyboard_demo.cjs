const assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const code=fs.readFileSync(require('node:path').join(__dirname,'../jackal-site/native-keyboard-demo.js'),'utf8');
let now=0,tick,closed=false,estop=false,payload;
const buttons={},events={},status={};
const sim={gate:{isEstopped:()=>estop},run:{phase:'ready'},deliver(c){this.last=c;},reset(){this.run.phase='ready';}};
const arbiter={paused:false};
const window={__sim:sim,__arbiter:arbiter,__bridge:{close(){closed=true;},connect(){}},addEventListener:(k,f)=>events[k]=f,dispatchEvent:e=>payload=e.detail};
const context={URLSearchParams,location:{search:'?controller=3.2a&input=native'},window,document:{body:{appendChild(){}},createElement:()=>({style:{},querySelector:s=>s==='span'?status:(buttons[s]={addEventListener:(t,f)=>buttons[s].click=f})}),addEventListener(){}},performance:{now:()=>now},CustomEvent:class{constructor(t,o){this.detail=o.detail;}},setInterval:f=>tick=f};
vm.runInNewContext(code,context);tick();assert.ok(closed);assert.ok(arbiter.paused);
buttons['[data-start]'].click({currentTarget:{blur(){}}});tick();assert.equal(payload.phase,'waiting');
for(const [t,n] of [[800,3],[1800,2],[2800,1]]){now=t;tick();assert.equal(payload.remaining,n);assert.ok(arbiter.paused);assert.equal(sim.last.v,0);}
now=3800;tick();assert.equal(payload.phase,'running');assert.equal(arbiter.paused,false);
buttons['[data-stop]'].click();tick();assert.ok(arbiter.paused);assert.equal(window.awareDemo.enabled,false);
estop=true;buttons['[data-start]'].click({currentTarget:{blur(){}}});tick();assert.equal(payload.reason,'Simulator estop');assert.ok(arbiter.paused);
estop=false;buttons['[data-start]'].click({currentTarget:{blur(){}}});sim.reset();tick();assert.equal(payload.phase,'stopped');
console.log('Native preview: remote isolation, Ready, 3/2/1, release to native mapping, stop, E-stop and reset passed.');
