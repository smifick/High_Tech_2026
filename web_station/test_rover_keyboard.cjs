const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict');
const handlers={},elements={},commands=[];
function element(){return {hidden:false,disabled:false,value:'.12',dataset:{},classList:{add(){},remove(){}},reportValidity:()=>true,addEventListener(){},closest:()=>null}}
for(const name of ['forward','backward','left','right','turn_left','turn_right'])elements[`[data-drive="${name}"]`]=Object.assign(element(),{dataset:{drive:name}});
function $(key){return elements[key]??=(element())}
const ctx={$,apiReady:true,state:{},notice(){},task:fn=>fn,post:async(action,data)=>{commands.push(data);return {lease:'test'}},document:{hidden:false,querySelector:s=>s==='dialog[open]'?null:$(s),querySelectorAll:()=>Object.entries(elements).filter(([k])=>k.startsWith('[data-drive=')).map(([,v])=>v),addEventListener:(name,fn)=>handlers[name]=fn},window:{addEventListener:(name,fn)=>handlers[name]=fn},setTimeout:()=>1,clearTimeout(){},console};
let src=fs.readFileSync(require('node:path').join(__dirname,'static/app.js'),'utf8');src=src.slice(src.indexOf('let roverHold=null;'),src.indexOf('function renderRover('));vm.createContext(ctx);vm.runInContext(src,ctx);
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function event(code,target=element()){return {code,target,preventDefault(){},repeat:false}}
(async()=>{
for(const [code,direction] of Object.entries({KeyW:'forward',KeyS:'backward',KeyA:'left',KeyD:'right',KeyQ:'turn_right',KeyE:'turn_left'})){
commands.length=0;handlers.keydown(event(code));await flush();assert.equal(commands[0].direction,direction);handlers.keydown({...event(code),repeat:true});await flush();assert.equal(commands.length,1);handlers.keyup(event(code));await flush();assert.equal(commands.at(-1).operation,'stop');}
commands.length=0;handlers.keydown(event('KeyW',{closest:()=>({})}));await flush();assert.equal(commands.length,0);
$('#tab-rover').hidden=true;handlers.keydown(event('KeyW'));await flush();assert.equal(commands.length,0);$('#tab-rover').hidden=false;
handlers.keydown(event('KeyW'));await flush();handlers.blur();await flush();assert.equal(commands.at(-1).operation,'stop');
commands.length=0;handlers.keydown(event('KeyW'));handlers.keyup(event('KeyW'));await flush();assert.equal(commands.length,0);
handlers.keydown(event('Space'));await flush();assert.equal(commands.at(-1).operation,'stop');
console.log('Keyboard: 6 directions, repeat, release, input focus, hidden tab, blur, early release and Space passed');
})().catch(e=>{console.error(e);process.exitCode=1});
