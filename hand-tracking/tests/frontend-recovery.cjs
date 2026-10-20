// Behavioral regressions for the actual browser functions, without a camera.
const fs=require('node:fs');
const vm=require('node:vm');
const assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../web/main.js'),'utf8');
const elements=new Map();
const element=id=>{if(!elements.has(id))elements.set(id,{clientWidth:300,clientHeight:240,appendChild(){},textContent:'',src:''});return elements.get(id);};
const context={console,document:{hidden:false},$ :element,handEdges:[],lastUiAt:0,latest:null,
  receivedAt:0,stopped:false,socket:null,performance:{now:()=>100},Date,
  setTimeout:()=>{},comparisonPreview:data=>data,update:()=>{},updateHardware:()=>{throw Error('SDK panel should stay hidden')},
};
let controls,renderCount=0;
class Controls{
  constructor(){controls=this;this.target={set(){}};this.listeners=[];this.moving=false;}
  update(){if(this.moving){for(const listener of this.listeners)listener();this.moving=false;}}
  addEventListener(_,listener){this.listeners.push(listener);}
}
class Geometry{constructor(){this.attributes={};}setAttribute(key,value){this.attributes[key]=value;}computeBoundingSphere(){}}
context.OrbitControls=Controls;
context.ResizeObserver=class{constructor(callback){this.callback=callback;}observe(){this.callback();}};
context.THREE={Scene:class{add(){}},Color:class{},PerspectiveCamera:class{constructor(){this.position={set(){}};}updateProjectionMatrix(){}},
  WebGLRenderer:class{setPixelRatio(){}setSize(){}render(){renderCount++;}},BufferGeometry:Geometry,
  BufferAttribute:class{constructor(array){this.array=array;}},LineSegments:class{},LineBasicMaterial:class{},Points:class{},PointsMaterial:class{}};
const sockets=[];
context.WebSocket=class{static OPEN=1;static CONNECTING=0;constructor(){this.readyState=1;sockets.push(this);}close(){this.readyState=3;this.onclose?.();}};
context.location={host:'127.0.0.1:8090'};
vm.createContext(context);
vm.runInContext(source.slice(source.indexOf('function skeletonView('),source.indexOf('function comparisonPreview(')),context);
context.skeletonView('skeletonA',0xffffff);
controls.moving=true;controls.update();
assert.ok(renderCount>=2,'orbit change must render without recursively updating controls');
vm.runInContext(source.slice(source.indexOf('function connect()'),source.indexOf('async function action(')),context);
context.connect();const first=context.socket;
context.comparisonPreview=()=>{throw Error('bad view');};
context.console={error(){}};
first.onmessage({data:'{}'});
assert.equal(context.receivedAt,100,'view failure must not stop receive heartbeat');
context.comparisonPreview=data=>data;let updates=0;context.update=()=>updates++;
context.lastUiAt=0;first.onmessage({data:'{}'});assert.equal(updates,1);
context.reconnectStream();assert.notEqual(context.socket,first);assert.equal(first.readyState,3);
first.onclose();assert.equal(context.socket,sockets[1],'old close event must not clear the new connection');
assert.match(element('camera').src,/^\/video\?reconnect=/);
let watchdog;
context.setInterval=callback=>{watchdog=callback;};
vm.runInContext(source.slice(source.lastIndexOf('setInterval(()=>{'),source.indexOf('loadModel().catch')),context);
context.performance.now=()=>3000;const stale=context.socket;watchdog();
assert.notEqual(context.socket,stale,'an OPEN socket without messages must reconnect');
context.document.hidden=true;context.performance.now=()=>6000;const hidden=context.socket;watchdog();
assert.equal(context.socket,hidden,'hidden pages must not restart streams');
console.log('PASS: orbit render, view-error isolation, receive heartbeat, reconnect, stale-close and watchdog handling');
