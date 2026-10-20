const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const elements=new Map(),contexts=new Map();
function element(id){
 if(!elements.has(id))elements.set(id,{textContent:'',disabled:false,hidden:false,
  getContext(){if(!contexts.has(id))contexts.set(id,{points:0,waiting:false,clearRect(){this.points=0;this.waiting=false;},fillText(){this.waiting=true;},beginPath(){},moveTo(){},lineTo(){},stroke(){},arc(){this.points++;},fill(){}});return contexts.get(id);},setPointerCapture(){}});
 return elements.get(id);
}
const context={document:{getElementById:element},Math};vm.createContext(context);
const source=fs.readFileSync(require('node:path').join(__dirname,'../web/depth.js'),'utf8').replace('export function','function');
vm.runInContext(source,context);
context.updateDepth({});assert.equal(element('depthPanel').hidden,true);
const points=Array.from({length:21},(_,i)=>[i*.001,-i*.002,.5]);
const data={index_follow:{},depth:{state:'RUNNING',age_ms:20,device:{usb:'3.2'},timestamp_skew_ms:8,
 control_gate:'请完成标定',hand_tracking:{state:'FUSED',hands:[{model_points_m:points,fused_points_m:points,
 sources:Array(21).fill('depth_band'),quality:{depth_constraints:21,bone_error_p95_mm:2},personal_reference:true}]}}};
context.updateDepth(data);assert.equal(element('depthPanel').hidden,false);
assert.match(element('depthStatus').textContent,/21\/21/);assert.match(element('depthPerFinger').textContent,/拇指：4\/4/);
assert.equal(element('indexFollowStart').disabled,true);assert.equal(contexts.get('depthFused').points,21);
element('depthFused').onpointerdown({clientX:0,clientY:0,pointerId:1});
element('depthFused').onpointermove({clientX:10,clientY:2});assert.equal(contexts.get('depthModel').points,21);
data.depth.age_ms=700;context.updateDepth(data);
assert.equal(contexts.get('depthFused').points,0);assert.equal(contexts.get('depthFused').waiting,true);
data.depth.age_ms=20;data.depth.hand_tracking.hands=[];context.updateDepth(data);
assert.equal(contexts.get('depthModel').points,0);
data.depth.algorithm='d435-articulated-v2';data.depth.thumb_direct={fixed_terminal_deg:12};
data.depth.hand_tracking.hands=[{model_points_m:points,fused_points_m:points,sources:Array(21).fill('depth_band')}];
context.updateDepth(data);assert.equal(contexts.get('depthFused').points,0,'v2 must not show v1 points as articulated');
data.depth.hand_tracking.hands[0].articulated_points_m=points;
context.updateDepth(data);assert.equal(contexts.get('depthFused').points,21);
data.human_structure={finger_solver:'rgb_depth_root_correction',root_correction:{palm_used:true,
 palm_correction_deg:1.4,fingers:{index:{near_depth_used:true}}}};
context.updateDepth(data);
assert.match(element('depthStatus').textContent,/掌面修正 1.4°/);
assert.match(element('depthStatus').textContent,/近节深度 1\/4 指/);
console.log('PASS: D435 dual skeleton, source counts, control gate, view rotation and stale clearing.');
