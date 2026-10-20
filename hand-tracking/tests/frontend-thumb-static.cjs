const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../web/main.js'),'utf8');
const elements=new Map();
function $(id){if(!elements.has(id))elements.set(id,{textContent:'',value:'rest',disabled:false,replaceChildren(...children){this.children=children;}});return elements.get(id);}
const context={$ ,document:{createElement(){return {};}}};vm.createContext(context);
const start=source.indexOf('const thumbCalNames=');const end=source.indexOf('for(const [id,name] of Object.entries({thumbCalCapture:');
vm.runInContext(source.slice(start,end),context);
const cal={kind:'depth_thumb_static',available:true,enabled:false,has_profile:false,compatible:false,ready:false,
 samples:{},rejections:{},collecting:null,message:'四位置',references_deg:{rest:[26,10,18],index_base:[55,12,48],pinky_side:[72,26,65],lift:[60,10,18]},
 bounds_deg:[[2,78],[-8,28.37],[2,78]],reference_preview:false};
const data={calibration:{enabled:true,collecting:null}};
context.updateThumbCalibration(cal,data);
assert.equal($('thumbCalStep').children.length,4);assert.equal($('thumbCalSteps').children.length,4);
assert.match($('thumbCalTitle').textContent,/静态/);assert.equal($('thumbRef0').value,26);
assert.equal($('thumbStaticReferences').hidden,false);
cal.samples.rest={valid_frames:40};context.updateThumbCalibration(cal,data);
assert.equal($('thumbCalStep').value,'index_base');assert.equal($('thumbRef0').value,55);
assert.equal($('thumbCalProgress').value,.25);
$('thumbRef0').value=53;context.updateThumbCalibration(cal,data);assert.equal($('thumbRef0').value,53,'status must not erase local edits');
cal.reference_preview=true;context.updateThumbCalibration(cal,data);assert.match($('modelPoseMode').textContent,/不发送/);
cal.collecting='index_base';cal.phase='sample';cal.progress=.5;cal.valid_frames=12;cal.rejections={'指尖深度不足':5};
context.updateThumbCalibration(cal,data);assert.equal($('thumbCalPreview').disabled,true);assert.equal($('thumbRef0').disabled,true);
assert.match($('thumbCalMessage').textContent,/指尖深度不足/);
// Run the actual render-target application independently of Three.js.
const renderStart=source.indexOf('    const frameKey=');const renderEnd=source.indexOf('    controls.update(); renderer.render',renderStart);
assert.ok(renderStart>0&&renderEnd>renderStart);
context.applied={};context.robot={setJointValue(name,value){context.applied[name]=value;}};
context.latest={frame_id:1,joints:{left_thumb_j0:.1,left_index_j1:.3},thumb_calibration:{reference_revision:1,reference_joints:{left_thumb_j0:.8}}};
vm.runInContext('let renderedFrame=-1;function applyReference(){'+source.slice(renderStart,renderEnd)+'}',context);
context.applyReference();assert.equal(context.applied.left_thumb_j0,.8);assert.equal(context.latest.joints.left_thumb_j0,.1);
context.latest.thumb_calibration={reference_revision:2,reference_joints:{}};context.applyReference();assert.equal(context.applied.left_thumb_j0,.1);
console.log('PASS: four static poses, editable references, progress, capture locks and display-only pose override.');
