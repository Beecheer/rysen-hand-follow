// Exercise the actual status renderer with a minimal DOM, without a camera.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../web/main.js'),'utf8');
const elements=new Map();
function $(id){if(!elements.has(id))elements.set(id,{textContent:'',value:'rest',disabled:false,replaceChildren(...children){this.children=children;}});return elements.get(id);}
const context={$ ,document:{createElement(){return {};}}};vm.createContext(context);
const start=source.indexOf('const thumbCalNames=');
const end=source.indexOf('for(const [id,name] of Object.entries({thumbCalCapture:');
assert.ok(start>=0&&end>start);vm.runInContext(source.slice(start,end),context);
const data={calibration:{enabled:true,collecting:null}};
const cal={collecting:'rest',samples:{},phase:'sample',progress:.5,valid_frames:12,rejections:{'拇指被遮挡':3},ready:false,has_profile:false,enabled:false,compatible:false};
context.updateThumbCalibration(cal,data);
for(const id of ['resume','calCapture','calApply','calToggle','calReset','indexFollowStart'])assert.equal($(id).disabled,true,id);
assert.match($('thumbCalMessage').textContent,/有效 12 帧/);assert.match($('thumbCalMessage').textContent,/拇指被遮挡/);
cal.collecting=null;cal.samples.rest={valid_frames:40};cal.message='已完成';
context.updateThumbCalibration(cal,data);
assert.equal($('thumbCalStep').value,'plane');assert.equal($('thumbCalCapture').disabled,false);
assert.match($('thumbCalInstruction').textContent,/沿掌面/);
cal.has_profile=true;cal.compatible=true;cal.enabled=false;
context.updateThumbCalibration(cal,data);assert.equal($('thumbCalToggle').disabled,false);
cal.compatible=false;context.updateThumbCalibration(cal,data);assert.equal($('thumbCalToggle').disabled,true);
assert.match($('thumbCalMode').textContent,/需重采/);
console.log('PASS: thumb capture progress, mutual exclusion, rejection feedback, next-step guide and profile compatibility');
