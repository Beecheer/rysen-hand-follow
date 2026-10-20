import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from './vendor/urdf/URDFLoader.js';
import {updateDepth} from './depth.js?v=root-correction-v1';

const $ = id => document.getElementById(id);
let previewBackend='mediapipe';
let comparisonDrawFailed=false;
const skeletonViews={};
const handEdges=[[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],[13,17],[17,18],[18,19],[19,20],[17,0]];
function skeletonView(id,color){
  const host=$(id),scene=new THREE.Scene();scene.background=new THREE.Color('#101c2b');
  const camera=new THREE.PerspectiveCamera(38,1,.01,30);camera.position.set(0,0,4.4);
  const renderer=new THREE.WebGLRenderer({antialias:false});renderer.setPixelRatio(1);host.appendChild(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,.5,0);controls.update();
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(handEdges.length*6),3));
  const line=new THREE.LineSegments(geometry,new THREE.LineBasicMaterial({color}));scene.add(line);
  const pointsGeometry=new THREE.BufferGeometry();pointsGeometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(63),3));
  const dots=new THREE.Points(pointsGeometry,new THREE.PointsMaterial({color,size:.045}));scene.add(dots);
  // OrbitControls dispatches change inside update(), before committing its
  // previous pose. Calling update() from the change handler recurses forever.
  const render=()=>{if(document.hidden)return;const w=host.clientWidth,h=host.clientHeight;if(w<1||h<1)return;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();renderer.render(scene,camera);};
  controls.addEventListener('change',render);new ResizeObserver(render).observe(host);
  return hands=>{const hand=hands?.find(h=>h.label==='Left');line.visible=dots.visible=!!hand;if(hand){const p=hand.world,root=p[0],length=Math.hypot(...p[9].map((x,i)=>x-root[i]));if(length>1e-5){const q=p.map(v=>[(v[0]-root[0])/length,-(v[1]-root[1])/length,-(v[2]-root[2])/length]);pointsGeometry.attributes.position.array.set(q.flat());geometry.attributes.position.array.set(handEdges.flatMap(([a,b])=>[...q[a],...q[b]]));geometry.attributes.position.needsUpdate=pointsGeometry.attributes.position.needsUpdate=true;geometry.computeBoundingSphere();pointsGeometry.computeBoundingSphere();}}render();};
}
function comparisonPreview(data){
  const c=data.comparison;$('comparisonPanel').hidden=!c;if(!c)return data;
  if(!skeletonViews.a){skeletonViews.a=skeletonView('skeletonA',0x6cdac8);skeletonViews.b=skeletonView('skeletonB',0xffc46d);}
  $('comparisonStatus').textContent=c.error||`WiLoR 已就绪 · ${c.device||''} · 完成 ${c.completed} 帧，覆盖待处理旧帧 ${c.dropped} 次`;
  const pair=c.pair;
  if(c.recording?.path)$('recordingStatus').textContent=`${c.recording.active?'正在录制':'已保存'} · ${c.recording.elapsed.toFixed(1)} 秒 / ${c.recording.frames} 帧 · ${c.recording.path}`;
  if(c.record_error)$('recordingStatus').textContent=c.record_error;
  if(pair){
    if(!comparisonDrawFailed){try{skeletonViews.a(pair.age_ms<1000?pair.a:[]);skeletonViews.b(pair.age_ms<1000?pair.b:[]);}
      catch(error){comparisonDrawFailed=true;console.error('人手骨架绘制异常',error);}}
    if(comparisonDrawFailed)$('comparisonStatus').textContent+=' · 骨架视图异常，请刷新；状态数据仍更新';
    $('comparisonTiming').textContent=`同源帧 #${pair.frame_id} · A 推理 ${pair.a_inference_ms.toFixed(1)} ms / 映射 ${pair.a_mapping_ms.toFixed(1)} ms · B 推理 ${pair.b_inference_ms.toFixed(1)} ms / 映射 ${pair.b_mapping_ms.toFixed(1)} ms · B 采集至目标 ${pair.processing_ms.toFixed(1)} ms · 当前结果年龄 ${pair.age_ms.toFixed(0)} ms · 最近 120 帧年龄 P95 ${c.age_p95_ms.toFixed(0)} ms`;
  }
  $('comparisonCalibration').textContent=previewBackend==='wilor'?'当前操作 B：WiLoR 使用四指并拢、四指微张、拇指向外三个姿态，每步准备 1 秒、采样 7 秒，保存到 wilor-current.json；A 标定不变。':'当前操作 A：每步准备 1 秒、采样 3 秒，使用原有 current.json。B 使用独立标定配置。';
  for(const id of ['checkOpenClose','checkPalm'])$(id).disabled=previewBackend==='wilor';
  if(!c.b.calibration.has_profile)c.b.human_structure.message='WiLoR 尚未进行个人标定，当前使用原始几何解算';
  return previewBackend==='wilor'?{...data,...c.b,comparison:c,robot_connected:false,hardware:null,index_follow:null}:data;
}
$('previewBackend').onchange=()=>{previewBackend=$('previewBackend').value;lastCalCompleted='';renderedFrame=-1;};
for(const [id,action] of [['recordComparison','start'],['stopRecording','stop']])$(id).onclick=async()=>{
  try{const r=await fetch(`/api/comparison/record/${action}`,{method:'POST'});const body=await r.json();if(!r.ok)throw Error(body.detail);}
  catch(error){$('recordingStatus').textContent=error.message;}
};
const fingers = [['index','食指'],['middle','中指'],['ring','无名指'],['pinky','小指']];
const calNames={neutral:'中立位',spread:'四指舒适张开',hook:'钩手',mcp:'指根轻弯、指间较直',index:'拇指碰食指尖',middle:'拇指碰中指尖',ring:'拇指碰无名指尖',pinky:'拇指碰小指尖'};
const calInstructions={neutral:'四指自然伸直、接近平行，拇指舒适展开。采集中不要转动或收起手掌。',
  spread:'掌心朝前，四指舒适张开，保持指间可见；不强行撑到极限。用于侧摆范围检查，不把此姿态当成固定角度。',
  hook:'掌心朝前，指根尽量伸展，主要弯曲指间关节，像钩子。保持各指关节尽量可见。',
  mcp:'掌心朝前，四指从指根轻轻向掌心方向弯曲，指间尽量较直。不要让指尖完全指向镜头，不要求准确角度。',
  index:'拇指与食指指尖自然轻触，食指可以适度弯曲。让两个指尖都尽量可见，保持 3 秒。',
  middle:'拇指与中指指尖自然轻触，其他手指保持舒适。不要用力挤压或扭动手掌。',
  ring:'拇指与无名指指尖自然轻触，允许无名指适度弯曲。保持手掌和指尖完整入镜。',
  pinky:'拇指与小指指尖自然轻触，以舒适姿态为准，不要强行伸展。保持 3 秒。'};
const wilorCalNames={together:'四指并拢伸直',spread:'四指微张',thumb_direction:'拇指向外标定'};
const wilorCalInstructions={
  together:'掌心朝摄像头，四指自然伸直并拢，拇指舒适放松；保持整只手完整入镜。',
  spread:'掌心和手腕保持不动，四指伸直后自然微张约 5–15°，不要用力撑到极限。',
  thumb_direction:'保持掌心朝前和四指并拢伸直，拇指沿掌面向外舒适张开并略微抬起，拇指根部与指尖都要可见。'};
let lastCalCompleted='';
let calibrationMode='legacy';
$('calPose').onchange=()=>{$('calInstruction').textContent=calInstructions[$('calPose').value];};
function updateCalibration(cal,data){
  if(!cal)return;
  const wilorSimple=cal.finger_policy==='wilor_three_pose'||cal.schema===5;
  const simple=cal.schema===4;
  const names=wilorSimple?wilorCalNames:simple?{neutral:'伸直略分开',together:'并拢伸直'}:calNames;
  const mode=wilorSimple?'wilor-simple':simple?'simple':'legacy';
  if(calibrationMode!==mode){
    calibrationMode=mode;lastCalCompleted='';
    const options=Object.entries(names).map(([key,name],i)=>{const option=document.createElement('option');option.value=key;option.textContent=`${i+1} · ${name}`;return option;});
    $('calPose').replaceChildren(...options);$('calPose').value=wilorSimple?'together':'neutral';
    $('calPose').onchange=()=>{$('calInstruction').textContent=wilorSimple?wilorCalInstructions[$('calPose').value]:simple?
      ($('calPose').value==='neutral'?'掌心正对摄像头，四指自然伸直、略分开，拇指自然展开。':'保持掌心朝前，四指并拢伸直；拇指不作要求。'):calInstructions[$('calPose').value];};
    $('calPose').onchange();
  }
  if($('calTitle'))$('calTitle').textContent=wilorSimple?'WiLoR · 四指两姿态＋拇指方向标定':simple?'掌心朝前 · 两步骨架标定':'掌心朝前 · 八姿态个人标定';
  const caution=document.querySelector('.cal-caution');
  if(caution)caution.textContent=`掌心尽量正对摄像头，每步准备 1 秒、采样 ${(cal.capture_seconds||4)-1} 秒。`+(wilorSimple?'WiLoR 三维模型校正四指并拢零位、微张侧摆比例和拇指方向；无需握拳或做四指对指。':simple?'无需握拳、对指或拇指专项采样。':'不要求准确角度；对指由你确认轻触，不把视觉距离当接触证明。');
  $('calMode').textContent=cal.enabled?'已启用标定':cal.has_profile?'对比：未启用标定':'未标定模式';
  const completed=Object.keys(cal.samples);
  $('calSteps').replaceChildren(...Object.entries(names).map(([key,name])=>{
    const tag=document.createElement('span');tag.className=cal.samples[key]?'done':'';
    tag.textContent=`${cal.samples[key]?'✓ ':''}${name}`;return tag;
  }));
  const signature=completed.join(',');
  if(signature!==lastCalCompleted && !cal.collecting){
    lastCalCompleted=signature;
    const next=Object.keys(names).find(key=>!cal.samples[key]);
    if(next){$('calPose').value=next;$('calPose').onchange();}
  }
  $('calProgress').value=cal.collecting?cal.progress:completed.length/Object.keys(names).length;
  $('calMessage').textContent=cal.collecting?
    `${cal.collecting==='side_zero'?'四指侧摆零位':names[cal.collecting]} · ${cal.phase==='prepare'?'准备中':'采样中'} · 有效 ${cal.valid_frames} 帧`+
    (Object.keys(cal.rejections||{}).length?' · 暂未采入：'+Object.entries(cal.rejections).sort((a,b)=>b[1]-a[1])[0][0]:'') : cal.message;
  $('calPose').disabled=!!cal.collecting;
  $('calCapture').disabled=!!cal.collecting;
  $('calCancel').disabled=!cal.collecting;
  $('calApply').disabled=!cal.ready||!!cal.collecting;
  $('calToggle').disabled=!cal.has_profile||!!cal.collecting;
  $('calToggle').textContent=cal.enabled?'对比：停用标定':'对比：启用标定';
  $('resume').disabled=!!cal.collecting;
  $('sideZeroPanel').hidden=data.tracker!=='wilor-d435';
  $('sideZeroCapture').disabled=!cal.enabled||!cal.has_profile||!!cal.collecting;
  $('sideZeroApply').disabled=!cal.side_zero_pending||!!cal.collecting;
  $('sideZeroMessage').textContent=cal.collecting==='side_zero'?$('calMessage').textContent:
    cal.side_zero_pending?cal.message:cal.side_zero_ready?'两路侧摆零位已配对。':
    '请并拢伸直后采集并保存。此前深度侧摆不参与融合；拇指标定保留。';
  if(wilorSimple){
    $('pinchStatus').textContent='WiLoR MANO 三维关节用于连续解算；并拢姿态校正四指侧摆零位，微张姿态按个人手型归一侧摆幅度，拇指向外姿态对齐人手与模型的拇指方向。仍保留 URDF 软件限位。';
  }else if(simple){
    $('pinchStatus').textContent='四指以 RGB 三维骨架为主，深度只弱修正掌面及根部屈伸／侧摆；PIP、DIP 沿用视觉结果。拇指静态标定、固定末端和现有两步标定保留。';
  }else if(cal.enabled && data.valid && data.contact?.finger){
    const name=fingers.find(([key])=>key===data.contact.finger)?.[1];
    const residual=data.contact.target_error_mm;
    $('pinchStatus').textContent=`正在增强拇指—${name}对指约束，权重 ${(data.contact.strength*100).toFixed(0)}%`+
      (Number.isFinite(residual)?`，求解残差 ${residual.toFixed(1)} mm。`:'。')+'这是模型目标误差，不是实际接触测量；受限位影响可能无法完全贴合。';
  }else{
    $('pinchStatus').textContent=cal.enabled?(cal.schema===3?'个人骨长、掌面参考、二维侧摆与三维屈伸拟合已启用；拇指使用个人骨长重建后做机械手映射。中立位不扣除四指屈伸角。':'当前使用旧五姿态标定。完成八个姿态并应用后启用个人结构优化；旧文件会自动备份。'):'按顺序采集八个姿态后“应用并保存”。只保存关键点与统计，不保存画面；重置会保留备份。';
  }
}
const degree = x => Number.isFinite(x) ? (x * 180 / Math.PI).toFixed(1) : '—';
const stateNames = {WAITING:'等待左手', ACQUIRING:'正在捕获', TRACKING:'正在跟随', HOLD:'短暂丢手 · 保持姿态', PAUSED:'已暂停', ERROR:'摄像头异常'};
const reasonNames = {'Show your left hand':'请将左手完整放入画面', 'Direct joint-angle preview':'关键点直接计算角度，模型实时预览',
  'No hand detected':'当前没有检测到手；检查挡板、距离和光线',
  'Only right hand detected; check handedness setting':'当前只检测到 Right；若举起的是左手，请检查左右手校正',
  'Left label score below 0.65':'检测到 Left，但左右手分类分数低于 65%，暂不采用',
  'Palm outside image or invalid':'掌部关键点出画面或无效，请完整展示手掌',
  'Frame processing exceeded 200 ms':'单帧处理超过 200 ms，已丢弃过期结果',
  'No complete left hand (or low L/R score)':'左手不完整、未检测到，或左右手分类分数偏低',
  'Left hand lost >500 ms; click Resume':'丢手超过 500 ms；准备好后点击“恢复 / 重新捕获”',
  'Paused; click Resume to reacquire':'姿态已保持，点击恢复重新捕获左手',
  'Reacquiring left hand':'正在重新捕获左手', 'Waiting for 3 valid frames':'等待连续 3 帧有效关键点',
  'Stale landmarks; holding target':'数据超时，保持最后目标姿态',
  'Hand label changed; click Resume after checking left hand':'左右手校正已切换；确认标签后点击恢复',
  'Abrupt hand position change rejected':'手部位置突变，暂不采用该帧'};
for (const [key, name] of fingers) {
  const row = document.createElement('tr');
  row.innerHTML = `<td class="finger-name">${name}</td><td id="${key}-abd">—</td><td class="target" id="${key}-tabd">0.0</td><td id="${key}-mcp">—</td><td class="target" id="${key}-tmcp">0.0</td><td id="${key}-pip">—</td><td class="target" id="${key}-tpip">0.0</td>`;
  $('angles').appendChild(row);
}
const thumbNames=['根部外展 j0','根部旋转 j1','根部屈伸 j2','MCP 屈伸 j3'];
thumbNames.forEach((name,i)=>{
  const row=document.createElement('tr');
  row.innerHTML=`<td>${name}</td><td id="thumb-raw-${i}">—</td><td class="target" id="thumb-target-${i}">0.0</td><td id="thumb-state-${i}">等待</td>`;
  $('thumbAngles').appendChild(row);
});
let robot, controls, renderer, scene, camera, latest = null, receivedAt = 0, stopped = false, socket;
let modelEnabled=true,modelVisible=true,lastRenderAt=0,renderedFrame=-1,lastUiAt=0,renderCount=0;
$('toggleModel').onclick=()=>{
  modelEnabled=!modelEnabled;
  $('toggleModel').textContent=`三维预览：${modelEnabled?'开':'关'}`;
  $('toggleModel').setAttribute('aria-pressed',String(modelEnabled));
  $('viewer').dataset.rendering=String(modelEnabled);
};
let frameView = () => {};
async function loadModel() {
  const viewer = $('viewer');
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(36, 1, 0.001, 10);
  camera.up.set(0, 0, 1);
  renderer = new THREE.WebGLRenderer({antialias:false, alpha:true});
  renderer.setPixelRatio(1);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  viewer.appendChild(renderer.domElement);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xcce6ff, 0x273b4a, 2.6));
  const key = new THREE.DirectionalLight(0xffffff, 3); key.position.set(0.3, -0.3, 0.6); scene.add(key);
  const fill = new THREE.DirectionalLight(0x8edbe1, 1.8); fill.position.set(-0.3, 0.2, 0.1); scene.add(fill);
  const metadata = await fetch('/api/model').then(r => {if(!r.ok) throw Error('模型接口不可用'); return r.json();});
  const manager = new THREE.LoadingManager();
  const errors = [];
  manager.onError = url => errors.push(url);
  const loaded = new Promise((resolve, reject) => {
    manager.onLoad = () => errors.length ? reject(Error(`网格加载失败：${errors[0]}`)) : resolve();
  });
  const loader = new URDFLoader(manager);
  robot = await loader.loadAsync(metadata.asset);
  await loaded;
  const movable = Object.keys(robot.joints).filter(n => robot.joints[n].jointType === 'revolute');
  if (movable.length !== 21 || !Object.keys(metadata.joints).every(n => robot.joints[n])) throw Error('URDF 关节定义不匹配');
  robot.traverse(obj => {
    if(obj.isMesh) {
      obj.material = new THREE.MeshStandardMaterial({color:obj.name.includes('sensor')?0x405469:0xa8bccb, roughness:0.55, metalness:0.22});
    }
  });
  scene.add(robot);
  const box = new THREE.Box3().setFromObject(robot), center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const extent = Math.max(size.x, size.y, size.z);
  if (!Number.isFinite(extent) || extent <= 0) throw Error('模型尺寸无效');
  robot.updateMatrixWorld(true);
  const positionOf = name => robot.joints[name].getWorldPosition(new THREE.Vector3());
  const across = positionOf('left_index_j0').sub(positionOf('left_pinky_j0')).normalize();
  const forward = positionOf('left_middle_j2').sub(positionOf('left_middle_j0')).normalize();
  const palmNormal = new THREE.Vector3().crossVectors(across, forward).normalize();
  camera.up.copy(forward);
  frameView = () => {
    controls.target.copy(center);
    camera.position.copy(center).addScaledVector(palmNormal, extent * 1.9)
      .addScaledVector(across, extent * 0.25).addScaledVector(forward, extent * 0.12);
    controls.update(); controls.saveState();
  };
  frameView();
  controls.minDistance = extent * 0.5; controls.maxDistance = extent * 7;
  const resize = () => {const w=viewer.clientWidth, h=viewer.clientHeight; renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();};
  new ResizeObserver(resize).observe(viewer); resize();
  new IntersectionObserver(entries=>{modelVisible=entries[0].isIntersecting;}).observe(viewer);
  $('modelMessage').hidden = true;
  viewer.dataset.modelReady = 'true';
  viewer.dataset.jointCount = String(movable.length);
  renderer.setAnimationLoop(now => {
    if(document.hidden||!modelEnabled||!modelVisible||now-lastRenderAt<1000/30)return;
    lastRenderAt=now;
    const frameKey=latest?`${latest.frame_id}:${latest.thumb_calibration?.reference_revision||0}`:null;
    if(latest&&frameKey!==renderedFrame){
      for(const [name, value] of Object.entries({...latest.joints,...(latest.thumb_calibration?.reference_joints||{})})) robot.setJointValue(name, value);
      renderedFrame=frameKey;
    }
    controls.update(); renderer.render(scene,camera);
    renderCount++;
  });
  // Read-only diagnostics for render verification.
  window.previewDiagnostics = () => ({modelReady:true, joints:movable.length, meshes:countMeshes(),
    modelSize:size.toArray(), state:latest?.state, frame:latest?.frame_id,
    renderCount,renderFpsLimit:30,pixelRatio:renderer.getPixelRatio(),modelEnabled,modelVisible,
    targets:Object.fromEntries(movable.map(n=>[n,robot.joints[n].jointValue[0]]))});
  function countMeshes(){let n=0;robot.traverse(o=>{if(o.isMesh)n++;});return n;}
}
function updateDepthVideo(data){
  const depth=data.depth_video,enabled=!!depth?.enabled;
  $('depthCameraPanel').hidden=!enabled;
  $('cameraWorkspace').classList.toggle('with-depth-video',enabled);
  const image=$('depthCamera');
  if(!enabled){image.removeAttribute('src');return;}
  if(!document.hidden&&!stopped&&!image.getAttribute('src'))image.src='/depth-video';
  const fresh=depth.ready&&data.frame_age_ms!=null&&data.frame_age_ms<500&&data.state!=='ERROR';
  image.style.opacity=fresh?'1':'.45';
  $('depthCameraState').textContent=fresh?'对齐 RGB':'等待新鲜深度帧';
  $('depthNear').textContent=`近 · ${depth.near_m.toFixed(2)} m`;
  $('depthFar').textContent=`远 · ${depth.far_m.toFixed(2)} m`;
}
function update(data) {
  latest = data; receivedAt = performance.now();
  updateIndexFollow(data);
  updateDepth(data);
  updateDepthVideo(data);
  updateHardware(data.hardware);
  updateCalibration(data.calibration,data);
  updateThumbCalibration(data.thumb_calibration,data);
  const human=data.human_structure;
  $('humanStructure').textContent=human?.message||'等待个人结构标定';
  if(human?.active){
    const held=Object.entries(human.fingers||{}).filter(([,q])=>data.depth?.algorithm==='d435-articulated-v2'?q.held:!q.valid||!q.side_observable).map(([name])=>fingers.find(([key])=>key===name)?.[1]);
    if(held.length)$('humanStructure').textContent+=' · 局部保持：'+held.join('、');
  }
  if(data.side_sway){
    const side=data.side_sway;
    const updated=(side.corrected||[]).map(name=>fingers.find(([key])=>key===name)?.[1]||name);
    $('humanStructure').textContent+=' · 侧摆：'+(updated.length?`深度修正 ${updated.join('、')}`:(side.message||'沿用 WiLoR'));
  }
  const check=data.motion_check;
  if(check?.active)$('motionCheckResult').textContent=`${check.active==='open_close'?'缓慢张合三次':'保持手形、小幅转掌'} · ${check.elapsed.toFixed(1)} / 10 秒`;
  else if(check?.result){
    const r=check.result;
    const ranges=fingers.map(([key,name])=>`${name}：侧摆 ${r.range_deg[key].abd?.toFixed(1)??'—'}°，MCP ${r.range_deg[key].mcp?.toFixed(1)??'—'}°，PIP ${r.range_deg[key].pip?.toFixed(1)??'—'}°`).join('；');
    $('motionCheckResult').textContent=`完成：可用帧 ${r.valid_frames}/${r.frames}（${(r.valid_ratio*100).toFixed(0)}%）。${ranges}。范围为 5–95 百分位变化量，是一致性指标，不是角度真值误差。`;
  }
  $('status').textContent = stateNames[data.state] || data.state;
  $('status').className = `status ${data.valid?'live':data.state==='ERROR'?'error':''}`;
  $('reason').textContent = reasonNames[data.reason] || data.reason;
  if(data.frame_diagnostics){
    const diagnostic=data.frame_diagnostics;
    const labels=diagnostic.detected_hands.map(h=>`${h.label}${h.score===null?'（类别）':` ${(h.score*100).toFixed(0)}%`}`).join('，')||'未检测到手';
    $('reason').textContent+=` · 检测：${labels} · 解算 ${data.mapping_ms.toFixed(0)} ms / 整帧 ${diagnostic.processing_ms.toFixed(0)} ms`;
    if(diagnostic.current_frame_held)$('reason').textContent+=' · 短暂漏检，保持上一有效目标';
    if(diagnostic.hand_label_held)$('reason').textContent+=' · 左右标签短暂变化，沿用同手跟踪';
    if(diagnostic.rejection_reason&&diagnostic.rejection_reason!==data.reason)$('reason').textContent+=' · 当前帧：'+(reasonNames[diagnostic.rejection_reason]||diagnostic.rejection_reason);
  }
  $('fps').textContent = data.fps.toFixed(1);
  $('inference').textContent = data.inference_ms.toFixed(1);
  $('age').textContent = data.age_ms===null?'—':Math.round(data.age_ms);
  $('mirror').textContent = `镜像：${data.mirror?'开':'关'}`;
  $('swap').textContent = `左右手校正：${data.swap_handedness?'开':'关'}`;
  for(const [finger] of fingers) {
    const raw=data.raw_angles?.[finger];
    $(finger+'-mcp').textContent=degree(raw?.mcp);
    $(finger+'-pip').textContent=degree(raw?.pip);
    $(finger+'-abd').textContent=degree(raw?.abd);
    $(finger+'-tabd').textContent=degree(data.joints[`left_${finger}_j0`]);
    const mcp=data.joints[`left_${finger}_j1`], pip=data.joints[`left_${finger}_j2`];
    $(finger+'-tmcp').textContent=degree(mcp); $(finger+'-tpip').textContent=degree(pip);
  }
  const held=data.held_joints || [];
  for(let i=0;i<4;i++) {
    $('thumb-raw-'+i).textContent=degree(data.raw_angles?.thumb?.['j'+i]);
    $('thumb-target-'+i).textContent=degree(data.joints[`left_thumb_j${i}`]);
    $('thumb-state-'+i).textContent=held.includes(`left_thumb_j${i}`)?'保持':data.valid?'更新':'等待';
  }
  $('thumbQuality').textContent=!data.valid?'等待有效跟随':data.thumb_reason?'拇指暂保持':Number.isFinite(data.thumb_fit_error)?`拟合残差 ${(data.thumb_fit_error*100).toFixed(1)}%`:'等待拇指数据';
  $('partStatus').textContent=held.length?`局部保持 ${held.length} 个关节；其余有效关节继续更新。弯曲较大时侧摆可能暂保持；拇指 IP 随 MCP 联动。`:'拇指按骨段方向与指尖位置拟合。拟合残差是归一化模型误差，不是识别置信度；IP 随 MCP 联动。';
  if(data.depth?.algorithm==='d435-articulated-v2'){
    const tip=data.thumb_diagnostics?.method==='depth_tip_analytic';
    $('thumbQuality').textContent=data.thumb_reason?'拇指暂保持 · 末端固定':tip?'指尖深度 · 解析角度':'近端方向回退';
    $('partStatus').textContent=`拇指优先由指尖深度定位根部三个关节；指尖不可见时退回近端方向。j3/j4 固定 ${data.depth.thumb_direct.fixed_terminal_deg.toFixed(1)}°。`+
      (data.thumb_diagnostics?.workspace_projected?' 当前指尖目标超出可达范围，已投影/限幅。':'');
  }
}
let hardwareRows=[];
let indexLease=false;
const followReasons={ready_disabled:'已连接，尚未使能',index_only_0_to_10_deg:'正在跟随，仅食指 MCP',
  sdk_hardware_fault:'SDK 报告硬件故障，已停止',sdk_disconnected:'SDK 与机械手连接断开，已停止',
  sdk_feedback_timeout:'机械手关节反馈超过 250 ms，已停止',camera_target_timeout:'摄像头目标未送达 SDK 超过 250 ms，已停止',
  full_hand_following:'正在跟随全手',pose_outside_start_envelope:'当前姿态超出接入范围',
  disable_not_confirmed:'失能尚未确认，请检查设备停止措施',invalid_full_hand_target:'全手目标无效，已停止',
  manual_or_source_stop:'已暂停并失能',feedback_source_timeout_or_fault:'目标/反馈超时或设备故障，已失能',
  session_30_second_limit:'本次 30 秒已结束，已失能',start_requires_fresh_target_feedback:'等待新鲜目标与反馈',
  start_requires_all_disabled:'开始前需五指均失能',index_pose_outside_start_envelope:'食指初始姿态超出接入范围',
  other_joint_or_finger_changed:'其他关节或使能状态变化，已停止',follow_command_rejected:'SDK 拒绝跟随指令，已失能'};
function updateIndexFollow(data){
  const config=data.index_follow, hw=data.hardware;
  $('indexFollowPanel').hidden=!config;if(!config)return;
  const f=hw?.follow;
  const full=!!config.full_hand;
  $('followTitle').textContent=full?`${data.tracker==='wilor'?'WiLoR · ':data.tracker==='wilor-d435'?'WiLoR + D435 · ':data.tracker==='d435'?'D435 · MediaPipe · ':''}全手摄像头跟随 · 真机`:'食指摄像头跟随 · 真机';
  $('followRange').textContent=full?'16 个独立目标 · SDK 原有速度参数 · 无运行时限':'仅 MCP · 0–10°';
  $('indexFollowStart').textContent=full?'开始全手跟随':'开始食指跟随';
  $('followInstructions').textContent=full?'先恢复识别并完整展示左手，再开始。四指屈伸 0–65°、侧摆 ±25°；拇指 j1 上限约 28.4°，结合实际 SDK 日志限位并预留 2° 余量。运行中拇指无效或越界优先保持拇指；单指屈伸短暂无效保持至 500 ms，持续失效再停止。无固定运行时限；丢手、心跳或反馈超时、故障仍失能。停止不自动回零。':'先恢复识别并举起左手，再开始。人手 0–65° 映射到真机 0–10°；其余四指不使能。丢手、页面心跳中断或反馈超时会失能；每次最多 30 秒。停止不自动回零。';
  indexLease=!!config.requested;
  const fresh=hw?.connected&&hw.feedback_age_ms<250&&hw.monitor_age_ms<250;
  const prefix=full?'全手':'食指';
  $('runtimeModeBadge').textContent=!fresh?prefix+'真机模式 · 反馈待确认':Object.values(hw.enabled||{}).some(Boolean)?prefix+'真机使能中':prefix+'真机模式 · 当前未使能';
  $('indexFollowStart').disabled=!!f?.active||f?.state==='FAULT'||hw?.fault_code!==0||!data.valid||!fresh;
  $('indexFollowStatus').textContent=fresh?(followReasons[f?.reason]||config.reason||f?.reason||'等待状态'):(hw?.connection_error_layer==='local_bridge'?'本地 SDK 桥通信异常；设备状态待确认':'SDK 反馈过期；设备状态待确认');
  if(fresh&&!config.requested&&config.reason)$('indexFollowStatus').textContent=config.reason;
  if(fresh&&f?.active&&f.thumb_range_held)$('indexFollowStatus').textContent='拇指目标被 SDK 限制：保持上次可用拇指姿态，四指继续跟随';
  if(data.limit_clipped_joints?.length)$('indexFollowStatus').textContent+=` · 软件限幅 ${data.limit_clipped_joints.length} 个关节`;
  if(fresh&&f?.reason?.startsWith('follow_command_rejected_code_'))$('indexFollowStatus').textContent='SDK 拒绝目标，已失能；错误码 '+f.reason.split('_').at(-1)+'，请保留日志';
  if(fresh && f?.motion_calls>0 && Number.isFinite(f?.command_deg))$('indexFollowStatus').textContent+=` · 最近指令 ${f.command_deg.toFixed(1)}° / 实际 ${degree(hw.positions?.left_index_j1)}°`;
}
async function indexFollowAction(action){
  try{
    const response=await fetch(`/api/index-follow/${action}`,{method:'POST'});
    const body=await response.json();if(!response.ok)throw Error(body.detail||'操作失败');
    if(action==='stop')indexLease=false;
  }catch(error){$('indexFollowStatus').textContent=error.message;}
}
$('indexFollowStart').onclick=()=>indexFollowAction('start');
$('indexFollowStop').onclick=()=>indexFollowAction('stop');
setInterval(()=>{if(indexLease&&!document.hidden&&!stopped)indexFollowAction('heartbeat');},200);
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){
    if(indexLease)indexFollowAction('stop');
    socket?.close();$('camera').removeAttribute('src');$('depthCamera').removeAttribute('src');
  }else if(!stopped){
    $('camera').src='/video';if(latest?.depth_video?.enabled)$('depthCamera').src='/depth-video';connect();
  }
});
function updateHardware(hw){
  $('hardwarePanel').hidden=!hw;if(!hw)return;
  const version=hw.version||{};
  $('hardwareMode').textContent=hw.control_capable?(hw.full_hand?'全手实机模式 · 需手动开始':'食指实机模式 · 需手动开始'):'只读 · 模拟发送';
  $('hardwareCommandHeader').textContent=hw.control_capable?'控制指令 °':'模拟命令 °';
  $('hardwareFootnote').textContent=hw.control_capable?(hw.full_hand?'此模式会使能五指并真实下发全手位置目标。使用 SDK 原有速度/加速度参数，不额外限速。保留识别滤波和故障停止；停止会失能，不自动回零。':'此模式点击开始后会真实驱动食指 MCP，控制范围 0–10°；其余四指不使能。停止会失能，不自动回零。'):'模拟命令仅显示和记录，不会发送；SDK 断开时可能失能。';
  $('hardwareSummary').textContent=`${hw.connected?'已连接左手':'未连接'} · SDK ${version.sdk_version||'—'} · 固件 ${version.hand_firmware_version||'—'} · 本进程运动调用 ${hw.motion_calls??0} 次`;
  if(hw.control_capable&&!hw.motion_calls)$('hardwareSummary').textContent+=' · 尚未发送运动指令';
  const enabled=Object.values(hw.enabled||{}).filter(Boolean).length;
  const outside=hw.dry_run?.outside_model||[];
  $('hardwareReason').textContent=(hw.dry_run?.reason||hw.reason||'等待状态')+
    `；使能 ${enabled}/5 指`+(Number.isFinite(hw.feedback_age_ms)?`；反馈年龄 ${hw.feedback_age_ms.toFixed(0)} ms`:'')+
    (outside.length?`；${outside.length} 个反馈超出模型限位容差，后续运动前需核对零位`:'');
  if(hw.joint_layout && hardwareRows.length===0){
    hardwareRows=hw.joint_layout.map(({id,name})=>{
      const row=document.createElement('tr');
      const cells=Array.from({length:4},()=>row.appendChild(document.createElement('td')));
      cells[0].textContent=`${id} · ${name.replace('left_','')}`;
      $('hardwareAngles').appendChild(row);return {id,name,cells};
    });
  }
  const fresh=hw.connected&&hw.feedback_age_ms<=250&&hw.monitor_age_ms<=250;
  const commands=new Map((hw.dry_run?.commands||[]).map(c=>[c.id,c.position]));
  if(hw.full_hand&&Array.isArray(hw.follow?.commands))hw.follow.commands.forEach((value,id)=>commands.set(id,value));
  else if(hw.control_capable&&Number.isFinite(hw.follow?.command_deg))commands.set(6,hw.follow.command_deg*Math.PI/180);
  for(const {id,name,cells} of hardwareRows){
    cells[1].textContent=degree(hw.preview_targets?.[name]);
    cells[2].textContent=hw.control_capable&&!hw.motion_calls?'—':degree(commands.get(id));
    cells[3].textContent=fresh?degree(hw.positions?.[name]):'—';
  }
}
const thumbCalNames={rest:'自然放松',plane:'沿掌面展开与收回',lift:'向镜头抬起与回落',flex:'末端弯曲与伸直',
  index:'碰食指尖后返回',middle:'碰中指尖后返回',ring:'碰无名指尖后返回',pinky:'碰小指尖后返回'};
const thumbCalInstructions={
  rest:'掌心朝前，四指自然展开，拇指舒适放松；准备 2 秒后保持不动 4 秒。',
  plane:'手掌不转动，拇指沿掌面缓慢展开、向食指侧收回，再展开；在采样 12 秒内做 2–3 次完整往返。',
  lift:'掌心朝前，将拇指向摄像头方向抬起、再回落到掌面附近；缓慢往返 2–3 次，不要用转手腕代替。',
  flex:'拇指根部尽量不动，主要弯曲末端两节再伸直；缓慢往返 2–3 次，不要求完全独立，只做舒适动作。',
  index:'从自然展开出发，拇指与食指尖轻触后返回展开；缓慢完成 2–3 次。真实接触由你确认，不用力挤压。',
  middle:'从自然展开出发，拇指与中指尖轻触后返回展开；缓慢完成 2–3 次，保持两指尖可见。',
  ring:'从自然展开出发，拇指与无名指尖轻触后返回展开；缓慢完成 2–3 次，允许无名指自然弯曲。',
  pinky:'从自然展开出发，拇指与小指尖轻触后返回展开；缓慢完成 2–3 次，不勉强撑开或扭动手掌。'};
let lastThumbCalCompleted='';
let thumbCalMode='legacy',lastStaticCal=null,lastStaticRefKey='';
const staticThumbNames={rest:'自然展开',index_base:'向食指根部收拢',pinky_side:'向小指方向收拢',lift:'朝镜头抬起'};
const staticThumbInstructions={rest:'掌心朝前，四指自然展开，拇指舒适展开并保持不动。',
 index_base:'手掌保持不动，拇指向食指根部收拢，保持指尖可见；不要求接触。',
 pinky_side:'拇指向小指根部方向舒适收拢，指尖不要被其他手指盖住；不勉强触碰。',
 lift:'手掌不转动，将拇指朝镜头方向抬起并静止，形成与掌面内姿态不同的深度。'};
function updateStaticReferences(cal){
 const step=$('thumbCalStep').value,q=cal.references_deg?.[step];if(!q)return;
 const key=JSON.stringify([step,q]);if(key===lastStaticRefKey)return;lastStaticRefKey=key;
 q.forEach((v,i)=>{const input=$('thumbRef'+i);input.value=v;input.min=cal.bounds_deg[i][0];input.max=cal.bounds_deg[i][1];});
}
$('thumbCalStep').onchange=()=>{$('thumbCalInstruction').textContent=thumbCalInstructions[$('thumbCalStep').value];};
function updateThumbCalibration(cal,data){
  if(!cal)return;
  $('thumbCalibrationPanel').hidden=cal.available===false||data.tracker==='wilor'||data.calibration?.finger_policy==='wilor_three_pose';
  if(cal.available===false){$('calReset').disabled=!!data.calibration?.collecting;return;}
  const isStatic=cal.kind==='depth_thumb_static',names=isStatic?staticThumbNames:thumbCalNames;
  $('modelPoseMode').textContent=isStatic&&cal.reference_preview?'URDF · 拇指参考预览（不发送）':'URDF · LEFT';
  if(thumbCalMode!==(isStatic?'static':'legacy')){
    thumbCalMode=isStatic?'static':'legacy';lastThumbCalCompleted='';lastStaticRefKey='';
    $('thumbCalStep').replaceChildren(...Object.entries(names).map(([key,name],i)=>{const option=document.createElement('option');option.value=key;option.textContent=`${i+1} · ${name}`;return option;}));
    $('thumbCalStep').value='rest';
    $('thumbCalStep').onchange=()=>{
      $('thumbCalInstruction').textContent=(isStatic?staticThumbInstructions:thumbCalInstructions)[$('thumbCalStep').value];
      if(isStatic&&lastStaticCal)updateStaticReferences(lastStaticCal);
    };
    $('thumbCalStep').onchange();
  }
  $('thumbStaticReferences').hidden=!isStatic;$('thumbStaticReset').hidden=!isStatic;
  $('thumbCalTitle').textContent=isStatic?'拇指 · 四位置静态标定':'拇指专项 · 连续运动标定';
  $('thumbCalHelp').textContent=isStatic?'复用当前两步骨架配置，每步准备 1 秒、静止采样 3 秒。先查看对应机械手参考姿态，可调整根部三个角度。参考预览和采样会暂停并确认失能，参考姿态只在模型中显示。':'自然放松保持不动，其余每步缓慢往返 2–3 次；保持手掌朝前，采集和应用均暂停并失能。';
  if(isStatic){lastStaticCal=cal;updateStaticReferences(cal);
    $('thumbReferenceHint').textContent=cal.reference_preview?`三维模型当前显示“${names[cal.selected]||''}”参考姿态，角度表/SDK 目标未被替换。选择下一步后点“查看 / 更新参考姿态”；恢复识别可退出参考预览。`:'三维模型当前显示识别跟随目标。';
    $('thumbCalFootnote').textContent='只保存指尖相对位置、参考角度和统计，基础骨架不变。四位置校正方向、尺度与耦合；远超采样范围平滑退回基础映射。末端仍固定，不能通过标定恢复独立末端屈伸。';
  }
  const busy=!!cal.collecting,baseBusy=!!data.calibration?.collecting;
  $('thumbCalMode').textContent=cal.enabled?'专项参考已启用':cal.has_profile?(cal.compatible?'专项参考已停用':'基础标定已改变，需重采'):'尚未应用专项参考';
  if(isStatic&&cal.reference_preview)$('thumbCalMode').textContent+=' · 模型参考预览';
  $('thumbCalSteps').replaceChildren(...Object.entries(names).map(([key,name])=>{
    const tag=document.createElement('span');tag.className=cal.samples[key]?'done':'';
    tag.textContent=`${cal.samples[key]?'✓ ':''}${name}`;return tag;
  }));
  const completed=Object.keys(cal.samples),signature=completed.join(',');
  if(signature!==lastThumbCalCompleted&&!busy){
    lastThumbCalCompleted=signature;const next=Object.keys(names).find(k=>!cal.samples[k]);
    if(next){$('thumbCalStep').value=next;$('thumbCalStep').onchange();}
  }
  $('thumbCalProgress').value=busy?cal.progress:completed.length/Object.keys(names).length;
  const rejection=Object.entries(cal.rejections).sort((a,b)=>b[1]-a[1])[0];
  $('thumbCalMessage').textContent=busy?`${names[cal.collecting]} · ${cal.phase==='prepare'?'准备中':'采样中'} · 有效 ${cal.valid_frames} 帧`+
    (rejection?` · 暂未采入：${rejection[0]}`:''):cal.message;
  $('thumbCalCapture').disabled=busy||baseBusy||!data.calibration?.enabled;
  $('thumbCalStep').disabled=busy||baseBusy;
  $('thumbCalCancel').disabled=!busy;
  $('thumbCalApply').disabled=!cal.ready||busy||baseBusy;
  $('thumbCalToggle').disabled=!cal.has_profile||busy||baseBusy||!cal.compatible;
  $('thumbCalToggle').textContent=cal.enabled?'对比：停用专项参考':'对比：启用专项参考';
  if(isStatic){$('thumbCalPreview').disabled=busy||baseBusy||!data.calibration?.enabled;$('thumbStaticReset').disabled=busy||baseBusy;
    for(let i=0;i<3;i++)$('thumbRef'+i).disabled=busy||baseBusy;}
  if(busy){$('resume').disabled=true;$('calCapture').disabled=true;$('calApply').disabled=true;$('calToggle').disabled=true;$('calReset').disabled=true;$('indexFollowStart').disabled=true;}
  else $('calReset').disabled=baseBusy;
}
for(const [id,name] of Object.entries({thumbCalCapture:'capture',thumbCalCancel:'cancel',thumbCalApply:'apply',thumbCalToggle:'toggle'})){
  $(id).onclick=async()=>{
    $(id).disabled=true;
    try{
      const r=await fetch(`/api/thumb-calibration/${name}`,{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({step:$('thumbCalStep').value,backend:previewBackend,
          ...(thumbCalMode==='static'&&name==='capture'?{reference_deg:[0,1,2].map(i=>Number($('thumbRef'+i).value))}:{})})});
      const body=await r.json();if(!r.ok)throw Error(body.detail||'专项标定操作失败');
    }catch(error){$('thumbCalMessage').textContent=error.message;}
  };
}
for(const [id,name] of Object.entries({thumbCalPreview:'preview',thumbStaticReset:'reset'})){
 $(id).onclick=async()=>{try{
  const response=await fetch(`/api/thumb-calibration/${name}`,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({step:$('thumbCalStep').value,reference_deg:[0,1,2].map(i=>Number($('thumbRef'+i).value))})});
  const body=await response.json();if(!response.ok)throw Error(body.detail||'操作失败');
 }catch(e){$('thumbCalMessage').textContent=e.message;}};
}
async function calibrationAction(name){
  try{
    const response=await fetch(`/api/calibration/${name}`,{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({pose:$('calPose').value,backend:previewBackend})});
    const body=await response.json();
    if(!response.ok)throw Error(body.detail||'标定操作失败');
  }catch(error){$('calMessage').textContent=error.message;}
}
for(const [id,actionName] of Object.entries({calCapture:'capture',calCancel:'cancel',calApply:'apply',calToggle:'toggle',calReset:'reset'})){
  $(id).onclick=()=>calibrationAction(actionName);
}
$('sideZeroCapture').onclick=async()=>{
  try{const r=await fetch('/api/calibration/capture',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({pose:'side_zero'})});const body=await r.json();if(!r.ok)throw Error(body.detail);}
  catch(error){$('sideZeroMessage').textContent=error.message;}
};
$('sideZeroApply').onclick=()=>calibrationAction('apply');
for(const [id,mode] of Object.entries({checkOpenClose:'open_close',checkPalm:'palm_turn',checkCancel:'cancel'})){
  $(id).onclick=async()=>{
    try{const response=await fetch(`/api/validation/${mode}`,{method:'POST'});const body=await response.json();if(!response.ok)throw Error(body.detail);if(mode==='cancel')$('motionCheckResult').textContent='已取消检查';}
    catch(error){$('motionCheckResult').textContent=error.message;}
  };
}
function connect() {
  if(stopped||document.hidden||socket?.readyState===WebSocket.OPEN||socket?.readyState===WebSocket.CONNECTING) return;
  socket = new WebSocket(`ws://${location.host}/ws`);
  receivedAt=performance.now();
  const connection=socket;
  connection.onmessage=e=>{
    if(socket!==connection)return;
    receivedAt=performance.now();
    try{
      const data=comparisonPreview(JSON.parse(e.data));latest=data;
      if(receivedAt-lastUiAt>=95){lastUiAt=receivedAt;update(data);}
    }catch(error){$('reason').textContent=`页面更新异常：${error.message}；请刷新页面`;console.error(error);}
  };
  connection.onclose=()=>{
    if(socket!==connection)return;
    socket=null;
    if(!stopped&&!document.hidden){$('status').textContent='连接中断 · 正在重连';$('status').className='status error';if(latest?.hardware)updateHardware({connected:false,dry_run:{reason:'页面数据连接中断',commands:[]}});setTimeout(connect,1500);}
  };
}
function reconnectStream(){
  const previous=socket;socket=null;previous?.close();
  receivedAt=performance.now();connect();
  if(!document.hidden)$('camera').src='/video?reconnect='+Date.now();
  if(!document.hidden&&latest?.depth_video?.enabled)$('depthCamera').src='/depth-video?reconnect='+Date.now();
}
function clearDepthPreview(){
  if(latest?.depth_video?.enabled){$('depthCameraState').textContent='深度数据中断';$('depthCamera').style.opacity='.45';}
  if(latest?.depth)updateDepth({depth:{...latest.depth,state:'STALE',age_ms:1000,
    hand_tracking:{state:'STALE',hands:[],message:'页面数据中断，已清空骨架'},control_gate:'等待新鲜 D435 数据'}});
}
async function action(name) {
  try {
    const response=await fetch(`/api/control/${name}?backend=${previewBackend}`,{method:'POST'});
    if(!response.ok) throw Error(`操作失败 (${response.status})`);
    if(name==='resume'&&(!receivedAt||performance.now()-receivedAt>500||socket?.readyState!==WebSocket.OPEN))reconnectStream();
    if(name==='stop') {
      clearDepthPreview();
      stopped=true; socket?.close(); $('camera').removeAttribute('src');$('depthCamera').removeAttribute('src');
      $('status').textContent='服务已停止';$('status').className='status';
      $('reason').textContent='摄像头正在释放；重新运行启动脚本可再次打开';
      for(const id of ['pause','resume','mirror','swap','stop']) $(id).disabled=true;
    }
  } catch(error) {$('reason').textContent=error.message;}
}
for(const id of ['pause','resume','mirror','swap','stop']) $(id).onclick=()=>action(id);
$('resetView').onclick=()=>frameView();
setInterval(()=>{
  if(!stopped && !document.hidden && receivedAt && performance.now()-receivedAt>500){
    clearDepthPreview();
    $('status').textContent='数据中断 · 正在重连';$('status').className='status error';
    if(latest?.hardware)updateHardware({connected:false,dry_run:{reason:'页面数据超时，实际值与模拟命令已隐藏',commands:[]}});
    if(performance.now()-receivedAt>2000)reconnectStream();
  }
},200);
loadModel().catch(error=>{$('modelMessage').textContent=error.message;$('modelMessage').style.color='#ffaaaa';console.error(error);});
if(document.hidden)$('camera').removeAttribute('src');
connect();
