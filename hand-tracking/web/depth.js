const $=id=>document.getElementById(id);
const chains=[[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20]];
let yaw=0,pitch=-.25,hand=null,initialized=false,articulated=false;
function draw(id,points,sources){
 const canvas=$(id),ctx=canvas.getContext('2d');ctx.clearRect(0,0,480,320);
 if(!points){ctx.fillStyle='#9ab4ca';ctx.fillText('等待新鲜三维骨架',20,35);return;}
 const root=points[0],c=Math.cos(yaw),s=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
 const p=points.map(v=>{const x=v[0]-root[0],y=v[1]-root[1],z=v[2]-root[2];return [240+1400*(x*c+z*s),245+1400*(y*cp-(-x*s+z*c)*sp)];});
 ctx.strokeStyle='#8295a5';ctx.lineWidth=3;
 for(const chain of chains){ctx.beginPath();chain.forEach((i,j)=>ctx[j?'lineTo':'moveTo'](...p[i]));ctx.stroke();}
 p.forEach((xy,i)=>{ctx.beginPath();ctx.arc(...xy,4,0,2*Math.PI);ctx.fillStyle=sources?.[i]==='depth_band'?'#7ce650':'#ffbe28';ctx.fill();});
}
function redraw(){draw('depthModel',hand?.model_points_m);draw('depthFused',articulated?hand?.articulated_points_m:hand?.fused_points_m,hand?.sources);}
export function updateDepth(data){
 const d=data.depth,panel=$('depthPanel');if(!panel)return;panel.hidden=!d;document.body?.classList.toggle('depth-mode',!!d);if(!d)return;
 articulated=d.algorithm==='d435-articulated-v2';
 document.title=data.index_follow?'D435 · 全手跟随':'D435 · 标定与模型预览';
 $('workspaceTitle').textContent=document.title;$('cameraMode').textContent='RGB ＋ 对齐深度';
 $('runtimeFooter').textContent='图像在本机处理，默认不录制。关闭网页不停止后台；页面“停止服务”关闭识别并请求失能。结束使用请运行 Stop-DepthFollow.cmd，一并停止相机、SDK 桥与专用容器；不自动回零。';
 if(!initialized){
  for(const id of ['depthModel','depthFused']){let drag=null;$(id).onpointerdown=e=>{drag=[e.clientX,e.clientY];$(id).setPointerCapture(e.pointerId);};$(id).onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.01;pitch+=(e.clientY-drag[1])*.01;drag=[e.clientX,e.clientY];redraw();};$(id).onpointerup=$(id).onpointercancel=()=>drag=null;}
  initialized=true;
 }
 const report=d.hand_tracking||{};hand=d.age_ms!=null&&d.age_ms<500?report.hands?.[0]:null;
 const direct=$('directThumbPanel');
 if(direct){direct.hidden=d.algorithm!=='d435-articulated-v2';
  if(!direct.hidden){
   const fixed=$('thumbFixed');
   if(document.activeElement!==fixed)fixed.value=d.thumb_direct.fixed_terminal_deg;
   const limit=d.thumb_terminal_bounds_deg||[-18,78];fixed.min=limit[0];fixed.max=limit[1];
   $('thumbFixedSave').disabled=!data.calibration?.has_profile||!!data.calibration?.collecting;
   $('thumbFixedSave').onclick=async()=>{
    try{const r=await fetch('/api/depth-config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({fixed_terminal_deg:Number(fixed.value)})});
     const body=await r.json();if(!r.ok)throw Error(body.detail||'保存失败');$('thumbFixedMessage').textContent='已保存并暂停；恢复识别后检查预览，再手动开始跟随。';
    }catch(e){$('thumbFixedMessage').textContent=e.message;}
   };
  }
 }
 $('depthMode').textContent=`${d.state} · USB ${d.device?.usb||'—'}`;
 const q=hand?.quality;
 $('depthStatus').textContent=d.error||`${report.state||'等待'} · ${report.message||''} · 深度 ${q?.depth_constraints??0}/21 点 · 骨长 P95 ${q?.bone_error_p95_mm?.toFixed(1)??'—'} mm · 彩深差 ${d.timestamp_skew_ms?.toFixed(1)??'—'} ms · ${hand?.personal_reference?'个人米制骨长':'自动骨长参考'}`;
 if(d.algorithm==='d435-articulated-v2'&&!d.error){
  const fingers=Object.values(data.human_structure?.fingers||{}),errors=fingers.map(f=>f.image_error_px).filter(Number.isFinite);
  const source={regional:'区域掌面',palm_keys:'掌部关键点',wrist_depth:'新鲜腕深度＋已有朝向'}[data.human_structure?.palm_source]||'等待掌面';
  $('depthStatus').textContent=`${report.state||'等待'} · 深度邻域 ${q?.depth_constraints??0}/21 · ${source} ${data.human_structure?.palm_depth_points??0} 点 · 投影误差 ${errors.length?Math.max(...errors).toFixed(1):'—'} px · 彩深差 ${d.timestamp_skew_ms?.toFixed(1)??'—'} ms · ${data.calibration?.enabled?'个人固定骨长':'请完成两步标定'}`;
  if(data.human_structure?.finger_solver==='rgb_model_3d'){
   $('depthStatus').textContent=`${report.state||'等待'} · 四指 RGB 三维骨架直接解算 · ${source} ${data.human_structure?.palm_depth_points??0} 点 · 拇指沿用指尖深度及静态标定 · 彩深差 ${d.timestamp_skew_ms?.toFixed(1)??'—'} ms`;
  }
  if(data.human_structure?.finger_solver==='rgb_depth_root_correction'){
   const c=data.human_structure.root_correction||{};
   const used=Object.values(c.fingers||{}).filter(f=>f.near_depth_used).length;
   $('depthStatus').textContent=`${report.state||'等待'} · RGB 为主／深度根部弱修正 · 掌面修正 ${(c.palm_correction_deg||0).toFixed(1)}° · ${c.palm_used?'可靠掌面':'退回视觉'} · 近节深度 ${used}/4 指 · 拇指标定保留`;
  }
 }
 $('depthGate').textContent=d.control_gate?`真机启动条件：${d.control_gate}`:(data.index_follow?'标定和深度条件已满足，设备仍需手动开始。':'当前为仅预览模式；实机入口：Start-DepthFollow.cmd。');
 const names=['拇指','食指','中指','无名指','小指'];
 $('depthPerFinger').textContent=chains.map((ids,i)=>`${names[i]}：${ids.slice(1).filter(j=>hand?.sources?.[j]==='depth_band').length}/4 点有深度`).join(' · ');
 if(data.index_follow&&d.control_gate)$('indexFollowStart').disabled=true;
 redraw();
}
