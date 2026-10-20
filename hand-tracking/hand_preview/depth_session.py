"""D435 source through personal calibration, URDF retargeting and SDK follower."""
from copy import deepcopy
import json
import threading
import time
import cv2
import numpy as np
from .mapping_app import MappingSession,ROOT
from .depth_source import Camera,camera_context,landmark_result,validate_reference,reference_id
from .calibration import model_hash
from .finger_filter import FingerAngleFilter
from .depth_articulation import ArticulatedDepthEstimator,DirectThumb,thumb_features
from .thumb_tip import TipDepthThumb
from .depth_simple_calibration import SimpleDepthCalibration,VERSION,DEFAULT_THUMB
from .thumb_static import StaticThumbCalibration
from .geometry import FINGERS
from .calibration import correct_fingers


class DepthMappingSession(MappingSession):
    shutdown_timeout=7
    def __init__(self,args,profile_path=None):
        path=profile_path or ROOT/'profiles'/'d435-articulated.json'
        super().__init__(args,profile_path=path,tracker='d435')
        # Preview must use the same known bounds as full-hand SDK mode.
        if not args.full_hand_follow:
            limit_profile=json.loads((ROOT/'profiles'/'full-hand-limits.json').read_text())
            if limit_profile.get('preview_sha256')!=model_hash():raise ValueError('SDK 限位缓存与当前 URDF 不匹配，请先重新生成限位')
            limits=limit_profile['bounds']
            for name,pair in limits.items():
                self.mapper.bounds[name]=tuple(pair)
                self.mapper.specs[name]['lower'],self.mapper.specs[name]['upper']=pair
                self.mapper.positions[name]=float(np.clip(self.mapper.positions[name],*pair))
            self.mapper.responsive=True;self.mapper._reset_motion()
        self.profile_path=path;self.depth_bound=False;self.depth_context=None
        self.finger_filter=FingerAngleFilter()
        self.front=ArticulatedDepthEstimator(vision_prior=True,depth_root_correction=True)
        self.thumb=TipDepthThumb(self.robot,self.mapper.bounds,deepcopy(DEFAULT_THUMB))
        self.depth_camera=Camera(getattr(args,'depth_serial',None),track_hands=True)
        self.depth_camera.articulated_hands=True
        self.depth_status={'state':'STARTING','hand_tracking':{'enabled':True,'hands':[]}}
        self.depth_reference_key=None
        self.calibration=SimpleDepthCalibration(self.robot,self.mapper.bounds,{'tracker':'d435','pending_camera':True},path=path)
        self.thumb_calibration=StaticThumbCalibration(self.calibration,self.robot,self.mapper.bounds)

    def gate_reason(self):
        if not self.depth_bound:return '等待 D435 相机身份和内参'
        if not self.calibration.enabled or not self.calibration.profile:return '请完成并启用 D435 两步骨架标定后开始真机跟随'
        if self.calibration.collecting or self.thumb_calibration.collecting:return '标定期间保持失能'
        if self.thumb_calibration.reference_preview:return '当前为拇指参考姿态预览，请恢复识别后再开始实机'
        try:validate_reference(self.calibration.profile.get('depth_reference'))
        except ValueError as exc:return str(exc)
        report=self.depth_status.get('hand_tracking',{});hands=report.get('hands',[])
        if not hands or not hands[0].get('fused_points_m'):return report.get('message','等待可靠 D435 深度骨架')
        h=hands[0]
        if not h.get('personal_reference'):return '等待个人米制骨长参考进入融合解算'
        if h.get('reference_id')!=reference_id(self.calibration.profile['depth_reference']):
            return '等待当前标定的米制骨长进入融合解算'
        if h.get('anchor_age_ms',1000)>250:return 'D435 深度锚点已过期'
        return None

    def hardware_source(self):
        with self.lock:
            source=self.mapper.snapshot(time.perf_counter())
            reason=self.gate_reason()
            if reason:source.update(valid=False,reason=reason)
            return source

    def snapshot(self):
        data=super().snapshot()
        with self.lock:
            status=deepcopy(self.depth_status)
            data['depth']={'state':status.get('state'),'error':status.get('error'),
                           'fps':status.get('fps'),'age_ms':status.get('age_ms'),
                           'timestamp_skew_ms':status.get('timestamp_skew_ms'),
                           'device':status.get('device'),'camera_released':status.get('camera_released'),
                           'hand_tracking':status.get('hand_tracking'),
                           'bound':self.depth_bound,'control_gate':self.gate_reason(),
                           'algorithm':VERSION,'thumb_direct':deepcopy(self.thumb.config),
                           'thumb_terminal_bounds_deg':np.degrees(self.mapper.bounds['left_thumb_j3']).tolist()}
            if (self.front.last_geometry is not None and self.front.at is not None
                    and 0<=time.perf_counter()-self.front.at<.25 and status.get('age_ms',1000)<500):
                hands=data['depth']['hand_tracking'].get('hands',[])
                if hands:hands[0]['articulated_points_m']=self.front.last_geometry.tolist()
        return data

    def on_control(self,action):
        if action=='resume':
            self.finger_filter.reset()
            self.front.reset();self.thumb.reset()
            self.thumb_calibration.reference_preview=False;self.thumb_calibration.revision+=1
            if hasattr(self,'preview_thumb_zero'):del self.preview_thumb_zero
            self.depth_camera.paused=False;self.depth_camera.reset_hands.set()
        elif action=='swap':
            self.depth_context=None;self.depth_bound=False
        elif action=='stop':self.depth_camera.stop.set()

    def bind_camera(self,status):
        context=camera_context(status,self.swap)
        context['solver']=VERSION
        if context==self.depth_context:return
        if self.follow and self.follow.snapshot()['requested']:
            self.follow.stop_confirmed()
        with self.lock:
            if not self.mapper.paused:self.mapper.resume()
            self.last_wrist=self.last_selected=None
            self.finger_filter.reset()
            self.front.reset()
            self.calibration=SimpleDepthCalibration(self.robot,self.mapper.bounds,context,path=self.profile_path)
            config=self.calibration.profile['thumb_direct'] if self.calibration.profile else DEFAULT_THUMB
            self.thumb=TipDepthThumb(self.robot,self.mapper.bounds,deepcopy(config))
            self.thumb_calibration=StaticThumbCalibration(self.calibration,self.robot,self.mapper.bounds)
            self.depth_context=context;self.depth_bound=True;self.depth_reference_key=None

    def on_calibration(self,action,pose):
        if action in ('capture','apply','toggle','reset'):
            # Do not let an automatic reference learned in an arbitrary pose
            # prevent a new operator from collecting a clean neutral reference.
            self.depth_camera.reset_hands.set();self.depth_reference_key=None
            self.finger_filter.reset()
            self.front.reset();self.thumb.reset()
            self.thumb.config=deepcopy(self.calibration.profile['thumb_direct'] if self.calibration.profile else DEFAULT_THUMB)

    def select_angles(self,result,swap,now):
        hand=getattr(result,'depth_hand',None)
        self.detected_hands=[] if hand is None else [{'label':hand['label'],'score':hand.get('score')}]
        label=None if hand is None else hand['label']
        if swap and label:label='Right' if label=='Left' else 'Left'
        reason=None
        if hand is None:reason=getattr(result,'backend_error',None) or 'No hand detected'
        elif label!='Left':reason='Only right hand detected; check handedness setting'
        elif hand.get('score',0)<.65:reason='Left label score below 0.65'
        if reason:
            self.calibration.reject(reason,now);self.calibration.advance(now)
            self.thumb_calibration.reject(reason,now);self.thumb_calibration.advance(now)
            return None,None,reason
        pixels=np.asarray(hand['pixels'],float)
        size=np.array([self.args.width,self.args.height])
        if not np.isfinite(pixels).all() or (pixels<2).any() or (pixels>size-3).any():
            reason='请将整只手完整放入画面';self.calibration.reject(reason,now);self.calibration.advance(now)
            self.thumb_calibration.reject(reason,now);self.thumb_calibration.advance(now)
            return None,None,reason
        profile=self.calibration.profile if self.calibration.enabled else None
        if self.calibration.collecting:
            try:
                p,basis,rays=self.front.calibration_geometry(hand,now)
                obs=hand.get('observations',[])
                # Straight calibration is the one phase where visible joint
                # surface depth helps measure actual metric bone lengths.
                # Projecting every point onto a plane would bias lengths even
                # for a few degrees of natural MCP flexion.
                for i in range(4,21):
                    if i<len(obs) and obs[i].get('valid'):
                        z=obs[i]['z_m']+.006
                        if abs(z-p[i,2])<.025:p[i]=rays[i]*z
                straight=True
                for ids in FINGERS.values():
                    segments=np.diff(p[list(ids)],axis=0)@basis
                    if np.any(segments[:,1]<.002):straight=False
                    directions=segments/np.maximum(np.linalg.norm(segments,axis=1)[:,None],1e-6)
                    if np.any(np.sum(directions[:-1]*directions[1:],axis=1)<.75):straight=False
                thumb_ok=True
                for i in (1,2,3):
                    if i<len(obs) and obs[i].get('valid'):
                        z=obs[i]['z_m']+.006
                        if abs(z-p[i,2])<.075:p[i]=rays[i]*z
                        else:thumb_ok=False
                    else:thumb_ok=False
                observation={'metric_points_m':p.tolist(),'basis':basis.tolist(),
                    'sync_valid':hand['sync_valid'],'palm_depth_points':self.front.plane_count,
                    'straight':straight,'thumb_reliable':thumb_ok,
                    'thumb_features':(self.calibration.samples['neutral']['thumb_zero']
                        if self.calibration.collecting['pose']=='together' else thumb_features(p,basis).tolist())}
                self.calibration.ingest({},p,label,now,observation)
            except ValueError as exc:self.calibration.reject(str(exc),now);self.calibration.advance(now)
            return None,hand['score'],'正在采集两步骨架标定，实机保持失能'
        angles,clean,basis,thumb_ok=self.front.estimate(hand,now,profile)
        self.thumb_calibration.ingest_depth(clean,basis,now,self.front.report,hand)
        for f in FINGERS:
            if angles[f]['abd'] is not None:angles[f]['abd']*=self.robot.abduction_signs[f]
        self.uncalibrated_angles=deepcopy(angles)
        angles=correct_fingers(angles,profile)
        angles=self.finger_filter.update(angles,now)
        reference=profile['skeleton']['thumb_zero'] if profile else None
        # An uncalibrated preview uses a persistent first-frame reference.
        if reference is None and thumb_ok:
            if not hasattr(self,'preview_thumb_zero'):self.preview_thumb_zero=thumb_features(clean,basis).tolist()
            reference=self.preview_thumb_zero
        tip_ok=self.front.report.get('thumb_tip_reliable',False) and profile is not None
        if tip_ok:
            angles['thumb']=self.thumb.calculate_tip(clean,basis,profile['skeleton'],now,
                static_model=self.thumb_calibration.active_model())
        else:angles['thumb']=self.thumb.calculate(clean,basis,reference,now,thumb_ok)
        self.uncalibrated_angles['thumb']=deepcopy(angles['thumb'])
        self.thumb_reason=None if tip_ok or thumb_ok else '拇指指尖和近端深度暂不可用，保持根部；末端固定'
        self.contact_cues=None
        if not any(v is not None for f in FINGERS for v in angles[f].values()):
            return None,None,'四指关节链暂不可观测'
        self.last_wrist=pixels[0]/size;self.last_selected=now
        return angles,float(hand['score']),'四指 RGB 三维骨架与指尖深度拇指'

    def sync_reference(self):
        with self.lock:reference=deepcopy(self.calibration.reference())
        key=json.dumps(reference,sort_keys=True)
        if key!=self.depth_reference_key:
            with self.depth_camera.lock:
                self.depth_camera.hand_reference=reference;self.depth_camera.reference_changed.set()
            self.depth_reference_key=key

    def worker(self):
        worker=threading.Thread(target=self.depth_camera.run,name='d435-source',daemon=True)
        worker.start();last=-1
        try:
            while not self.stop.wait(.005):
                status=self.depth_camera.snapshot()
                with self.lock:
                    self.depth_status=status
                    self.error=status.get('error')
                if status['state']!='RUNNING':continue
                self.bind_camera(status);self.sync_reference()
                if status['frames']==last:continue
                last=status['frames'];intr=status['intrinsics']
                self.args.width,self.args.height=intr['width'],intr['height']
                report=status['hand_tracking'];result=landmark_result(report,self.args.width,self.args.height)
                now=time.perf_counter();acquired=now-status['age_ms']/1000
                with self.lock:
                    begin=time.perf_counter()
                    try:
                        if getattr(result,'backend_error',None):
                            angles,score,reason=None,None,result.backend_error
                            self.detected_hands=[{'label':h['label'],'score':h.get('score')} for h in report.get('hands',[])]
                            self.calibration.reject(reason,now);self.thumb_calibration.reject(reason,now)
                        else:angles,score,reason=self.select_angles(result,self.swap,now)
                    except ValueError as exc:
                        angles,score,reason=None,None,str(exc)
                        self.calibration.reject(reason,now);self.thumb_calibration.reject(reason,now)
                    mapped=time.perf_counter()
                    self.mapping_ms=(mapped-begin)*1000;self.processing_ms=(mapped-acquired)*1000
                    self.rejection_reason=reason if angles is None else ('Frame processing exceeded 200 ms' if self.processing_ms>200 else None)
                    if self.rejection_reason:self.rejected_frames+=1
                    else:self.accepted_frames+=1
                    self.mapper.update(angles,acquired,mapped,reason)
                    self.motion_check.ingest(angles,self.mapper.snapshot(mapped)['valid'],mapped)
                    self.hand_score=score;self.fps=status['fps'];self.inference_ms=report.get('processing_ms',0.)
                    self.frame_id+=1;self.frame_at=acquired
                    mirror=self.mirror
                # Display-only mirroring preserves the RGB/depth panel order.
                with self.depth_camera.lock:jpeg=self.depth_camera.jpeg
                if jpeg and mirror:
                    image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                    if image is not None:
                        half=image.shape[1]//2
                        _,encoded=cv2.imencode('.jpg',np.hstack((cv2.flip(image[:,:half],1),cv2.flip(image[:,half:],1))))
                        jpeg=encoded.tobytes()
                with self.lock:self.jpeg=jpeg
        except Exception as exc:
            with self.lock:self.error='D435 链路异常：'+str(exc);self.mapper.pause()
            print(self.error,flush=True)
        finally:
            self.depth_camera.stop.set();worker.join(6)
            with self.lock:self.depth_bound=False
            print('D435 fusion source released.',flush=True)
