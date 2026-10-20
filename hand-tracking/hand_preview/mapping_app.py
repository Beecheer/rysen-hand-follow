"""Camera-to-URDF web app with optional explicitly started SDK follow."""
import asyncio
import hashlib
import json
import math
import os
from contextlib import asynccontextmanager
import threading
import time
import webbrowser
from pathlib import Path
from copy import deepcopy
from collections import Counter, deque
from runpy import run_path

RUNTIME_POLICY=run_path(str(Path(__file__).resolve().parents[2]/'vision_runtime.py'))['apply_process_policy']()

import cv2
import mediapipe as mp
import numpy as np
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from .app import parser, create_detector, open_camera, draw_preview, hand_label
from .geometry import finger_angles, FINGERS
from .mapping import PreviewMapper, TransientLossHold, update_live_tracking, ASSET_DIR
from .kinematics import RobotKinematics, ThumbRetargeter
from .calibration import CalibrationWizard,correct_fingers,finger_targets,calibration_cues
from .model import DEFAULT_MODEL, validate_model
from .hardware_monitor import HardwareMonitor
from .index_follow import IndexFollow
from .human_pose import FrontHandEstimator,MotionCheck

ROOT = DEFAULT_MODEL.parents[1]


class MappingSession:
    def __init__(self, args, profile_path=None, tracker=None):
        cv2.setNumThreads(1)
        self.args = args
        self.lock = threading.Lock()
        self.stop = threading.Event()
        hybrid=getattr(args,'tracker','')=='wilor-d435'
        self.mapper = PreviewMapper(responsive=args.full_hand_follow,tracking_grace=.4 if hybrid else .2)
        self.hybrid_diagnostics={}
        self.hybrid_trace=deque(maxlen=600)
        self.hybrid_counts=Counter()
        self.dropout_hold = TransientLossHold(max_gap=0.30)
        self.current_frame_held = False
        self.dropout_hold_frames = 0
        self.current_label_held = False
        self.robot = RobotKinematics()
        self.thumb = ThumbRetargeter(self.robot,self.mapper.specs)
        self.thumb_reason = None
        self.front=FrontHandEstimator()
        self.side_sway=None
        if getattr(args,'tracker','')=='wilor-d435':
            from .side_sway_fusion import SideSwayFusion
            self.side_sway=SideSwayFusion()
            self.wilor_fallback_frames=0
        self.motion_check=MotionCheck()
        if getattr(args,'tracker','mediapipe') in ('wilor','wilor-d435'):
            tracker='wilor';profile_path=ROOT/'profiles'/'wilor-current.json'
        context={'camera':args.camera,'backend':args.backend,'swap_handedness':args.swap_handedness}
        if getattr(args,'tracker','')=='wilor-d435':
            # WiLoR poses are camera-independent. Keep the user's accepted
            # WiLoR offsets while replacing only the RGB/depth source.
            path=profile_path or ROOT/'profiles'/'wilor-current.json'
            try:context=deepcopy(json.loads(path.read_text(encoding='utf-8'))['context'])
            except (OSError,ValueError,KeyError,TypeError):context={'camera':'d435','backend':'realsense',
                'swap_handedness':args.swap_handedness,'tracker':'wilor'}
            context['swap_handedness']=args.swap_handedness
        if tracker:context['tracker']=tracker
        if tracker=='wilor':
            from .wilor_calibration import WiLoRCalibrationWizard
            self.calibration=WiLoRCalibrationWizard(self.robot,self.mapper.bounds,context,
                profile_path or ROOT/'profiles'/'wilor-current.json')
        else:
            self.calibration=CalibrationWizard(self.robot,self.mapper.bounds,context,
                **({'path':profile_path} if profile_path else {}))
        if args.full_hand_follow:
            limits=json.loads((ROOT/'profiles'/'full-hand-limits.json').read_text())['bounds']
            for name,pair in limits.items():
                self.mapper.bounds[name]=tuple(pair)
                self.mapper.specs[name]['lower'],self.mapper.specs[name]['upper']=pair
                self.mapper.positions[name]=max(pair[0],min(pair[1],self.mapper.positions[name]))
            self.mapper._reset_motion()
            self.thumb=ThumbRetargeter(self.robot,self.mapper.specs)
        self.uncalibrated_angles=None
        from .thumb_calibration import ThumbCalibrationWizard
        self.thumb_calibration=ThumbCalibrationWizard(self.calibration,self.robot,self.mapper.bounds,self.mapper.specs)
        self.contact_cues=None
        self.jpeg = self.error = None
        self.depth_jpeg=None
        self.video_frame_id=-1
        self.depth_preview_range=(.2,1.5)
        self.mirror, self.swap = not args.no_mirror, args.swap_handedness
        self.fps = self.inference_ms = 0.0
        self.mapping_ms = 0.0
        self.processing_ms = 0.0
        self.detected_hands = []
        self.rejection_reason = None
        self.accepted_frames = self.rejected_frames = 0
        self.hand_score = self.frame_at = None
        self.frame_id = 0
        self.last_wrist = self.last_selected = None
        self.last_left_confirmed = None
        self.hardware=HardwareMonitor(self.mapper,self.hardware_source,self.stop,
            ROOT/'logs'/'sdk-dry-run.jsonl') if args.sdk_monitor or args.index_follow or args.full_hand_follow else None
        self.follow=IndexFollow(self.hardware_source,self.stop,ROOT/'profiles'/'sdk-control-token.txt',args.full_hand_follow,
            self.hardware.snapshot,ROOT/'logs'/'follow-events.jsonl',target_max_age_ms=400 if hybrid else 200) if args.index_follow or args.full_hand_follow else None
        self.comparison=None
        if getattr(args,'compare_wilor',False):
            from .comparison import Comparison
            self.comparison=Comparison(self)

    def hardware_source(self):
        with self.lock:return self.mapper.snapshot(time.perf_counter())

    def snapshot(self):
        with self.lock:
            now = time.perf_counter()
            data = self.mapper.snapshot(now)
            data['mapped_angles']=data['raw_angles']
            if data['age_ms'] is not None and data['age_ms']<200:
                data['raw_angles']=self.uncalibrated_angles
            data['calibration']=self.calibration.snapshot(now)
            data['thumb_calibration']=self.thumb_calibration.snapshot(now)
            data['human_structure']=deepcopy(self.front.report)
            data['side_sway']=deepcopy(self.side_sway.report) if self.side_sway else None
            if self.side_sway:data['side_sway']['fallback_frames']=self.wilor_fallback_frames
            data['motion_check']=self.motion_check.snapshot(now)
            data['contact']=None if self.contact_cues is None else {
                'finger':self.contact_cues['active'],'strength':self.contact_cues['strength'],
                'target_error_mm':getattr(self.thumb,'tip_error_mm',None)}
            if self.error:
                data.update(state='ERROR', reason=self.error, valid=False)
            data.update(fps=self.fps, inference_ms=self.inference_ms, handedness_score=self.hand_score,
                        frame_id=self.frame_id, mirror=self.mirror, swap_handedness=self.swap,
                        frame_age_ms=None if self.frame_at is None else (now - self.frame_at) * 1000)
            data.update(thumb_fit_error=self.thumb.fit_error, thumb_reason=self.thumb_reason, mapping_ms=self.mapping_ms)
            data['thumb_diagnostics']=deepcopy(self.thumb.diagnostics) if data['valid'] else None
            data['frame_diagnostics']={'processing_ms':self.processing_ms,'detected_hands':deepcopy(self.detected_hands),
                                       'max_hands':self.args.num_hands,
                                       'rejection_reason':self.rejection_reason,'accepted_frames':self.accepted_frames,
                                       'rejected_frames':self.rejected_frames,
                                       'current_frame_held':self.current_frame_held,
                                       'hand_label_held':self.current_label_held,
                                       'dropout_hold_frames':self.dropout_hold_frames}
            data['hardware']=self.hardware.snapshot() if self.hardware else None
            data['index_follow']=self.follow.snapshot() if self.follow else None
            data['tracker']=getattr(self.args,'tracker','mediapipe')
            data['runtime_power_policy']=RUNTIME_POLICY
            data['runtime_thread_limits']={'opencv':cv2.getNumThreads(),
                **{key:os.environ.get(key) for key in ('OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','OMP_NUM_THREADS')}}
            data['depth_video']={'enabled':self.side_sway is not None,'ready':self.depth_jpeg is not None,
                'near_m':self.depth_preview_range[0],'far_m':self.depth_preview_range[1]}
            if self.side_sway:data['hybrid_diagnostics']=deepcopy(self.hybrid_diagnostics)
            raw=self.mapper.raw or {}
            clipped=[]
            if self.args.full_hand_follow:
                for finger,part in raw.items():
                    fields=[(f'j{i}',i) for i in range(4)] if finger=='thumb' else [('abd',0),('mcp',1),('pip',2)]
                    for key,i in fields:
                        value=part.get(key);name=f'left_{finger}_j{i}'
                        if value is not None and name in self.mapper.bounds:
                            lo,hi=self.mapper.bounds[name]
                            if value<lo or value>hi:clipped.append(name)
            data['limit_clipped_joints']=clipped
            if self.comparison:data['comparison']=self.comparison.snapshot()
            return data

    def select_angles(self, result, swap, now):
        self.current_label_held=False
        using_fallback=bool(getattr(self,'_using_mp_fallback',False))
        minimum_score=.45 if using_fallback else .65
        candidates = []
        self.detected_hands=[{'label':hand_label(c[0],swap),'score':float(c[0].score)} for c in result.handedness if c]
        for image_points, world, categories in zip(result.hand_landmarks, result.hand_world_landmarks, result.handedness):
            category = categories[0]
            corrected_label=hand_label(category,swap)
            if category.score < minimum_score:
                continue
            image = np.array([(p.x, p.y) for p in image_points])
            palm=image[[0,5,9,17]]
            if not np.isfinite(palm).all() or (palm < 0.01).any() or (palm > 0.99).any():
                continue
            distance = 0 if self.last_wrist is None else float(np.linalg.norm(image[0] - self.last_wrist))
            same_track=(self.side_sway is not None and len(result.handedness)==1
                        and not self.calibration.collecting and not self.thumb_calibration.collecting
                        and self.last_wrist is not None and self.last_selected is not None
                        and self.last_left_confirmed is not None
                        and 0 <= now-self.last_selected <= .30
                        and 0 <= now-self.last_left_confirmed <= .75 and distance < .12)
            # WiLoR/MediaPipe can flip handedness briefly as a palm rotates or
            # fingers occlude one another. Keep the already selected physical
            # hand across that short label glitch. Its model coordinates still
            # use the raw model label: WiLoR mirrors 3D coordinates by that label.
            if corrected_label!='Left' and not (corrected_label=='Right' and same_track):
                continue
            candidates.append((distance, -category.score, image, world, category))
        if not candidates:
            if getattr(result,'backend_error',None):reason=result.backend_error
            elif not self.detected_hands:reason='No hand detected'
            elif not any(h['label']=='Left' for h in self.detected_hands):reason='Only right hand detected; check handedness setting'
            elif max(h['score'] for h in self.detected_hands if h['label']=='Left')<minimum_score:reason=f'Left label score below {minimum_score:.2f}'
            else:reason='Palm outside image or invalid'
            self.calibration.reject(reason,now)
            self.thumb_calibration.reject(reason,now)
            return None, None, reason
        _, _, image, world, category = min(candidates, key=lambda c: (c[0], c[1]))
        geometry_label=category.category_name
        if (self.last_selected is not None and now - self.last_selected < 0.2
                and np.linalg.norm(image[0] - self.last_wrist) > 0.3):
            return None, None, 'Abrupt hand position change rejected'
        # Selection uses the corrected anatomical label; coordinate chirality
        # follows the raw model label (important for driver-mirrored cameras).
        world_points=[(p.x,p.y,p.z) for p in world]
        profile=self.calibration.profile if self.calibration.enabled and not using_fallback else None
        enhanced=bool(profile and profile.get('schema') in (3,5))
        observation=None;thumb_valid=True;clean_world=world_points
        if getattr(result,'depth_hand',None) is not None:
            from .depth_source import structure_observation
            angles=finger_angles(world_points,geometry_label)
            observation=structure_observation(world_points,image*np.array([self.args.width,self.args.height]),geometry_label,result.depth_hand)
            errors=result.depth_hand.get('quality',{}).get('bone_error_mm',[0.]*20)
            for part,i in [('index',1),('middle',2),('ring',3),('pinky',4)]:
                if max(errors[i*4:(i+1)*4])>15:angles[part]=dict.fromkeys(angles[part])
            thumb_valid=max(errors[:4])<=15
            self.front.report={'active':True,'front_ok':True,'message':'D435 米制融合直接解算；未经过单目重建'}
        elif enhanced or self.calibration.collecting:
            zero_capture=bool(self.calibration.collecting and self.calibration.collecting['pose']=='side_zero')
            personal=profile['structure'] if profile and (not self.calibration.collecting or zero_capture) else None
            size=np.array([getattr(self.args,'width',640),getattr(self.args,'height',480)])
            angles,clean_world,thumb_valid,observation=self.front.estimate(world_points,image*size,geometry_label,now,personal)
        else:
            angles = finger_angles(world_points, geometry_label)
            self.front.report={'active':False,'message':'旧标定已保留；完成八姿态标定后启用个人结构优化'}
        # Track each part independently; an occluded fingertip must not stop
        # the remaining fingers. None means hold this part, never go to zero.
        for finger,indices in {'index':[5,6,7,8],'middle':[9,10,11,12],
                               'ring':[13,14,15,16],'pinky':[17,18,19,20]}.items():
            part=image[indices]
            if not np.isfinite(part).all() or (part<0.01).any() or (part>0.99).any():
                angles[finger]={key:None for key in angles[finger]}
            if angles[finger]['abd'] is not None:
                angles[finger]['abd'] *= self.robot.abduction_signs[finger]
        side_observation=getattr(self,'_side_observation',None)
        if self.side_sway and not using_fallback:
            from .side_sway_fusion import use_model_side_prior
            quality=self.front.report.get('fingers',{})
            if observation is not None:
                observation['side_sway_model']={f:info['world_side']*self.robot.abduction_signs[f]
                    for f,info in quality.items() if info.get('world_side') is not None}
            angles=use_model_side_prior(angles,quality,profile,self.robot.abduction_signs)
        calibration_angles=deepcopy(angles)
        if self.side_sway and side_observation is not None and not using_fallback:
            angles=self.side_sway.correct(angles,side_observation['mediapipe'],side_observation['depth'],
                side_observation['scale'],side_observation['intr'],label='Left',swap=swap,
                signs=self.robot.abduction_signs,reference=profile.get('side_sway_zero') if profile else None)
            if observation is not None:
                observation['side_sway_depth']={f:math.radians(info['depth_image_deg'])
                    for f,info in self.side_sway.report.get('fingers',{}).items()}
        if using_fallback:
            # The fallback has no WiLoR side zero; never replace a calibrated
            # side target with its unrelated absolute angle.
            for finger in FINGERS:angles[finger]['abd']=None
        elif np.isfinite(image).all() and (image>=.01).all() and (image<=.99).all():
            self.calibration.ingest(calibration_angles if self.side_sway else angles,world_points,geometry_label,now,observation)
            self.thumb_calibration.ingest(angles,world_points,clean_world,geometry_label,now,thumb_valid)
        else:
            self.calibration.reject('部分手指超出画面边缘，请将整只手完整入镜',now)
            self.thumb_calibration.reject('请将手掌和指尖完整入镜',now)
        raw_four=deepcopy(angles)
        self.uncalibrated_angles=deepcopy(raw_four)
        angles=correct_fingers(angles,profile)
        if hasattr(self,'finger_filter') and not (self.calibration.collecting or self.thumb_calibration.collecting):
            angles=self.finger_filter.update(angles,now)
        angles['thumb']={f'j{i}':None for i in range(4)}
        self.thumb_reason=None
        self.contact_cues=None
        try:
            if self.calibration.collecting or self.thumb_calibration.collecting:raise ValueError('采集中，仅统计人手结构；机械手拇指拟合暂缓')
            if not thumb_valid:raise ValueError('拇指结构或掌面观测不可靠，保持已有拇指目标')
            part=image[1:5]
            if not np.isfinite(part).all() or (part<0.01).any() or (part>0.99).any():
                raise ValueError('Thumb outside image')
            if profile and profile.get('schema')!=5:
                valid=[f for f in raw_four if raw_four[f].get('mcp') is not None and raw_four[f].get('pip') is not None]
                # Contact targets use the actual displayed finger posture,
                # so thumb constraints follow the filtered model fingertips.
                self.contact_cues=calibration_cues(profile,world_points,geometry_label,
                    self.robot,self.mapper.positions,valid)
            angles['thumb']=self.thumb.fit(clean_world,geometry_label,cues=self.contact_cues,now=now,
                thumb_model=self.thumb_calibration.active_model(),
                direction_reference=profile.get('thumb_direction') if profile else None)
            self.uncalibrated_angles['thumb']=deepcopy(angles['thumb'])
        except ValueError as exc:
            self.thumb_reason=str(exc)
            self.thumb.fit_error=None
        if not any(v is not None for part in angles.values() for v in part.values()):
            return None,None,'No reliable finger geometry'
        self.last_wrist, self.last_selected = image[0], now
        self.current_label_held=hand_label(category,swap)!='Left'
        if hand_label(category,swap)=='Left':self.last_left_confirmed=now
        return angles, float(category.score), 'Valid left hand'

    def worker(self):
        if getattr(self.args,'tracker','')=='wilor-d435':
            self.worker_wilor_d435()
            return
        camera = None
        try:
            validate_model(self.args.model)
            if getattr(self.args,'tracker','mediapipe') in ('wilor','wilor-d435'):
                from .wilor_detector import WiLoRDetector
                detector_context=WiLoRDetector()
            else:detector_context=create_detector(self.args)
            with detector_context as detector:
                replay=getattr(self.args,'compare_video',None)
                if replay:
                    camera=cv2.VideoCapture(str(replay));ok,frame=camera.read();backend='video replay'
                    if not ok:raise RuntimeError('无法读取测试视频')
                    replay_fps=camera.get(cv2.CAP_PROP_FPS)
                    if not 1<=replay_fps<=120:replay_fps=15.
                    replay_due=time.perf_counter()
                else:camera, frame, backend = open_camera(self.args)
                self.args.height,self.args.width=frame.shape[:2]
                start = previous = acquired = time.perf_counter()
                timestamp = -1
                while not self.stop.is_set():
                    timestamp = max(timestamp + 1, int((acquired - start) * 1000))
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    begin = time.perf_counter()
                    result = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,
                                                                 data=np.ascontiguousarray(rgb)), timestamp)
                    now = time.perf_counter()
                    with self.lock:
                        mirror, swap = self.mirror, self.swap
                        mapping_begin = time.perf_counter()
                        try:
                            angles, score, reason = self.select_angles(result, swap, now)
                            if getattr(result,'backend_error',None):reason=result.backend_error
                        except ValueError as exc:
                            angles, score, reason = None, None, str(exc)
                        mapped_at = time.perf_counter()
                        self.mapping_ms = (mapped_at - mapping_begin) * 1000
                        self.processing_ms = (mapped_at - acquired) * 1000
                        self.rejection_reason=reason if angles is None else ('Frame processing exceeded 200 ms' if self.processing_ms>200 else None)
                        if self.rejection_reason:self.rejected_frames+=1
                        else:self.accepted_frames+=1
                        self.mapper.update(angles, acquired, mapped_at, reason)
                        self.motion_check.ingest(angles,self.mapper.snapshot(mapped_at)['valid'],mapped_at)
                        self.hand_score = score
                        if getattr(self.args,'tracker','mediapipe') in ('wilor','wilor-d435'):
                            self.hand_score=None
                            for h in self.detected_hands:h['score']=None
                        instant = 1 / max(now - previous, 1e-6)
                        self.fps = instant if not self.frame_id else 0.9 * self.fps + 0.1 * instant
                        self.inference_ms = (now - begin) * 1000
                        self.frame_id += 1
                        self.frame_at = acquired
                        fps, inference_ms = self.fps, self.inference_ms
                    previous = now
                    if self.comparison:
                        self.comparison.submit(frame,self.frame_id,acquired,result,
                            {'inference_ms':self.inference_ms,'mapping_ms':self.mapping_ms,'swap':self.swap})
                    display = draw_preview(frame, result, mirror=mirror, swap=swap, show_ids=self.args.show_ids,
                                           fps=fps, inference_ms=inference_ms, backend=backend, show_shortcuts=False)
                    ok, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 78])
                    if ok:
                        with self.lock:
                            self.jpeg = jpeg.tobytes()
                    if self.stop.is_set():
                        break
                    if replay:
                        replay_due+=1/replay_fps
                        self.stop.wait(max(0,replay_due-time.perf_counter()))
                    ok, frame = camera.read()
                    acquired = time.perf_counter()
                    if not ok or frame is None or not frame.size:
                        if replay:
                            with self.lock:self.mapper.pause();self.mapper.reason='视频回放结束'
                            self.stop.wait()
                            break
                        raise RuntimeError('Camera read failed; stop and restart preview')
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            print(f'CAMERA ERROR: {exc}', flush=True)
        finally:
            if self.comparison:self.comparison.close_recording()
            if camera is not None:
                camera.release()
            print('Mapping camera worker released.', flush=True)

    def worker_wilor_d435(self):
        """Run WiLoR and MediaPipe on one D435 RGB frame; depth only steers abduction."""
        camera=None;camera_worker=None
        try:
            validate_model(self.args.model)
            from .depth_source import Camera
            from .wilor_detector import WiLoRDetector
            camera=Camera(getattr(self.args,'depth_serial',None),track_hands=False,encode_preview=False)
            camera_worker=threading.Thread(target=camera.run,name='wilor-d435-source',daemon=True)
            camera_worker.start()
            mp_args=deepcopy(self.args)
            # This model is the dropout fallback, so let it retain partially
            # occluded/flexed hands; mapper reacquisition still needs 3 frames.
            mp_args.detection_confidence=min(float(mp_args.detection_confidence),.45)
            mp_args.presence_confidence=min(float(mp_args.presence_confidence),.45)
            mp_args.tracking_confidence=min(float(mp_args.tracking_confidence),.45)
            with WiLoRDetector() as wilor, create_detector(mp_args) as mediapipe_detector:
                last_frame=-1;timestamp=-1;previous_processed=None
                self._wilor_fallback_active=False;self._wilor_reacquire_streak=0
                while not self.stop.wait(.003):
                    with camera.lock:
                        if camera.state=='ERROR':raise RuntimeError(camera.error or 'D435 相机错误')
                        if camera.frames==last_frame or camera.color is None or camera.depth is None:continue
                        last_frame=camera.frames;frame=camera.color.copy();depth=camera.depth.copy()
                        intr=camera.intr;scale=camera.scale;camera_fps=camera.fps;acquired=camera.frame_at
                        depth_near,depth_far=camera.near,camera.far
                    if acquired is None:continue
                    self.args.height,self.args.width=frame.shape[:2]
                    rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                    image=mp.Image(image_format=mp.ImageFormat.SRGB,data=np.ascontiguousarray(rgb))
                    timestamp=max(timestamp+1,int(acquired*1000))
                    begin=time.perf_counter()
                    mp_result=mediapipe_detector.detect_for_video(image,timestamp)
                    mp_finished=time.perf_counter()
                    result=wilor.detect_for_video(image,timestamp)
                    received=time.perf_counter()
                    with self.lock:
                        mirror,swap=self.mirror,self.swap
                        self._side_observation={'mediapipe':mp_result,'depth':depth,'scale':scale,'intr':intr}
                        mapping_begin=time.perf_counter()
                        active_result=result;using_fallback=False;wilor_reason=mp_reason=None
                        try:
                            mp_left=self._has_left_hand(mp_result,swap)
                            wilor_left=self._has_left_hand(result,swap)
                            if self._wilor_fallback_active:
                                self._wilor_reacquire_streak=self._wilor_reacquire_streak+1 if wilor_left else 0
                                if self._wilor_reacquire_streak>=3:
                                    self._wilor_fallback_active=False;self._wilor_reacquire_streak=0
                            use_mp=self._wilor_fallback_active and mp_left
                            if use_mp:
                                angles=score=reason=None
                            else:
                                try:angles,score,reason=self.select_angles(result,swap,received)
                                except ValueError as exc:angles,score,reason=None,None,str(exc)
                                wilor_reason=reason
                            if (not use_mp and angles is None and mp_left and
                                    not self.calibration.collecting and not self.thumb_calibration.collecting):
                                self._wilor_fallback_active=True;self._wilor_reacquire_streak=0
                            if use_mp or (angles is None and self._wilor_fallback_active and mp_left):
                                using_fallback=True;active_result=mp_result;self._using_mp_fallback=True
                                self.wilor_fallback_frames+=1
                                try:angles,score,reason=self.select_angles(mp_result,swap,received)
                                except ValueError as exc:angles,score,reason=None,None,str(exc)
                                mp_reason=reason
                                self.side_sway.report={'active':True,'corrected':[],
                                    'message':'WiLoR 暂时漏检；MediaPipe 补位屈伸，侧摆保持上一 WiLoR 目标'}
                        except ValueError as exc:angles,score,reason=None,None,str(exc)
                        finally:self._side_observation=None;self._using_mp_fallback=False
                        mapped=time.perf_counter();self.mapping_ms=(mapped-mapping_begin)*1000
                        self.processing_ms=(mapped-acquired)*1000
                        self.rejection_reason=reason if angles is None else None
                        if self.rejection_reason:self.rejected_frames+=1
                        else:self.accepted_frames+=1
                        # Reacquire the visual preview after automatic loss.
                        # IndexFollow never re-enables a stopped robot itself.
                        if angles is not None:self.mapper.resume_after_loss()
                        collecting=self.calibration.collecting or self.thumb_calibration.collecting
                        live_tracking=self.follow is not None or self.hardware is not None
                        if live_tracking:
                            self.dropout_hold.clear()
                            mapper_angles=angles
                            held_dropout=update_live_tracking(self.mapper,angles,acquired,mapped,reason)
                        elif self.mapper.paused or collecting:
                            self.dropout_hold.clear()
                            mapper_angles,held_dropout=angles,False
                        else:
                            mapper_angles,held_dropout=self.dropout_hold.update(angles,acquired)
                        self.current_frame_held=held_dropout
                        if held_dropout:self.dropout_hold_frames+=1
                        if not live_tracking:self.mapper.update(mapper_angles,acquired,mapped,reason)
                        self.motion_check.ingest(angles,self.mapper.snapshot(mapped)['valid'] and not held_dropout,mapped)
                        if previous_processed is not None:
                            hz=1/max(.001,mapped-previous_processed)
                            self.fps=hz if not self.fps else .85*self.fps+.15*hz
                        previous_processed=mapped
                        self.hand_score=score if using_fallback else None
                        self.inference_ms=(received-begin)*1000;self.frame_id+=1;self.frame_at=acquired
                        self.detected_hands=[{'label':hand_label(items[0],swap),'score':items[0].score if using_fallback else None}
                            for items in active_result.handedness if items]
                        models={name:self._describe_result(value,swap) for name,value in
                                (('wilor',result),('mediapipe',mp_result))}
                        for name,info in models.items():
                            labels=','.join(h['label'] for h in info['hands']) or 'empty'
                            self.hybrid_counts[f'{name}:{labels}']+=1
                        source='mediapipe' if using_fallback else 'wilor'
                        outcome='held' if held_dropout else ('accepted' if angles is not None else reason)
                        self.hybrid_counts[f'output:{outcome}']+=1
                        if self.processing_ms>200:self.hybrid_counts['stale_input']+=1
                        diagnostic={'frame':self.frame_id,'time':mapped,'models':models,'source':source,
                            'wilor_reason':wilor_reason,'mediapipe_reason':mp_reason,
                            'held':held_dropout,'label_held':self.current_label_held,
                            'state':self.mapper.state,'reason':self.mapper.reason,
                            'capture_fps':camera_fps,'processed_fps':self.fps,
                            'mediapipe_ms':(mp_finished-begin)*1000,
                            'wilor_ms':(received-mp_finished)*1000,'processing_ms':self.processing_ms,
                            'mapping_ms':self.mapping_ms,'capture_age_ms':(begin-acquired)*1000,
                            'inference_wall_ms':(received-begin)*1000}
                        self.hybrid_trace.append(diagnostic)
                        self.hybrid_diagnostics={**diagnostic,'counts':dict(self.hybrid_counts)}
                    display=draw_preview(frame,active_result,mirror=mirror,swap=swap,show_ids=self.args.show_ids,
                        fps=self.fps,inference_ms=self.inference_ms,
                        backend='D435 RGB + depth'+(' · MediaPipe 补位' if using_fallback else ''),show_shortcuts=False)
                    ok,jpeg=cv2.imencode('.jpg',display,[cv2.IMWRITE_JPEG_QUALITY,78])
                    normalized=np.clip((depth.astype(np.float32)*scale-depth_near)/(depth_far-depth_near),0,1)
                    depth_display=cv2.applyColorMap((normalized*255).astype(np.uint8),cv2.COLORMAP_TURBO)
                    depth_display[depth==0]=0
                    if mirror:depth_display=cv2.flip(depth_display,1)
                    depth_ok,depth_jpeg=cv2.imencode('.jpg',depth_display,[cv2.IMWRITE_JPEG_QUALITY,78])
                    with self.lock:
                        if ok and depth_ok:
                            self.jpeg=jpeg.tobytes()
                            self.depth_jpeg=depth_jpeg.tobytes()
                            self.video_frame_id=self.frame_id
                        self.depth_preview_range=(depth_near,depth_far)
        except Exception as exc:
            with self.lock:self.error='WiLoR + D435 链路异常：'+str(exc)
            print(self.error,flush=True)
        finally:
            if camera is not None:camera.stop.set()
            if camera_worker:camera_worker.join(6)
            print('WiLoR + D435 camera worker released.',flush=True)

    @staticmethod
    def _describe_result(result,swap):
        hands=[]
        for points,categories in zip(result.hand_landmarks,result.handedness):
            if not categories:continue
            palm=np.array([(points[i].x,points[i].y) for i in (0,5,9,17)])
            hands.append({'label':hand_label(categories[0],swap),'score':float(categories[0].score),
                'palm_in_frame':bool(np.isfinite(palm).all() and (palm>=.01).all() and (palm<=.99).all()),
                'wrist':[float(points[0].x),float(points[0].y)]})
        return {'hands':hands,'error':getattr(result,'backend_error',None)}

    @staticmethod
    def _has_left_hand(result,swap):
        for categories in getattr(result,'handedness',[]):
            if not categories or float(getattr(categories[0],'score',0.))<.45:continue
            label=categories[0].category_name
            if swap:label={'Left':'Right','Right':'Left'}.get(label,label)
            if label=='Left':return True
        return False


def make_app(session):
    @asynccontextmanager
    async def lifespan(app):
        if session.comparison:session.comparison.thread.start()
        worker = threading.Thread(target=session.worker, name='mapping-camera', daemon=True)
        worker.start()
        monitor=None
        if session.hardware:
            monitor=threading.Thread(target=session.hardware.run,name='sdk-dry-run',daemon=True)
            monitor.start()
        follower=None
        if session.follow:
            follower=threading.Thread(target=session.follow.run,name='index-camera-follow',daemon=True)
            follower.start()
        yield
        session.stop.set()
        await asyncio.to_thread(worker.join, getattr(session,'shutdown_timeout',3))
        if monitor:await asyncio.to_thread(monitor.join, 2)
        if follower:await asyncio.to_thread(follower.join, 2)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)

    @app.get('/')
    def index():
        web=ROOT/'web'
        version=hashlib.sha256(b''.join((web/name).read_bytes() for name in ('main.js','style.css','depth.js','depth.css'))).hexdigest()[:12]
        content=(web/'index.html').read_text(encoding='utf-8').replace('src="/static/main.js"',f'src="/static/main.js?v={version}"').replace('href="/static/style.css"',f'href="/static/style.css?v={version}"')
        return HTMLResponse(content,headers={'Cache-Control':'no-store'})

    @app.get('/api/status')
    def status():
        return session.snapshot()

    @app.get('/api/diagnostics/hybrid')
    def hybrid_diagnostics():
        with session.lock:
            return {'tracker':getattr(session.args,'tracker','mediapipe'),
                    'counts':dict(session.hybrid_counts),'frames':deepcopy(list(session.hybrid_trace))}

    @app.get('/api/model')
    def model():
        return {'joints': session.mapper.specs, 'asset': '/assets/apex_left/apex_hand_left.urdf'}

    @app.get('/video')
    async def video():
        async def frames():
            last_id = -1
            while not session.stop.is_set():
                with session.lock:
                    data=session.jpeg
                    current=session.video_frame_id if getattr(session.args,'tracker','')=='wilor-d435' else session.frame_id
                if data and current != last_id:
                    last_id = current
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n'
                await asyncio.sleep(1/15)
        return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame',
                                 headers={'Cache-Control': 'no-store'})

    @app.get('/depth-video')
    async def depth_video():
        if getattr(session.args,'tracker','')!='wilor-d435':
            raise HTTPException(404,'Aligned depth preview requires WiLoR + D435')
        async def frames():
            last_id=-1
            while not session.stop.is_set():
                with session.lock:data,current=session.depth_jpeg,session.video_frame_id
                if data and current!=last_id:
                    last_id=current
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+data+b'\r\n'
                await asyncio.sleep(1/15)
        return StreamingResponse(frames(),media_type='multipart/x-mixed-replace; boundary=frame',
                                 headers={'Cache-Control':'no-store'})

    @app.websocket('/ws')
    async def websocket(ws: WebSocket):
        origin = ws.headers.get('origin')
        allowed = (f'http://127.0.0.1:{session.args.port}', f'http://localhost:{session.args.port}')
        if origin and origin not in allowed:
            await ws.close(code=1008)
            return
        await ws.accept()
        try:
            while not session.stop.is_set():
                await ws.send_json(session.snapshot())
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError):
            pass

    @app.post('/api/control/{action}')
    async def control(action: str, request: Request):
        if request.headers.get('origin') != f'http://{request.headers.get("host")}':
            raise HTTPException(403, 'Same-origin browser control required')
        if session.follow and action in ('pause','swap','stop'):
            try:
                if action=='swap':await asyncio.to_thread(session.follow.stop_confirmed)
                else:await asyncio.to_thread(session.follow.action,'stop')
            except Exception as exc:
                if action=='swap':raise HTTPException(503,'未确认停止实机跟随：'+str(exc))
        if action=='resume' and (session.calibration.collecting or session.thumb_calibration.collecting or
                (session.comparison and (session.comparison.session.calibration.collecting or
                                        session.comparison.session.thumb_calibration.collecting))):
            raise HTTPException(409,'正在标定采样，请完成或取消后恢复识别')
        if session.comparison and action=='swap' and request.query_params.get('backend')=='wilor':
            b=session.comparison.session
            with b.lock:
                b.swap=not b.swap;b.calibration.cancel();b.calibration.enabled=False
                b.thumb_calibration.cancel()
                b.calibration.profile=None;b.calibration.pending=None;b.calibration.samples={}
                b.calibration.context['swap_handedness']=b.swap;b.mapper.pause()
            return {'ok':True}
        if session.comparison and action in ('pause','resume'):
            b=session.comparison.session
            with b.lock:
                if action=='pause':b.mapper.pause()
                elif action=='resume':b.mapper.resume();b.front.reset();b.thumb.observation_filter.reset()
        with session.lock:
            if action == 'pause':
                session.mapper.pause()
            elif action == 'resume':
                session.mapper.resume()
                session.front.reset()
                session.thumb.observation_filter.reset()
                if getattr(session.thumb,'config',None) is None:
                    session.thumb.previous=np.array([session.mapper.positions[f'left_thumb_j{i}'] for i in range(4)])
                session.last_wrist = session.last_selected = None
                session.last_left_confirmed=None
                session.dropout_hold.clear()
            elif action == 'mirror':
                session.mirror = not session.mirror
            elif action == 'swap':
                session.calibration.cancel()
                session.thumb_calibration.cancel()
                session.calibration.enabled=False
                session.calibration.profile=None
                session.calibration.pending=None
                session.calibration.samples={}
                session.calibration.context['swap_handedness']=not session.swap
                session.calibration.message='左右手校正已改变，旧标定已停用，请重新标定'
                session.swap = not session.swap
                session.mapper.pause()
                session.mapper.reason = 'Hand label changed; click Resume after checking left hand'
                session.last_wrist = session.last_selected = None
                session.last_left_confirmed=None
                session.dropout_hold.clear()
            elif action == 'stop':
                session.stop.set()
            else:
                raise HTTPException(404, 'Unknown action')
            if hasattr(session,'on_control'):session.on_control(action)
        return {'ok': True}

    @app.post('/api/index-follow/{action}')
    def index_follow(action:str,request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':
            raise HTTPException(403,'Same-origin browser control required')
        if not session.follow:raise HTTPException(409,'未启用食指跟随模式')
        if action=='start' and (session.calibration.collecting or session.thumb_calibration.collecting):
            raise HTTPException(409,'标定采样期间不能启动实机跟随')
        if action=='start' and hasattr(session,'gate_reason'):
            reason=session.gate_reason()
            if reason:raise HTTPException(409,reason)
        try:session.follow.action(action)
        except Exception as exc:raise HTTPException(400,str(exc))
        return {'ok':True}

    @app.post('/api/calibration/{action}')
    async def calibration(action:str,request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':
            raise HTTPException(403,'Same-origin browser control required')
        body=await request.json()
        target=session
        if body.get('backend')=='wilor':
            if not session.comparison:raise HTTPException(409,'WiLoR comparison is not enabled')
            target=session.comparison.session
        if action in ('capture','apply','toggle','reset') and session.follow:
            try:await asyncio.to_thread(session.follow.stop_confirmed)
            except Exception as exc:raise HTTPException(503,'未确认停止实机跟随：'+str(exc))
        with target.lock:
            try:
                if getattr(target,'depth_bound',True) is False:raise ValueError('等待 D435 相机身份与内参就绪')
                if target.thumb_calibration.collecting:raise ValueError('请先完成或取消拇指专项采样')
                if action=='capture':
                    target.calibration.start(body.get('pose'),time.perf_counter())
                    target.mapper.pause()
                elif action=='cancel':target.calibration.cancel()
                elif action=='apply':target.calibration.apply();target.front.reset();target.mapper.resume()
                elif action=='toggle':
                    if not target.calibration.profile:raise ValueError('尚无已保存标定')
                    target.calibration.enabled=not target.calibration.enabled
                    target.front.reset()
                    target.mapper.resume()
                elif action=='reset':target.calibration.reset();target.front.reset();target.mapper.resume()
                else:raise ValueError('未知标定操作')
            except (ValueError,KeyError,TypeError) as exc:
                target.calibration.message=str(exc)
                raise HTTPException(400,str(exc))
            target.mapper._reset_motion()
            target.thumb.observation_filter.reset()
            if hasattr(target,'on_calibration'):target.on_calibration(action,body.get('pose'))
        return {'ok':True}

    @app.post('/api/depth-config')
    async def depth_config(request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':raise HTTPException(403,'Same-origin required')
        if not hasattr(session,'depth_camera'):raise HTTPException(409,'仅用于 D435 直接角度模式')
        body=await request.json()
        from .depth_simple_calibration import validate_thumb
        if session.follow:
            try:await asyncio.to_thread(session.follow.stop_confirmed)
            except Exception as exc:raise HTTPException(503,'未确认停止实机跟随：'+str(exc))
        try:
            with session.lock:
                if session.calibration.collecting or session.thumb_calibration.collecting:raise ValueError('请先完成或取消标定')
                if not session.calibration.profile:raise ValueError('请先应用两步骨架标定后保存拇指设置')
                config=validate_thumb({**session.thumb.config,**body},session.mapper.bounds)
                profile=deepcopy(session.calibration.profile);profile['thumb_direct']=config
                session.calibration.write_profile(profile);session.calibration.profile=profile
                if session.calibration.pending is not None:session.calibration.pending['thumb_direct']=deepcopy(config)
                session.thumb.config=config;session.thumb.reset();session.mapper.pause()
        except (ValueError,KeyError,TypeError,OSError) as exc:raise HTTPException(400,str(exc))
        return {'ok':True,'thumb_direct':config}

    @app.post('/api/thumb-calibration/{action}')
    async def thumb_calibration(action:str,request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':
            raise HTTPException(403,'Same-origin browser control required')
        body=await request.json();target=session
        if body.get('backend')=='wilor':
            if not session.comparison:raise HTTPException(409,'WiLoR comparison is not enabled')
            target=session.comparison.session
        static=hasattr(target.thumb_calibration,'preview')
        if action not in (('preview','capture','cancel','apply','toggle','reset') if static else ('capture','cancel','apply','toggle')):
            raise HTTPException(404,'未知专项标定操作')
        if session.follow and action!='cancel':
            try:await asyncio.to_thread(session.follow.stop_confirmed)
            except Exception as exc:raise HTTPException(503,'未确认停止实机跟随：'+str(exc))
        with target.lock:
            wizard=target.thumb_calibration
            try:
                if target.calibration.collecting:raise ValueError('请先完成或取消基础标定采样')
                if action=='preview':wizard.preview(body.get('step'),body.get('reference_deg'))
                elif action=='capture':
                    if static and 'reference_deg' in body:wizard.preview(body.get('step'),body['reference_deg'])
                    wizard.start(body.get('step'),time.perf_counter())
                elif action=='cancel':wizard.cancel()
                elif action=='apply':wizard.apply()
                elif action=='reset':wizard.reset()
                elif action=='toggle':
                    if wizard.collecting:raise ValueError('请先完成或取消当前采样')
                    if not wizard.model:raise ValueError('尚无专项标定配置')
                    if static:
                        wizard.check_base()
                        if not wizard.compatible():raise ValueError('基础配置已改变，请重新采集拇指静态参考')
                    wizard.enabled=not wizard.enabled
                    if static:wizard.reference_preview=False;wizard.revision+=1
                    wizard.message='专项参考已启用' if wizard.enabled else '已停用专项参考，恢复基础标定解算'
            except (ValueError,KeyError,TypeError,OSError) as exc:
                wizard.message=str(exc);raise HTTPException(400,str(exc))
            target.mapper.pause();target.thumb.observation_filter.reset()
        return {'ok':True}

    @app.post('/api/validation/{action}')
    async def validation(action:str,request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':raise HTTPException(403,'Same-origin required')
        with session.lock:
            if action=='cancel':session.motion_check.active=None
            elif action in ('open_close','palm_turn'):
                if session.follow and session.follow.snapshot()['requested']:raise HTTPException(409,'请先暂停并失能，再做摄像头验证')
                if not session.mapper.snapshot(time.perf_counter())['valid']:raise HTTPException(409,'请先恢复识别并展示左手')
                session.motion_check.start(action,time.perf_counter())
            else:raise HTTPException(404,'Unknown validation')
        return {'ok':True}

    @app.post('/api/comparison/record/{action}')
    def record(action:str,request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':raise HTTPException(403,'Same-origin required')
        if not session.comparison:raise HTTPException(409,'Comparison is not enabled')
        if getattr(session.args,'compare_video',None):raise HTTPException(409,'回放模式不再录制')
        try:session.comparison.record(action)
        except ValueError as exc:raise HTTPException(400,str(exc))
        return {'ok':True}

    app.mount('/assets', StaticFiles(directory=ROOT / 'assets'), name='assets')
    app.mount('/static', StaticFiles(directory=ROOT / 'web'), name='web')
    return app


def main():
    p = parser()
    p.set_defaults(num_hands=1)
    p.description = 'Left-hand angle mapping, 3D preview and optional explicit SDK follow.'
    p.add_argument('--port', type=int, default=8090)
    p.add_argument('--no-browser', action='store_true')
    p.add_argument('--sdk-monitor',action='store_true',help='Read-only SDK feedback and dry-run payloads; never sends motion')
    p.add_argument('--index-follow',action='store_true',help='Bounded index camera control; requires explicit browser start')
    p.add_argument('--full-hand-follow',action='store_true',help='Full hand camera control, no session time limit; manual start')
    p.add_argument('--compare-wilor',action='store_true',help='Camera-only A/B comparison with loopback WiLoR backend')
    p.add_argument('--tracker',choices=('mediapipe','wilor','d435','wilor-d435'),default='mediapipe')
    p.add_argument('--depth-serial',help='D435 serial; never selects a webcam')
    p.add_argument('--compare-video',type=Path,help='Replay a local test video instead of opening the camera')
    p.add_argument('--wilor-swap-handedness',action='store_true',help='Correct WiLoR labels independently of MediaPipe')
    args = p.parse_args()
    if args.compare_wilor and args.tracker!='mediapipe':p.error('Comparison requires the MediaPipe camera source')
    if args.tracker=='wilor-d435' and args.index_follow:
        p.error('WiLoR + D435 supports full-hand follow rather than the index-only trial')
    if args.compare_wilor and (args.sdk_monitor or args.index_follow or args.full_hand_follow):
        p.error('WiLoR comparison is camera-only and cannot run with SDK modes')
    if args.compare_video and (not args.compare_wilor or not args.compare_video.is_file()):
        p.error('--compare-video requires comparison mode and an existing local video')
    if args.download_model or args.self_test or args.headless or args.max_frames:
        p.error('Use python -m hand_preview for model download, self-test and bounded camera tests.')
    if not 1024 <= args.port <= 65535:
        p.error('Port must be between 1024 and 65535')
    if args.tracker=='d435':
        from .depth_session import DepthMappingSession
        session=DepthMappingSession(args)
    else:session = MappingSession(args)
    app = make_app(session)
    server = uvicorn.Server(uvicorn.Config(app, host='127.0.0.1', port=args.port, access_log=False,
                                          log_level='warning', timeout_graceful_shutdown=3))

    async def serve():
        async def lifecycle():
            while not server.started and not server.should_exit:
                await asyncio.sleep(0.1)
            if server.started:
                print(f'Mapping preview ready: http://127.0.0.1:{args.port}', flush=True)
                if not args.no_browser:
                    webbrowser.open(f'http://127.0.0.1:{args.port}')
            while not server.should_exit:
                if session.stop.is_set():
                    server.should_exit = True
                    break
                await asyncio.sleep(0.1)
        task = asyncio.create_task(lifecycle())
        try:
            await server.serve()
        finally:
            session.stop.set()
            task.cancel()
    asyncio.run(serve())


if __name__ == '__main__':
    main()
