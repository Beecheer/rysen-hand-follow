"""Latest-frame, camera-only WiLoR comparison; no SDK import or transport."""
from copy import copy,deepcopy
from types import SimpleNamespace as NS
import http.client
import json
import threading
import time
from datetime import datetime
from pathlib import Path
from collections import deque
import cv2
import numpy as np


def landmark_result(hands):
    image=[];world=[];labels=[]
    for h in hands:
        p=np.asarray(h['world'],float);uv=np.asarray(h['image'],float)
        if p.shape!=(21,3) or uv.shape!=(21,2) or not np.isfinite(p).all() or not np.isfinite(uv).all():
            raise ValueError('WiLoR returned invalid landmarks')
        if h['label'] not in ('Left','Right'):raise ValueError('Invalid hand label')
        image.append([NS(x=float(x),y=float(y)) for x,y in uv])
        world.append([NS(x=float(x),y=float(y),z=float(z)) for x,y,z in p])
        # WiLoR detector class is a label, not a MediaPipe confidence score.
        labels.append([NS(category_name=h['label'],score=1.)])
    return NS(hand_landmarks=image,hand_world_landmarks=world,handedness=labels)


class Comparison:
    def __init__(self,parent):
        from .mapping_app import MappingSession,ROOT
        args=copy(parent.args);args.compare_wilor=False
        args.sdk_monitor=args.index_follow=args.full_hand_follow=False
        args.tracker='wilor'
        args.swap_handedness=getattr(parent.args,'wilor_swap_handedness',False)
        self.session=MappingSession(args,profile_path=ROOT/'profiles'/'wilor-current.json',tracker='wilor')
        self.session.calibration.capture_seconds=8
        self.session.mapper.resume()
        self.stop=parent.stop;self.pending=None;self.event=threading.Event();self.lock=threading.Lock()
        self.data={'enabled':True,'ready':False,'error':None,'completed':0,'dropped':0,'pair':None}
        self.samples=deque(maxlen=120);self.thread=threading.Thread(target=self.run,daemon=True,name='wilor-comparison')
        self.record_request=None;self.writer=None;self.record_path=None;self.record_start=None;self.record_last=0.;self.record_frames=0
        self.record_folder=ROOT/'logs'

    def record(self,action):
        with self.lock:
            if action=='start':
                if self.writer or self.record_request:raise ValueError('正在录制')
                self.record_request='start'
            elif action=='stop':self.record_request='stop'
            else:raise ValueError('Unknown recording action')

    def close_recording(self):
        with self.lock:
            if self.writer:self.writer.release();self.writer=None
            self.record_request=None

    def record_frame(self,frame,now):
        with self.lock:
            if self.record_request=='start':
                folder=self.record_folder;folder.mkdir(exist_ok=True)
                self.record_path=folder/f'comparison-{datetime.now().strftime("%Y%m%d-%H%M%S-%f")}.avi'
                self.writer=cv2.VideoWriter(str(self.record_path),cv2.VideoWriter_fourcc(*'MJPG'),15,(frame.shape[1],frame.shape[0]))
                if not self.writer.isOpened():
                    self.writer.release();self.writer=None;self.record_request=None;self.data['record_error']='无法创建录像文件';return
                self.record_start=now;self.record_last=0.;self.record_frames=0;self.record_request=None
            if self.writer and (self.record_request=='stop' or now-self.record_start>=15):
                self.writer.release();self.writer=None;self.record_request=None
            if self.writer and now-self.record_last>=1/15:
                self.writer.write(frame);self.record_last=now;self.record_frames+=1
            self.data['recording']={'active':self.writer is not None,'frames':self.record_frames,
                'elapsed':min(15,now-self.record_start) if self.record_start else 0,
                'path':str(self.record_path) if self.record_path else None}

    def submit(self,frame,frame_id,acquired,a_result,a_status):
        self.record_frame(frame,acquired)
        # One waiting frame; a slow model never creates an inference backlog.
        with self.lock:
            if self.pending is not None:self.data['dropped']+=1
            self.pending=(frame.copy(),frame_id,acquired,a_result,deepcopy(a_status))
        self.event.set()

    def snapshot(self):
        with self.lock:data=deepcopy(self.data)
        if data.get('pair'):
            data['pair']['age_ms']=(time.perf_counter()-data['pair']['captured_at'])*1000
        data['b']=self.session.snapshot()
        return data

    def run(self):
        connection=http.client.HTTPConnection('127.0.0.1',8093,timeout=15)
        try:
            while not self.stop.is_set():
                if not self.event.wait(.2):continue
                self.event.clear()
                with self.lock:item=self.pending;self.pending=None
                if item is None:continue
                frame,frame_id,acquired,a_result,a_status=item
                try:
                    connection.request('GET','/health');health_response=connection.getresponse()
                    health=json.loads(health_response.read())
                    if not health.get('ready'):raise RuntimeError(health.get('error') or 'WiLoR 模型正在下载 / 加载')
                    ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,95])
                    if not ok:raise RuntimeError('JPEG encoding failed')
                    started=time.perf_counter()
                    connection.request('POST','/infer',body=jpeg.tobytes(),headers={'Content-Type':'image/jpeg'})
                    response=connection.getresponse();payload=json.loads(response.read())
                    if response.status!=200:raise RuntimeError(str(payload.get('detail',payload)))
                    received=time.perf_counter();result=landmark_result(payload['hands'])
                    b=self.session
                    with b.lock:
                        b.args.height,b.args.width=frame.shape[:2]
                        mapping_at=time.perf_counter()
                        try:angles,score,reason=b.select_angles(result,b.swap,received)
                        except ValueError as exc:angles,score,reason=None,None,str(exc)
                        ended=time.perf_counter()
                        b.mapping_ms=(ended-mapping_at)*1000
                        b.mapper.update(angles,acquired,ended,reason)
                        b.hand_score=None;b.frame_id=frame_id;b.frame_at=acquired
                        for detected in b.detected_hands:detected['score']=None
                        b.inference_ms=payload['inference_ms'];b.processing_ms=(ended-acquired)*1000
                        b.rejection_reason=reason if angles is None else None
                        b.motion_check.ingest(angles,b.mapper.snapshot(ended)['valid'],ended)
                        if angles is None:b.rejected_frames+=1
                        else:b.accepted_frames+=1
                    self.samples.append((payload['inference_ms'],(ended-acquired)*1000))
                    def raw(result,swap=False):
                        return [{'label':('Right' if c[0].category_name=='Left' else 'Left') if swap else c[0].category_name,
                                 'raw_label':c[0].category_name,'world':[[p.x,p.y,p.z] for p in w],
                                 'image':[[p.x,p.y] for p in im]}
                                for im,w,c in zip(result.hand_landmarks,result.hand_world_landmarks,result.handedness)]
                    with self.lock:
                        previous=self.data.get('completed_at',ended)
                        b.fps=1/max(ended-previous,.001)
                        self.data.update(ready=True,error=None,completed=self.data['completed']+1,completed_at=ended,
                            device=health.get('device'),inference_median_ms=float(np.median(np.array(self.samples)[:,0])),
                            age_p95_ms=float(np.percentile(np.array(self.samples)[:,1],95)),
                            pair={'frame_id':frame_id,'captured_at':acquired,'a':raw(a_result,a_status.get('swap',False)),'b':raw(result,b.swap),
                                  'a_inference_ms':a_status['inference_ms'],'a_mapping_ms':a_status['mapping_ms'],
                                  'b_inference_ms':payload['inference_ms'],'roundtrip_ms':(received-started)*1000,
                                  'b_mapping_ms':b.mapping_ms,'processing_ms':(ended-acquired)*1000})
                except Exception as exc:
                    connection.close()
                    message='WiLoR 后端尚未启动，等待推理服务' if isinstance(exc,ConnectionRefusedError) else str(exc)
                    with self.lock:self.data.update(ready=False,error=message)
                    if self.stop.wait(.5):break
        finally:
            connection.close()
            self.close_recording()
