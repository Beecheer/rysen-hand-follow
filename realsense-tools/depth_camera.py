"""Local D435 diagnostics with optional depth hand fusion. No robot transport."""
import argparse
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
from copy import deepcopy

import cv2
import numpy as np
import pyrealsense2 as rs
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse
import uvicorn

ROOT=Path(__file__).resolve().parent
SERVICE='rysen-realsense-basic'


def device_info(device):
    fields={'name':rs.camera_info.name,'serial':rs.camera_info.serial_number,
            'firmware':rs.camera_info.firmware_version,'usb':rs.camera_info.usb_type_descriptor}
    return {k:device.get_info(v) if device.supports(v) else None for k,v in fields.items()}


def enumerate_devices():
    devices=[]
    for device in rs.context().query_devices():
        info=device_info(device);info['sensors']=[]
        for sensor in device.query_sensors():
            profiles=[]
            for profile in sensor.get_stream_profiles():
                if profile.is_video_stream_profile():
                    p=profile.as_video_stream_profile()
                    profiles.append({'stream':str(p.stream_type()),'index':p.stream_index(),
                                     'width':p.width(),'height':p.height(),'fps':p.fps(),'format':str(p.format())})
            info['sensors'].append({'name':sensor.get_info(rs.camera_info.name),'profiles':profiles})
        devices.append(info)
    return devices


def intrinsics_dict(intr):
    return {key:getattr(intr,key) for key in ('width','height','fx','fy','ppx','ppy')} | {
        'model':str(intr.model),'coeffs':list(intr.coeffs)}


def sample_depth(depth,scale,x,y,radius=2):
    h,w=depth.shape
    if not 0<=x<w or not 0<=y<h:raise ValueError('像素超出画面')
    patch=depth[max(0,y-radius):min(h,y+radius+1),max(0,x-radius):min(w,x+radius+1)]
    valid=patch[patch>0]*scale
    return {'pixel':[x,y],'raw_z_m':None if depth[y,x]==0 else float(depth[y,x]*scale),
            'patch_z_m':None if valid.size==0 else float(np.median(valid)),
            'patch_valid_ratio':float(valid.size/patch.size),
            'patch_std_mm':None if valid.size==0 else float(np.std(valid)*1000)}


class Camera:
    def __init__(self,serial=None,track_hands=False,encode_preview=True):
        self.encode_preview=encode_preview
        self.serial=serial;self.lock=threading.Lock();self.stop=threading.Event();self.paused=False
        self.state='STARTING';self.error=None;self.info=None;self.scale=None;self.intr=None
        self.depth=None;self.color=None;self.jpeg=None;self.frame_at=None;self.frames=0;self.fps=0
        self.valid_ratio=0.;self.in_range_ratio=0.;self.frame_id=None;self.skew_ms=None
        self.near=.2;self.far=1.5;self.selected=None;self.last_capture=None
        self.stream_fps=None
        self.released=True
        self.track_hands=track_hands;self.hand_report={'enabled':track_hands,'state':'STARTING','hands':[]}
        self.reset_hands=threading.Event();self.swap_hands=False
        self.hand_reference=None;self.reference_changed=threading.Event()
        self.articulated_hands=False

    def run(self):
        while not self.stop.is_set():
            if self.paused:
                with self.lock:
                    self.state='ERROR' if self.error else 'PAUSED';self.depth=self.color=None;self.jpeg=None;self.frame_at=None
                    self.hand_report={'enabled':self.track_hands,'state':'PAUSED','hands':[]}
                self.stop.wait(.1);continue
            pipeline=rs.pipeline();started=False;tracker=None
            try:
                with self.lock:self.error=None;self.state='STARTING'
                config=rs.config()
                devices=list(rs.context().query_devices())
                if self.serial:devices=[d for d in devices if d.get_info(rs.camera_info.serial_number)==self.serial]
                if not devices:raise RuntimeError('未找到 RealSense 相机，请检查数据线和连接')
                if len(devices)>1:raise RuntimeError('发现多台 RealSense，请用 -Serial 指定设备')
                info=device_info(devices[0]);config.enable_device(info['serial'])
                fps=15 if (info['usb'] or '').startswith('2') else 30
                with self.lock:self.info=info;self.stream_fps=fps
                config.enable_stream(rs.stream.depth,640,480,rs.format.z16,fps)
                config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,fps)
                profile=pipeline.start(config);started=True
                with self.lock:self.released=False
                device=profile.get_device();scale=device.first_depth_sensor().get_depth_scale()
                align=rs.align(rs.stream.color);previous=None
                if self.track_hands:
                    from hand_fusion import HandTracker
                    from prepare_hand_model import MODEL
                    tracker=HandTracker(MODEL)
                    tracker.fusion.observe_only=self.articulated_hands
                    with self.lock:tracker.fusion.set_reference(deepcopy(self.hand_reference))
                with self.lock:self.info=device_info(device);self.scale=scale;self.error=None
                while not self.stop.is_set() and not self.paused:
                    frames=pipeline.wait_for_frames(3000)
                    aligned=align.process(frames);d=aligned.get_depth_frame();c=aligned.get_color_frame()
                    if not d or not c:continue
                    depth=np.asanyarray(d.get_data()).copy();color=np.asanyarray(c.get_data()).copy()
                    intr=d.profile.as_video_stream_profile().intrinsics
                    at=time.perf_counter()
                    near,far=self.near,self.far
                    meters=depth*scale
                    colored=None
                    if self.encode_preview or tracker or self.selected:
                        normalized=np.clip((meters-near)/(far-near),0,1)
                        colored=cv2.applyColorMap((normalized*255).astype(np.uint8),cv2.COLORMAP_TURBO)
                        colored[depth==0]=0
                    hand_report={'enabled':False,'hands':[]}
                    if tracker:
                        if self.reference_changed.is_set():
                            with self.lock:
                                reference=deepcopy(self.hand_reference);self.reference_changed.clear()
                            tracker.fusion.set_reference(reference)
                        if self.reset_hands.is_set():tracker.reset();self.reset_hands.clear()
                        skew=abs(d.get_timestamp()-c.get_timestamp())
                        try:
                            hand_report=tracker.process(color,depth,scale,intr,at,skew,self.swap_hands)
                            from hand_fusion import draw_hands
                            draw_hands(color,hand_report);draw_hands(colored,hand_report)
                        except Exception as exc:
                            hand_report={'enabled':True,'state':'ERROR','message':str(exc),'hands':[]}
                    if self.selected:
                        x,y=self.selected
                        for image in (color,colored):cv2.drawMarker(image,(x,y),(255,255,255),cv2.MARKER_CROSS,14,1)
                    ok,jpeg=cv2.imencode('.jpg',np.hstack((color,colored)),[cv2.IMWRITE_JPEG_QUALITY,85]) if self.encode_preview else (False,None)
                    with self.lock:
                        self.depth=depth;self.color=np.asanyarray(c.get_data()).copy();self.intr=intr
                        self.jpeg=jpeg.tobytes() if ok else None;self.frame_at=at;self.frames+=1
                        if previous is not None:self.fps=.85*self.fps+.15/max(at-previous,.001) if self.fps else 1/(at-previous)
                        self.valid_ratio=float(np.mean(depth>0));self.in_range_ratio=float(np.mean((meters>=near)&(meters<=far)&(depth>0)))
                        self.frame_id=d.get_frame_number();self.skew_ms=abs(d.get_timestamp()-c.get_timestamp());self.state='RUNNING'
                        self.hand_report=hand_report
                    previous=at
            except Exception as exc:
                with self.lock:self.error=str(exc);self.state='ERROR';self.depth=self.color=None;self.jpeg=None;self.frame_at=None
                # Explicit Resume retries after unplugging, occupancy or a stream error.
                self.paused=True
            finally:
                if tracker:
                    try:tracker.close()
                    except Exception as exc:
                        with self.lock:self.error=str(exc)
                if started:
                    try:
                        pipeline.stop()
                        with self.lock:self.released=True
                    except Exception as exc:
                        with self.lock:self.error=str(exc)
        with self.lock:self.state='STOPPED'

    def measure(self,x,y):
        with self.lock:
            if self.depth is None or self.frame_at is None or time.perf_counter()-self.frame_at>.5:
                raise ValueError('没有新鲜深度帧，请恢复相机')
            result=sample_depth(self.depth,self.scale,x,y)
            z=result['raw_z_m']
            result['xyz_m']=None if z is None else rs.rs2_deproject_pixel_to_point(self.intr,[x,y],z)
            result['frame_id']=self.frame_id
            return result

    def snapshot(self):
        with self.lock:
            selected=self.selected
            data={'service':SERVICE,'state':self.state,'error':self.error,'device':self.info,
                  'fps':self.fps,'frames':self.frames,'depth_scale_m':self.scale,
                  'valid_ratio':self.valid_ratio,'in_range_ratio':self.in_range_ratio,
                  'near_m':self.near,'far_m':self.far,'timestamp_skew_ms':self.skew_ms,
                  'stream_fps':self.stream_fps,
                  'camera_released':self.released,
                  'age_ms':None if self.frame_at is None else (time.perf_counter()-self.frame_at)*1000,
                  'intrinsics':None if self.intr is None else intrinsics_dict(self.intr),'last_capture':self.last_capture}
            data['hand_tracking']=deepcopy(self.hand_report)
            data['hand_tracking']['swap_handedness']=self.swap_hands
            if self.frame_at is None or time.perf_counter()-self.frame_at>.5:
                data['hand_tracking']['hands']=[]
                if data['hand_tracking'].get('enabled') and self.state=='RUNNING':
                    data['hand_tracking'].update(state='STALE',message='等待新鲜相机帧，已清空旧骨架')
        try:data['measurement']=None if selected is None else self.measure(*selected)
        except ValueError:data['measurement']=None
        return data

    def capture(self):
        with self.lock:
            if self.depth is None or self.frame_at is None or time.perf_counter()-self.frame_at>.5:raise ValueError('没有新鲜图像')
            depth=self.depth.copy();color=self.color.copy();intr=intrinsics_dict(self.intr)
            metadata={'service':SERVICE,'device':self.info,'depth_scale_m':self.scale,'intrinsics':intr,
                      'depth_aligned_to':'color','frame_id':self.frame_id,'timestamp_skew_ms':self.skew_ms,
                      'note':'Z16 depth is camera-axis Z, not radial range. Zero is invalid.'}
        folder=ROOT/'captures'/datetime.now().astimezone().strftime('%Y%m%d-%H%M%S-%f')
        folder.mkdir(parents=True)
        if not cv2.imwrite(str(folder/'color.png'),color) or not cv2.imwrite(str(folder/'depth-z16.png'),depth):
            raise ValueError('保存图像失败')
        (folder/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        self.last_capture=str(folder)
        return self.last_capture


def make_app(camera,shutdown):
    @asynccontextmanager
    async def lifespan(app):
        worker=threading.Thread(target=camera.run,name='realsense-camera',daemon=True);worker.start()
        yield
        camera.stop.set();worker.join(5)
    app=FastAPI(lifespan=lifespan)

    @app.get('/')
    def home():return FileResponse(ROOT/'index.html',headers={'Cache-Control':'no-store'})

    @app.get('/api/status')
    def status():return camera.snapshot()

    @app.get('/video')
    async def video():
        import asyncio
        async def stream():
            last=0
            while not camera.stop.is_set():
                with camera.lock:frame=camera.jpeg;number=camera.frames
                if frame and number!=last:
                    last=number
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+frame+b'\r\n'
                await asyncio.sleep(1/15)
        return StreamingResponse(stream(),media_type='multipart/x-mixed-replace; boundary=frame')

    @app.post('/api/{action}')
    async def action(action:str,request:Request):
        if request.headers.get('origin')!=f'http://{request.headers.get("host")}':raise HTTPException(403,'Same-origin required')
        body=await request.json()
        try:
            if action=='stop':camera.stop.set();shutdown()
            elif action=='pause':camera.paused=True
            elif action=='resume':camera.paused=False
            elif action=='select':
                x,y=int(body['x']),int(body['y']);camera.measure(x,y);camera.selected=(x,y)
            elif action=='range':
                near,far=float(body['near']),float(body['far'])
                if not np.isfinite([near,far]).all() or not 0<near<far<=10:raise ValueError('显示范围需要满足 0 < 近端 < 远端 ≤ 10 米')
                with camera.lock:camera.near=near;camera.far=far
            elif action=='capture':return {'path':camera.capture()}
            elif action=='hand-reset':
                if not camera.track_hands:raise ValueError('当前为基础相机模式，请启动手部深度模式')
                camera.reset_hands.set()
            elif action=='hand-swap':
                if not camera.track_hands:raise ValueError('当前为基础相机模式，请启动手部深度模式')
                camera.swap_hands=not camera.swap_hands;camera.reset_hands.set()
            else:raise HTTPException(404,'Unknown action')
        except (ValueError,KeyError,TypeError) as exc:raise HTTPException(400,str(exc))
        return {'ok':True}
    return app


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--list',action='store_true');parser.add_argument('--probe',type=int,default=0)
    parser.add_argument('--serial');parser.add_argument('--port',type=int,default=8095)
    parser.add_argument('--hands',action='store_true',help='MediaPipe + per-joint depth constraints')
    args=parser.parse_args()
    if args.list:
        print(json.dumps(enumerate_devices(),ensure_ascii=False,indent=2));return
    if args.hands:
        from prepare_hand_model import ensure_model
        ensure_model()
    camera=Camera(args.serial,track_hands=args.hands)
    if args.probe:
        worker=threading.Thread(target=camera.run);worker.start();deadline=time.perf_counter()+args.probe+10
        samples=[];last=0;first_at=None
        try:
            while time.perf_counter()<deadline:
                data=camera.snapshot()
                if data['error']:raise RuntimeError(data['error'])
                if data['frames']!=last:
                    last=data['frames'];samples.append(data)
                    if first_at is None:first_at=time.perf_counter()
                    if time.perf_counter()-first_at>=args.probe:break
                time.sleep(.01)
        finally:camera.stop.set();worker.join(6)
        if len(samples)<10:raise RuntimeError('Insufficient camera frames')
        if worker.is_alive() or not camera.released:raise RuntimeError('Camera release not confirmed')
        report={'service':SERVICE,'device':samples[-1]['device'],'frames':len(samples),
                'fps':samples[-1]['fps'],'depth_scale_m':samples[-1]['depth_scale_m'],
                'stream_fps':samples[-1]['stream_fps'],
                'mean_valid_ratio':float(np.mean([s['valid_ratio'] for s in samples])),
                'intrinsics':samples[-1]['intrinsics'],'timestamp_skew_ms':samples[-1]['timestamp_skew_ms'],
                'pipeline_released':not worker.is_alive(),'images_saved':False}
        (ROOT/'logs').mkdir(exist_ok=True)
        (ROOT/'logs'/'probe.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2));return
    server=uvicorn.Server(uvicorn.Config(make_app(camera,lambda:setattr(server,'should_exit',True)),host='127.0.0.1',port=args.port,log_level='warning'))
    server.run()


if __name__=='__main__':main()
