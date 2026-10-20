"""Loopback inference only. No camera, calibration or robot control here."""
import inspect
import threading
import time
from pathlib import Path
from contextlib import asynccontextmanager
from runpy import run_path

RUNTIME_POLICY=run_path(str(Path(__file__).resolve().parents[1]/'vision_runtime.py'))['apply_process_policy']()

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request
import uvicorn

# MANO's legacy chumpy pickle expects aliases removed by NumPy/Python.
for name, value in {'bool':bool,'int':int,'float':float,'complex':complex,
                    'object':object,'str':str,'unicode':str}.items():
    if name not in np.__dict__:setattr(np,name,value)
if not hasattr(inspect,'getargspec'):inspect.getargspec=inspect.getfullargspec

state={'ready':False,'error':None,'device':None,'runtime_power_policy':RUNTIME_POLICY};pipeline=None
infer_lock=threading.Lock()

def load():
    global pipeline
    try:
        import faulthandler
        faulthandler.dump_traceback_later(60)
        state['stage']='import torch'
        import torch
        cv2.setNumThreads(1)
        torch.set_num_threads(2)
        state['stage']='import WiLoR dependencies'
        from wilor_mini.pipelines.wilor_hand_pose3d_estimation_pipeline import WiLorHandPose3dEstimationPipeline
        if not torch.cuda.is_available():raise RuntimeError('CUDA is unavailable; GPU inference is required for this experiment')
        state['device']=torch.cuda.get_device_name(0)
        state['stage']='load model weights'
        pipeline=WiLorHandPose3dEstimationPipeline(device=torch.device('cuda'),dtype=torch.float16,
            verbose=False,wilor_pretrained_dir=str(Path(__file__).parent/'models'))
        # Warm up both model and CUDA kernels using a supplied box on blank data.
        state['stage']='warm up GPU'
        pipeline.predict_with_bboxes(np.zeros((480,640,3),np.uint8),np.array([[180,80,460,400]]),np.array([0.]))
        torch.cuda.synchronize()
        state['runtime_threads']={'opencv':cv2.getNumThreads(),'torch':torch.get_num_threads()}
        state['ready']=True
        state['stage']='ready'
    except Exception as exc:
        import traceback
        traceback.print_exc()
        state['error']=f'{type(exc).__name__}: {exc}'
    finally:
        faulthandler.cancel_dump_traceback_later()

@asynccontextmanager
async def lifespan(app):
    threading.Thread(target=load,daemon=True).start()
    yield

app=FastAPI(lifespan=lifespan,docs_url=None,redoc_url=None)
server=None

@app.get('/health')
def health():return dict(state,service='rysen-wilor-inference',version=1)

@app.post('/shutdown')
def shutdown(request:Request):
    if request.headers.get('origin') or request.headers.get('x-rysen-control')!='stop':
        raise HTTPException(403,'Local script control required')
    if server:server.should_exit=True
    return {'ok':True}

@app.post('/infer')
async def infer(request:Request):
    if request.headers.get('origin'):raise HTTPException(403,'Use the local comparison service')
    if not state['ready']:raise HTTPException(503,state['error'] or 'Models downloading/loading')
    content=await request.body()
    if len(content)>2_000_000:raise HTTPException(413,'Frame too large')
    frame=cv2.imdecode(np.frombuffer(content,np.uint8),cv2.IMREAD_COLOR)
    if frame is None:raise HTTPException(400,'Invalid JPEG')
    import asyncio
    def run():
        import torch
        with infer_lock:
            torch.cuda.synchronize();start=time.perf_counter()
            results=pipeline.predict(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            torch.cuda.synchronize();elapsed=(time.perf_counter()-start)*1000
            state['last_inference_ms']=elapsed
            state['last_hand_count']=len(results)
            detector_results=getattr(pipeline.hand_detector.predictor,'results',None)
            if detector_results:state['last_detector_ms']=dict(detector_results[0].speed)
        hands=[]
        for hand in results:
            pred=hand.get('wilor_preds')
            if pred is None:continue
            world=np.asarray(pred['pred_keypoints_3d'])[0]
            image=np.asarray(pred['pred_keypoints_2d'])[0]/np.array([frame.shape[1],frame.shape[0]])
            if world.shape!=(21,3) or image.shape!=(21,2) or not np.isfinite(world).all() or not np.isfinite(image).all():continue
            hands.append({'label':'Right' if hand['is_right'] else 'Left','world':world.tolist(),'image':image.tolist()})
        return {'hands':hands,'inference_ms':elapsed}
    return await asyncio.to_thread(run)

if __name__=='__main__':
    server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=8093,access_log=False))
    server.run()
