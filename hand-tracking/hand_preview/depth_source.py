"""Shared D435 acquisition/fusion imports and metric calibration contract."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np
from .calibration import CalibrationWizard,canonical_hand
from .geometry import FINGERS,palm_frame
from .human_pose import image_frame

# One algorithm implementation, shared by the standalone diagnostic and app.
TOOLS=Path(__file__).resolve().parents[2]/'realsense-tools'
if str(TOOLS) not in sys.path:sys.path.insert(0,str(TOOLS))
from depth_camera import Camera
from hand_fusion import PARENTS,CHILDREN,PALM

ALGORITHM='d435-metric-fusion-v1'


def camera_context(status,swap=False):
    intr=status['intrinsics']
    if not intr or not status['device']:raise ValueError('D435 尚未就绪')
    fingerprint=hashlib.sha256(json.dumps(intr,sort_keys=True).encode()).hexdigest()
    return {'tracker':'d435','algorithm':ALGORITHM,'serial':status['device']['serial'],
            'intrinsics_sha256':fingerprint,'depth_scale_m':status['depth_scale_m'],
            'firmware':status['device'].get('firmware'),
            'swap_handedness':bool(swap)}


def validate_reference(reference):
    if not isinstance(reference,dict) or reference.get('algorithm')!=ALGORITHM:
        raise ValueError('D435 标定缺少匹配的米制骨长参考，请重新采集中立位')
    lengths=np.asarray(reference.get('lengths_m'),float)
    if lengths.shape!=(20,) or not np.isfinite(lengths).all() or np.any(lengths<.005) or np.any(lengths>.13):
        raise ValueError('D435 米制骨长参考无效')
    if reference.get('valid_frames',0)<20:raise ValueError('D435 米制参考有效帧不足')
    return reference


def reference_id(reference):
    return hashlib.sha256(json.dumps(reference,sort_keys=True).encode()).hexdigest() if reference is not None else None


def structure_observation(points,pixels,label,hand):
    p,basis,length=palm_frame(points,label)
    _,_,aspect=image_frame(pixels)
    canonical=canonical_hand(points,label)
    lengths={f:(np.linalg.norm(np.diff(p[list(ids)],axis=0),axis=1)/length).tolist() for f,ids in FINGERS.items()}
    lengths['thumb']=(np.linalg.norm(np.diff(p[1:5],axis=0),axis=1)/length).tolist()
    return {'canonical':canonical.tolist(),'camera_normal':basis[:,2].tolist(),
            'image_aspect':aspect,'bone_lengths':lengths,'metric_points_m':p.tolist(),
            'depth_quality':deepcopy(hand['quality']),'depth_sources':hand['sources'],
            'sync_valid':hand['sync_valid'],'reliable_palm_points':hand['reliable_palm_points']}


def landmark_result(report,width,height):
    result=NS(hand_landmarks=[],hand_world_landmarks=[],handedness=[],depth_hand=None)
    hands=report.get('hands',[])
    if not hands:return result
    hand=hands[0]
    if not hand.get('fused_points_m'):
        result.backend_error=report.get('message','等待可靠 D435 深度');return result
    result.hand_landmarks=[[NS(x=x/width,y=y/height,z=0.) for x,y in hand['pixels']]]
    result.hand_world_landmarks=[[NS(x=x,y=y,z=z) for x,y,z in hand['fused_points_m']]]
    result.handedness=[[NS(category_name=hand['label'],score=hand.get('score',0.))]]
    result.depth_hand=hand
    return result


class DepthCalibrationWizard(CalibrationWizard):
    """Eight poses plus a metric neutral reference, saved in the same transaction."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if self.profile is not None:
            try:validate_reference(self.profile.get('depth_reference'))
            except ValueError as exc:self.enabled=False;self.profile=None;self.message=str(exc)

    def ingest(self,angles,world,handedness,now,observation=None):
        if self.collecting:
            pose=self.collecting['pose'];o=observation or {};q=o.get('depth_quality',{})
            minimum=10 if pose=='neutral' else 5
            if not o.get('sync_valid') or o.get('reliable_palm_points',0)<3 or q.get('depth_constraints',0)<minimum or q.get('bone_error_p95_mm',100)>=15:
                self.reject('可靠掌部或逐指深度不足，请调整距离和遮挡',now);self.advance(now);return
        super().ingest(angles,world,handedness,now,observation)

    def advance(self,now):
        collecting=deepcopy(self.collecting);buffer=list(self.buffer)
        old=self.samples.get('neutral')
        super().advance(now)
        if collecting and self.collecting is None and collecting['pose']=='neutral' and self.samples.get('neutral') is not old:
            points=[np.asarray(b['observation']['metric_points_m']) for b in buffer if b.get('observation',{}).get('metric_points_m')]
            if len(points)>=20:
                bones=[np.linalg.norm(p[CHILDREN]-p[PARENTS],axis=1) for p in points]
                reference={'algorithm':ALGORITHM,'lengths_m':np.median(bones,axis=0).tolist(),
                           'valid_frames':len(points),'units':'meters'}
                try:self.samples['neutral']['depth_reference']=validate_reference(reference)
                except ValueError as exc:self.samples={};self.pending=None;self.message=str(exc)
            else:self.samples={};self.pending=None;self.message='中立位缺少米制观测，请重新采集'
        if self.pending is not None:
            try:self.pending['depth_reference']=deepcopy(validate_reference(self.samples['neutral'].get('depth_reference')))
            except ValueError as exc:self.pending=None;self.message=str(exc)

    def apply(self):
        if self.pending is not None:validate_reference(self.pending.get('depth_reference'))
        super().apply()

    def reference(self):
        if self.collecting and self.collecting['pose']=='neutral':return None
        if not self.enabled and self.profile is not None and not self.collecting:return None
        if 'neutral' in self.samples:return self.samples['neutral'].get('depth_reference')
        return self.profile.get('depth_reference') if self.enabled and self.profile else None
