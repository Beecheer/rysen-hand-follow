"""Camera-only thumb trajectory calibration and continuous IK posture prior.

Robot references are geometric fits, not measured joint-angle ground truth.
The existing finger/contact calibration is never overwritten.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import numpy as np

from .geometry import unit
from .calibration import calibration_cues, correct_fingers, finger_targets, canonical_hand, TIP_IDS
from .kinematics import ThumbRetargeter

STEPS=('rest','plane','lift','flex','index','middle','ring','pinky')


def signature(profile):
    keys=('created_at','model_sha256','context','alignment','contacts','structure')
    if 'depth_reference' in profile:keys+=('depth_reference',)
    return hashlib.sha256(json.dumps({k:profile.get(k) for k in keys},sort_keys=True).encode()).hexdigest()


def feature(target):
    target=np.asarray(target,float)
    if target.shape!=(4,3) or not np.isfinite(target).all():raise ValueError('拇指骨段无效')
    return np.concatenate([unit(v) for v in np.diff(target,axis=0)])


def metrics(target):
    d=feature(target).reshape(3,3)
    return np.array([np.arctan2(d[0,1],d[0,0]),np.arcsin(np.clip(d[0,2],-1,1)),
                     np.arccos(np.clip(d[0]@d[1],-1,1))+np.arccos(np.clip(d[1]@d[2],-1,1))])


def cycles(values):
    values=np.asarray(values,float);lo,hi=np.percentile(values,[15,85])
    state=None;transitions=0
    for v in values:
        current=0 if v<lo+.25*(hi-lo) else 1 if v>lo+.75*(hi-lo) else None
        if current is not None:
            if state is not None and current!=state:transitions+=1
            state=current
    return transitions//2


def validate(model,base,bounds):
    if model.get('schema')!=1 or model.get('base_signature')!=signature(base):
        raise ValueError('基础标定已改变，请重新采集拇指专项标定')
    x=np.asarray(model['features'],float);q=np.asarray(model['postures'],float)
    if x.ndim!=2 or x.shape[1]!=9 or not 10<=len(x)<=64 or q.shape!=(len(x),4):
        raise ValueError('拇指专项参考维度无效')
    if not np.isfinite(x).all() or not np.isfinite(q).all() or not .04<=model['radius']<=.5:
        raise ValueError('拇指专项参考数值无效')
    if not np.allclose(np.linalg.norm(x.reshape(-1,3,3),axis=2),1,atol=.01):
        raise ValueError('拇指专项骨段方向无效')
    for j in range(4):
        lo,hi=bounds[f'left_thumb_j{j}']
        if np.any(q[:,j]<lo-1e-8) or np.any(q[:,j]>hi+1e-8):raise ValueError('拇指参考超出当前限位，请重新生成')
    return model


def posture_prior(model,target):
    """Gaussian blend of nearby postures; fade out beyond sampled support."""
    x=feature(target);refs=np.asarray(model['features']);q=np.asarray(model['postures'])
    distance=np.sqrt(np.mean((refs-x)**2,axis=1))
    radius=model['radius'];nearest=float(distance.min())
    confidence=float(np.clip((2*radius-nearest)/radius,0,1))
    if confidence==0:return None
    weights=np.exp(-.5*(distance/radius)**2);weights/=weights.sum()
    return {'q':weights@q,'weight':.45*confidence}


class ThumbCalibrationWizard:
    def __init__(self,calibration,robot,bounds,specs):
        self.base=calibration;self.robot=robot;self.bounds=bounds;self.specs=specs
        self.path=calibration.path.with_name(calibration.path.stem+'-thumb.json')
        self.samples={};self.collecting=None;self.buffer=[];self.rejections={}
        self.model=None;self.enabled=False;self.message='先保持自然放松，再按顺序采集七组往返动作'
        self._base_id=None;self._compatible=False
        if self.path.exists():
            try:
                self.model=validate(json.loads(self.path.read_text(encoding='utf-8')),self.base.profile,self.bounds)
                self.enabled=True;self.message='已加载拇指专项参考；可停用对比或重新采集'
            except (ValueError,KeyError,TypeError,OSError,AttributeError) as exc:self.message=str(exc)

    def active_model(self):
        if not self.enabled or not self.base.enabled or self.base.profile is None:return None
        if id(self.base.profile)!=self._base_id:
            self._base_id=id(self.base.profile)
            self._compatible=self.model is not None and self.model.get('base_signature')==signature(self.base.profile)
        return self.model if self._compatible else None

    def start(self,step,now):
        if step not in STEPS:raise ValueError('未知拇指标定动作')
        if not self.base.enabled or not self.base.profile:raise ValueError('请先启用已保存的八姿态基础标定')
        if self.collecting or self.base.collecting:raise ValueError('已有采样进行中，请先完成或取消')
        if step!='rest' and 'rest' not in self.samples:raise ValueError('请先采自然放松参考')
        if self.samples and self.samples['rest']['base_signature']!=signature(self.base.profile):
            if step!='rest':raise ValueError('基础标定已改变，请从自然放松重新采集')
        self.collecting={'step':step,'start':now,'seconds':6 if step=='rest' else 14}
        self.buffer=[];self.rejections={};self.message='准备 2 秒；自然放松保持不动，其余动作缓慢往返 2–3 次'

    def reject(self,reason,now):
        if self.collecting and now-self.collecting['start']>=2:
            self.rejections[reason]=self.rejections.get(reason,0)+1

    def ingest(self,angles,world,clean_world,handedness,now,front_ok=True):
        if not self.collecting:return
        self.advance(now)
        if not self.collecting or now-self.collecting['start']<2:return
        if not front_ok:self.reject('请保持掌心正对摄像头、拇指完整可见',now);return
        try:
            solver=ThumbRetargeter(self.robot,self.specs)
            target=solver.target_from_landmarks(clean_world,handedness)
            valid=[f for f in TIP_IDS if all(angles.get(f,{}).get(k) is not None for k in ('mcp','pip'))]
            positions=finger_targets(correct_fingers(angles,self.base.profile),self.bounds)
            cues=calibration_cues(self.base.profile,world,handedness,self.robot,positions,valid)
            p=canonical_hand(world,handedness)
            step=self.collecting['step']
            distance=None if step not in TIP_IDS else float(np.linalg.norm(p[4]-p[TIP_IDS[step]]))
            if step in TIP_IDS and step not in valid:raise ValueError('目标手指观测不完整')
            if len(self.buffer)<220:
                self.buffer.append({'target':target.tolist(),'tip':cues['tip'].tolist(),'strength':cues['strength'],
                                    'feature':feature(target).tolist(),'metrics':metrics(target).tolist(),'distance':distance})
        except (ValueError,KeyError) as exc:self.reject(str(exc),now)

    def advance(self,now):
        if not self.collecting or now-self.collecting['start']<self.collecting['seconds']:return
        step=self.collecting['step'];self.collecting=None
        minimum=20 if step=='rest' else 35
        if len(self.buffer)<minimum:
            self.message=f'有效帧不足 {minimum} 帧，请恢复识别后重采'
            if self.rejections:self.message+='；'+max(self.rejections,key=self.rejections.get)
            return
        x=np.asarray([b['feature'] for b in self.buffer]);center=np.median(x,axis=0)
        spread=float(np.percentile(np.sqrt(np.mean((x-center)**2,axis=1)),75))
        if step=='rest':
            if spread>.08:self.message='放松参考变化过大，请保持拇指静止后重采';return
            self.samples={};coordinate=np.zeros(len(x));count=0
        else:
            if step in TIP_IDS:
                coordinate=np.asarray([b['distance'] for b in self.buffer])
                if np.percentile(coordinate,15)>.55:
                    self.message='尚未接近目标指尖，请确认真实轻触再缓慢返回';return
                minimum_span=.12
            else:
                index={'plane':0,'lift':1,'flex':2}[step]
                coordinate=np.asarray([b['metrics'][index] for b in self.buffer])
                if step=='plane':coordinate=np.unwrap(coordinate)
                minimum_span=.15
            span=float(np.ptp(np.percentile(coordinate,[10,90])))
            count=cycles(coordinate)
            if span<minimum_span:self.message='该方向运动范围不足，请保持手掌不动、缓慢增加拇指动作后重采';return
            if count<2:self.message='未采到两次完整往返，请在采样期间缓慢往返 2–3 次';return
        # Store robust representative knots, not video or every captured frame.
        knots=[]
        bins=[np.arange(len(x))] if step=='rest' else np.array_split(np.argsort(coordinate),7)
        for indices in bins:
            target=np.median([self.buffer[i]['target'] for i in indices],axis=0)
            knots.append({'target':target.tolist(),'feature':feature(target).tolist(),
                          'tip':np.median([self.buffer[i]['tip'] for i in indices],axis=0).tolist(),
                          'strength':float(np.median([self.buffer[i]['strength'] for i in indices]))})
        self.samples[step]={'knots':knots,'valid_frames':len(x),'spread':spread,'cycles':count,
                            'base_signature':signature(self.base.profile),
                            'range':np.percentile([b['metrics'] for b in self.buffer],[5,95],axis=0).tolist()}
        self.message=f'已完成 {len(self.samples)}/8 步；全部完成后可生成并应用'
        self.buffer=[]

    def build(self):
        if self.collecting or set(self.samples)!=set(STEPS):raise ValueError('请先完成八步拇指专项采集')
        if not self.base.enabled or not self.base.profile:raise ValueError('请先启用与采样一致的基础标定')
        stamp=signature(self.base.profile)
        if any(s['base_signature']!=stamp for s in self.samples.values()):raise ValueError('基础标定已改变，请重新采集')
        solver=ThumbRetargeter(self.robot,self.specs)
        neutral=np.clip(self.base.profile['neutral_thumb'],solver.lower,solver.upper)
        features=[];postures=[];errors=[]
        for step in STEPS:
            solver.previous=neutral.copy()
            for knot in self.samples[step]['knots']:
                if step=='rest':q=neutral.copy()
                else:
                    cues={'tip':np.asarray(knot['tip']),'strength':knot['strength']}
                    result=solver.fit_target(knot['target'],temporal_weight=.2,cues=cues)
                    q=np.array([result[f'j{i}'] for i in range(4)])
                    errors.append(solver.tip_error_mm)
                features.append(knot['feature']);postures.append(q.tolist())
        radius=float(np.clip(.06+2*self.samples['rest']['spread'],.06,.16))
        model={'schema':1,'created_at':datetime.now(timezone.utc).isoformat(),'base_signature':stamp,
               'features':features,'postures':postures,'radius':radius,
               'samples':deepcopy(self.samples),'fit_error_p95_mm':float(np.percentile(errors,95)),
               'reference_kind':'geometric_ik_not_joint_ground_truth'}
        return validate(model,self.base.profile,self.bounds)

    def apply(self):
        model=self.build()
        if self.path.exists():
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
            self.path.with_name(f'{self.path.stem}-backup-{stamp}.json').write_bytes(self.path.read_bytes())
        temporary=self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(model,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(self.path)
        self.model=model;self.enabled=True;self._base_id=None
        self.message='专项参考已应用；请恢复识别先观察预览，机械手保持失能'

    def cancel(self):
        self.collecting=None;self.buffer=[];self.message='已取消当前采样，已完成步骤保留'

    def snapshot(self,now):
        self.advance(now);c=self.collecting
        return {'steps':list(STEPS),'samples':{k:{key:s[key] for key in ('valid_frames','cycles','spread')} for k,s in self.samples.items()},
                'collecting':None if c is None else c['step'],'progress':0 if c is None else min(1,(now-c['start'])/c['seconds']),
                'phase':'prepare' if c and now-c['start']<2 else 'sample','valid_frames':len(self.buffer),
                'ready':set(self.samples)==set(STEPS) and c is None,'enabled':self.active_model() is not None,
                'has_profile':self.model is not None,'compatible':self.active_model() is not None if self.enabled else
                bool(self.model and self.base.profile and self.model.get('base_signature')==signature(self.base.profile)),
                'rejections':dict(self.rejections),'message':self.message}
