"""Two straight-hand poses. Separate schema/path from legacy eight-pose data."""
from copy import deepcopy
from datetime import datetime,timezone
import json
from pathlib import Path
import numpy as np
from .calibration import model_hash,CalibrationWizard
from .geometry import FINGERS
from .depth_articulation import PALM,thumb_features
from .depth_source import ALGORITHM,validate_reference
from hand_fusion import PARENTS,CHILDREN

VERSION='d435-articulated-v2'
POSES=('neutral','together')
DEFAULT_THUMB={'fixed_terminal_deg':12.,'neutral_deg':[26.,10.,18.],
               'gains':[-1.,1.,1.]}


def validate_thumb(config,bounds):
    result=deepcopy(config)
    fixed=float(result['fixed_terminal_deg']);neutral=np.asarray(result['neutral_deg'],float)
    gains=np.asarray(result['gains'],float)
    if not np.isfinite(fixed) or neutral.shape!=(3,) or gains.shape!=(3,) or not np.isfinite([neutral,gains]).all():
        raise ValueError('拇指直接映射配置无效')
    lo,hi=bounds['left_thumb_j3']
    if not lo<=np.radians(fixed)<=hi:raise ValueError('末端固定角度超出当前 SDK 软件限位')
    if np.any(abs(gains)>3):raise ValueError('拇指映射比例应在 -3 到 3 之间')
    for i,v in enumerate(np.radians(neutral)):
        lo,hi=bounds[f'left_thumb_j{i}']
        if not lo<=v<=hi:raise ValueError('拇指参考角度超出软件限位')
    return {'fixed_terminal_deg':fixed,'neutral_deg':neutral.tolist(),'gains':gains.tolist()}


def validate(profile,context,bounds):
    if profile.get('schema')!=4 or profile.get('algorithm')!=VERSION or profile.get('model_sha256')!=model_hash():
        raise ValueError('需要新版两步 D435 标定，旧标定文件保留')
    if profile.get('context')!=context:raise ValueError('相机身份、内参或左右手设置改变，请重新标定')
    validate_reference(profile.get('depth_reference'))
    s=profile['skeleton'];p=np.asarray(s['neutral_points_m'],float)
    if p.shape!=(21,3) or not np.isfinite(p).all():raise ValueError('中立骨架无效')
    for f in FINGERS:
        lengths=np.asarray(s['finger_lengths_m'][f],float)
        if lengths.shape!=(3,) or not np.isfinite(lengths).all() or np.any(lengths<.005) or np.any(lengths>.09):raise ValueError('个人骨长无效')
        offset=profile['finger_offsets'][f]['abd']
        if not np.isfinite(offset) or abs(offset)>.6:raise ValueError('并拢侧摆零位无效')
    t=np.asarray(s['thumb_lengths_m'],float);zero=np.asarray(s['thumb_zero'],float)
    if t.shape!=(3,) or zero.shape!=(3,) or not np.isfinite([t,zero]).all() or np.any(t<.005) or np.any(t>.09):raise ValueError('拇指参考无效')
    validate_thumb(profile['thumb_direct'],bounds)
    return profile


class DisabledThumbCalibration:
    collecting=None;enabled=False;model=None
    def __init__(self):self.message='直接角度模式无需拇指专项标定'
    def reject(self,*_):pass
    def ingest(self,*_):pass
    def cancel(self):pass
    def active_model(self):return None
    def start(self,*_):raise ValueError('D435 直接角度模式只需要两步基础标定')
    def apply(self):raise ValueError(self.message)
    def snapshot(self,now):
        return {'available':False,'enabled':False,'has_profile':False,'ready':False,'collecting':None,
                'samples':{},'rejections':{},'message':self.message}


class SimpleDepthCalibration(CalibrationWizard):
    def __init__(self,robot,bounds,context,path):
        # Deliberately skip legacy profile validation/automatic migrations.
        self.robot,self.bounds,self.context,self.path=robot,bounds,context,Path(path)
        self.capture_seconds=4.;self.samples={};self.pending=None;self.collecting=None;self.buffer=[]
        self.rejections={};self.profile=None;self.enabled=False
        self.message='仅两步：四指伸直略分开、拇指自然展开；再四指并拢伸直'
        if self.path.exists():
            try:
                self.profile=validate(json.loads(self.path.read_text(encoding='utf-8')),context,bounds)
                self.enabled=True;self.message='已加载两步骨架标定；拇指直接角度，无需对指采样'
            except (ValueError,KeyError,TypeError,OSError) as exc:self.message=str(exc)

    def start(self,pose,now):
        if pose not in POSES:raise ValueError('仅支持伸直略分开和并拢伸直两个姿态')
        if self.collecting:raise ValueError('正在采样，请等待或取消')
        if pose=='together' and 'neutral' not in self.samples:raise ValueError('请先采集伸直略分开姿态')
        self.pending=None;self.buffer=[];self.rejections={};self.collecting={'pose':pose,'start':now}
        self.message='准备 1 秒，然后保持 3 秒；掌心正对摄像头'

    def ingest(self,angles,world,handedness,now,observation=None):
        if not self.collecting:return
        self.advance(now)
        if not self.collecting or now-self.collecting['start']<1:return
        o=observation or {};p=np.asarray(o.get('metric_points_m'),float);basis=np.asarray(o.get('basis'),float)
        reason=None
        if p.shape!=(21,3) or basis.shape!=(3,3) or not np.isfinite(p).all():reason='等待完整骨架'
        elif not o.get('sync_valid') or o.get('palm_depth_points',0)<5:reason='可靠掌面深度不足'
        elif abs(basis[2,2])<.8:reason='请掌心正对摄像头'
        elif not o.get('straight',False):reason='请四指自然伸直，不要握拳或勾指'
        elif self.collecting['pose']=='neutral' and not o.get('thumb_reliable',False):reason='请自然展开拇指，让近端骨段可见'
        if reason:self.reject(reason,now);return
        lengths=np.linalg.norm(p[CHILDREN]-p[PARENTS],axis=1)
        checked=lengths if self.collecting['pose']=='neutral' else lengths[4:]
        if np.any(checked<.005) or np.any(checked>.13):self.reject('骨段太短或被遮挡，请调整姿态',now);return
        if len(self.buffer)<120:self.buffer.append(deepcopy(o))

    def advance(self,now):
        if not self.collecting or now-self.collecting['start']<self.capture_seconds:return
        pose=self.collecting['pose'];self.collecting=None
        if len(self.buffer)<20:
            self.message='有效帧不足 20 帧，请保持当前姿态重采'
            if self.rejections:self.message+='；'+max(self.rejections,key=self.rejections.get)
            return
        b=self.buffer
        # Compare in palm coordinates so camera translation does not masquerade
        # as changing bone lengths. Reject unstable frames instead of averaging.
        bones=np.array([np.linalg.norm(np.asarray(o['metric_points_m'])[CHILDREN]-np.asarray(o['metric_points_m'])[PARENTS],axis=1) for o in b])
        checked=bones if pose=='neutral' else bones[:,4:]
        if np.max(np.ptp(np.percentile(checked,[10,90],axis=0),axis=0))>.009:
            self.message='骨长采样变化超过 9 mm，请保持伸直并减少遮挡后重采';return
        points=np.median([o['metric_points_m'] for o in b],axis=0)
        # Save local geometry, not an averaged global pose under palm motion.
        local=np.median([(np.asarray(o['metric_points_m'])-np.asarray(o['metric_points_m'])[0])@np.asarray(o['basis']) for o in b],axis=0)
        sample={'valid_frames':len(b),'spread':float(np.max(np.std(bones,axis=0))),
                'lengths_m':np.median(bones,axis=0).tolist(),'local_points_m':local.tolist(),
                'points_m':points.tolist(),
                'thumb_zero':np.median([o['thumb_features'] for o in b],axis=0).tolist()}
        if pose=='neutral':self.samples={}
        self.samples[pose]=sample
        self.message=f'已采集 {len(self.samples)}/2 个姿态'
        if set(self.samples)==set(POSES):
            try:self.pending=self.build();self.message='两步采集完成，可应用并保存'
            except (ValueError,KeyError) as exc:self.pending=None;self.message=str(exc)

    def build(self):
        n=self.samples['neutral'];t=self.samples['together'];p=np.asarray(n['local_points_m']);together=np.asarray(t['local_points_m'])
        lengths=np.asarray(n['lengths_m'])
        offsets={}
        for i,(f,ids) in enumerate(FINGERS.items()):
            bone=together[ids[1]]-together[ids[0]]
            angle=float(np.arctan2(bone[0],bone[1]))
            offsets[f]={'abd':angle*self.robot.abduction_signs[f],'mcp':0.,'pip':0.}
        profile={'schema':4,'algorithm':VERSION,'model_sha256':model_hash(),'context':deepcopy(self.context),
                 'created_at':datetime.now(timezone.utc).isoformat(),'finger_policy':'articulated_image_first',
                 'finger_offsets':offsets,'skeleton':{'neutral_points_m':p.tolist(),
                    'finger_lengths_m':{f:lengths[i*4+5:i*4+8].tolist() for i,f in enumerate(FINGERS)},
                    'thumb_lengths_m':lengths[1:4].tolist(),'thumb_zero':n['thumb_zero']},
                 'depth_reference':{'algorithm':ALGORITHM,'lengths_m':lengths.tolist(),'valid_frames':n['valid_frames'],'units':'meters'},
                 'thumb_direct':deepcopy(self.profile['thumb_direct'] if self.profile else DEFAULT_THUMB),
                 'samples':deepcopy(self.samples)}
        return validate(profile,self.context,self.bounds)

    def write_profile(self,profile):
        validate(profile,self.context,self.bounds)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        if self.path.exists():
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
            self.path.with_name(f'{self.path.stem}-backup-{stamp}.json').write_bytes(self.path.read_bytes())
        temporary=self.path.with_suffix('.tmp');temporary.write_text(json.dumps(profile,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(self.path)

    def apply(self):
        if self.collecting:raise ValueError('请等待采样结束')
        if self.pending is None:raise ValueError('请先完成两个姿态')
        self.write_profile(self.pending);self.profile=deepcopy(self.pending);self.enabled=True
        self.message='两步骨架标定已保存，恢复识别即可预览'

    def reference(self):
        # Old automatic reference must never bias new straight-hand lengths.
        if self.collecting:return None
        return self.profile['depth_reference'] if self.enabled and self.profile else None

    def snapshot(self,now):
        self.advance(now);elapsed=0 if not self.collecting else now-self.collecting['start']
        return {'enabled':self.enabled,'has_profile':self.profile is not None,'ready':self.pending is not None,
                'schema':4,'poses':list(POSES),'finger_policy':'articulated_image_first',
                'samples':{k:{'valid_frames':v['valid_frames'],'spread':v['spread']} for k,v in self.samples.items()},
                'collecting':None if not self.collecting else self.collecting['pose'],
                'phase':'prepare' if elapsed<1 else 'sample','progress':min(1,elapsed/self.capture_seconds),
                'capture_seconds':self.capture_seconds,'valid_frames':len(self.buffer) if self.collecting else 0,
                'rejections':dict(self.rejections),'message':self.message}
