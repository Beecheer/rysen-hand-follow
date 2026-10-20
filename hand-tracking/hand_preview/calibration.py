"""Personal front-view calibration, transactional capture and contact cues.

Only landmark/angle statistics explicitly sampled by the user are persisted.
There is no image recording, gesture classifier or robot transport.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .geometry import FINGERS, palm_frame
from .mapping import ASSET_DIR
from .human_pose import build_structure

POSES = ('neutral', 'index', 'middle', 'ring', 'pinky')
FRONT_POSES=('neutral','spread','hook','mcp','index','middle','ring','pinky')
CAPTURE_SECONDS=4
TIP_IDS = {'index':8, 'middle':12, 'ring':16, 'pinky':20}
PROFILE_PATH = ASSET_DIR.parents[1] / 'profiles' / 'current.json'


def model_hash():
    return hashlib.sha256((ASSET_DIR/'apex_hand_left.urdf').read_bytes()).hexdigest()


def canonical_hand(world, handedness):
    points,basis,length = palm_frame(world,handedness)
    return (points-points[0])@basis/length


def similarity_fit(source,target):
    """Proper similarity mapping: row-vector target = scale * source @ R + t."""
    source,target=np.asarray(source,float),np.asarray(target,float)
    if source.shape!=target.shape or source.shape!=(5,3) or not np.isfinite([source,target]).all():
        raise ValueError('标定参考点不完整或无效')
    x,y=source-source.mean(axis=0),target-target.mean(axis=0)
    if np.linalg.svd(x,compute_uv=False)[1] < 0.015:
        raise ValueError('五个姿态区分不足，请重新采集中立位或对指姿态')
    u,s,vt=np.linalg.svd(x.T@y)
    correction=np.eye(3);correction[-1,-1]=1 if np.linalg.det(u@vt)>0 else -1
    r=u@correction@vt
    scale=float(np.sum(s*np.diag(correction))/np.sum(x*x))
    if not .005<scale<.3:
        raise ValueError('标定尺寸比例异常，请检查是否完整入镜')
    translation=target.mean(axis=0)-scale*source.mean(axis=0)@r
    rms=float(np.sqrt(np.mean(np.sum((scale*source@r+translation-target)**2,axis=1))))
    return {'scale':scale,'rotation':r.tolist(),'translation':translation.tolist(),'rms_mm':rms*1000}


def correct_fingers(angles,profile):
    # A single neutral pose cannot separate real flexion from tracking bias.
    # Subtracting it from MCP/PIP removes usable bend and creates a dead zone.
    result=deepcopy(angles)
    if profile:
        for finger in FINGERS:
            value=result.get(finger,{}).get('abd')
            if value is not None:
                value-=profile['finger_offsets'][finger]['abd']
                value*=profile.get('finger_gains',{}).get(finger,1.0)
                result[finger]['abd']=value
    return result


def finger_targets(angles,bounds,current=None):
    result=dict(current or {})
    for finger in FINGERS:
        for key,suffix in (('abd',0),('mcp',1),('pip',2)):
            name=f'left_{finger}_j{suffix}';value=angles.get(finger,{}).get(key)
            if value is not None:
                lo,hi=bounds[name];result[name]=float(np.clip(value,lo,hi))
        # Kinematics also resolves this from URDF; keep explicit output useful.
        result[f'left_{finger}_j3']=result.get(f'left_{finger}_j2',0.0)
    return result


def build_profile(samples,robot,bounds,context):
    enhanced=set(samples)==set(FRONT_POSES)
    if not enhanced and set(samples)!=set(POSES):raise ValueError('请完成八个姿态的有效采样')
    offsets={finger:{key:samples['neutral']['angles'][finger][key] for key in ('abd','mcp','pip')}
             for finger in FINGERS}
    if any(value is None or not math.isfinite(value) for part in offsets.values() for value in part.values()):
        raise ValueError('中立位关节不完整，请重采')
    for part in offsets.values():
        if abs(part['mcp'])>math.radians(35) or abs(part['pip'])>math.radians(40):
            raise ValueError('中立位手指弯曲过大，请自然伸直后重采')
    provisional={'finger_offsets':offsets}
    neutral_q=np.array([.45,.2,.3,.12])
    neutral_points=robot.thumb_points(neutral_q)
    base=neutral_points[0]
    source=[samples['neutral']['canonical'][4]]
    targets=[(neutral_points[-1]-base)@robot.basis]
    for finger in FINGERS:
        adjusted=correct_fingers(samples[finger]['angles'],provisional)
        positions=finger_targets(adjusted,bounds)
        links,_=robot.forward(positions)
        source.append(samples[finger]['canonical'][4])
        targets.append((links[f'left_{finger}_tip'][:3,3]-base)@robot.basis)
    alignment=similarity_fit(source,targets)
    if alignment['rms_mm']>50:raise ValueError('标定参考点误差过大，请检查四个对指动作是否采错')
    profile={'schema':3 if enhanced else 2,'finger_policy':'abduction_only','created_at':datetime.now(timezone.utc).isoformat(),
            'model_sha256':model_hash(),'context':context,'finger_offsets':offsets,
            'alignment':alignment,'neutral_thumb':neutral_q.tolist(),
            'contacts':{finger:{'close':max(.025,float(samples[finger]['distance'])+.025)} for finger in FINGERS},
            'samples':deepcopy(samples)}
    if enhanced:profile['structure']=build_structure(samples)
    return profile


def calibration_cues(profile,world,handedness,robot,finger_positions,valid_fingers):
    points=canonical_hand(world,handedness)
    if not np.isfinite(points[4]).all():raise ValueError('拇指尖无效')
    a=profile['alignment']
    calibrated_tip=a['scale']*points[4]@np.array(a['rotation'])+np.array(a['translation'])
    links,_=robot.forward(finger_positions)
    base=robot.thumb_points(np.zeros(4))[0]
    weights={}
    for finger in FINGERS:
        if finger not in valid_fingers or not np.isfinite(points[TIP_IDS[finger]]).all():
            weights[finger]=0.0;continue
        distance=float(np.linalg.norm(points[4]-points[TIP_IDS[finger]]))
        close=profile['contacts'][finger]['close']
        far=close+max(.12,close*.8)
        fraction=float(np.clip((far-distance)/(far-close),0,1))
        weights[finger]=fraction*fraction*(3-2*fraction)
    total=sum(weights.values())
    strength=max(weights.values())
    goal=(1-strength)*calibrated_tip
    if total>0:
        for finger,weight in weights.items():
            target=(links[f'left_{finger}_tip'][:3,3]-base)@robot.basis
            goal+=strength*weight/total*target
    active=max(weights,key=weights.get) if strength>.05 else None
    return {'tip':goal,'strength':strength,'active':active,'weights':weights}


def validate_profile(profile,context=None):
    if profile.get('schema') not in (1,2,3,5) or profile.get('model_sha256')!=model_hash():
        raise ValueError('标定版本或模型不匹配，请重新标定')
    if profile['schema'] in (2,3) and profile.get('finger_policy')!='abduction_only':
        raise ValueError('不支持的四指标定方式')
    if profile['schema']==5 and profile.get('finger_policy')!='wilor_three_pose':
        raise ValueError('不支持的 WiLoR 三姿态标定')
    if context is not None and profile.get('context')!=context:
        raise ValueError('摄像头或左右手校正设置已改变，未自动应用旧标定')
    values=[profile['finger_offsets'][f][k] for f in FINGERS for k in ('abd','mcp','pip')]
    a=profile['alignment'];r=np.asarray(a['rotation'],float);t=np.asarray(a['translation'],float)
    if not np.isfinite(values).all() or max(abs(x) for x in values)>math.pi:
        raise ValueError('标定偏置无效')
    if r.shape!=(3,3) or t.shape!=(3,) or not np.isfinite([*r.ravel(),*t]).all():
        raise ValueError('标定变换无效')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-4) or np.linalg.det(r)<.99 or not .005<a['scale']<.3:
        raise ValueError('标定变换范围无效')
    if any(not .025<=profile['contacts'][f]['close']<=.6 for f in FINGERS):
        raise ValueError('对指阈值无效')
    if profile['schema'] in (3,5):
        s=profile['structure'];palm=np.asarray(s['palm_template'],float);normal=np.asarray(s['camera_normal'],float)
        if palm.shape!=(5,3) or normal.shape!=(3,) or not np.isfinite(palm).all() or not np.isfinite(normal).all():raise ValueError('掌面参考无效')
        if np.linalg.svd(palm-palm.mean(axis=0),compute_uv=False)[1]<.015:raise ValueError('掌面参考退化，请重采中立位')
        if not .95<np.linalg.norm(normal)<1.05 or not .1<s['image_aspect']<5:raise ValueError('掌面朝向参考无效')
        for finger in (*FINGERS,'thumb'):
            lengths=np.asarray(s['bone_lengths'][finger],float)
            if lengths.shape!=(3,) or not np.isfinite(lengths).all() or np.any(lengths<.035) or np.any(lengths>1.4):raise ValueError('个人骨长无效')
            noise=np.asarray(s.get('noise',{}).get(finger,[0,0,0]),float)
            if noise.shape!=(3,) or not np.isfinite(noise).all() or np.any(noise<0) or np.any(noise>1):raise ValueError('骨长噪声统计无效')
    if profile['schema']==5:
        gains=profile['finger_gains'];thumb=profile['thumb_direction']
        if 'side_sway_zero' in profile:
            zero=profile['side_sway_zero']
            if set(zero)!=set(FINGERS) or any(not math.isfinite(float(zero[f][key])) or abs(float(zero[f][key]))>math.pi
                    for f in FINGERS for key in ('wilor','depth')):
                raise ValueError('双源侧摆零位无效')
        if 'side_sway_model_zero' in profile:
            zero=profile['side_sway_model_zero']
            if set(zero)!=set(FINGERS) or any(not math.isfinite(float(v)) or abs(float(v))>math.pi for v in zero.values()):
                raise ValueError('三维侧摆参考无效')
        if set(gains)!=set(FINGERS) or any(not .2<=float(v)<=8 for v in gains.values()):raise ValueError('四指侧摆比例无效')
        for key in ('human_axis','robot_axis'):
            axis=np.asarray(thumb[key],float)
            if axis.shape!=(3,) or not np.isfinite(axis).all() or not .95<np.linalg.norm(axis)<1.05:
                raise ValueError('拇指方向参考无效')
    return profile


class CalibrationWizard:
    def __init__(self,robot,bounds,context,path=PROFILE_PATH,capture_seconds=CAPTURE_SECONDS):
        self.robot,self.bounds,self.context,self.path=robot,bounds,context,Path(path)
        self.capture_seconds=capture_seconds
        self.samples={};self.pending=None;self.collecting=None;self.buffer=[]
        self.rejections={}
        self.message='先采集中立位，再采集张开、钩手、指根弯曲和四个对指姿态'
        self.profile=None;self.enabled=False
        if self.path.exists():
            try:
                self.profile=validate_profile(json.loads(self.path.read_text()),context)
                if self.profile['schema']==1:
                    # Thumb contact references must use runtime finger mapping.
                    # Rebuild from saved samples, retaining the original file.
                    original_created_at=self.profile.get('created_at')
                    self.profile=build_profile(self.profile['samples'],robot,bounds,context)
                    self.profile['source_created_at']=original_created_at
                    validate_profile(self.profile,context)
                    self.message='已兼容旧标定：保留侧摆校正，恢复四指屈伸幅度；原文件保留'
                else:self.message='已加载八姿态个人结构标定' if self.profile['schema']==3 else '旧五姿态标定已保留；完成八姿态采集后启用个人结构优化'
                self.enabled=True
            except (ValueError,KeyError,TypeError,OSError) as exc:self.message=str(exc)

    def start(self,pose,now):
        if pose not in FRONT_POSES:raise ValueError('未知标定姿态')
        if pose!='neutral' and 'neutral' not in self.samples:raise ValueError('请先采集中立位')
        if self.collecting:raise ValueError('正在采样，请等待结束或取消')
        self.pending=None
        self.collecting={'pose':pose,'start':now};self.buffer=[]
        self.rejections={}
        self.message=f'请保持掌心朝前：准备 1 秒，随后采样 {self.capture_seconds-1} 秒'

    def reject(self,reason,now):
        if self.collecting and 1<=now-self.collecting['start']<self.capture_seconds:
            self.rejections[reason]=self.rejections.get(reason,0)+1

    def ingest(self,angles,world,handedness,now,observation=None):
        if not self.collecting:return
        self.advance(now)
        if not self.collecting:return
        elapsed=now-self.collecting['start']
        if not 1<=elapsed<=self.capture_seconds:return
        pose=self.collecting['pose']
        p=canonical_hand(world,handedness)
        structural=pose in ('neutral','spread','hook','mcp')
        required=list(FINGERS) if structural else [pose]
        if not np.isfinite(p).all():
            self.reject('关键点数值无效',now);return
        for finger in required:
            keys=('abd','mcp','pip') if pose in ('neutral','spread') else ('mcp','pip')
            if any(angles.get(finger,{}).get(key) is None for key in keys):
                names={'index':'食指','middle':'中指','ring':'无名指','pinky':'小指'}
                self.reject(names[finger]+'结构或侧摆观测不可靠',now);return
        if observation and pose!='neutral':
            reference=self.samples['neutral'].get('structure_observation')
            if reference:
                facing=np.dot(reference['camera_normal'],observation['camera_normal'])
                ratio=observation['image_aspect']/reference['image_aspect']
                if facing<.65 or not .65<ratio<1.55:
                    self.reject('掌面偏离中立参考',now)
                    self.message='掌面偏离中立参考，请正对摄像头后重采';return
        distance=0.0 if structural else float(np.linalg.norm(p[4]-p[TIP_IDS[pose]]))
        if distance>.55:
            self.reject('拇指未接近目标指尖',now);return
        if len(self.buffer)<100:
            self.buffer.append({'angles':deepcopy(angles),'canonical':p.tolist(),'distance':distance,'observation':deepcopy(observation)})

    def advance(self,now):
        if not self.collecting or now-self.collecting['start']<self.capture_seconds:return
        pose=self.collecting['pose'];self.collecting=None
        if len(self.buffer)<20:
            self.message=('有效帧不足 20 帧，请完整入镜、保持姿态后重采' if pose in ('neutral','spread','hook','mcp')
                          else '有效帧不足 20 帧，请完整入镜、保持轻触后重采')
            if self.rejections:self.message+='；主要原因：'+max(self.rejections,key=self.rejections.get)
            return
        points=np.array([s['canonical'] for s in self.buffer])
        center=np.median(points,axis=0)
        spread=float(np.percentile(np.linalg.norm(points[:,4]-center[4],axis=1),75))
        if spread>.12:
            self.message='拇指姿态不够稳定，请保持姿态后重采';return
        angles={}
        for finger in FINGERS:
            angles[finger]={}
            for key in ('abd','mcp','pip','dip'):
                values=[s['angles'][finger].get(key) for s in self.buffer]
                values=[v for v in values if v is not None and math.isfinite(v)]
                angles[finger][key]=float(np.median(values)) if values else None
                if finger in (list(FINGERS) if pose in ('neutral','spread','hook','mcp') else [pose]) and key in ('mcp','pip'):
                    if len(values)<20 or np.ptp(np.percentile(values,[10,90]))>math.radians(20):
                        self.message='关节姿态变化过大，请保持不动后重采';return
        sample={'angles':angles,'canonical':center.tolist(),
                'distance':float(np.median([s['distance'] for s in self.buffer])),
                'valid_frames':len(self.buffer),'spread':spread}
        observations=[s['observation'] for s in self.buffer if s.get('observation')]
        if observations:
            observation={key:np.median([s[key] for s in observations],axis=0).tolist() for key in ('canonical','image_aspect','camera_normal')}
            normal=np.asarray(observation['camera_normal']);observation['camera_normal']=(normal/np.linalg.norm(normal)).tolist()
            observation['bone_lengths']={f:np.median([s['bone_lengths'][f] for s in observations],axis=0).tolist() for f in (*FINGERS,'thumb')}
            sample['structure_observation']=observation
            sample['structure_noise']={f:np.median(np.abs(np.array([s['bone_lengths'][f] for s in observations])-observation['bone_lengths'][f]),axis=0).tolist() for f in (*FINGERS,'thumb')}
        if pose=='neutral':
            if any(abs(angles[f]['mcp'])>math.radians(35) or angles[f]['pip']>math.radians(40) for f in FINGERS):
                self.message='中立位请自然伸直四指，不要握拢';return
            # Replacing neutral invalidates all contact samples referenced to it.
            self.samples={}
        self.samples[pose]=sample
        self.message=f'已采集 {len(self.samples)}/8 个姿态，可重采或继续'
        if len(self.samples)==8:
            try:
                self.pending=build_profile(self.samples,self.robot,self.bounds,self.context)
                self.message='采集完成，可应用并保存；参考对齐 RMS %.1f mm' % self.pending['alignment']['rms_mm']
            except ValueError as exc:self.message=str(exc)

    def cancel(self):
        self.collecting=None;self.buffer=[];self.message='已取消当前采样，已有样本保留'

    def apply(self):
        if not self.pending:raise ValueError('请先完成八个姿态的有效采样')
        profile=validate_profile(self.pending,self.context)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        if self.path.exists():
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
            self.path.with_name(f'backup-{stamp}.json').write_bytes(self.path.read_bytes())
        temporary=self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(profile,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(self.path)
        self.profile=deepcopy(profile);self.enabled=True;self.message='标定已应用并保存，下次启动自动加载'

    def reset(self):
        if self.path.exists():
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
            self.path.replace(self.path.with_name(f'reset-backup-{stamp}.json'))
        self.profile=self.pending=None;self.enabled=False;self.samples={};self.cancel()
        self.message='已恢复未标定模式；旧配置如存在已保留备份'

    def snapshot(self,now):
        self.advance(now)
        elapsed=0 if not self.collecting else now-self.collecting['start']
        return {'enabled':self.enabled,'has_profile':self.profile is not None,'ready':self.pending is not None,
                'finger_policy':'abduction_only',
                'schema':None if self.profile is None else self.profile['schema'],'poses':list(FRONT_POSES),
                'samples':{k:{'valid_frames':v['valid_frames'],'spread':v['spread']} for k,v in self.samples.items()},
                'collecting':None if not self.collecting else self.collecting['pose'],
                'phase':'prepare' if elapsed<1 else 'sample','progress':min(1,elapsed/self.capture_seconds),
                'capture_seconds':self.capture_seconds,
                'rejections':dict(self.rejections),
                'valid_frames':len(self.buffer) if self.collecting else 0,'message':self.message}
