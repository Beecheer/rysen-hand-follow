"""Compact WiLoR calibration: parallel fingers, slight spread, thumb direction."""
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np

from .calibration import CalibrationWizard, canonical_hand, model_hash, validate_profile
from .geometry import FINGERS, unit

POSES=('together','spread','thumb_direction')


def side_gains(together,spread,bounds):
    """Scale both signed spread directions without reversing or crushing either."""
    gains={}
    for finger in FINGERS:
        delta=float(spread['angles'][finger]['abd']-together['angles'][finger]['abd'])
        if abs(delta)<.035:
            gains[finger]=1.;continue
        lo,hi=bounds[f'left_{finger}_j0']
        target=.35*(hi if delta>0 else lo)
        gains[finger]=float(np.clip(target/delta,.2,8.))
    return gains


class WiLoRCalibrationWizard(CalibrationWizard):
    """Keep WiLoR's 3D hand model, calibrating only personal offsets and axes."""
    def __init__(self,robot,bounds,context,path):
        self.robot,self.bounds,self.context,self.path=robot,bounds,context,Path(path)
        self.capture_seconds=8
        self.samples={};self.pending=None;self.collecting=None;self.buffer=[];self.rejections={}
        self.profile=None;self.enabled=False
        self.message='按顺序采集四指并拢、四指微张和拇指方向三个姿态'
        if self.path.exists():
            try:
                self.profile=validate_profile(json.loads(self.path.read_text(encoding='utf-8')),context)
                if self.profile['schema']!=5:raise ValueError('已有 WiLoR 标定为旧版；完成三姿态采集以更新，旧文件会自动备份')
                saved=self.profile.get('samples',{})
                if all(pose in saved for pose in ('together','spread')):
                    self.profile['finger_gains']=side_gains(saved['together'],saved['spread'],bounds)
                self.enabled=True;self.message='已加载 WiLoR 三姿态标定'
            except (ValueError,KeyError,TypeError,OSError) as exc:self.message=str(exc)

    def start(self,pose,now):
        if pose not in (*POSES,'side_zero'):raise ValueError('未知 WiLoR 标定姿态')
        if pose=='side_zero' and (not self.profile or not self.enabled):raise ValueError('请先启用已有 WiLoR 标定')
        if pose not in ('together','side_zero') and 'together' not in self.samples:raise ValueError('请先采集四指并拢伸直')
        if self.collecting:raise ValueError('正在采样，请等待结束或取消')
        self.pending=None;self.buffer=[];self.rejections={};self.collecting={'pose':pose,'start':now}
        self.message='准备 1 秒，然后保持姿态 7 秒；掌心朝前、整只手入镜'

    def reject(self,reason,now):
        if self.collecting and 1<=now-self.collecting['start']<self.capture_seconds:
            self.rejections[reason]=self.rejections.get(reason,0)+1

    def ingest(self,angles,world,handedness,now,observation=None):
        if not self.collecting:return
        self.advance(now)
        if not self.collecting or now-self.collecting['start']<1:return
        pose=self.collecting['pose']
        try:
            canonical=canonical_hand(world,handedness)
            if not np.isfinite(canonical).all():raise ValueError('三维关键点无效')
            if observation is None:raise ValueError('等待 WiLoR 三维结构观测')
            required=('abd','mcp','pip')
            for finger in FINGERS:
                if any(angles.get(finger,{}).get(k) is None for k in required):
                    raise ValueError('四指关键点不完整或关节角不可靠')
            if pose in ('together','spread','side_zero'):
                if any(abs(angles[f]['mcp'])>math.radians(38) or angles[f]['pip']>math.radians(45) for f in FINGERS):
                    raise ValueError('请四指伸直后再采')
            if pose=='side_zero':
                depth=observation.get('side_sway_depth',{})
                if any(f not in depth or not math.isfinite(depth[f]) for f in FINGERS):
                    raise ValueError('等待四指可靠深度；保持掌心朝前、并拢伸直')
            else:
                thumb_bone=canonical[2]-canonical[1]
                if np.linalg.norm(thumb_bone)<.03:raise ValueError('拇指近端关键点不可靠')
            if len(self.buffer)<160:
                self.buffer.append({'angles':deepcopy(angles),'canonical':canonical,
                                    'observation':deepcopy(observation)})
        except (ValueError,KeyError,TypeError) as exc:self.reject(str(exc),now)

    def advance(self,now):
        if not self.collecting or now-self.collecting['start']<self.capture_seconds:return
        pose=self.collecting['pose'];self.collecting=None
        if len(self.buffer)<20:
            self.message='有效 WiLoR 三维帧不足 20 帧，请保持姿态后重采'
            if self.rejections:self.message+='；主要原因：'+max(self.rejections,key=self.rejections.get)
            return
        points=np.asarray([s['canonical'] for s in self.buffer])
        center=np.median(points,axis=0)
        sample_angles={f:{} for f in FINGERS}
        for finger in FINGERS:
            for key in ('abd','mcp','pip','dip'):
                values=np.asarray([s['angles'][finger].get(key) for s in self.buffer],dtype=object)
                values=np.asarray([v for v in values if v is not None and math.isfinite(v)],float)
                sample_angles[finger][key]=float(np.median(values)) if len(values) else None
                if key in ('abd','mcp','pip') and len(values)<20:
                    self.message=f'{finger}有效角度不足，请完整入镜后重采';return
                if key in ('abd','mcp','pip') and np.ptp(np.percentile(values,[10,90]))>math.radians(24):
                    self.message='姿态变化过大，请保持手掌和手指稳定后重采';return
        if pose=='side_zero':
            pending=deepcopy(self.profile)
            pending['side_sway_zero']={f:{'wilor':sample_angles[f]['abd'],
                'depth':float(np.median([s['observation']['side_sway_depth'][f] for s in self.buffer]))}
                for f in FINGERS}
            for finger in FINGERS:pending['finger_offsets'][finger]['abd']=sample_angles[finger]['abd']
            pending['side_sway_zero_frames']=len(self.buffer)
            paired_model=[s['observation'].get('side_sway_model',{}) for s in self.buffer]
            if all(all(f in s for f in FINGERS) for s in paired_model):
                pending['side_sway_model_zero']={f:float(np.median([s[f] for s in paired_model])) for f in FINGERS}
            pending['created_at']=datetime.now(timezone.utc).isoformat()
            self.pending=validate_profile(pending,self.context)
            self.buffer=[]
            self.message='四指两路侧摆零位已采集；点击应用并保存，拇指及屈伸配置保留'
            return
        observations=[s['observation'] for s in self.buffer]
        med={key:np.median([o[key] for o in observations],axis=0).tolist()
             for key in ('canonical','image_aspect','camera_normal')}
        normal=np.asarray(med['camera_normal'],float);med['camera_normal']=(normal/np.linalg.norm(normal)).tolist()
        med['bone_lengths']={f:np.median([o['bone_lengths'][f] for o in observations],axis=0).tolist()
                             for f in (*FINGERS,'thumb')}
        noise={f:np.median(np.abs(np.asarray([o['bone_lengths'][f] for o in observations])-
                                      np.asarray(med['bone_lengths'][f])),axis=0).tolist()
               for f in (*FINGERS,'thumb')}
        med['noise']=noise
        sample={'angles':sample_angles,'canonical':center.tolist(),'structure_observation':med,
                'valid_frames':len(self.buffer),
                'spread':float(np.median([np.std(np.asarray(s['canonical']),axis=0) for s in self.buffer]))}
        paired=[s['observation']['side_sway_depth'] for s in self.buffer
                if all(f in s['observation'].get('side_sway_depth',{}) for f in FINGERS)]
        if len(paired)>=20:sample['side_sway_depth']={f:float(np.median([s[f] for s in paired])) for f in FINGERS}
        model=[s['observation']['side_sway_model'] for s in self.buffer
               if all(f in s['observation'].get('side_sway_model',{}) for f in FINGERS)]
        if len(model)>=20:sample['side_sway_model']={f:float(np.median([s[f] for s in model])) for f in FINGERS}
        if pose=='together':
            self.samples={}
            self.samples[pose]=sample
        else:self.samples[pose]=sample
        self.buffer=[]
        self.message=f'已采集 {len(self.samples)}/3 个姿态'
        if pose=='spread':
            base=self.samples['together']['angles'];delta=[abs(sample['angles'][f]['abd']-base[f]['abd']) for f in FINGERS]
            if sum(v>.04 for v in delta)<2:
                self.samples.pop('spread',None);self.message='微张幅度不明显；四指自然分开约 5–15° 后重采'
                return
        if pose=='thumb_direction':
            base_axis=unit(np.asarray(self.samples['together']['canonical'])[2]-np.asarray(self.samples['together']['canonical'])[1])
            pose_axis=unit(np.asarray(sample['canonical'])[2]-np.asarray(sample['canonical'])[1])
            if math.acos(float(np.clip(base_axis@pose_axis,-1,1)))<math.radians(8):
                self.samples.pop('thumb_direction',None);self.message='拇指方向变化不足；保持四指并拢，将拇指沿掌面向外舒适张开后重采'
                return
        if set(self.samples)==set(POSES):
            try:self.pending=self.build_profile();self.message='三姿态 WiLoR 标定完成，可应用并保存'
            except (ValueError,KeyError,TypeError) as exc:self.pending=None;self.message=str(exc)

    def build_profile(self):
        together=self.samples['together'];spread=self.samples['spread'];thumb=self.samples['thumb_direction']
        offsets={f:{'abd':float(together['angles'][f]['abd']),'mcp':0.,'pip':0.} for f in FINGERS}
        gains=side_gains(together,spread,self.bounds)
        obs=together['structure_observation']
        structure={'bone_lengths':obs['bone_lengths'],'palm_template':np.asarray(obs['canonical'])[ [0,5,9,13,17] ].tolist(),
                   'image_aspect':obs['image_aspect'],'camera_normal':obs['camera_normal'],'noise':obs['noise'],
                   'spread_reference':spread['angles'],'flexion_checks':{}}
        q=np.array([.45,.2,.3,.12],float)
        for i in range(4):q[i]=np.clip(q[i],*self.bounds[f'left_thumb_j{i}'])
        robot_points=self.robot.thumb_points(q)
        robot_axis=unit((robot_points[1]-robot_points[0])@self.robot.basis)
        human_axis=unit(np.asarray(thumb['canonical'])[2]-np.asarray(thumb['canonical'])[1])
        profile={'schema':5,'finger_policy':'wilor_three_pose','created_at':datetime.now(timezone.utc).isoformat(),
            'model_sha256':model_hash(),'context':deepcopy(self.context),'finger_offsets':offsets,'finger_gains':gains,
            'structure':structure,'alignment':{'scale':.05,'rotation':np.eye(3).tolist(),'translation':[0.,0.,0.],'rms_mm':0.},
            'neutral_thumb':q.tolist(),'contacts':{f:{'close':.12} for f in FINGERS},
            'thumb_direction':{'human_axis':human_axis.tolist(),'robot_axis':robot_axis.tolist(),'reference_joints':q.tolist()},
            'samples':deepcopy(self.samples)}
        depth=together.get('side_sway_depth',{})
        if all(f in depth for f in FINGERS):
            profile['side_sway_zero']={f:{'wilor':offsets[f]['abd'],'depth':depth[f]} for f in FINGERS}
        if 'side_sway_model' in together:profile['side_sway_model_zero']=deepcopy(together['side_sway_model'])
        return validate_profile(profile,self.context)

    def apply(self):
        if self.collecting:raise ValueError('请等待采样结束')
        if self.pending is None:raise ValueError('请先完成三个有效姿态')
        profile=validate_profile(self.pending,self.context)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        if self.path.exists():
            stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')
            self.path.with_name(f'{self.path.stem}-backup-{stamp}.json').write_bytes(self.path.read_bytes())
        temporary=self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(profile,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(self.path);self.profile=deepcopy(profile);self.enabled=True
        self.message='WiLoR 三姿态标定已应用并保存；旧配置已备份'

    def snapshot(self,now):
        self.advance(now);elapsed=0 if not self.collecting else now-self.collecting['start']
        return {'enabled':self.enabled,'has_profile':self.profile is not None,'ready':self.pending is not None,
                'finger_policy':'wilor_three_pose','schema':5 if self.profile is not None else (5 if self.pending else None),
                'side_zero_ready':bool(self.profile and self.profile.get('side_sway_zero') and self.enabled),
                'side_zero_pending':bool(self.pending and self.pending.get('side_sway_zero')),
                'poses':list(POSES),'samples':{k:{'valid_frames':v['valid_frames'],'spread':v['spread']} for k,v in self.samples.items()},
                'collecting':None if not self.collecting else self.collecting['pose'],
                'phase':'prepare' if elapsed<1 else 'sample','progress':min(1,elapsed/self.capture_seconds),
                'capture_seconds':self.capture_seconds,'rejections':dict(self.rejections),
                'valid_frames':len(self.buffer) if self.collecting else 0,'message':self.message}
