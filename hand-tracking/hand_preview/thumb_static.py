"""Four static D435 tip correspondences; optional, isolated from base profile."""
from copy import deepcopy
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np

STEPS=('rest','index_base','pinky_side','lift')
DEFAULT_REFERENCES={'rest':[26.,10.,18.],'index_base':[55.,12.,48.],
                    'pinky_side':[72.,26.,65.],'lift':[60.,10.,18.]}


def signature(base,bounds):
    data={k:base.get(k) for k in ('created_at','context','model_sha256','skeleton','depth_reference','thumb_direct')}
    data['bounds']={f'j{i}':bounds[f'left_thumb_j{i}'] for i in range(4)}
    return hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()


def validate(model,base,bounds):
    if not base or model.get('schema')!=1 or model.get('base_signature')!=signature(base,bounds):
        raise ValueError('基础骨架、末端角度或限位已改变，需重采拇指静态标定')
    human=np.asarray(model['human_m'],float);target=np.asarray(model['robot_m'],float)
    matrix=np.asarray(model['matrix'],float);offset=np.asarray(model['offset_m'],float)
    if human.shape!=(4,3) or target.shape!=(4,3) or matrix.shape!=(3,3) or offset.shape!=(3,):raise ValueError('静态参考维度无效')
    if not all(np.isfinite(v).all() for v in (human,target,matrix,offset)):raise ValueError('静态参考数值无效')
    singular=np.linalg.svd(human[1:]-human[0],compute_uv=False)
    if singular[-1]<.0025 or singular[0]/singular[-1]>40:raise ValueError('四个位置空间区分不足，请重采收拢或抬起姿态')
    if not np.allclose(human@matrix+offset,target,atol=1e-6):raise ValueError('静态位置变换不一致')
    gain=np.linalg.svd(matrix,compute_uv=False)
    if gain[0]>10 or gain[-1]<.015:raise ValueError('静态映射放大或压缩过大，请检查参考姿态和采样位置')
    for step in STEPS:
        sample=model['samples'][step];q=np.asarray(sample['reference_deg'],float)
        if q.shape!=(3,) or not np.isfinite(q).all() or sample['valid_frames']<20:raise ValueError('静态样本不完整')
        for i,value in enumerate(np.radians(q)):
            lo,hi=bounds[f'left_thumb_j{i}']
            if not lo<=value<=hi:raise ValueError('参考姿态超出当前限位')
    return model


def calibrated_target(model,human,nominal):
    p=np.asarray(human,float);anchors=np.asarray(model['human_m'])
    coordinates=np.linalg.solve((anchors[1:]-anchors[0]).T,p-anchors[0])
    # Smoothly return to the ordinary mapping far beyond sampled support.
    amount=float(np.clip(3.-np.max(abs(coordinates)),0,1))
    mapped=p@np.asarray(model['matrix'])+np.asarray(model['offset_m'])
    return amount*mapped+(1-amount)*nominal,amount


class StaticThumbCalibration:
    def __init__(self,base,robot,bounds):
        self.base=base;self.robot=robot;self.bounds=bounds
        self.path=base.path.with_name(base.path.stem+'-thumb-static.json')
        self.samples={};self.pending=None;self.model=None;self.enabled=False
        self.collecting=None;self.buffer=[];self.rejections={};self.capture_seconds=4.
        self.selected='rest';self.reference_preview=False;self.revision=0
        self.references=deepcopy(DEFAULT_REFERENCES)
        self.message='保留两步骨架标定；额外采四个静态拇指位置，每步保持 3 秒'
        if self.path.exists():
            try:
                self.model=validate(json.loads(self.path.read_text(encoding='utf-8')),base.profile,bounds)
                self.references={s:deepcopy(self.model['samples'][s]['reference_deg']) for s in STEPS}
                self.enabled=True;self.message='已加载拇指四位置静态标定'
            except (ValueError,KeyError,TypeError,OSError) as exc:self.message=str(exc)

    def compatible(self):
        return bool(self.model and self.base.profile and self.model.get('base_signature')==signature(self.base.profile,self.bounds))

    def active_model(self):
        return self.model if self.enabled and self.base.enabled and self.compatible() else None

    def check_base(self):
        if not self.base.enabled or not self.base.profile:raise ValueError('请先启用已保存的两步骨架标定')
        if self.base.collecting:raise ValueError('请先完成或取消基础采样')

    def preview(self,step,reference=None):
        self.check_base()
        if self.collecting:raise ValueError('正在采样，请先结束或取消')
        if step not in STEPS:raise ValueError('未知静态标定姿态')
        if reference is not None:
            q=np.asarray(reference,float)
            if q.shape!=(3,) or not np.isfinite(q).all():raise ValueError('请输入三个有限参考角度')
            for i,value in enumerate(np.radians(q)):
                lo,hi=self.bounds[f'left_thumb_j{i}']
                if not lo<=value<=hi:raise ValueError('参考角度超出当前软件限位')
            if not np.allclose(q,self.references[step]):
                self.samples.pop(step,None);self.pending=None
                self.references[step]=q.tolist()
        self.selected=step;self.reference_preview=True;self.revision+=1

    def start(self,step,now):
        self.check_base()
        if self.collecting:raise ValueError('正在采样，请等待或取消')
        if step not in STEPS:raise ValueError('未知静态标定姿态')
        if step!='rest' and 'rest' not in self.samples:raise ValueError('请先采自然展开')
        key=signature(self.base.profile,self.bounds)
        if self.samples and self.samples.get('rest',{}).get('base_signature')!=key and step!='rest':
            raise ValueError('基础配置已改变，请从自然展开重新采集')
        self.selected=step;self.reference_preview=True;self.revision+=1
        self.collecting={'step':step,'start':now,'signature':key};self.buffer=[];self.rejections={};self.pending=None
        self.message='准备 1 秒，静止采样 3 秒；掌心朝前，指尖与根部保持可见'

    def reject(self,reason,now):
        if self.collecting and now-self.collecting['start']>=1:
            self.rejections[reason]=self.rejections.get(reason,0)+1

    def ingest_depth(self,points,basis,now,quality,hand):
        if not self.collecting:return
        self.advance(now)
        if not self.collecting or now-self.collecting['start']<1:return
        reason=None
        if self.collecting['signature']!=signature(self.base.profile,self.bounds):reason='基础配置已改变'
        elif not quality.get('thumb_tip_reliable'):reason='拇指指尖或根部缺少可靠深度'
        elif not hand.get('sync_valid') or quality.get('palm_depth_points',0)<5:reason='彩深不同步或可见掌面不足'
        elif not quality.get('front_ok'):reason='请掌心尽量正对摄像头'
        p=np.asarray(points,float);b=np.asarray(basis,float)
        if reason:self.reject(reason,now);return
        relative=(p[4]-p[1])@b
        if not np.isfinite(relative).all():self.reject('拇指位置无效',now);return
        if self.collecting['step']!='rest':
            ref=np.asarray(self.samples['rest']['basis'])
            if np.trace(ref.T@b)<2.65:self.reject('手掌转动过大，请保持标定朝向',now);return
        if len(self.buffer)<120:self.buffer.append({'position':relative.tolist(),'basis':b.tolist()})

    def advance(self,now):
        if not self.collecting or now-self.collecting['start']<self.capture_seconds:return
        current=self.collecting;self.collecting=None
        if len(self.buffer)<20:
            self.message='有效帧不足 20 帧，请重采当前姿态'
            if self.rejections:self.message+='；'+max(self.rejections,key=self.rejections.get)
            return
        points=np.array([r['position'] for r in self.buffer]);center=np.median(points,axis=0)
        jitter=float(np.percentile(np.linalg.norm(points-center,axis=1),90))
        if jitter>.006:self.message='拇指位置不稳定，请静止保持后重采';return
        step=current['step']
        if step=='rest':self.samples={}
        if any(np.linalg.norm(center-np.asarray(v['position_m']))<.008 for v in self.samples.values()):
            self.message='当前位置与已有姿态太接近，请扩大舒适活动范围后重采';return
        self.samples[step]={'position_m':center.tolist(),'basis':np.median([r['basis'] for r in self.buffer],axis=0).tolist(),
            'reference_deg':deepcopy(self.references[step]),'valid_frames':len(points),'spread':jitter,'base_signature':current['signature']}
        self.message=f'已采 {len(self.samples)}/4 个位置；P90 位置波动 {jitter*1000:.1f} mm'
        if set(self.samples)==set(STEPS):
            try:self.pending=self.build();self.message='四位置采集完成，可应用并保存；先在纯预览检查'
            except ValueError as exc:self.message=str(exc)

    def build(self):
        self.check_base();key=signature(self.base.profile,self.bounds)
        if any(s['base_signature']!=key for s in self.samples.values()):raise ValueError('采样对应的基础配置不一致，请重新采集')
        human=np.array([self.samples[s]['position_m'] for s in STEPS])
        singular=np.linalg.svd(human[1:]-human[0],compute_uv=False)
        if singular[-1]<.0025 or singular[0]/singular[-1]>40:raise ValueError('四个位置空间区分不足，请重新采集收拢或向镜头抬起')
        fixed=np.radians(self.base.profile['thumb_direct']['fixed_terminal_deg'])
        base=self.robot.thumb_points([0,0,0,0])[0]
        target=np.array([(self.robot.thumb_points([*np.radians(self.samples[s]['reference_deg']),fixed])[-1]-base)@self.robot.basis for s in STEPS])
        matrix=np.linalg.solve(human[1:]-human[0],target[1:]-target[0]);offset=target[0]-human[0]@matrix
        model={'schema':1,'kind':'depth_thumb_static','created_at':datetime.now(timezone.utc).isoformat(),
            'base_signature':key,'human_m':human.tolist(),'robot_m':target.tolist(),'matrix':matrix.tolist(),
            'offset_m':offset.tolist(),'condition':float(singular[0]/singular[-1]),'samples':deepcopy(self.samples)}
        return validate(model,self.base.profile,self.bounds)

    def apply(self):
        self.check_base()
        if self.collecting or self.pending is None:raise ValueError('请先完成四位置有效采样')
        model=validate(self.pending,self.base.profile,self.bounds)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        if self.path.exists():self.path.with_name(self.path.stem+'-backup-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.json').write_bytes(self.path.read_bytes())
        temporary=self.path.with_suffix('.tmp');temporary.write_text(json.dumps(model,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8');temporary.replace(self.path)
        self.model=deepcopy(model);self.enabled=True;self.reference_preview=False;self.revision+=1
        self.message='拇指静态标定已保存；恢复识别后预览，可停用对比'

    def cancel(self):
        self.collecting=None;self.buffer=[];self.reference_preview=False;self.revision+=1
        self.message='已取消采样/参考预览，已有样本保留；恢复识别可继续'

    def reset(self):
        if self.path.exists():self.path.replace(self.path.with_name(self.path.stem+'-reset-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.json'))
        self.model=self.pending=None;self.enabled=False;self.samples={};self.cancel();self.message='已重置拇指静态标定，基础骨架不变，旧文件保留备份'

    def snapshot(self,now):
        self.advance(now);elapsed=0 if not self.collecting else now-self.collecting['start'];q=self.references[self.selected]
        fixed=(self.base.profile or {}).get('thumb_direct',{}).get('fixed_terminal_deg',12.)
        joints={f'left_thumb_j{i}':float(np.radians(v)) for i,v in enumerate([*q,fixed,fixed])}
        return {'kind':'depth_thumb_static','available':True,'steps':list(STEPS),'enabled':self.active_model() is not None,
            'has_profile':self.model is not None,'compatible':self.compatible(),'ready':self.pending is not None,
            'collecting':None if not self.collecting else self.collecting['step'],'capture_seconds':self.capture_seconds,
            'phase':'prepare' if elapsed<1 else 'sample','progress':min(1,elapsed/self.capture_seconds),
            'valid_frames':len(self.buffer) if self.collecting else 0,'samples':{s:{'valid_frames':v['valid_frames'],'spread':v['spread']} for s,v in self.samples.items()},
            'rejections':dict(self.rejections),'message':self.message,'references_deg':deepcopy(self.references),
            'selected':self.selected,'reference_preview':self.reference_preview,'reference_revision':self.revision,
            'reference_joints':joints if self.reference_preview else {},
            'bounds_deg':[np.degrees(self.bounds[f'left_thumb_j{i}']).tolist() for i in range(3)]}
