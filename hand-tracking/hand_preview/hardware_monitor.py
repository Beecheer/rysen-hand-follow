"""Monitor live SDK feedback; retain the optional read-only dry-run planner."""
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time
from .bridge_client import BridgeClient

from .joint_layout import JOINT_NAMES


def feedback_problem(data,transit_ms=0):
    if data.get('mode')!='READ_ONLY' or data.get('motion_calls')!=0:return '桥接模式不符'
    if not data.get('connected') or data.get('hand')!='LEFT':return '左手未连接'
    if data.get('error'):return '反馈异常：'+str(data['error'])
    if data.get('fault_code')!=0:return '设备故障：'+str(data.get('fault'))
    age=data.get('feedback_age_ms')
    if not isinstance(age,(float,int)) or not math.isfinite(age) or age<0 or age+transit_ms>250:
        return '关节反馈超时'
    positions=data.get('positions',{})
    if set(positions)!=set(JOINT_NAMES) or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in positions.values()):
        return '关节反馈不完整'
    if data.get('joint_layout')!=[{'id':i,'name':n} for i,n in enumerate(JOINT_NAMES)]:return 'SDK 关节顺序不匹配'
    for key in ('max_speed','max_accel'):
        values=data.get(key,[])
        if len(values)!=21 or not all(isinstance(v,(float,int)) and math.isfinite(v) and v>0 for v in values):
            return '设备运动参数不完整'
    return None


class DryRunPlanner:
    """Position/velocity-limited simulation; emits explicit 21 JointId entries.

    Initial output is actual feedback, never a jump to the camera pose.
    This is diagnostic only, not a validated physical trajectory controller.
    """
    def __init__(self,mapper):
        self.specs=mapper.specs;self.bounds=mapper.bounds;self.reset()

    def reset(self):self.positions=None;self.velocity={};self.at=None

    def step(self,source,feedback,now,transit_ms=0):
        reason=feedback_problem(feedback,transit_ms)
        age=source.get('age_ms')
        if not reason and (not source.get('valid') or source.get('state')!='TRACKING'
                           or age is None or age>200):reason='等待有效左手跟随；不生成命令'
        targets=source.get('joints',{})
        if not reason and (set(targets)!=set(JOINT_NAMES) or not all(math.isfinite(v) for v in targets.values())):
            reason='预览目标无效'
        if reason:
            self.reset();return {'state':'BLOCKED','reason':reason,'commands':[]}
        if self.positions is None or now-self.at>.2:
            self.positions=dict(feedback['positions']);self.velocity={n:0.0 for n in self.bounds};self.at=now
            phase='SEEDED';reason='从真机反馈位置初始化模拟轨迹'
        else:
            dt=min(.1,max(.001,now-self.at));self.at=now
            phase='SIMULATING';reason='仅模拟位置跟随命令，未发送到设备'
            for name,(lo,hi) in self.bounds.items():
                index=JOINT_NAMES.index(name)
                speed=min(math.radians(20),feedback['max_speed'][index])
                goal=max(lo,min(hi,targets[name]));delta=goal-self.positions[name]
                # Position slew limit: avoid asserting unverified acceleration
                # guarantees at goal crossings or mimic projection boundaries.
                step=max(-speed*dt,min(speed*dt,delta))
                self.positions[name]+=step;self.velocity[name]=step/dt
            for name,spec in self.specs.items():
                mimic=spec['mimic']
                if mimic:
                    self.positions[name]=self.positions[mimic['joint']]*mimic['multiplier']+mimic['offset']
        outside=[n for n,p in feedback['positions'].items()
                 if p<self.specs[n]['lower']-.035 or p>self.specs[n]['upper']+.035]
        return {'state':phase,'reason':reason,'speed_limit_deg_s':20,'outside_model':outside,
                'commands':[{'id':i,'name':name,'position':self.positions[name]} for i,name in enumerate(JOINT_NAMES)]}


class HardwareMonitor:
    def __init__(self,mapper,source,stop,log_path=None):
        self.planner=DryRunPlanner(mapper);self.source=source;self.stop=stop
        self.lock=threading.Lock();self.data={'mode':'DRY_RUN','reason':'正在连接只读桥','motion_calls':0}
        self.log_path=Path(log_path) if log_path else None

    def snapshot(self):
        with self.lock:data=deepcopy(self.data)
        stamp=data.pop('_at',None)
        if stamp is not None:
            elapsed=(time.monotonic()-stamp)*1000
            data['monitor_age_ms']=elapsed
            if data.get('feedback_age_ms') is not None:data['feedback_age_ms']+=elapsed
            if elapsed>250 or (data.get('feedback_age_ms') or 0)>250:
                data['dry_run']={'state':'BLOCKED','reason':'监控或关节反馈超时','commands':[]}
        return data

    def run(self):
        client=BridgeClient(timeout=.4);last_log=0;log=None
        try:
            if self.log_path:
                self.log_path.parent.mkdir(exist_ok=True,parents=True)
                log=self.log_path.open('a',encoding='utf-8')
            while not self.stop.is_set():
                begin=time.monotonic()
                try:
                    feedback=client.request('GET','/status')
                    transit=(time.monotonic()-begin)*1000
                    source=self.source()
                    live=feedback.get('control_capable',False)
                    dry=({'state':'LIVE_MODE','reason':('全手' if feedback.get('full_hand') else '食指')+'实机模式：由独立控制面板开始或暂停','commands':[]}
                         if live else self.planner.step(source,feedback,time.monotonic(),transit))
                    data={**feedback,'mode':feedback.get('mode','INDEX_FOLLOW') if live else 'DRY_RUN',
                          'motion_calls':feedback.get('motion_calls',0),'bridge_rtt_ms':transit,
                          'feedback_age_ms':(feedback.get('feedback_age_ms') or 0)+transit,
                          'dry_run':dry,'preview_targets':source['joints'],'_at':time.monotonic()}
                except Exception as exc:
                    self.planner.reset()
                    with self.lock:data=deepcopy(self.data)
                    data.update(connected=False,connection_error_layer='local_bridge',connection_error=type(exc).__name__+': '+str(exc),
                                dry_run={'state':'BLOCKED','reason':'本地 SDK 桥通信异常：'+str(exc),'commands':[]})
                    if 'follow' in data:data['follow']['active']=False
                with self.lock:self.data=data
                if log and (begin-last_log>=1 or data.get('connection_error')):
                    log.write(json.dumps({'time':time.time(),**{k:v for k,v in data.items() if k!='_at'}},ensure_ascii=False)+'\n')
                    log.flush();last_log=begin
                self.stop.wait(max(0,.05-(time.monotonic()-begin)))
        finally:
            client.close()
            if log:log.close()
