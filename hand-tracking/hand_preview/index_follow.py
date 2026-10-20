"""Windows index/full-hand targets, browser lease and follow event logging."""
import json
import math
from pathlib import Path
import threading
import time
from .bridge_client import BridgeClient
from .joint_layout import JOINT_NAMES


def full_hand_target(source,bounds,allow_held_thumb=False,allow_held_flexion=False,max_age_ms=200):
    if max_age_ms not in (200,400):raise ValueError('Unsupported camera target age limit')
    if not source.get('valid') or source.get('state')!='TRACKING' or source.get('age_ms') is None or source['age_ms']>=max_age_ms:
        raise ValueError('请完整展示左手并恢复识别')
    for name in source.get('held_joints',[]):
        if allow_held_thumb and name.startswith('left_thumb_'):continue
        if allow_held_flexion and not name.startswith('left_thumb_'):continue
        if name.startswith('left_thumb_') or not name.endswith('_j0'):
            raise ValueError('拇指或手指屈伸识别无效，已暂停全手跟随')
    values=source.get('joints',{})
    if set(values)!=set(JOINT_NAMES) or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in values.values()):
        raise ValueError('全手目标不完整或无效')
    result={n:max(bounds[n][0],min(bounds[n][1],values[n])) for n in JOINT_NAMES}
    for child,parent in ((4,3),(8,7),(12,11),(16,15),(20,19)):result[JOINT_NAMES[child]]=result[JOINT_NAMES[parent]]
    return [result[n] for n in JOINT_NAMES]


def index_target(source):
    if (not source.get('valid') or source.get('state')!='TRACKING' or source.get('age_ms') is None
            or source['age_ms']>150 or 'left_index_j1' in source.get('held_joints',[])):
        raise ValueError('请完整展示左手食指并恢复识别')
    value=source.get('joints',{}).get('left_index_j1')
    if not isinstance(value,(float,int)) or not math.isfinite(value):raise ValueError('食指目标无效')
    return max(0.,min(10.,math.degrees(value)*10/65))


class IndexFollow:
    def __init__(self,source,stop,token_path,full_hand=False,hardware_status=None,event_path=None,target_max_age_ms=200):
        self.source=source;self.stop=stop;self.token=Path(token_path).read_text().strip()
        self.full_hand=full_hand
        if target_max_age_ms not in (200,400):raise ValueError('Unsupported camera target age limit')
        self.target_max_age_ms=target_max_age_ms
        self.hardware_status=hardware_status
        self.started_at=0
        self.bounds=json.loads(Path(token_path).with_name('full-hand-limits.json').read_text())['bounds'] if full_hand else None
        self.lock=threading.Lock();self.io_lock=threading.Lock();self.lease=0;self.requested=False
        self.reason='尚未开始；设备保持失能';self.client=BridgeClient()
        self.event_path=Path(event_path) if event_path else None
        self.event_lock=threading.Lock()
        self.held_since={}

    def event(self,kind,reason):
        if self.event_path:
            self.event_path.parent.mkdir(parents=True,exist_ok=True)
            with self.event_lock,self.event_path.open('a',encoding='utf-8') as stream:
                stream.write(json.dumps({'time':time.time(),'event':kind,'reason':reason},ensure_ascii=False)+'\n')

    def post(self,action,degrees=0):
        payload=json.dumps({'action':action,('positions' if self.full_hand else 'degrees'):degrees}).encode()
        with self.io_lock:
            self.client.request('POST','/control',payload,{'Content-Type':'application/json','X-Control-Token':self.token})

    def action(self,action):
        if action=='heartbeat':
            with self.lock:
                if self.requested:self.lease=time.monotonic()
            return
        if action=='stop':
            self.event('manual_stop',self.reason)
            with self.lock:self.requested=False;self.lease=0;self.reason='已请求暂停并失能'
            self.post('stop');return
        if action!='start':raise ValueError('未知跟随操作')
        self.held_since={}
        target=self.target()
        self.post('start',target)
        with self.lock:self.requested=True;self.lease=self.started_at=time.monotonic();self.reason='已请求开始全手跟随' if self.full_hand else '已请求开始食指跟随'
        self.event('start',self.reason)

    def snapshot(self):
        with self.lock:return {'requested':self.requested,'reason':self.reason,'full_hand':self.full_hand,
                              'target_max_age_ms':self.target_max_age_ms}

    def stop_confirmed(self):
        """Configuration changes require observed fresh disabled feedback."""
        self.action('stop');deadline=time.monotonic()+1.2
        while time.monotonic()<deadline:
            with self.io_lock:status=self.client.request('GET','/status')
            if (status.get('connected') and status.get('feedback_age_ms',1000)<250 and
                    status.get('worker_age_ms',1000)<250 and not status.get('follow',{}).get('active') and
                    len(status.get('enabled',{}))==5 and not any(status['enabled'].values())):
                return
            self.stop.wait(.025)
        raise ValueError('SDK 未确认五指失能，配置保持不变')

    def target(self,allow_held_thumb=False):
        source=self.source()
        if not self.full_hand:return index_target(source)
        held={n for n in source.get('held_joints',[]) if not n.startswith('left_thumb_') and not n.endswith('_j0')}
        now=time.monotonic()
        self.held_since={n:self.held_since.get(n,now) for n in held}
        if allow_held_thumb and any(now-at>.5 for at in self.held_since.values()):
            raise ValueError('局部手指屈伸连续无效超过 500 ms，已暂停并失能')
        return full_hand_target(source,self.bounds,allow_held_thumb,allow_held_flexion=allow_held_thumb,
                                max_age_ms=getattr(self,'target_max_age_ms',200))

    def run(self):
        try:
            while not self.stop.wait(.05):
                with self.lock:requested=self.requested;lease=self.lease
                if not requested:continue
                try:
                    if time.monotonic()-lease>.6:raise ValueError('页面心跳超时，已暂停并失能')
                    if self.hardware_status and time.monotonic()-self.started_at>.5:
                        status=self.hardware_status();follow=status.get('follow',{})
                        if status.get('connection_error_layer')=='local_bridge':
                            raise ValueError('本地 SDK 桥通信异常：'+str(status.get('connection_error')))
                        if not follow.get('active'):
                            raise ValueError('跟随未启动或已停止：'+str(follow.get('last_stop_reason') or follow.get('reason') or 'SDK 状态待确认'))
                    self.post('target',self.target(allow_held_thumb=True))
                except Exception as exc:
                    message='控制桥通信超时，已暂停并失能' if isinstance(exc,TimeoutError) else str(exc)
                    with self.lock:self.requested=False;self.lease=0;self.reason=message
                    self.event('automatic_stop',message)
                    try:self.post('stop')
                    except Exception:pass # Native 250 ms target watchdog is independent.
        finally:
            try:self.post('stop')
            except Exception:pass
            with self.io_lock:self.client.close()
