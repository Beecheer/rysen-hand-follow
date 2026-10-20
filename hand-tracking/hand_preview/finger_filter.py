"""D435 geometric angle estimate: robust samples and coherent-motion smoothing."""
from collections import deque
from copy import deepcopy
import math
import numpy as np


class FingerAngleFilter:
    def __init__(self):self.reset()
    def reset(self):self.states={}
    def update(self,angles,now):
        result=deepcopy(angles)
        for finger,part in angles.items():
            if finger=='thumb':continue
            for key,value in part.items():
                name=(finger,key)
                if value is None or not math.isfinite(value):
                    self.states.pop(name,None);continue
                state=self.states.get(name)
                if state is None or not 0<now-state['at']<.25:
                    self.states[name]={'at':now,'value':value,'raw':value,
                        'history':deque([value]*3,maxlen=3),'steps':deque(maxlen=4)}
                    continue
                dt=now-state['at'];state['at']=now
                state['history'].append(value);median=float(np.median(state['history']))
                state['steps'].append((median-state['raw'],dt));state['raw']=median
                net=abs(sum(v for v,_ in state['steps']));travel=sum(abs(v) for v,_ in state['steps'])
                coherence=net/max(travel,1e-9);speed=net/sum(t for _,t in state['steps'])
                activity=float(np.clip((speed-.08)/1.2,0,1))*float(np.clip((coherence-.55)/.35,0,1))
                tau=.14-.115*activity;difference=median-state['value']
                if activity>.1 or abs(difference)>math.radians(.35):
                    state['value']+=(1-math.exp(-dt/tau))*difference
                result[finger][key]=state['value']
        return result
