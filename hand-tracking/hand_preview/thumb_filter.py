"""Time-aware thumb task-space smoothing before inverse kinematics."""
from copy import deepcopy
from collections import deque
import math
import numpy as np


class ThumbObservationFilter:
    def __init__(self):self.reset()
    def reset(self):
        self.at=None;self.target=None;self.cues=None;self.raw_goal=None
        self.history=deque(maxlen=3)
        self.steps=deque(maxlen=4)

    def update(self,target,cues,now):
        target=np.asarray(target,float)
        goal=np.asarray(cues['tip'] if cues is not None else target[-1],float)
        if target.shape!=(4,3) or goal.shape!=(3,) or not np.isfinite(target).all() or not np.isfinite(goal).all():
            raise ValueError('Invalid thumb observation')
        if cues is not None and not 0<=float(cues['strength'])<=1:
            raise ValueError('Invalid thumb contact strength')
        if self.at is None or not 0<now-self.at<.25 or (cues is None)!=(self.cues is None):
            self.reset()
            self.target=target.copy();self.cues=deepcopy(cues);self.at=now;self.raw_goal=goal.copy()
            for _ in range(3):self.history.append((target.copy(),goal.copy(),0. if cues is None else float(cues['strength'])))
            return self.target.copy(),deepcopy(self.cues),1.
        dt=now-self.at;self.at=now
        # Median confirmation removes isolated spikes in both geometry and
        # calibrated contacts, even if their amplitude is below a jump gate.
        self.history.append((target.copy(),goal.copy(),0. if cues is None else float(cues['strength'])))
        target=np.median([h[0] for h in self.history],axis=0)
        goal=np.median([h[1] for h in self.history],axis=0)
        step=goal-self.raw_goal;self.raw_goal=goal.copy()
        self.steps.append((step,dt))
        net=float(np.linalg.norm(np.sum([s[0] for s in self.steps],axis=0)))
        travel=sum(float(np.linalg.norm(s[0])) for s in self.steps)
        coherence=net/max(travel,1e-9)
        speed=net/sum(s[1] for s in self.steps)
        activity=float(np.clip((speed-.012)/.045,0,1))*float(np.clip((coherence-.65)/.25,0,1))
        tau=.24-.20*activity
        alpha=1-math.exp(-dt/tau)
        def settle(previous,desired):
            difference=desired-previous
            # A small task-space dead zone suppresses static depth flutter.
            length=float(np.linalg.norm(difference))
            if activity<.1 and length<.0012:return previous.copy()
            return previous+alpha*difference
        self.target=np.vstack((target[0],*[settle(old,new) for old,new in zip(self.target[1:],target[1:])]))
        if cues is not None:
            previous=np.asarray(self.cues['tip'],float)
            strength=float(self.cues['strength'])
            self.cues=deepcopy(cues)
            self.cues['tip']=settle(previous,goal)
            median_strength=float(np.median([h[2] for h in self.history]))
            self.cues['strength']=strength+alpha*(median_strength-strength)
        return self.target.copy(),deepcopy(self.cues),activity
