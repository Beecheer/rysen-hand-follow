"""Small confidence-gated corrections to a visual pose; never a finger IK."""
from copy import deepcopy
import math
import numpy as np
from .geometry import FINGERS,unit,palm_frame,finger_angles


def angle_delta(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


def root_angles(bone,root,basis):
    n=basis[:,2];ref=unit(root-n*(root@n))
    lateral=unit(basis[:,0]-ref*(basis[:,0]@ref))
    planar=math.hypot(bone@ref,bone@lateral)
    mcp=math.atan2(bone@n,planar)
    abd=math.atan2(bone@basis[:,0],bone@basis[:,1]) if planar/np.linalg.norm(bone)>.5 else None
    return mcp,abd


class DepthRootCorrection:
    def __init__(self):self.reset()
    def reset(self):
        self.at=None;self.rotation=np.zeros(3);self.offsets={};self.previous_depth={}

    def update(self,hand,flat,depth_basis,plane_source,plane_count,plane_fresh,now):
        if self.at is None or not 0<now-self.at<.25:
            self.reset();dt=0.
        else:dt=now-self.at
        self.at=now
        raw=np.asarray(hand['model_points_m'],float)
        _,visual,_=palm_frame(raw,hand['label']);base=finger_angles(raw,hand['label'])
        target=np.zeros(3);reason='掌面不可用，退回视觉';palm_ok=False
        n=depth_basis[:,2].copy()
        if n@visual[:,2]<0:n=-n
        disagreement=math.acos(float(np.clip(n@visual[:,2],-1,1)))
        if (plane_fresh and hand.get('sync_valid') and
                ((plane_source=='regional' and plane_count>=10) or (plane_source=='palm_keys' and plane_count>=5))
                and abs(n[2])>.65 and disagreement<math.radians(30)):
            cross=np.cross(visual[:,2],n);size=np.linalg.norm(cross)
            if size>1e-8:target=cross/size*min(math.radians(8),disagreement*.3)
            reason='可靠深度掌面弱修正';palm_ok=True
        alpha=0. if dt==0 else 1-math.exp(-dt/(.18 if palm_ok else .3))
        self.rotation+=(target-self.rotation)*alpha
        size=float(np.linalg.norm(self.rotation));normal=visual[:,2].copy()
        if size>1e-8:
            axis=self.rotation/size
            normal=normal*math.cos(size)+np.cross(axis,normal)*math.sin(size)+axis*(axis@normal)*(1-math.cos(size))
        normal=unit(normal)
        forward=unit(visual[:,1]-normal*(visual[:,1]@normal))
        across=unit(visual[:,0]-normal*(visual[:,0]@normal)-forward*(visual[:,0]@forward))
        basis=np.column_stack((across,forward,normal))
        oriented=finger_angles(raw,hand['label'],basis_override=basis)
        angles=deepcopy(base);details={};obs=hand.get('observations',[])
        pixels=np.asarray(hand['pixels'],float);rays=np.asarray(hand['rays'],float)
        for finger,ids in FINGERS.items():
            root,pip,*_=ids;part=base[finger];desired=np.zeros(2);near_ok=False
            why='近节深度不可用，仅用掌面修正'
            if part['mcp'] is None:
                self.offsets.pop(finger,None);details[finger]={'near_depth_used':False,'reason':'视觉近节无效'};continue
            bone=raw[pip]-raw[root]
            for j,key in enumerate(('mcp','abd')):
                if part[key] is not None and oriented[finger][key] is not None:
                    desired[j]=angle_delta(oriented[finger][key],part[key])
            o=obs[pip] if pip<len(obs) else {}
            # A locally smooth patch alone is insufficient: reject overlapping
            # fingers, foreshortening, incompatible lengths and depth jumps.
            other=[i for ids2 in FINGERS.values() for i in ids2[1:] if i not in ids]
            overlap=bool(other and np.min(np.linalg.norm(pixels[other]-pixels[pip],axis=1))<8.)
            z=o.get('z_m');last=self.previous_depth.get(finger)
            if o.get('valid') and z is not None and math.isfinite(z):self.previous_depth[finger]=(float(z),now)
            else:self.previous_depth.pop(finger,None)
            stable=(last is not None and 0<now-last[1]<.25 and abs(z-last[0])<.008) if z is not None else False
            if (palm_ok and o.get('valid') and stable and o.get('sigma_m',1.)<=.004
                    and not overlap and np.linalg.norm(pixels[pip]-pixels[root])>8.
                    and part['abd'] is not None and abs(part['mcp'])<math.radians(65)):
                measured=rays[pip]*(z+.006)-flat[root]
                length=np.linalg.norm(bone);ratio=np.linalg.norm(measured)/length
                if .75<ratio<1.25 and np.linalg.norm(measured-bone)<.02:
                    corrected=bone+.15*(measured-bone)
                    mcp,abd=root_angles(corrected,raw[root]-raw[0],basis)
                    desired[0]=angle_delta(mcp,part['mcp'])
                    if abd is not None:desired[1]=angle_delta(abd,part['abd'])
                    near_ok=True;why='掌面＋可靠近节深度弱修正'
                else:why='近节深度与视觉骨架不一致，忽略局部深度'
            desired=np.clip(desired,-np.radians([8.,4.]),np.radians([8.,4.]))
            old=self.offsets.get(finger,np.zeros(2))
            offset=old+(desired-old)*(0. if dt==0 else 1-math.exp(-dt/.16))
            self.offsets[finger]=offset
            angles[finger]['mcp']=part['mcp']+float(offset[0])
            # A missing side angle remains missing; never infer it from hidden
            # surfaces. PIP/DIP are copied exactly from the visual model.
            if part['abd'] is not None:angles[finger]['abd']=part['abd']+float(offset[1])
            details[finger]={'near_depth_used':near_ok,'reason':why,
                'mcp_correction_deg':math.degrees(offset[0]),'abd_correction_deg':math.degrees(offset[1]),
                'visual_mcp_deg':math.degrees(part['mcp']),
                'visual_abd_deg':None if part['abd'] is None else math.degrees(part['abd'])}
        report={'palm_used':palm_ok,'palm_reason':reason,'palm_disagreement_deg':math.degrees(disagreement),
                'palm_correction_deg':math.degrees(size),'fingers':details}
        return angles,basis,report
