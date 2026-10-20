"""Front-facing hand reconstruction. Human lengths/DOFs precede robot mapping."""
import math
import numpy as np
from .geometry import FINGERS,palm_frame,unit,joint_angle

PALM=[0,5,9,13,17]


def image_frame(image):
    p=np.asarray(image,float)
    y=unit(p[9]-p[0]);x=p[5]-p[17];x=unit(x-y*np.dot(x,y))
    return x,y,float(np.linalg.norm(p[5]-p[17])/np.linalg.norm(p[9]-p[0]))


def robust_frame(world,handedness,template=None):
    p,raw,length=palm_frame(world,handedness)
    if not np.isfinite(p[PALM]).all():raise ValueError('Invalid palm points')
    if template is not None:
        x=np.asarray(template,float);y=(p[PALM]-p[0])/length
        x=x-x.mean(axis=0);y=y-y.mean(axis=0)
        u,_,vt=np.linalg.svd(x.T@y)
        basis=(u@vt).T
        # Canonical L/R frames can have opposite chirality. Fix the plane's
        # unconstrained normal against the anatomical reference, not det=+1.
        if np.dot(basis[:,2],raw[:,2])<0:basis[:,2]*=-1
        if np.dot(basis[:,1],raw[:,1])<.5:raise ValueError('Unstable palm alignment')
    else:
        points=p[PALM]-p[PALM].mean(axis=0)
        _,_,vt=np.linalg.svd(points)
        n=vt[-1];n*=1 if np.dot(n,raw[:,2])>=0 else -1
        y=unit(raw[:,1]-n*np.dot(raw[:,1],n))
        x=unit(raw[:,0]-n*np.dot(raw[:,0],n)-y*np.dot(raw[:,0],y))
        basis=np.column_stack((x,y,n))
    return p,basis,length


def finger_fk(q,lengths):
    """3 joint points and analytic 9x4 Jacobian: abd, MCP, PIP, DIP."""
    a,m,p,d=q;t=np.array([m,m+p,m+p+d])
    lateral=np.array([math.sin(a),math.cos(a),0.])
    da=np.array([math.cos(a),-math.sin(a),0.]);normal=np.array([0.,0.,1.])
    directions=np.cos(t)[:,None]*lateral+np.sin(t)[:,None]*normal
    positions=np.cumsum(np.asarray(lengths)[:,None]*directions,axis=0)
    jac=np.zeros((3,3,4))
    bend=np.asarray(lengths)[:,None]*(-np.sin(t)[:,None]*lateral+np.cos(t)[:,None]*normal)
    for i in range(3):
        jac[i,:,0]=sum(lengths[j]*math.cos(t[j])*da for j in range(i+1))
        for k in range(1,4):
            if i>=k-1:jac[i,:,k]=bend[k-1:i+1].sum(axis=0)
    return positions,jac.reshape(9,4)


class FrontHandEstimator:
    def __init__(self):self.reset()

    def reset(self):
        self.basis=None;self.at=None;self.previous={};self.thumb_dirs=None
        self.report={'active':False,'message':'完成八姿态标定后启用个人结构约束'}

    def estimate(self,world,image,handedness,now,structure=None):
        p,basis,length=robust_frame(world,handedness,None if structure is None else structure['palm_template'])
        # Filter rotation only. Re-orthogonalize without changing L/R chirality.
        if self.basis is not None and self.at is not None and 0<now-self.at<.2:
            delta=np.linalg.norm(basis-self.basis)
            alpha=.35+.6*min(1.,delta/.3)
            u,_,vt=np.linalg.svd((1-alpha)*self.basis+alpha*basis);basis=u@vt
        self.basis=basis;self.at=now
        ix,iy,aspect=image_frame(image)
        front_score=1.
        if structure:
            ratio=aspect/structure['image_aspect']
            width_score=min(1.,ratio/.8,1.3/max(ratio,1e-6))
            normal_score=float(np.clip(np.dot(basis[:,2],structure['camera_normal']),0,1))
            front_score=min(width_score,normal_score)
        front_ok=front_score>=.7
        clean=p.copy();angles={};quality={}
        for finger,indices in FINGERS.items():
            root,*tips=indices;bones=np.diff(p[list(indices)],axis=0)
            observed_lengths=np.linalg.norm(bones,axis=1)/length
            if not np.isfinite(bones).all() or np.any(observed_lengths<.035) or np.any(observed_lengths>1.4):
                angles[finger]=dict.fromkeys(('abd','mcp','pip','dip'));quality[finger]={'valid':False,'reason':'骨段无效'};continue
            local=bones@basis
            planar=float(np.linalg.norm(local[0,:2])/np.linalg.norm(local[0]))
            side_ok=front_ok and planar>.5
            image_bone=np.asarray(image)[tips[0]]-np.asarray(image)[root]
            side=math.atan2(float(np.dot(image_bone,ix)),float(np.dot(image_bone,iy)))
            a=side if side_ok else self.previous.get(finger,np.zeros(4))[0]
            m=math.atan2(local[0,2],np.linalg.norm(local[0,:2]))
            raw=np.array([a,m,joint_angle(bones[0],bones[1]),joint_angle(bones[1],bones[2])])
            lengths=observed_lengths if structure is None else np.array(structure['bone_lengths'][finger])
            target=(p[tips]-p[root])@basis/length
            # Image yaw and inferred world yaw are different observations.
            # Align the observed chain's yaw before fitting flexion: otherwise
            # a strong 2D side constraint creates an irreducible 3D residual
            # and rejects a perfectly visible finger during calibration.
            world_side=math.atan2(local[0,0],local[0,1])
            if side_ok:
                delta=side-world_side;c,s=math.cos(delta),math.sin(delta)
                target=target@np.array([[c,-s,0],[s,c,0],[0,0,1]])
            q=raw.copy();prior=self.previous.get(finger,raw)
            noise=np.zeros(3) if structure is None else np.asarray(structure.get('noise',{}).get(finger,[0,0,0]))
            reliability=np.clip(.025/(.01+np.cumsum(noise)),.25,1.)
            # Three tiny Gauss-Newton steps, analytic derivatives, no image
            # optimization or mechanical-hand collision solver.
            for _ in range(3):
                predicted,jac=finger_fk(q,lengths);residual=(target-predicted).ravel()
                weights=np.repeat(reliability*np.minimum(1.,.05/(np.linalg.norm(target-predicted,axis=1)+1e-9)),3)
                matrix=jac.T@(weights[:,None]*jac)+.002*np.eye(4)
                gradient=jac.T@(weights*residual)+.002*(prior-q)
                if side_ok:matrix[0,0]+=4.;gradient[0]+=4.*(side-q[0])
                step=np.linalg.solve(matrix,gradient)
                q+=np.clip(step,-.25,.25)
                q=np.clip(q,[-.8,-.4,0,0],[.8,1.8,2.0,2.0])
            predicted,_=finger_fk(q,lengths)
            error=float(np.sqrt(np.mean(np.sum((target-predicted)**2,axis=1))))
            valid=error<.16
            angles[finger]={'abd':float(q[0]) if side_ok and valid else None,
                            'mcp':float(q[1]) if valid else None,'pip':float(q[2]) if valid else None,
                            'dip':float(q[3]) if valid else None}
            quality[finger]={'valid':valid,'side_observable':side_ok,'fit_error_palm':error,
                             'image_side':side,'world_side':world_side}
            if valid:
                self.previous[finger]=q
                clean[tips]=p[root]+predicted@basis.T*length
        thumb_valid=front_ok
        thumb_bones=np.diff(p[1:5],axis=0);thumb_lengths=np.linalg.norm(thumb_bones,axis=1)/length
        if not np.isfinite(thumb_bones).all() or np.any(thumb_lengths<.035) or np.any(thumb_lengths>1.4):thumb_valid=False
        if structure and thumb_valid:
            lengths=np.array(structure['bone_lengths']['thumb'])
            ratio=thumb_lengths/lengths
            if np.any(ratio<.55) or np.any(ratio>1.65):thumb_valid=False
            else:
                directions=thumb_bones@basis/(thumb_lengths[:,None]*length)
                if self.thumb_dirs is not None:
                    change=np.max(np.linalg.norm(directions-self.thumb_dirs,axis=1))
                    alpha=.3+.65*min(1.,change/.35)
                    directions=np.array([unit((1-alpha)*old+alpha*new) for old,new in zip(self.thumb_dirs,directions)])
                self.thumb_dirs=directions
                clean[2:5]=p[1]+np.cumsum(directions*lengths[:,None]*length,axis=0)@basis.T
        self.report={'active':structure is not None,'front_score':front_score,'front_ok':front_ok,
                     'message':('采样中：请保持掌心朝前' if structure is None else '掌心朝向符合参考') if front_ok else '请将掌心转回摄像头；侧摆和拇指暂保持',
                     'fingers':quality,'thumb_valid':thumb_valid}
        canonical=(p-p[0])@basis/length
        observation={'canonical':canonical.tolist(),'image_aspect':aspect,'camera_normal':basis[:,2].tolist(),
                     'bone_lengths':{f:(np.linalg.norm(np.diff(p[list(ids)],axis=0),axis=1)/length).tolist() for f,ids in FINGERS.items()}}
        observation['bone_lengths']['thumb']=thumb_lengths.tolist()
        return angles,clean,thumb_valid,observation


def build_structure(samples):
    neutral=samples['neutral'];obs=neutral.get('structure_observation')
    if not obs:raise ValueError('中立位缺少个人结构观测，请重新采集')
    neutral_angles=neutral['angles']
    if np.median([samples['hook']['angles'][f]['pip']-neutral_angles[f]['pip'] for f in FINGERS])<.2:
        raise ValueError('钩手的指间弯曲变化不足，请重新采集钩手')
    if np.median([samples['mcp']['angles'][f]['mcp']-neutral_angles[f]['mcp'] for f in FINGERS])<.15:
        raise ValueError('指根弯曲变化不足，请重新采集指根轻弯')
    if max(abs(samples['spread']['angles'][f]['abd']-neutral_angles[f]['abd']) for f in FINGERS)<.07:
        raise ValueError('张开姿态与中立位过于接近，请重新采集张开')
    lengths=obs['bone_lengths']
    return {'bone_lengths':lengths,'palm_template':np.asarray(obs['canonical'])[PALM].tolist(),
            'image_aspect':obs['image_aspect'],'camera_normal':obs['camera_normal'],
            'noise':neutral.get('structure_noise',{}),
            'spread_reference':samples['spread']['angles'],
            'flexion_checks':{name:samples[name]['angles'] for name in ('hook','mcp')}}


class MotionCheck:
    """Ten-second consistency report, never a ground-truth accuracy score."""
    def __init__(self):self.active=None;self.result=None;self.samples=[];self.total=0

    def start(self,mode,now):
        if mode not in ('open_close','palm_turn'):raise ValueError('未知验证动作')
        self.active=mode;self.began=now;self.samples=[];self.total=0;self.result=None

    def ingest(self,angles,valid,now):
        if self.active is None:return
        self.total+=1
        if valid and angles and all(angles[f].get(k) is not None for f in FINGERS for k in ('mcp','pip')):
            self.samples.append({f:dict(angles[f]) for f in FINGERS})
        self.snapshot(now)

    def snapshot(self,now):
        elapsed=0 if self.active is None else now-self.began
        if self.active and elapsed>=10:
            ranges={}
            for f in FINGERS:
                ranges[f]={}
                for key in ('abd','mcp','pip'):
                    values=[s[f][key] for s in self.samples if s[f].get(key) is not None]
                    ranges[f][key]=None if len(values)<2 else math.degrees(float(np.percentile(values,95)-np.percentile(values,5)))
            self.result={'mode':self.active,'frames':self.total,'valid_frames':len(self.samples),
                         'valid_ratio':len(self.samples)/max(1,self.total),'range_deg':ranges}
            self.active=None
        return {'active':self.active,'elapsed':min(10,elapsed),'result':self.result}
