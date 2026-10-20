"""RGB-led articulated fingers and a non-iterative thumb for the D435.

The legacy fused joint coordinates are deliberately not an input. Depth patches
are surface observations; they are never substituted for joint centres.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from .geometry import FINGERS, unit, finger_angles, palm_frame
from .human_pose import finger_fk
from .depth_root_correction import DepthRootCorrection

PALM=np.array([0,5,9,13,17])
LOWER=np.array([-.6,-.12,0.,0.])
UPPER=np.array([.6,1.65,2.,1.5])
KEYS=('abd','mcp','pip','dip')


def project(points):
    p=np.asarray(points,float)
    return p[:,:2]/np.maximum(p[:,2:3],.08)


def rays_for(hand):
    if 'rays' in hand:rays=np.asarray(hand['rays'],float)
    else:
        # Compatibility with recorded numerical diagnostics, not camera fusion.
        p=np.asarray(hand['model_points_m'],float);rays=p/p[:,2:3]
    if rays.shape!=(21,3) or not np.isfinite(rays).all():raise ValueError('无效图像投影射线')
    return rays


def fit_plane(points,reference_normal=None):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<5 or not np.isfinite(p).all():raise ValueError('掌部深度点不足')
    # Mixed palm/finger surfaces during a fist are not one noisy plane. Pick
    # a supported plane first, then refine its inliers, instead of rejecting
    # the complete hand after a global SVD through both surfaces.
    keep=np.ones(len(p),bool)
    if len(p)>=10:
        rng=np.random.default_rng(19);best=None
        for _ in range(48):
            sample=p[rng.choice(len(p),3,replace=False)]
            cross=np.cross(sample[1]-sample[0],sample[2]-sample[0])
            if np.linalg.norm(cross)<.00002:continue
            n=unit(cross)
            if n[2]>0:n=-n
            if abs(n[2])<.65:continue
            if reference_normal is not None and n@reference_normal<.85:continue
            error=abs((p-sample[0])@n);inliers=error<.004
            if inliers.sum()<max(6,len(p)*.3):continue
            spread=np.linalg.svd(p[inliers]-p[inliers].mean(axis=0),compute_uv=False)
            if spread[1]<.006:continue
            score=float(inliers.sum())-float(np.median(error[inliers]))*100
            if best is None or score>best[0]:best=(score,inliers)
        if best is not None:keep=best[1]
    for _ in range(3):
        center=np.median(p[keep],axis=0)
        _,s,vt=np.linalg.svd(p[keep]-center,full_matrices=False)
        if s[1]<.006:raise ValueError('掌部深度覆盖不足')
        normal=vt[-1];error=abs((p-center)@normal)
        # Never re-admit a separate finger surface during refinement.
        keep=keep&(error<max(.003,float(np.median(error[keep]))*2.5))
        if keep.sum()<5:raise ValueError('掌部深度表面不一致')
    if np.percentile(error[keep],90)>.008:raise ValueError('掌部深度噪声过大')
    if normal[2]>0:normal=-normal
    return center,unit(normal),int(keep.sum())


def intersect(rays,center,normal):
    den=rays@normal
    if np.any(abs(den)<.2):raise ValueError('掌面过于侧向，请掌心正对摄像头')
    z=float(center@normal)/den
    if np.any(z<.12) or np.any(z>2.):raise ValueError('掌面超出深度范围')
    return rays*z[:,None]


def thumb_features(points,basis):
    # Landmark 4 is intentionally unused, including for validity checks.
    v=np.diff(np.asarray(points)[1:4],axis=0)@basis
    a,b=unit(v[0]),unit(v[1])
    plane=math.atan2(a[1],a[0])
    lift=math.atan2(a[2],np.linalg.norm(a[:2]))
    axis=unit(np.cross(a,[0.,0.,1.])) if np.linalg.norm(a[:2])>.1 else np.array([0.,1.,0.])
    bend=math.atan2(float(np.cross(a,b)@axis),float(a@b))
    return np.array([plane,lift,bend])


class DirectThumb:
    """Three measured angles mapped independently; no FK fitting or optimiser."""
    def __init__(self,bounds,config):
        self.bounds=bounds;self.config=config;self.previous=None;self.at=None
        self.fit_error=None;self.diagnostics={};self.observation_filter=self
    def reset(self):self.previous=None;self.at=None;self.pending=None;self.history=[];self.diagnostics={}
    def calculate(self,points,basis,reference,now,reliable=True):
        fixed=math.radians(self.config['fixed_terminal_deg'])
        lo,hi=self.bounds['left_thumb_j3'];fixed=float(np.clip(fixed,lo,hi))
        if not reliable:
            self.diagnostics={'method':'direct_angles','held':True,'fixed_terminal_deg':math.degrees(fixed)}
            return {'j0':None,'j1':None,'j2':None,'j3':fixed}
        feature=thumb_features(points,basis)
        zero=np.asarray(reference if reference is not None else feature)
        delta=np.arctan2(np.sin(feature-zero),np.cos(feature-zero))
        if self.at is None or not 0<now-self.at<.25:self.history=[]
        self.history=(getattr(self,'history',[])+[delta.copy()])[-3:]
        if len(self.history)==3:delta=np.median(self.history,axis=0)
        gains=np.asarray(self.config['gains'])
        q=np.radians(self.config['neutral_deg'])+gains*delta
        q=self.smooth(q,now)
        result={f'j{i}':float(np.clip(q[i],*self.bounds[f'left_thumb_j{i}'])) for i in range(3)}
        result['j3']=fixed
        self.diagnostics={'method':'direct_angles','measured_deg':np.degrees(feature).tolist(),
                          'fixed_terminal_deg':math.degrees(fixed),'held':False}
        return result

    def smooth(self,q,now):
        q=np.array([np.clip(q[i],*self.bounds[f'left_thumb_j{i}']) for i in range(3)])
        if self.previous is not None and 0<now-self.at<.25:
            # Confirm isolated large changes before accepting them; continuous
            # motion is smoothed in angle space, never by chasing a fingertip.
            if np.max(abs(q-self.previous))>math.radians(35):
                pending=getattr(self,'pending',None);self.pending=q.copy()
                if pending is None or np.max(abs(q-pending))>math.radians(10):q=self.previous.copy()
            else:self.pending=None
            alpha=1-math.exp(-(now-self.at)/.065)
            q=self.previous+alpha*(q-self.previous)
        q=np.array([np.clip(q[i],*self.bounds[f'left_thumb_j{i}']) for i in range(3)])
        self.previous=q.copy();self.at=now
        return q


class ArticulatedDepthEstimator:
    def __init__(self,vision_prior=False,depth_root_correction=False):
        self.vision_prior=vision_prior;self.depth_root_correction=depth_root_correction;self.reset()
    def reset(self):
        self.basis=None;self.center=None;self.at=None;self.depth_at=None
        self.previous={};self.pending={};self.last_targets={};self.last_geometry=None
        self.root_correction=DepthRootCorrection();self.root_report={}
        self.report={'active':True,'message':'图像主导关节链；等待可靠掌面'}

    def palm(self,hand,now):
        self.palm_measured=False
        rays=rays_for(hand)
        samples=hand.get('palm_surface_points_m',[])
        if len(samples)<5:
            obs=hand.get('observations',[])
            samples=[rays[i]*obs[i]['z_m'] for i in PALM if i<len(obs) and obs[i].get('valid')]
        try:
            if not hand.get('sync_valid',False):raise ValueError('彩深不同步')
            try:
                center,n,count=fit_plane(samples,None if self.basis is None else self.basis[:,2])
                plane_source='regional'
            except ValueError:
                obs=hand.get('observations',[])
                keys=[rays[i]*obs[i]['z_m'] for i in PALM if i<len(obs) and obs[i].get('valid')]
                try:
                    center,n,count=fit_plane(keys,None if self.basis is None else self.basis[:,2])
                    plane_source='palm_keys'
                except ValueError:
                    # Frontal-operation constraint: a visible wrist can refresh
                    # translation while the fingers cover the palm. This uses
                    # new measured depth, not an indefinitely stale Z value.
                    wrist=obs[0] if obs else {}
                    if self.basis is None or not wrist.get('valid'):raise
                    n=self.basis[:,2].copy();center=rays[0]*wrist['z_m'];count=1
                    plane_source='wrist_depth'
            # Surface -> approximate joint-centre palm plane, fixed 6 mm offset.
            center=center-n*.006
            palm=intersect(rays[PALM],center,n)
            y=unit(palm[2]-palm[0]);y=unit(y-n*(y@n))
            x=unit(palm[1]-palm[4]);x=unit(x-n*(x@n)-y*(x@y))
            # Keep anatomical flexion normal independent of raw MP chirality.
            if self.basis is not None and self.at is not None and 0<now-self.at<.25:
                if n@self.basis[:,2]<.5:raise ValueError('掌面朝向突变')
                alpha=1-math.exp(-(now-self.at)/.08)
                n=unit((1-alpha)*self.basis[:,2]+alpha*n)
                y=unit((1-alpha)*self.basis[:,1]+alpha*y);y=unit(y-n*(y@n))
                x=unit(x-n*(x@n)-y*(x@y))
                center=(1-alpha)*self.center+alpha*center
            self.center=center;self.basis=np.column_stack((x,y,n));self.depth_at=now
            self.plane_count=count
            self.plane_source=plane_source
            self.palm_measured=plane_source!='wrist_depth'
        except ValueError:
            if self.depth_at is None or now-self.depth_at>.25:raise
        self.at=now
        return rays,intersect(rays,self.center,self.basis[:,2]),self.basis

    def calibration_geometry(self,hand,now):
        rays,flat,basis=self.palm(hand,now)
        # Both calibration poses are straight and frontal. Ray/plane lengths
        # avoid fitting occluded joints and reject obvious non-straight poses.
        points=flat.copy()
        return points,basis,rays

    def estimate(self,hand,now,profile=None):
        if self.at is not None and now-self.at>.3:self.reset()
        rays,flat,basis=self.palm(hand,now)
        raw=np.asarray(hand['model_points_m'],float)
        visual_angles=finger_angles(raw,hand['label']) if self.vision_prior else None
        if self.vision_prior and self.depth_root_correction:
            visual_angles,_,self.root_report=self.root_correction.update(hand,flat,basis,
                self.plane_source,self.plane_count,self.palm_measured,now)
        structure=None if profile is None else profile['skeleton']
        lengths={f:np.linalg.norm(np.diff(raw[list(ids)],axis=0),axis=1)
                 for f,ids in FINGERS.items()} if structure is None else structure['finger_lengths_m']
        obs=hand.get('observations',[{} for _ in range(21)])
        palm_scale=float(np.linalg.norm(flat[9]-flat[0]))
        if not .025<palm_scale<.15:raise ValueError('手掌尺度异常')
        focal=float(hand.get('focal_px',600.))
        clean=flat.copy();angles={};details={}
        # A surface normal facing the camera is not necessarily the anatomical
        # flexion normal (raw model chirality / mirrored driver coordinates).
        # Keep the camera frame for the stored thumb-position calibration, and
        # derive finger flexion exactly as geometry.palm_frame does.
        finger_basis=basis.copy()
        finger_basis[:,2]=unit(np.cross(basis[:,0],basis[:,1]))*(-1 if hand['label']=='Left' else 1)
        # Use the last articulated prediction for conservative visibility tests.
        predicted=flat.copy()
        if self.last_geometry is not None:
            predicted=flat[0]+(self.last_geometry-self.last_geometry[0])
        for finger,ids in FINGERS.items():
            root,*tips=ids;length=np.asarray(lengths[finger],float)
            if length.shape!=(3,) or np.any(length<.005) or np.any(length>.09):
                angles[finger]=dict.fromkeys(KEYS);continue
            if visual_angles is not None:
                # Restore the monocular model's coherent 3D pose. Local
                # surface depths and ambiguous 2D reprojection do not refit
                # finger angles. Fixed lengths affect the display only.
                part=visual_angles[finger];angles[finger]=part.copy()
                valid=part['mcp'] is not None
                if valid:
                    bones=np.diff(raw[list(ids)],axis=0)
                    directions=bones/np.linalg.norm(bones,axis=1)[:,None]
                    if self.depth_root_correction:
                        # Display the corrected proximal direction as a rigid
                        # chain rotation; distal bend angles stay unchanged.
                        _,vb,_=palm_frame(raw,hand['label'])
                        original=finger_angles(raw,hand['label'])[finger]
                        side=part['abd'] if part['abd'] is not None else original['abd']
                        if side is not None:
                            desired=(math.cos(part['mcp'])*(math.sin(side)*vb[:,0]+math.cos(side)*vb[:,1])
                                     +math.sin(part['mcp'])*vb[:,2])
                            cross=np.cross(directions[0],desired);sine=np.linalg.norm(cross)
                            if sine>1e-8:
                                axis=cross/sine;theta=math.atan2(sine,float(directions[0]@desired))
                                directions=(directions*math.cos(theta)+np.cross(axis,directions)*math.sin(theta)
                                            +np.outer(directions@axis,axis)*(1-math.cos(theta)))
                    clean[tips]=flat[root]+np.cumsum(directions*length[:,None],axis=0)
                details[finger]={'valid':valid,'held':not valid,
                    'side_observable':part['abd'] is not None,
                    'side_held':part['abd'] is None,'method':'rgb_model_3d'}
                if self.depth_root_correction:details[finger].update(self.root_report['fingers'].get(finger,{}))
                continue
            previous=self.previous.get(finger)
            # Initialise MCP from projected shortening; MP relative Z is not
            # trusted as a motion prior. Nonnegative PIP/DIP prohibit backfolds.
            projected=flat[tips[0]]-flat[root]
            side=math.atan2(projected@basis[:,0],projected@basis[:,1])
            m=math.acos(float(np.clip(np.linalg.norm(projected)/length[0],0,1)))
            # The visible proximal endpoint lies on an RGB ray and at a fixed
            # distance from MCP. Its ray/sphere intersection gives a direct
            # flexion observation and prevents exchanging MCP bend for PIP.
            ray=rays[tips[0]];root_point=flat[root]
            aa=ray@ray;bb=-2*(ray@root_point);cc=root_point@root_point-length[0]**2
            disc=bb*bb-4*aa*cc;proximal=None
            if disc>=0:
                choices=[]
                for z in ((-bb-math.sqrt(disc))/(2*aa),(-bb+math.sqrt(disc))/(2*aa)):
                    v=(ray*z-root_point)@finger_basis
                    bend=math.atan2(v[2],np.linalg.norm(v[:2]))
                    if -.12<=bend<=1.65 and v[1]>-.001:choices.append((bend,z))
                if choices:
                    proximal_obs=obs[tips[0]] if tips[0]<len(obs) else {}
                    def candidate_cost(candidate):
                        bend,z=candidate
                        continuity=abs(bend-(0. if previous is None else previous[1]))/.5
                        measured=abs(z-proximal_obs['z_m']-.006)/.006 if proximal_obs.get('valid') else 0.
                        return continuity+measured
                    proximal=min(choices,key=candidate_cost)[0]
                    m=proximal
            seed=np.array([side,m,.15,.1]) if previous is None else previous.copy()
            weights=np.ones(3);depth_weights=np.zeros(3);occluded=[]
            for j,i in enumerate(tips):
                o=obs[i] if i<len(obs) else {}
                if o.get('valid') and hand.get('sync_valid'):
                    depth_weights[j]=1.
                    # If a surface is clearly in front of the predicted joint,
                    # it belongs to an occluder, not to this joint centre.
                    if previous is not None and predicted[i,2]>o['z_m']+.024:
                        depth_weights[j]=0.;weights[j]=.25;occluded.append(i)
                # Folded overlapping phalanges are ambiguous even when the
                # depth patch is statistically clean. Keep it one-sided below.
                other=[k for k in range(5,21) if k not in ids and k not in PALM]
                if other and np.min(np.linalg.norm(project(rays[[i]])-project(rays[other]),axis=1))*focal<7:
                    weights[j]=min(weights[j],.35);depth_weights[j]*=.2
                    if i not in occluded:occluded.append(i)
                if j>0 and np.linalg.norm(project(rays[[i]])-project(rays[[tips[j-1]]]))*focal<6:
                    weights[j]=min(weights[j],.3);depth_weights[j]*=.2
                    if i not in occluded:occluded.append(i)
            target=project(rays[tips])
            relative_target=target-project(rays[[root]])
            old_target=self.last_targets.get(finger)
            image_motion=0. if old_target is None else float(np.sqrt(np.mean((relative_target-old_target)**2))*focal)
            self.last_targets[finger]=relative_target.copy()
            temporal_weights=np.array([2.,2.,2.,1.5])*(1-.85*float(np.clip(image_motion/12,0,1)))
            if occluded:temporal_weights*=2.
            # Flexion moves the image even when abduction is stationary. Do
            # not use that motion to weaken abduction continuity. A shortened
            # proximal bone has progressively less information about its yaw.
            lateral_visibility=1. if previous is None else abs(math.cos(previous[1]))
            temporal_weights[0]=16./max(.25,lateral_visibility)
            if tips[0] in occluded:temporal_weights[0]*=2.
            def residual_jac(q):
                local,jlocal=finger_fk(q,length);p=flat[root]+local@finger_basis.T
                jp=np.einsum('ab,nbk->nak',finger_basis,jlocal.reshape(3,3,4))
                z=p[:,2,None,None]
                ji=(jp[:,:2,:]/z-p[:,:2,None]*jp[:,2:3,:]/z**2)*focal/2.5*weights[:,None,None]
                image=((project(p)-target)*focal/2.5*weights[:,None]).ravel()
                depth=[];jd=np.zeros((3,4))
                for j,i in enumerate(tips):
                    o=obs[i] if i<len(obs) else {}
                    if not o.get('valid'):depth.append(0.);continue
                    z=p[j,2];surface=o['z_m']
                    # A detected occlusion contributes only "behind surface".
                    if i in occluded:
                        depth.append(min(0.,z-surface+.003)/.015)
                        if z<surface-.003:jd[j]=jp[j,2]/.015
                    else:
                        depth.append((min(0.,z-surface+.003)+max(0.,z-surface-.016))/.015*depth_weights[j])
                        if z<surface-.003 or z>surface+.016:jd[j]=jp[j,2]/.015*depth_weights[j]
                prior=seed if previous is None else previous
                temporal=(q-prior)*temporal_weights
                proximal_weight=0. if proximal is None or tips[0] in occluded else 3.
                proximal_error=0. if proximal is None else proximal_weight*(q[1]-proximal)
                residual=np.concatenate((image,depth,[proximal_error],temporal))
                jac=np.vstack((ji.reshape(6,4),jd,[[0.,proximal_weight,0.,0.]],np.diag(temporal_weights)))
                return residual,jac
            def loss(squared):
                r=np.sqrt(1+squared);rho=np.vstack((2*(r-1),1/r,-.5/r**3))
                rho[0,-4:]=squared[-4:];rho[1,-4:]=1.;rho[2,-4:]=0.
                return rho
            fit=least_squares(lambda q:residual_jac(q)[0],np.clip(seed,LOWER+1e-5,UPPER-1e-5),
                              jac=lambda q:residual_jac(q)[1],bounds=(LOWER,UPPER),
                              loss=loss,max_nfev=18,ftol=1e-4,xtol=1e-4)
            q=fit.x
            def image_error(candidate):
                points=flat[root]+finger_fk(candidate,length)[0]@finger_basis.T
                error=(project(points)-target)*focal
                return float(np.sqrt(np.mean(error**2))),float(np.sqrt(np.mean((error*weights[:,None])**2)))
            err,weighted_error=image_error(q)
            reseeded=False
            if err>12 and weighted_error>4:
                # A straight chain has a poor flexion gradient. A fast fold
                # needs one bent initial pose, not another solve at the same
                # straight bound. Run only when observations contradict it.
                alternate=np.array([np.clip(side,-.5,.5),max(.65,m),1.05,.6])
                other=least_squares(lambda q:residual_jac(q)[0],np.clip(alternate,LOWER+1e-5,UPPER-1e-5),
                                    jac=lambda q:residual_jac(q)[1],bounds=(LOWER,UPPER),loss=loss,
                                    max_nfev=16,ftol=1e-4,xtol=1e-4)
                _,other_error=image_error(other.x)
                if other.cost<fit.cost*.8 and other_error<weighted_error*.8:
                    fit=other;q=other.x;reseeded=True
            local,_=finger_fk(q,length);p=flat[root]+local@finger_basis.T
            err=float(np.sqrt(np.mean(((project(p)-target)*focal)**2)))
            weighted_error=float(np.sqrt(np.mean((((project(p)-target)*focal)*weights[:,None])**2)))
            # Apply the same visibility weights used by the fit. A hidden tip
            # cannot veto a chain whose visible proximal points still agree.
            if weighted_error>18 or not np.isfinite(q).all():
                angles[finger]=dict.fromkeys(KEYS);details[finger]={'held':True,'image_error_px':err,'weighted_error_px':weighted_error};continue
            jump_held=False
            if previous is not None and np.max(abs(q[1:]-previous[1:]))>math.radians(35):
                _,previous_error=image_error(previous)
                supported=previous_error-weighted_error>5 and weighted_error<previous_error*.6
                pending=self.pending.get(finger);self.pending[finger]=q.copy()
                if not supported and (pending is None or np.max(abs(q-pending))>math.radians(12)):
                    q=previous.copy();jump_held=True
                else:self.pending.pop(finger,None)
            else:self.pending.pop(finger,None)
            side_observable=tips[0] not in occluded and abs(math.cos(q[1]))>.35
            side_held=not side_observable
            if side_held:
                # Keep only the unobservable DOF; the flexion chain continues
                # updating and will not latch while the proximal bone folds.
                q[0]=0. if previous is None else previous[0]
            fit_error,fit_weighted_error=err,weighted_error
            err,weighted_error=image_error(q)
            self.previous[finger]=q.copy()
            clean[tips]=flat[root]+finger_fk(q,length)[0]@finger_basis.T
            angles[finger]={k:float(v) for k,v in zip(KEYS,q)}
            details[finger]={'held':False,'image_error_px':err,'occluded_landmarks':occluded,
                             'fit_image_error_px':fit_error,'fit_weighted_error_px':fit_weighted_error,
                             'weighted_error_px':weighted_error,'depth_weights':depth_weights.tolist(),'valid':True,
                             'side_observable':side_observable,'side_held':side_held,
                             'reseeded':reseeded,'jump_held':jump_held,'input_motion_px':image_motion,
                             'iterations':int(fit.nfev)}
        # Thumb: metric ray/surface points, with independent tip availability.
        thumb_ok=True
        measured={}
        for i in (1,2,3,4):
            o=obs[i] if i<len(obs) else {}
            if o.get('valid') and hand.get('sync_valid'):
                z=o['z_m']+.006
                if abs(z-flat[i,2])<.12:
                    clean[i]=rays[i]*z;measured[i]=True
                elif i!=4:thumb_ok=False
            elif self.last_geometry is not None:
                # RGB ray with previous relative depth for a short dropout.
                z=flat[0,2]+self.last_geometry[i,2]-self.last_geometry[0,2]
                clean[i]=rays[i]*z
                if i!=4:thumb_ok=False
            elif i!=4:thumb_ok=False
        thumb_lengths=np.linalg.norm(np.diff(clean[1:4],axis=0),axis=1)
        ref=None if structure is None else np.asarray(structure['thumb_lengths_m'])[:2]
        if np.any(thumb_lengths<.008) or (ref is not None and np.any(abs(thumb_lengths-ref)>np.maximum(.012,ref*.4))):thumb_ok=False
        self.last_geometry=clean.copy()
        self.report={'active':True,'front_ok':bool(abs(basis[2,2])>.65),
                     'message':'固定骨长关节链 · 图像主导 / 深度修正 · 拇指直接角度',
                     'fingers':details,'palm_depth_points':self.plane_count,'thumb_reliable':thumb_ok}
        span=float(np.linalg.norm(clean[4]-clean[1]))
        maximum=.15 if structure is None else sum(structure['thumb_lengths_m'])*1.35
        self.report['thumb_tip_reliable']=bool(measured.get(1) and measured.get(4) and .012<span<maximum)
        self.report['palm_source']=self.plane_source
        self.report['finger_solver']='rgb_model_3d' if self.vision_prior else 'depth_articulated'
        if self.vision_prior:self.report['message']='四指 RGB 三维骨架直接角度；深度掌面与拇指静态标定'
        if self.vision_prior and self.depth_root_correction:
            self.report['finger_solver']='rgb_depth_root_correction'
            self.report['root_correction']=self.root_report
            self.report['message']='RGB 骨架为主；深度弱修正掌面和根部屈伸／侧摆'
        return angles,clean,basis,thumb_ok
