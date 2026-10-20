"""Per-joint depth bands + RGB shape and bone constraints; no robot mapping.

Depth observes a visible surface, not a joint center. Accepted measurements
therefore constrain a depth interval rather than replacing a joint coordinate.
"""
from collections import deque
import hashlib
import json
import time
import numpy as np
from scipy.optimize import least_squares

CHAINS={'thumb':(0,1,2,3,4),'index':(0,5,6,7,8),'middle':(0,9,10,11,12),
        'ring':(0,13,14,15,16),'pinky':(0,17,18,19,20)}
EDGES=[(a,b) for chain in CHAINS.values() for a,b in zip(chain,chain[1:])]
PARENTS=np.array([e[0] for e in EDGES]);CHILDREN=np.array([e[1] for e in EDGES])
PALM=np.array([0,5,9,13,17])


def unit(v):
    length=np.linalg.norm(v)
    if length<1e-8:raise ValueError('退化骨段')
    return v/length


def depth_patch(depth,scale,pixel,expected=None,gate_m=.07):
    h,w=depth.shape;x,y=np.rint(pixel).astype(int)
    if x<2 or y<2 or x>=w-2 or y>=h-2:return {'valid':False,'reason':'图像边缘'}
    patch=depth[y-2:y+3,x-2:x+3].astype(float)*scale
    values=patch[(patch>=.12)&(patch<=2.)]
    if expected is not None:values=values[np.abs(values-expected)<gate_m]
    if len(values)<9:return {'valid':False,'reason':'深度缺失或偏离手部'}
    center=float(np.median(values));mad=float(np.median(np.abs(values-center)))
    # Reject mixed surfaces instead of filling a hole with background depth.
    if np.ptp(np.percentile(values,[10,90]))>.025 or mad>.006:
        return {'valid':False,'reason':'轮廓或混合表面'}
    return {'valid':True,'z_m':center,'sigma_m':max(.003,1.4826*mad),
            'valid_ratio':len(values)/25,'reason':'邻域表面深度'}


def angles(points):
    p=np.asarray(points)
    forward=unit(p[9]-p[0]);across=unit(p[5]-p[17]);across=unit(across-forward*(across@forward))
    normal=unit(np.cross(across,forward))
    if normal[2]>0:normal=-normal
    def bend(a,b):return float(np.degrees(np.arccos(np.clip(unit(a)@unit(b),-1,1))))
    result={}
    for name,ids in CHAINS.items():
        a,b,c,d=ids[1:];first=p[b]-p[a]
        if name=='thumb':
            result[name]={'plane':float(np.degrees(np.arctan2(first@across,first@forward))),
                          'lift':float(np.degrees(np.arctan2(first@normal,np.hypot(first@across,first@forward)))),
                          'mcp':bend(first,p[c]-p[b]),'ip':bend(p[c]-p[b],p[d]-p[c])}
        else:
            result[name]={'abd':float(np.degrees(np.arctan2(first@across,first@forward))),
                          'mcp':float(np.degrees(np.arctan2(first@normal,np.linalg.norm(first-normal*(first@normal))))),
                          'pip':bend(first,p[c]-p[b]),'dip':bend(p[c]-p[b],p[d]-p[c])}
    return result


def solve_depths(rays,prior,lengths,observations,previous=None,previous_prior=None):
    """21 unknown Z coordinates. XY stay on the detected RGB projection rays."""
    prior_z=prior[:,2]
    valid=np.array([o['valid'] for o in observations])
    surface=np.array([o.get('z_m',0) for o in observations])
    sigma=np.array([o.get('sigma_m',.005) for o in observations])
    band=np.full(21,.012);band[PALM]=.018;band[[4,8,12,16,20]]=.009
    indices=np.arange(1,21)
    rel_jac=np.zeros((20,21));rel_jac[np.arange(20),indices]=1;rel_jac[:,0]=-1
    prior_sigma=.012 if valid.sum()>=5 else .006
    initial=prior_z.copy();temporal_relative=None;temporal_sigma=.006
    if previous is not None:
        temporal_relative=previous[1:,2]-previous[0,2]
        motion=np.zeros(20)
        if previous_prior is not None:
            motion=(prior_z[1:]-prior_z[0])-(previous_prior[1:,2]-previous_prior[0,2])
            temporal_relative=temporal_relative+motion
        temporal_sigma=.004+min(.012,float(np.percentile(abs(motion),90))*.8)
        initial[0]=prior_z[0]
        initial[1:]=initial[0]+temporal_relative
    def residual_jac(z):
        points=rays*z[:,None];delta=points[CHILDREN]-points[PARENTS]
        distance=np.linalg.norm(delta,axis=1);direction=delta/np.maximum(distance[:,None],1e-9)
        jb=np.zeros((20,21));jb[np.arange(20),CHILDREN]=np.sum(direction*rays[CHILDREN],axis=1)/.004
        jb[np.arange(20),PARENTS]=-np.sum(direction*rays[PARENTS],axis=1)/.004
        relative=z[1:]-z[0]-(prior_z[1:]-prior_z[0])
        below=z<surface-.003;above=z>surface+band
        active=valid&(below|above)
        depth_error=np.where(valid,np.where(below,z-surface+.003,np.where(above,z-surface-band,0)),0)/sigma
        pieces=[(distance-lengths)/.004,relative/prior_sigma,depth_error,np.array([(z[0]-prior_z[0])/.01])]
        jac=[jb,rel_jac/prior_sigma,np.diag(active/sigma),np.eye(21)[0:1]/.01]
        if previous is not None:
            pieces.append((z[1:]-z[0]-temporal_relative)/temporal_sigma)
            jac.append(rel_jac/temporal_sigma)
        return np.concatenate(pieces),np.vstack(jac)
    def loss(squared):
        root=np.sqrt(1+squared)
        rho=np.vstack((2*(root-1),1/root,-.5/root**3))
        if previous is not None:
            # A branch jump is not an outlier to be downweighted. Keep the
            # motion-relative temporal prior quadratic even for large errors.
            rho[0,62:]=squared[62:];rho[1,62:]=1.;rho[2,62:]=0.
        return rho
    fit=least_squares(lambda z:residual_jac(z)[0],np.clip(initial,.121,1.999),
                      jac=lambda z:residual_jac(z)[1],bounds=(.12,2.),loss=loss,f_scale=1.,max_nfev=20)
    points=rays*fit.x[:,None]
    bone_error=np.abs(np.linalg.norm(points[CHILDREN]-points[PARENTS],axis=1)-lengths)
    return points,{'bone_error_p95_mm':float(np.percentile(bone_error,95)*1000),
                   'bone_error_mm':(bone_error*1000).tolist(),
                   'depth_constraints':int(valid.sum()),'iterations':fit.nfev,
                   'surface_band_residual_p95_mm':float(np.percentile(np.maximum(surface-.003-fit.x,fit.x-surface-band)[valid].clip(0),95)*1000) if valid.any() else None}


class DepthHandFusion:
    def __init__(self):self.reference=None;self.observe_only=False;self.reset()
    def set_reference(self,reference):
        if reference is not None:
            lengths=np.asarray(reference['lengths_m'],float)
            if lengths.shape!=(20,) or not np.isfinite(lengths).all() or np.any(lengths<.005) or np.any(lengths>.13):
                raise ValueError('D435 米制骨长参考无效')
        self.reference=reference;self.reset()
    def reset(self):
        self.lengths=None;self.length_samples=deque(maxlen=30);self.previous=None;self.last_at=None
        self.last_pixels=None;self.last_label=None;self.scale=None
        self.last_depth_at=None
        self.previous_prior=None

    def estimate(self,pixels,world,depth,scale,intr,now,label,skew_ms):
        pixels=np.asarray(pixels,float);world=np.asarray(world,float)
        if pixels.shape!=(21,2) or world.shape!=(21,3) or not np.isfinite(pixels).all() or not np.isfinite(world).all():
            raise ValueError('无效手部关键点')
        h,w=depth.shape
        if (pixels[:,0]<2).any() or (pixels[:,0]>=w-2).any() or (pixels[:,1]<2).any() or (pixels[:,1]>=h-2).any():
            raise ValueError('请将整只手和指尖保持在画面内')
        if self.last_at is not None and (now-self.last_at>.5 or label!=self.last_label or
                np.linalg.norm(pixels[0]-self.last_pixels[0])>w*.25):self.reset()
        if any(abs(c)>1e-10 for c in getattr(intr,'coeffs',[])):
            import pyrealsense2 as rs
            rays=np.array([rs.rs2_deproject_pixel_to_point(intr,p.tolist(),1.) for p in pixels])
        else:
            rays=np.column_stack(((pixels[:,0]-intr.ppx)/intr.fx,(pixels[:,1]-intr.ppy)/intr.fy,np.ones(21)))
        palm=[depth_patch(depth,scale,pixels[i]) for i in PALM]
        good=[(i,o) for i,o in zip(PALM,palm) if o['valid']]
        sync_ok=skew_ms is not None and skew_ms<=25
        recent=self.last_at is not None and now-self.last_at<.25 and self.previous is not None
        if len(good)<3 or not sync_ok:
            if not recent or self.last_depth_at is None or now-self.last_depth_at>.25:
                raise ValueError('等待可靠掌部深度' if sync_ok else '彩色/深度时间差过大')
            wrist_z=float(self.previous[0,2]);factor=self.scale
        else:
            self.last_depth_at=now
            z=np.median([o['z_m'] for _,o in good])
            lengths=[]
            for a,b in ((5,17),(0,9),(5,9),(9,17)):
                den=np.linalg.norm(world[a]-world[b])
                if den>1e-4:lengths.append(np.linalg.norm((rays[a]-rays[b])*z)/den)
            factor=float(np.clip(np.median(lengths),.5,2.))
            if self.reference is not None and not self.observe_only:
                model=np.linalg.norm((world-world[0])[CHILDREN]-(world-world[0])[PARENTS],axis=1)
                if np.any(model<1e-6):raise ValueError('模型骨长退化')
                factor=float(np.clip(np.median(np.asarray(self.reference['lengths_m'])/model),.5,2.))
            if self.scale is not None:factor=.85*self.scale+.15*factor
            wrist_z=float(np.median([o['z_m']-(world[i,2]-world[0,2])*factor for i,o in good]))
        relative=(world-world[0])*factor
        prior=relative+rays[0]*wrist_z
        self.scale=factor
        if not self.observe_only and (np.any(prior[:,2]<.12) or np.any(prior[:,2]>2.)):raise ValueError('手部超出当前 0.12–2 米融合范围')
        observations=[depth_patch(depth,scale,pixels[i],wrist_z if self.observe_only else prior[i,2],
                                  gate_m=.12 if self.observe_only else .07) if sync_ok else
                      {'valid':False,'reason':'彩深时间差过大'} for i in range(21)]
        model_lengths=np.linalg.norm(relative[CHILDREN]-relative[PARENTS],axis=1)
        if not self.observe_only and (np.any(model_lengths<.005) or np.any(model_lengths>.13)):raise ValueError('手部骨段估计异常')
        if len(self.length_samples)<30 and sum(o['valid'] for o in observations)>=10:
            self.length_samples.append(model_lengths)
        if self.length_samples:self.lengths=np.median(self.length_samples,axis=0)
        lengths=np.asarray(self.reference['lengths_m']) if self.reference is not None else (self.lengths if self.lengths is not None else model_lengths)
        previous=self.previous if recent and np.mean(np.linalg.norm(pixels-self.last_pixels,axis=1))<10 else None
        if self.observe_only:
            points=prior.copy()
            quality={'bone_error_p95_mm':0.,'bone_error_mm':[0.]*20,
                     'depth_constraints':sum(o['valid'] for o in observations),'iterations':0,
                     'observation_only':True}
        else:
            points,quality=solve_depths(rays,prior,lengths,observations,previous,
                                      self.previous_prior if previous is not None else None)
        # Sample the palm interior, excluding projected distal finger segments.
        # The app robustly fits these measured surface points, not MP world Z.
        palm_samples=[]
        if self.observe_only and sync_ok:
            import cv2
            mask=np.zeros(depth.shape,np.uint8)
            region=pixels[PALM].copy()
            # The wrist-side palm remains visible much longer during flexion.
            # Avoid fitting the knuckles and curled phalanges as the palm.
            region[1:]=region[0]+.72*(region[1:]-region[0])
            cv2.fillConvexPoly(mask,cv2.convexHull(np.rint(region).astype(np.int32)),255)
            mask=cv2.erode(mask,np.ones((7,7),np.uint8))
            for chain in CHAINS.values():
                for a,b in zip(chain[2:],chain[3:]):
                    cv2.line(mask,tuple(np.rint(pixels[a]).astype(int)),tuple(np.rint(pixels[b]).astype(int)),0,13)
            yy,xx=np.nonzero(mask[::6,::6]);sample_pixels=np.column_stack((xx*6,yy*6))
            for pixel in sample_pixels[::max(1,len(sample_pixels)//80)]:
                o=depth_patch(depth,scale,pixel)
                if o['valid'] and abs(o['z_m']-wrist_z)<.06:
                    if any(abs(c)>1e-10 for c in getattr(intr,'coeffs',[])):
                        ray=np.asarray(rs.rs2_deproject_pixel_to_point(intr,pixel.astype(float).tolist(),1.))
                    else:ray=np.array([(pixel[0]-intr.ppx)/intr.fx,(pixel[1]-intr.ppy)/intr.fy,1.])
                    palm_samples.append((ray*o['z_m']).tolist())
        usable=quality['bone_error_p95_mm']<15 and quality['depth_constraints']>=5
        self.previous=points;self.previous_prior=prior.copy();self.last_at=now;self.last_pixels=pixels.copy();self.last_label=label
        sources=['depth_band' if o['valid'] else 'model_prior' for o in observations]
        return {'state':'FUSED' if usable else 'MODEL_ASSISTED','label':label,'pixels':pixels.tolist(),
                'rays':rays.tolist(),'focal_px':float((intr.fx+intr.fy)/2),
                'palm_surface_points_m':palm_samples,
                'model_points_m':prior.tolist(),'fused_points_m':points.tolist(),'sources':sources,
                'observations':observations,'quality':quality,'bone_reference_frames':len(self.length_samples),
                'personal_reference':self.reference is not None,
                'reference_id':hashlib.sha256(json.dumps(self.reference,sort_keys=True).encode()).hexdigest() if self.reference else None,
                'anchor_age_ms':(now-self.last_depth_at)*1000,
                'sync_valid':sync_ok,'reliable_palm_points':len(good),
                'model_angles_deg':angles(prior),'fused_angles_deg':angles(points),
                'units':'meters','note':'深度约束骨架估计，非关节中心实测值'}


class HandTracker:
    def __init__(self,model_path):
        import mediapipe as mp
        self.mp=mp;self.fusion=DepthHandFusion();self.last_timestamp=0
        options=mp.tasks.vision.HandLandmarkerOptions(base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,num_hands=1,min_hand_detection_confidence=.5,
            min_hand_presence_confidence=.5,min_tracking_confidence=.5)
        self.detector=mp.tasks.vision.HandLandmarker.create_from_options(options)

    def close(self):self.detector.close()
    def reset(self):self.fusion.reset()

    def process(self,bgr,depth,scale,intr,now,skew_ms,swap=False):
        import cv2
        begin=time.perf_counter();stamp=max(self.last_timestamp+1,int(now*1000));self.last_timestamp=stamp
        image=self.mp.Image(image_format=self.mp.ImageFormat.SRGB,data=np.ascontiguousarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)))
        result=self.detector.detect_for_video(image,stamp)
        report={'enabled':True,'state':'NO_HAND','message':'请把一只手完整放入 D435 画面，建议先保持掌心朝前',
                'hands':[],'processing_ms':(time.perf_counter()-begin)*1000}
        for image_points,world,category in zip(result.hand_landmarks,result.hand_world_landmarks,result.handedness):
            pixels=np.array([[p.x*bgr.shape[1],p.y*bgr.shape[0]] for p in image_points])
            label=category[0].category_name
            if swap:label='Right' if label=='Left' else 'Left'
            try:
                hand=self.fusion.estimate(pixels,[[p.x,p.y,p.z] for p in world],depth,scale,intr,now,label,skew_ms)
                report.update(state=hand['state'],message=('RGB 关键点与表面深度采集中；主页面执行关节链解算' if self.fusion.observe_only else '逐指深度约束运行中；黄色点使用模型估计'))
            except ValueError as exc:
                hand={'state':'WAITING_DEPTH','label':label,'pixels':pixels.tolist(),'sources':['model_prior']*21}
                report.update(state='WAITING_DEPTH',message=str(exc))
            hand['score']=float(category[0].score)
            report['hands'].append(hand)
        report['processing_ms']=(time.perf_counter()-begin)*1000
        return report


def draw_hands(image,report):
    import cv2
    for hand in report.get('hands',[]):
        p=np.rint(hand['pixels']).astype(int)
        for a,b in EDGES:cv2.line(image,tuple(p[a]),tuple(p[b]),(180,160,80),1,cv2.LINE_AA)
        for i,xy in enumerate(p):
            color=(120,230,80) if hand['sources'][i]=='depth_band' else (40,190,255)
            cv2.circle(image,tuple(xy),3,color,-1,cv2.LINE_AA)
        cv2.putText(image,hand['label']+' / '+hand['state'],tuple(p[0]+[0,20]),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
