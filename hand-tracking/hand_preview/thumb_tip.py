"""D435 fingertip depth -> three root angles, with fixed distal joints.

Closed-form geometry for the installed Apex root axes/offsets. No numerical
optimiser, candidate posture library or fingertip/contact attraction iteration.
"""
import math
import numpy as np
from .depth_articulation import DirectThumb


def wrap(value):return float(math.atan2(math.sin(value),math.cos(value)))


class TipDepthThumb(DirectThumb):
    def __init__(self,robot,bounds,config):
        super().__init__(bounds,config);self.robot=robot;self.tip_history=[]
        root=robot.zero_joints['left_thumb_j0'];second=robot.zero_joints['left_thumb_j2']
        self.origin=root[:3,3];self.rotation=root[:3,:3]
        offset=self.rotation.T@(second[:3,3]-self.origin)
        expected=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
        if (np.linalg.norm(offset[1:])>1e-4 or not .01<offset[0]<.05
                or not np.allclose(self.rotation.T@second[:3,:3],expected,atol=1e-4)):
            raise ValueError('当前拇指根部结构不支持解析位置换算')
        self.offset=float(offset[0])

    def reset(self):super().reset();self.tip_history=[]

    def solve_position(self,target,fixed,previous=None):
        """Canonical palm-space target relative to robot thumb j0 origin."""
        target=np.asarray(target,float)
        if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('无效拇指空间目标')
        actual=self.robot.thumb_points([0.,0.,0.,fixed])[-1]
        frame=self.robot.zero_joints['left_thumb_j2']
        v=frame[:3,:3].T@(actual-frame[:3,3])
        radius=float(np.linalg.norm(v[:2]));c=self.offset
        local=self.rotation.T@(self.robot.basis@target)
        v_component=float((c*c+radius*radius-local@local)/(2*c))
        unreachable=abs(v_component)>radius
        v_component=float(np.clip(v_component,-radius+1e-7,radius-1e-7))
        transverse=math.sqrt(max(1e-12,radius*radius-v_component*v_component))
        baseline=np.radians(self.config['neutral_deg']) if previous is None else np.asarray(previous)
        candidates=[]
        for signed in (-transverse,transverse):
            ratio=float(local[2]/signed)
            for sign in (-1.,1.):
                j1=sign*math.acos(float(np.clip(ratio,-1,1)))
                j2=wrap(math.atan2(v_component,signed)-math.atan2(v[1],v[0]))
                j0=wrap(math.atan2(local[1],local[0])-math.atan2(math.sin(j1)*signed,c-v_component))
                q=np.array([j0,j1,j2])
                clipped=np.array([np.clip(q[i],*self.bounds[f'left_thumb_j{i}']) for i in range(3)])
                point=(self.robot.thumb_points([*clipped,fixed])[-1]-self.origin)@self.robot.basis
                error=float(np.linalg.norm(point-target))
                cost=error+.001*float(np.linalg.norm(clipped-baseline))
                candidates.append((cost,clipped,error,bool(unreachable or abs(ratio)>1 or np.max(abs(clipped-q))>1e-5)))
        _,q,error,projected=min(candidates,key=lambda item:item[0])
        return q,{'target_error_mm':error*1000,'workspace_projected':projected}

    def calculate_tip(self,points,basis,structure,now,static_model=None):
        p=np.asarray(points,float);human=(p[4]-p[1])@basis
        if structure is None:raise ValueError('指尖位置映射需要两步骨架标定')
        neutral=np.asarray(structure['neutral_points_m'])
        reference=neutral[4]-neutral[1]
        fixed=float(np.clip(math.radians(self.config['fixed_terminal_deg']),*self.bounds['left_thumb_j3']))
        neutral_q=[*np.radians(self.config['neutral_deg']),fixed]
        neutral_points=self.robot.thumb_points(neutral_q)
        neutral_target=(neutral_points[-1]-self.origin)@self.robot.basis
        scale=float(np.sum(self.robot.thumb_lengths)/sum(structure['thumb_lengths_m']))
        target=neutral_target+scale*(human-reference)
        static_weight=0.
        if static_model is not None:
            from .thumb_static import calibrated_target
            target,static_weight=calibrated_target(static_model,human,target)
        q,diagnostics=self.solve_position(target,fixed,self.previous)
        if self.at is None or not 0<now-self.at<.25:self.tip_history=[]
        self.tip_history=(self.tip_history+[q.copy()])[-3:]
        if len(self.tip_history)==3:q=np.median(self.tip_history,axis=0)
        q=self.smooth(q,now)
        self.diagnostics={'method':'depth_tip_analytic','tip_camera_m':p[4].tolist(),
            'target_palm_m':target.tolist(),'human_tip_relative_m':human.tolist(),
            'fixed_terminal_deg':math.degrees(fixed),'held':False,'static_weight':static_weight,**diagnostics}
        return {**{f'j{i}':float(q[i]) for i in range(3)},'j3':fixed}
