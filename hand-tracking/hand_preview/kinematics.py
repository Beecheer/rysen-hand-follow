"""URDF forward kinematics and bounded thumb retargeting, preview only."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares

from .geometry import unit, palm_frame
from .mapping import ASSET_DIR


def rotation(axis, angle):
    x, y, z = unit(np.asarray(axis, dtype=float))
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)


def transform(xyz=(0,0,0), rpy=(0,0,0)):
    roll, pitch, yaw = rpy
    result=np.eye(4)
    result[:3,:3] = rotation([0,0,1],yaw) @ rotation([0,1,0],pitch) @ rotation([1,0,0],roll)
    result[:3,3]=xyz
    return result


class RobotKinematics:
    def __init__(self):
        root=ET.parse(ASSET_DIR/'apex_hand_left.urdf').getroot()
        self.joints={}
        self.child_joint={}
        for node in root.findall('joint'):
            origin=node.find('origin')
            xyz=[float(x) for x in origin.get('xyz','0 0 0').split()] if origin is not None else [0,0,0]
            rpy=[float(x) for x in origin.get('rpy','0 0 0').split()] if origin is not None else [0,0,0]
            axis=node.find('axis'); mimic=node.find('mimic')
            item={'name':node.get('name'),'parent':node.find('parent').get('link'),
                  'child':node.find('child').get('link'),'origin':transform(xyz,rpy),
                  'axis':[float(x) for x in axis.get('xyz').split()] if axis is not None else [0,0,1],
                  'moving':node.get('type')=='revolute',
                  'mimic':None if mimic is None else (mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0')))}
            self.joints[item['name']]=item;self.child_joint[item['child']]=item
        self.root_link=next(n.get('name') for n in root.findall('link') if n.get('name') not in self.child_joint)
        self.zero_links,self.zero_joints=self.forward({})
        forward=unit(self.zero_joints['left_middle_j2'][:3,3]-self.zero_joints['left_middle_j0'][:3,3])
        across=unit(self.zero_joints['left_index_j0'][:3,3]-self.zero_joints['left_pinky_j0'][:3,3])
        across=unit(across-forward*np.dot(across,forward))
        normal=unit(np.cross(across,forward))
        _, bent=self.forward({'left_middle_j1':0.1})
        delta=bent['left_middle_j2'][:3,3]-self.zero_joints['left_middle_j2'][:3,3]
        normal *= 1 if np.dot(delta,normal)>0 else -1
        self.basis=np.column_stack((across,forward,normal))
        self.abduction_signs={}
        for finger in ('index','middle','ring','pinky'):
            _, moved=self.forward({f'left_{finger}_j0':0.05})
            delta=moved[f'left_{finger}_j2'][:3,3]-self.zero_joints[f'left_{finger}_j2'][:3,3]
            self.abduction_signs[finger]=1 if np.dot(delta,across)>0 else -1
        self.thumb_chain=[]
        link='left_thumb_tip'
        while link in self.child_joint:
            j=self.child_joint[link];self.thumb_chain.append(j);link=j['parent']
            if j['name']=='left_thumb_j0':break
        self.thumb_chain.reverse()
        self.thumb_parent=self.zero_links[self.thumb_chain[0]['parent']]
        self.thumb_lengths=np.linalg.norm(np.diff(self.thumb_points(np.zeros(4)),axis=0),axis=1)

    @staticmethod
    def joint_transform(joint, positions):
        angle=positions.get(joint['name'],0.0)
        if joint['mimic']:
            parent,mult,offset=joint['mimic'];angle=positions.get(parent,0.0)*mult+offset
        result=joint['origin'].copy()
        if joint['moving']:result[:3,:3]=result[:3,:3]@rotation(joint['axis'],angle)
        return result

    def forward(self,positions):
        links={self.root_link:np.eye(4)};frames={};pending=list(self.joints.values())
        while pending:
            remaining=[]
            for joint in pending:
                if joint['parent'] not in links:remaining.append(joint);continue
                matrix=links[joint['parent']]@self.joint_transform(joint,positions)
                links[joint['child']]=matrix;frames[joint['name']]=matrix
            if len(remaining)==len(pending):raise ValueError('Invalid URDF joint tree')
            pending=remaining
        return links,frames

    def thumb_points(self,q):
        positions={f'left_thumb_j{i}':float(q[i]) for i in range(4)}
        current=self.thumb_parent.copy();points=[]
        for joint in self.thumb_chain:
            current=current@self.joint_transform(joint,positions)
            if joint['name'] in ('left_thumb_j0','left_thumb_j3','left_thumb_j4'):
                points.append(current[:3,3].copy())
            if joint['child']=='left_thumb_tip':points.append(current[:3,3].copy())
        return np.asarray(points)


class ThumbRetargeter:
    def __init__(self,robot,specs,stable_continuity=False):
        self.stable_continuity=stable_continuity
        self.previous_input=None
        self.robot=robot
        self.lower=np.array([specs[f'left_thumb_j{i}']['lower'] for i in range(4)])
        self.upper=np.array([specs[f'left_thumb_j{i}']['upper'] for i in range(4)])
        self.previous=np.array([0.6,0.25,0.2,0.15])
        self.fit_error=None
        self.tip_error_mm=None
        from .thumb_filter import ThumbObservationFilter
        self.observation_filter=ThumbObservationFilter()
        self.diagnostics={}

    @staticmethod
    def _align_axis(source,target):
        """Return a column-vector rotation mapping one unit axis onto another."""
        a=unit(np.asarray(source,float));b=unit(np.asarray(target,float))
        cross=np.cross(a,b);dot=float(np.clip(a@b,-1.,1.))
        if dot < -0.9999:
            axis=unit(np.cross(a,[1.,0.,0.]) if abs(a[0])<.8 else np.cross(a,[0.,1.,0.]))
            return 2*np.outer(axis,axis)-np.eye(3)
        k=np.array([[0.,-cross[2],cross[1]],[cross[2],0.,-cross[0]],[-cross[1],cross[0],0.]])
        return np.eye(3)+k+k@k/(1.+dot)

    def target_from_landmarks(self,world,handedness,direction_reference=None):
        p,basis,palm_length=palm_frame(world,handedness)
        vectors=np.diff(p[1:5],axis=0)
        lengths=np.linalg.norm(vectors,axis=1)
        if np.any(lengths/palm_length<0.035) or np.any(lengths/palm_length>1.4):
            raise ValueError('Unreliable thumb bones')
        # Match directions and robot segment lengths, not user hand size.
        directions=(vectors/lengths[:,None])@basis
        if direction_reference is not None:
            # Calibrate the operator's thumb-out direction to the selected
            # comfortable robot reference posture, then preserve live motion.
            rotation=self._align_axis(direction_reference['human_axis'],direction_reference['robot_axis'])
            directions=directions@rotation.T
        canonical=np.vstack((np.zeros(3),np.cumsum(directions*self.robot.thumb_lengths[:,None],axis=0)))
        return canonical

    def fit_target(self,target,temporal_weight=0.025,cues=None,posture=None):
        target=np.asarray(target,dtype=float)
        if target.shape!=(4,3) or not np.isfinite(target).all():raise ValueError('Invalid thumb target')
        scale=float(sum(self.robot.thumb_lengths))
        previous=self.previous.copy()
        lower,upper=self.lower,self.upper
        if self.stable_continuity:
            movement=1. if self.previous_input is None else float(np.max(np.linalg.norm(target-self.previous_input,axis=1))/scale)
            activity=float(np.clip((movement-.01)/.12,0,1))
            radius=math.radians(3+77*activity)
            if self.previous_input is not None:
                center=np.clip(previous,self.lower,self.upper)
                lower=np.maximum(self.lower,center-radius);upper=np.minimum(self.upper,center+radius)
            temporal_weight=max(temporal_weight,.65-.5*activity)
            self.diagnostics['ik_continuity']={'input_motion_palm':movement,'trust_radius_deg':math.degrees(radius)}
        strength=0 if cues is None else float(cues['strength'])
        if cues is not None and (np.asarray(cues['tip']).shape!=(3,) or not np.isfinite(cues['tip']).all()
                                 or not 0<=strength<=1):raise ValueError('Invalid calibrated thumb goal')
        if posture is not None and (np.asarray(posture['q']).shape!=(4,) or
                not np.isfinite(posture['q']).all() or not 0<=posture['weight']<=1):
            raise ValueError('Invalid thumb posture reference')
        def residual(q):
            points=self.robot.thumb_points(q)
            canonical=(points-points[0])@self.robot.basis
            position=((canonical[1:]-target[1:])/scale*np.array([[0.7],[0.8],[1.5]])).ravel()
            actual_direction=unit(canonical[-1]-canonical[-2])
            target_direction=unit(target[-1]-target[-2])
            terms=[position*(1 if cues is None else .45*(1-.7*strength)),
                   .25*(1-.6*strength)*(actual_direction-target_direction),temporal_weight*(q-previous)]
            if cues is not None:
                terms.append((3+5*strength)*(canonical[-1]-cues['tip'])/scale)
            if posture is not None:
                terms.append(posture['weight']*(1-.8*strength)*(q-posture['q']))
            return np.concatenate(terms)
        # TRF started almost exactly at four lower bounds can spend its entire
        # short frame budget expanding a tiny trust region instead of moving.
        margin=np.minimum(.02,(upper-lower)*.25)
        result=least_squares(residual,np.clip(previous,lower+margin,upper-margin),
                             bounds=(lower,upper),max_nfev=18,ftol=2e-3,xtol=2e-3,gtol=2e-3)
        points=self.robot.thumb_points(result.x)
        actual=(points-points[0])@self.robot.basis
        error=float(np.sqrt(np.mean(np.sum((actual[1:]-target[1:])**2,axis=1)))/scale)
        tip_error=0 if cues is None else float(np.linalg.norm(actual[-1]-cues['tip']))
        if error>0.2 or tip_error>.012:
            alternate=least_squares(residual,np.clip((self.lower+self.upper)*0.4,lower+margin,upper-margin),
                                    bounds=(lower,upper),max_nfev=18,
                                    ftol=2e-3,xtol=2e-3,gtol=2e-3)
            candidate=self.robot.thumb_points(alternate.x)
            candidate=(candidate-candidate[0])@self.robot.basis
            candidate_error=float(np.sqrt(np.mean(np.sum((candidate[1:]-target[1:])**2,axis=1)))/scale)
            cost=float(np.sum(residual(result.x)**2));alternative_cost=float(np.sum(residual(alternate.x)**2))
            nearby=np.max(np.abs(alternate.x-result.x))<math.radians(15)
            # Do not jump to a distant IK branch for a negligible cost gain.
            if alternative_cost<cost and (nearby or cost-alternative_cost>max(1e-4,.15*cost)):
                result,error,actual=alternate,candidate_error,candidate
        if not np.isfinite(result.x).all() or (cues is None and error>0.32):
            raise ValueError('Thumb target cannot be fitted reliably')
        self.tip_error_mm=None if cues is None else float(np.linalg.norm(actual[-1]-cues['tip'])*1000)
        self.previous=result.x
        self.previous_input=target.copy()
        self.fit_error=error
        return {f'j{i}':float(value) for i,value in enumerate(result.x)}

    def fit(self,world,handedness,cues=None,now=None,thumb_model=None,direction_reference=None):
        import time
        raw_target=self.target_from_landmarks(world,handedness,direction_reference)
        raw_cues=None if cues is None else {'tip':np.asarray(cues['tip']).tolist(),'strength':float(cues['strength'])}
        target,cues,activity=self.observation_filter.update(raw_target,cues,
            time.perf_counter() if now is None else now)
        self.diagnostics={'raw_target':raw_target.tolist(),'raw_cues':raw_cues,
                          'filtered_target':target.tolist(),'filtered_cues':None if cues is None else
                          {'tip':np.asarray(cues['tip']).tolist(),'strength':float(cues['strength'])},'activity':activity}
        posture=None
        if thumb_model is not None:
            from .thumb_calibration import posture_prior
            posture=posture_prior(thumb_model,target)
        self.diagnostics['posture_weight']=0 if posture is None else posture['weight']
        return self.fit_target(target,temporal_weight=.04+.08*(1-activity),cues=cues,posture=posture)
