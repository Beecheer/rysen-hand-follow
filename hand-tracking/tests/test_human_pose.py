from copy import deepcopy
import math
from pathlib import Path
import tempfile
import json
import unittest
import numpy as np
from hand_preview.geometry import FINGERS
from hand_preview.human_pose import FrontHandEstimator,MotionCheck,finger_fk,robust_frame
from hand_preview.calibration import build_profile,validate_profile,CalibrationWizard,FRONT_POSES
from hand_preview.kinematics import RobotKinematics,rotation
from hand_preview.mapping import PreviewMapper
from test_mapping import make_hand
from test_calibration import synthetic_samples,CONTEXT


def known_hand(q):
    p=make_hand();_,basis,length=robust_frame(p,'Left')
    lengths=np.array([.03,.02,.015])
    for root,pip,dip,tip in FINGERS.values():
        positions,_=finger_fk(q,lengths)
        p[[pip,dip,tip]]=p[root]+positions@basis.T
    return p


class HumanPoseTests(unittest.TestCase):
    def test_analytic_finger_jacobian(self):
        q=np.array([.2,.6,.5,.3]);lengths=np.array([.4,.3,.2]);points,jac=finger_fk(q,lengths)
        for i in range(4):
            changed=q.copy();changed[i]+=1e-6
            numerical=(finger_fk(changed,lengths)[0]-points).ravel()/1e-6
            np.testing.assert_allclose(jac[:,i],numerical,atol=1e-6)

    def test_known_angles_and_independent_human_dip(self):
        q=[.1,.45,.7,.2];p=known_hand(q);est=FrontHandEstimator()
        angles,clean,valid,obs=est.estimate(p,p[:,:2]*1000,'Left',0)
        self.assertTrue(valid)
        for f in FINGERS:
            for key,value in zip(('abd','mcp','pip','dip'),q):self.assertAlmostEqual(angles[f][key],value,delta=.025)
            self.assertNotAlmostEqual(angles[f]['pip'],angles[f]['dip'])
        self.assertEqual(len(obs['bone_lengths']['thumb']),3)

    def test_fixed_lengths_are_reconstructed(self):
        p=known_hand([.1,.3,.5,.2]);est=FrontHandEstimator()
        a,_,_,obs=est.estimate(p,p[:,:2]*1000,'Left',0)
        structure={'palm_template':np.array(obs['canonical'])[[0,5,9,13,17]].tolist(),
                   'bone_lengths':obs['bone_lengths'],'image_aspect':obs['image_aspect'],'camera_normal':obs['camera_normal']}
        noisy=p.copy();noisy[7]+=[.001,0,.001]
        a,clean,valid,_=est.estimate(noisy,noisy[:,:2]*1000,'Left',.03,structure)
        self.assertIsNotNone(a['index']['pip'])
        _,_,length=robust_frame(noisy,'Left')
        np.testing.assert_allclose(np.linalg.norm(np.diff(clean[[5,6,7,8]],axis=0),axis=1)/length,structure['bone_lengths']['index'],atol=1e-8)

    def test_left_right_reflection_keeps_anatomical_angles(self):
        p=known_hand([.1,.4,.6,.2]);mirrored=p*np.array([-1,1,1])
        left=FrontHandEstimator().estimate(p,p[:,:2]*1000,'Left',0)[0]
        right=FrontHandEstimator().estimate(mirrored,mirrored[:,:2]*1000,'Right',0)[0]
        for f in FINGERS:
            for k in ('abd','mcp','pip','dip'):self.assertAlmostEqual(left[f][k],right[f][k],places=7)

    def test_front_view_image_side_resists_world_landmark_error(self):
        p=known_hand([.1,.3,.4,.2]);image=p[:,:2]*1000
        bad=p.copy();bad[6,0]+=.006
        a=FrontHandEstimator().estimate(bad,image,'Left',0)[0]['index']['abd']
        _,basis,_=robust_frame(bad,'Left');bone=(bad[6]-bad[5])@basis
        raw=math.atan2(bone[0],bone[1])
        self.assertLess(abs(a-.1),abs(raw-.1))

    def test_turning_palm_holds_side_and_thumb_not_all_flexion(self):
        p=known_hand([.1,.3,.4,.2]);est=FrontHandEstimator();_,_,_,obs=est.estimate(p,p[:,:2]*1000,'Left',0)
        structure={'palm_template':np.array(obs['canonical'])[[0,5,9,13,17]].tolist(),'bone_lengths':obs['bone_lengths'],
                   'image_aspect':obs['image_aspect'],'camera_normal':obs['camera_normal']}
        moved=p@rotation([0,1,0],math.radians(65)).T
        a,_,thumb,_=est.estimate(moved,moved[:,:2]*1000,'Left',1,structure)
        self.assertFalse(thumb);self.assertFalse(est.report['front_ok'])
        self.assertIsNone(a['index']['abd']);self.assertIsNotNone(a['index']['mcp'])

    def test_image_world_yaw_disagreement_does_not_reject_capture(self):
        p=known_hand([-.35,.2,.4,.3]);image=p[:,:2]*1000
        reference=FrontHandEstimator().estimate(p,image,'Left',0)[0]
        # Simulate image/world yaw disagreement without changing bone bend.
        c,s=math.cos(.35),math.sin(.35)
        for root,*tips in FINGERS.values():
            image[tips]=image[root]+(image[tips]-image[root])@np.array([[c,-s],[s,c]])
        estimator=FrontHandEstimator()
        angles,_,_,obs=estimator.estimate(p,image,'Left',0)
        for f in FINGERS:
            self.assertTrue(estimator.report['fingers'][f]['valid'])
            self.assertAlmostEqual(angles[f]['abd'],0,delta=.03)
            self.assertAlmostEqual(angles[f]['mcp'],reference[f]['mcp'],delta=.03)
            self.assertAlmostEqual(angles[f]['pip'],reference[f]['pip'],delta=.03)
        with tempfile.TemporaryDirectory() as d:
            robot=RobotKinematics();mapper=PreviewMapper()
            wizard=CalibrationWizard(robot,mapper.bounds,CONTEXT,Path(d)/'profile.json')
            wizard.start('neutral',0)
            for dt in np.arange(1.05,3.96,.1):wizard.ingest(angles,p,'Left',float(dt),obs)
            wizard.advance(4.1)
            self.assertIn('neutral',wizard.samples,wizard.message)

    def test_structure_profiles_preserve_old_file_until_apply(self):
        robot=RobotKinematics();mapper=PreviewMapper();legacy=build_profile(synthetic_samples(robot,mapper),robot,mapper.bounds,CONTEXT)
        samples=deepcopy(legacy['samples'])
        p=known_hand([0,.03,.04,.02]);_,_,_,obs=FrontHandEstimator().estimate(p,p[:,:2]*1000,'Left',0)
        samples['neutral']['structure_observation']=obs
        for name in ('spread','hook','mcp'):samples[name]=deepcopy(samples['neutral'])
        for f in FINGERS:
            samples['spread']['angles'][f]['abd']+=.2
            samples['hook']['angles'][f]['pip']+=.5
            samples['mcp']['angles'][f]['mcp']+=.4
        profile=build_profile(samples,robot,mapper.bounds,CONTEXT)
        self.assertEqual(profile['schema'],3);validate_profile(profile,CONTEXT)
        broken=deepcopy(profile);broken['structure']['bone_lengths']['thumb'][0]=float('nan')
        with self.assertRaises(ValueError):validate_profile(broken,CONTEXT)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'profile.json';original=json.dumps(legacy);path.write_text(original)
            wizard=CalibrationWizard(robot,mapper.bounds,CONTEXT,path)
            self.assertEqual(path.read_text(),original);self.assertEqual(wizard.profile['schema'],2)
            wizard.pending=profile;wizard.apply()
            self.assertEqual(len(list(Path(d).glob('backup-*.json'))),1)
            loaded=CalibrationWizard(robot,mapper.bounds,CONTEXT,path)
            self.assertEqual(loaded.profile['schema'],3)

    def test_motion_check_reports_consistency_not_accuracy(self):
        check=MotionCheck();check.start('open_close',0)
        angles={f:{'abd':0,'mcp':0,'pip':0} for f in FINGERS}
        check.ingest(angles,True,1);angles=deepcopy(angles);angles['index']['mcp']=1
        check.ingest(angles,True,2);check.ingest(None,False,3)
        result=check.snapshot(10.1)['result']
        self.assertEqual(result['valid_frames'],2);self.assertAlmostEqual(result['valid_ratio'],2/3)
        self.assertGreater(result['range_deg']['index']['mcp'],40)

    def test_all_eight_poses_capture_through_wizard(self):
        robot=RobotKinematics();mapper=PreviewMapper()
        with tempfile.TemporaryDirectory() as d:
            wizard=CalibrationWizard(robot,mapper.bounds,CONTEXT,Path(d)/'profile.json')
            for i,name in enumerate(FRONT_POSES):
                q=[0,.03,.04,.02]
                if name=='spread':q[0]=.2
                elif name=='hook':q[2:]=[.8,.5]
                elif name=='mcp':q[1]=.65
                elif name not in ('neutral','spread','hook','mcp'):q[1:]=[.5,.65,.3]
                p=known_hand(q)
                if name in FINGERS:p[4]=p[FINGERS[name][-1]]
                estimator=FrontHandEstimator()
                angles,_,_,obs=estimator.estimate(p,p[:,:2]*1000,'Left',0)
                start=i*5;wizard.start(name,start)
                for dt in np.arange(1.05,3.96,.1):wizard.ingest(angles,p,'Left',start+float(dt),obs)
                wizard.advance(start+4.1)
                self.assertIn(name,wizard.samples,wizard.message)
            self.assertIsNotNone(wizard.pending,wizard.message)
            self.assertEqual(wizard.pending['schema'],3)
