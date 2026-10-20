from copy import deepcopy
import json
import tempfile
from pathlib import Path
import unittest

import numpy as np

from hand_preview.calibration import (CalibrationWizard,POSES,TIP_IDS,build_profile,
    calibration_cues,correct_fingers,finger_targets,similarity_fit,validate_profile)
from hand_preview.geometry import FINGERS,finger_angles
from hand_preview.kinematics import RobotKinematics,ThumbRetargeter,rotation
from hand_preview.mapping import PreviewMapper
from test_mapping import make_hand,pose


CONTEXT={'camera':0,'backend':'auto','swap_handedness':False}


def synthetic_samples(robot,mapper):
    neutral=pose(3,4)
    for finger in FINGERS:neutral[finger]['abd']=.03
    samples={}
    base=robot.thumb_points(np.zeros(4))[0]
    neutral_points=robot.thumb_points([.45,.2,.3,.12])
    for name in POSES:
        angles=deepcopy(neutral)
        if name!='neutral':angles[name].update(mcp=.7,pip=.6)
        offsets={'finger_offsets':{f:{k:neutral[f][k] for k in ('abd','mcp','pip')} for f in FINGERS}}
        if name=='neutral':target=(neutral_points[-1]-base)@robot.basis
        else:
            positions=finger_targets(correct_fingers(angles,offsets),mapper.bounds)
            links,_=robot.forward(positions)
            target=(links[f'left_{name}_tip'][:3,3]-base)@robot.basis
        canonical=np.zeros((21,3));canonical[4]=target/.08
        if name!='neutral':canonical[TIP_IDS[name]]=canonical[4]
        samples[name]={'angles':angles,'canonical':canonical.tolist(),'distance':.02,
                       'valid_frames':40,'spread':.01}
    return samples


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot=RobotKinematics();cls.mapper=PreviewMapper()

    def test_similarity_recovers_known_rotation_scale_translation(self):
        x=np.array([[1,0,0],[0,1,0],[0,0,1],[.4,.7,.1],[-.5,.2,.4]])
        r=rotation([1,2,3],.4);y=.08*x@r+[.01,-.02,.03]
        fit=similarity_fit(x,y)
        np.testing.assert_allclose(fit['rotation'],r,atol=1e-8)
        self.assertAlmostEqual(fit['scale'],.08)
        self.assertLess(fit['rms_mm'],1e-8)

    def test_repeated_pose_points_are_rejected(self):
        with self.assertRaises(ValueError):similarity_fit(np.ones((5,3)),np.ones((5,3)))

    def test_profile_offsets_and_save_load_are_transactional(self):
        samples=synthetic_samples(self.robot,self.mapper)
        profile=build_profile(samples,self.robot,self.mapper.bounds,CONTEXT)
        adjusted=correct_fingers(samples['neutral']['angles'],profile)
        for finger in FINGERS:
            self.assertAlmostEqual(adjusted[finger]['abd'],0)
            for key in ('mcp','pip'):
                self.assertEqual(adjusted[finger][key],samples['neutral']['angles'][finger][key])
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'profile.json'
            wizard=CalibrationWizard(self.robot,self.mapper.bounds,CONTEXT,path)
            with self.assertRaises(ValueError):wizard.apply()
            self.assertFalse(path.exists())
            wizard.pending=profile;wizard.apply()
            loaded=CalibrationWizard(self.robot,self.mapper.bounds,CONTEXT,path)
            self.assertTrue(loaded.enabled)
            loaded.reset()
            self.assertFalse(path.exists())
            self.assertEqual(len(list(Path(temporary).glob('reset-backup-*.json'))),1)

    def test_large_neutral_flexion_does_not_reduce_bend_or_create_dead_zone(self):
        angles=pose(45,24)
        for finger in FINGERS:angles[finger]['abd']=0.0
        profile={'finger_offsets':{f:{'abd':.2,'mcp':np.deg2rad(30),'pip':np.deg2rad(26)}
                                   for f in FINGERS}}
        adjusted=correct_fingers(angles,profile)
        for finger in FINGERS:
            for key in ('mcp','pip'):
                self.assertEqual(adjusted[finger][key],angles[finger][key])
            self.assertAlmostEqual(adjusted[finger]['abd'],angles[finger]['abd']-.2)
        angles['index']['abd']=None;angles['index']['pip']=None
        adjusted=correct_fingers(angles,profile)
        self.assertIsNone(adjusted['index']['abd']);self.assertIsNone(adjusted['index']['pip'])
        self.assertEqual(angles['middle']['abd'],0)

    def test_legacy_profile_rebuilds_contact_alignment_without_overwriting_file(self):
        profile=build_profile(synthetic_samples(self.robot,self.mapper),self.robot,self.mapper.bounds,CONTEXT)
        profile['schema']=1;profile.pop('finger_policy')
        profile['alignment']['translation'][0]+=.01
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'profile.json'
            original=json.dumps(profile);path.write_text(original)
            wizard=CalibrationWizard(self.robot,self.mapper.bounds,CONTEXT,path)
            self.assertTrue(wizard.enabled)
            self.assertEqual(wizard.profile['schema'],2)
            self.assertEqual(wizard.profile['finger_policy'],'abduction_only')
            self.assertEqual(wizard.profile['samples'],profile['samples'])
            self.assertNotEqual(wizard.profile['alignment'],profile['alignment'])
            self.assertEqual(path.read_text(),original)

    def test_context_and_bad_transform_not_loaded(self):
        profile=build_profile(synthetic_samples(self.robot,self.mapper),self.robot,self.mapper.bounds,CONTEXT)
        with self.assertRaises(ValueError):validate_profile(profile,{**CONTEXT,'camera':1})
        profile['alignment']['rotation']=[[2,0,0],[0,1,0],[0,0,1]]
        with self.assertRaises(ValueError):validate_profile(profile,CONTEXT)

    def test_capture_requires_frames_and_cancel_keeps_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            wizard=CalibrationWizard(self.robot,self.mapper.bounds,CONTEXT,Path(temporary)/'p.json')
            with self.assertRaises(ValueError):wizard.start('index',0)
            wizard.start('neutral',0);wizard.advance(4.1)
            self.assertNotIn('neutral',wizard.samples)
            points=make_hand();angles=finger_angles(points)
            wizard.start('neutral',4)
            for t in np.arange(5.05,6.96,.05):wizard.ingest(angles,points,'Left',float(t))
            wizard.advance(8.1)
            self.assertIn('neutral',wizard.samples)
            wizard.start('index',9);wizard.cancel()
            self.assertIn('neutral',wizard.samples)
            self.assertIsNone(wizard.collecting)

    def test_contact_cues_use_current_robot_fingertip(self):
        profile=build_profile(synthetic_samples(self.robot,self.mapper),self.robot,self.mapper.bounds,CONTEXT)
        world=make_hand();world[4]=world[8]
        positions={n:.1 for n in self.mapper.bounds}
        cues=calibration_cues(profile,world,'Left',self.robot,positions,list(FINGERS))
        self.assertEqual(cues['active'],'index');self.assertAlmostEqual(cues['strength'],1)
        links,_=self.robot.forward(positions);base=self.robot.thumb_points(np.zeros(4))[0]
        expected=(links['left_index_tip'][:3,3]-base)@self.robot.basis
        np.testing.assert_allclose(cues['tip'],expected,atol=1e-9)
        # Invalid selected finger cannot generate a contact constraint.
        cues=calibration_cues(profile,world,'Left',self.robot,positions,[])
        self.assertIsNone(cues['active']);self.assertEqual(cues['strength'],0)

    def test_tip_constraint_reduces_known_goal_error(self):
        target_points=self.robot.thumb_points([.2,.15,.2,.12])
        skeleton=(target_points-target_points[0])@self.robot.basis
        desired_points=self.robot.thumb_points([.7,.4,.6,.5])
        desired=(desired_points[-1]-desired_points[0])@self.robot.basis
        baseline=ThumbRetargeter(self.robot,self.mapper.specs)
        q=baseline.fit_target(skeleton)
        output=self.robot.thumb_points([q[f'j{i}'] for i in range(4)])
        before=np.linalg.norm((output[-1]-output[0])@self.robot.basis-desired)
        calibrated=ThumbRetargeter(self.robot,self.mapper.specs)
        calibrated.fit_target(skeleton,cues={'tip':desired,'strength':1})
        self.assertLess(calibrated.tip_error_mm,before*1000*.2)
        self.assertLess(calibrated.tip_error_mm,3)


if __name__=='__main__':unittest.main()
