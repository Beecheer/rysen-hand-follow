from pathlib import Path
import json
import tempfile
import unittest

import numpy as np

from hand_preview.calibration import correct_fingers, validate_profile
from hand_preview.geometry import FINGERS
from hand_preview.kinematics import RobotKinematics, ThumbRetargeter
from hand_preview.mapping import PreviewMapper
from hand_preview.wilor_calibration import WiLoRCalibrationWizard


CONTEXT={'camera':2,'backend':'dshow','swap_handedness':False,'tracker':'wilor'}


class WiLoRCalibrationTests(unittest.TestCase):
    def make_wizard(self,directory):
        robot=RobotKinematics();mapper=PreviewMapper()
        path=Path(directory)/'wilor-current.json'
        wizard=WiLoRCalibrationWizard(robot,mapper.bounds,CONTEXT,path)
        points=np.zeros((21,3))
        points[0]=[0,0,0]
        for i,joint in enumerate((5,9,13,17)):
            points[joint]=[(i-1.5)*.12,.35,0]
        points[1]=[-.2,.06,0];points[2]=[-.32,.16,.03]
        bone_lengths={finger:[.18,.15,.12] for finger in FINGERS}
        bone_lengths['thumb']=[.16,.13,.11]
        observation={'canonical':points.tolist(),'image_aspect':1.6,'camera_normal':[0,0,1],
            'bone_lengths':bone_lengths,'noise':{finger:[0,0,0] for finger in (*FINGERS,'thumb')}}
        def sample(abd,thumb_axis):
            angles={finger:{'abd':abd,'mcp':0.,'pip':0.,'dip':0.} for finger in FINGERS}
            pose=points.copy();pose[2]=pose[1]+np.asarray(thumb_axis)*.2
            return {'angles':angles,'canonical':pose.tolist(),'structure_observation':observation,
                    'valid_frames':50,'spread':.01}
        wizard.samples={'together':sample(0.,[1,0,0]),'spread':sample(.1,[1,0,0]),
                        'thumb_direction':sample(.15,[0,1,0])}
        return wizard,mapper

    def test_three_pose_profile_scales_side_spread_and_aligns_thumb_axis(self):
        with tempfile.TemporaryDirectory() as directory:
            wizard,mapper=self.make_wizard(directory)
            profile=wizard.build_profile()
            validate_profile(profile,CONTEXT)
            self.assertEqual(profile['schema'],5)
            self.assertEqual(profile['finger_policy'],'wilor_three_pose')
            for finger in FINGERS:
                output=correct_fingers({finger:{'abd':.1}},profile)[finger]['abd']
                lo,hi=mapper.bounds[f'left_{finger}_j0']
                self.assertAlmostEqual(output,.35*hi,places=6)
            rotation=ThumbRetargeter._align_axis(profile['thumb_direction']['human_axis'],
                                                 profile['thumb_direction']['robot_axis'])
            np.testing.assert_allclose(rotation@profile['thumb_direction']['human_axis'],
                                       profile['thumb_direction']['robot_axis'],atol=1e-6)

    def test_negative_spread_gets_same_fraction_of_negative_joint_range(self):
        with tempfile.TemporaryDirectory() as directory:
            wizard,mapper=self.make_wizard(directory)
            for finger in ('ring','pinky'):
                wizard.samples['spread']['angles'][finger]['abd']=-.1
            profile=wizard.build_profile()
            for finger in ('ring','pinky'):
                output=correct_fingers({finger:{'abd':-.1}},profile)[finger]['abd']
                self.assertAlmostEqual(output,.35*mapper.bounds[f'left_{finger}_j0'][0])
            old=json.loads(json.dumps(profile))
            old['finger_gains']['ring']=old['finger_gains']['pinky']=.2
            wizard.path.write_text(json.dumps(old),encoding='utf-8')
            loaded=WiLoRCalibrationWizard(wizard.robot,mapper.bounds,CONTEXT,wizard.path)
            self.assertTrue(loaded.enabled)
            self.assertEqual(loaded.profile['finger_gains'],profile['finger_gains'])
            self.assertEqual(loaded.profile['finger_offsets'],profile['finger_offsets'])
            self.assertEqual(loaded.profile['thumb_direction'],profile['thumb_direction'])

    def test_quick_side_zero_pairs_sources_and_preserves_thumb_and_flexion(self):
        with tempfile.TemporaryDirectory() as directory:
            wizard,mapper=self.make_wizard(directory)
            wizard.profile=wizard.build_profile();wizard.enabled=True
            previous=json.loads(json.dumps(wizard.profile))
            sample=wizard.samples['together']
            observation={**sample['structure_observation'],
                         'side_sway_depth':{f:-.12 for f in FINGERS}}
            angles={f:{'abd':.15,'mcp':.1,'pip':.2,'dip':0.} for f in FINGERS}
            world=np.array(sample['canonical'])*.2
            wizard.start('side_zero',0.)
            for i in range(25):wizard.ingest(angles,world,'Left',1.1+i*.15,observation)
            wizard.advance(9.)
            self.assertIsNotNone(wizard.pending)
            for finger in FINGERS:
                self.assertEqual(wizard.pending['side_sway_zero'][finger],{'wilor':.15,'depth':-.12})
                self.assertEqual(wizard.pending['finger_offsets'][finger]['mcp'],0.)
                self.assertEqual(wizard.pending['finger_offsets'][finger]['pip'],0.)
            self.assertEqual(wizard.pending['thumb_direction'],previous['thumb_direction'])
            self.assertEqual(wizard.pending['neutral_thumb'],previous['neutral_thumb'])
            wizard.apply()
            loaded=WiLoRCalibrationWizard(wizard.robot,mapper.bounds,CONTEXT,wizard.path)
            self.assertTrue(loaded.enabled)
            self.assertEqual(loaded.profile['side_sway_zero'],wizard.profile['side_sway_zero'])


if __name__=='__main__':
    unittest.main()
