import json
import math
from pathlib import Path
import unittest

import numpy as np

from hand_preview.geometry import FINGERS, finger_angles, palm_frame, unit
from hand_preview.kinematics import RobotKinematics, ThumbRetargeter
from hand_preview.mapping import PreviewMapper, load_joint_specs
from test_mapping import make_hand, pose


class SideMotionTests(unittest.TestCase):
    def test_side_and_flexion_decompose_independently(self):
        p=make_hand(0,0)
        for finger,indices in FINGERS.items():
            mcp,pip,dip,tip=indices
            reference=np.array([0.,1,0]);across=np.array([1.,0,0])
            lateral=unit(across-reference*np.dot(across,reference))
            abd,flex=math.radians(18),math.radians(40)
            direction=math.cos(flex)*(math.cos(abd)*reference+math.sin(abd)*lateral)+math.sin(flex)*np.array([0,0,-1])
            p[pip]=p[mcp]+direction*.03
            p[dip]=p[pip]+direction*.02
            p[tip]=p[dip]+direction*.015
        angles=finger_angles(p)
        for finger in FINGERS:
            self.assertAlmostEqual(angles[finger]['abd'],math.radians(18),places=6)
            self.assertAlmostEqual(angles[finger]['mcp'],math.radians(40),places=6)

    def test_projection_degenerate_holds_only_side(self):
        angles=finger_angles(make_hand(89,0))
        self.assertIsNone(angles['index']['abd'])
        self.assertAlmostEqual(angles['index']['mcp'],math.radians(89))

    def test_parallel_fingers_have_zero_side_even_with_fanned_palm_bones(self):
        p=make_hand()
        for mcp,pip,dip,tip in FINGERS.values():
            p[pip]=p[mcp]+[0,.03,0]
            p[dip]=p[pip]+[0,.02,0]
            p[tip]=p[dip]+[0,.015,0]
        for values in finger_angles(p).values():self.assertAlmostEqual(values['abd'],0,places=7)

    def test_one_bad_tip_does_not_stop_other_fingers(self):
        points=make_hand(30,40);points[8]=np.nan
        angles=finger_angles(points)
        self.assertIsNone(angles['index']['mcp'])
        self.assertIsNotNone(angles['middle']['mcp'])

    def test_mirrored_side_invariant(self):
        points=make_hand(20,35)
        left=finger_angles(points,'Left')
        right=finger_angles(points*np.array([-1,1,1]),'Right')
        for finger in FINGERS:
            self.assertAlmostEqual(left[finger]['abd'],right[finger]['abd'])


class ThumbTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot=RobotKinematics()
        cls.specs=load_joint_specs()

    def test_known_robot_thumb_poses_can_be_reconstructed(self):
        for q in [[.7,.3,.4,.5],[.2,.6,.7,.2],[1.,.5,.2,.8]]:
            solver=ThumbRetargeter(self.robot,self.specs)
            points=self.robot.thumb_points(q)
            target=(points-points[0])@self.robot.basis
            result=solver.fit_target(target,temporal_weight=0)
            self.assertLess(solver.fit_error,.025)
            for i in range(4):
                self.assertTrue(solver.lower[i]<=result[f'j{i}']<=solver.upper[i])

    def test_real_thumb_fixture_and_left_right_parity(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/thumb_up_world.json').read_text())
        solver=ThumbRetargeter(self.robot,self.specs)
        right=np.array(fixture['world']);left=right*np.array([-1,1,1])
        target_right=solver.target_from_landmarks(right,'Right')
        target_left=solver.target_from_landmarks(left,'Left')
        np.testing.assert_allclose(target_left,target_right,atol=1e-9)
        result=solver.fit(right,'Right')
        self.assertLess(solver.fit_error,.32)
        self.assertTrue(all(math.isfinite(x) for x in result.values()))

    def test_invalid_thumb_is_rejected(self):
        solver=ThumbRetargeter(self.robot,self.specs)
        with self.assertRaises(ValueError):solver.fit_target(np.full((4,3),np.nan))

    def test_resume_from_zero_does_not_stick_at_joint_bounds(self):
        solver=ThumbRetargeter(self.robot,self.specs)
        solver.previous=np.zeros(4)
        points=self.robot.thumb_points([.8,.5,.4,.6])
        target=(points-points[0])@self.robot.basis
        solver.fit_target(target)
        self.assertLess(solver.fit_error,.025)

    def test_motion_mapper_drives_all_16_and_respects_coupling(self):
        mapper=PreviewMapper()
        desired=pose()
        for finger in FINGERS:desired[finger]['abd']=math.radians(-18 if finger=='pinky' else 18)
        desired['thumb']={f'j{i}':v for i,v in enumerate([.6,.3,.4,.5])}
        for i in range(100):mapper.update(desired,i/30,i/30)
        self.assertEqual(len(mapper.bounds),16)
        for name in mapper.bounds:self.assertGreater(abs(mapper.positions[name]),.01,name)
        self.assertAlmostEqual(mapper.positions['left_thumb_j4'],mapper.positions['left_thumb_j3'])
        held=dict(mapper.positions)
        partial=pose(20,20);partial['thumb']={f'j{i}':None for i in range(4)}
        mapper.update(partial,100/30,100/30)
        for i in range(5):self.assertEqual(held[f'left_thumb_j{i}'],mapper.positions[f'left_thumb_j{i}'])
        self.assertIn('left_thumb_j0',mapper.snapshot(100/30)['held_joints'])
        self.assertNotEqual(held['left_index_j1'],mapper.positions['left_index_j1'])


if __name__=='__main__':unittest.main()
