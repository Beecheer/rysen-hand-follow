from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from hand_preview.thumb_tip import TipDepthThumb
from hand_preview.depth_simple_calibration import DEFAULT_THUMB
from hand_preview.depth_articulation import ArticulatedDepthEstimator,fit_plane
from hand_preview.depth_source import landmark_result
import test_depth_articulation as helpers
from test_depth_chain import hand
from test_human_pose import known_hand


class TipAndPalmTests(unittest.TestCase):
    def test_analytic_root_angles_recover_known_fk_targets_without_optimizer(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder)
            thumb=TipDepthThumb(s.robot,s.mapper.bounds,deepcopy(DEFAULT_THUMB))
            for q in ([.45,.2,.3],[.8,.35,.6],[.25,-.08,1.],[1.1,.1,.2]):
                fixed=np.radians(12);p=s.robot.thumb_points([*q,fixed])
                target=(p[-1]-p[0])@s.robot.basis
                with patch('hand_preview.depth_articulation.least_squares',side_effect=AssertionError('no numerical IK')):
                    solved,diagnostics=thumb.solve_position(target,fixed,np.array(q))
                np.testing.assert_allclose(solved,q,atol=2e-4)
                self.assertLess(diagnostics['target_error_mm'],.05)

    def test_tip_motion_changes_root_targets_with_missing_middle_landmark(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder);profile=s.calibration.profile
            # Start at the calibrated neutral local pose.
            p=np.asarray(profile['skeleton']['neutral_points_m'])+[0,0,.6]
            basis=np.eye(3)
            a=s.thumb.calculate_tip(p,basis,profile['skeleton'],0.)
            p[4]+=[-.01,-.012,.008]
            b=s.thumb.calculate_tip(p,basis,profile['skeleton'],1.)
            self.assertGreater(max(abs(a[f'j{i}']-b[f'j{i}']) for i in range(3)),.1)
            self.assertEqual(a['j3'],b['j3'])
            h=hand(known_hand([0,.3,.4,.2]));h['observations'][2]={'valid':False}
            s.select_angles(landmark_result({'hands':[h]},640,480),False,2.)
            self.assertEqual(s.thumb.diagnostics['method'],'depth_tip_analytic')

    def test_unreachable_tip_is_projected_and_all_roots_remain_in_bounds(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder)
            q,diagnostics=s.thumb.solve_position([1.,1.,1.],np.radians(12))
            self.assertTrue(diagnostics['workspace_projected'])
            for i,v in enumerate(q):self.assertTrue(s.mapper.bounds[f'left_thumb_j{i}'][0]<=v<=s.mapper.bounds[f'left_thumb_j{i}'][1])

    def test_mixed_palm_and_finger_surfaces_fit_inlier_palm(self):
        palm=np.array([[x,y,.6] for x in np.linspace(-.04,.04,8) for y in np.linspace(0,.04,6)])
        fingers=np.array([[x,y,.565+.15*x] for x in np.linspace(-.04,.04,5) for y in np.linspace(.04,.07,6)])
        center,normal,count=fit_plane(np.vstack((palm,fingers)),np.array([0,0,-1.]))
        self.assertAlmostEqual(center[2],.6,delta=.003);self.assertLess(normal[2],-.99)
        self.assertGreater(count,35)

    def test_wrist_depth_keeps_tracking_when_palm_is_covered_but_not_without_depth(self):
        est=ArticulatedDepthEstimator();h=hand(known_hand([0,.3,.4,.2]));est.estimate(h,0.)
        h['palm_surface_points_m']=[]
        for i in (5,9,13,17):h['observations'][i]={'valid':False}
        for t in (.1,.2,.35,.5,.7):
            a,_,_,_=est.estimate(h,t)
            self.assertIsNotNone(a['index']['mcp']);self.assertEqual(est.report['palm_source'],'wrist_depth')
        h['observations'][0]={'valid':False}
        with self.assertRaises(ValueError):est.estimate(h,1.)

    def test_anatomical_flexion_does_not_assume_camera_facing_surface_normal(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder)
            # A rigid half-turn reverses the palm's anatomical normal while
            # the measured surface normal still faces the camera.
            p=known_hand([.1,.5,.75,.35])@np.diag([1.,-1.,-1.])
            h=hand(p);est=ArticulatedDepthEstimator()
            a,_,_,_=est.estimate(h,0.,s.calibration.profile)
            for f in ('index','middle','ring','pinky'):
                self.assertIsNotNone(a[f]['mcp'],est.report['fingers'][f])
                self.assertAlmostEqual(a[f]['mcp'],.5,delta=.15)
                self.assertGreater(a[f]['pip'],.2)

    def test_occluded_tip_does_not_veto_visible_finger_chain(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator()
            h=hand(known_hand([0,.5,.8,.4]));est.estimate(h,0.,s.calibration.profile)
            # A mistaken fingertip on the adjacent finger is detected as
            # overlapping. Its residual is not a reliable whole-finger veto.
            h['rays'][8]=list(h['rays'][12]);h['pixels'][8]=list(h['pixels'][12])
            a,_,_,_=est.estimate(h,.04,s.calibration.profile)
            self.assertIsNotNone(a['index']['mcp'])
            self.assertIn(8,est.report['fingers']['index']['occluded_landmarks'])


if __name__=='__main__':unittest.main()

