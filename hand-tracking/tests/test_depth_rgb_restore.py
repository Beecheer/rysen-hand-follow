"""Restore visual finger geometry without altering depth thumb calibration."""
from copy import deepcopy
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import test_depth_articulation as helpers
from test_depth_chain import hand
from test_human_pose import known_hand
from hand_preview.depth_articulation import ArticulatedDepthEstimator
from hand_preview.geometry import finger_angles,FINGERS


class RGBRestoreTests(unittest.TestCase):
    def test_fingers_use_model_pose_and_ignore_local_surface_and_image_noise(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder)
            self.assertTrue(s.front.vision_prior)
            est=ArticulatedDepthEstimator(vision_prior=True);h=hand(known_hand([.15,.5,.7,.3]))
            expected=finger_angles(h['model_points_m'],'Left')
            with patch('hand_preview.depth_articulation.least_squares',side_effect=AssertionError('no finger refit')):
                actual,clean,_,_=est.estimate(h,0.,s.calibration.profile)
                noisy=deepcopy(h)
                for ids in FINGERS.values():
                    for i in ids[1:]:
                        noisy['observations'][i]['z_m']+=.05*(-1)**i
                        noisy['rays'][i][0]+=.02*(-1)**i
                again,_,_,_=est.estimate(noisy,.04,s.calibration.profile)
            self.assertEqual(actual,expected);self.assertEqual(again,expected)
            for f,ids in FINGERS.items():
                np.testing.assert_allclose(np.linalg.norm(np.diff(clean[list(ids)],axis=0),axis=1),
                    s.calibration.profile['skeleton']['finger_lengths_m'][f],atol=1e-9)
            self.assertEqual(est.report['finger_solver'],'rgb_model_3d')

    def test_thumb_points_basis_and_quality_match_existing_depth_path(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder)
            h=hand(known_hand([.1,.5,.6,.2]))
            _,old,basis_old,valid_old=ArticulatedDepthEstimator().estimate(h,0.,s.calibration.profile)
            _,new,basis_new,valid_new=s.front.estimate(h,0.,s.calibration.profile)
            np.testing.assert_allclose(old[1:5],new[1:5],atol=1e-12)
            np.testing.assert_allclose(basis_old,basis_new,atol=1e-12)
            self.assertEqual(valid_old,valid_new)

    def test_fold_keeps_flexion_and_unobservable_side_is_none(self):
        h=hand(known_hand([.1,1.4,.8,.4]))
        a,_,_,_=ArticulatedDepthEstimator(vision_prior=True).estimate(h,0.)
        self.assertGreater(a['index']['mcp'],1.2)
        self.assertGreater(a['index']['pip'],.7)
        self.assertIsNone(a['index']['abd'])


if __name__=='__main__':unittest.main()
