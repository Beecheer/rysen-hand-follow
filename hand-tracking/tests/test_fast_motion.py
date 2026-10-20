"""Rapid flexion must escape the straight warm start without losing stability."""
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scipy.optimize import least_squares as scipy_least_squares
from hand_preview.depth_articulation import ArticulatedDepthEstimator
import test_depth_articulation as helpers
from test_depth_chain import hand
from test_human_pose import known_hand


class FastMotionTests(unittest.TestCase):
    def test_fast_fold_from_straight_recovers_pip_without_repeated_confirmation(self):
        with tempfile.TemporaryDirectory() as folder:
            session=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator()
            for i in range(5):est.estimate(hand(known_hand([0,.01,.01,.01])),i*.04,session.calibration.profile)
            values=[]
            for i in range(4):
                a,_,_,_=est.estimate(hand(known_hand([0,.85,1.2,.65])),.2+i*.04,session.calibration.profile)
                values.append(a['index']['pip'])
            self.assertTrue(all(v is not None for v in values))
            self.assertGreater(values[1],.8)
            self.assertLess(est.report['fingers']['index']['weighted_error_px'],8.)

    def test_repeated_fast_open_close_does_not_latch_straight(self):
        with tempfile.TemporaryDirectory() as folder:
            session=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator();values=[]
            for i in range(30):
                folded=(i//3)%2==1
                q=[0,.8,1.1,.6] if folded else [0,.02,.02,.02]
                a,_,_,_=est.estimate(hand(known_hand(q)),i*.04,session.calibration.profile)
                if a['index']['pip'] is not None:values.append((folded,a['index']['pip']))
            self.assertGreater(np.median([v for f,v in values if f]),.65)
            self.assertLess(np.median([v for f,v in values if not f]),.4)

    def test_analytic_image_depth_and_temporal_jacobian_matches_differences(self):
        with tempfile.TemporaryDirectory() as folder:
            session=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator();checked=[]
            def verify(fun,x0,**kwargs):
                q=np.array([.11,.45,.65,.28]);eps=1e-7
                numerical=np.column_stack([(fun(q+np.eye(4)[j]*eps)-fun(q-np.eye(4)[j]*eps))/(2*eps) for j in range(4)])
                np.testing.assert_allclose(kwargs['jac'](q),numerical,atol=2e-4,rtol=1e-5)
                checked.append(True)
                return scipy_least_squares(fun,x0,**kwargs)
            with patch('hand_preview.depth_articulation.least_squares',side_effect=verify):
                est.estimate(hand(known_hand([.1,.4,.5,.3])),0.,session.calibration.profile)
            self.assertGreaterEqual(len(checked),4)


if __name__=='__main__':unittest.main()
