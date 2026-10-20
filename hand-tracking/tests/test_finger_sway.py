"""Abduction observability must not be confused with fast flexion."""
import tempfile
import unittest
import numpy as np
import test_depth_articulation as helpers
from test_depth_chain import hand
from test_human_pose import known_hand
from hand_preview.depth_articulation import ArticulatedDepthEstimator


class FingerSwayTests(unittest.TestCase):
    def test_hidden_proximal_holds_only_side_and_recovers_visible_side(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator()
            for i in range(8):
                est.estimate(hand(known_hand([.12,.3,.4,.2])),i*.04,s.calibration.profile)
            previous=est.previous['index'].copy()
            for i in range(30):
                h=hand(known_hand([.12,1.4,.8,.4]))
                h['observations'][6]['z_m']-=.05  # foreground covers proximal joint
                a,_,_,_=est.estimate(h,.32+i*.04,s.calibration.profile)
                if est.report['fingers']['index']['side_held']:
                    self.assertAlmostEqual(a['index']['abd'],previous[0])
                previous=est.previous['index'].copy()
            self.assertFalse(est.report['fingers']['index']['side_observable'])
            self.assertGreater(a['index']['mcp'],1.2)
            self.assertGreater(a['index']['pip'],.65)
            for i in range(20):
                a,_,_,_=est.estimate(hand(known_hand([.3,.2,.3,.2])),1.52+i*.04,s.calibration.profile)
            self.assertTrue(est.report['fingers']['index']['side_observable'])
            self.assertAlmostEqual(a['index']['abd'],.3,delta=.035)

    def test_visible_side_motion_is_not_locked_by_continuity(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator()
            for i in range(25):
                side=.3*min(1.,i/12)
                a,_,_,_=est.estimate(hand(known_hand([side,.2,.3,.2])),i*.04,s.calibration.profile)
            self.assertAlmostEqual(a['index']['abd'],.3,delta=.035)

    def test_pixel_noise_during_flexion_does_not_free_side_prior(self):
        with tempfile.TemporaryDirectory() as folder:
            s=helpers.ArticulationTests().setup_session(folder);est=ArticulatedDepthEstimator();values=[]
            for i in range(60):
                h=hand(known_hand([.1,.5+.2*np.sin(i/6),.5,.25]))
                rays=np.asarray(h['rays'])
                rays[[6,7,8],0]+=(-1)**i*2/600  # alternating two-pixel landmark error
                h['rays']=rays.tolist()
                a,_,_,_=est.estimate(h,i*.04,s.calibration.profile)
                if i>10:values.append(a['index']['abd'])
            self.assertLess(np.degrees(np.std(values)),2.)
            self.assertLess(np.degrees(np.max(abs(np.diff(values)))),4.)


if __name__=='__main__':unittest.main()
