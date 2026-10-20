"""Depth corrections remain subordinate to the complete visual pose."""
from copy import deepcopy
import math
import unittest
import numpy as np
from hand_preview.depth_root_correction import DepthRootCorrection
from hand_preview.geometry import FINGERS,palm_frame,finger_angles
from hand_preview.kinematics import rotation
from hand_preview.depth_articulation import ArticulatedDepthEstimator
from test_depth_chain import hand
from test_human_pose import known_hand


class RootCorrectionTests(unittest.TestCase):
    def inputs(self,q=(.1,.45,.6,.3)):
        h=hand(known_hand(q));p=np.asarray(h['model_points_m'])
        _,basis,_=palm_frame(p,'Left')
        return h,p,basis

    def test_palm_tilt_is_bounded_smooth_and_only_changes_root_angles(self):
        h,p,basis=self.inputs();base=finger_angles(p,'Left')
        h['observations']=[{'valid':False} for _ in range(21)]
        depth=basis.copy();depth[:,2]=rotation(basis[:,1],math.radians(20))@basis[:,2]
        solver=DepthRootCorrection();previous=0.
        for i in range(60):
            a,corrected,report=solver.update(h,p,depth,'regional',30,True,i*.04)
            self.assertLessEqual(abs(report['palm_correction_deg']-previous),1.3)
            previous=report['palm_correction_deg']
            self.assertLessEqual(previous,8.)
            np.testing.assert_allclose(corrected.T@corrected,np.eye(3),atol=1e-8)
            for f in FINGERS:
                self.assertEqual(a[f]['pip'],base[f]['pip']);self.assertEqual(a[f]['dip'],base[f]['dip'])
                self.assertLessEqual(abs(a[f]['mcp']-base[f]['mcp']),math.radians(8.001))
                if a[f]['abd'] is not None:self.assertLessEqual(abs(a[f]['abd']-base[f]['abd']),math.radians(4.001))
        self.assertGreater(previous,5.8)
        self.assertGreater(abs(a['index']['abd']-base['index']['abd']),.01)
        for i in range(60,130):
            a,_,report=solver.update(h,p,depth,'wrist_depth',1,False,i*.04)
        self.assertFalse(report['palm_used'])
        self.assertLess(report['palm_correction_deg'],.01)
        self.assertAlmostEqual(a['index']['abd'],base['index']['abd'],delta=.001)

    def test_local_depth_requires_temporal_support_and_rejects_outliers(self):
        h,p,basis=self.inputs();solver=DepthRootCorrection()
        h['observations'][6]['z_m']+=.004
        _,_,first=solver.update(h,p,basis,'palm_keys',5,True,0.)
        self.assertFalse(first['fingers']['index']['near_depth_used'])
        a,_,second=solver.update(h,p,basis,'palm_keys',5,True,.04)
        self.assertTrue(second['fingers']['index']['near_depth_used'])
        self.assertGreater(abs(second['fingers']['index']['mcp_correction_deg']),0.)
        bad=deepcopy(h);bad['observations'][6]['z_m']+=.1
        for i in range(2,8):
            _,_,report=solver.update(bad,p,basis,'palm_keys',5,True,i*.04)
            self.assertFalse(report['fingers']['index']['near_depth_used'])
        # A reacquisition cannot revive stale correction offsets.
        _,_,report=solver.update(h,p,basis,'palm_keys',5,True,3.)
        self.assertEqual(report['fingers']['index']['mcp_correction_deg'],0.)

    def test_folded_or_overlapping_proximal_never_supplies_side_from_depth(self):
        h,p,basis=self.inputs((.1,1.4,.8,.4));solver=DepthRootCorrection()
        for i in range(10):a,_,r=solver.update(h,p,basis,'palm_keys',5,True,i*.04)
        self.assertIsNone(a['index']['abd'])
        self.assertFalse(r['fingers']['index']['near_depth_used'])
        h,p,basis=self.inputs();h['pixels'][10]=h['pixels'][6].copy()
        for i in range(10,20):a,_,r=solver.update(h,p,basis,'palm_keys',5,True,i*.04)
        self.assertFalse(r['fingers']['index']['near_depth_used'])

    def test_large_palm_disagreement_and_stale_plane_do_not_correct(self):
        h,p,basis=self.inputs();depth=basis.copy()
        depth[:,2]=rotation(basis[:,1],math.radians(40))@basis[:,2]
        s=DepthRootCorrection()
        for i in range(10):a,_,r=s.update(h,p,depth,'regional',30,True,i*.04)
        self.assertFalse(r['palm_used']);self.assertEqual(r['palm_correction_deg'],0.)
        self.assertEqual(a,finger_angles(p,'Left'))

    def test_main_path_preserves_thumb_and_distal_angles(self):
        h,_,_=self.inputs()
        base=ArticulatedDepthEstimator(vision_prior=True)
        corrected=ArticulatedDepthEstimator(vision_prior=True,depth_root_correction=True)
        for i in range(20):
            a,p,b,ok=base.estimate(h,i*.04)
            c,q,d,valid=corrected.estimate(h,i*.04)
            np.testing.assert_allclose(p[1:5],q[1:5],atol=1e-12)
            np.testing.assert_allclose(b,d,atol=1e-12);self.assertEqual(ok,valid)
            for f in FINGERS:
                self.assertEqual(a[f]['pip'],c[f]['pip']);self.assertEqual(a[f]['dip'],c[f]['dip'])


if __name__=='__main__':unittest.main()
