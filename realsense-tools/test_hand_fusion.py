import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from hand_fusion import DepthHandFusion,depth_patch,solve_depths,angles,PARENTS,CHILDREN


def fixture():
    points=np.array([[0,.06,.55],[-.04,.035,.55],[-.06,.015,.55],[-.075,0,.55],[-.09,-.015,.55],
        [-.03,0,.55],[-.03,-.035,.55],[-.03,-.06,.55],[-.03,-.08,.55],
        [0,-.005,.55],[0,-.045,.55],[0,-.075,.55],[0,-.095,.55],
        [.03,0,.55],[.03,-.035,.55],[.03,-.06,.55],[.03,-.08,.55],
        [.055,.01,.55],[.055,-.02,.55],[.055,-.04,.55],[.055,-.055,.55]])
    intr=SimpleNamespace(fx=600,fy=600,ppx=320,ppy=240)
    rays=points/points[:,2:3];pixels=rays[:,:2]*600+[320,240]
    depth=np.zeros((480,640),np.uint16)
    for x,y in np.rint(pixels).astype(int):depth[y-2:y+3,x-2:x+3]=545
    return points,rays,pixels,depth,intr


class FusionTests(unittest.TestCase):
    def test_articulated_observation_mode_never_calls_point_depth_optimizer(self):
        points,rays,pixels,depth,intr=fixture();depth[:]=545
        fusion=DepthHandFusion();fusion.observe_only=True
        with patch('hand_fusion.solve_depths',side_effect=AssertionError('legacy point solver')):
            report=fusion.estimate(pixels,points,depth,.001,intr,1.,'Left',5.)
        self.assertTrue(report['quality']['observation_only'])
        self.assertEqual(report['quality']['iterations'],0)
        self.assertGreater(len(report['palm_surface_points_m']),10)
        np.testing.assert_allclose(report['rays'],rays)

    def test_depth_holes_background_and_mixed_surface_rejected(self):
        d=np.zeros((20,20),np.uint16)
        self.assertFalse(depth_patch(d,.001,[10,10])['valid'])
        d[:]=1500
        self.assertFalse(depth_patch(d,.001,[10,10],.55)['valid'])
        d[:]=545;d[8:10,8:13]=600
        self.assertFalse(depth_patch(d,.001,[10,10])['valid'])
        d[:]=545
        self.assertAlmostEqual(depth_patch(d,.001,[10,10])['z_m'],.545)

    def test_depth_corrects_noisy_model_and_preserves_projection(self):
        truth,rays,_,_,_=fixture();prior=truth.copy();prior[:,2]+=.035*np.sin(np.arange(21))
        lengths=np.linalg.norm(truth[CHILDREN]-truth[PARENTS],axis=1)
        obs=[{'valid':True,'z_m':.545,'sigma_m':.003} for _ in range(21)]
        result,quality=solve_depths(rays,prior,lengths,obs)
        self.assertLess(np.mean(abs(result[:,2]-.55)),np.mean(abs(prior[:,2]-.55))*.6)
        np.testing.assert_allclose(result/result[:,2:3],rays)
        self.assertLess(quality['bone_error_p95_mm'],5)

    def test_surface_band_does_not_force_center_onto_skin(self):
        points,rays,_,_,_=fixture();lengths=np.linalg.norm(points[CHILDREN]-points[PARENTS],axis=1)
        result,_=solve_depths(rays,points,lengths,[{'valid':True,'z_m':.545,'sigma_m':.003} for _ in range(21)])
        np.testing.assert_allclose(result,points,atol=1e-7)

    def test_missing_depth_expires_even_with_continuous_model_frames(self):
        points,_,pixels,depth,intr=fixture();fusion=DepthHandFusion()
        report=fusion.estimate(pixels,points,depth,.001,intr,1.,'Left',5)
        self.assertEqual(report['quality']['depth_constraints'],21)
        empty=np.zeros_like(depth)
        for now in [1.05,1.1,1.2]:
            r=fusion.estimate(pixels,points,empty,.001,intr,now,'Left',5)
            self.assertEqual(r['state'],'MODEL_ASSISTED');self.assertEqual(r['quality']['depth_constraints'],0)
        with self.assertRaisesRegex(ValueError,'掌部深度'):
            fusion.estimate(pixels,points,empty,.001,intr,1.3,'Left',5)

    def test_sync_and_frame_guards_and_new_operator_reset(self):
        points,_,pixels,depth,intr=fixture();fusion=DepthHandFusion()
        with self.assertRaisesRegex(ValueError,'时间差'):
            fusion.estimate(pixels,points,depth,.001,intr,1.,'Left',40)
        for t in range(35):fusion.estimate(pixels,points,depth,.001,intr,2+t*.03,'Left',5)
        self.assertEqual(len(fusion.length_samples),30)
        report=fusion.estimate(pixels,points,depth,.001,intr,4.,'Right',5)
        self.assertEqual(report['bone_reference_frames'],1)
        pixels[4,0]=-1
        with self.assertRaisesRegex(ValueError,'画面内'):
            fusion.estimate(pixels,points,depth,.001,intr,4.1,'Right',5)

    def test_all_fingers_and_thumb_angles(self):
        points,*_=fixture();result=angles(points)
        self.assertEqual(set(result),{'thumb','index','middle','ring','pinky'})
        self.assertAlmostEqual(result['index']['pip'],0)
        self.assertAlmostEqual(result['thumb']['lift'],0)
        points[8,2]-=.02
        self.assertGreater(angles(points)['index']['dip'],30)

    def test_temporal_depth_prior_does_not_flip_an_ambiguous_bone(self):
        points,*_=fixture();points[[10,11,12],2]+=[.025,.03,.035]
        rays=points/points[:,2:3];lengths=np.linalg.norm(points[CHILDREN]-points[PARENTS],axis=1)
        prior=points.copy();prior[[10,11,12],2]=.55
        out,_=solve_depths(rays,prior,lengths,[{'valid':False} for _ in range(21)],points,prior)
        self.assertLess(np.max(abs((out[:,2]-out[0,2])-(points[:,2]-points[0,2]))),.01)

    def test_model_motion_prediction_preserves_deliberate_flexion(self):
        previous,*_=fixture();points=previous.copy()
        lengths=np.linalg.norm(points[CHILDREN]-points[PARENTS],axis=1)
        for j,theta in zip([10,11,12],[.6,1.1,1.4]):
            length=np.linalg.norm(previous[j]-previous[j-1])
            points[j]=points[j-1]+[0,-length*np.cos(theta),-length*np.sin(theta)]
        rays=points/points[:,2:3]
        out,_=solve_depths(rays,points,lengths,[{'valid':False} for _ in range(21)],previous,previous)
        np.testing.assert_allclose(out,points,atol=.003)
        self.assertGreater(abs(out[12,2]-previous[12,2]),.03)


if __name__=='__main__':unittest.main()
