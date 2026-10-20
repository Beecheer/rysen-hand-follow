"""Known-angle, occlusion, calibration and no-thumb-IK regression tests."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from fastapi import HTTPException
from fastapi.encoders import jsonable_encoder
from hand_preview.depth_articulation import ArticulatedDepthEstimator,DirectThumb,thumb_features,fit_plane
from hand_preview.depth_simple_calibration import SimpleDepthCalibration,DEFAULT_THUMB,validate,validate_thumb
from hand_preview.depth_source import landmark_result
from hand_preview.geometry import FINGERS
from hand_preview.kinematics import rotation
from hand_preview.mapping_app import make_app
from test_depth_chain import hand,args,status,capture
from test_human_pose import known_hand
from hand_preview.depth_session import DepthMappingSession


class ArticulationTests(unittest.TestCase):
    def setup_session(self,folder):
        s=DepthMappingSession(args(),Path(folder)/'new.json');s.bind_camera(status())
        capture(s.calibration);s.calibration.apply()
        return s

    def test_fixed_bone_lengths_and_fold_sequence_recover_without_backfolds(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);est=ArticulatedDepthEstimator()
            outputs=[]
            for i,t in enumerate(np.linspace(0,1,32)):
                q=np.array([.1,.9*t,1.2*t,.7*t]);h=hand(known_hand(q))
                a,p,_,_=est.estimate(h,i/30,s.calibration.profile)
                outputs.append([a['middle'][k] for k in ('mcp','pip','dip')])
                for f,ids in FINGERS.items():
                    np.testing.assert_allclose(np.linalg.norm(np.diff(p[list(ids)],axis=0),axis=1),
                        s.calibration.profile['skeleton']['finger_lengths_m'][f],atol=1e-8)
                    self.assertGreaterEqual(a[f]['pip'],0);self.assertGreaterEqual(a[f]['dip'],0)
            self.assertGreater(outputs[-1][0],.7);self.assertGreater(outputs[-1][1],1.)
            self.assertLess(np.max(abs(np.diff(outputs,axis=0))),np.radians(20))

    def test_model_z_corruption_does_not_change_calibrated_finger_solution(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);h=hand(known_hand([.12,.5,.8,.4]));bad=deepcopy(h)
            for i in range(21):bad['model_points_m'][i][2]+=.04*(-1)**i
            first=ArticulatedDepthEstimator().estimate(h,0,s.calibration.profile)[0]
            second=ArticulatedDepthEstimator().estimate(bad,0,s.calibration.profile)[0]
            self.assertEqual(first,second)

    def test_front_surface_on_hidden_tip_does_not_pull_joint_to_palm(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);est=ArticulatedDepthEstimator()
            for i,t in enumerate(np.linspace(.1,1,25)):
                h=hand(known_hand([0,.8*t,1.2*t,.6*t]));a,p,_,_=est.estimate(h,i/30,s.calibration.profile)
            previous=a['middle']['pip']
            # Surface 4 cm in front of the predicted hidden fingertip.
            h['observations'][12]['z_m']=p[12,2]-.04
            h['model_points_m'][12][2]+=.1
            a,_,_,_=est.estimate(h,25/30,s.calibration.profile)
            self.assertIn(12,est.report['fingers']['middle']['occluded_landmarks'])
            self.assertLess(abs(a['middle']['pip']-previous),np.radians(12))

    def test_palm_plane_robust_fit_rejects_depth_outliers(self):
        grid=np.array([[x,y,.6] for x in np.linspace(-.04,.04,7) for y in np.linspace(0,.07,7)])
        grid[::9,2]-=.04
        center,n,count=fit_plane(grid)
        self.assertLess(n[2],-.99);self.assertAlmostEqual(center[2],.6,places=3)
        self.assertLess(count,len(grid))

    def test_missing_plane_only_allows_short_carry_then_requires_reacquire(self):
        est=ArticulatedDepthEstimator();h=hand(known_hand([0,.2,.3,.1]));est.estimate(h,0)
        bad=deepcopy(h);bad['palm_surface_points_m']=[];bad['observations']=[{'valid':False}]*21
        est.estimate(bad,.1)
        with self.assertRaises(ValueError):est.estimate(bad,.4)

    def test_thumb_tip_never_affects_angles_and_fixed_terminal_is_exact(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);p=np.asarray(hand(known_hand([0,0,0,0]))['model_points_m'])
            basis=np.diag([1.,1.,-1.]);reference=thumb_features(p,basis)
            a=DirectThumb(s.mapper.bounds,deepcopy(DEFAULT_THUMB))
            b=DirectThumb(s.mapper.bounds,deepcopy(DEFAULT_THUMB))
            with patch('hand_preview.depth_articulation.least_squares',side_effect=AssertionError('thumb must not optimise')):
                expected=a.calculate(p,basis,reference,0)
                p[4]=[10.,-20.,30.];actual=b.calculate(p,basis,reference,0)
            self.assertEqual(expected,actual);self.assertAlmostEqual(actual['j3'],np.radians(12))
            p[3]=p[2]+rotation([0,1,0],.3)@(p[3]-p[2])
            changed=b.calculate(p,basis,reference,1.)
            self.assertNotAlmostEqual(changed['j2'],actual['j2'])
            self.assertEqual(changed['j3'],actual['j3'])

    def test_two_pose_profile_rejects_legacy_and_thumb_out_of_bounds(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);profile=deepcopy(s.calibration.profile)
            self.assertEqual(set(profile['samples']),{'neutral','together'})
            profile['schema']=3
            with self.assertRaises(ValueError):validate(profile,s.calibration.context,s.mapper.bounds)
            with self.assertRaises(ValueError):validate_thumb({**DEFAULT_THUMB,'fixed_terminal_deg':100},s.mapper.bounds)
            self.assertEqual(s.thumb_calibration.snapshot(0)['kind'],'depth_thumb_static')

    def test_second_pose_does_not_require_thumb_pose_or_thumb_depth(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);s.calibration.start('together',0.)
            h=hand(known_hand([0,0,0,0]))
            for i in (1,2,3):h['observations'][i]={'valid':False}
            r=landmark_result({'hands':[h]},640,480)
            with patch('hand_preview.depth_session.thumb_features',side_effect=AssertionError('second pose must ignore thumb')):
                for now in np.arange(1.05,3.95,.1):s.select_angles(r,False,float(now))
            s.calibration.advance(4.1)
            self.assertGreaterEqual(s.calibration.samples['together']['valid_frames'],20,s.calibration.message)
            self.assertIsNotNone(s.calibration.pending,s.calibration.message)

    def test_fixed_angle_api_persists_and_pauses_without_enabling(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);app=make_app(s)
            endpoint=next(r.endpoint for r in app.routes if getattr(r,'path','')=='/api/depth-config')
            async def body():return {'fixed_terminal_deg':20.}
            req=NS(headers={'host':'127.0.0.1:8090','origin':'http://127.0.0.1:8090'},json=body)
            s.follow=NS(stop_confirmed=lambda:None)
            asyncio.run(endpoint(req))
            self.assertTrue(s.mapper.paused);self.assertEqual(s.thumb.config['fixed_terminal_deg'],20.)
            self.assertEqual(json.loads(s.calibration.path.read_text(encoding='utf-8'))['thumb_direct']['fixed_terminal_deg'],20.)
            saved=s.calibration.path.read_bytes()
            s.follow=NS(stop_confirmed=lambda:(_ for _ in ()).throw(ValueError('not disabled')))
            with self.assertRaises(HTTPException):asyncio.run(endpoint(req))
            self.assertEqual(s.calibration.path.read_bytes(),saved)

    def test_live_snapshot_is_json_serializable_after_articulated_estimate(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.setup_session(folder);h=hand(known_hand([0,.3,.4,.2]))
            s.select_angles(landmark_result({'hands':[h]},640,480),False,1.)
            encoded=jsonable_encoder(s.snapshot())
            json.dumps(encoded,allow_nan=False)
            self.assertIsInstance(encoded['human_structure']['front_ok'],bool)


if __name__=='__main__':unittest.main()
