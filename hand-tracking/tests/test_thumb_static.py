import asyncio
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np
from fastapi import HTTPException
from hand_preview.thumb_static import StaticThumbCalibration,STEPS,calibrated_target,validate
from hand_preview.mapping_app import make_app
from hand_preview.depth_source import landmark_result
from test_depth_chain import hand
from test_human_pose import known_hand
import test_depth_articulation as helpers

HUMAN=np.array([[.04,.015,0.],[.015,.035,0.],[-.02,.03,0.],[.04,.015,.03]])


def capture(wizard,points=HUMAN):
    for i,step in enumerate(STEPS):
        start=i*5.;wizard.start(step,start)
        p=np.zeros((21,3));p[1]=[.02,0,.6];p[4]=p[1]+points[i]
        quality={'thumb_tip_reliable':True,'front_ok':True,'palm_depth_points':30}
        for t in np.arange(1.05,3.95,.08):wizard.ingest_depth(p,np.eye(3),start+float(t),quality,{'sync_valid':True})
        wizard.advance(start+4.1)
        if step not in wizard.samples:raise AssertionError(wizard.message)


class StaticThumbTests(unittest.TestCase):
    def session(self,folder):return helpers.ArticulationTests().setup_session(folder)

    def test_four_pose_save_reload_and_each_anchor_matches_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);w=s.thumb_calibration;base=s.calibration.path.read_bytes()
            capture(w);self.assertIsNotNone(w.pending,w.message)
            w.apply();self.assertEqual(s.calibration.path.read_bytes(),base)
            loaded=StaticThumbCalibration(s.calibration,s.robot,s.mapper.bounds)
            self.assertTrue(loaded.enabled,loaded.message);model=loaded.active_model()
            for human,target in zip(model['human_m'],model['robot_m']):
                result,weight=calibrated_target(model,human,np.zeros(3))
                np.testing.assert_allclose(result,target,atol=1e-9);self.assertEqual(weight,1.)
            self.assertEqual(len(model['samples']),4)
            w.apply();self.assertEqual(len(list(Path(folder).glob('*thumb-static-backup-*.json'))),1)
            w.reset();self.assertFalse(w.path.exists());self.assertEqual(s.calibration.path.read_bytes(),base)

    def test_static_mapping_reaches_reference_root_pose_without_iterations(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);w=s.thumb_calibration;capture(w);w.apply()
            for i,step in enumerate(STEPS):
                p=np.zeros((21,3));p[1]=[.02,0,.6];p[4]=p[1]+HUMAN[i]
                s.thumb.reset()
                result=s.thumb.calculate_tip(p,np.eye(3),s.calibration.profile['skeleton'],i+1,static_model=w.active_model())
                q=[result[f'j{j}'] for j in range(4)]
                actual=(s.robot.thumb_points(q)[-1]-s.thumb.origin)@s.robot.basis
                np.testing.assert_allclose(actual,w.model['robot_m'][i],atol=5e-5)
                self.assertEqual(s.thumb.diagnostics['static_weight'],1.)

    def test_coplanar_samples_and_unstable_pose_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);w=s.thumb_calibration;bad=HUMAN.copy();bad[3]=[.04,-.015,0.]
            capture(w,bad);self.assertIsNone(w.pending);self.assertIn('区分不足',w.message)
            w.start('rest',30.)
            for i,t in enumerate(np.arange(1.05,3.95,.08)):
                p=np.zeros((21,3));p[4]=[.04+(-1)**i*.012,.015,0.]
                w.ingest_depth(p,np.eye(3),30+float(t),{'thumb_tip_reliable':True,'front_ok':True,'palm_depth_points':30},{'sync_valid':True})
            w.advance(34.1);self.assertIn('不稳定',w.message)

    def test_bad_depth_and_palm_turn_do_not_accumulate_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);w=s.thumb_calibration;w.start('rest',0.)
            for t in np.arange(1.1,3.9,.1):w.ingest_depth(np.zeros((21,3)),np.eye(3),float(t),{'thumb_tip_reliable':False},{'sync_valid':True})
            w.advance(4.1);self.assertIsNone(w.pending);self.assertFalse(w.samples);self.assertIn('深度',str(w.rejections))

    def test_changed_base_or_fixed_angle_invalidates_without_overwriting_base(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);w=s.thumb_calibration;capture(w);w.apply();base=s.calibration.path.read_bytes()
            self.assertIsNotNone(w.active_model())
            s.calibration.profile=deepcopy(s.calibration.profile)
            s.calibration.profile['thumb_direct']['fixed_terminal_deg']=20.
            self.assertIsNone(w.active_model());self.assertFalse(w.compatible())
            with self.assertRaises(ValueError):w.apply()
            self.assertEqual(s.calibration.path.read_bytes(),base)

    def test_preview_only_changes_reference_snapshot_and_requires_stop_confirmation(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);app=make_app(s);before=dict(s.mapper.positions)
            endpoint=next(r.endpoint for r in app.routes if getattr(r,'path','')=='/api/thumb-calibration/{action}')
            async def body():return {'step':'index_base','reference_deg':[50.,15.,45.]}
            request=NS(headers={'host':'127.0.0.1:8090','origin':'http://127.0.0.1:8090'},json=body)
            s.follow=NS(stop_confirmed=lambda:(_ for _ in ()).throw(ValueError('not disabled')))
            with self.assertRaises(HTTPException) as exc:asyncio.run(endpoint('preview',request))
            self.assertEqual(exc.exception.status_code,503);self.assertFalse(s.thumb_calibration.reference_preview)
            s.follow=None;asyncio.run(endpoint('preview',request))
            state=s.thumb_calibration.snapshot(0.)
            self.assertTrue(state['reference_preview']);self.assertAlmostEqual(state['reference_joints']['left_thumb_j0'],np.radians(50))
            self.assertEqual(s.mapper.positions,before);self.assertTrue(s.mapper.paused)
            self.assertIn('参考',s.gate_reason());s.on_control('resume');self.assertFalse(s.thumb_calibration.reference_preview)

    def test_far_outside_support_returns_to_nominal_and_json_is_finite(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);w=s.thumb_calibration;capture(w);w.apply()
            nominal=np.array([.08,.06,.04]);target,amount=calibrated_target(w.model,[2.,2.,2.],nominal)
            self.assertEqual(amount,0.);np.testing.assert_allclose(target,nominal)
            json.dumps(s.snapshot(),allow_nan=False)
            bad=deepcopy(w.model);bad['matrix'][0][0]+=1.
            with self.assertRaises(ValueError):validate(bad,s.calibration.profile,s.mapper.bounds)

    def test_camera_observations_enter_static_capture_through_session(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(folder);base=s.calibration.path.read_bytes();basis=np.diag([1.,1.,-1.])
            for i,step in enumerate(STEPS):
                s.thumb_calibration.start(step,i*5.)
                p=known_hand([0,.03,.04,.02]);p[4]=p[1]+HUMAN[i]@basis.T
                h=hand(p);result=landmark_result({'hands':[h]},640,480)
                for t in np.arange(1.05,3.95,.08):s.select_angles(result,False,i*5.+float(t))
                s.thumb_calibration.advance(i*5.+4.1)
                self.assertIn(step,s.thumb_calibration.samples,s.thumb_calibration.message)
            self.assertIsNotNone(s.thumb_calibration.pending,s.thumb_calibration.message)
            self.assertEqual(s.calibration.path.read_bytes(),base)


if __name__=='__main__':unittest.main()
