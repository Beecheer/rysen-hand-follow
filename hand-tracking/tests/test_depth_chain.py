"""D435 contracts and synthetic metric input through the complete target chain."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
import sys
if __name__=='__main__':sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from unittest.mock import patch
import numpy as np
from fastapi import HTTPException
from hand_preview.depth_source import (DepthCalibrationWizard,camera_context,landmark_result,
    structure_observation,validate_reference,Camera)
from hand_preview.depth_session import DepthMappingSession
from hand_preview.depth_simple_calibration import SimpleDepthCalibration,POSES as SIMPLE_POSES
from hand_preview.depth_articulation import ArticulatedDepthEstimator,thumb_features
from hand_preview.geometry import finger_angles,FINGERS
from hand_preview.calibration import FRONT_POSES
from hand_preview.mapping_app import make_app
from hand_preview.index_follow import full_hand_target,IndexFollow
from hand_preview.joint_layout import JOINT_NAMES
from hand_fusion import DepthHandFusion,PARENTS,CHILDREN
from test_human_pose import known_hand


def args():
    return NS(full_hand_follow=False,sdk_monitor=False,index_follow=False,tracker='d435',
              camera=0,backend='auto',swap_handedness=False,no_mirror=False,num_hands=1,
              compare_wilor=False,width=640,height=480,port=8090,depth_serial=None)


def status():
    return {'state':'RUNNING','device':{'serial':'synthetic-camera','firmware':'test','usb':'3.2'},
            'depth_scale_m':.001,'intrinsics':{'width':640,'height':480,'fx':600.,'fy':600.,
                                            'ppx':320.,'ppy':240.,'coeffs':[0]*5,'model':'none'}}


def hand(points):
    p=np.array(points,float)+[0,0,.6]
    pixels=p[:,:2]/p[:,2:3]*600+[320,160]
    return {'state':'FUSED','label':'Left','score':.99,'pixels':pixels.tolist(),
            'rays':(p/p[:,2:3]).tolist(),'focal_px':600.,
            'palm_surface_points_m':(p[[0,5,9,13,17]]-[0,0,.006]).tolist(),
            'observations':[{'valid':True,'z_m':float(z-.006),'sigma_m':.003} for z in p[:,2]],
            'fused_points_m':p.tolist(),'model_points_m':p.tolist(),
            'sources':['depth_band']*21,'quality':{'depth_constraints':21,'bone_error_p95_mm':0.},
            'sync_valid':True,'reliable_palm_points':5,'anchor_age_ms':0.,'personal_reference':True}


def capture(wizard):
    simple=isinstance(wizard,SimpleDepthCalibration)
    for i,name in enumerate(SIMPLE_POSES if simple else FRONT_POSES):
        q=[0,.03,.04,.02]
        if name=='spread':q[0]=.2
        elif name=='hook':q[2:]=[.8,.5]
        elif name=='mcp':q[1]=.65
        elif name in FINGERS:q[1:]=[.5,.65,.3]
        p=known_hand(q)
        if name in FINGERS:p[4]=p[FINGERS[name][-1]]
        h=hand(p);p=np.asarray(h['fused_points_m']);image=np.asarray(h['pixels'])
        observation=structure_observation(p,image,'Left',h)
        if simple:
            estimator=ArticulatedDepthEstimator();points,basis,_=estimator.calibration_geometry(h,0.)
            points=np.asarray(h['model_points_m']).copy()
            observation.update(metric_points_m=points.tolist(),basis=basis.tolist(),sync_valid=True,
                palm_depth_points=5,straight=True,thumb_reliable=True,thumb_features=thumb_features(points,basis).tolist())
        angles=finger_angles(p,'Left')
        for f in FINGERS:angles[f]['abd']*=wizard.robot.abduction_signs[f]
        start=i*5.;wizard.start(name,start)
        for dt in np.arange(1.05,3.96,.1):wizard.ingest(angles,p,'Left',start+float(dt),observation)
        wizard.advance(start+4.1)
        if name not in wizard.samples:raise AssertionError(name+': '+wizard.message)
    if wizard.pending is None:raise AssertionError(wizard.message)


class DepthChainTests(unittest.TestCase):
    def session(self,path):return DepthMappingSession(args(),path)

    def test_two_pose_capture_save_reload_metric_reference_and_isolation(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'d435-current.json';old=Path(folder)/'current.json';old.write_text('original A')
            session=self.session(path);session.bind_camera(status())
            wizard=session.calibration;capture(wizard)
            reference=validate_reference(wizard.pending['depth_reference'])
            self.assertEqual(len(reference['lengths_m']),20)
            self.assertFalse(path.exists());wizard.apply()
            loaded=SimpleDepthCalibration(session.robot,session.mapper.bounds,wizard.context,path=path)
            self.assertTrue(loaded.enabled,loaded.message)
            self.assertEqual(loaded.reference(),reference)
            loaded.enabled=False;self.assertIsNone(loaded.reference())
            changed={**wizard.context,'serial':'different'}
            invalid=SimpleDepthCalibration(session.robot,session.mapper.bounds,changed,path=path)
            self.assertFalse(invalid.enabled)
            wizard.apply();self.assertEqual(len(list(Path(folder).glob('*-backup-*.json'))),1)
            wizard.reset();self.assertFalse(path.exists())
            self.assertEqual(old.read_text(),'original A')

    def test_depth_calibration_rejects_inadequate_depth_without_overwriting(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status());w=s.calibration
            p=known_hand([0,0,0,0]);h=hand(p);o=structure_observation(h['fused_points_m'],h['pixels'],'Left',h)
            o.update(basis=np.eye(3).tolist(),palm_depth_points=0,sync_valid=True,straight=True,thumb_reliable=True)
            w.start('neutral',0);o['depth_quality']['depth_constraints']=2
            for dt in np.arange(1.1,3.9,.05):w.ingest(finger_angles(p),p,'Left',float(dt),o)
            w.advance(4.1);self.assertFalse(w.samples);self.assertIn('深度',str(w.rejections))

    def test_context_intrinsics_scale_and_swap_are_bound(self):
        original=status();a=camera_context(original)
        for changed in ({'depth_scale_m':.002},{'device':{'serial':'other'}},
                        {'intrinsics':{**original['intrinsics'],'fx':610.}}):
            self.assertNotEqual(a,camera_context({**original,**changed}))
        self.assertNotEqual(a,camera_context(original,True))

    def test_articulated_fit_ignores_legacy_fused_point_corruption(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status())
            h=hand(known_hand([.1,.4,.5,.2]));h['fused_points_m'][6]=[.5,-.8,.2]
            r=landmark_result({'hands':[h]},640,480)
            angles,score,reason=s.select_angles(r,False,time.perf_counter())
            self.assertIsNotNone(angles,reason);self.assertGreater(score,.9)
            self.assertAlmostEqual(angles['index']['mcp'],.4,delta=.12)
            self.assertIsNone(s.hardware);self.assertIsNone(s.follow)
            self.assertIn('标定',s.gate_reason())

    def test_swap_rebind_preserves_pause_and_resume_requests_camera_recovery(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status())
            s.mapper.pause();s.swap=True;s.on_control('swap');s.bind_camera(status())
            self.assertTrue(s.mapper.paused)
            self.assertTrue(s.calibration.context['swap_handedness'])
            s.depth_camera.paused=True;s.on_control('resume')
            self.assertFalse(s.depth_camera.paused);self.assertTrue(s.depth_camera.reset_hands.is_set())

    def test_metric_reference_reaches_solver_and_survives_fusion_reset(self):
        h=hand(known_hand([0,.25,.4,.2]));p=np.array(h['fused_points_m']);uv=np.array(h['pixels'])
        intr=NS(fx=600.,fy=600.,ppx=320.,ppy=160.,coeffs=[0]*5)
        depth=np.zeros((480,640),np.uint16)
        for (x,y),z in zip(np.rint(uv).astype(int),p[:,2]):depth[y-2:y+3,x-2:x+3]=round((z-.005)*1000)
        ref={'lengths_m':np.linalg.norm(p[CHILDREN]-p[PARENTS],axis=1).tolist()}
        fusion=DepthHandFusion();fusion.set_reference(ref);fusion.reset()
        report=fusion.estimate(uv,p,depth,.001,intr,1.,'Left',5.)
        self.assertTrue(report['personal_reference']);self.assertLess(report['quality']['bone_error_p95_mm'],5.)
        fusion.set_reference(None)
        self.assertFalse(fusion.estimate(uv,p,depth,.001,intr,1.1,'Left',5.)['personal_reference'])

    def test_direct_thumb_needs_no_special_calibration_and_preserves_base(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'d435.json';s=self.session(path);s.bind_camera(status())
            capture(s.calibration);s.calibration.apply();base=path.read_bytes()
            s.thumb_calibration.start('rest',0);s.thumb_calibration.cancel()
            self.assertTrue(s.thumb_calibration.snapshot(0)['available'])
            self.assertIsNone(s.thumb_calibration.active_model())
            self.assertEqual(path.read_bytes(),base)
            with patch('hand_preview.kinematics.ThumbRetargeter.fit',side_effect=AssertionError('no thumb IK')):
                h=hand(known_hand([0,.3,.4,.2]));a,_,_=s.select_angles(landmark_result({'hands':[h]},640,480),False,1.)
            self.assertAlmostEqual(a['thumb']['j3'],np.radians(12))

    def test_missing_thumb_tip_and_proximal_depth_holds_thumb_but_fingers_continue(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status())
            h=hand(known_hand([.1,.4,.5,.2]))
            for i in (1,2,4):h['observations'][i]={'valid':False}
            result=landmark_result({'hands':[h]},640,480)
            angles,_,_=s.select_angles(result,False,1.)
            self.assertTrue(all(angles['thumb'][f'j{i}'] is None for i in range(3)))
            self.assertAlmostEqual(angles['thumb']['j3'],np.radians(12))
            self.assertIsNotNone(angles['ring']['pip'])
            self.assertIsNotNone(angles['index']['mcp']);self.assertIsNotNone(angles['pinky']['pip'])

    def test_sdk_target_chain_clamps_and_couples_actual_fused_input(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status());capture(s.calibration);s.calibration.apply()
            s.sync_reference();reference=s.depth_camera.hand_reference
            self.assertIsNotNone(reference)
            outputs=[]
            for i,q in enumerate(([0,.1,.15,.1],[.2,.6,.7,.4],[.4,1.3,1.4,.8])):
                h=hand(known_hand(q));s.depth_status={'hand_tracking':{'hands':[h]}}
                # Feed a perspective-aligned metric depth image through the real solver.
                p=np.array(h['fused_points_m']);uv=np.array(h['pixels']);depth=np.zeros((480,640),np.uint16)
                for (x,y),z in zip(np.rint(uv).astype(int),p[:,2]):
                    if 2<=x<638 and 2<=y<478:depth[y-2:y+3,x-2:x+3]=round((z-.005)*1000)
                fusion=DepthHandFusion();fusion.set_reference(reference)
                fused=fusion.estimate(uv,p,depth,.001,NS(fx=600.,fy=600.,ppx=320.,ppy=160.),1+i,'Left',5.)
                fused['score']=.99;s.depth_status={'hand_tracking':{'hands':[fused]}}
                result=landmark_result({'hands':[fused]},640,480);s.mapper.resume()
                base=time.perf_counter()+i
                for step in range(5):
                    now=base+step*.04;a,_,reason=s.select_angles(result,False,now);s.mapper.update(a,now,now,reason)
                with patch('hand_preview.depth_session.time.perf_counter',return_value=now):source=s.hardware_source()
                self.assertTrue(source['valid'],source['reason'])
                target=full_hand_target(source,s.mapper.bounds,allow_held_thumb=True);outputs.append(target)
                for n,v in zip(JOINT_NAMES,target):self.assertTrue(s.mapper.specs[n]['lower']-1e-8<=v<=s.mapper.specs[n]['upper']+1e-8)
                for child,parent in ((4,3),(8,7),(12,11),(16,15),(20,19)):self.assertEqual(target[child],target[parent])
                self.assertLessEqual(target[1],.495094)
            self.assertGreater(abs(outputs[-1][6]-outputs[0][6]),.1)
            s.depth_status={'hand_tracking':{'hands':[]}}
            self.assertFalse(s.hardware_source()['valid'])
            self.assertEqual(len(outputs[0]),21)
            # Explicit export is used only by the separate native SDK-double test.
            if EXPORT:
                EXPORT.write_text(json.dumps({'synthetic':True,'targets':outputs,'bounds':s.mapper.bounds}),encoding='utf-8')

    def test_api_blocks_start_and_configuration_change_when_stop_unconfirmed(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status())
            s.follow=NS(stop_confirmed=lambda:(_ for _ in ()).throw(ValueError('not disabled')))
            app=make_app(s)
            async def read():return {'pose':'neutral'}
            req=NS(headers={'host':'127.0.0.1:8090','origin':'http://127.0.0.1:8090'},json=read,query_params={})
            endpoint=next(r.endpoint for r in app.routes if getattr(r,'path','')=='/api/calibration/{action}')
            with self.assertRaises(HTTPException) as e:asyncio.run(endpoint('capture',req))
            self.assertEqual(e.exception.status_code,503);self.assertIsNone(s.calibration.collecting)
            s.follow=NS(action=lambda *_:self.fail('must not start'))
            start=next(r.endpoint for r in app.routes if getattr(r,'path','')=='/api/index-follow/{action}')
            with self.assertRaises(HTTPException) as e:start('start',req)
            self.assertEqual(e.exception.status_code,409)

    def test_neutral_capture_resets_automatic_reference_before_sampling(self):
        with tempfile.TemporaryDirectory() as folder:
            s=self.session(Path(folder)/'d435.json');s.bind_camera(status());s.depth_reference_key='old'
            app=make_app(s)
            async def read():return {'pose':'neutral'}
            request=NS(headers={'host':'127.0.0.1:8090','origin':'http://127.0.0.1:8090'},json=read)
            route=next(r.endpoint for r in app.routes if getattr(r,'path','')=='/api/calibration/{action}')
            asyncio.run(route('capture',request))
            self.assertTrue(s.depth_camera.reset_hands.is_set());self.assertIsNone(s.depth_reference_key)
            self.assertTrue(s.mapper.paused);self.assertEqual(s.calibration.collecting['pose'],'neutral')

    def test_confirm_stop_requires_fresh_five_finger_disabled_feedback(self):
        c=IndexFollow.__new__(IndexFollow);c.action=lambda _:None
        import threading
        c.io_lock=threading.Lock();c.stop=threading.Event()
        state={'connected':True,'feedback_age_ms':5,'worker_age_ms':5,'follow':{'active':False},'enabled':dict.fromkeys(range(5),False)}
        c.client=NS(request=lambda *_:state);c.stop_confirmed()
        state['enabled'][0]=True
        with patch('hand_preview.index_follow.time.monotonic',side_effect=[0,.1,2.]):
            with self.assertRaisesRegex(ValueError,'五指失能'):c.stop_confirmed()


EXPORT=None
if __name__=='__main__':
    import sys
    if '--export' in sys.argv:
        EXPORT=Path(sys.argv[sys.argv.index('--export')+1]);sys.argv=sys.argv[:1]
    unittest.main()
