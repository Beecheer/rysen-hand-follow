from copy import deepcopy
import json
import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np

from hand_preview.thumb_calibration import (ThumbCalibrationWizard,STEPS,signature,feature,metrics,
                                            cycles,validate,posture_prior)
from hand_preview.calibration import build_profile
from hand_preview.kinematics import RobotKinematics,ThumbRetargeter,rotation
from hand_preview.mapping import PreviewMapper
from test_calibration import synthetic_samples,CONTEXT
from test_mapping import make_hand,pose


class ThumbCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.robot=RobotKinematics();self.mapper=PreviewMapper()
        self.profile=build_profile(synthetic_samples(self.robot,self.mapper),self.robot,self.mapper.bounds,CONTEXT)
        self.path=Path(self.temporary.name)/'wilor-current.json'
        self.path.write_text(json.dumps(self.profile))
        self.base=NS(profile=self.profile,enabled=True,collecting=None,path=self.path)
        self.wizard=ThumbCalibrationWizard(self.base,self.robot,self.mapper.bounds,self.mapper.specs)

    def target(self,q):
        p=self.robot.thumb_points(q)
        return (p-p[0])@self.robot.basis

    def fill_samples(self):
        for s,step in enumerate(STEPS):
            knots=[]
            for i in range(1 if step=='rest' else 7):
                q=np.array([.45+.04*i,.2,.3+.015*s*i,.12+.01*i])
                target=self.target(q)
                knots.append({'target':target.tolist(),'feature':feature(target).tolist(),'tip':target[-1].tolist(),'strength':.8 if s>3 else 0})
            self.wizard.samples[step]={'knots':knots,'valid_frames':70,'spread':.01,'cycles':0 if step=='rest' else 2,
                                       'range':[[0]*3,[1]*3],'base_signature':signature(self.profile)}

    def test_independent_atomic_save_reload_backup_and_base_invalidation(self):
        original=self.path.read_bytes();self.fill_samples();self.wizard.apply()
        self.assertEqual(self.path.read_bytes(),original)
        self.assertEqual(self.wizard.path.name,'wilor-current-thumb.json')
        loaded=ThumbCalibrationWizard(self.base,self.robot,self.mapper.bounds,self.mapper.specs)
        self.assertIsNotNone(loaded.active_model())
        self.wizard.apply()
        self.assertEqual(len(list(self.path.parent.glob('wilor-current-thumb-backup-*.json'))),1)
        self.base.profile=deepcopy(self.profile);self.base.profile['created_at']='new calibration'
        self.assertIsNone(loaded.active_model())
        with self.assertRaises(ValueError):self.wizard.apply()
        self.assertEqual(self.path.read_bytes(),original)

    def test_missing_steps_and_capture_order_do_not_overwrite(self):
        with self.assertRaises(ValueError):self.wizard.start('flex',0)
        with self.assertRaises(ValueError):self.wizard.apply()
        self.assertFalse(self.wizard.path.exists())
        self.wizard.start('rest',0)
        with self.assertRaises(ValueError):self.wizard.start('rest',1)
        self.wizard.cancel();self.assertIsNone(self.wizard.collecting)

    def test_valid_static_capture_and_insufficient_frames(self):
        self.wizard.start('rest',0)
        for i in range(35):self.wizard.ingest(pose(),make_hand(),make_hand(),'Left',2+i*.1)
        self.wizard.advance(6)
        self.assertIn('rest',self.wizard.samples)
        self.wizard.start('plane',10);self.wizard.reject('test missing hand',13);self.wizard.advance(24)
        self.assertNotIn('plane',self.wizard.samples)
        self.assertIn('test missing hand',self.wizard.message)
        self.assertIn('rest',self.wizard.samples)

    def test_dynamic_capture_requires_excursion_and_two_complete_roundtrips(self):
        self.fill_samples();self.wizard.samples={k:v for k,v in self.wizard.samples.items() if k=='rest'}
        def capture(turns):
            self.wizard.start('plane',0)
            target=self.target([.45,.2,.3,.12])
            for phase in np.linspace(0,2*np.pi*turns,100):
                t=target@rotation([0,0,1],.35*np.sin(phase))
                self.wizard.buffer.append({'target':t.tolist(),'feature':feature(t).tolist(),'metrics':metrics(t).tolist(),
                                           'tip':t[-1].tolist(),'strength':0,'distance':None})
            self.wizard.advance(14)
        capture(.5);self.assertNotIn('plane',self.wizard.samples)
        capture(2.5);self.assertIn('plane',self.wizard.samples)
        self.assertGreaterEqual(self.wizard.samples['plane']['cycles'],2)
        self.assertEqual(len(self.wizard.samples['plane']['knots']),7)

    def test_contact_capture_rejects_far_apart_fingertips(self):
        self.fill_samples();del self.wizard.samples['index'];self.wizard.start('index',0)
        t=self.target([.45,.2,.3,.12])
        self.wizard.buffer=[{'target':t.tolist(),'feature':feature(t).tolist(),'metrics':metrics(t).tolist(),
                             'tip':t[-1].tolist(),'strength':0,'distance':1+.2*np.sin(p)} for p in np.linspace(0,15,80)]
        self.wizard.advance(14)
        self.assertNotIn('index',self.wizard.samples)
        self.assertIn('尚未接近',self.wizard.message)

    def test_prior_fades_outside_support_and_remains_within_limits(self):
        self.fill_samples();model=self.wizard.build();target=self.target([.5,.2,.35,.13])
        prior=posture_prior(model,target);self.assertIsNotNone(prior)
        for i,q in enumerate(prior['q']):
            lo,hi=self.mapper.bounds[f'left_thumb_j{i}'];self.assertTrue(lo<=q<=hi)
        self.assertIsNone(posture_prior(model,-target))
        model['postures'][0][1]=100
        with self.assertRaises(ValueError):validate(model,self.profile,self.mapper.bounds)

    def test_disabled_base_and_cancel_preserve_completed_samples(self):
        self.fill_samples();self.wizard.apply();self.base.enabled=False
        self.assertIsNone(self.wizard.active_model())
        with self.assertRaises(ValueError):self.wizard.start('rest',0)
        self.base.enabled=True;self.wizard.start('plane',0);self.wizard.cancel()
        self.assertEqual(set(self.wizard.samples),set(STEPS))

    def test_posture_reference_affects_ik_without_bypassing_limits(self):
        q=np.array([.7,.1,.5,.4]);target=self.target(q)
        solver=ThumbRetargeter(self.robot,self.mapper.specs)
        result=solver.fit_target(target,posture={'q':q,'weight':.5},cues={'tip':target[-1],'strength':1})
        actual=np.array(list(result.values()))
        self.assertLess(np.linalg.norm(actual-q),.15)
        self.assertTrue(np.all(actual>=solver.lower));self.assertTrue(np.all(actual<=solver.upper))
        with self.assertRaises(ValueError):solver.fit_target(target,posture={'q':[float('nan')]*4,'weight':.5})

    def test_routes_stop_hardware_and_block_start_and_resume_during_capture(self):
        from hand_preview.mapping_app import MappingSession,make_app
        from fastapi import HTTPException
        args=NS(full_hand_follow=False,sdk_monitor=False,index_follow=False,camera=0,backend='auto',
                swap_handedness=False,no_mirror=False,num_hands=1,compare_wilor=False)
        session=MappingSession(args,profile_path=self.path)
        session.calibration.profile=self.profile;session.calibration.enabled=True
        calls=[]
        session.follow=NS(action=lambda action:calls.append(action))
        session.follow.stop_confirmed=lambda:session.follow.action('stop')
        app=make_app(session)
        endpoint=lambda path:next(route.endpoint for route in app.routes if getattr(route,'path',None)==path)
        async def body():return {'step':'rest'}
        request=NS(headers={'origin':'http://127.0.0.1:8090','host':'127.0.0.1:8090'},json=body)
        capture=endpoint('/api/thumb-calibration/{action}')
        asyncio.run(capture('capture',request))
        self.assertEqual(calls,['stop']);self.assertTrue(session.mapper.paused)
        self.assertEqual(session.thumb_calibration.collecting['step'],'rest')
        with self.assertRaises(HTTPException) as error:endpoint('/api/index-follow/{action}')('start',request)
        self.assertEqual(error.exception.status_code,409)
        with self.assertRaises(HTTPException) as error:asyncio.run(endpoint('/api/control/{action}')('resume',request))
        self.assertEqual(error.exception.status_code,409)
        asyncio.run(capture('cancel',request));self.assertIsNone(session.thumb_calibration.collecting)
        def failed_stop(action):raise TimeoutError('bridge unavailable')
        session.follow.action=failed_stop
        with self.assertRaises(HTTPException) as error:asyncio.run(capture('capture',request))
        self.assertEqual(error.exception.status_code,503)
        self.assertIsNone(session.thumb_calibration.collecting)
        request.headers['origin']='https://example.invalid'
        with self.assertRaises(HTTPException) as error:asyncio.run(capture('capture',request))
        self.assertEqual(error.exception.status_code,403)


if __name__=='__main__':unittest.main()
