from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import threading
import cv2
import numpy as np

from hand_preview.comparison import landmark_result,Comparison
from hand_preview.mapping_app import MappingSession
from test_mapping import make_hand


def hand():
    p=make_hand()
    return {'label':'Left','world':p.tolist(),'image':(p[:,:2]+.5).tolist()}


class ComparisonTests(unittest.TestCase):
    def test_optional_recording_closes_as_a_readable_video(self):
        c=Comparison.__new__(Comparison);c.lock=threading.Lock();c.data={}
        c.record_request=None;c.writer=None;c.record_path=None;c.record_start=None;c.record_last=0.;c.record_frames=0
        with tempfile.TemporaryDirectory() as d:
            c.record_folder=Path(d)
            frame=np.full((48,64,3),80,np.uint8)
            c.record_frame(frame,1.)
            self.assertEqual(list(Path(d).iterdir()),[])
            c.record('start')
            for now in np.arange(2.,2.5,.08):c.record_frame(frame,float(now))
            c.record('stop');c.record_frame(frame,2.6)
            video=cv2.VideoCapture(str(c.record_path))
            try:
                ok,decoded=video.read();self.assertTrue(ok)
                self.assertEqual(decoded.shape,frame.shape)
                self.assertGreaterEqual(video.get(cv2.CAP_PROP_FRAME_COUNT),5)
            finally:video.release();c.close_recording()

    def test_landmarks_keep_mano_openpose_order_and_chirality(self):
        h=hand();r=landmark_result([h])
        self.assertEqual(r.handedness[0][0].category_name,'Left')
        np.testing.assert_allclose([[p.x,p.y,p.z] for p in r.hand_world_landmarks[0]],h['world'])
        np.testing.assert_allclose([[p.x,p.y] for p in r.hand_landmarks[0]],h['image'])
        h['label']='Right'
        self.assertEqual(landmark_result([h]).handedness[0][0].category_name,'Right')

    def test_invalid_model_data_is_rejected(self):
        h=hand();h['world'][8][0]=float('nan')
        with self.assertRaises(ValueError):landmark_result([h])
        h=hand();h['image'].pop()
        with self.assertRaises(ValueError):landmark_result([h])

    def test_latest_frame_replaces_waiting_frame(self):
        # Exercise bounded mailbox without opening camera/model or network.
        import threading
        c=Comparison.__new__(Comparison);c.lock=threading.Lock();c.pending=None
        c.data={'dropped':0};c.event=threading.Event()
        c.record_frame=lambda frame,now:None
        image=np.zeros((4,4,3),np.uint8);status={'inference_ms':1,'mapping_ms':2}
        c.submit(image,1,0,None,status);c.submit(image,2,.03,None,status)
        image[:]=255;status['inference_ms']=99
        self.assertEqual(c.pending[1],2);self.assertEqual(c.data['dropped'],1)
        self.assertEqual(int(c.pending[0].max()),0);self.assertEqual(c.pending[4]['inference_ms'],1)

    def test_comparison_session_has_separate_profile_and_no_sdk(self):
        args=SimpleNamespace(full_hand_follow=False,sdk_monitor=False,index_follow=False,
            camera=0,backend='auto',swap_handedness=False,no_mirror=False,num_hands=1,compare_wilor=False)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'wilor.json'
            session=MappingSession(args,profile_path=path,tracker='wilor')
            self.assertEqual(session.calibration.path,path)
            self.assertEqual(session.calibration.context['tracker'],'wilor')
            self.assertIsNone(session.hardware);self.assertIsNone(session.follow)
            self.assertFalse(path.exists())

    def test_slower_backend_can_collect_twenty_frames_without_changing_a(self):
        from hand_preview.calibration import CalibrationWizard
        from hand_preview.geometry import finger_angles
        from hand_preview.kinematics import RobotKinematics
        from hand_preview.mapping import PreviewMapper
        from hand_preview.human_pose import FrontHandEstimator
        robot=RobotKinematics();mapper=PreviewMapper();p=make_hand()
        angles=finger_angles(p,'Left')
        obs=FrontHandEstimator().estimate(p,p[:,:2]*1000,'Left',0)[3]
        with tempfile.TemporaryDirectory() as d:
            context={'tracker':'wilor'}
            wizard=CalibrationWizard(robot,mapper.bounds,context,Path(d)/'wilor.json',capture_seconds=8)
            wizard.start('neutral',0)
            for dt in np.arange(1.1,7.9,.3):wizard.ingest(angles,p,'Left',float(dt),obs)
            wizard.advance(8.1)
            self.assertIn('neutral',wizard.samples,wizard.message)
            self.assertGreaterEqual(wizard.samples['neutral']['valid_frames'],20)
            standard=CalibrationWizard(robot,mapper.bounds,context,Path(d)/'a.json')
            self.assertEqual(standard.capture_seconds,4)
