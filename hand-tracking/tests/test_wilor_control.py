import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
import numpy as np

from hand_preview.wilor_detector import WiLoRDetector
from hand_preview.index_follow import IndexFollow,full_hand_target
from hand_preview.joint_layout import JOINT_NAMES
from test_index_follow import source


class WiLoRControlTests(unittest.TestCase):
    def test_http_failure_marks_frame_invalid_then_next_frame_recovers(self):
        class Connection:
            failed=True
            def request(self,*args):
                if self.failed:self.failed=False;raise TimeoutError('test stall')
            def close(self):pass
            def getresponse(self):return NS(status=200,read=lambda:json.dumps({'hands':[]}).encode())
        with patch('hand_preview.wilor_detector.http.client.HTTPConnection',return_value=Connection()):
            detector=WiLoRDetector();image=NS(numpy_view=lambda:np.zeros((48,64,3),np.uint8))
            failed=detector.detect_for_video(image,0)
            self.assertIn('WiLoR',failed.backend_error)
            recovered=detector.detect_for_video(image,33)
            self.assertFalse(hasattr(recovered,'backend_error'));self.assertEqual(recovered.hand_landmarks,[])

    def test_full_hand_accepts_180ms_but_rejects_expired_source(self):
        bounds={n:[-1,1] for n in JOINT_NAMES}
        valid={**source(),'age_ms':180,'joints':dict.fromkeys(JOINT_NAMES,0.)}
        self.assertEqual(len(full_hand_target(valid,bounds)),21)
        with self.assertRaises(ValueError):full_hand_target({**valid,'age_ms':200},bounds)

    def test_hybrid_uses_explicit_400ms_capture_age_without_changing_default(self):
        bounds={n:[-1,1] for n in JOINT_NAMES}
        current={**source(),'age_ms':350,'joints':dict.fromkeys(JOINT_NAMES,0.)}
        with self.assertRaises(ValueError):full_hand_target(current,bounds)
        self.assertEqual(len(full_hand_target(current,bounds,max_age_ms=400)),21)
        with self.assertRaises(ValueError):full_hand_target({**current,'age_ms':400},bounds,max_age_ms=400)
        with self.assertRaises(ValueError):full_hand_target(current,bounds,max_age_ms=1000)

    def test_short_partial_finger_loss_holds_but_long_loss_stops(self):
        controller=IndexFollow.__new__(IndexFollow)
        controller.full_hand=True;controller.bounds={n:[-1,1] for n in JOINT_NAMES};controller.held_since={}
        current={**source(),'joints':dict.fromkeys(JOINT_NAMES,0.),'held_joints':['left_ring_j1']}
        controller.source=lambda:current
        with self.assertRaises(ValueError):controller.target()
        controller.held_since={}
        with patch('hand_preview.index_follow.time.monotonic',return_value=10.):
            self.assertEqual(len(controller.target(True)),21)
        with patch('hand_preview.index_follow.time.monotonic',return_value=10.51):
            with self.assertRaises(ValueError):controller.target(True)
        current['held_joints']=[]
        controller.target(True);self.assertEqual(controller.held_since,{})
