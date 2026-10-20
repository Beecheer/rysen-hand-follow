import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from hand_preview.index_follow import IndexFollow,index_target
from hand_preview.index_follow import full_hand_target
from hand_preview.joint_layout import JOINT_NAMES


def source():
    return {'state':'TRACKING','valid':True,'age_ms':10,'joints':{'left_index_j1':math.radians(32.5)}}


class IndexFollowTests(unittest.TestCase):
    def test_native_start_failure_stops_requested_state(self):
        stop=threading.Event();calls=[]
        with tempfile.TemporaryDirectory() as directory:
            token=Path(directory)/'token';token.write_text('test-token')
            controller=IndexFollow(source,stop,token,hardware_status=lambda:{'follow':{'active':False,'reason':'start_rejected'}})
            controller.post=lambda a,degrees=0:calls.append(a)
            controller.action('start');controller.started_at-=1
            thread=threading.Thread(target=controller.run);thread.start()
            deadline=time.monotonic()+1
            while controller.snapshot()['requested'] and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(controller.snapshot()['requested'])
            self.assertIn('start_rejected',controller.snapshot()['reason'])
            self.assertIn('stop',calls)
            stop.set();thread.join(1)

    def test_full_hand_limits_and_mimic(self):
        bounds={n:[-.2,.8] for n in JOINT_NAMES};bounds['left_thumb_j0']=[0,.5]
        values=dict.fromkeys(JOINT_NAMES,1.)
        target=full_hand_target({**source(),'joints':values},bounds)
        self.assertEqual(len(target),21);self.assertEqual(target[0],.5)
        self.assertEqual(target[6],.8)
        for child,parent in ((4,3),(8,7),(12,11),(16,15),(20,19)):self.assertEqual(target[child],target[parent])

    def test_full_hand_missing_thumb_or_flexion_stops_but_side_hold_is_allowed(self):
        bounds={n:[-1,1] for n in JOINT_NAMES};s={**source(),'joints':dict.fromkeys(JOINT_NAMES,0)}
        full_hand_target({**s,'held_joints':['left_index_j0']},bounds)
        for held in ('left_index_j1','left_thumb_j0'):
            with self.assertRaises(ValueError):full_hand_target({**s,'held_joints':[held]},bounds)
        target=full_hand_target({**s,'held_joints':['left_thumb_j0']},bounds,allow_held_thumb=True)
        self.assertEqual(len(target),21)
        with self.assertRaises(ValueError):full_hand_target({**s,'held_joints':['left_index_j1']},bounds,allow_held_thumb=True)
        with self.assertRaises(ValueError):full_hand_target(source(),bounds)
    def test_only_index_mcp_maps_to_small_range(self):
        self.assertAlmostEqual(index_target(source()),5)
        self.assertEqual(index_target({**source(),'joints':{'left_index_j1':100}}),10)
        self.assertEqual(index_target({**source(),'joints':{'left_index_j1':-1}}),0)

    def test_invalid_and_held_landmarks_cannot_start(self):
        for bad in ({'age_ms':200},{'valid':False},{'state':'PAUSED'},
                    {'held_joints':['left_index_j1']},{'joints':{'left_index_j1':float('nan')}}):
            with self.assertRaises(ValueError):index_target({**source(),**bad})

    def test_source_loss_stops_and_never_automatically_restarts(self):
        stop=threading.Event();current=source();calls=[]
        with tempfile.TemporaryDirectory() as directory:
            token=Path(directory)/'token';token.write_text('test-token')
            controller=IndexFollow(lambda:current.copy(),stop,token)
            controller.post=lambda action,degrees=0:calls.append((action,degrees))
            controller.action('start');thread=threading.Thread(target=controller.run);thread.start()
            current['valid']=False
            deadline=time.monotonic()+1
            while controller.snapshot()['requested'] and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(controller.snapshot()['requested'])
            current['valid']=True;time.sleep(.1)
            self.assertEqual(sum(action=='start' for action,_ in calls),1)
            self.assertIn(('stop',0),calls)
            stop.set();thread.join(1)

    def test_browser_lease_expiry_stops_targets(self):
        stop=threading.Event();calls=[]
        with tempfile.TemporaryDirectory() as directory:
            token=Path(directory)/'token';token.write_text('test-token')
            controller=IndexFollow(source,stop,token);controller.post=lambda a,degrees=0:calls.append(a)
            controller.action('start');controller.lease=time.monotonic()-1
            thread=threading.Thread(target=controller.run);thread.start()
            deadline=time.monotonic()+1
            while controller.snapshot()['requested'] and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(controller.snapshot()['requested']);self.assertIn('stop',calls)
            stop.set();thread.join(1)
