from copy import deepcopy
from datetime import timedelta
from http.server import ThreadingHTTPServer
import importlib.util
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import Request,urlopen

from hand_preview.hardware_monitor import DryRunPlanner,feedback_problem
from hand_preview.joint_layout import JOINT_NAMES,SDK_ENUM_NAMES,verify_sdk_layout
from hand_preview.mapping import PreviewMapper

# The same standalone bridge file runs on Linux without project dependencies.
import sys
import hand_preview.joint_layout as layout
sys.modules['joint_layout']=layout
from hand_preview.sdk_bridge import ReadOnlyDevice,handler_for


def feedback():
    return {'mode':'READ_ONLY','motion_calls':0,'connected':True,'hand':'LEFT','fault_code':0,
            'feedback_age_ms':10,'positions':dict.fromkeys(JOINT_NAMES,0.),
            'joint_layout':[{'id':i,'name':n} for i,n in enumerate(JOINT_NAMES)],
            'max_speed':[1.]*21,'max_accel':[2.]*21}


def source():
    return {'state':'TRACKING','valid':True,'age_ms':10,'joints':dict.fromkeys(JOINT_NAMES,.6)}


class HardwareTests(unittest.TestCase):
    def test_feedback_rejects_wrong_hand_stale_fault_ids_and_nonfinite(self):
        self.assertIsNone(feedback_problem(feedback()))
        for changes in [{'hand':'RIGHT'},{'feedback_age_ms':251},{'fault_code':9},
                        {'joint_layout':[]},{'motion_calls':1},{'max_speed':[0]*21}]:
            self.assertIsNotNone(feedback_problem({**feedback(),**changes}))
        self.assertIsNotNone(feedback_problem(feedback(),241))
        bad=feedback();bad['positions']['left_thumb_j0']=float('nan')
        self.assertIsNotNone(feedback_problem(bad))

    def test_simulation_seeds_from_feedback_and_drops_commands_on_loss(self):
        planner=DryRunPlanner(PreviewMapper());actual=feedback()
        actual['positions']['left_index_j1']=.15
        seeded=planner.step(source(),actual,1)
        self.assertEqual(seeded['state'],'SEEDED')
        self.assertEqual(seeded['commands'][6]['position'],.15)
        planner.step(source(),actual,1.05)
        self.assertEqual(planner.step({**source(),'valid':False},actual,1.1)['commands'],[])
        actual['positions']['left_index_j1']=.25
        self.assertEqual(planner.step(source(),actual,1.2)['commands'][6]['position'],.25)

    def test_simulated_rate_limits_mimics_and_explicit_sdk_order(self):
        mapper=PreviewMapper();planner=DryRunPlanner(mapper);actual=feedback()
        previous=planner.step(source(),actual,1)['commands']
        for tick in range(1,101):
            commands=planner.step(source(),actual,1+tick*.05)['commands']
            for before,after in zip(previous,commands):
                self.assertEqual(after['name'],JOINT_NAMES[after['id']])
                self.assertLessEqual(abs(after['position']-before['position']),math.radians(20)*.05+1e-8)
            positions={c['name']:c['position'] for c in commands}
            for name,spec in mapper.specs.items():
                mimic=spec['mimic']
                if mimic:self.assertAlmostEqual(positions[name],positions[mimic['joint']]*mimic['multiplier']+mimic['offset'])
            previous=commands

    def test_preview_nonfinite_does_not_generate_commands(self):
        bad=source();bad['joints']['left_index_j1']=float('nan')
        self.assertEqual(DryRunPlanner(PreviewMapper()).step(bad,feedback(),1)['commands'],[])

    def test_explicit_sdk_enum_mismatch_is_rejected(self):
        ids=dict(zip(SDK_ENUM_NAMES,range(21)))
        sdk=SimpleNamespace(JointId=SimpleNamespace(**ids));verify_sdk_layout(sdk)
        sdk.JointId.JOINT_ID_INDEX_J0=6
        with self.assertRaises(ValueError):verify_sdk_layout(sdk)

    def test_readonly_lifecycle_uses_no_motion_or_enable_methods(self):
        calls=[]
        class Bot:
            def connect(self,*args):calls.append('connect');return 0
            def disconnect(self):calls.append('disconnect')
            def get_hand_dir(self):return 0
            def get_version_info(self):return SimpleNamespace(sdk_version='test',hand_firmware_version='test',touch_sensor_version='test')
            def get_parameters(self):return SimpleNamespace(max_speed=[1]*21,max_accel=[1]*21,max_current=[1]*5)
            def register_joint_states_callback(self,callback,freq):calls.append('subscribe');return 0
            def unregister_joint_states_callback(self):calls.append('unsubscribe')
            def is_connected(self):return True
            def is_finger_enabled(self,*args):return False
            def get_hardware_error_code(self):return 0
            def __getattr__(self,name):raise AssertionError('Unexpected SDK method: '+name)
        sdk=SimpleNamespace(Rysen=Bot,JointId=SimpleNamespace(**dict(zip(SDK_ENUM_NAMES,range(21)))),
            ConnectionType=SimpleNamespace(CONNECTION_TYPE_ETHERNET=1),HandDir=SimpleNamespace(LEFT=0),
            ErrorCode=SimpleNamespace(ERROR_CODE_OK=0),FingerId=SimpleNamespace(**dict.fromkeys(
            ['FINGER_ID_THUMB','FINGER_ID_INDEX','FINGER_ID_MIDDLE','FINGER_ID_RING','FINGER_ID_LITTLE'],0)))
        device=ReadOnlyDevice(sdk,'192.168.0.102');device.connect()
        states=SimpleNamespace(timestamp=timedelta(seconds=1),joint_states=[
            SimpleNamespace(joint_id=i,position=0.,velocity=0.) for i in range(21)])
        device.on_joints(states);first=device.packet_at
        device.on_joints(states);self.assertEqual(first,device.packet_at)
        self.assertEqual(device.snapshot()['packet_id'],1)
        device.close();self.assertEqual(calls,['connect','subscribe','unsubscribe','disconnect'])

    def test_bridge_rejects_all_post_routes(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),handler_for(SimpleNamespace(snapshot=feedback)))
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        origin=f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(origin+'/status') as response:self.assertEqual(json.load(response)['mode'],'READ_ONLY')
            for path in ('/move','/enable','/status'):
                with self.assertRaises(HTTPError) as error:urlopen(Request(origin+path,data=b'{}',method='POST'))
                self.assertEqual(error.exception.code,405)
        finally:server.shutdown();server.server_close();worker.join()
