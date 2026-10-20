"""Linux SDK read-only bridge. No enable, parameter write or motion routes.

Runs separately from ROS/Explorer using the already installed wslc image.
"""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

from joint_layout import JOINT_NAMES, verify_sdk_layout


class ReadOnlyDevice:
    def __init__(self, sdk, address):
        verify_sdk_layout(sdk)
        self.sdk=sdk;self.address=address;self.bot=sdk.Rysen()
        self.lock=threading.Lock();self.last_packet=None;self.packet_at=None
        self.packet_id=0;self.positions={};self.velocities={};self.error=None
        self.metadata={};self.connected=False;self.registered=False

    def on_joints(self, states):
        try:
            positions={};velocities={}
            for state in states.joint_states:
                index=int(state.joint_id)
                if not 0<=index<len(JOINT_NAMES) or JOINT_NAMES[index] in positions:
                    raise ValueError('Unexpected joint IDs')
                positions[JOINT_NAMES[index]]=float(state.position)
                velocities[JOINT_NAMES[index]]=float(state.velocity)
            if len(positions)!=21 or not all(math.isfinite(v) for v in (*positions.values(),*velocities.values())):
                raise ValueError('Incomplete or non-finite feedback')
            timestamp=str(states.timestamp)
            with self.lock:
                # Repeated cached packets do not refresh freshness.
                if timestamp!=self.last_packet:
                    self.packet_at=time.monotonic();self.packet_id+=1
                    self.positions=positions;self.velocities=velocities;self.last_packet=timestamp
                    self.error=None
        except Exception as exc:
            with self.lock:self.error=str(exc)

    def connect(self):
        result=self.bot.connect(self.address,self.sdk.ConnectionType.CONNECTION_TYPE_ETHERNET)
        if result!=self.sdk.ErrorCode.ERROR_CODE_OK:raise RuntimeError('SDK connect failed: '+str(result))
        self.connected=True
        direction=self.bot.get_hand_dir()
        if direction!=self.sdk.HandDir.LEFT:raise RuntimeError('Connected device is not LEFT')
        version=self.bot.get_version_info();params=self.bot.get_parameters()
        self.metadata={'hand':'LEFT','version':{k:str(getattr(version,k)) for k in
                       ('sdk_version','hand_firmware_version','touch_sensor_version')},
                       'max_speed':list(params.max_speed),'max_accel':list(params.max_accel),
                       'max_current':list(params.max_current),
                       'joint_layout':[{'id':i,'name':name} for i,name in enumerate(JOINT_NAMES)]}
        result=self.bot.register_joint_states_callback(self.on_joints,50)
        if result!=self.sdk.ErrorCode.ERROR_CODE_OK:raise RuntimeError('Feedback registration failed: '+str(result))
        self.registered=True
        self.refresh()

    def refresh(self):
        connected=bool(self.bot.is_connected())
        fault=self.bot.get_hardware_error_code()
        enabled={name:bool(self.bot.is_finger_enabled(getattr(self.sdk.FingerId,enum))) for name,enum in
                 [('thumb','FINGER_ID_THUMB'),('index','FINGER_ID_INDEX'),('middle','FINGER_ID_MIDDLE'),
                  ('ring','FINGER_ID_RING'),('pinky','FINGER_ID_LITTLE')]}
        with self.lock:
            self.connected=connected
            self.metadata.update(fault_code=int(fault),fault=str(fault),enabled=enabled)

    def snapshot(self):
        with self.lock:
            age=None if self.packet_at is None else (time.monotonic()-self.packet_at)*1000
            return {'mode':'READ_ONLY','motion_calls':0,'address':self.address,
                    'connected':self.connected,'feedback_age_ms':age,'packet_id':self.packet_id,
                    'feedback_timestamp':self.last_packet,'positions':dict(self.positions),
                    'velocities':dict(self.velocities),'error':self.error,**self.metadata}

    def close(self):
        # No explicit enable/parameter writes here. Native SDK 1.4.7 Disconnect
        # was observed to disable all fingers; this lifecycle side effect must
        # not be mistaken for a connection that preserves another controller.
        if self.registered:self.bot.unregister_joint_states_callback()
        if self.connected:self.bot.disconnect()


def handler_for(device):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path!='/status':self.send_error(404);return
            payload=json.dumps(device.snapshot(),allow_nan=False).encode()
            self.send_response(200);self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(payload)))
            self.end_headers();self.wfile.write(payload)
        def do_POST(self):self.send_error(405,'Read-only bridge')
        def log_message(self,*args):pass
    return Handler


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--address',default='192.168.0.102')
    parser.add_argument('--port',type=int,default=8092);args=parser.parse_args()
    # Deliberately fixed to the native SDK already inside the installed image.
    library=Path('/workspace/rysen_sdk/lib/x86_64');sys.path.insert(0,str(library))
    import _rysen_sdk as sdk
    device=ReadOnlyDevice(sdk,args.address);stop=threading.Event()
    signal.signal(signal.SIGTERM,lambda *_:stop.set());signal.signal(signal.SIGINT,lambda *_:stop.set())
    server=None
    try:
        device.connect()
        device.metadata['native_sha256']=hashlib.sha256((library/'librysen_sdk.so').read_bytes()).hexdigest()
        server=ThreadingHTTPServer(('0.0.0.0',args.port),handler_for(device))
        threading.Thread(target=server.serve_forever,daemon=True).start()
        print('Read-only SDK bridge ready; no motion or enable commands',flush=True)
        while not stop.wait(.5):device.refresh()
    finally:
        if server:server.shutdown();server.server_close()
        device.close()


if __name__=='__main__':main()
