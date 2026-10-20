"""Authenticated localhost bridge to the native index/full-hand SDK worker."""
from copy import deepcopy
import hmac
import hashlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from joint_layout import JOINT_NAMES

ROOT=Path('/opt/hand-follow')


def main():
    token=(ROOT/'control-token.txt').read_text().strip()
    full_hand=json.loads((ROOT/'follow-profile.json').read_text()).get('full_hand',False) if (ROOT/'follow-profile.json').exists() else False
    metadata={'joint_layout':[{'id':i,'name':n} for i,n in enumerate(JOINT_NAMES)],
              'native_sha256':hashlib.sha256(Path('/workspace/rysen_sdk/lib/x86_64/librysen_sdk.so').read_bytes()).hexdigest()}
    stop=threading.Event();lock=threading.Lock();write_lock=threading.Lock()
    latest={'state':'STARTING','reason':'waiting_native_worker','active':False,'connected':False}
    received=None
    child=subprocess.Popen([str(ROOT/'index_camera_worker')]+(['--full-hand'] if full_hand else []),stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT,text=True,bufsize=1)
    def reader():
        nonlocal latest,received
        for line in child.stdout:
            if line.startswith('STATE '):
                try:
                    value=json.loads(line[6:])
                    with lock:latest=value;received=time.monotonic()
                except (ValueError,TypeError):pass
            else:print(line.rstrip(),flush=True)
        stop.set()
    threading.Thread(target=reader,daemon=True).start()
    def send(line):
        with write_lock:
            if child.poll() is not None:raise RuntimeError('native worker exited')
            child.stdin.write(line+'\n');child.stdin.flush()
    def status():
        with lock:data=deepcopy(latest);stamp=received
        age=1e9 if stamp is None else (time.monotonic()-stamp)*1000
        positions=data.pop('positions',[]);enabled=data.pop('enabled',[])
        feedback_age=data.get('feedback_age_ms',1e9)+age
        return {**metadata,'mode':'FULL_HAND_FOLLOW' if full_hand else 'INDEX_FOLLOW','control_capable':True,'full_hand':full_hand,
                'version':data.get('version',{}),
                'connected':bool(data.get('connected')) and age<250,
                'hand':'LEFT','motion_calls':data.get('motion_calls',0),'fault_code':data.get('fault_code',0),
                'feedback_age_ms':feedback_age,'worker_age_ms':age,
                'positions':dict(zip(JOINT_NAMES,positions)),
                'enabled':dict(zip(('thumb','index','middle','ring','pinky'),enabled)),
                'follow':{**data,'active':bool(data.get('active')) and age<250}}
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'
        def setup(self):
            super().setup()
            self.connection.settimeout(2)
            self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        def reply(self,code,data):
            payload=json.dumps(data,allow_nan=False).encode();self.send_response(code)
            self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
        def do_GET(self):
            if self.path!='/status':self.reply(404,{});return
            self.reply(200,status())
        def do_POST(self):
            if self.path!='/control':self.reply(404,{});return
            if not hmac.compare_digest(self.headers.get('X-Control-Token',''),token):self.reply(403,{});return
            try:
                length=int(self.headers.get('Content-Length',0))
                if not 0<length<1024:raise ValueError('invalid body size')
                body=json.loads(self.rfile.read(length));action=body['action']
                if action=='stop':send('STOP')
                elif action in ('target','start'):
                    if full_hand:
                        positions=body['positions']
                        if not isinstance(positions,list) or len(positions)!=21 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in positions):raise ValueError('invalid 21-joint target')
                        line='TARGET21 '+' '.join('%.10f'%v for v in positions)
                    else:
                        degrees=body['degrees']
                        if not isinstance(degrees,(int,float)) or not 0<=degrees<=10:raise ValueError('target outside 0..10')
                        line='TARGET %.8f'%degrees
                    send(line+('\nSTART' if action=='start' else ''))
                else:raise ValueError('unknown action')
                self.reply(200,{'ok':True})
            except (ValueError,KeyError,TypeError,RuntimeError,OSError) as exc:self.reply(400,{'error':str(exc)})
        def log_message(self,*_):pass
    class Server(ThreadingHTTPServer):
        daemon_threads=True
        request_queue_size=64
    server=Server(('0.0.0.0',8092),Handler)
    signal.signal(signal.SIGTERM,lambda *_:stop.set());signal.signal(signal.SIGINT,lambda *_:stop.set())
    threading.Thread(target=server.serve_forever,daemon=True).start()
    try:
        while not stop.wait(.1):pass
    finally:
        try:send('STOP');child.stdin.close()
        except (OSError,RuntimeError):pass
        try:child.wait(timeout=4)
        except subprocess.TimeoutExpired:
            child.terminate()
            try:child.wait(timeout=2)
            except subprocess.TimeoutExpired:print('NATIVE_EXIT_NOT_CONFIRMED',flush=True)
        server.shutdown();server.server_close()


if __name__=='__main__':main()
