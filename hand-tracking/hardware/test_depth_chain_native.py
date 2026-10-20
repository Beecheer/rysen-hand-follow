"""Synthetic D435 targets -> production follower/HTTP bridge -> SDK double.

Must run in the existing test container with no real bridge on 8092. The SDK
double is compiled only into a temporary executable; real binaries are untouched.
"""
import http.client
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

ROOT=Path('/opt/hand-follow')
fixture=json.loads((ROOT/'depth-native-targets.json').read_text())
assert fixture['synthetic'] is True
probe=socket.socket();probe.settimeout(.2)
try:
    if probe.connect_ex(('127.0.0.1',8092))==0:raise RuntimeError('8092 occupied; no test started')
finally:probe.close()


def get_status():
    conn=http.client.HTTPConnection('127.0.0.1',8092,timeout=.5)
    try:conn.request('GET','/status');r=conn.getresponse();assert r.status==200;return json.loads(r.read())
    finally:conn.close()


def until(condition,seconds=3):
    deadline=time.monotonic()+seconds;last=None
    while time.monotonic()<deadline:
        try:
            last=get_status()
            if condition(last):return last
        except (OSError,http.client.HTTPException):pass
        time.sleep(.025)
    raise AssertionError('state deadline: '+str(last))


results=[]
with tempfile.TemporaryDirectory(prefix='d435-sdk-double-') as folder:
    temp=Path(folder);binary=temp/'index_camera_worker'
    subprocess.run(['g++','-std=c++17','-O2','-pthread','-I'+str(ROOT/'sdk_stub'),
                    str(ROOT/'index_camera_worker.cpp'),'-o',str(binary)],check=True)
    (temp/'control-token.txt').write_text('synthetic-only-token')
    (temp/'follow-profile.json').write_text(json.dumps({'full_hand':True}))
    (temp/'full-hand-limits.json').write_text(json.dumps({'bounds':fixture['bounds']}))
    package=temp/'hand_preview';package.mkdir();(package/'__init__.py').touch()
    for name in ('index_follow.py','bridge_client.py','joint_layout.py'):shutil.copyfile(ROOT/name,package/name)
    sys.path.insert(0,str(temp))
    from hand_preview.index_follow import IndexFollow
    from hand_preview.joint_layout import JOINT_NAMES
    launch='import sys;sys.path.insert(0,"/opt/hand-follow");import sdk_follow_bridge as b;from pathlib import Path;b.ROOT=Path(sys.argv[1]);b.main()'
    log=temp/'bridge.log'
    with log.open('w') as stream:
        child=subprocess.Popen([sys.executable,'-c',launch,str(temp)],stdout=stream,stderr=subprocess.STDOUT,
                               env={**os.environ,'TRIAL_TEST_MODE':'normal'})
    try:
        ready=until(lambda s:s.get('connected') and s['feedback_age_ms']<100)
        assert ready['version']['sdk_version']=='fake' and ready['motion_calls']==0
        assert not any(ready['enabled'].values());results.append('startup_disabled')
        current={'state':'TRACKING','valid':True,'age_ms':5,
                 'joints':dict(zip(JOINT_NAMES,fixture['targets'][0])),'held_joints':[]}
        stop=threading.Event()
        follower=IndexFollow(lambda:current.copy(),stop,temp/'control-token.txt',True,get_status)
        thread=threading.Thread(target=follower.run);thread.start()
        try:
            follower.action('start');until(lambda s:s['follow']['active'])
            for target in fixture['targets']:
                current['joints']=dict(zip(JOINT_NAMES,target));deadline=time.monotonic()+.3
                while time.monotonic()<deadline:follower.action('heartbeat');time.sleep(.04)
                actual=get_status()
                assert actual['follow']['active'],actual
                assert max(abs(a-b) for a,b in zip(actual['follow']['commands'],target))<1e-5,actual
            results.append('depth_targets_through_http_and_native')
            current['valid']=False
            until(lambda s:not s['follow']['active'] and not any(s['enabled'].values()))
            current['valid']=True;time.sleep(.25)
            assert not follower.snapshot()['requested'];assert not get_status()['follow']['active']
            results.append('source_loss_disabled_no_automatic_restart')
            follower.action('start');until(lambda s:s['follow']['active']);follower.stop_confirmed()
            assert not any(get_status()['enabled'].values());results.append('manual_stop_confirmed')
            follower.action('start');until(lambda s:s['follow']['active'])
            follower.lease=time.monotonic()-1
            until(lambda s:not s['follow']['active'] and not any(s['enabled'].values()))
            results.append('browser_lease_expiry_disabled')
        finally:stop.set();thread.join(3)
    finally:
        child.terminate();child.wait(timeout=6)
    assert child.returncode==0,log.read_text()[-1500:]
    assert 'FAKE_SET_SPEED' not in log.read_text() and 'FAKE_SET_ACCEL' not in log.read_text()
report={'synthetic_sdk_only':True,'actual_robot_motion':False,'passed':results}
(ROOT/'depth-native-validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
