"""Native camera worker tests against the SDK double; never uses hardware."""
import json
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time

ROOT=Path('/opt/hand-follow')
with tempfile.TemporaryDirectory(prefix='follow-test-') as temporary:
    executable=Path(temporary)/'fake-camera-worker'
    subprocess.run(['g++','-std=c++17','-O2','-pthread','-I'+str(ROOT/'sdk_stub'),
                    str(ROOT/'index_camera_worker.cpp'),'-o',str(executable)],check=True)
    def trial(mode,scenario):
        process=subprocess.Popen([str(executable)],env={**os.environ,'TRIAL_TEST_MODE':mode},
                                 stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        states=queue.Queue();lines=[]
        def reader():
            for line in process.stdout:
                lines.append(line)
                if line.startswith('STATE '):states.put(json.loads(line[6:]))
        thread=threading.Thread(target=reader,daemon=True);thread.start()
        def send(text):process.stdin.write(text+'\n');process.stdin.flush()
        def until(predicate,timeout=3):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                data=states.get(timeout=max(.01,end-time.monotonic()))
                if predicate(data):return data
            raise AssertionError('state timeout: '+''.join(lines))
        try:
            until(lambda s:s['feedback_age_ms']<100)
            if scenario=='unarmed':
                send('START');time.sleep(.15)
                assert 'FAKE_ENABLE' not in ''.join(lines)
            else:
                send('TARGET 8\nSTART')
                until(lambda s:s['active'])
                if scenario=='manual_stop':
                    for _ in range(5):send('TARGET 8');time.sleep(.05)
                    send('STOP');until(lambda s:not s['active'])
                    assert 'FAKE_DISABLE' in ''.join(lines)
                elif scenario=='target_timeout':
                    until(lambda s:not s['active'] and s['reason']=='camera_target_timeout')
                elif scenario in ('stale','fault'):
                    for _ in range(12):send('TARGET 8');time.sleep(.04)
                    until(lambda s:not s['active'])
                elif scenario=='range_reject':
                    send('TARGET 50');until(lambda s:not s['active'] and s['reason']=='invalid_target')
                elif scenario=='eof':pass
            process.stdin.close();process.wait(timeout=4);thread.join(timeout=1)
            text=''.join(lines)
            assert process.returncode==0,text
            if scenario!='unarmed':assert 'FAKE_DISABLE' in text,text
            snapshots=[json.loads(line[6:]) for line in lines if line.startswith('STATE ')]
            assert all(not any(s['enabled'][i] for i in (0,2,3,4)) for s in snapshots)
            assert all(0<=s['command_deg']<=10 for s in snapshots)
            print('PASS camera',scenario,flush=True)
        finally:
            if process.poll() is None:process.kill();process.wait()
    for mode,scenario in [('normal','unarmed'),('normal','manual_stop'),('normal','target_timeout'),
                           ('stale','stale'),('fault','fault'),('normal','range_reject'),('normal','eof')]:
        trial(mode,scenario)
