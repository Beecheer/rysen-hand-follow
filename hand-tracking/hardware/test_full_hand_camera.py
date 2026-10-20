"""Full-hand native follow tests; SDK double only, including >30s run."""
import json
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time

ROOT=Path('/opt/hand-follow')
with tempfile.TemporaryDirectory(prefix='full-follow-test-') as directory:
    executable=Path(directory)/'fake-full-worker'
    subprocess.run(['g++','-std=c++17','-O2','-pthread','-I'+str(ROOT/'sdk_stub'),
                    str(ROOT/'index_camera_worker.cpp'),'-o',str(executable)],check=True)
    for scenario in ('target_timeout','bad_mimic','out_of_range','fault','stale','manual_stop','relaxed','thumb_range','nonthumb_range','follow_comm_error','unlimited'):
        mode=scenario if scenario in ('fault','stale','relaxed','thumb_range','nonthumb_range','follow_comm_error') else 'normal'
        p=subprocess.Popen([str(executable),'--full-hand'],env={**os.environ,'TRIAL_TEST_MODE':mode},
                           stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        states=queue.Queue();lines=[]
        def reader():
            for line in p.stdout:
                lines.append(line)
                if line.startswith('STATE '):states.put(json.loads(line[6:]))
        t=threading.Thread(target=reader,daemon=True);t.start()
        def send(line):p.stdin.write(line+'\n');p.stdin.flush()
        def until(condition,seconds=3):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                try:s=states.get(timeout=max(.01,deadline-time.monotonic()))
                except queue.Empty:raise AssertionError('state timeout: '+''.join(lines[-10:]))
                if condition(s):return s
            raise AssertionError('state timeout: '+''.join(lines[-10:]))
        target=[.2]*21;line='TARGET21 '+' '.join(map(str,target))
        try:
            until(lambda s:s['feedback_age_ms']<100)
            send(line+'\nSTART');started=until(lambda s:s['active'])
            assert all(started['enabled']) and started['full_hand'] and started['speed_deg_s'] is None,started
            assert started['speed_policy']=='sdk_original',started
            if scenario in ('bad_mimic','out_of_range'):
                invalid=target.copy();invalid[4]=.3 if scenario=='bad_mimic' else 100
                send('TARGET21 '+' '.join(map(str,invalid)))
                stopped=until(lambda s:not s['active'] and s['reason']=='invalid_full_hand_target')
            elif scenario=='target_timeout':stopped=until(lambda s:not s['active'])
            elif scenario in ('fault','stale'):
                for _ in range(12):send(line);time.sleep(.04)
                stopped=until(lambda s:not s['active'])
            elif scenario in ('thumb_range','nonthumb_range','follow_comm_error'):
                until(lambda s:s['active'] and s['motion_calls']>0)
                changed=target.copy();changed[0]=.4;changed[6]=.25 if scenario=='thumb_range' else .4
                update='TARGET21 '+' '.join(map(str,changed))
                send(update)
                if scenario=='thumb_range':
                    held=until(lambda s:s['active'] and s.get('thumb_range_held'))
                    assert abs(held['commands'][0]-.2)<1e-6,held
                    assert abs(held['commands'][6]-.25)<1e-6,held
                    for _ in range(5):send(update);time.sleep(.05)
                    send('STOP');stopped=until(lambda s:not s['active'])
                else:
                    stopped=until(lambda s:not s['active'])
                    assert stopped['last_stop_reason']=='follow_command_rejected_code_'+('1' if scenario=='follow_comm_error' else '3'),stopped
            else:
                duration=31 if scenario=='unlimited' else .5
                end=time.monotonic()+duration
                while time.monotonic()<end:send(line);time.sleep(.05)
                # Drain queued snapshots so this checks the most recent state.
                while not states.empty():last=states.get()
                assert last['active'],last
                assert any(abs(v)>.01 for v in last['commands'][:5]),last
                assert all(abs(v-.2)<1e-6 for v in last['commands']),last
                for child,parent in ((4,3),(8,7),(12,11),(16,15),(20,19)):
                    assert abs(last['commands'][child]-last['commands'][parent])<1e-6,last
                send('STOP');stopped=until(lambda s:not s['active'])
            assert not any(stopped['enabled']),stopped
            print('PASS full-hand',scenario,flush=True)
        finally:
            p.stdin.close();p.wait(timeout=4);t.join(timeout=1)
        assert not any('FAKE_SET_SPEED' in x or 'FAKE_SET_ACCEL' in x for x in lines),lines
