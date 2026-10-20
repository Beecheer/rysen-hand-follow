"""Runs the actual trial state machine against a compiled SDK double, not hardware.

Run inside the existing Linux container after uploading trial + stub files.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT=Path('/opt/hand-follow')
with tempfile.TemporaryDirectory(prefix='trial-test-') as temporary:
    executable=Path(temporary)/'index-trial-fake'
    subprocess.run(['g++','-std=c++17','-O2','-pthread','-I'+str(ROOT/'sdk_stub'),
                    str(ROOT/'index_step_trial.cpp'),'-o',str(executable)],check=True)
    for mode,execute,expected in [('normal',False,0),('normal',True,0),
                                   ('other_enabled',True,1),('enable_error',True,1),
                                   ('fault',True,1),('stale',True,1),('move_rejected',True,1),
                                   ('early_return',True,0)]:
        result=subprocess.run([str(executable),'--execute' if execute else '--preflight'],
                              env={**os.environ,'TRIAL_TEST_MODE':mode},capture_output=True,text=True,timeout=8)
        output=result.stdout+result.stderr
        assert result.returncode==expected,(mode,output)
        assert output.count('FAKE_MOVE') <= 1,(mode,output)
        if not execute or mode=='other_enabled':
            assert 'FAKE_ENABLE' not in output and 'FAKE_MOVE' not in output,(mode,output)
        else:
            assert output.count('FAKE_ENABLE')==1,(mode,output)
            assert 'INDEX_DISABLED_CONFIRMED' in output,(mode,output)
        if mode=='normal' and execute:
            assert 'FAKE_MOVE count=1 id=6' in output and 'TRIAL_COMPLETE' in output,output
        if mode=='stale':assert 'feedback stale' in output,output
        if mode=='fault':assert 'connection/fault stop' in output,output
        if mode=='move_rejected':assert 'target/result validation failed; no retry' in output,output
        if mode=='early_return':assert 'TRIAL_COMPLETE' in output,output
        print('PASS',mode,'execute='+str(execute),flush=True)
    for mode,expected,moves in [('normal',0,2),('early_return',0,2),('move_rejected',1,1),
                                 ('return_rejected',1,2),('fault',1,1),('stale',1,1)]:
        result=subprocess.run([str(executable),'--cycle'],env={**os.environ,'TRIAL_TEST_MODE':mode},
                              capture_output=True,text=True,timeout=12)
        output=result.stdout+result.stderr
        assert result.returncode==expected,(mode,output)
        assert output.count('FAKE_MOVE')==moves,(mode,output)
        assert output.count('FAKE_ENABLE')==1 and 'INDEX_DISABLED_CONFIRMED' in output,(mode,output)
        if expected==0:
            assert 'TRIAL_COMPLETE cycle_8_deg_return_zero' in output,output
            move_lines=[line for line in output.splitlines() if line.startswith('FAKE_MOVE')]
            assert 'target=0' in move_lines[-1],output
        else:assert 'TRIAL_COMPLETE' not in output,output
        print('PASS cycle',mode,flush=True)
    result=subprocess.run([str(executable),'--return-zero'],env={**os.environ,'TRIAL_TEST_MODE':'return_zero'},
                          capture_output=True,text=True,timeout=8)
    output=result.stdout+result.stderr
    assert result.returncode==0,output
    assert output.count('FAKE_MOVE')==1 and 'target=0' in output,output
    assert 'INDEX_DISABLED_CONFIRMED' in output and 'TRIAL_COMPLETE return_zero_only' in output,output
    print('PASS return-zero-only from 7 degrees',flush=True)
