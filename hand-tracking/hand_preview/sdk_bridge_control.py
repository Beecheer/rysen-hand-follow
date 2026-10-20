"""Manage the dedicated SDK bridge process selected by SCRIPT and PID."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path('/opt/hand-follow');PID=Path('/tmp/hand-sdk.pid')
SCRIPT=ROOT/'sdk_bridge.py'
LOG=Path('/tmp/hand-sdk.log')


def running():
    if not PID.exists():
        matches=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                if str(SCRIPT).encode() in proc.joinpath('cmdline').read_bytes().split(b'\0'):
                    if proc.joinpath('stat').read_text().split()[2]!='Z':matches.append(int(proc.name))
            except (FileNotFoundError,ProcessLookupError):continue
        if len(matches)>1:raise RuntimeError('Multiple bridge processes; refusing ambiguous operation')
        if not matches:return None
        PID.write_text(str(matches[0]))
    try:
        pid=int(PID.read_text());proc=Path('/proc')/str(pid)
        if proc.joinpath('stat').read_text().split()[2]=='Z':return None
        if str(SCRIPT).encode() not in proc.joinpath('cmdline').read_bytes().split(b'\0'):
            raise RuntimeError('PID belongs to a different process; refusing to signal')
        return pid
    except (FileNotFoundError,ProcessLookupError):return None


def main():
    action=sys.argv[1];pid=running()
    if action=='stop':
        if pid:
            os.kill(pid,signal.SIGTERM)
            for _ in range(60):
                if not running():break
                time.sleep(.1)
            else:raise RuntimeError('SDK bridge did not exit; no forced termination attempted')
        PID.unlink(missing_ok=True);print('SDK bridge stopped')
    elif action=='start':
        if not pid:
            env=dict(os.environ,LD_LIBRARY_PATH='/workspace/rysen_sdk/lib/x86_64')
            with LOG.open('ab',buffering=0) as log:
                child=subprocess.Popen([sys.executable,'-u',str(SCRIPT)],
                    env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
            PID.write_text(str(child.pid));pid=child.pid
        print('SDK bridge PID',pid)
    else:raise ValueError('Unknown action')


if __name__=='__main__':main()
