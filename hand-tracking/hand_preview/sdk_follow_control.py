from pathlib import Path
import sdk_bridge_control as control
control.SCRIPT=control.ROOT/'sdk_follow_bridge.py'
control.PID=Path('/tmp/hand-sdk-follow.pid')
control.LOG=Path('/tmp/hand-sdk-follow.log')
if __name__=='__main__':control.main()
