"""Process-local Windows scheduling policy for latency-sensitive vision.

Keep all CPUs available and explicitly opt out of execution-speed throttling.
This does not change the machine power plan, priority, or GPU settings.
"""
import ctypes
import os


def apply_process_policy():
    report={'platform':os.name,'applied':False}
    if os.name!='nt':
        return report
    try:
        from ctypes import wintypes

        class PowerThrottling(ctypes.Structure):
            _fields_=[('Version',wintypes.ULONG),('ControlMask',wintypes.ULONG),
                      ('StateMask',wintypes.ULONG)]

        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.GetCurrentProcess.restype=wintypes.HANDLE
        args=[wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD]
        kernel.GetProcessInformation.argtypes=args
        kernel.GetProcessInformation.restype=wintypes.BOOL
        kernel.SetProcessInformation.argtypes=args
        kernel.SetProcessInformation.restype=wintypes.BOOL
        handle=kernel.GetCurrentProcess()
        policy=PowerThrottling(1,0,0)
        if not kernel.GetProcessInformation(handle,4,ctypes.byref(policy),ctypes.sizeof(policy)):
            raise ctypes.WinError(ctypes.get_last_error())
        report['previous_control_mask']=int(policy.ControlMask)
        report['previous_state_mask']=int(policy.StateMask)
        # ControlMask=0 leaves the decision to Windows. StateMask=0 alone
        # therefore does not explicitly disable automatic background throttling.
        policy.ControlMask|=1
        policy.StateMask&=~1
        if not kernel.SetProcessInformation(handle,4,ctypes.byref(policy),ctypes.sizeof(policy)):
            raise ctypes.WinError(ctypes.get_last_error())
        if not kernel.GetProcessInformation(handle,4,ctypes.byref(policy),ctypes.sizeof(policy)):
            raise ctypes.WinError(ctypes.get_last_error())
        report.update(control_mask=int(policy.ControlMask),state_mask=int(policy.StateMask),
                      applied=bool(policy.ControlMask&1 and not policy.StateMask&1))
    except (OSError,AttributeError) as exc:
        report['error']=str(exc)
    return report
