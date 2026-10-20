"""Explicit SDK JointId mapping; never use MotorId or dict iteration order."""
JOINT_NAMES = tuple(f'left_{finger}_j{j}'
                    for finger, count in [('thumb',5),('index',4),('middle',4),('ring',4),('pinky',4)]
                    for j in range(count))
SDK_ENUM_NAMES = tuple('JOINT_ID_'+name.removeprefix('left_').upper() for name in JOINT_NAMES)


def verify_sdk_layout(sdk):
    for expected, name in enumerate(SDK_ENUM_NAMES):
        if int(getattr(sdk.JointId,name)) != expected:
            raise ValueError('SDK joint ID mismatch: '+name)

