"""Direct landmark geometry; no gesture classification. Angles are radians."""
import math
import numpy as np

FINGERS = {'index': (5, 6, 7, 8), 'middle': (9, 10, 11, 12),
           'ring': (13, 14, 15, 16), 'pinky': (17, 18, 19, 20)}


def unit(v):
    length = np.linalg.norm(v)
    if length < 1e-6:
        raise ValueError('Degenerate hand geometry')
    return v / length


def joint_angle(a, b):
    a, b = unit(a), unit(b)
    return math.atan2(float(np.linalg.norm(np.cross(a, b))), float(np.dot(a, b)))


def palm_frame(world, handedness='Left'):
    p = np.asarray(world, dtype=float)
    if p.shape != (21, 3) or not np.isfinite(p[[0,5,9,17]]).all():
        raise ValueError('Expected a finite 3D palm frame')
    if handedness not in ('Left', 'Right'):
        raise ValueError('Unknown handedness')
    forward, across = unit(p[9] - p[0]), unit(p[5] - p[17])
    cross = np.cross(across, forward)
    if np.linalg.norm(cross) < 0.25:
        raise ValueError('Unstable palm plane')
    # MediaPipe world-coordinate chirality: the flexion-facing normal is
    # opposite across x forward for a Left-labelled image, positive for Right.
    normal = unit(cross) * (-1 if handedness == 'Left' else 1)
    palm_length = float(np.linalg.norm(p[9] - p[0]))
    if not 0.015 < palm_length < 0.20:
        raise ValueError('Implausible palm scale')
    across = unit(across - forward * np.dot(across, forward))
    return p, np.column_stack((across, forward, normal)), palm_length


def finger_angles(world, handedness='Left', basis_override=None):
    p, basis, palm_length = palm_frame(world, handedness)
    if basis_override is not None:basis=np.asarray(basis_override,float)
    across, normal = basis[:, 0], basis[:, 2]
    angles = {}
    for finger, (mcp, pip, dip, tip) in FINGERS.items():
        bones = [p[pip] - p[mcp], p[dip] - p[pip], p[tip] - p[dip]]
        if not np.isfinite(p[list((mcp, pip, dip, tip))]).all() or any(
                not 0.035 < np.linalg.norm(v) / palm_length < 1.4 for v in bones):
            angles[finger] = {'mcp': None, 'pip': None, 'dip': None, 'abd': None}
            continue
        reference = p[mcp] - p[0]
        reference = unit(reference - normal * np.dot(reference, normal))
        # Signed flexion separates extension from flexion. L/R is anatomical,
        # independent of display mirroring. PIP/DIP are segment bend magnitudes.
        lateral = unit(across - reference * np.dot(across, reference))
        along = float(np.dot(bones[0], reference))
        sideways = float(np.dot(bones[0], lateral))
        planar = math.hypot(along, sideways)
        mcp_angle = math.atan2(float(np.dot(bones[0], normal)), planar)
        # Near 90 degree flexion the palm projection is unobservable: hold
        # abduction only, while flexion and other fingers continue tracking.
        # URDF zero posture has parallel proximal phalanges. Wrist-to-MCP
        # rays already fan out across the palm; using those as side zero
        # incorrectly subtracts real finger spread and pulls fingers inward.
        abd = math.atan2(float(np.dot(bones[0],basis[:,0])),float(np.dot(bones[0],basis[:,1]))) \
            if planar / np.linalg.norm(bones[0]) > 0.5 else None
        angles[finger] = {'mcp': mcp_angle, 'pip': joint_angle(bones[0], bones[1]),
                          'dip': joint_angle(bones[1], bones[2]), 'abd': abd}
    return angles
