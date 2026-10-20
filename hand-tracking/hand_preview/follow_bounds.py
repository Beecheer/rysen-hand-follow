"""Intersect installed SDK model and preview limits; preserve mimic domains."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
try:from .joint_layout import JOINT_NAMES
except ImportError:from joint_layout import JOINT_NAMES


def build_bounds(preview_path,sdk_path,device_limits=None):
    preview=ET.parse(preview_path).getroot();sdk=ET.parse(sdk_path).getroot()
    p={j.get('name'):j for j in preview.findall('joint') if j.get('type')=='revolute'}
    s={j.get('name'):j for j in sdk.findall('joint') if j.get('type')=='revolute'}
    if set(p)!=set(JOINT_NAMES) or len(s)!=21:raise ValueError('Unexpected 21-joint model layout')
    bounds={};aliases={}
    for finger_id,(finger,count) in enumerate((('thumb',5),('index',4),('middle',4),('ring',4),('pinky',4))):
        for j in range(count):
            name=f'left_{finger}_j{j}';alias=f'f{finger_id}_joint{j}';aliases[name]=alias
            a,b=p[name].find('limit'),s[alias].find('limit')
            lo=max(float(a.get('lower')),float(b.get('lower')));hi=min(float(a.get('upper')),float(b.get('upper')))
            if finger!='thumb':
                lo=max(lo,math.radians(-25) if j==0 else 0)
                hi=min(hi,math.radians(25 if j==0 else 65))
            else:
                # Keep camera targets away from rounded URDF end stops.
                margin=min(math.radians(2),(hi-lo)/4)
                lo+=margin;hi-=margin
            bounds[name]=[lo,hi]
    if device_limits:
        margin=math.radians(device_limits['margin_deg'])
        if not math.isfinite(margin) or not 0<=margin<.2:raise ValueError('Invalid device limit margin')
        for name,pair in device_limits['bounds'].items():
            if name not in bounds or len(pair)!=2 or not all(math.isfinite(v) for v in pair):raise ValueError('Invalid device limits')
            bounds[name]=[max(bounds[name][0],pair[0]+margin),min(bounds[name][1],pair[1]-margin)]
    for name in JOINT_NAMES:
        mimic=p[name].find('mimic')
        if mimic is not None:
            if float(mimic.get('multiplier',1))!=1 or float(mimic.get('offset',0))!=0:raise ValueError('Unsupported mimic')
            parent=mimic.get('joint');domain=[max(bounds[name][0],bounds[parent][0]),min(bounds[name][1],bounds[parent][1])]
            bounds[name]=domain;bounds[parent]=domain
    if any(not all(math.isfinite(x) for x in pair) or pair[0]>pair[1] for pair in bounds.values()):raise ValueError('Invalid limits')
    return {'schema':2,'thumb_margin_deg':2,'bounds':bounds,'sdk_aliases':aliases,'device_limits':device_limits,
            'preview_sha256':hashlib.sha256(Path(preview_path).read_bytes()).hexdigest(),
            'sdk_sha256':hashlib.sha256(Path(sdk_path).read_bytes()).hexdigest()}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--preview',required=True);parser.add_argument('--sdk',required=True)
    parser.add_argument('--device-limits',type=Path)
    args=parser.parse_args();profile=build_bounds(args.preview,args.sdk,json.loads(args.device_limits.read_text()) if args.device_limits else None)
    root=Path('/opt/hand-follow')
    (root/'full-hand-limits.txt').write_text(''.join(f'{i} {profile["bounds"][name][0]:.12f} {profile["bounds"][name][1]:.12f}\n' for i,name in enumerate(JOINT_NAMES)))
    print(json.dumps(profile))
