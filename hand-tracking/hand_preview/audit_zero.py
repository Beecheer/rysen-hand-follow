"""Record a user-confirmed zero posture. Diagnostic only: applies no offsets."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from copy import deepcopy
import re
from pathlib import Path
import statistics
import time
from urllib.request import build_opener,ProxyHandler
import xml.etree.ElementTree as ET

from .hardware_monitor import feedback_problem
from .joint_layout import JOINT_NAMES
from .mapping import ASSET_DIR


def rpy_matrix(text):
    roll,pitch,yaw=map(float,text.split())
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
            sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=5)
    parser.add_argument('--from-report',type=Path,help='Reanalyze saved feedback without sampling hardware')
    args=parser.parse_args()
    if not 2<=args.seconds<=30:parser.error('seconds must be between 2 and 30')
    logs=ASSET_DIR.parents[1]/'logs'
    model=ASSET_DIR/'apex_hand_left.urdf'
    sdk_model=json.loads((logs/'sdk-model-audit.json').read_text(encoding='utf-8'))
    # Names explicitly documented alongside JointId in the installed SDK header.
    fingers=('thumb','index','middle','ring','pinky')
    def canonical_name(value):
        if value=='Palm_link_left':return 'left_palm_base'
        match=re.fullmatch(r'f([0-4])_(joint|link)([0-4])',value)
        if match:
            finger,kind,index=match.groups()
            return 'left_'+fingers[int(finger)]+('_j' if kind=='joint' else '_link')+index
        return value
    sdk_joints={}
    for old_name,old_spec in sdk_model['joints'].items():
        spec=deepcopy(old_spec)
        for tag in ('parent','child','mimic'):
            for key in ('link','joint'):
                if key in spec['attributes'].get(tag,{}):
                    spec['attributes'][tag][key]=canonical_name(spec['attributes'][tag][key])
        sdk_joints[canonical_name(old_name)]=spec
    preview={n.get('name'):{'type':n.get('type'),'attributes':{c.tag:dict(c.attrib) for c in n}}
             for n in ET.parse(model).getroot().findall('joint') if n.get('type')=='revolute'}
    # Ignore numeric text formatting and limit effort/speed for geometric checks.
    differences=[]
    for name in sorted(set(preview)|set(sdk_joints)):
        a,b=preview.get(name),sdk_joints.get(name)
        if a is None or b is None:
            differences.append({'joint':name,'field':'joint presence'});continue
        if a['type']!=b['type']:differences.append({'joint':name,'field':'type'})
        for tag,keys in {'parent':['link'],'child':['link'],'axis':['xyz'],
                         'origin':['xyz','rpy'],'limit':['lower','upper'],
                         'mimic':['joint','multiplier','offset']}.items():
            x,y=a['attributes'].get(tag),b['attributes'].get(tag)
            if (x is None)!=(y is None):
                differences.append({'joint':name,'field':tag,'preview':x,'sdk':y});continue
            if x is None:continue
            for key in keys:
                left,right=x.get(key),y.get(key)
                if left==right:continue
                try:
                    v,w=[float(z) for z in left.split()],[float(z) for z in right.split()]
                    if tag=='origin' and key=='rpy':v,w=rpy_matrix(left),rpy_matrix(right)
                    # URDFs round dimensions/angles to different precision.
                    tolerance=1e-3 if tag=='limit' else 1e-4
                    same=len(v)==len(w) and all(abs(i-j)<tolerance for i,j in zip(v,w))
                except (ValueError,AttributeError):same=False
                if not same:differences.append({'joint':name,'field':tag+'.'+key,'preview':left,'sdk':right})
    opener=build_opener(ProxyHandler({}));samples=[];last_id=None;rejects=0
    prior=None
    if args.from_report:
        prior=json.loads(args.from_report.read_text(encoding='utf-8'))
        samples=prior['samples'];args.seconds=prior['duration_s'];rejects=prior['duplicate_packets']
        data=prior
    deadline=time.monotonic()+(0 if prior else args.seconds)
    while time.monotonic()<deadline:
        begin=time.monotonic()
        with opener.open('http://127.0.0.1:8092/status',timeout=1) as response:data=json.load(response)
        problem=feedback_problem(data,(time.monotonic()-begin)*1000)
        if problem:raise RuntimeError(problem)
        if data['packet_id']!=last_id:
            samples.append({'elapsed_s':args.seconds-(deadline-time.monotonic()),'packet_id':data['packet_id'],
                            'positions':data['positions'],'enabled':data['enabled']})
            last_id=data['packet_id']
        else:rejects+=1
        time.sleep(max(0,.05-(time.monotonic()-begin)))
    if len(samples)<args.seconds*8:raise RuntimeError('Not enough unique fresh packets')
    rows=[]
    for name in JOINT_NAMES:
        values=[s['positions'][name] for s in samples];median=statistics.median(values)
        spec=preview[name]['attributes'];limit=spec['limit']
        lower,upper=float(limit['lower']),float(limit['upper'])
        row={'name':name,'median_deg':math.degrees(median),
             'std_deg':math.degrees(statistics.pstdev(values)),
             'range_deg':math.degrees(max(values)-min(values)),
             'lower_deg':math.degrees(lower),'upper_deg':math.degrees(upper),
             'outside_limit_deg':math.degrees(max(lower-median,median-upper,0)),
             'mimic_error_deg':None}
        mimic=spec.get('mimic')
        if mimic:
            errors=[abs(s['positions'][name]-s['positions'][mimic['joint']]*float(mimic.get('multiplier',1))
                        -float(mimic.get('offset',0))) for s in samples]
            row['mimic_error_deg']=math.degrees(max(errors))
        rows.append(row)
    now=datetime.now(timezone.utc)
    report={'created_at':now.isoformat(),'label':'user-confirmed physical zero; no calibration applied',
            'feedback_captured_at':prior.get('feedback_captured_at',prior['created_at']) if prior else now.isoformat(),
            'sample_count':len(samples),'duration_s':args.seconds,'duplicate_packets':rejects,
            'version':data['version'],'native_sha256':data['native_sha256'],
            'preview_model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
            'sdk_model':sdk_model,'model_differences':differences,
            'model_comparison':'SDK header joint aliases normalized; numeric tolerances: limit .001 rad, origin/axis .0001',
            'all_fingers_disabled':all(not any(s['enabled'].values()) for s in samples),
            'joints':rows,'samples':samples}
    stamp=now.strftime('%Y%m%dT%H%M%S%fZ')
    path=logs/f'zero-audit-{stamp}.json'
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print('Report:',path)
    print('Samples:',len(samples),'Model differences:',len(differences),'All disabled:',report['all_fingers_disabled'])
    for row in rows:
        print('%-20s median=%7.2f deg std=%.3f range=%.3f outside=%.2f mimic=%s' %
              (row['name'],row['median_deg'],row['std_deg'],row['range_deg'],row['outside_limit_deg'],row['mimic_error_deg']))
    if differences:print('Model differences:',json.dumps(differences,ensure_ascii=False))


if __name__=='__main__':main()
