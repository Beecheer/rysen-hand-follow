"""Replay numerical observations against the uncorrected visual baseline."""
import json
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hand_preview.depth_articulation import ArticulatedDepthEstimator
from hand_preview.geometry import FINGERS


def replay(path,profile_path):
    rows=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    profile=json.loads(Path(profile_path).read_text(encoding='utf-8-sig'))
    baseline=ArticulatedDepthEstimator(vision_prior=True)
    corrected=ArticulatedDepthEstimator(vision_prior=True,depth_root_correction=True)
    seen=set();frames=0;palm=0;near=0;errors=0;distal=0.;thumb=0.;root=[];times=[]
    for i,row in enumerate(rows):
        h=row.get('hand')
        if not h or 'rays' not in h or row['frame'] in seen:continue
        seen.add(row['frame']);now=row.get('time',i*.1)
        try:
            a,p,b,_=baseline.estimate(h,now,profile)
            start=time.perf_counter();c,q,d,_=corrected.estimate(h,now,profile)
            times.append((time.perf_counter()-start)*1000)
        except ValueError:errors+=1;continue
        frames+=1;r=corrected.report['root_correction'];palm+=r['palm_used']
        near+=sum(v.get('near_depth_used',False) for v in r['fingers'].values())
        thumb=max(thumb,float(np.max(abs(p[1:5]-q[1:5]))),float(np.max(abs(b-d))))
        for f in FINGERS:
            for k in ('pip','dip'):
                if a[f][k] is not None:distal=max(distal,abs(c[f][k]-a[f][k]))
            if a[f]['mcp'] is not None:
                root.append([abs(c[f]['mcp']-a[f]['mcp']),0. if a[f]['abd'] is None else abs(c[f]['abd']-a[f]['abd'])])
    return dict(frames=frames,rejected=errors,palm_used_frames=palm,near_depth_used_finger_frames=near,
        root_max_correction_deg=np.degrees(np.max(root,axis=0)).tolist() if root else None,
        distal_max_difference_rad=distal,thumb_inputs_max_difference=thumb,
        corrected_ms_p50_p95=np.percentile(times,[50,95]).tolist() if times else None,
        note='Numerical replay; no angle ground truth. Correction limits and unchanged distal/thumb inputs are contracts, not accuracy scores.')


if __name__=='__main__':
    output=json.dumps(replay(sys.argv[1],sys.argv[2]),ensure_ascii=False,indent=2)
    if len(sys.argv)>3:Path(sys.argv[3]).write_text(output,encoding='utf-8')
    print(output)
