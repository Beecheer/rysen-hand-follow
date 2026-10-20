"""Replay numeric D435 observations through the current mapper; no camera/SDK."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hand_preview.mapping_app import MappingSession
from hand_preview.kinematics import ThumbRetargeter
from hand_preview.finger_filter import FingerAngleFilter
from hand_preview.depth_source import DepthCalibrationWizard,landmark_result
from hand_fusion import solve_depths
from hand_preview.thumb_calibration import ThumbCalibrationWizard

def replay(path,resolve=False):
    args=NS(full_hand_follow=False,sdk_monitor=False,index_follow=False,tracker='d435',camera=0,
            backend='auto',swap_handedness=False,no_mirror=False,num_hands=1,compare_wilor=False,
            width=640,height=480,port=8090,depth_serial=None)
    # This historical diagnostic intentionally replays the v1 fused-point
    # pipeline. The current v2 session requires two-pose articulated profiles.
    session=MappingSession(args,profile_path=Path('profiles/d435-current.json'),tracker='d435')
    session.thumb=ThumbRetargeter(session.robot,session.mapper.specs,stable_continuity=True)
    session.finger_filter=FingerAngleFilter()
    profile=Path('profiles/d435-current.json');context=json.loads(profile.read_text())['context']
    session.calibration=DepthCalibrationWizard(session.robot,session.mapper.bounds,context,path=profile)
    session.thumb_calibration=ThumbCalibrationWizard(session.calibration,session.robot,session.mapper.bounds,session.mapper.specs)
    rows=json.loads(Path(path).read_text());out=[];session.mapper.resume()
    previous=previous_prior=None
    for i,row in enumerate(rows):
        report=row.get('depth',{}).get('hand_tracking',{});result=landmark_result(report,640,480)
        if resolve and result.depth_hand is not None:
            h=result.depth_hand;prior=np.asarray(h['model_points_m']);points=np.asarray(h['fused_points_m'])
            rays=points/points[:,2:3]
            points,quality=solve_depths(rays,prior,np.asarray(session.calibration.profile['depth_reference']['lengths_m']),
                                        h['observations'],previous,previous_prior)
            previous=points;previous_prior=prior.copy();h['fused_points_m']=points.tolist();h['quality']=quality
            result=landmark_result(report,640,480)
        now=i/24
        try:angles,_,reason=session.select_angles(result,context['swap_handedness'],now)
        except ValueError:continue
        session.mapper.update(angles,now,now,reason)
        out.append({'raw':angles,'output':dict(session.mapper.positions),'hand':result.depth_hand,
                    'thumb_diagnostics':dict(session.thumb.diagnostics)})
    return out

def report(rows):
    def flat(d,p=''):
        out={}
        for k,v in (d or {}).items():
            if isinstance(v,dict):out.update(flat(v,p+k+'.'))
            elif isinstance(v,(float,int)):out[p+k]=v
        return out
    summary={'frames':len(rows)}
    for field in ('raw','output'):
        a=[flat(r[field]) for r in rows[30:]];metrics=[]
        for k in sorted(set().union(*(x.keys() for x in a))):
            v=np.array([x.get(k,np.nan) for x in a])*180/np.pi
            d=np.abs(np.diff(v));finite=d[np.isfinite(d)]
            if finite.size:metrics.append({'joint':k,'std_deg':float(np.nanstd(v)),
                'step_p95_deg':float(np.percentile(finite,95)),'max_step_deg':float(finite.max())})
        summary[field]=sorted(metrics,key=lambda x:x['max_step_deg'],reverse=True)
    for field in ('model_points_m','fused_points_m'):
        a=[np.array(r['hand'][field]) for r in rows[30:] if r['hand'] and field in r['hand']]
        if len(a)>1:
            a=np.array(a);a-=a[:,0:1].copy();d=np.linalg.norm(np.diff(a,axis=0),axis=2)*1000
            summary[field]={'step_p95_mm':float(np.percentile(d,95)),'max_step_mm':float(d.max())}
    return summary

if __name__=='__main__':
    result=report(replay(sys.argv[1],'--resolve-depth' in sys.argv));print(json.dumps(result,ensure_ascii=False,indent=2))
