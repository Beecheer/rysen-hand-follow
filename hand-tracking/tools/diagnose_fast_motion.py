"""Replay numeric fast-motion frames without camera, SDK or profile writes."""
import json
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hand_preview.depth_articulation import ArticulatedDepthEstimator


def replay(path,profile_path):
    rows=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    profile=json.loads(Path(profile_path).read_text(encoding='utf-8-sig'))
    estimator=ArticulatedDepthEstimator();seen=set();results=[];times=[]
    for i,row in enumerate(rows):
        hand=row.get('hand')
        if not hand or 'rays' not in hand or row['frame'] in seen:continue
        seen.add(row['frame']);begin=time.perf_counter()
        try:
            angles,_,_,_=estimator.estimate(hand,i*.1,profile)
            times.append((time.perf_counter()-begin)*1000)
            results.append({'frame':row['frame'],'before':row.get('raw'),
                            'after':angles,'details':estimator.report['fingers']})
        except ValueError:continue
    def span(field,key):
        values=[r[field]['index'][key] for r in results if r.get(field) and r[field].get('index',{}).get(key) is not None]
        return np.degrees([min(values),max(values)]).tolist() if values else None
    return {'frames':len(results),'index_pip_before_deg':span('before','pip'),'index_pip_after_deg':span('after','pip'),
            'index_reseed_frames':sum(r['details']['index'].get('reseeded',False) for r in results),
            'algorithm_ms_p50_p95':np.percentile(times,[50,95]).tolist() if times else None,
            'note':'Recorded observations, approximate replay timing, no angle ground truth.','samples':results}


if __name__=='__main__':
    report=replay(sys.argv[1],sys.argv[2])
    if len(sys.argv)>3:Path(sys.argv[3]).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},ensure_ascii=False,indent=2))
