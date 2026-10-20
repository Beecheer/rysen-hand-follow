"""Replay numerical curl observations only; never writes profiles or uses SDK."""
from collections import Counter
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hand_preview.depth_articulation import ArticulatedDepthEstimator


def replay(path,profile_path):
    rows=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    profile=json.loads(Path(profile_path).read_text(encoding='utf-8-sig'))
    estimator=ArticulatedDepthEstimator();before=Counter();after=Counter();sources=Counter();seen=set()
    for i,row in enumerate(rows):
        if row['frame'] in seen:continue
        seen.add(row['frame']);before[row['reason'] or 'accepted']+=1
        if not row.get('hand') or 'rays' not in row['hand']:
            after[row.get('message') or 'no hand']+=1;continue
        try:
            estimator.estimate(row['hand'],i*.12,profile)
            after['accepted']+=1;sources[estimator.report['palm_source']]+=1
        except ValueError as exc:after[str(exc)]+=1
    return {'frames':len(seen),'before':dict(before),'after':dict(after),'palm_sources':dict(sources),
            'note':'Same observations replay, approximate timing; not angle ground truth or live acceptance.'}


if __name__=='__main__':
    output=json.dumps(replay(sys.argv[1],sys.argv[2]),ensure_ascii=False,indent=2)
    if len(sys.argv)>3:Path(sys.argv[3]).write_text(output,encoding='utf-8')
    else:print(output)
