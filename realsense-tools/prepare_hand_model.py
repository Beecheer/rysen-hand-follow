"""Reuse or download the pinned official MediaPipe task model."""
import hashlib
from pathlib import Path
import shutil
import urllib.request

ROOT=Path(__file__).resolve().parent
MODEL=ROOT/'models'/'hand_landmarker.task'
SHA='fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1'
URL='https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task'


def ensure_model():
    MODEL.parent.mkdir(exist_ok=True)
    for source in (MODEL,ROOT.parent/'hand-tracking'/'models'/'hand_landmarker.task'):
        if source.is_file() and hashlib.sha256(source.read_bytes()).hexdigest()==SHA:
            if source!=MODEL:shutil.copyfile(source,MODEL)
            return MODEL
    temporary=MODEL.with_suffix('.part')
    try:
        with urllib.request.urlopen(URL,timeout=60) as response:temporary.write_bytes(response.read())
        if hashlib.sha256(temporary.read_bytes()).hexdigest()!=SHA:raise ValueError('MediaPipe model hash mismatch')
        temporary.replace(MODEL)
    finally:temporary.unlink(missing_ok=True)
    return MODEL


if __name__=='__main__':print(ensure_model())
