"""Download the official model once, then run entirely offline."""

import hashlib
import os
from pathlib import Path
import urllib.request
import zipfile

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "models" / "hand_landmarker.task"


def validate_model(path: Path) -> str:
    if not path.is_file():
        raise FileNotFoundError(f"Model missing: {path}. Run with --download-model first.")
    with zipfile.ZipFile(path) as archive:
        if len([name for name in archive.namelist() if name.endswith('.tflite')]) < 2:
            raise ValueError("Model does not contain both palm and hand-landmark models.")
        if archive.testzip() is not None:
            raise ValueError("Model archive is corrupt.")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def download_model(path: Path) -> None:
    if path.exists():
        print(f"Model already present. SHA256: {validate_model(path)}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.part')
    print(f"Downloading official MediaPipe model: {MODEL_URL}", flush=True)
    try:
        request = urllib.request.Request(MODEL_URL, headers={"User-Agent": "RysenHandPreview/0.1"})
        with urllib.request.urlopen(request, timeout=60) as response, temporary.open('wb') as output:
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
        checksum = validate_model(temporary)
        os.replace(temporary, path)
        print(f"Saved model: {path}\nSHA256: {checksum}")
    finally:
        temporary.unlink(missing_ok=True)
