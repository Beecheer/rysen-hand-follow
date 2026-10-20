"""Windows webcam preview using MediaPipe Tasks and OpenCV.

Inference always receives the original frame. Display mirroring only changes
the rendering; it does not silently change handedness or landmark coordinates.
"""

import argparse
from collections import Counter
from pathlib import Path
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

from .model import DEFAULT_MODEL, download_model, validate_model

WINDOW = 'Rysen - Hand recognition preview (camera only)'
CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
)


def confidence(value: str) -> float:
    result = float(value)
    if not 0 <= result <= 1:
        raise argparse.ArgumentTypeError('Confidence must be between 0 and 1.')
    return result


def positive_int(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError('Value must be positive.')
    return number


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera', type=int, default=0, help='Camera index (default: 0)')
    p.add_argument('--backend', choices=('auto', 'dshow', 'msmf'), default='auto')
    p.add_argument('--width', type=positive_int, default=640)
    p.add_argument('--height', type=positive_int, default=480)
    p.add_argument('--fps', type=positive_int, default=30, help='Requested capture rate; actual rate may differ')
    p.add_argument('--num-hands', type=int, choices=(1, 2), default=2)
    p.add_argument('--detection-confidence', type=confidence, default=0.6)
    p.add_argument('--presence-confidence', type=confidence, default=0.6)
    p.add_argument('--tracking-confidence', type=confidence, default=0.6)
    p.add_argument('--no-mirror', action='store_true')
    p.add_argument('--swap-handedness', action='store_true', help='Manually swap displayed Left/Right labels')
    p.add_argument('--show-ids', action='store_true')
    p.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    p.add_argument('--download-model', action='store_true', help='Download model if absent, then exit')
    p.add_argument('--self-test', action='store_true', help='Run inference on a blank image, without camera')
    p.add_argument('--headless', action='store_true', help='Capture and infer without a window')
    p.add_argument('--max-frames', type=positive_int, help='Exit after this many frames')
    return p


def create_detector(args):
    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(
            model_asset_path=str(args.model.resolve()),
            delegate=mp.tasks.BaseOptions.Delegate.CPU,
        ),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=args.num_hands,
        min_hand_detection_confidence=args.detection_confidence,
        min_hand_presence_confidence=args.presence_confidence,
        min_tracking_confidence=args.tracking_confidence,
    )
    return mp.tasks.vision.HandLandmarker.create_from_options(options)


def open_camera(args):
    backends = {'dshow': cv2.CAP_DSHOW, 'msmf': cv2.CAP_MSMF}
    candidates = ('dshow', 'msmf') if args.backend == 'auto' else (args.backend,)
    for name in candidates:
        print(f'Opening camera {args.camera} via {name}...', flush=True)
        camera = cv2.VideoCapture(args.camera, backends[name])
        try:
            if not camera.isOpened():
                camera.release()
                continue
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            camera.set(cv2.CAP_PROP_FPS, args.fps)
            # Some integrated cameras need several reads to warm up.
            for _ in range(12):
                ok, frame = camera.read()
                if ok and frame is not None and frame.size:
                    print(f'Camera ready: {frame.shape[1]}x{frame.shape[0]} via {name}', flush=True)
                    return camera, frame, name
                time.sleep(0.05)
        except BaseException:
            camera.release()
            raise
        camera.release()
    raise RuntimeError(
        f'Cannot read camera {args.camera}. Close Teams/Camera/browser camera users, '
        'check Windows camera privacy permissions and the lens shutter, '
        'or try --camera 1 / --backend msmf.'
    )


def hand_label(category, swap):
    name = category.category_name
    if swap:
        name = {'Left': 'Right', 'Right': 'Left'}.get(name, name)
    return name


def pixel_point(landmark, width, height, mirror):
    x = int(round(landmark.x * (width - 1)))
    y = int(round(landmark.y * (height - 1)))
    if mirror:
        x = width - 1 - x
    return max(0, min(width - 1, x)), max(0, min(height - 1, y))


def text_line(image, message, xy, color=(240, 240, 240), scale=0.55):
    cv2.putText(image, message, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (15, 15, 15), 3, cv2.LINE_AA)
    cv2.putText(image, message, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_preview(frame, result, *, mirror, swap, show_ids, fps, inference_ms, backend, show_shortcuts=True):
    image = cv2.flip(frame, 1) if mirror else frame.copy()
    height, width = image.shape[:2]
    for landmarks, categories in zip(result.hand_landmarks, result.handedness):
        category = categories[0]
        name = hand_label(category, swap)
        color = (90, 230, 90) if name == 'Left' else (255, 190, 70)
        points = [pixel_point(point, width, height, mirror) for point in landmarks]
        for a, b in CONNECTIONS:
            cv2.line(image, points[a], points[b], color, 2, cv2.LINE_AA)
        for i, point in enumerate(points):
            cv2.circle(image, point, 4, (245, 245, 245), -1, cv2.LINE_AA)
            if show_ids:
                text_line(image, str(i), (point[0] + 5, point[1] - 5), scale=0.35)
        x = max(5, min(point[0] for point in points))
        y = max(85, min(point[1] for point in points) - 12)
        y = min(height - 40, y)
        text_line(image, f'{name} | label {category.score:.0%}', (x, y), color)
    text_line(image, f'FPS {fps:.1f} | inference {inference_ms:.1f} ms | hands {len(result.hand_landmarks)}', (10, 24))
    text_line(image, f'CAMERA ONLY | {backend} | mirror {"ON" if mirror else "OFF"} | swap {"ON" if swap else "OFF"}', (10, 48))
    if not result.hand_landmarks:
        text_line(image, 'No hand - show your palm to the camera', (10, 76), (80, 220, 255))
    if show_shortcuts:
        text_line(image, 'Q/Esc quit | M mirror | H swap L/R | K point IDs', (10, height - 14), scale=0.47)
    return image


def run_preview(args, detector):
    camera = None
    count = 0
    detected_frames = 0
    labels = Counter()
    inference_total = 0.0
    mirror, swap, show_ids = not args.no_mirror, args.swap_handedness, args.show_ids
    start = previous = time.perf_counter()
    fps = 0.0
    last_timestamp = -1
    try:
        camera, frame, backend = open_camera(args)
        if not args.headless:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, args.width, args.height)
        print('Local processing only; no image recording or robot commands. Q/Esc exits.', flush=True)
        start = previous = time.perf_counter()
        while True:
            timestamp = max(last_timestamp + 1, int((time.perf_counter() - start) * 1000))
            last_timestamp = timestamp
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
            inference_start = time.perf_counter()
            result = detector.detect_for_video(image, timestamp)
            inference_ms = (time.perf_counter() - inference_start) * 1000
            inference_total += inference_ms
            count += 1
            if result.hand_landmarks:
                detected_frames += 1
            labels.update(hand_label(categories[0], swap) for categories in result.handedness)
            now = time.perf_counter()
            instant = 1 / max(now - previous, 1e-6)
            fps = instant if count == 1 else 0.9 * fps + 0.1 * instant
            previous = now
            if not args.headless:
                display = draw_preview(frame, result, mirror=mirror, swap=swap, show_ids=show_ids,
                                       fps=fps, inference_ms=inference_ms, backend=backend)
                cv2.imshow(WINDOW, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q'), ord('Q')):
                    break
                if key in (ord('m'), ord('M')):
                    mirror = not mirror
                elif key in (ord('h'), ord('H')):
                    swap = not swap
                elif key in (ord('k'), ord('K')):
                    show_ids = not show_ids
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if args.max_frames and count >= args.max_frames:
                break
            ok, frame = camera.read()
            if not ok or frame is None or not frame.size:
                raise RuntimeError('Camera frame read failed. Camera released; restart the preview to retry.')
    finally:
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
        elapsed = time.perf_counter() - start
        print(f'Stopped. Frames={count}; elapsed={elapsed:.1f}s; '
              f'average FPS={count / max(elapsed, 1e-6):.1f}; '
              f'average inference={inference_total / max(count, 1):.1f}ms; '
              f'frames with hands={detected_frames}; labels={dict(labels)}', flush=True)


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.download_model:
            download_model(args.model)
            return 0
        validate_model(args.model)
        with create_detector(args) as detector:
            if args.self_test:
                blank = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.zeros((480, 640, 3), dtype=np.uint8))
                for timestamp in (0, 33, 67):
                    result = detector.detect_for_video(blank, timestamp)
                    if result.hand_landmarks:
                        raise RuntimeError('Unexpected hand detection on a blank image.')
                draw_preview(np.zeros((480, 640, 3), dtype=np.uint8), result, mirror=True, swap=False,
                             show_ids=False, fps=30, inference_ms=0, backend='self-test')
                print('PASS: model load, VIDEO inference, increasing timestamps and empty-hand overlay. No camera used.')
            else:
                run_preview(args, detector)
        return 0
    except KeyboardInterrupt:
        print('Interrupted; camera released.')
        return 130
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
