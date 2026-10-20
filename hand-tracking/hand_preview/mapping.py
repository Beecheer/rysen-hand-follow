"""URDF target mapping, filtering, joint coupling and tracking state."""
from pathlib import Path
from copy import deepcopy
import math
import xml.etree.ElementTree as ET

ASSET_DIR = Path(__file__).resolve().parents[1] / 'assets' / 'apex_left'
FINGER_IDS = {'index': 1, 'middle': 2, 'ring': 3, 'pinky': 4}


class TransientLossHold:
    """Keep the last complete preview target through a brief detector dropout."""
    def __init__(self, max_gap=0.30):
        self.max_gap = float(max_gap)
        self.last_angles = None
        self.last_at = None

    def clear(self):
        self.last_angles = None
        self.last_at = None

    def update(self, angles, acquired_at):
        if angles is not None:
            self.last_angles = deepcopy(angles)
            self.last_at = acquired_at
            return angles, False
        if (self.last_angles is not None and self.last_at is not None
                and 0 <= acquired_at - self.last_at <= self.max_gap):
            return deepcopy(self.last_angles), True
        return None, False


def update_live_tracking(mapper,angles,acquired_at,now,reason):
    """Hold output without ever renewing the capture timestamp of missing data."""
    if angles is None or not 0<=now-acquired_at<=.2:
        mapper.tick(now)
        return (not mapper.paused and mapper.last_good is not None
                and mapper.state=='TRACKING' and now-mapper.last_good<mapper.tracking_grace)
    mapper.update(angles,acquired_at,now,reason)
    return False


def load_joint_specs():
    specs = {}
    for node in ET.parse(ASSET_DIR / 'apex_hand_left.urdf').getroot().findall('joint'):
        if node.get('type') != 'revolute':
            continue
        limit, mimic = node.find('limit'), node.find('mimic')
        specs[node.get('name')] = {
            'lower': float(limit.get('lower')), 'upper': float(limit.get('upper')),
            'axis': [float(x) for x in node.find('axis').get('xyz').split()],
            'mimic': None if mimic is None else {'joint': mimic.get('joint'),
                'multiplier': float(mimic.get('multiplier', '1')), 'offset': float(mimic.get('offset', '0'))},
        }
    if len(specs) != 21:
        raise ValueError('Expected official 21-joint model')
    return specs


class PreviewMapper:
    TAU = 0.08
    MAX_SPEED = math.radians(90)
    MAX_ACCEL = math.radians(360)

    def __init__(self, specs=None, responsive=False, tracking_grace=0.2):
        self.tracking_grace=float(tracking_grace)
        if not 0.2 <= self.tracking_grace < 0.5:
            raise ValueError('Tracking grace must be at least 200 ms and below loss timeout')
        self.responsive = responsive
        self.side_previous = {}
        self.side_pending = {}
        self.specs = specs or load_joint_specs()
        self.positions = {n: max(s['lower'], min(0.0, s['upper'])) for n, s in self.specs.items()}
        self.velocities = {n: 0.0 for n in self.positions}
        self.filtered = dict(self.positions)
        self.last_step = self.last_good = None
        self.state, self.reason = 'WAITING', 'Show your left hand'
        self.raw = None
        self.stable_frames = 0
        self.paused = False
        self.loss_paused = False
        self.bounds = {}
        self.held_joints = []
        for finger in (*FINGER_IDS, 'thumb'):
            for suffix in (range(4) if finger == 'thumb' else range(3)):
                name = f'left_{finger}_j{suffix}'
                spec = self.specs[name]
                if finger == 'thumb':
                    lo, hi = spec['lower'], spec['upper']
                elif suffix == 0:
                    lo, hi = max(math.radians(-25),spec['lower']), min(math.radians(25),spec['upper'])
                else:
                    lo, hi = max(0.0, spec['lower']), min(math.radians(65), spec['upper'])
                for child in self.specs.values():
                    m = child['mimic']
                    if m and m['joint'] == name:
                        if m['multiplier'] == 0:
                            if not child['lower'] <= m['offset'] <= child['upper']:
                                raise ValueError('Impossible mimic limits')
                        else:
                            domain = sorted(((child['lower'] - m['offset']) / m['multiplier'],
                                             (child['upper'] - m['offset']) / m['multiplier']))
                            lo, hi = max(lo, domain[0]), min(hi, domain[1])
                if lo > hi:
                    raise ValueError(f'Empty joint domain: {name}')
                self.bounds[name] = (lo, hi)

    def _reset_motion(self):
        self.side_previous.clear()
        self.side_pending.clear()
        self.velocities = {n: 0.0 for n in self.positions}
        self.filtered = dict(self.positions)
        self.last_step = None

    def pause(self):
        self.paused = True
        self.loss_paused = False
        self.state, self.reason = 'PAUSED', 'Paused; click Resume to reacquire'
        self.stable_frames = 0
        self._reset_motion()

    def resume(self):
        self.paused = False
        self.loss_paused = False
        self.state, self.reason = 'WAITING', 'Reacquiring left hand'
        self.last_good = None
        self.stable_frames = 0
        self.raw = None
        self._reset_motion()

    def resume_after_loss(self):
        """Recover an automatic lost-hand pause; explicit user pauses stay latched."""
        if not self.loss_paused:
            return False
        self.resume()
        return True

    def tick(self, now):
        if self.paused or self.last_good is None:
            return
        age = now - self.last_good
        if age >= 0.5:
            self.pause()
            self.loss_paused = True
            self.reason = 'Left hand lost >500 ms; click Resume'
        elif age >= self.tracking_grace:
            self.state, self.reason = 'HOLD', 'Stale landmarks; holding target'
            self._reset_motion()

    def update(self, angles, acquired_at, now, reason='No valid left hand'):
        stale=not 0 <= now - acquired_at <= 0.2
        if angles is None or stale:
            self.tick(now)
            self.raw = None
            self.stable_frames = 0
            if not self.paused:
                self.reason = 'Frame processing exceeded 200 ms' if stale else reason
            return
        if any(v is not None and not math.isfinite(v) for finger in angles.values() for v in finger.values()):
            self.tick(now)
            self.raw = None
            self.stable_frames = 0
            return
        # Frame continuity is measured capture-to-capture. Comparing this
        # completion time to the previous capture double-counted latency and
        # repeatedly erased acquisition at ~10 FPS / 100 ms processing.
        # Keep the 500 ms latch: a returning hand cannot erase a long loss.
        if self.last_good is not None:
            if now-self.last_good>=.5:self.tick(now)
            elif acquired_at-self.last_good>=self.tracking_grace:
                self.stable_frames=0
                self._reset_motion()
        self.raw = angles
        if self.paused:
            return
        self.last_good = acquired_at
        self.stable_frames += 1
        if self.stable_frames < 3:
            self.state, self.reason = 'ACQUIRING', 'Waiting for 3 valid frames'
            return
        self.state, self.reason = 'TRACKING', 'Direct joint-angle preview'
        dt = 1 / 30 if self.last_step is None else min(0.1, max(0.001, now - self.last_step))
        self.last_step = now
        alpha = 1 - math.exp(-dt / self.TAU)
        if self.responsive:
            # One common interpolation factor preserves simultaneous finger motion.
            # Smooth small estimation noise; follow deliberate movement promptly.
            errors = [abs(v-self.positions[f'left_{f}_j{s}'])
                      for f in FINGER_IDS
                      for k,s in ([(f'j{i}',i) for i in range(4)] if f=='thumb' else [('abd',0),('mcp',1),('pip',2)])
                      if (v := angles.get(f,{}).get(k)) is not None]
            activity = min(1.,max(errors,default=0.)/math.radians(12))
            alpha = 1-math.exp(-dt/(.09-.07*activity))
        # Thumb noise must not inherit the fast response of moving fingers.
        thumb_errors = [abs(max(self.bounds[f'left_thumb_j{i}'][0],
                           min(self.bounds[f'left_thumb_j{i}'][1],v))-
                           self.positions[f'left_thumb_j{i}']) for i in range(4)
                        if (v := angles.get('thumb',{}).get(f'j{i}')) is not None]
        thumb_activity=min(1.,max(thumb_errors,default=0.)/math.radians(12))
        thumb_alpha=1-math.exp(-dt/(.10-.075*thumb_activity))
        self.held_joints = []
        for finger in (*FINGER_IDS, 'thumb'):
            items = [(f'j{i}',i) for i in range(4)] if finger=='thumb' else [('abd',0),('mcp',1),('pip',2)]
            for key, suffix in items:
                name = f'left_{finger}_j{suffix}'
                value = angles.get(finger,{}).get(key)
                if value is None:
                    self.side_pending.pop(name,None)
                    self.held_joints.append(name)
                    self.filtered[name] = self.positions[name]
                    self.velocities[name] = 0.0
                    continue
                if self.responsive and key == 'abd':
                    previous = self.side_previous.get(name,value)
                    pending = self.side_pending.pop(name,None)
                    # A large lateral jump must persist for a second camera frame.
                    # Never convert a held value into a new accepted observation.
                    if abs(value-previous)>math.radians(12) and (pending is None or abs(value-pending)>math.radians(6)):
                        self.side_pending[name]=value
                        value=previous
                    else:
                        self.side_previous[name]=value
                    self.side_previous.setdefault(name,value)
                lo, hi = self.bounds[name]
                target = max(lo, min(hi, value))
                if self.responsive:
                    old=self.positions[name]
                    weight=thumb_alpha if finger=='thumb' else alpha
                    if finger=='thumb' and abs(target-old)<math.radians(.4):
                        target=old
                    self.positions[name]=old+weight*(target-old)
                    self.filtered[name]=self.positions[name]
                    self.velocities[name]=(self.positions[name]-old)/dt
                    continue
                self.filtered[name] += alpha * (target - self.filtered[name])
                error = self.filtered[name] - self.positions[name]
                speed = min(self.MAX_SPEED, abs(error) / dt, math.sqrt(2 * self.MAX_ACCEL * abs(error)))
                desired = math.copysign(speed, error) if error else 0.0
                previous = self.velocities[name]
                velocity = max(previous - self.MAX_ACCEL * dt, min(previous + self.MAX_ACCEL * dt, desired))
                old = self.positions[name]
                self.positions[name] = max(lo, min(hi, old + velocity * dt))
                self.velocities[name] = (self.positions[name] - old) / dt
        for name, spec in self.specs.items():
            m = spec['mimic']
            if m:
                self.positions[name] = self.positions[m['joint']] * m['multiplier'] + m['offset']
        if any(not s['lower'] - 1e-9 <= self.positions[n] <= s['upper'] + 1e-9 for n, s in self.specs.items()):
            raise ValueError('Joint target outside model limits')

    def snapshot(self, now):
        self.tick(now)
        age = None if self.last_good is None else max(0, (now - self.last_good) * 1000)
        return {'state': self.state, 'reason': self.reason,
                'valid': self.state == 'TRACKING' and age is not None and age < self.tracking_grace*1000,
                'age_ms': age, 'raw_angles': self.raw if age is not None and age < self.tracking_grace*1000 else None,
                'joints': dict(self.positions), 'held_joints': list(self.held_joints),
                'units': 'radians', 'robot_connected': False}
