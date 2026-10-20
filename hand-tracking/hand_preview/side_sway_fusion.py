"""D435-assisted correction of WiLoR's four proximal-finger abduction.

WiLoR remains the pose prior. MediaPipe supplies same-frame 2D MCP/PIP
locations; aligned D435 depth lifts those observations into the camera frame.
Only the four ``abd`` values are changed. Flexion and thumb remain untouched.
"""
import math
import numpy as np

from .geometry import FINGERS

PALM = (0, 5, 9, 13, 17)


def model_side_zeros(canonical,signs):
    p=np.asarray(canonical,float)
    return {f:math.atan2(float((p[ids[1]]-p[ids[0]])[0]),
                         float((p[ids[1]]-p[ids[0]])[1]))*signs.get(f,1.)
            for f,ids in FINGERS.items()}


def use_model_side_prior(angles,quality,profile,signs):
    """Keep calibrated zero while taking side motion from WiLoR 3D, not 2D yaw."""
    if not profile:return angles
    zero=profile.get('side_sway_model_zero')
    if zero is None:
        sample=profile.get('samples',{}).get('together',{}).get('canonical')
        if sample is None:return angles
        zero=model_side_zeros(sample,signs)
    for finger in FINGERS:
        info=quality.get(finger,{})
        side=info.get('world_side')
        if (angles[finger].get('abd') is not None and info.get('side_observable')
                and side is not None and math.isfinite(side)):
            angles[finger]['abd']=side*signs.get(finger,1.)-zero[finger]+profile['finger_offsets'][finger]['abd']
    return angles


def _unit(v):
    n = float(np.linalg.norm(v))
    if n < 1e-8:
        raise ValueError('掌面坐标退化')
    return v / n


class SideSwayFusion:
    def __init__(self):
        self.report = {'active': False, 'message': '等待同步的 MediaPipe / WiLoR / D435 帧'}
        self.previous = {}

    def reset(self):
        self.previous.clear()

    @staticmethod
    def _depth(depth, scale, uv):
        h, w = depth.shape
        x, y = np.rint(uv).astype(int)
        if x < 3 or y < 3 or x >= w - 3 or y >= h - 3:
            return None
        patch = depth[y-2:y+3, x-2:x+3].astype(float) * scale
        values = patch[(patch >= .18) & (patch <= 1.5)]
        if values.size < 10:
            return None
        med = float(np.median(values))
        mad = float(np.median(np.abs(values-med)))
        if mad > .012 or np.percentile(values, 90)-np.percentile(values, 10) > .05:
            return None
        return med, mad

    @staticmethod
    def _point(rs, intr, uv, z):
        return np.asarray(rs.rs2_deproject_pixel_to_point(intr, np.asarray(uv, float).tolist(), float(z)))

    def correct(self, angles, mp_result, depth, scale, intr, label='Left', swap=False, signs=None, reference=None):
        """Blend depth/image side observations into raw WiLoR angles, in radians."""
        self.report = {'active': True, 'corrected': [], 'held': {}, 'message': '深度弱修正四指侧摆'}
        flexion_weights={}
        for finger in FINGERS:
            mcp=angles.get(finger,{}).get('mcp')
            bend=abs(float(mcp)) if mcp is not None and math.isfinite(mcp) else None
            # A short palm-plane projection cannot distinguish sideways motion
            # from a few pixels/mm of error. Depth must not reopen a side angle
            # that the pose prior has made unobservable during MCP flexion.
            if bend is not None and bend>=math.radians(55):
                angles[finger]['abd']=None
                self.report['held'][finger]='MCP 深弯，近节侧摆不可观测；只保持侧摆'
                flexion_weights[finger]=0.
            else:
                flexion_weights[finger]=1. if bend is None else float(np.clip(
                    (math.radians(55)-bend)/math.radians(20),0.,1.))
        flexion_held=dict(self.report['held'])
        try:
            import pyrealsense2 as rs
            candidates = []
            for image, category in zip(mp_result.hand_landmarks, mp_result.handedness):
                if not category:
                    continue
                name = category[0].category_name
                if swap:
                    name = {'Left':'Right', 'Right':'Left'}.get(name, name)
                if name == label:
                    uv = np.asarray([(p.x*depth.shape[1], p.y*depth.shape[0]) for p in image], float)
                    if uv.shape == (21, 2) and np.isfinite(uv).all():
                        candidates.append(uv)
            if not candidates:
                raise ValueError('MediaPipe 未检出同侧手，沿用 WiLoR 侧摆')
            uv = candidates[0]
            sampled = {}
            points = {}
            for i in PALM:
                value = self._depth(depth, scale, uv[i])
                if value:
                    sampled[i] = value
                    points[i] = self._point(rs, intr, uv[i], value[0])
            if len(points) < 4 or not all(i in points for i in (0, 5, 9, 17)):
                raise ValueError('掌部有效深度不足，沿用 WiLoR 侧摆')
            palm = np.asarray([points[i] for i in points])
            center = np.median(palm, axis=0)
            _, singular, vt = np.linalg.svd(palm-center, full_matrices=False)
            normal = _unit(vt[-1])
            # The palm surface normal points toward the camera.
            if normal @ (-center) < 0:
                normal = -normal
            forward = points[9]-points[0]
            forward = _unit(forward-normal*(forward@normal))
            across = points[5]-points[17]
            across = _unit(across-normal*(across@normal)-forward*(across@forward))
            # Keep basis chirality consistent with the image's middle-finger axis.
            image_forward = uv[9]-uv[0]
            ray_forward = self._point(rs, intr, uv[9], sampled[9][0]) - self._point(rs, intr, uv[0], sampled[0][0])
            if forward @ ray_forward < 0:
                forward = -forward
            plane_quality = float(singular[-1] / max(singular[0], 1e-8))
            if plane_quality > .16:
                raise ValueError('掌面深度平面不稳定，沿用 WiLoR 侧摆')
            for finger, ids in FINGERS.items():
                if flexion_weights[finger]==0.:continue
                field = angles.get(finger, {}).get('abd')
                has_prior=field is not None
                if not has_prior:
                    self.report['held'][finger]='WiLoR 侧摆不可观测；深度不接管'
                    continue
                root, pip = ids[:2]
                root_z = sampled.get(root)
                pip_sample = self._depth(depth, scale, uv[pip])
                if root_z is None or pip_sample is None:
                    self.report['held'][finger] = '近节深度不足'
                    continue
                a = self._point(rs, intr, uv[root], root_z[0])
                b = self._point(rs, intr, uv[pip], pip_sample[0])
                bone = b-a
                # Project out palm-normal motion so MCP flexion does not masquerade as sway.
                bone = bone-normal*(bone@normal)
                if np.linalg.norm(bone) < .006:
                    self.report['held'][finger] = '近节投影太短'
                    continue
                observed = math.atan2(float(bone@across), float(bone@forward))
                if signs and finger in signs:
                    observed *= float(signs[finger])
                raw_observed=observed
                zero=(reference or {}).get(finger)
                if zero is None:
                    self.report['held'][finger]='深度缺少独立侧摆零位，沿用 WiLoR'
                    self.report.setdefault('fingers',{})[finger]={
                        'wilor_deg':None if field is None else math.degrees(field),
                        'depth_image_deg':math.degrees(raw_observed),
                        'output_deg':None if field is None else math.degrees(field),'weight':0.}
                    continue
                # Align each sensor's own neutral before blending; subtracting
                # a WiLoR-only zero from an absolute depth angle biases fingers.
                observed=math.atan2(math.sin(observed-zero['depth']),math.cos(observed-zero['depth']))+zero['wilor']
                delta = math.atan2(math.sin(observed-field), math.cos(observed-field))
                # A large disagreement usually means a mixed/occluding depth patch.
                if abs(delta) > math.radians(25):
                    self.report['held'][finger] = '深度与 WiLoR 先验冲突，保留 WiLoR'
                    continue
                depth_quality = max(0., 1.-(root_z[1]+pip_sample[1])/.024)
                plane_weight = float(np.clip((.16-plane_quality)/.12, 0., 1.))
                agreement=float(np.clip((math.radians(25)-abs(delta))/math.radians(15),0.,1.))
                weight = .35*depth_quality*plane_weight*flexion_weights[finger]*agreement
                if finger in self.previous and abs(math.atan2(math.sin(observed-self.previous[finger]),
                                                              math.cos(observed-self.previous[finger]))) > math.radians(35):
                    weight *= .5
                corrected = field + float(np.clip(weight*delta,-math.radians(4),math.radians(4)))
                angles[finger]['abd'] = corrected
                self.previous[finger] = observed
                self.report['corrected'].append(finger)
                self.report.setdefault('fingers', {})[finger] = {
                    'wilor_deg': math.degrees(field), 'depth_image_deg': math.degrees(raw_observed),
                    'output_deg': math.degrees(corrected), 'weight': weight,
                }
            self.report.update(palm_points=len(points), plane_quality=plane_quality)
        except (ValueError, RuntimeError, IndexError, np.linalg.LinAlgError) as exc:
            self.report = {'active': False, 'corrected': [], 'held': flexion_held,
                           'message': str(exc), 'fallback': '沿用 WiLoR 四指侧摆'}
        return angles
