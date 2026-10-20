import math
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from hand_preview.geometry import FINGERS, finger_angles, unit
from hand_preview.mapping import ASSET_DIR, PreviewMapper, TransientLossHold, update_live_tracking


def make_hand(mcp_degrees=0, pip_degrees=0):
    points = np.zeros((21, 3))
    points[1:5] = [[.03, .01, 0], [.045, .02, 0], [.055, .03, 0], [.06, .04, 0]]
    roots = [[.03, .06, 0], [0, .07, 0], [-.01, .065, 0], [-.03, .05, 0]]
    for indices, root in zip(FINGERS.values(), roots):
        mcp, pip, dip, tip = indices
        points[mcp] = root
        reference = unit(points[mcp])
        for current, previous, angle, length in (
            (pip, mcp, mcp_degrees, .03), (dip, pip, mcp_degrees+pip_degrees, .02),
            (tip, dip, mcp_degrees+pip_degrees*2, .015)):
            theta=math.radians(angle)
            direction=reference*math.cos(theta)+np.array([0,0,-1])*math.sin(theta)
            points[current]=points[previous]+length*direction
    return points


def pose(mcp=45, pip=55):
    return {name: {'mcp': math.radians(mcp), 'pip': math.radians(pip), 'dip': 0.0} for name in FINGERS}


class GeometryTests(unittest.TestCase):
    def test_real_mediapipe_folded_fingers_flex_positive(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/thumb_up_world.json').read_text())
        angles=finger_angles(fixture['world'],fixture['label'])
        for finger in FINGERS:
            self.assertGreater(angles[finger]['mcp'],math.radians(15))
            self.assertGreater(angles[finger]['pip'],math.radians(60))

    def test_known_angles(self):
        for mcp,pip in [(0,0),(30,45),(60,70),(-10,20)]:
            for angles in finger_angles(make_hand(mcp,pip)).values():
                self.assertAlmostEqual(angles['mcp'],math.radians(mcp),places=7)
                self.assertAlmostEqual(angles['pip'],math.radians(pip),places=7)

    def test_rigid_motion_invariance(self):
        points=make_hand(35,50)
        matrix,_=np.linalg.qr(np.random.default_rng(42).normal(size=(3,3)))
        if np.linalg.det(matrix)<0: matrix[:,0]*=-1
        moved=points@matrix.T+np.array([.2,-.3,1])
        expected,actual=finger_angles(points),finger_angles(moved)
        for name in FINGERS:
            for joint in ('mcp','pip','dip'):
                self.assertAlmostEqual(expected[name][joint],actual[name][joint],places=7)

    def test_anatomical_left_right_reflection(self):
        left=make_hand(40,55)
        right=left*np.array([-1,1,1])
        for finger in FINGERS:
            self.assertAlmostEqual(finger_angles(left,'Left')[finger]['mcp'],
                                   finger_angles(right,'Right')[finger]['mcp'])

    def test_bad_geometry(self):
        for points in [np.zeros((21,3)),np.full((21,3),np.nan),np.zeros((20,3))]:
            with self.assertRaises(ValueError): finger_angles(points)


class MapperTests(unittest.TestCase):
    def test_fresh_slow_frames_acquire_despite_processing_delay(self):
        mapper=PreviewMapper()
        for i in range(5):
            captured=i*.1
            mapper.update(pose(),captured,captured+.11)
            if i>=2:
                self.assertEqual(mapper.state,'TRACKING')
                self.assertTrue(mapper.snapshot(captured+.15)['valid'])
            # Results still expire at 200 ms; a short HOLD is not a missing
            # camera observation and must not erase valid-frame continuity.
            self.assertFalse(mapper.snapshot(captured+.205)['valid'])

    def test_large_capture_gaps_do_not_count_as_continuous(self):
        mapper=PreviewMapper()
        for i in range(4):mapper.update(pose(),i*.25,i*.25+.01)
        self.assertEqual(mapper.state,'ACQUIRING')

    def test_responsive_flexion_is_not_slew_limited(self):
        mapper=PreviewMapper(responsive=True)
        for i in range(4):mapper.update(pose(0,0),i/30,i/30)
        mapper.update(pose(65,65),4/30,4/30)
        self.assertGreater(mapper.positions['left_index_j1'],math.radians(40))
        self.assertAlmostEqual(mapper.positions['left_index_j1'],mapper.positions['left_ring_j2'])

    def test_responsive_side_spike_needs_confirmation(self):
        mapper=PreviewMapper(responsive=True)
        def update(deg,i):
            angles=pose(0,0)
            for finger in FINGERS:angles[finger]['abd']=math.radians(deg)
            mapper.update(angles,i/30,i/30)
        for i in range(4):update(0,i)
        update(20,4)
        self.assertEqual(mapper.positions['left_index_j0'],0)
        update(0,5)
        self.assertEqual(mapper.positions['left_index_j0'],0)
        update(20,6);update(20,7)
        self.assertGreater(mapper.positions['left_index_j0'],math.radians(10))

    def test_mesh_paths_complete(self):
        root=ET.parse(ASSET_DIR/'apex_hand_left.urdf').getroot()
        for mesh in root.findall('.//mesh'):
            self.assertTrue((ASSET_DIR/mesh.get('filename')).is_file(),mesh.get('filename'))

    def test_limits_coupling_and_speed(self):
        mapper=PreviewMapper()
        for i in range(150):
            previous=dict(mapper.positions)
            now=i/30
            mapper.update(pose(150,150),now,now)
            for name,spec in mapper.specs.items():
                q=mapper.positions[name]
                self.assertTrue(spec['lower']<=q<=spec['upper'])
                m=spec['mimic']
                if m: self.assertAlmostEqual(q,mapper.positions[m['joint']]*m['multiplier']+m['offset'])
            for name in mapper.bounds:
                self.assertLessEqual(abs(mapper.positions[name]-previous[name]),mapper.MAX_SPEED/30+1e-8)
        self.assertAlmostEqual(mapper.positions['left_index_j1'],math.radians(65),places=3)
        self.assertTrue(all(mapper.positions[f'left_thumb_j{i}']==0 for i in range(5)))

    def test_lost_hand_requires_manual_resume(self):
        mapper=PreviewMapper()
        for t in [0,.033,.066]:mapper.update(pose(),t,t)
        self.assertEqual(mapper.snapshot(.07)['state'],'TRACKING')
        self.assertEqual(mapper.snapshot(.3)['state'],'HOLD')
        held=dict(mapper.positions)
        mapper.update(pose(60,60),.7,.7)
        self.assertEqual(mapper.state,'PAUSED')
        self.assertEqual(held,mapper.positions)
        mapper.resume()
        for t in [.8,.833,.866]:mapper.update(pose(),t,t)
        self.assertEqual(mapper.state,'TRACKING')

    def test_preview_can_recover_automatic_loss_without_clearing_manual_pause(self):
        mapper=PreviewMapper()
        for t in [0,.033,.066]:mapper.update(pose(),t,t)
        mapper.tick(.6)
        self.assertTrue(mapper.paused);self.assertTrue(mapper.loss_paused)
        self.assertTrue(mapper.resume_after_loss())
        self.assertFalse(mapper.paused)
        mapper.pause()
        self.assertFalse(mapper.resume_after_loss())
        self.assertTrue(mapper.paused)

    def test_transient_dropout_holds_last_pose_only_for_short_gap(self):
        hold=TransientLossHold(max_gap=.30)
        target=pose(15,25)
        actual,held=hold.update(target,1.)
        self.assertIs(actual,target)
        self.assertFalse(held)
        actual,held=hold.update(None,1.2)
        self.assertTrue(held)
        self.assertEqual(actual,target)
        actual,held=hold.update(None,1.31)
        self.assertIsNone(actual)
        self.assertFalse(held)
        hold.clear()
        self.assertIsNone(hold.update(None,1.32)[0])

    def test_stale_and_nonfinite_not_consumed(self):
        mapper=PreviewMapper()
        mapper.update(pose(),0,1)
        self.assertIsNone(mapper.last_good)
        self.assertEqual(mapper.reason,'Frame processing exceeded 200 ms')
        bad=pose();bad['index']['mcp']=float('nan')
        mapper.update(bad,1,1)
        self.assertIsNone(mapper.last_good)

    def test_preview_grace_tolerates_inference_interval_but_rejects_old_input(self):
        mapper=PreviewMapper(tracking_grace=.4)
        for t in [0,.1,.2]:mapper.update(pose(),t,t+.08)
        self.assertTrue(mapper.snapshot(.45)['valid'])
        mapper.update(pose(),.46,.54)
        self.assertEqual(mapper.state,'TRACKING')
        self.assertEqual(mapper.stable_frames,4)
        mapper.update(pose(),.5,.75)
        self.assertEqual(mapper.last_good,.46)
        self.assertEqual(mapper.reason,'Frame processing exceeded 200 ms')

    def test_live_dropouts_and_slow_frames_never_refresh_capture_age(self):
        mapper=PreviewMapper(tracking_grace=.4)
        for t in [0,.05,.1]:mapper.update(pose(),t,t)
        original=mapper.last_good
        self.assertTrue(update_live_tracking(mapper,None,.2,.25,'No hand detected'))
        self.assertEqual(mapper.last_good,original)
        self.assertTrue(update_live_tracking(mapper,pose(60,60),.1,.35,'slow'))
        self.assertEqual(mapper.last_good,original)
        self.assertFalse(mapper.snapshot(.51)['valid'])
        self.assertFalse(update_live_tracking(mapper,None,.55,.6,'No hand detected'))
        self.assertTrue(mapper.paused)


if __name__=='__main__': unittest.main()
