import math
import unittest
import numpy as np
from hand_preview.finger_filter import FingerAngleFilter
from hand_preview.kinematics import RobotKinematics,ThumbRetargeter
from hand_preview.mapping import PreviewMapper


class DepthStabilityTests(unittest.TestCase):
    def test_finger_noise_is_reduced_and_coherent_motion_responds(self):
        f=FingerAngleFilter();values=[]
        for i in range(80):
            a={'index':{'mcp':.3+(-1)**i*.06,'pip':.5,'abd':0.,'dip':.2}}
            out=f.update(a,i*.04)
            if i>20:values.append(out['index']['mcp'])
        self.assertLess(np.std(values),.02)
        for i in range(12):out=f.update({'index':{'mcp':.3+i*.05,'pip':.5}},3.2+i*.04)
        self.assertGreater(out['index']['mcp'],.72)
        f.reset();self.assertEqual(f.update({'index':{'mcp':.9}},5.)['index']['mcp'],.9)

    def test_thumb_static_input_cannot_switch_to_distant_ik_branch(self):
        robot=RobotKinematics();mapper=PreviewMapper();solver=ThumbRetargeter(robot,mapper.specs,stable_continuity=True)
        p=robot.thumb_points([.6,.2,.4,.2]);target=(p-p[0])@robot.basis
        solver.fit_target(target)
        for i in range(20):
            previous=solver.previous.copy()
            cues={'tip':target[-1]+np.array([.008*(-1)**i,0,0]),'strength':.8}
            q=solver.fit_target(target,cues=cues)
            self.assertLessEqual(np.max(abs(solver.previous-previous)),math.radians(3.001))
            self.assertTrue(np.all(solver.previous>=solver.lower));self.assertTrue(np.all(solver.previous<=solver.upper))
        p=robot.thumb_points([1.,.3,.9,.7]);moving=(p-p[0])@robot.basis
        solver.fit_target(moving)
        self.assertGreater(solver.diagnostics['ik_continuity']['trust_radius_deg'],10)
