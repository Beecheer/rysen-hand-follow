from types import SimpleNamespace as NS
import unittest

import numpy as np
import pyrealsense2 as rs

from hand_preview.geometry import FINGERS, finger_angles
from hand_preview.side_sway_fusion import SideSwayFusion, use_model_side_prior


class SideSwayFusionTests(unittest.TestCase):
    def setUp(self):
        self.intr=rs.intrinsics();self.intr.width=640;self.intr.height=480
        self.intr.fx=600.;self.intr.fy=600.;self.intr.ppx=320.;self.intr.ppy=240.
        self.intr.model=rs.distortion.none;self.intr.coeffs=[0.]*5
        self.depth=np.full((480,640),600,dtype=np.uint16)
        self.reference={f:{'wilor':0.,'depth':0.} for f in FINGERS}
        pixels=np.zeros((21,2),float)
        pixels[0]=[320,360];pixels[5]=[270,270];pixels[9]=[310,245]
        pixels[13]=[345,252];pixels[17]=[380,280]
        for finger,(root,pip,dip,tip) in FINGERS.items():
            x,y=pixels[root]
            pixels[pip]=[x+8,y-42];pixels[dip]=[x+13,y-76];pixels[tip]=[x+16,y-103]
        self.result=NS(hand_landmarks=[[NS(x=x/640,y=y/480) for x,y in pixels]],
                       handedness=[[NS(category_name='Left')]])

    def test_corrects_only_four_abductions_and_keeps_flexion_and_thumb(self):
        angles={name:{'abd':.05,'mcp':.2,'pip':.3,'dip':.1} for name in FINGERS}
        angles['thumb']={'j0':.1,'j1':.2,'j2':.3,'j3':.4}
        original={name:part.copy() for name,part in angles.items()}
        fusion=SideSwayFusion()
        result=fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=self.reference)
        self.assertEqual(set(fusion.report['corrected']),set(FINGERS))
        for name in FINGERS:
            self.assertNotAlmostEqual(result[name]['abd'],original[name]['abd'])
            for joint in ('mcp','pip','dip'):
                self.assertEqual(result[name][joint],original[name][joint])
        self.assertEqual(result['thumb'],original['thumb'])

    def test_missing_mediapipe_hand_falls_back_to_wilor(self):
        angles={name:{'abd':.25} for name in FINGERS}
        result=SideSwayFusion().correct(angles,NS(hand_landmarks=[],handedness=[]),
                                       self.depth,.001,self.intr)
        self.assertEqual([result[name]['abd'] for name in FINGERS],[.25]*4)

    def test_depth_does_not_reopen_an_unobservable_wilor_side(self):
        angles={name:{'abd':None,'mcp':.2,'pip':.3,'dip':.1} for name in FINGERS}
        fusion=SideSwayFusion()
        result=fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=self.reference)
        self.assertEqual(fusion.report['corrected'],[])
        for name in FINGERS:self.assertIsNone(result[name]['abd'])

    def test_depth_and_wilor_share_forward_first_palm_axes(self):
        points=[rs.rs2_deproject_pixel_to_point(self.intr,[p.x*640,p.y*480],.6)
                for p in self.result.hand_landmarks[0]]
        expected=finger_angles(points,'Left')
        fusion=SideSwayFusion()
        fusion.correct({f:{'abd':.05} for f in FINGERS},self.result,self.depth,.001,self.intr)
        for finger in FINGERS:
            self.assertAlmostEqual(fusion.report['fingers'][finger]['depth_image_deg'],
                                   np.degrees(expected[finger]['abd']),places=6)

    def test_deep_mcp_holds_only_side_even_if_depth_looks_available(self):
        for prior in (None,.35):
            angles={f:{'abd':.05,'mcp':.2,'pip':.3,'dip':.1} for f in FINGERS}
            angles['middle'].update(abd=prior,mcp=1.2,pip=.8)
            fusion=SideSwayFusion()
            fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=self.reference)
            self.assertIsNone(angles['middle']['abd'])
            self.assertEqual(angles['middle']['mcp'],1.2)
            self.assertEqual(angles['middle']['pip'],.8)
            self.assertIn('middle',fusion.report['held'])
            self.assertIn('ring',fusion.report['corrected'])
            angles['middle'].update(abd=.25,mcp=.2)
            fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=self.reference)
            self.assertIsNotNone(angles['middle']['abd'])

    def test_deep_mcp_side_stays_held_when_depth_or_detector_is_missing(self):
        angles={f:{'abd':.25,'mcp':.2} for f in FINGERS}
        angles['middle']['mcp']=1.2
        fusion=SideSwayFusion()
        fusion.correct(angles,NS(hand_landmarks=[],handedness=[]),self.depth,.001,self.intr)
        self.assertIsNone(angles['middle']['abd'])
        self.assertIn('middle',fusion.report['held'])

    def test_each_source_neutral_is_aligned_before_blend(self):
        fusion=SideSwayFusion()
        angles={f:{'abd':.2,'mcp':.1} for f in FINGERS}
        fusion.correct(angles,self.result,self.depth,.001,self.intr)
        self.assertEqual([angles[f]['abd'] for f in FINGERS],[.2]*4)
        reference={f:{'wilor':.2,'depth':np.radians(fusion.report['fingers'][f]['depth_image_deg'])}
                   for f in FINGERS}
        fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=reference)
        for finger in FINGERS:self.assertAlmostEqual(angles[finger]['abd']-.2,0.,places=8)

    def test_image_yaw_alone_cannot_move_model_side_prior(self):
        profile={'side_sway_model_zero':{f:.1 for f in FINGERS},
                 'finger_offsets':{f:{'abd':.2} for f in FINGERS}}
        quality={f:{'world_side':.12,'side_observable':True} for f in FINGERS}
        for image_side in (.2,-.4):
            angles={f:{'abd':image_side,'mcp':.4,'pip':.3} for f in FINGERS}
            use_model_side_prior(angles,quality,profile,{f:1 for f in FINGERS})
            for finger in FINGERS:
                self.assertAlmostEqual(angles[finger]['abd'],.22)
                self.assertEqual(angles[finger]['mcp'],.4)

    def test_depth_correction_is_bounded_and_rejects_large_disagreement(self):
        fusion=SideSwayFusion()
        angles={f:{'abd':.05,'mcp':.1} for f in FINGERS}
        fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=self.reference)
        for finger in FINGERS:self.assertLessEqual(abs(angles[finger]['abd']-.05),np.radians(4))
        angles={f:{'abd':1.,'mcp':.1} for f in FINGERS}
        fusion.correct(angles,self.result,self.depth,.001,self.intr,reference=self.reference)
        for finger in FINGERS:self.assertEqual(angles[finger]['abd'],1.)


if __name__=='__main__':unittest.main()
