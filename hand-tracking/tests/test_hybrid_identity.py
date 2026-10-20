"""Exercise selection before geometry to isolate hand identity decisions."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np

from hand_preview.mapping_app import MappingSession


class GeometryReached(Exception):
    pass


class HybridIdentityTest(unittest.TestCase):
    def session(self, hybrid=True):
        session=MappingSession.__new__(MappingSession)
        session.side_sway=object() if hybrid else None
        session.last_wrist=np.array([.5,.5])
        session.last_selected=1.0
        session.last_left_confirmed=1.0
        session.calibration=NS(enabled=False,collecting=False,reject=Mock())
        session.thumb_calibration=NS(collecting=False,reject=Mock())
        return session

    def result(self, label='Right', wrist=.5):
        points=[NS(x=wrist,y=.5,z=0.) for _ in range(21)]
        return NS(hand_landmarks=[points],hand_world_landmarks=[points],
                  handedness=[[NS(category_name=label,score=1.)]])

    def test_short_label_flip_keeps_selection_and_raw_geometry_chirality(self):
        session=self.session()
        with patch('hand_preview.mapping_app.finger_angles',side_effect=GeometryReached) as geometry:
            with self.assertRaises(GeometryReached):
                session.select_angles(self.result(),False,1.1)
        self.assertEqual(geometry.call_args.args[1],'Right')

    def test_right_hand_cannot_start_or_extend_identity_indefinitely(self):
        session=self.session()
        session.last_left_confirmed=None
        self.assertIsNone(session.select_angles(self.result(),False,1.1)[0])
        session.last_left_confirmed=1.0
        session.last_selected=1.7
        self.assertIsNone(session.select_angles(self.result(),False,1.8)[0])

    def test_distant_hand_collection_and_standard_tracker_remain_strict(self):
        session=self.session()
        self.assertIsNone(session.select_angles(self.result(wrist=.8),False,1.1)[0])
        session.calibration.collecting=True
        self.assertIsNone(session.select_angles(self.result(),False,1.1)[0])
        session=self.session(hybrid=False)
        self.assertIsNone(session.select_angles(self.result(),False,1.1)[0])

    def test_fallback_selection_uses_its_configured_confidence(self):
        result=self.result(label='Left')
        result.handedness[0][0].score=.55
        session=self.session()
        self.assertIsNone(session.select_angles(result,False,1.1)[0])
        session._using_mp_fallback=True
        with patch('hand_preview.mapping_app.finger_angles',side_effect=GeometryReached):
            with self.assertRaises(GeometryReached):
                session.select_angles(result,False,1.1)


if __name__=='__main__':unittest.main()
