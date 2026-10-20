import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from hand_preview.follow_bounds import build_bounds
from hand_preview.mapping import ASSET_DIR


class FollowBoundsTests(unittest.TestCase):
    def test_thumb_margin_and_mimic_domain(self):
        preview=ASSET_DIR/'apex_hand_left.urdf'
        root=ET.parse(preview).getroot()
        names={}
        for finger_id,(finger,count) in enumerate((('thumb',5),('index',4),('middle',4),('ring',4),('pinky',4))):
            for i in range(count):names[f'left_{finger}_j{i}']=f'f{finger_id}_joint{i}'
        original={n:root.find(f"joint[@name='{n}']/limit").attrib.copy() for n in names}
        for joint in root.findall('joint'):
            if joint.get('name') in names:joint.set('name',names[joint.get('name')])
        with tempfile.TemporaryDirectory() as directory:
            sdk=Path(directory)/'sdk.urdf';ET.ElementTree(root).write(sdk)
            profile=build_bounds(preview,sdk);bounds=profile['bounds']
            override={'margin_deg':2,'bounds':{'left_thumb_j1':[-.1745,.53]}}
            device=build_bounds(preview,sdk,override)
            self.assertAlmostEqual(device['bounds']['left_thumb_j1'][1],.53-math.radians(2))
            with self.assertRaises(ValueError):build_bounds(preview,sdk,{'margin_deg':2,'bounds':{'left_thumb_j1':[2,3]}})
        for i in range(5):
            name=f'left_thumb_j{i}';lo,hi=bounds[name]
            self.assertGreaterEqual(lo,float(original[name]['lower'])+math.radians(2)-1e-9)
            self.assertLessEqual(hi,float(original[name]['upper'])-math.radians(2)+1e-9)
        self.assertEqual(bounds['left_thumb_j3'],bounds['left_thumb_j4'])
        self.assertEqual(profile['thumb_margin_deg'],2)
