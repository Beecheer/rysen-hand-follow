import asyncio
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import pyrealsense2 as rs
from fastapi import HTTPException
from depth_camera import Camera,make_app,sample_depth


class DepthTests(unittest.TestCase):
    def camera(self):
        c=Camera();c.scale=.001;c.depth=np.full((5,5),1000,np.uint16)
        c.color=np.zeros((5,5,3),np.uint8);c.frame_at=time.perf_counter();c.frame_id=42
        intr=rs.intrinsics();intr.width=intr.height=5;intr.fx=intr.fy=100;intr.ppx=intr.ppy=2
        intr.model=rs.distortion.none;intr.coeffs=[0]*5;c.intr=intr
        return c

    def test_raw_invalid_is_not_replaced_by_neighborhood(self):
        c=self.camera();c.depth[2,2]=0
        m=c.measure(2,2)
        self.assertIsNone(m['raw_z_m']);self.assertIsNone(m['xyz_m'])
        self.assertAlmostEqual(m['patch_z_m'],1.)
        self.assertAlmostEqual(m['patch_valid_ratio'],24/25)

    def test_deprojection_and_stale_guard(self):
        c=self.camera();m=c.measure(2,2)
        np.testing.assert_allclose(m['xyz_m'],[0,0,1])
        c.frame_at=time.perf_counter()-1
        with self.assertRaises(ValueError):c.measure(2,2)

    def test_edges_and_empty_neighborhood(self):
        d=np.zeros((5,5),np.uint16)
        result=sample_depth(d,.001,0,0)
        self.assertIsNone(result['patch_z_m']);self.assertEqual(result['patch_valid_ratio'],0)
        with self.assertRaises(ValueError):sample_depth(d,.001,5,0)

    def test_capture_retains_z16_and_intrinsics(self):
        c=self.camera();c.depth[0,0]=60000
        with tempfile.TemporaryDirectory() as folder,patch('depth_camera.ROOT',Path(folder)):
            saved=Path(c.capture())
            raw=cv2.imread(str(saved/'depth-z16.png'),cv2.IMREAD_UNCHANGED)
            self.assertEqual(raw.dtype,np.uint16);np.testing.assert_array_equal(raw,c.depth)
            metadata=json.loads((saved/'metadata.json').read_text())
            self.assertEqual(metadata['depth_scale_m'],.001);self.assertEqual(metadata['frame_id'],42)
            self.assertEqual(metadata['depth_aligned_to'],'color')

    def test_control_validation_and_shutdown(self):
        c=self.camera();stops=[];app=make_app(c,lambda:stops.append(True))
        endpoint=next(r.endpoint for r in app.routes if getattr(r,'path',None)=='/api/{action}')
        body={}
        async def read():return body
        request=NS(headers={'host':'127.0.0.1:8095','origin':'http://127.0.0.1:8095'},json=read)
        body.update(near=2,far=1)
        with self.assertRaises(HTTPException) as e:asyncio.run(endpoint('range',request))
        self.assertEqual(e.exception.status_code,400);self.assertEqual(c.near,.2)
        body.update(near=.3,far=2)
        asyncio.run(endpoint('range',request));self.assertEqual(c.far,2)
        request.headers['origin']='https://example.invalid'
        with self.assertRaises(HTTPException) as e:asyncio.run(endpoint('stop',request))
        self.assertEqual(e.exception.status_code,403);self.assertFalse(c.stop.is_set())
        request.headers['origin']='http://127.0.0.1:8095'
        asyncio.run(endpoint('stop',request));self.assertTrue(c.stop.is_set());self.assertEqual(stops,[True])


if __name__=='__main__':unittest.main()
