import math
import unittest
import numpy as np
from hand_preview.thumb_filter import ThumbObservationFilter
from hand_preview.mapping import PreviewMapper
from test_mapping import pose


class ThumbFilterTests(unittest.TestCase):
    def test_stationary_noise_is_attenuated_in_bones_and_calibrated_tip(self):
        f=ThumbObservationFilter()
        base=np.array([[0,0,0],[.02,0,0],[.04,0,0],[.06,0,0]])
        values=[]
        for i in range(100):
            delta=.001*(-1)**i
            target=base.copy();target[1:,1]=delta
            cues={'tip':target[-1].copy(),'strength':.5+delta*100,'finger':None}
            filtered,contact,_=f.update(target,cues,i/15)
            self.assertAlmostEqual(cues['tip'][1],delta)
            if i>20:values.append([filtered[-1,1],contact['tip'][1],contact['strength']-.5])
        self.assertTrue(np.all(np.std(values,axis=0)<np.array([.0004,.0004,.04])))

    def test_outlier_and_persistent_step(self):
        f=ThumbObservationFilter();base=np.zeros((4,3));jump=base.copy();jump[-1,0]=.02
        f.update(base,None,0)
        held,_,_=f.update(jump,None,.07)
        self.assertEqual(held[-1,0],0)
        recovered,_,_=f.update(base,None,.14)
        self.assertEqual(recovered[-1,0],0)
        f.update(jump,None,.21)
        accepted,_,activity=f.update(jump,None,.28)
        self.assertGreater(accepted[-1,0],.016)
        self.assertEqual(activity,1)
        next_value,_,_=f.update(jump,None,.35)
        self.assertGreater(next_value[-1,0],accepted[-1,0])

    def test_gap_and_reset_do_not_reuse_old_observation(self):
        f=ThumbObservationFilter();a=np.zeros((4,3));b=np.ones((4,3))*.01
        f.update(a,None,0)
        np.testing.assert_array_equal(f.update(b,None,1)[0],b)
        f.reset()
        np.testing.assert_array_equal(f.update(a,None,1.05)[0],a)

    def test_small_single_frame_spike_is_removed_and_invalid_input_does_not_poison_state(self):
        f=ThumbObservationFilter();base=np.zeros((4,3))
        f.update(base,None,0)
        spike=base.copy();spike[-1,0]=.004
        np.testing.assert_array_equal(f.update(spike,None,.07)[0],base)
        np.testing.assert_array_equal(f.update(base,None,.14)[0],base)
        with self.assertRaises(ValueError):f.update(np.full((4,3),np.nan),None,.21)
        np.testing.assert_array_equal(f.update(base,None,.21)[0],base)

    def test_continuous_deliberate_motion_is_not_held_as_noise(self):
        f=ThumbObservationFilter();base=np.zeros((4,3));values=[]
        for i in range(21):
            target=base.copy();target[1:,0]=i*.003
            filtered,_,activity=f.update(target,None,i/15)
            values.append(filtered[-1,0])
        self.assertGreater(activity,.5)
        self.assertGreater(values[-1],.052)
        self.assertGreater(values[-1]-values[-2],.0025)

    def test_thumb_filter_is_independent_of_four_finger_motion(self):
        still=PreviewMapper(responsive=True);moving=PreviewMapper(responsive=True)
        for i in range(40):
            for mapper,bend in ((still,0),(moving,65 if i%2 else 0)):
                angles=pose(bend,bend)
                angles['thumb']={f'j{j}':math.radians(20+(-1)**i) for j in range(4)}
                mapper.update(angles,i/15,i/15)
            for j in range(5):
                self.assertAlmostEqual(still.positions[f'left_thumb_j{j}'],moving.positions[f'left_thumb_j{j}'])
        angles=pose();angles['thumb']={f'j{j}':math.radians(45) for j in range(4)}
        moving.update(angles,40/15,40/15)
        self.assertGreater(moving.positions['left_thumb_j2'],math.radians(40))
        self.assertEqual(moving.positions['left_thumb_j3'],moving.positions['left_thumb_j4'])


if __name__=='__main__':unittest.main()
