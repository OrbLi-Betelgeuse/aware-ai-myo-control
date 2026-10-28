import _paths  # Resolve experimental modules for direct test execution.
"""Synthetic sample generator through actual 3.2a processing; no network/hardware."""
import contextlib
import io
import unittest
from unittest.mock import patch
import numpy as np
import simulate_3_2a as sim

class SyntheticChecks(unittest.TestCase):
    def test_neutral_stream_countdown_then_creep(self):
        c=sim.controller
        with patch.object(c.pylsl,'local_clock',return_value=1000.) as lsl, patch.object(c.time,'monotonic',return_value=100.) as clock, contextlib.redirect_stdout(io.StringIO()):
            c.reset_control();c.LOG=None
            c.CAL=dict(zero=1.,full=10.,up0=np.array([0.,0.,1.]),axis=np.array([1.,0.,0.]),
                       tilt_axis=np.array([0.,1.,0.]),reach_left=.5,reach_right=-.5,reach_up=.5,rms_relax=np.zeros(8))
            c.EMG=sim.SyntheticInlet('emg');c.IMU=sim.SyntheticInlet('imu')
            first_prepare=None;first_motion=None;phases=set()
            for i in range(1,101):
                now=100+i*.05;clock.return_value=now;lsl.return_value=1000+i*.05
                state=dict(type='state',t=i*.05,_received_at=now,status='ok',pose=dict(x=0.,y=0.,yaw=0.),
                           vel=dict(v=0.,w=0.),cmd_applied=dict(v=0.,w=0.),collisions=0,course=None,
                           scan=dict(ranges=[10.]*180,angle_min=-.9,angle_inc=.01,range_max=10.))
                v,w=c.step(state,c.my_input(),[])
                phases.add((c.PREPARE_UI['phase'],c.PREPARE_UI['remaining']))
                if c.PREPARE_UI['phase']=='countdown' and first_prepare is None:first_prepare=now
                if v>0 and first_motion is None:first_motion=now
                self.assertEqual(w,0.)
            self.assertIsNotNone(first_prepare);self.assertIsNotNone(first_motion)
            self.assertGreaterEqual(first_motion-first_prepare,3.)
            for n in [3,2,1]:self.assertIn(('countdown',n),phases)
            self.assertAlmostEqual(v,.4)
            c.reset_control()

if __name__=='__main__':unittest.main()
