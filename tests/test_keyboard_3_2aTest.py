import _paths  # Resolve experimental modules for direct test execution.
"""Keyboard adapter regression through synthetic sensors and unchanged 3.2a."""
import contextlib
import io
import unittest
from unittest.mock import patch
import numpy as np
import simulate_3_2aTest as tool

class KeyboardChecks(unittest.TestCase):
    def setUp(self):
        self.c=tool.c;self.c.reset_control();self.c.LOG=None
        tool.keyboard=tool.KeyboardState()
        self.c.CAL=dict(zero=1.,full=10.,up0=np.array([0.,0.,1.]),axis=np.array([1.,0.,0.]),
            tilt_axis=np.array([0.,1.,0.]),reach_left=.5,reach_right=-.5,reach_up=.5,rms_relax=np.zeros(8))
        self.mono=patch.object(tool.time,'monotonic',return_value=100.).start()
        self.lsl=patch.object(self.c.pylsl,'local_clock',return_value=1000.).start()
        self.c.EMG=tool.KeyboardInlet('emg');self.c.IMU=tool.KeyboardInlet('imu')
        self.i=0;self.seq=0
        self.output=contextlib.redirect_stdout(io.StringIO());self.output.__enter__()
    def tearDown(self):patch.stopall();self.output.__exit__(None,None,None)
    def tick(self, keys=(), enabled=True, distance=10., send=True):
        self.i+=1;now=100+self.i*.05;self.mono.return_value=now;self.lsl.return_value=1000+self.i*.05
        if send:
            self.seq+=1;tool.keyboard.accept(dict(type='keys',seq=self.seq,keys=list(keys),enabled=enabled))
        ranges=[10.]*180;ranges[90]=distance
        state=dict(type='state',t=self.i*.05,_received_at=now,status='ok',pose=dict(x=0.,y=0.,yaw=0.),
            vel=dict(v=0.,w=0.),cmd_applied=dict(v=0.,w=0.),collisions=0,course=None,
            scan=dict(ranges=ranges,angle_min=-.9,angle_inc=.01,range_max=10.))
        inp=tool.keyboard_input()
        return self.c.step(state,inp,[])
    def prepare(self):
        for _ in range(100):v,w=self.tick()
        self.assertAlmostEqual(v,.4);self.assertEqual(w,0.)
    def test_resume_after_long_pause_has_no_stale_backlog(self):
        self.prepare()
        for _ in range(100): self.tick(enabled=False)
        for _ in range(100):
            self.tick()
            self.assertNotIn('Old/future', self.c.LAST_FAULT)
        self.assertTrue(self.c.PREPARE_DONE)

    def test_ready_has_minimum_dwell(self):
        for _ in range(15):
            self.assertEqual(self.tick(), (0,0))
            self.assertIsNone(self.c.PREPARE_STARTED)
        for _ in range(10): self.tick()
        self.assertIsNotNone(self.c.PREPARE_STARTED)

    def test_fist_accelerates(self):
        self.prepare()
        for _ in range(70):v,w=self.tick(['KeyW'])
        self.assertGreater(v,1.9);self.assertEqual(w,0.)
    def test_left_right_and_combined_effort(self):
        self.prepare()
        for _ in range(40):v,w=self.tick(['KeyA'])
        self.assertGreater(w,.3)
        for _ in range(60):v,w=self.tick(['KeyD','KeyW'])
        self.assertLess(w,-1.5);self.assertGreater(v,1.8)
    def test_lift_reverses_straight(self):
        self.prepare();seen_zero=False
        for _ in range(35):
            v,w=self.tick(['KeyS']);seen_zero=seen_zero or v==0
        self.assertTrue(seen_zero);self.assertAlmostEqual(v,-.6);self.assertEqual(w,0)
    def test_forward_lidar_limit_with_fist(self):
        self.prepare()
        for _ in range(40):self.tick(['KeyW'])
        v,w=self.tick(['KeyW'],distance=.1)
        self.assertEqual(v,0.)
    def test_pause_stops_then_countdown(self):
        self.prepare();self.assertEqual(self.tick(enabled=False),(0,0))
        for _ in range(60):self.assertEqual(self.tick(),(0,0))
        self.assertFalse(self.c.PREPARE_DONE)
        for _ in range(30):v,w=self.tick()
        self.assertGreater(v,0.)
    def test_heartbeat_loss_stops(self):
        self.prepare()
        for _ in range(7):v,w=self.tick(send=False)
        self.assertEqual((v,w),(0,0));self.assertFalse(self.c.ARMED['on'])
    def test_bad_or_replayed_messages_pause(self):
        self.assertTrue(tool.keyboard.accept(dict(type='keys',seq=1,keys=['KeyA'],enabled=True)))
        self.assertFalse(tool.keyboard.accept(dict(type='keys',seq=1,keys=['KeyA'],enabled=True)))
        self.assertFalse(tool.keyboard.active())
        self.assertFalse(tool.keyboard.accept(dict(type='keys',seq=2,keys=['unknown'],enabled=True)))
    def test_holding_key_prevents_preparation(self):
        for _ in range(100):self.assertEqual(self.tick(['KeyW']),(0,0))
        self.assertFalse(self.c.PREPARE_DONE)

if __name__=='__main__':unittest.main()
