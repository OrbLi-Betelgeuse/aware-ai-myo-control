"""Deterministic offline regression/fault tests; no LSL or robot connections opened."""
import asyncio
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

base = load('competition32', 'controllers/competition/my_team3_2.py')
safe = load('experimental32a', 'controllers/experimental/3.2a/my_team3_2a.py')
class Log:
    def __init__(self): self.rows = []
    def writerow(self, row): self.rows.append(row)
class Inlet:
    def __init__(self, rows, stamps): self.packet = rows, stamps
    def pull_chunk(self, timeout=0): return self.packet
class Stream:
    def __init__(self, messages): self.messages = messages
    async def __aiter__(self):
        for m in self.messages: yield json.dumps(m)
        await asyncio.Event().wait()

class ControllerChecks(unittest.TestCase):
    def setUp(self):
        self.output = contextlib.redirect_stdout(io.StringIO()); self.output.__enter__()
        self.clock = patch.object(safe.time, 'monotonic', return_value=100.0); self.clock.start()
        self.lsl_clock = patch.object(safe.pylsl, 'local_clock', return_value=100.0); self.lsl_clock.start()
        for m in (base, safe): m.reset_control(); m.LOG = Log()
        safe.LAST_FAULT = ''
        safe.PREPARE_DONE = True  # These existing regression tests start after preparation.
    def tearDown(self):
        self.clock.stop(); self.lsl_clock.stop(); self.output.__exit__(None,None,None)
    def state(self, distance=10):
        ranges = [10.] * 180; ranges[90] = distance
        return dict(type='state', t=1., _received_at=100., status='ok',
            pose=dict(x=0.,y=0.,yaw=0.), vel=dict(v=0.,w=0.), cmd_applied=dict(v=0.,w=0.),
            collisions=0, course=None,
            scan=dict(ranges=ranges,angle_min=-.9,angle_inc=.01,range_max=10.))
    def inp(self, **kwargs):
        return dict(dict(fresh=True,level=1.,squeeze=0.,open=False,open_effort=0.,
                         steer=0.,tilt=0.,up=False,new_emg=True),**kwargs)
    def arm(self):
        safe.PREPARE_DONE = True
        safe.step(self.state(),self.inp(),[])
    def stopped(self, state=None, inp=None):
        self.assertEqual(safe.step(self.state() if state is None else state,
                                   self.inp(squeeze=1.) if inp is None else inp,[]),(0.,0.))
        self.assertFalse(safe.ARMED['on'])
        self.assertTrue(safe.LAST_FAULT)
        self.assertEqual(len(safe.LOG.rows[-1]),24)
    def test_valid_motion_matches_32(self):
        sequence = [(10,self.inp())]*15 + [(10,self.inp(squeeze=.8,steer=.6))]*35
        sequence += [(d,self.inp(squeeze=.8,steer=-.4)) for d in [.2,.5,.93,1.55,10] for _ in range(20)]
        sequence += [(10,self.inp(up=True,tilt=1))]*20 + [(10,self.inp())]*20
        for distance, inp in sequence:
            a = base.step(self.state(distance),inp,[])
            b = safe.step(self.state(distance),inp,[])
            np.testing.assert_allclose(a,b,rtol=0,atol=1e-12)
            self.assertEqual(base.SMOOTH['assist_limit'],safe.SMOOTH['assist_limit'])
    def test_lidar_table_knots_and_release(self):
        self.assertEqual(safe.ASSIST_DIST,base.ASSIST_DIST)
        self.assertEqual(safe.ASSIST_SPEED,base.ASSIST_SPEED)
        self.arm()
        for d,v in zip(safe.ASSIST_DIST,safe.ASSIST_SPEED):
            safe.SMOOTH['assist_limit']=safe.V_MAX
            safe.step(self.state(d),self.inp(squeeze=1),[])
            self.assertAlmostEqual(safe.SMOOTH['assist_limit'],v)
        safe.step(self.state(.1),self.inp(),[])
        safe.step(self.state(10),self.inp(),[])
        self.assertAlmostEqual(safe.SMOOTH['assist_limit'],.025)
    def test_bad_scans_stop(self):
        for scan in [None,{},dict(ranges=[]),dict(self.state()['scan'],ranges=[float('nan')]*180),
                     dict(self.state()['scan'],ranges=[-1.]*180),dict(self.state()['scan'],ranges=[11.]*180)]:
            self.arm();s=self.state();s['scan']=scan;self.stopped(s)
    def test_zero_range_stops_forward(self):
        self.arm();self.assertEqual(safe.step(self.state(0),self.inp(squeeze=1),[])[0],0)
    def test_invalid_input_stops(self):
        for inp in [self.inp(fresh=False),self.inp(level=float('nan')),self.inp(steer=2),self.inp(open=True),{}]:
            self.arm();self.stopped(inp=inp)
    def test_stale_state_and_terminal_stop(self):
        for update in [dict(_received_at=99),dict(status='estop'),dict(status='finished'),dict(_terminal=True)]:
            self.arm();s=self.state();s.update(update);self.stopped(s)
    def test_rearm_requires_neutral(self):
        self.arm();s=self.state();s.pop('scan');self.stopped(s)
        self.assertEqual(safe.step(self.state(),self.inp(squeeze=1),[]),(0,0))
        self.assertEqual(safe.step(self.state(),self.inp(tilt=-.8),[]),(0,0))
        safe.step(self.state(),self.inp(),[]);self.assertFalse(safe.ARMED['on'])
        self.assertEqual(safe.PREPARE_UI['phase'],'waiting')
    def test_loop_stall_stops(self):
        self.arm();safe.LAST_STEP=99.;self.stopped()
    def test_sample_shape_and_quaternion(self):
        for rows in [[[0]*7],[[float('inf')]*8]]:
            with self.assertRaises(safe.ControlFault):safe.sample_array(rows,'emg')
        with self.assertRaises(safe.ControlFault):safe.gravity([[0]*10])
        np.testing.assert_allclose(safe.gravity([[2,0,0,0]+[0]*6]),[0,0,1])
    def test_source_time_checks(self):
        for stamps in [[99.],[101.],[float('nan')]]:
            with self.assertRaises(safe.ControlFault):safe.read_samples(Inlet([[1]*8],stamps),'emg')
        packet=Inlet([[1]*8],[100.]);self.assertIsNotNone(safe.read_samples(packet,'emg'))
        self.assertIsNone(safe.read_samples(packet,'emg'))
    def test_live_input_buffers_and_loss(self):
        safe.CAL=dict(zero=1.,full=10.,up0=[0,0,1],axis=[1,0,0],tilt_axis=[0,1,0],
                      reach_left=.5,reach_right=-.5,reach_up=.5)
        safe.EMG=Inlet([[0]*8]*40,np.linspace(99.805,100,40))
        safe.IMU=Inlet([[1,0,0,0]+[0]*6]*10,np.linspace(99.82,100,10))
        self.assertTrue(safe.my_input()['fresh'])
        safe.EMG=Inlet([],[]);safe.IMU=Inlet([],[])
        with patch.object(safe.time,'monotonic',return_value=100.3):
            self.assertFalse(safe.my_input()['fresh'])
        self.assertEqual(len(safe.EMG_BUF),0)
    def test_degenerate_calibration_rejected(self):
        with self.assertRaises(safe.ControlFault):safe.validate_calibration(dict(zero=1.,full=1.))
    def test_bad_speed_table_rejected(self):
        with patch.object(safe,'ASSIST_DIST',[1,.5]):
            with self.assertRaises(safe.ControlFault):safe.validate_configuration()
    def test_listener_terminal_latch_and_reset(self):
        async def probe(messages):
            state={};events=[];task=asyncio.create_task(safe.listen_state(Stream(messages),state,events))
            await asyncio.sleep(0)
            result=copy.deepcopy(state)
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):await task
            return result
        s=self.state();s.pop('_received_at')
        later=dict(s,t=2.)
        result=asyncio.run(probe([s,dict(type='event',name='finish'),later]))
        self.assertTrue(result['_terminal'])
        result=asyncio.run(probe([s,dict(type='event',name='finish'),dict(s,t=0.)]))
        self.assertNotIn('_terminal',result)
    def test_listener_repeated_state_does_not_refresh(self):
        async def probe():
            state={};s=self.state();s.pop('_received_at')
            class Repeated:
                async def __aiter__(self):
                    yield json.dumps(s)
                    state['_received_at']=99.
                    yield json.dumps(s)
                    await asyncio.Event().wait()
            task=asyncio.create_task(safe.listen_state(Repeated(),state,[]));await asyncio.sleep(0)
            self.assertEqual(state['_received_at'],99.)
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):await task
        asyncio.run(probe())
    def test_listener_invalid_json_clears_state(self):
        class Invalid:
            async def __aiter__(self):yield '{bad'
        state=self.state()
        with self.assertRaises(safe.ControlFault):asyncio.run(safe.listen_state(Invalid(),state,[]))
        self.assertEqual(state,{})
    def test_run_cancellation_sends_final_zero(self):
        sent=[]
        class StateSocket:
            async def __aiter__(self):
                await asyncio.Event().wait()
                yield ''
        class CommandSocket:
            async def send(self, payload):
                sent.append(json.loads(payload))
                if len(sent)==1: raise asyncio.CancelledError()
        class Connection:
            def __init__(self, socket): self.socket=socket
            async def __aenter__(self): return self.socket
            async def __aexit__(self, *args): return False
        def connect(url):return Connection(StateSocket() if url.endswith('/state') else CommandSocket())
        with patch.object(safe.websockets,'connect',side_effect=connect):
            with self.assertRaises(asyncio.CancelledError):asyncio.run(safe.run('ws://fake'))
        self.assertGreaterEqual(len(sent),2)
        self.assertEqual((sent[-1]['v'],sent[-1]['w']),(0.,0.))
        self.assertFalse(safe.ARMED['on'])

    def countdown_tick(self, t, inp=None, state=None):
        state = self.state() if state is None else state
        state['_received_at'] = t
        with patch.object(safe.time,'monotonic',return_value=t):
            return safe.step(state,self.inp() if inp is None else inp,[])
    def test_countdown_full_three_seconds(self):
        safe.reset_control()
        for i in range(16):
            self.assertEqual(self.countdown_tick(100+i*.05),(0,0))
            self.assertEqual(safe.PREPARE_UI["phase"],"waiting")
        for i in range(60):
            self.assertEqual(self.countdown_tick(100.8+i*.05),(0,0))
            self.assertFalse(safe.ARMED['on'])
            self.assertEqual(safe.PREPARE_UI['remaining'],3-int(i*.05))
        v,w=self.countdown_tick(103.8)
        self.assertGreater(v,0);self.assertEqual(w,0)
        self.assertEqual(safe.PREPARE_UI['phase'],'running')
    def test_countdown_non_neutral_restarts(self):
        safe.reset_control()
        for i in range(25):self.countdown_tick(100+i*.05)
        self.countdown_tick(101.25,self.inp(squeeze=.5))
        self.assertIsNone(safe.PREPARE_STARTED)
        self.assertEqual(self.countdown_tick(101.3),(0,0))
        self.assertEqual(safe.PREPARE_UI['remaining'],3)
    def test_countdown_fault_cancels(self):
        safe.reset_control();self.countdown_tick(100)
        s=self.state();s.pop('scan')
        self.assertEqual(self.countdown_tick(100.05,state=s),(0,0))
        self.assertIsNone(safe.PREPARE_STARTED)
        self.assertEqual(safe.PREPARE_UI['phase'],'stopped')
        self.countdown_tick(100.1)
        self.assertEqual(safe.PREPARE_UI['remaining'],3)
    def test_course_change_cancels_running_state(self):
        async def probe():
            state={}
            s=self.state();s.pop('_received_at')
            s['course']=dict(id='L1',next_gate=0,gates_passed=0,elapsed=0.,done=False)
            changed=copy.deepcopy(s);changed['t']=2.;changed['course']['id']='L2'
            class Courses:
                async def __aiter__(self):
                    yield json.dumps(s)
                    safe.PREPARE_DONE=True
                    yield json.dumps(changed)
                    await asyncio.Event().wait()
            task=asyncio.create_task(safe.listen_state(Courses(),state,[]));await asyncio.sleep(0)
            self.assertFalse(safe.PREPARE_DONE)
            self.assertEqual(safe.PREPARE_UI['phase'],'waiting')
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):await task
        asyncio.run(probe())

    def test_zero_command_sender(self):
        class Cmd:
            async def send(self,payload):self.sent=json.loads(payload)
        cmd=Cmd();asyncio.run(safe._send_zero(cmd))
        self.assertEqual((cmd.sent['v'],cmd.sent['w']),(0,0))

if __name__=='__main__':unittest.main()
