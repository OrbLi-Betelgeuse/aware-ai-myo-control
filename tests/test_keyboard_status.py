import _paths  # Resolve experimental modules for direct test execution.
"""The test publisher must not expose transient READY during a fault."""
import asyncio
import json
import simulate_3_2aTest as tool
class Probe:
    def __init__(self): self.payload=None
    async def send(self, raw):
        self.payload=json.loads(raw)
        raise asyncio.CancelledError
async def check():
    for phase,fault,expected in [('stopped','Waiting for fresh Myo windows','waiting'),('waiting','Simulator finished','stopped'),('waiting','','waiting'),('countdown','','countdown'),('running','','running')]:
        tool.c.PREPARE_UI.update(phase=phase,remaining=3,reason='fixture')
        tool.c.LAST_FAULT=fault
        probe=Probe()
        try: await tool.test_preparation_client(probe)
        except asyncio.CancelledError: pass
        assert probe.payload['phase']==expected
    print('Preparation status regression passed.')
asyncio.run(check())
