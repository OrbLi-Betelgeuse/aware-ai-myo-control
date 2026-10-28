import _paths  # Resolve experimental modules for direct test execution.
"""Offline regression for idle pages retaining the keyboard connection."""
import asyncio
import json
import simulate_3_2aTest as tool

class Socket:
    def __init__(self): self.inbox=asyncio.Queue();self.outbox=asyncio.Queue();self.seq=0
    def __aiter__(self): return self
    async def __anext__(self):
        value=await self.inbox.get()
        if value is None: raise StopAsyncIteration
        return value
    async def send(self,data): await self.outbox.put(json.loads(data))
    async def close(self,**kwargs): pass
    async def keys(self,enabled):
        await self.inbox.put(json.dumps(dict(type='keys',seq=self.seq,enabled=enabled,keys=[])))
        self.seq+=1
        return await asyncio.wait_for(self.outbox.get(),1)

async def check():
    tool.keyboard=tool.KeyboardState()
    a,b=Socket(),Socket()
    ta=asyncio.create_task(tool.keyboard_client(a));tb=asyncio.create_task(tool.keyboard_client(b))
    assert not (await a.keys(False))['enabled']
    assert (await b.keys(True))['enabled'], 'idle first page must not block second'
    await a.keys(False)
    assert tool.keyboard.active(), 'idle heartbeat must not pause active page'
    assert (await a.keys(True))['busy'], 'active driving must not be stolen'
    await b.keys(False)
    assert (await a.keys(True))['enabled'], 'paused owner must release control'
    await b.inbox.put(None);await tb
    assert tool.keyboard.active(), 'non-owner disconnect must not stop owner'
    await a.inbox.put(None);await ta
    assert not tool.keyboard.active(), 'owner disconnect must stop'
    print('Connection regression passed: idle page, exclusive driving, release, disconnect.')

asyncio.run(check())
