"""Offline behavior probes; no publisher, bridge, or hardware connection is started."""
import importlib.util,io,contextlib,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
def load(name,path):
 s=importlib.util.spec_from_file_location(name,root/path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
m=load('controller32','controllers/competition/my_team3_2.py')
class Log:
 def writerow(self,row):pass
m.LOG=Log()
checks=[]
def check(name,value):
 assert value,name
 checks.append(name)
check('3.2 missing scan is 10m (known fail-open)',m.path_clearance({})==10)
state={'scan':{'ranges':[0.1],'angle_min':0.,'angle_inc':0.1,'range_max':10.},'status':'ok'}
check('3.2 forward corridor clearance',abs(m.path_clearance(state)-.1)<1e-9)
base=dict(fresh=True,level=1.,squeeze=0.,open=False,open_effort=0.,steer=0.,tilt=0.,up=False)
with contextlib.redirect_stdout(io.StringIO()):
 m.reset_control();v,w=m.step(state,base,[]);check('3.2 obstacle caps forward at zero',v==0)
 m.reset_control();m.ARMED['on']=True;v,w=m.step(state,dict(base,up=True,tilt=1.),[])
 check('3.2 reverse transition has zero tick',(v,w)==(0,0))
 v,w=m.step(state,dict(base,up=True,tilt=1.),[]);check('3.2 reverse bypasses forward cap',v<0 and w==0)
 v,w=m.step(state,dict(base,fresh=False),[]);check('3.2 stale input disarms and zeros',(v,w)==(0,0) and not m.ARMED['on'])
print(json.dumps({'checks_passed':len(checks),'checks':checks},indent=2))
