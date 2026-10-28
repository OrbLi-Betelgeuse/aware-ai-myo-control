"""Prediction only: replay a CSV or read live LSL. This program NEVER sends robot commands."""
from __future__ import annotations
import argparse, json, time
from collections import deque
from pathlib import Path
import joblib
import numpy as np
from gesture_model import parse_recording, rms_features, CHANNELS

def infer(bundle, window):
    p=bundle['model'].predict_proba(rms_features(window).reshape(1,-1))[0]
    i=int(np.argmax(p))
    return dict(label=str(bundle['model'].classes_[i]), class_index=i,
                confidence=float(p[i]), valid=True)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model',default=str(Path(__file__).with_name('outputs')/'model.joblib'))
    ap.add_argument('--replay',help='Replay a single CSV; no device required')
    ap.add_argument('--stream-name',default='Myo_EMG')
    ap.add_argument('--source-id',help='Exact LSL source_id; required if multiple matching streams exist')
    ap.add_argument('--lsl-output',action='store_true',help='Publish GesturePrediction [class_index, confidence, valid]')
    ap.add_argument('--stale-ms',type=float,default=250)
    args=ap.parse_args()
    bundle=joblib.load(args.model)  # Load only the trusted model trained by this project.
    cfg=bundle['config'];n=round(cfg['sample_rate_hz']*cfg['window_ms']/1000)
    step=round(cfg['sample_rate_hz']*cfg['hop_ms']/1000)
    if args.replay:
        x,_,meta=parse_recording(Path(args.replay).read_bytes())
        if float(meta.get('sample_rate',cfg['sample_rate_hz']))!=cfg['sample_rate_hz']:
            raise ValueError('Replay sample rate differs from trained configuration.')
        trim=round(cfg['sample_rate_hz']*cfg['trim_ms']/1000)
        for start in range(trim,len(x)-trim-n+1,step):
            print(json.dumps(dict(sample_start=start,**infer(bundle,x[start:start+n]))))
        return
    import pylsl
    streams=pylsl.resolve_byprop('name',args.stream_name,timeout=3)
    if args.source_id:
        streams=[s for s in streams if s.source_id()==args.source_id]
    if len(streams)!=1:
        raise SystemExit(f'Expected one stream, found {len(streams)}. Select the intended publisher with --source-id; do not connect to a teammate/other team by accident.')
    info=streams[0]
    if info.channel_count()!=8 or abs(info.nominal_srate()-cfg['sample_rate_hz'])>1:
        raise SystemExit('Expected 8 EMG channels at the configured nominal sample rate.')
    inlet=pylsl.StreamInlet(info,max_buflen=1)
    inlet.open_stream(timeout=5)
    info=inlet.info(timeout=5)  # Discovery results omit extended channel metadata.
    channel=info.desc().child('channels').child('channel'); names=[]
    for _ in range(8):
        names.append(channel.child_value('label'));channel=channel.next_sibling()
    if names!=CHANNELS:
        raise SystemExit(f'Channel order mismatch: {names}; expected {CHANNELS}.')
    correction=inlet.time_correction(timeout=5)
    inlet.flush()
    outlet=None
    if args.lsl_output:
        outinfo=pylsl.StreamInfo('GesturePrediction','GesturePrediction',3,1000/cfg['hop_ms'],'float32','shima-rms-lda-v1')
        outinfo.desc().append_child_value('class_labels_json',json.dumps(bundle['classes']))
        outinfo.desc().append_child_value('channels_json',json.dumps(['class_index','confidence','valid']))
        outlet=pylsl.StreamOutlet(outinfo)
    buf=deque(maxlen=n);last_timestamp=None;last_print=0
    print('Prediction only; no robot commands. Ctrl+C to exit.',flush=True)
    try:
        while True:
            start=time.monotonic()
            chunk,ts=inlet.pull_chunk(timeout=0.0,max_samples=4096)
            if chunk:
                if last_timestamp is not None and max(ts)-last_timestamp>cfg['max_gap_ms']/1000:
                    buf.clear()
                buf.extend(chunk);last_timestamp=max(ts)
            fresh=last_timestamp is not None and 0 <= pylsl.local_clock()-(last_timestamp+correction) <= args.stale_ms/1000
            if not fresh:
                buf.clear()
            result=infer(bundle,np.asarray(buf)) if fresh and len(buf)==n else dict(label='invalid',class_index=-1,confidence=0.,valid=False)
            if outlet:
                outlet.push_sample([result['class_index'],result['confidence'],float(result['valid'])])
            if start-last_print>=.2:
                print(json.dumps(result),flush=True);last_print=start
            time.sleep(max(0,cfg['hop_ms']/1000-(time.monotonic()-start)))
    except KeyboardInterrupt:
        pass
    finally:
        if outlet: outlet.push_sample([-1.,0.,0.])
        inlet.close_stream()
if __name__=='__main__':main()
