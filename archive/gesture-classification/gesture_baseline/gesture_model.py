"""Reusable EMG loader and RMS features. No timestamps or filenames enter the model."""
from __future__ import annotations
import csv, hashlib, io, json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

CHANNELS = [f'EMG_{i}' for i in range(1, 9)]

def recordings(source):
    source = Path(source)
    if source.is_file() and source.suffix.lower() == '.zip':
        with ZipFile(source) as z:
            for name in sorted(z.namelist()):
                p = Path(name)
                if p.suffix.lower() == '.csv' and '__MACOSX' not in p.parts and not p.name.startswith('.'):
                    yield name, z.read(name)
    elif source.is_dir():
        for p in sorted(source.rglob('*.csv')):
            if '__MACOSX' not in p.parts and not p.name.startswith('.'):
                yield p.relative_to(source).as_posix(), p.read_bytes()
    else:
        raise ValueError('Data must be a ZIP or a directory containing label folders.')

def parse_recording(raw):
    text = raw.decode('utf-8-sig')
    meta = {}
    for line in text.splitlines():
        if line.startswith('#') and ':' in line:
            k, v = line[1:].split(':', 1)
            meta[k.strip()] = v.strip()
    reader = csv.DictReader(io.StringIO('\n'.join(l for l in text.splitlines() if not l.startswith('#'))))
    missing = set(['timestamp', *CHANNELS]) - set(reader.fieldnames or [])
    if missing:
        raise ValueError(f'Missing columns: {sorted(missing)}')
    rows = list(reader)
    x = np.array([[float(r[c]) for c in CHANNELS] for r in rows], dtype=np.float64)
    t = np.array([float(r['timestamp']) for r in rows], dtype=np.float64)
    if len(x) < 2 or not np.isfinite(x).all() or not np.isfinite(t).all():
        raise ValueError('Insufficient rows or nonfinite values.')
    return x, t, meta

def rms_features(x):
    """Raw RMS per channel; same implementation in offline and online code."""
    x = np.asarray(x, dtype=np.float64)
    if x.ndim != 2 or x.shape[1] != 8 or not len(x) or not np.isfinite(x).all():
        raise ValueError('Expected a finite (samples, 8) EMG window.')
    return np.sqrt(np.mean(x*x, axis=0))

def load_dataset(source, config):
    features, targets, groups, audit = [], [], [], []
    known_hashes = {}
    for name, raw in recordings(source):
        parts = Path(name).parts
        labels = [p for p in parts[:-1] if p in config['labels']]
        if len(labels) != 1:
            raise ValueError(f'{name}: expected exactly one configured label folder.')
        label = labels[0]
        digest = hashlib.sha256(raw).hexdigest()
        if digest in known_hashes:
            raise ValueError(f'Duplicate recording: {name} and {known_hashes[digest]}')
        known_hashes[digest] = name
        x, t, meta = parse_recording(raw)
        sr = float(meta.get('sample_rate', config['sample_rate_hz']))
        if abs(sr-config['sample_rate_hz']) > 1e-6:
            raise ValueError(f'{name}: sample rate {sr} differs from configured/live rate.')
        n = round(sr*config['window_ms']/1000)
        step = round(sr*config['hop_ms']/1000)
        trim = round(sr*config['trim_ms']/1000)
        windows = 0
        dropped = 0
        for start in range(trim, len(x)-trim-n+1, step):
            ts = t[start:start+n]
            # Preserve recorded row order: timestamp reversals may arise from packet stamping.
            # Skip windows containing large forward gaps; do not sort or invent samples.
            if np.max(np.diff(np.maximum.accumulate(ts)), initial=0) > config['max_gap_ms']/1000:
                dropped += 1
                continue
            features.append(rms_features(x[start:start+n]))
            targets.append(label)
            groups.append(name)
            windows += 1
        if not windows:
            raise ValueError(f'{name}: no usable windows; inspect recording/configuration.')
        span = float(t.max()-t.min())
        audit.append(dict(file=name, label=label, participant_id=config.get('participant_id', 'unknown'), sha256=digest, samples=len(x),
                          declared_hz=sr, timestamp_span_s=span,
                          rows_per_timestamp_second=(len(x)-1)/span if span>0 else None,
                          nonincreasing_timestamps=int(np.sum(np.diff(t)<=0)),
                          maximum_forward_gap_s=float(np.max(np.diff(t))),
                          usable_windows=windows, dropped_gap_windows=dropped,
                          metadata_label=meta.get('label'), stream=meta.get('stream')))
    if not features:
        raise ValueError('No recordings found.')
    missing = set(config['labels'])-set(targets)
    if missing:
        raise ValueError(f'Missing configured classes: {sorted(missing)}')
    return np.array(features), np.array(targets), np.array(groups), audit
