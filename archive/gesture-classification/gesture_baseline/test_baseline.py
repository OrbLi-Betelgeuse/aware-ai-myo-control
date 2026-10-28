"""Focused regression checks: shared features and recording isolation."""
import json, unittest
from pathlib import Path
import numpy as np
from gesture_model import parse_recording, rms_features

class BaselineChecks(unittest.TestCase):
    def test_rms_retains_channels(self):
        x=np.tile(np.arange(1,9),(40,1))
        np.testing.assert_allclose(rms_features(x),np.arange(1,9))
    def test_comment_header_and_order(self):
        raw=('# sample_rate: 200.0\ntimestamp,'+','.join(f'EMG_{i}' for i in range(1,9))+'\n2,1,2,3,4,5,6,7,8\n1,8,7,6,5,4,3,2,1\n').encode()
        x,t,m=parse_recording(raw)
        self.assertEqual(list(t),[2,1])
        self.assertEqual(x.shape,(2,8))
        self.assertEqual(m['sample_rate'],'200.0')
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError): rms_features(np.full((40,8),np.nan))
    def test_generated_predictions_grouping(self):
        import csv
        p=Path(__file__).with_name('outputs')/'oof_predictions.csv'
        if not p.exists(): self.skipTest('Run training first')
        groups={}
        with p.open() as f:
            for row in csv.DictReader(f): groups.setdefault(row['recording'],set()).add(row['test_fold'])
        self.assertTrue(groups)
        self.assertTrue(all(len(v)==1 for v in groups.values()))
if __name__=='__main__':unittest.main()
