from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
for folder in ['3.2a','keyboard-demo']:
    sys.path.insert(0,str(ROOT/'controllers/experimental'/folder))
