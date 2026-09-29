"""Pass this file to collect_runs.py --verdict-module (Python 3.8+)."""
from pathlib import Path
import sys

# WorkBuddy's exec-based plugin loader does not supply __file__.
# Resolve from this module when imported, or from the collector's script path.
_script = Path(globals().get('__file__', sys.argv[0])).resolve()
_root = next((p for p in _script.parents if (p / 'src/robocup_navigation/src').is_dir()), None)
if _root is None:
    raise RuntimeError('Cannot locate robocup_ws for coordination verdict')
sys.path.insert(0, str(_root / 'src/robocup_navigation/src'))
from robocup_navigation.coordination.verdict import verdict
