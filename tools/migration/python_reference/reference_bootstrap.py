"""Explicit test-only access to retired implementations; no runtime env hook.

Call before importing a retired module. Frozen historical baselines should use
their explicit PYTHONPATH instead and must not call this helper.
"""
import importlib
from pathlib import Path
import sys


REFERENCE_ROOT = Path(__file__).resolve().parent
REPOSITORY = REFERENCE_ROOT.parents[2]
PACKAGES = ('astribot_s1_chassis_effort_drive', 'astribot_s1_navigation',
            'astribot_s1_navigation_policy', 'astribot_s1_robot_geometry')


def enable():
    """Select reference files first, source helpers second, installed extensions last."""
    for name in PACKAGES:
        source = REPOSITORY / 'ws_robot/src' / name
        sys.path.insert(0, str(source))
    sys.path.insert(0, str(REFERENCE_ROOT))
    for name in PACKAGES:
        package = importlib.import_module(name)
        reference = REFERENCE_ROOT / name
        source = REPOSITORY / 'ws_robot/src' / name / name
        # A test may already have imported a live helper from the regular source
        # package. Extend only this test process; never alter production __init__.
        package.__path__ = list(dict.fromkeys([
            str(reference), str(source), *package.__path__]))
        for file in reference.glob('*.py'):
            loaded = sys.modules.get(name + '.' + file.stem)
            if loaded is not None and Path(loaded.__file__).resolve() != file.resolve():
                raise RuntimeError('A different oracle was imported before reference_bootstrap: '
                                   + str(loaded.__file__))


def source_or_reference(path):
    """Resolve a formerly packaged source path for validation provenance only."""
    path = Path(path)
    if path.is_file():
        return path
    candidate = REFERENCE_ROOT / path.parent.name / path.name
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(path)
