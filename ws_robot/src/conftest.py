"""Opt in to historical Python oracles for source-tree regression tests only."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'tools/migration/python_reference'))
from reference_bootstrap import enable

enable()
