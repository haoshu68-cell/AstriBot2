"""Verify the pinned upstream header subset before installing it."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parent
lock = json.loads((root / 'vendor.lock.json').read_text())
source = root / 'vendor/eigen'
actual = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
          for p in source.rglob('*') if p.is_file()}
if actual != lock['sha256']:
    changed = sorted(name for name in actual.keys() | lock['sha256'].keys()
                     if actual.get(name) != lock['sha256'].get(name))
    raise SystemExit('Eigen source differs from vendor.lock.json: ' + ', '.join(changed))
print(f"Eigen {lock['version']}: {len(actual)} source files verified")
