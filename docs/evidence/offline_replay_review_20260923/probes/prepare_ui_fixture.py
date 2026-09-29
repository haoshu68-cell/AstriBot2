#!/usr/bin/env python3
"""Validation-only child stubs; neither process joins ROS or launches RViz."""
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
prefix = root / "ui_prefix"
index = prefix / "share/ament_index/resource_index/packages/astribot_operator_station"
index.parent.mkdir(parents=True, exist_ok=True)
index.touch()
player = prefix / "lib/astribot_operator_station/replay_stream"
player.parent.mkdir(parents=True, exist_ok=True)
player.write_text('''#!/usr/bin/python3
import json,os,time
from pathlib import Path
p=Path(os.environ['REVIEW_UI_COUNTER'])
n=int(p.read_text()) if p.exists() else 0
p.write_text(str(n+1))
print('META '+json.dumps(dict(start_ns=100000000000,duration_sec=10,topics=1)),flush=True)
time.sleep(.15)
print('POSITION '+str(106000000000 if n==0 else 100000000000),flush=True)
while True:time.sleep(1)
''')
player.chmod(0o755)
rviz = prefix / "bin/rviz2"
rviz.parent.mkdir(parents=True, exist_ok=True)
rviz.write_text('#!/usr/bin/python3\nimport time\nwhile True:time.sleep(1)\n')
rviz.chmod(0o755)
print(prefix)
