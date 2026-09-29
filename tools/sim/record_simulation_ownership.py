#!/usr/bin/env python3
"""Read-only sampled ownership evidence for an exclusive simulation benchmark."""
import argparse
import json
import os
from pathlib import Path
import time


def snapshot():
    result = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            executable = (process / 'exe').resolve().name
            command = (process / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
            if not (executable in ('gzserver', 'gzclient') or
                    command.startswith(('ign gazebo ', 'gz sim ')) or
                    (executable not in ('bash', 'sh', 'dash', 'timeout') and
                     'gazebo server' in command)):
                continue
            environment = dict(item.split('=', 1) for item in
                               (process / 'environ').read_bytes().decode().split('\0') if '=' in item)
            result.append(dict(pid=int(process.name), command=command,
                               instance=environment.get('ASTRIBOT_SIM_INSTANCE'),
                               domain=environment.get('ROS_DOMAIN_ID'),
                               partition=environment.get('IGN_PARTITION'),
                               stdout=os.readlink(process / 'fd/1')))
        except (OSError, UnicodeError, ValueError):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--instance', required=True)
    parser.add_argument('--seconds', type=float, required=True)
    arguments = parser.parse_args()
    if not 0 < arguments.seconds <= 5400:
        parser.error('seconds must be within (0, 5400]')
    deadline = time.monotonic() + arguments.seconds
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open('x') as output:
        while True:
            rows = snapshot()
            output.write(json.dumps(dict(wall=time.time(), monotonic=time.monotonic(),
                processes=rows, other_instances=[r for r in rows if
                    r['instance'] != arguments.instance], load_average=os.getloadavg())) + '\n')
            output.flush()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(5., remaining))


if __name__ == '__main__':
    main()
