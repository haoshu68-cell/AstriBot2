#!/usr/bin/env python3
"""Capture an explicitly supplied owned supervisor PID; never discovers or stops one.

Supply the PID from your launch handle. Matching a process only records identity;
the caller still owns authorization for that handle and session.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

from prepare_empty_inventory import require, verify_owner


def capture(pid, proc=Path('/proc')):
    process = proc/str(pid)
    boot = (proc/'sys/kernel/random/boot_id').read_text().strip()
    ticks = process.joinpath('stat').read_text().rsplit(') ', 1)[1].split()[19]
    value = {'pid': pid, 'start_ticks': ticks, 'boot_id': boot,
             'exe': str(process.joinpath('exe').resolve()),
             'cmdline': process.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0')}
    require(ticks == process.joinpath('stat').read_text().rsplit(') ', 1)[1].split()[19]
            and boot == (proc/'sys/kernel/random/boot_id').read_text().strip(), 'PROCESS_CHANGED_DURING_CAPTURE')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True, help='explicit supervisor PID obtained from the owning launch handle')
    parser.add_argument('--session', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(args.pid > 1, 'INVALID_PID')
    owner = capture(args.pid)
    verify_owner(owner, args.session, args.source)
    owner['captured_wall'] = time.time()
    owner['executable_sha256'] = hashlib.sha256(Path(owner['exe']).read_bytes()).hexdigest()
    owner['query_environment'] = {k: os.environ.get(k) for k in
        ('ROS_DOMAIN_ID', 'IGN_PARTITION', 'GZ_PARTITION', 'ASTRIBOT_SIM_INSTANCE', 'RMW_IMPLEMENTATION',
         'ROS_LOCALHOST_ONLY', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'IGN_IP', 'GZ_IP')}
    verify_owner(owner, args.session, args.source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(owner, stream, indent=2)
        stream.write('\n')
    print(str(args.output.resolve()))


if __name__ == '__main__':
    main()
