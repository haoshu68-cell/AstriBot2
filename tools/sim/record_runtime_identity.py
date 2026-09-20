#!/usr/bin/env python3
"""Read process identity and loaded native libraries for one isolated instance."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--instance',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();rows=[];hashes={}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            env=dict(s.split('=',1) for s in (proc/'environ').read_bytes().decode().split('\0') if '=' in s)
            if env.get('ASTRIBOT_SIM_INSTANCE')!=args.instance:continue
            argv=(proc/'cmdline').read_bytes().decode().split('\0')
            if not argv[0] or Path(argv[0]).name in ('bash','sh'):continue
            start=int((proc/'stat').read_text().rsplit(') ',1)[1].split()[19])
            libraries=set()
            for line in (proc/'maps').read_text().splitlines():
                path=line.split()[-1]
                if path.startswith('/') and ('astribot' in path or '_geometry_native' in path) and '.so' in path:
                    libraries.add(path)
            for path in libraries:
                if path not in hashes:hashes[path]=hashlib.sha256(Path(path).read_bytes()).hexdigest()
            rows.append(dict(pid=int(proc.name),start_ticks=start,argv=argv,
                domain=env.get('ROS_DOMAIN_ID'),partition=env.get('IGN_PARTITION'),
                log_dir=env.get('ASTRIBOT_LOG_DIR'),pythonpath=env.get('PYTHONPATH'),
                native_libraries={p:hashes[p] for p in sorted(libraries)}))
        except (OSError,UnicodeError,ValueError):continue
    Path(args.output).write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps(dict(instance=args.instance,records=len(rows),domains=sorted(set(r['domain'] or '' for r in rows)))))


if __name__=='__main__':main()
