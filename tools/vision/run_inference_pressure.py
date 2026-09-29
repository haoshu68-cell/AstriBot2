#!/usr/bin/env python3
"""Bounded real C++ GraspNet compute load on a frozen cloud; never robot output."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time


def main():
    p=argparse.ArgumentParser()
    for name in ('worker','model','input','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600.)
    a=p.parse_args()
    if not 1<=a.seconds<=1800:raise ValueError('seconds must be 1..1800')
    for path in (a.worker,a.model,a.input):
        if not path.is_file():raise ValueError('Missing explicit artifact: '+str(path))
    a.output.mkdir(parents=True,exist_ok=False)
    def digest(path):
        h=hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
        return h.hexdigest()
    report={'scope':'real LibTorch C++ GraspNet CUDA inference on frozen historical cloud; compute pressure only',
        'execution_permission':False,'live_camera_inference':False,
        'artifacts':{str(x.resolve()):digest(x) for x in (a.worker,a.model,a.input)},'trials':[]}
    interrupted=False
    def stop(_signal,_frame):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    started=time.monotonic();deadline=started+a.seconds
    try:
        while not interrupted and time.monotonic()<deadline:
            index=len(report['trials']);begin=time.monotonic()
            command=[str(a.worker.resolve()),'--model',str(a.model.resolve()),'--input',str(a.input.resolve()),
                '--output',str((a.output/f'{index:04}.grasps.bin').resolve()),'--device','cuda']
            try:
                result=subprocess.run(command,capture_output=True,text=True,timeout=min(15.,deadline-begin))
                (a.output/f'{index:04}.log').write_text(result.stdout+result.stderr)
                row={'index':index,'started_wall':time.time()-(time.monotonic()-begin),
                    'wall_sec':time.monotonic()-begin,'returncode':result.returncode}
                if result.returncode==0:row['metrics']=json.loads(result.stdout.strip().splitlines()[-1])
                report['trials'].append(row)
                if result.returncode:break
            except subprocess.TimeoutExpired:
                report['trials'].append({'index':index,'timeout':True,
                    'budget_cancellation':time.monotonic()>=deadline,
                    'wall_sec':time.monotonic()-begin});break
            (a.output/'report.json').write_text(json.dumps(report,indent=2))
            # Bound load to one invocation per two seconds. This is not a GPU saturation claim.
            while not interrupted and time.monotonic()<min(deadline,begin+2.):time.sleep(.05)
    finally:
        report.update(duration_wall_sec=time.monotonic()-started,interrupted=interrupted)
        completed=[r for r in report['trials'] if not r.get('budget_cancellation')]
        report['budget_cancellations']=sum(bool(r.get('budget_cancellation')) for r in report['trials'])
        report['all_completed_workers_succeeded']=bool(completed) and all(
            r.get('returncode')==0 for r in completed)
        (a.output/'report.json').write_text(json.dumps(report,indent=2))
    return 0 if report['all_completed_workers_succeeded'] and not interrupted else 1


if __name__=='__main__':raise SystemExit(main())
