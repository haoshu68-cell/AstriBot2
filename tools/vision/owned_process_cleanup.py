"""Bounded cleanup for a launch's verified process group and environment identity."""
import os
from pathlib import Path
import signal
import time


def members(group, session, instance):
    result=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            stat=proc.joinpath('stat').read_text().split(') ',1)[1].split()
            if int(stat[2])!=group or stat[0]=='Z':continue
            env=dict(x.split('=',1) for x in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
            if env.get('ASTRIBOT_LOG_DIR')!=str(Path(session).resolve()) or env.get('ASTRIBOT_SIM_INSTANCE')!=instance:
                raise RuntimeError(f'Unverified process {proc.name} in owned group; refusing cleanup')
            result.append({'pid':int(proc.name),'start_ticks':stat[19]})
        except (FileNotFoundError,ProcessLookupError):continue
    return result


def cleanup(group, session, instance):
    signaled=[]
    for sig, timeout in ((signal.SIGTERM,3.),(signal.SIGKILL,2.)):
        for member in members(group,session,instance):
            pid=member['pid']
            try:
                fd=os.pidfd_open(pid)
                try:
                    stat=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
                    if stat[19]!=member['start_ticks']:raise RuntimeError('PID reused before cleanup')
                    signal.pidfd_send_signal(fd,sig);signaled.append(dict(member,signal=sig.name))
                finally:
                    os.close(fd)
            except ProcessLookupError:pass
        deadline=time.monotonic()+timeout
        while members(group,session,instance) and time.monotonic()<deadline:time.sleep(.05)
    remaining=members(group,session,instance)
    if remaining:raise RuntimeError(f'Owned processes remain: {remaining}')
    return {'signals':signaled,'remaining':remaining}
