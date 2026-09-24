#!/usr/bin/env python3
"""Record only this validation driver's fixture and direct executor child."""
import hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
base=Path(__file__).resolve().parents[1]
mode,domain=sys.argv[1],int(sys.argv[2]);label=mode
assert (mode,domain) in (('normal',222),('capture_error',223))
out=base/'capture_evidence'/f'{label}.json';session_path=out.with_name(label+'_session.json')
assert not out.exists() and not session_path.exists()
assert not (Path.home()/f'.local/state/astribot/transport/domain_{domain}.jsonl').exists()
assert not Path(f'/tmp/astribot_transport_domain_{domain}.lock').exists()
ancestors=set();pid=os.getpid()
while pid>1:
    ancestors.add(pid);pid=int(Path('/proc',str(pid),'stat').read_text().split(') ',1)[1].split()[1])
foreign=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in ancestors:continue
    try:
        if f'ROS_DOMAIN_ID={domain}'.encode() in p.joinpath('environ').read_bytes().split(b'\0'):foreign.append({'pid':p.name,'cmdline':p.joinpath('cmdline').read_text().replace('\0',' ')})
    except (FileNotFoundError,ProcessLookupError,PermissionError):continue
assert not foreign,foreign
binary=base/'capture_install/astribot_s1_transport_native/lib/astribot_s1_transport_native/trajectory_executor'
cmd=['python3',str(base/'source/astribot_s1_transport_native/test/verify_plan_to_hold.py'),'--mode',mode,'--domain',str(domain),'--executable',str(binary),'--output',str(out)]
env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',IGN_PARTITION=f'astribot_capture_{domain}',GZ_PARTITION=f'astribot_capture_{domain}')
session={'command':cmd,'started_wall':time.time(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'executable_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'environment':{k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','IGN_PARTITION','GZ_PARTITION')},'foreign_processes':foreign,'processes':{}}
with out.with_name(label+'_driver.log').open('w') as log:
    proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    until=time.monotonic()+60
    while proc.poll() is None:
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:
                stat=p.joinpath('stat').read_text().split(') ',1)[1].split()
                if int(p.name)!=proc.pid and int(stat[1])!=proc.pid:continue
                identity={'start_ticks':stat[19],'exe':str(p.joinpath('exe').resolve()),'cmdline':p.joinpath('cmdline').read_text().replace('\0',' '),'mapped_project_libraries':{}}
                if p.name in session['processes']:identity['mapped_project_libraries']=session['processes'][p.name]['mapped_project_libraries']
                for line in p.joinpath('maps').read_text().splitlines():
                    fields=line.split()
                    if len(fields)>5 and '/astribot_sdk_ros2/' in fields[-1] and '.so' in fields[-1]:
                        library=Path(fields[-1]);identity['mapped_project_libraries'][str(library)]=hashlib.sha256(library.read_bytes()).hexdigest()
                session['processes'][p.name]=identity
            except (FileNotFoundError,ProcessLookupError,PermissionError):continue
        session_path.write_text(json.dumps(session,indent=2)+'\n')
        if time.monotonic()>until:
            proc.send_signal(signal.SIGINT);session['timeout_interrupt']=True;proc.wait(timeout=10);break
        time.sleep(.05)
    session['exit_code']=proc.wait();session['finished_wall']=time.time();session['remaining_owned_processes']=[]
    for pid,identity in session['processes'].items():
        p=Path('/proc',pid)
        if p.exists() and p.joinpath('stat').read_text().split(') ',1)[1].split()[19]==identity['start_ticks']:session['remaining_owned_processes'].append(pid)
    session_path.write_text(json.dumps(session,indent=2)+'\n')
print(json.dumps({'mode':mode,'exit_code':session['exit_code'],'remaining_owned_processes':session['remaining_owned_processes']}))
sys.exit(session['exit_code'])
