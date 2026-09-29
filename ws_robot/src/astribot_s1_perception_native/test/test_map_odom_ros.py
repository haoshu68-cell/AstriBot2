"""Same-input isolated ROS TF replay, never importing the production Python role."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import SetParametersAtomically

HERE=Path(__file__).resolve().parent
REFERENCE=HERE/'reference'
ORACLE=REFERENCE/'astribot_s1_perception/map_odom_tf_node.py'
ROOT=HERE.parents[1]
RUN_REFERENCE=("import sys; from astribot_s1_perception import map_odom_tf_node as m; "
    "assert m.__file__ == "+repr(str(ORACLE))+"; print('ORACLE_FILE='+m.__file__,flush=True); sys.exit(m.main())")


def command(impl):
    return [os.environ.get('MAP_ODOM_CPP','/tmp/codex_map_odom_20260921/build/map_odom_tf_node')] if impl=='cpp' else [sys.executable,'-c',RUN_REFERENCE]


def environment(domain):
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    env['PYTHONPATH']=str(REFERENCE)+os.pathsep+str(ROOT/'astribot_logging')+os.pathsep+env.get('PYTHONPATH','')
    return env


def stamp(seconds):
    ns=round(seconds*1e9)
    from builtin_interfaces.msg import Time
    return Time(sec=ns//10**9,nanosec=ns%10**9)


def edge(parent,child,seconds,x=0.,y=0.,z=0.,yaw=0.,tilt=0.):
    tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child;tf.header.stamp=stamp(seconds)
    tf.transform.translation.x=float(x);tf.transform.translation.y=float(y);tf.transform.translation.z=float(z)
    tf.transform.rotation.x=math.sin(tilt/2)
    tf.transform.rotation.z=math.sin(yaw/2)*math.cos(tilt/2)
    tf.transform.rotation.w=math.cos(yaw/2)*math.cos(tilt/2)
    return tf


class Runtime:
    def __init__(self,impl,directory,parameters=None):
        self.impl=impl;self.directory=directory;self.expected_exit=0
        self.name='mapodom_'+uuid.uuid4().hex[:10]
        self.domain=int(os.environ.get('MAP_ODOM_DOMAIN','160'))
        self.params={'use_sim_time':True,'publish_rate':20.,'report_period_sec':.1,'source_timeout_sec':60.,'tf_timeout_sec':.02}
        self.params.update(parameters or {})
        self.context=Context();os.environ['ROS_LOCALHOST_ONLY']='1'
        rclpy.init(context=self.context,domain_id=self.domain)
        self.node=Node(self.name+'_probe',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.samples=[]
        self.tf_pub=self.node.create_publisher(TFMessage,'/'+self.name+'/tf',100)
        self.static_pub=self.node.create_publisher(TFMessage,'/'+self.name+'/tf_static',QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.clock=self.node.create_publisher(Clock,'/'+self.name+'/clock',10)
        self.sub=self.node.create_subscription(TFMessage,'/'+self.name+'/tf',self.on_tf,100)
        self.parameters=self.node.create_client(SetParametersAtomically,'/'+self.name+'/set_parameters_atomically')
        args=command(impl)+['--ros-args','-r','__node:='+self.name]
        for topic in ['tf','tf_static','clock']:
            args += ['-r','/'+topic+':=/'+self.name+'/'+topic]
        for name,value in self.params.items():
            encoded=str(value).lower() if isinstance(value,bool) else str(value)
            if isinstance(value,float) and not math.isfinite(value):
                encoded='.nan' if math.isnan(value) else ('.inf' if value>0 else '-.inf')
            args += ['-p',f'{name}:={encoded}']
        self.log_path=directory/(self.name+'.log');self.log=self.log_path.open('w')
        env=environment(self.domain);env['ASTRIBOT_LOG_DIR']=str(directory/'python_logging')
        self.proc=subprocess.Popen(args,env=env,stdout=self.log,stderr=subprocess.STDOUT)
        try:
            self.until(lambda:self.parameters.service_is_ready() and self.tf_pub.get_subscription_count()>=2,timeout=6.)
            actual=Path(f'/proc/{self.proc.pid}/environ').read_bytes().split(b'\0')
            assert f'ROS_DOMAIN_ID={self.domain}'.encode() in actual and b'ROS_LOCALHOST_ONLY=1' in actual
        except BaseException:
            self.close();raise

    def on_tf(self,message):
        for tf in message.transforms:
            if tf.header.frame_id==self.params.get('map_frame','map') and tf.child_frame_id==self.params.get('odom_frame','odom'):
                t=tf.transform.translation;q=tf.transform.rotation
                self.samples.append(dict(receive_ns=time.perf_counter_ns(),stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9,
                    xyz=[t.x,t.y,t.z],quaternion=[q.x,q.y,q.z,q.w]))

    def until(self,predicate,timeout=4.,alive=True):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if alive: assert self.proc.poll() is None,self.log_path.read_text()
            self.executor.spin_once(timeout_sec=.005)
            if predicate():return
        raise AssertionError('isolated TF replay timeout: '+self.log_path.read_text()[-1000:])

    def pump(self,duration=.3,now=10.,slam=None,odom=None,advance=True,step=.02):
        deadline=time.monotonic()+duration;i=0
        while time.monotonic()<deadline:
            current=now+i*step if advance else now
            transforms=[]
            for kind,values in [('slam',slam),('odom',odom)]:
                if values is None:continue
                values=dict(values);source_stamp=values.pop('stamp',current)
                parent,child=(('map','aft_mapped') if kind=='slam' else ('odom','astribot_torso_base'))
                transforms.append(edge(parent,child,source_stamp,**values))
            if transforms:self.tf_pub.publish(TFMessage(transforms=transforms))
            self.clock.publish(Clock(clock=stamp(current)))
            until=time.monotonic()+.012
            while time.monotonic()<until:self.executor.spin_once(timeout_sec=.002)
            i+=1
        return now+i*step if advance else now

    def set(self,**values):
        request=SetParametersAtomically.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()])
        f=self.parameters.call_async(request);self.until(f.done)
        return f.result().result

    def close(self):
        if getattr(self,'proc',None) is not None:
            self.proc.send_signal(signal.SIGINT)
            try:self.proc.wait(3.)
            except subprocess.TimeoutExpired:
                self.proc.kill();self.proc.wait(2.)
                self.expected_exit='must_not_require_sigkill'
        self.executor.shutdown();self.node.destroy_node();self.context.try_shutdown()
        if getattr(self,'log',None):self.log.close()
        if getattr(self,'log_path',None):self.directory.joinpath(self.name+'.json').write_text(json.dumps(self.samples,indent=2))
        assert self.proc.returncode==self.expected_exit, self.log_path.read_text()


@pytest.fixture(params=['python','cpp'])
def runtime(request,tmp_path):
    r=Runtime(request.param,tmp_path)
    try:yield r
    finally:r.close()


def test_composition_height_and_tilt_warn_only(runtime):
    r=runtime
    r.pump(.55,slam={'x':4.,'y':2.,'z':1.2,'yaw':math.pi/2,'tilt':.4},odom={'x':1.,'y':.5,'z':.3})
    assert r.samples
    assert r.samples[-1]['xyz']==pytest.approx([4.5,1.,.9])
    assert r.samples[-1]['quaternion']==pytest.approx([0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
    assert '倾角超限=' in r.log_path.read_text()


@pytest.mark.parametrize('missing',['slam','odom'])
def test_missing_recovery_and_later_stale(runtime,missing):
    r=runtime
    args={'slam':{'x':2.},'odom':{'x':.5}}
    first=dict(args);first[missing]=None
    now=r.pump(.5,**first)
    assert not r.samples
    now=r.pump(.5,now=now,**args)
    assert r.samples and r.samples[-1]['xyz'][0]==pytest.approx(1.5)
    count=len(r.samples)
    r.pump(.4,now=now+3.,slam=None,odom=None)
    # Drain output already in flight, then require no additional publish on stale TF.
    count=len(r.samples)
    r.pump(.2,now=now+4.,slam=None,odom=None)
    assert len(r.samples)==count
    assert r.proc.poll() is None
    r.pump(.3,now=75.,slam=None,odom=None)
    assert r.proc.poll() is None  # one success permanently disables startup watchdog


def test_ros_pause_rollback_cache_and_recovery(runtime):
    r=runtime
    now=r.pump(.4,slam={'x':2.},odom={})
    assert r.samples
    r.pump(.1,now=now,slam=None,odom=None,advance=False)
    before=len(r.samples)
    r.pump(.2,now=now,slam=None,odom=None,advance=False)
    assert len(r.samples)==before
    # The cache survives rollback; its retained future sample is rejected by age.
    r.pump(.3,now=2.,slam=None,odom=None)
    count=len(r.samples);r.pump(.2,now=3.,slam=None,odom=None)
    assert len(r.samples)==count
    # Catch up with no new TF: clearing the old cache on rollback must fail this.
    before=len(r.samples)
    r.pump(.3,now=now+.1,slam=None,odom=None)
    assert len(r.samples)>before and r.samples[-1]['xyz'][0]==pytest.approx(2.)
    r.pump(.3,now=now+.7,slam={'x':3.},odom={})
    assert r.samples[-1]['xyz'][0]==pytest.approx(3.)


def test_runtime_parameter_success_is_snapshot_noop(runtime):
    r=runtime
    r.pump(.4,slam={'x':2.},odom={})
    assert r.samples
    result=r.set(map_frame='ignored_new_map',max_source_age_sec=0.,max_tilt_rad=-1.,publish_rate=1.)
    assert result.successful
    before=len(r.samples)
    r.pump(.3,now=30.,slam=None,odom=None)
    count=len(r.samples);r.pump(.2,now=31.,slam=None,odom=None)
    assert len(r.samples)==count
    r.pump(.3,now=32.,slam={'x':4.},odom={})
    assert r.samples[-1]['xyz'][0]==pytest.approx(4.)


@pytest.mark.parametrize('impl',['python','cpp'])
def test_watchdog_no_initial_data_exit_one(impl,tmp_path):
    r=Runtime(impl,tmp_path,{'source_timeout_sec':.3,'tf_timeout_sec':.02})
    r.expected_exit=1
    try:
        r.pump(.3,now=0.,advance=False)
        assert r.proc.poll() is None
        r.pump(.6,now=3.,slam=None,odom=None)
        r.until(lambda:r.proc.poll() is not None,alive=False)
        assert r.proc.returncode==1
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
@pytest.mark.parametrize('limit',[1.,0.,-1.])
def test_static_zero_stamp_and_disabled_age(impl,limit,tmp_path):
    r=Runtime(impl,tmp_path,{'max_source_age_sec':limit})
    try:
        r.static_pub.publish(TFMessage(transforms=[edge('map','aft_mapped',0.,x=2.),
            edge('odom','astribot_torso_base',0.,x=.5)]))
        r.pump(.45,now=10.,slam=None,odom=None)
        if limit>0: assert not r.samples
        else:
            assert r.samples and r.samples[-1]['xyz'][0]==pytest.approx(1.5)
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
def test_nan_age_limit_keeps_old_positive_stamp(impl,tmp_path):
    r=Runtime(impl,tmp_path,{'max_source_age_sec':float('nan')})
    try:
        r.pump(.4,now=10.,slam={'x':2.,'stamp':1.},odom={'stamp':1.})
        assert r.samples and r.samples[-1]['xyz'][0]==pytest.approx(2.)
        # NaN limit disables only the age upper bound, not future/zero rejection.
        r.pump(.4,now=11.,slam={'x':3.,'stamp':30.},odom={'stamp':30.})
        count=len(r.samples);r.pump(.2,now=12.,slam=None,odom=None)
        assert len(r.samples)==count
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
@pytest.mark.parametrize('rate',[float('inf'),2e9])
def test_zero_period_timer_does_not_starve_tf_or_watchdog(impl,rate,tmp_path):
    r=Runtime(impl,tmp_path,{'publish_rate':rate,'source_timeout_sec':.5,'tf_timeout_sec':.02})
    try:
        r.pump(.2,now=0.,advance=False)
        r.pump(.5,now=.1,slam={'x':2.},odom={})
        assert r.samples
        r.pump(.2,now=65.,slam=None,odom=None)
        assert r.proc.poll() is None
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
def test_zero_period_no_source_watchdog(impl,tmp_path):
    r=Runtime(impl,tmp_path,{'publish_rate':float('inf'),'source_timeout_sec':.3,'tf_timeout_sec':.02})
    r.expected_exit=1
    try:
        r.pump(.5,now=3.)
        r.until(lambda:r.proc.poll() is not None,alive=False)
        assert r.proc.returncode==1
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
@pytest.mark.parametrize('name,value,code',[
    ('slam_world_frame','different',2),('publish_rate','0.0',2),('publish_rate','-1.0',2),
    ('publish_rate','.nan',2),('jump_report_m','0.0',2),('jump_report_m','.nan',2),
    ('publish_rate','20',1),('publish_rate','true',1),('publish_rate','"20.0"',1),
    ('report_period_sec','.inf',1),('tf_timeout_sec','.nan',1),('tf_timeout_sec','.inf',1),
])
def test_fatal_startup_or_first_tick(impl,name,value,code,tmp_path):
    args=command(impl)+['--ros-args','-p',f'{name}:={value}']
    env=environment(160);env['ASTRIBOT_LOG_DIR']=str(tmp_path/'python_logging')
    result=subprocess.run(args,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=5.)
    (tmp_path/'fatal.log').write_bytes(result.stdout)
    assert result.returncode==code,result.stdout.decode(errors='replace')


@pytest.mark.parametrize('impl',['python','cpp'])
def test_default_timeout_blocks_before_missing_warning_and_recovers(impl,tmp_path):
    r=Runtime(impl,tmp_path,{'use_sim_time':False,'tf_timeout_sec':.2,'report_period_sec':.1})
    try:
        r.until(lambda:'第 1 次' in r.log_path.read_text(),timeout=2.)
        # Timer creation precedes discovery; use consecutive warnings to measure
        # timeout once initialization/discovery is out of the interval.
        first=time.monotonic()
        r.until(lambda:'第 2 次' in r.log_path.read_text(),timeout=2.)
        assert .16 <= time.monotonic()-first < .55
        # Supply both edges after the blocked cycles, then stop clock assumptions:
        # lookup stays latest and age uses the actual system clock in this case.
        until=time.monotonic()+1.5
        while not r.samples and time.monotonic()<until:
            now=time.time()
            r.tf_pub.publish(TFMessage(transforms=[edge('map','aft_mapped',now,x=2.),
                edge('odom','astribot_torso_base',now,x=.5)]))
            # Receiving our own TF can make one spin_once return immediately.
            # Bound the source rate; this timeout/recovery case is not a flood
            # plus busy-SIGINT stress test (that historical race is archived).
            next_send=time.monotonic()+.01
            while time.monotonic()<next_send:
                r.executor.spin_once(timeout_sec=.002)
        assert r.samples and r.samples[-1]['xyz'][0]==pytest.approx(1.5)
        assert '缺失 slam=' in r.log_path.read_text()
        # This case checks recovery plus idle SIGINT. Freeze the standard ROS
        # clock after the wall-time assertions, then stop all source publication
        # and drain pending delivery. Busy-SIGINT is separately classified in
        # evidence: the unchanged rclpy oracle can throw inside take_message.
        assert r.set(use_sim_time=True).successful
        drain_until=time.monotonic()+.1
        while time.monotonic()<drain_until:
            r.executor.spin_once(timeout_sec=.002)
        assert r.proc.poll() is None
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
def test_nonfinite_wire_rejection_then_recovery_and_jump_counter(impl,tmp_path):
    r=Runtime(impl,tmp_path)
    try:
        now=r.pump(.35,slam={'x':1.},odom={})
        assert r.samples
        # TF2 rejects a NaN translation before the decomposer sees it.
        r.pump(.3,now=now,slam={'x':float('nan')},odom={})
        assert all(math.isfinite(x) for s in r.samples for x in s['xyz'])
        now=r.pump(.3,now=now+.6,slam={'x':1.,'yaw':1.},odom={})
        assert '跳变=0' in r.log_path.read_text()  # rotation alone is not a jump
        r.pump(.3,now=now,slam={'x':2.},odom={})
        assert r.samples[-1]['xyz'][0]==pytest.approx(2.)
        assert '跳变=1' in r.log_path.read_text()
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
def test_finite_input_arithmetic_overflow_exit_two(impl,tmp_path):
    r=Runtime(impl,tmp_path)
    r.expected_exit=2
    try:
        r.pump(.35,slam={'x':1.7e308,'y':1.7e308,'yaw':.7},
               odom={'x':1.7e308,'y':1.7e308,'yaw':.7})
        r.until(lambda:r.proc.poll() is not None,alive=False)
        assert r.proc.returncode==2
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
@pytest.mark.parametrize('value',[-1.,0.,float('nan')])
def test_nonpositive_nan_source_timeout_at_first_watchdog(impl,value,tmp_path):
    r=Runtime(impl,tmp_path,{'source_timeout_sec':value})
    r.expected_exit=1
    try:
        r.pump(.4,now=3.)
        r.until(lambda:r.proc.poll() is not None,alive=False)
        assert r.proc.returncode==1
    finally:r.close()


@pytest.mark.parametrize('impl',['python','cpp'])
def test_infinite_disabled_and_negative_parameter_semantics(impl,tmp_path):
    r=Runtime(impl,tmp_path,{'source_timeout_sec':float('inf'),'report_period_sec':float('nan'),
        'tf_timeout_sec':-.1,'jump_report_m':float('inf'),'max_tilt_rad':-1.})
    try:
        now=r.pump(.2,now=100.)
        assert r.proc.poll() is None
        now=r.pump(.25,now=now,slam={'x':1.},odom={})
        assert r.samples
        r.pump(.25,now=now,slam={'x':100.},odom={})
        assert r.samples[-1]['xyz'][0]==pytest.approx(100.)
        log=r.log_path.read_text()
        assert '平面容差' in log
        assert 'map→odom 跳变' not in log and '更新=' not in log
    finally:r.close()
