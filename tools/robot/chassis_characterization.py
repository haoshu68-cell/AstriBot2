#!/usr/bin/env python3
"""SDK chassis characterization. Default is an offline plan, --execute enables motion."""
import argparse
import csv
import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import signal
import select
import termios
import tty
import sys
import threading
import time

DEFAULT = Path(__file__).with_name('config') / 'chassis_characterization.json'
AXES = {'x': 0, 'y': 1, 'yaw': 2}


def dump(path, obj):
    with Path(path).open('x') as f:
        json.dump(obj, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def validate(c):
    if c.get('feedback_schema_version') != 2:
        raise ValueError('反馈协议已更新为厂家主反馈；请从新模板建立配置，不能直接沿用旧配置')
    if not isinstance(c.get('slam_reference_required'), bool):
        raise ValueError('slam_reference_required 必须为布尔值')
    if c.get('manufacturer_topic') != '/astribot_chassis/joint_space_states':
        raise ValueError('厂家反馈必须使用 joint_space_states，不接受命令回显作为测量值')
    if c['mode'] not in ('min-speed', 'response', 'braking', 'descending') or c['axis'] not in AXES:
        raise ValueError('mode/axis 无效')
    if c['direction'] not in (-1, 1) or isinstance(c['direction'], bool):
        raise ValueError('direction 必须为 -1 或 1')
    if type(c['repeats']) is not int or not 1 <= c['repeats'] <= 20:
        raise ValueError('repeats 必须为 1..20 整数')
    for k,v in c.items():
        if k.endswith(('_s', '_m', '_rad', '_m_s', '_rad_s', '_m_s2', '_rad_s2')) or k == 'noise_multiplier':
            if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0:
                raise ValueError(f'{k} 必须为正有限数')
    for key, value in {'baseline_discard_s':1., 'motion_position_resolution_m':.0005,
                       'motion_rotation_resolution_rad':.001}.items():
        c.setdefault(key,value)
        if isinstance(c[key],bool) or not isinstance(c[key],(float,int)) or not math.isfinite(c[key]) or c[key]<=0:
            raise ValueError(f'{key} 必须为正有限数')
    if c['baseline_s']-c['baseline_discard_s']<2.:
        raise ValueError('剔除启动瞬态后至少保留 2 秒基线')
    if c['motion_position_resolution_m']>min(.001,c['max_travel_m']*.01) or c['motion_rotation_resolution_rad']>min(.002,c['max_rotation_rad']*.01):
        raise ValueError('累计运动分辨率过大，不能用于绕过行程保护')
    levels = c['levels']
    if not levels or len(levels)>30 or any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in levels):
        raise ValueError('levels 需要 1..30 个正有限数；零速基线自动添加')
    if levels != sorted(set(levels)):
        raise ValueError('levels 必须严格递增；descending 模式自动倒序执行')
    angular = c['axis']=='yaw'
    cap=c['max_yaw_rate_rad_s'] if angular else c['max_speed_m_s']
    accel=c['max_angular_acceleration_rad_s2'] if angular else c['max_acceleration_m_s2']
    if max(levels)>cap or max(levels)/c['ramp_s']>accel:
        raise ValueError('速度档位或 ramp 对应加速度超过配置上限')
    if c['plateau_s']<3 or c['hold_s']<2 or c['baseline_s']<3:
        raise ValueError('plateau>=3、hold>=2、baseline>=3 秒')
    return c


def plan(c):
    phases=[{'name':'baseline','trial':-1,'level':0.,'from':0.,'to':0.,'duration':c['baseline_s']}]
    index=0
    for repeat in range(c['repeats']):
        levels=list(reversed(c['levels'])) if c['mode']=='descending' else c['levels']
        previous=0.
        for level in levels:
            phases.extend([
                {'name':'ramp','trial':index,'level':level,'from':previous,'to':level,'duration':c['ramp_s']},
                {'name':'plateau','trial':index,'level':level,'from':level,'to':level,'duration':c['plateau_s']}])
            if c['mode']!='descending' or level==levels[-1]:
                phases.extend([
                    {'name':'stop','trial':index,'level':level,'from':level,'to':0.,
                     'duration':.02 if c['mode']=='braking' else c['ramp_s']},
                    {'name':'hold','trial':index,'level':level,'from':0.,'to':0.,'duration':c['hold_s']}])
                previous=0.
            else:
                previous=level
            index+=1
    elapsed=0.; travel=0.
    for p in phases:
        p['start_s']=elapsed; elapsed+=p['duration'];p['end_s']=elapsed
        travel+=(p['from']+p['to'])/2*p['duration']
    limit=c['max_rotation_rad'] if c['axis']=='yaw' else c['max_travel_m']
    if elapsed>c['max_duration_s'] or travel>limit*.9:
        raise ValueError(f'计划时间 {elapsed:.1f}s 或积分行程 {travel:.3f} 超预算；减少档位/重复/时长，或现场核实后调整边界')
    return {'phases':phases,'duration_s':elapsed,'integrated_axis_travel':travel}


def command(phase, t):
    fraction=max(0.,min(1.,(t-phase['start_s'])/phase['duration']))
    return phase['from']+(phase['to']-phase['from'])*fraction


class OperatorStop:
    """Latched operator request; no continue/resume command exists."""
    def __init__(self, event, stop_file):
        self.event=event;self.stop_file=Path(stop_file)
        self.reason=None;self.requested_mono_s=None
        self.closed=threading.Event();self.thread=None;self.fd=None;self.attributes=None

    def request(self, reason, publish=True):
        # Signal handlers also enter here, so metadata assignment avoids taking a thread lock.
        if self.reason is None:
            self.reason=reason;self.requested_mono_s=time.monotonic()
        if publish:self.event.set()

    def poll(self):
        if self.stop_file.exists():self.request('STOP file')
        if self.reason is not None:self.event.set()
        return self.event.is_set()

    def start_keyboard(self):
        if not sys.stdin.isatty():
            print('无交互终端：使用 Ctrl-C 或从另一个终端创建 STOP 文件。',flush=True)
            return
        self.fd=sys.stdin.fileno();self.attributes=termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd,termios.TCSANOW)
        def listen():
            try:
                while not self.closed.is_set() and not self.event.is_set():
                    ready,_,_=select.select([self.fd],[],[],.05)
                    if not ready:continue
                    key=os.read(self.fd,1)
                    if key in (b' ',b's',b'S',b'q',b'Q',b'\x04',b''):
                        self.request('keyboard '+repr(key))
                        print('\n已请求停车；本次测试结束，不再执行后续档位。',flush=True)
                        break
            except (OSError,ValueError):
                if not self.closed.is_set():self.request('terminal input lost')
        self.thread=threading.Thread(target=listen,daemon=True);self.thread.start()

    def close(self):
        self.closed.set()
        if self.thread:self.thread.join(timeout=.2)
        if self.attributes is not None:
            try:termios.tcsetattr(self.fd,termios.TCSADRAIN,self.attributes)
            except (termios.error,OSError):pass


def operator_stop_pending(c):
    event=c.get('_operator_stop_event')
    return (event is not None and event.is_set()) or bool(c.get('_operator_stop_file') and Path(c['_operator_stop_file']).exists())


def sdk_worker(pipe, c):
    """Separate SDK process avoids mixing its middleware executor with the odom reader."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    robot=None;middleware=None;spin_thread=None;sdk_initialization_started=False
    try:
        sdk_root=os.environ.get('ASTRIBOT_SDK_ROOT')
        if sdk_root:
            root=Path(sdk_root).expanduser().resolve()
            if not (root/'astribot_sdk').is_dir():raise ImportError(f'ASTRIBOT_SDK_ROOT 下没有 astribot_sdk: {root}')
            sys.path.insert(0,str(root))
            sys.path.insert(0,str(root/'astribot_sdk/core/common'))
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        import astribot_ros_middleware as middleware
        sdk_initialization_started=True
        robot=Astribot(freq=250., high_control_rights=False, node_name='chassis_characterization_sdk')
        def spin():
            try:
                while middleware.ok():middleware.spin()
            except Exception as exc:
                try:pipe.send({'type':'middleware_spin_exit','error':str(exc)})
                except (OSError,EOFError):pass
        spin_thread=threading.Thread(target=spin,daemon=True);spin_thread.start()
        chassis=robot.chassis_name
        if not robot.get_control_rights_status():
            raise RuntimeError('未取得控制权')
        desired=list(robot.get_desired_joints_position([chassis])[0])
        actual=list(robot.get_current_joints_position([chassis])[0])
        if len(desired)!=3 or len(actual)!=3 or not all(math.isfinite(x) for x in desired+actual):
            raise RuntimeError('底盘必须为有限的 3 自由度反馈')
        lower,upper=robot.get_joints_position_limit([chassis]); lower,upper=lower[0],upper[0]
        if len(lower)!=3 or len(upper)!=3:
            raise RuntimeError('无法取得底盘限位')
        axis=AXES[c['axis']]
        target=desired.copy(); velocity=0.; armed=False
        last=time.monotonic(); lease=last; last_report=last;integrated=0.
        pipe.send({'type':'ready','desired':desired,'actual':actual})
        while middleware.ok():
            if operator_stop_pending(c):
                pipe.send({'type':'operator_stop_received'})
                return
            now=time.monotonic();dt=now-last;last=now
            while pipe.poll():
                if operator_stop_pending(c):return
                req=pipe.recv();lease=now
                if req['type']=='finish':
                    return
                if req['type']=='abort':
                    raise RuntimeError('操作者或观测端中止')
                velocity=float(req['velocity']);armed=True
            if armed and (now-lease>.25 or dt>.05):
                raise RuntimeError(f'SDK 时序保护: loop_dt_s={dt:.6f}, command_age_s={now-lease:.6f}, velocity={velocity:.6f}; limits=0.05/0.25 s')
            if not robot.get_control_rights_status():
                raise RuntimeError('控制权丢失')
            actual=list(robot.get_current_joints_position([chassis])[0])
            if len(actual)!=3 or not all(math.isfinite(x) for x in actual):
                raise RuntimeError('SDK 反馈无效')
            if math.hypot(target[0]-actual[0],target[1]-actual[1])>c['max_position_lead_m'] or abs(target[2]-actual[2])>c['max_angular_lead_rad']:
                raise RuntimeError('SDK 指令与反馈偏差超过上限；不允许继续积累位置目标')
            cap=c['max_yaw_rate_rad_s'] if axis==2 else c['max_speed_m_s']
            if not math.isfinite(velocity) or abs(velocity)>cap+1e-9:
                raise RuntimeError('下发速度无效/超限')
            if armed:
                step=velocity*dt;integrated+=abs(step);target[axis]+=step
                limit=c['max_rotation_rad'] if axis==2 else c['max_travel_m']
                if integrated>limit or not all(lower[i]<=target[i]<=upper[i] for i in range(3)):
                    raise RuntimeError('积分行程或 SDK 关节限位超限')
                if operator_stop_pending(c):return
                robot.set_joints_position([chassis],[target.copy()],control_way='filter',use_wbc=False,add_default_torso=False)
            if now-last_report>=.02:
                pipe.send({'type':'state','sdk_mono_s':now,'desired':target.copy(),'actual':actual,
                           'integrated_axis_travel':integrated,'velocity':velocity})
                last_report=now
            time.sleep(max(0.,.004-(time.monotonic()-now)))
        raise RuntimeError('SDK middleware 已退出')
    except BaseException as exc:
        try:
            pipe.send({'type':'error','error':str(exc),'sdk_initialization_started':sdk_initialization_started})
        except (BrokenPipeError, EOFError):
            pass
    finally:
        if robot is not None:
            try:
                robot.stop_robot()
                pipe.send({'type':'stop_requested','note':'SDK stop_robot returned; not physical stop proof'})
                # Drain queued commands without executing them until the parent acknowledges the receipt.
                deadline=time.monotonic()+2.
                while time.monotonic()<deadline:
                    if pipe.poll(.02) and pipe.recv().get('type')=='stop_ack':break
            except BaseException as exc:
                try: pipe.send({'type':'stop_error','error':str(exc)})
                except (BrokenPipeError, EOFError): pass
        if middleware is not None and callable(getattr(middleware,'shutdown',None)):
            try:
                pipe.send({'type':'middleware_shutdown_started'})
                middleware.shutdown()
                if spin_thread is not None:spin_thread.join(timeout=.5)
                pipe.send({'type':'middleware_shutdown_returned'})
            except Exception as exc:
                try:pipe.send({'type':'middleware_shutdown_error','error':str(exc)})
                except (OSError,EOFError):pass
        pipe.close()


def time_mean(rows, value):
    duration=rows[-1]['stamp_s']-rows[0]['stamp_s'] if len(rows)>1 else 0.
    if duration<=0:return math.inf
    return sum((value(a)+value(b))*.5*(b['stamp_s']-a['stamp_s'])
               for a,b in zip(rows,rows[1:]))/duration


def stop_limits(c):
    linear=c['settle_speed_m_s'];angular=c['settle_yaw_rate_rad_s']
    if c['axis']=='yaw':angular=min(angular,min(c['levels'])*.25)
    else:linear=min(linear,min(c['levels'])*.25)
    return linear,angular


def feedback_settled(window,c):
    if len(window)<5 or window[-1]['stamp_s']-window[0]['stamp_s']<.9:return False
    if any(b['stamp_s']-a['stamp_s']>c.get('manufacturer_timeout_s',.25) for a,b in zip(window,window[1:])):return False
    linear,angular=stop_limits(c)
    # A low mean alone can hide oscillation; also bound the complete pose envelope.
    span=math.hypot(max(r['x'] for r in window)-min(r['x'] for r in window),
                    max(r['y'] for r in window)-min(r['y'] for r in window))
    yaw=[r.get('yaw_unwrapped',r['yaw']) for r in window]
    return (time_mean(window,lambda r:math.hypot(r['vx'],r['vy']))<=linear
            and time_mean(window,lambda r:abs(r['wz']))<=angular
            and span<=linear and max(yaw)-min(yaw)<=angular)


class MotionBudget:
    """Accumulate resolved excursions; retain raw total variation for diagnostics."""
    def __init__(self,c):
        self.c=c;self.last=None;self.xy=None;self.angle=None
        self.travel=0.;self.rotation=0.;self.raw_travel=0.;self.raw_rotation=0.
        self.pending_travel=0.;self.pending_rotation=0.

    def update(self,r):
        angle=r.get('yaw_unwrapped',r['yaw'])
        if self.last is None:
            self.xy=(r['x'],r['y']);self.angle=angle
        else:
            self.raw_travel+=math.hypot(r['x']-self.last['x'],r['y']-self.last['y'])
            self.raw_rotation+=abs(wrap(r['yaw']-self.last['yaw']))
        self.pending_travel=math.hypot(r['x']-self.xy[0],r['y']-self.xy[1])
        self.pending_rotation=abs(angle-self.angle)
        if self.pending_travel>=self.c.get('motion_position_resolution_m',.0005):
            self.travel+=self.pending_travel;self.xy=(r['x'],r['y']);self.pending_travel=0.
        if self.pending_rotation>=self.c.get('motion_rotation_resolution_rad',.001):
            self.rotation+=self.pending_rotation;self.angle=angle;self.pending_rotation=0.
        self.last=r
        if self.travel+self.pending_travel>self.c['max_travel_m']:
            return '厂家累计平移超限: '+json.dumps(self.report())
        if self.rotation+self.pending_rotation>self.c['max_rotation_rad']:
            return '厂家累计转角超限: '+json.dumps(self.report())
        return None

    def report(self):
        return {'resolved_travel_m':self.travel+self.pending_travel,
                'resolved_rotation_rad':self.rotation+self.pending_rotation,
                'raw_travel_m':self.raw_travel,'raw_rotation_rad':self.raw_rotation,
                'max_travel_m':self.c['max_travel_m'],'max_rotation_rad':self.c['max_rotation_rad']}


def await_worker_exit(worker,pipe,on_event,pump,timeout=5.):
    deadline=time.monotonic()+timeout;pipe_open=True
    while time.monotonic()<deadline:
        pump()
        if pipe_open:
            # Bound message draining so feedback and the deadline cannot be starved.
            for _ in range(100):
                if not pipe.poll():break
                try:on_event(pipe.recv())
                except (EOFError,OSError):pipe_open=False;break
        if not worker.is_alive():
            worker.join(timeout=0)
            return True
        worker.join(timeout=min(.001,max(0.,deadline-time.monotonic())))
    return not worker.is_alive()


def summarize(rows,c):
    # Each sample is a unique source-time odometry observation, never a repeated getter value.
    base=[r for r in rows if r['phase']=='baseline' and r['phase_elapsed_s']>=c.get('baseline_discard_s',1.)]
    axis=c['axis'];direction=c['direction'];start=base[0] if base else None
    if len(base)<10:
        return {'status':'insufficient_baseline','hardware_validated':False}
    yaw0=start['yaw']; cs,sn=math.cos(yaw0),math.sin(yaw0)
    def position(r):
        if axis=='yaw': return r['yaw_unwrapped']
        dx,dy=r['x']-start['x'],r['y']-start['y']
        if start.get('source') == 'manufacturer':
            return dx if axis=='x' else dy
        return dx*cs+dy*sn if axis=='x' else -dx*sn+dy*cs
    vals=[position(r) for r in base]; noise=max(vals)-min(vals)
    floor=c['min_rotation_rad'] if axis=='yaw' else c['min_displacement_m']
    threshold=max(floor,c['noise_multiplier']*noise)
    reported_key={'x':'vx','y':'vy','yaw':'wz'}[axis]
    baseline_windows=[]
    for i in range(int(base[-1]['stamp_s']-base[0]['stamp_s'])):
        window=[r for r in base if i<=r['stamp_s']-base[0]['stamp_s']<i+1.]
        if len(window)>=5 and window[-1]['stamp_s']-window[0]['stamp_s']>=.8:
            baseline_windows.append(abs(time_mean(window,lambda r:r[reported_key])))
    if not baseline_windows:
        return {'status':'insufficient_baseline','hardware_validated':False}
    velocity_noise=max(baseline_windows)
    baseline_stable=noise<=floor and velocity_noise<=floor
    trials=[]
    for trial in sorted({r['trial'] for r in rows if r['trial']>=0}):
        selected=[r for r in rows if r['trial']==trial and r['phase']=='plateau' and r['phase_elapsed_s']>=1.]
        if len(selected)<10: continue
        duration=selected[-1]['stamp_s']-selected[0]['stamp_s']
        delta=direction*(position(selected[-1])-position(selected[0]))
        bins=[]
        for i in range(3):
            part=[r for r in selected if i/3 <= (r['stamp_s']-selected[0]['stamp_s'])/max(duration,1e-9) <= (i+1)/3]
            bins.append(direction*(position(part[-1])-position(part[0])) if len(part)>1 else -math.inf)
        stable=duration>=c['plateau_s']-1.3 and delta>threshold and all(v>threshold/3 for v in bins)
        reported_key={'x':'vx','y':'vy','yaw':'wz'}[axis]
        reported_mean=time_mean(selected,lambda r:direction*r[reported_key])
        stable=stable and baseline_stable
        velocity_threshold=max(velocity_noise*c['noise_multiplier'],floor/max(duration,1e-9))
        reasons=[]
        if duration<c['plateau_s']-1.3:reasons.append('incomplete_plateau')
        if delta<=threshold:reasons.append('insufficient_displacement')
        if not all(v>threshold/3 for v in bins):reasons.append('insufficient_third_displacement')
        if not baseline_stable:reasons.append('unstable_baseline')
        if start.get('source')=='manufacturer':
            if reported_mean<=velocity_threshold:reasons.append('reported_velocity_below_threshold')
            stable=stable and reported_mean>velocity_threshold
        speed=delta/duration if duration>0 else None
        stopping=[r for r in rows if r['trial']==trial and r['phase'] in ('stop','hold')]
        metrics={}
        if stopping:
            event_t=stopping[0]['t_s']-stopping[0]['phase_elapsed_s']
            settled=None
            for j,r in enumerate(stopping):
                window=[v for v in stopping[j:] if v['t_s']<=r['t_s']+1.]
                if feedback_settled(window,c):
                    settled=r;break
            linear_limit,angular_limit=stop_limits(c)
            pre_stop=selected[-max(5,len(selected)//5):]
            initially_moving=(time_mean(pre_stop,lambda r:math.hypot(r['vx'],r['vy']))>linear_limit
                              or time_mean(pre_stop,lambda r:abs(r['wz']))>angular_limit)
            if not initially_moving:settled=None
            path=[r for r in stopping if settled is not None and r['t_s']<=settled['t_s']+1.]
            metrics={'planned_stop_t_s':event_t,
                     'stop_metric_status':('settled' if settled else 'unconfirmed') if initially_moving else 'initial_speed_not_distinguishable',
                     'effective_stop_speed_m_s':linear_limit,'effective_stop_yaw_rate_rad_s':angular_limit,
                     'stop_threshold_delay_s':settled['t_s']-event_t if settled else None,
                     'travel_through_stop_confirmation_m':sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(path,path[1:])) if settled else None,
                     'rotation_through_stop_confirmation_rad':sum(abs(b['yaw_unwrapped']-a['yaw_unwrapped']) for a,b in zip(path,path[1:])) if settled else None,
                     'timing_basis':'parent schedule + source-labelled feedback; inspect SDK event log and independent reference for exact braking'}
        trials.append({'trial':trial,'level':selected[0]['level'],'plateau_delta':delta,
                       'mean_reported_axis_velocity':reported_mean,'baseline_reported_velocity_noise':velocity_noise,
                       'mean_projected_velocity':speed,'steady_gain':speed/selected[0]['level'] if speed is not None else None,
                       'continuous_motion_candidate':stable,
                       'third_displacements':[v if math.isfinite(v) else None for v in bins],
                       'third_displacement_threshold':threshold/3,
                       'plateau_observation_duration_s':duration,
                       'reported_velocity_threshold':velocity_threshold if start.get('source')=='manufacturer' else None,
                       'continuous_motion_rejection_reasons':reasons, **metrics})
    qualifying=[]
    for level in c['levels']:
        group=[r for r in trials if r['level']==level]
        if len(group)==c['repeats'] and all(r['continuous_motion_candidate'] for r in group):qualifying.append(level)
    return {'status':'measurement_only' if baseline_stable else 'unstable_baseline',
            'baseline_discard_s':c.get('baseline_discard_s',1.),'baseline_reported_velocity_noise':velocity_noise,
            'baseline_velocity_noise_method':'maximum absolute source-time-weighted 1 s mean after startup discard',
            'source':start.get('source','legacy_odom'),'mode':c['mode'],'baseline_peak_to_peak':noise,
            'motion_displacement_threshold':threshold,'trials':trials,
            'minimum_continuous_motion_candidate':min(qualifying) if qualifying else None,
            'units':'rad and rad/s' if axis=='yaw' else 'm and m/s',
            'hardware_validated':False,
            'interpretation':'SDK filter chain measurement; candidate needs repeatability, slip/creep and external reference review'}



class FeedbackStream:
    """One producer and one monotonically advancing source clock per measurement stream."""
    def __init__(self, source, timeout, c):
        self.source=source;self.timeout=timeout;self.c=c
        self.latest=None;self.pending=[];self.gid=None;self.error=None
        self.counts={'received':0,'accepted':0,'duplicates':0,'conflicting_duplicates':0,
                     'backwards':0,'publisher_changes':0}

    def accept(self, data, stamp_ns, gid, now=None, wall=None):
        self.counts['received']+=1
        if self.error:return
        now=time.monotonic() if now is None else now
        wall=time.time() if wall is None else wall
        try:
            if self.gid is not None and gid != self.gid:
                self.counts['publisher_changes']+=1
                raise ValueError('多个发布端/发布端变化，禁止混合来源')
            self.gid=gid
            if stamp_ns<=0 or abs(wall-stamp_ns/1e9)>self.timeout:
                raise ValueError('源时间陈旧或墙钟未同步')
            if not all(math.isfinite(v) for v in data.values()):
                raise ValueError('非有限反馈')
            old=self.latest
            if old:
                dt=(stamp_ns-old['stamp_ns'])/1e9
                if dt<0:
                    self.counts['backwards']+=1
                    raise ValueError('源时间回退')
                if dt==0:
                    self.counts['duplicates']+=1
                    keys=('x','y','yaw') if self.source=='slam_reference' else ('x','y','yaw','vx','vy','wz')
                    if any(abs(data[k]-old[k])>1e-9 for k in keys):
                        self.counts['conflicting_duplicates']+=1
                        raise ValueError('相同源时间对应冲突测量')
                    return
                if dt>self.timeout:raise ValueError('源数据间断')
                if math.hypot(data['x']-old['x'],data['y']-old['y'])>self.c['max_odom_step_m'] or abs(wrap(data['yaw']-old['yaw']))>self.c['max_odom_step_rad']:
                    raise ValueError('位姿跳变')
                data['yaw_unwrapped']=old['yaw_unwrapped']+wrap(data['yaw']-old['yaw'])
                if self.source=='slam_reference':
                    # Never use the colleague bridge's alternating derivative/zero twist.
                    dx,dy=data['x']-old['x'],data['y']-old['y'];cs,sn=math.cos(data['yaw']),math.sin(data['yaw'])
                    data.update(vx=(dx*cs+dy*sn)/dt,vy=(-dx*sn+dy*cs)/dt,
                                wz=wrap(data['yaw']-old['yaw'])/dt)
            else:
                data['yaw_unwrapped']=data['yaw']
                if self.source=='slam_reference':data.update(vx=0.,vy=0.,wz=0.)
            data.update(source=self.source,stamp_ns=stamp_ns,stamp_s=stamp_ns/1e9,mono_s=now)
            self.latest=data;self.pending.append(data);self.counts['accepted']+=1
        except ValueError as exc:self.error=f'{self.source}: {exc}'

    def status(self):
        return {**self.counts,'error':self.error,'publisher_gid':self.gid,
                'last_unique_receive_age_s':time.monotonic()-self.latest['mono_s'] if self.latest else None}


class FeedbackReceiver:
    def __init__(self,node,c):
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from astribot_msgs.msg import RobotJointState
        self.c=c;self.node=node;self.publisher_ids={}
        self.primary=FeedbackStream('manufacturer',c['manufacturer_timeout_s'],c)
        self.reference=FeedbackStream('slam_reference',c['odom_timeout_s'],c)
        node.create_subscription(RobotJointState,c['manufacturer_topic'],self.manufacturer,qos_profile_sensor_data)
        node.create_subscription(Odometry,c['odom_topic'],self.slam,qos_profile_sensor_data)
        node.create_timer(.2,self.refresh_publishers)

    def refresh_publishers(self):
        # Humble callbacks expose messages only; identity comes from a periodic DDS graph snapshot.
        for topic,stream in ((self.c['manufacturer_topic'],self.primary),(self.c['odom_topic'],self.reference)):
            ids={bytes(p.endpoint_gid).hex() for p in self.node.get_publishers_info_by_topic(topic)}
            if len(ids)>1:
                stream.error=stream.source+': DDS 图发现多个发布端，禁用混合测量'
                stream.counts['publisher_changes']+=int(topic not in self.publisher_ids or self.publisher_ids[topic] is not None)
                self.publisher_ids[topic]=None
            else:self.publisher_ids[topic]=next(iter(ids),None)

    def manufacturer(self,m):
        gid=self.publisher_ids.get(self.c['manufacturer_topic'])
        if gid is None:return
        if len(m.position)!=3 or len(m.velocity)!=3:
            self.primary.error='厂家 position/velocity 必须为 3 维 [x,y,theta]；禁止用命令回显替代'
            return
        data=dict(zip(('x','y','yaw'),m.position));data.update(zip(('vx','vy','wz'),m.velocity))
        self.primary.accept(data,m.header.stamp.sec*1000000000+m.header.stamp.nanosec,gid)

    def slam(self,m):
        gid=self.publisher_ids.get(self.c['odom_topic'])
        if gid is None:return
        if m.header.frame_id!=self.c['expected_odom_frame'] or m.child_frame_id!=self.c['expected_base_frame']:
            self.reference.error='SLAM frame 不匹配';return
        q=m.pose.pose.orientation
        if not all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.02:
            self.reference.error='SLAM 四元数无效';return
        data={'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
              'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
        self.reference.accept(data,m.header.stamp.sec*1000000000+m.header.stamp.nanosec,gid)

    def failure(self):
        for stream in [self.primary]+([self.reference] if self.c['slam_reference_required'] else []):
            if stream.error:return stream.error
            if stream.latest is None or time.monotonic()-stream.latest['mono_s']>stream.timeout:
                return stream.source+' 唯一源数据接收超时'
        return None

    def report(self):return {'manufacturer':self.primary.status(),'slam_reference':self.reference.status(),
                            'slam_required':self.c['slam_reference_required']}


def probe_feedback(c,out,duration):
    """Subscriptions only: this entry never constructs SDK objects or workers."""
    import rclpy
    rclpy.init(args=[]);node=rclpy.create_node('chassis_feedback_readonly')
    reader=FeedbackReceiver(node,c);start=time.monotonic()
    try:
        with (out/'feedback.jsonl').open('x',buffering=1) as f:
            while time.monotonic()-start<duration:
                rclpy.spin_once(node,timeout_sec=.02)
                for stream in (reader.primary,reader.reference):
                    for row in stream.pending:f.write(json.dumps(row,allow_nan=False)+'\n')
                    stream.pending.clear()
    except KeyboardInterrupt:pass
    finally:
        result=reader.report();result.update(read_only=True,sdk_instantiated=False,hardware_validated=False)
        result['required_feedback_ready']=reader.failure() is None and reader.primary.counts['accepted']>=20
        dump(out/'feedback_report.json',result);print(json.dumps(result,ensure_ascii=False,indent=2))
        node.destroy_node();rclpy.shutdown()
    return 0 if result['required_feedback_ready'] else 1


def execute(c,p,out):
    import rclpy
    rclpy.init(args=[]);node=rclpy.create_node('chassis_characterization_feedback')
    reader=FeedbackReceiver(node,c);received=reader.primary.pending
    ctx=mp.get_context('spawn');stop_event=ctx.Event()
    controls=OperatorStop(stop_event,out/'STOP')
    c=dict(c);c['_operator_stop_event']=stop_event;c['_operator_stop_file']=str((out/'STOP').resolve())
    reference_rows=[]
    old_handlers={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    for sig in old_handlers:signal.signal(sig,lambda number,_frame:controls.request(signal.Signals(number).name,publish=False))
    budget=MotionBudget(c)
    worker=None;pipe=None;rows=[];outcome='aborted';error=None;stop_receipt=False;stop_receipt_wall=None;preinit_failure=False
    with (out/'samples.csv').open('x',newline='') as sf, (out/'slam_reference.csv').open('x',newline='') as rf, (out/'events.jsonl').open('x',buffering=1) as ef:
        writer=None;reference_writer=None
        def event(e):
            nonlocal stop_receipt,stop_receipt_wall,preinit_failure
            ef.write(json.dumps({'mono_s':time.monotonic(),**e},allow_nan=False)+'\n')
            if e.get('type')=='error' and e.get('sdk_initialization_started') is False:
                preinit_failure=True
            if e.get('type')=='stop_requested':
                stop_receipt=True;stop_receipt_wall=time.time()
                try:pipe.send({'type':'stop_ack'})
                except (OSError,EOFError):pass
        try:
            print(f'停车文件：{(out / "STOP").resolve()}',flush=True)
            print('等待厂家真实反馈（最多 60 秒），SLAM 单独作为参考；尚未创建 SDK 会话。',flush=True)
            until=time.monotonic()+60
            while time.monotonic()<until and not reader.primary.error and not controls.poll():
                rclpy.spin_once(node,timeout_sec=.02)
                if len(received)>=20 and reader.failure() is None:break
            if reader.failure() or controls.poll() or len(received)<20:raise RuntimeError(reader.failure() or '厂家数据不足/中断')
            pipe,child=ctx.Pipe();worker=ctx.Process(target=sdk_worker,args=(child,c));worker.start();child.close()
            ready=False;until=time.monotonic()+60
            while not ready and time.monotonic()<until and not controls.poll():
                rclpy.spin_once(node,timeout_sec=.01)
                if pipe.poll():
                    e=pipe.recv();event(e)
                    if e['type']=='error':raise RuntimeError(e['error'])
                    ready=e['type']=='ready'
                if reader.failure():raise RuntimeError(reader.failure())
                received.clear();reader.reference.pending.clear()
            if not ready:raise RuntimeError('SDK 初始化超时/中断')
            start=time.monotonic();budget.update(reader.primary.latest);idx=0;state_time=start
            received.clear();reader.reference.pending.clear()
            controls.start_keyboard()
            print('开始自动测试：空格 / s / q 无需回车，或 Ctrl-C，均请求整机停车并结束本次测试；不能代替物理急停。',flush=True)
            while True:
                rclpy.spin_once(node,timeout_sec=.005);now=time.monotonic();elapsed=now-start
                if controls.poll() or reader.failure():raise RuntimeError(reader.failure() or '操作者中断')
                while pipe.poll():
                    e=pipe.recv();event(e)
                    if e['type']=='error':raise RuntimeError(e['error'])
                    if e['type']=='state':state_time=e['sdk_mono_s']
                if now-state_time>.25:raise RuntimeError('SDK 状态反馈超时')
                if elapsed>c['max_duration_s']:raise RuntimeError('总时长超限')
                while idx<len(p['phases']) and elapsed>=p['phases'][idx]['end_s']:
                    old=p['phases'][idx]
                    if old['name'] in ('baseline','hold'):
                        recent=[r for r in rows if r['phase']==old['name'] and r['trial']==old['trial'] and r['t_s']>old['end_s']-1.]
                        if not feedback_settled(recent,c):
                            raise RuntimeError('静止基线/停车保持阶段未稳定，不开始下一档')
                    idx+=1
                    if idx<len(p['phases']):event({'type':'phase','phase':p['phases'][idx]})
                if idx==len(p['phases']):outcome='completed';break
                phase=p['phases'][idx];vel=c['direction']*command(phase,elapsed)
                if controls.poll():raise RuntimeError('操作者请求停车')
                pipe.send({'type':'command','velocity':vel})
                for r in received:
                    budget_error=budget.update(r)
                    if math.hypot(r['vx'],r['vy'])>c['max_speed_m_s']*1.5+.02 or abs(r['wz'])>c['max_yaw_rate_rad_s']*1.5+.03:raise RuntimeError('实测速度超过试验范围')
                    sample={**r,'t_s':r['mono_s']-start,'phase':phase['name'],'trial':phase['trial'],'level':phase['level'],
                            'phase_elapsed_s':r['mono_s']-start-phase['start_s'],'command_velocity':vel}
                    rows.append(sample)
                    if writer is None:writer=csv.DictWriter(sf,fieldnames=sample);writer.writeheader()
                    writer.writerow(sample)
                    if budget_error:raise RuntimeError(budget_error)
                received.clear()
                for r in reader.reference.pending:
                    sample={**r,'t_s':r['mono_s']-start,'phase':phase['name'],'trial':phase['trial'],'level':phase['level'],
                            'phase_elapsed_s':r['mono_s']-start-phase['start_s'],'command_velocity':vel}
                    reference_rows.append(sample)
                    if reference_writer is None:reference_writer=csv.DictWriter(rf,fieldnames=sample);reference_writer.writeheader()
                    reference_writer.writerow(sample)
                reader.reference.pending.clear()
        except (Exception,KeyboardInterrupt) as exc:
            if controls.poll():outcome='operator_stopped'
            error=str(exc);event({'type':'abort','error':error,'sdk_stop_request_returned':stop_receipt});print(f'中止：{error}',file=sys.stderr)
        finally:
            controls.close()
            if controls.poll():
                event({'type':'operator_stop_request','reason':controls.reason or 'STOP file observed by SDK',
                       'requested_mono_s':controls.requested_mono_s})
            observed=[];window=[];settled_after_stop=False
            def collect_stop_feedback():
                nonlocal settled_after_stop
                rclpy.spin_once(node,timeout_sec=.001)
                for _ in range(16):rclpy.spin_once(node,timeout_sec=0.)
                for row in received:
                    if stop_receipt and row['stamp_s']>=stop_receipt_wall:
                        observed.append(row);window.append(row)
                        while window and row['stamp_s']-window[0]['stamp_s']>1.:window.pop(0)
                        if feedback_settled(window,c):settled_after_stop=True
                        else:settled_after_stop=False
                received.clear();reader.reference.pending.clear()
            worker_exited=None
            if worker is not None:
                try:pipe.send({'type':'finish' if outcome=='completed' else 'abort'})
                except (BrokenPipeError,EOFError,OSError):pass
                worker_exited=await_worker_exit(worker,pipe,event,collect_stop_feedback)
                event({'type':'sdk_exit','exited':worker_exited,'stop_request_returned':stop_receipt})
                if not worker_exited:
                    event({'type':'sdk_exit_timeout','error':'SDK process did not exit within 5 seconds'})
                    print('SDK 进程 5 秒内未退出；停车回执与静止反馈分别记录，请现场确认。',file=sys.stderr)
                    worker.terminate();worker.join(timeout=1)
                    outcome='sdk_exit_timeout'
                pipe.close()
                if not stop_receipt and preinit_failure and not worker.is_alive():
                    outcome='initialization_failed'
                    print('SDK 导入/路径检查失败：未创建 SDK 对象，该子进程未发送运动指令，无需等待其停车回执。',file=sys.stderr)
                elif not stop_receipt:
                    outcome='stop_unconfirmed'
                    print('未收到 SDK 停止请求返回记录，请现场确认并使用独立急停。',file=sys.stderr)
            stop_observation={'status':'not_observed','note':'feedback threshold is not independent physical stop proof'}
            if worker is not None and stop_receipt:
                until=time.monotonic()+3.
                while time.monotonic()<until:
                    collect_stop_feedback()
                    if reader.primary.error:break
                    if (settled_after_stop and reader.primary.latest is not None
                            and time.monotonic()-reader.primary.latest['mono_s']<=reader.primary.timeout):
                        stop_observation['status']='manufacturer_feedback_settled';break
                with (out/'stop_feedback.jsonl').open('x') as f:
                    for row in observed:f.write(json.dumps(row,allow_nan=False)+'\n')
                if stop_observation['status']!='manufacturer_feedback_settled':
                    stop_observation['status']='unconfirmed';print('停车请求已返回，但厂家反馈未确认持续静止；请现场确认，必要时使用物理急停。',file=sys.stderr)
                    if outcome=='completed':outcome='stop_unconfirmed'
                else:print('厂家反馈已连续 1 秒低于停车阈值；本次测试已结束。',flush=True)
            node.destroy_node();rclpy.shutdown()
            for sig,handler in old_handlers.items():signal.signal(sig,handler)
            report=summarize(rows,c)
            report['feedback_health']=reader.report()
            report['slam_reference']=summarize(reference_rows,c) if not reader.reference.error else {'status':'invalid_source','error':reader.reference.error}
            report['ground_truth_verified']=False
            report['failure_before_sdk_initialization']=preinit_failure
            report['operator_stop']={'requested':controls.event.is_set() or controls.stop_file.exists(),
                                     'reason':controls.reason,'requested_mono_s':controls.requested_mono_s}
            report['stop_observation']={**stop_observation,'feedback_error':reader.primary.error,
                'effective_speed_m_s':stop_limits(c)[0],'effective_yaw_rate_rad_s':stop_limits(c)[1]}
            report['sdk_worker_exited']=worker_exited
            report['motion_budget']=budget.report()
            report.update({'execution_status':outcome,'error':error,'sdk_stop_request_returned':stop_receipt})
            if outcome!='completed':
                report['minimum_continuous_motion_candidate']=None
                report['slam_reference']['minimum_continuous_motion_candidate']=None
            dump(out/'summary.json',report)
    return 0 if outcome=='completed' else 1


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',type=Path,default=DEFAULT);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--execute',action='store_true',help='创建 SDK 会话并执行真机运动；默认只输出计划')
    ap.add_argument('--probe-only',type=float,metavar='SECONDS',help='只读检查厂家与 SLAM 两路反馈，不创建 SDK 会话')
    ap.add_argument('--analyze',type=Path,help='仅分析已有 samples.csv，不连接硬件')
    args=ap.parse_args()
    try:
        if sum((args.execute,args.analyze is not None,args.probe_only is not None))>1:raise ValueError('execute/analyze/probe-only 互斥')
        if args.probe_only is not None and (not math.isfinite(args.probe_only) or args.probe_only<=0):raise ValueError('probe-only 时长必须为正有限数')
        c=validate(json.loads(args.config.read_text()));p=plan(c)
        args.output.mkdir(parents=True,exist_ok=False)
        dump(args.output/'config.json',c);dump(args.output/'plan.json',p)
        dump(args.output/'metadata.json',{'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'environment':{k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','ASTRIBOT_SDK_ROOT','PYTHONPATH')},
             'execute':args.execute,'hardware_validated':False})
        if args.analyze:
            if args.execute:raise ValueError('analyze 与 execute 不能同时使用')
            with args.analyze.open() as f:
                rows=[{k:(v if k in ('phase','source') else int(v) if k in ('trial','stamp_ns') else float(v)) for k,v in r.items()} for r in csv.DictReader(f)]
            report=summarize(rows,c)
            report['analysis_only']=True
            original=args.analyze.parent/'summary.json'
            if original.exists():
                execution=json.loads(original.read_text())
                report['original_execution_status']=execution.get('execution_status','unknown')
                report['original_error']=execution.get('error')
                report['offline_candidate_before_execution_review']=report.get('minimum_continuous_motion_candidate')
                if execution.get('execution_status')!='completed':
                    report['minimum_continuous_motion_candidate']=None
            else:
                report['original_execution_status']='unknown'
                report['offline_candidate_before_execution_review']=report.get('minimum_continuous_motion_candidate')
                report['minimum_continuous_motion_candidate']=None
            budget=MotionBudget(c);violations=[]
            for row in rows:
                failure=budget.update(row)
                if failure and not violations:violations.append({'t_s':row['t_s'],'error':failure})
            report['motion_budget_replay']={**budget.report(),'first_violation':violations}
            dump(args.output/'summary.json',report);return 0
        print(f'计划 {p["duration_s"]:.1f} 秒，积分行程/转角 {p["integrated_axis_travel"]:.3f}；输出 {args.output}')
        if args.probe_only is not None:return probe_feedback(c,args.output,args.probe_only)
        if not args.execute:return 0
        if not c['sdk_axis_mapping_verified'] or not c['exclusive_control_verified'] or not all(c['conditions'].values()):
            raise ValueError('执行前需填写工况并确认 SDK 轴映射及唯一控制入口；详见操作手册')
        return execute(c,p,args.output)
    except (ValueError,KeyError,OSError) as exc:
        print(f'失败：{exc}',file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
