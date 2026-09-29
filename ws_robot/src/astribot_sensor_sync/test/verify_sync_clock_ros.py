#!/usr/bin/env python3
"""Isolated /clock pause/rollback and default simulated-evidence rejection tests."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from astribot_sensor_sync.msg import TriggerEdge,SensorTiming,SyncStatus

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Node('clock_sync_test',namespace='/sync_clock_probe');events=[];checks=[]
    clock=n.create_publisher(Clock,'clock',10)
    edges=n.create_publisher(TriggerEdge,'sensor_sync/trigger_edges',256)
    frames=n.create_publisher(SensorTiming,'sensor_sync/frame_timing',128)
    sub=n.create_subscription(SyncStatus,'sensor_sync/status',events.append,128)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.005)
    def stamp(seconds):
        c=Clock();c.clock.sec=seconds;clock.publish(c);spin(.03)
    def expect(label,reason):
        until=time.monotonic()+2
        while time.monotonic()<until:
            if any(x.reason==reason for x in events):checks.append(dict(case=label,reason=reason,passed=True));events.clear();return
            spin(.005)
        raise AssertionError((label,reason,[(x.frame_sequence,x.reason)for x in events[-10:]]))
    error=None
    for simulation in (True,False):
        log=(a.output/('monitor_sim.log' if simulation else 'monitor_default.log')).open('w')
        child=subprocess.Popen(['ros2','run','astribot_sensor_sync','sync_monitor','--ros-args','-r','__ns:=/sync_clock_probe','-r','/clock:=/sync_clock_probe/clock','-p','sources:=[head_rgbd/depth]','-p','reference_clock_epoch:=probe-clock','-p','use_sim_time:=true','-p',f'allow_simulated:={str(simulation).lower()}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+8
            while edges.get_subscription_count()<1 or frames.get_subscription_count()<1 or clock.get_subscription_count()<1 or n.count_publishers('/sync_clock_probe/sensor_sync/status')<1:
                if time.monotonic()>end:raise RuntimeError('discovery timeout')
                spin(.02)
            stamp(2)
            end=time.monotonic()+2
            while not any(x.evaluated_stamp_ns==2000000000 for x in events):
                if time.monotonic()>end:raise RuntimeError('monitor did not observe simulated clock')
                stamp(2)
            events.clear()
            t=TriggerEdge(group='bench',trigger_epoch='run1',clock_epoch='probe-clock',sequence=1,stamp_ns=2000000000,simulated=True)
            f=SensorTiming(source_id='head_rgbd/depth',source_epoch='device1',group='bench',trigger_epoch='run1',clock_epoch='probe-clock',frame_sequence=1,trigger_sequence=1,capture_stamp_ns=2000000000,clock_uncertainty_ns=1000,clock_locked=True,hardware_associated=True,simulated=True)
            edges.publish(t);frames.publish(f)
            if not simulation:expect('hardware_default_rejects_simulation','SIMULATED_EVIDENCE');continue
            expect('sim_clock_valid','SIMULATED_TRIGGER_MATCH')
            spin(.3);expect('paused_clock_wall_watchdog','SOURCE_TIMEOUT')
            stamp(1);expect('clock_rollback_without_new_frame','CLOCK_ROLLBACK')
            stamp(3);t.sequence=2;t.stamp_ns=3000000000;f.frame_sequence=2;f.trigger_sequence=2;f.capture_stamp_ns=3000000000
            edges.publish(t);frames.publish(f);expect('rollback_stays_latched','CLOCK_ROLLBACK')
        except Exception as exc:error=repr(exc)
        finally:
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
            log.close();spin(.1);events.clear()
        if error:break
    n.destroy_node();rclpy.shutdown()
    report=dict(evidence='isolated_synthetic_clock_only',hardware_tested=False,passed=not error,checks=checks,error=error)
    (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return int(bool(error))
if __name__=='__main__':raise SystemExit(main())
