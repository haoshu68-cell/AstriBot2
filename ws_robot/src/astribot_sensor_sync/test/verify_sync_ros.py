#!/usr/bin/env python3
"""Synthetic ROS metadata tests, not physical pulse/exposure or Gazebo sensor validation."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from astribot_sensor_sync.msg import TriggerEdge,SensorTiming,SyncStatus

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Node('sync_contract_probe',namespace='/sync_contract_probe')
    results=[];received=[]
    sub=node.create_subscription(SyncStatus,'sensor_sync/status',received.append,128)
    edges=node.create_publisher(TriggerEdge,'sensor_sync/trigger_edges',256)
    frames=node.create_publisher(SensorTiming,'sensor_sync/frame_timing',128)
    command=['ros2','run','astribot_sensor_sync','sync_monitor','--ros-args','-r','__ns:=/sync_contract_probe',
             '-p','sources:=[head_rgbd/depth, imu]','-p','allow_simulated:=true','-p','reference_clock_epoch:=probe-clock']
    log=(args.output/'monitor.log').open('w')
    child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
    def expect(label,reason,sequence=None):
        end=time.monotonic()+2
        while time.monotonic()<end:
            for s in received:
                if s.reason==reason and (sequence is None or s.frame_sequence==sequence):
                    results.append(dict(case=label,passed=True,reason=s.reason,valid=s.valid,hardware=s.hardware_trigger_verified))
                    return s
            rclpy.spin_once(node,timeout_sec=.01)
        raise AssertionError((label,reason,[(s.frame_sequence,s.reason) for s in received[-12:]]))
    def pair(seq,**updates):
        now=node.get_clock().now().nanoseconds-1000000
        t=TriggerEdge(group='bench',trigger_epoch='run1',clock_epoch='probe-clock',sequence=seq,stamp_ns=now,simulated=True)
        f=SensorTiming(source_id='head_rgbd/depth',source_epoch='device1',group='bench',trigger_epoch='run1',clock_epoch='probe-clock',frame_sequence=seq,trigger_sequence=seq,capture_stamp_ns=now+100,clock_uncertainty_ns=1000,clock_locked=True,hardware_associated=True,simulated=True)
        for k,v in updates.items():setattr(f,k,v)
        return t,f
    error=None
    try:
        end=time.monotonic()+8
        while edges.get_subscription_count()<1 or frames.get_subscription_count()<1:
            if time.monotonic()>end or child.poll() is not None:raise RuntimeError('monitor discovery failed')
            spin(.02)
        expect('missing_source','MISSING_SOURCE')
        t,f=pair(1);edges.publish(t);frames.publish(f)
        s=expect('matched_simulated_edge','SIMULATED_TRIGGER_MATCH',1);assert s.valid and not s.hardware_trigger_verified
        frames.publish(f);expect('duplicate_frame','NON_MONOTONIC_FRAME',1)
        t,f=pair(2);frames.publish(f);spin(.01);edges.publish(t);expect('frame_before_edge','SIMULATED_TRIGGER_MATCH',2)
        t,f=pair(3,hardware_associated=False);edges.publish(t);frames.publish(f);expect('no_association','NO_HARDWARE_ASSOCIATION',3)
        t,f=pair(4,clock_locked=False);edges.publish(t);frames.publish(f);expect('clock_unlock','CLOCK_UNLOCKED',4)
        t,f=pair(5,capture_stamp_ns=node.get_clock().now().nanoseconds-500000000);edges.publish(t);frames.publish(f);expect('old_acquisition','STALE_SAMPLE',5)
        t,f=pair(6,clock_epoch='unrelated');edges.publish(t);frames.publish(f);expect('different_clock','CLOCK_EPOCH_MISMATCH',6)
        t,f=pair(7);frames.publish(f);expect('lost_trigger','TRIGGER_MISSING',7)
        t,f=pair(8,source_id='imu',trigger_sequence=0,hardware_associated=False);frames.publish(f)
        s=expect('imu_clock_only','CLOCK_ONLY',8);assert s.valid and not s.hardware_trigger_verified
        t,f=pair(9);edges.publish(t);frames.publish(f);expect('recovered_frame','SIMULATED_TRIGGER_MATCH',9)
        spin(.3);s=expect('source_stops','SOURCE_TIMEOUT',9);assert not s.valid
        t,f=pair(10);edges.publish(t);frames.publish(f);expect('fresh_recovery','SIMULATED_TRIGGER_MATCH',10)
        t.stamp_ns+=1;edges.publish(t);expect('conflicting_trigger','TRIGGER_INVALID_OR_CONFLICT')
    except Exception as exc:error=repr(exc)
    finally:
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()
        report=dict(evidence='isolated_synthetic_ros_metadata_only',hardware_tested=False,gazebo_exposure_tested=False,domain=os.getenv('ROS_DOMAIN_ID'),checks=results,passed=error is None,error=error,child_returncode=child.returncode)
        (args.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 1 if error else 0
if __name__=='__main__':raise SystemExit(main())
