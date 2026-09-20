"""Validated social observations restrict the existing policy, never publish Twist."""
import json
import math
import threading
from dataclasses import asdict
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from astribot_navigation_msgs.msg import SocialAgentArray
from astribot_navigation_msgs.srv import ResolveRoute
from astribot_s1_social_navigation.contracts import validate_sample, SampleOrder, stamp_ns
from astribot_s1_social_navigation.observer_node import transform_sample
from .contracts import MetricBox, Vec3, Covariance3
from .ports import TrackedObstacle, PredictionModel, WorldSnapshot
from .social_behavior import SocialPolicy
from .stop_reference import describe_stop_reference


class SocialAdapter:
    def __init__(self,node):
        self.node=node
        default=get_package_share_directory('astribot_s1_navigation_policy')+'/config/social.json'
        node.declare_parameter('social_profile',default)
        node.declare_parameter('social_allow_simulation_truth',False)
        self.allow_truth=bool(node.get_parameter('social_allow_simulation_truth').value)
        if self.allow_truth and not node.get_parameter('use_sim_time').value:
            raise ValueError('Social simulation truth requires simulated time')
        self.config=json.loads(Path(node.get_parameter('social_profile').value).read_text())
        if self.config.get('schema_version')!=1:raise ValueError('social schema_version')
        for key,value in self.config.items():
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid social parameter '+key)
        if type(self.config['max_people']) is not int:raise ValueError('max_people must be integer')
        self.policy=SocialPolicy(node.profile,self.config)
        self.sample=None;self.pending=None;self.order=SampleOrder();self.epoch=None
        self.input_lock=threading.Lock();self.input_group=MutuallyExclusiveCallbackGroup()
        self.context_lock=threading.Lock();self.context_group=MutuallyExclusiveCallbackGroup()
        self.pending_context=None;self.response_snapshot=None
        self.reason='SOCIAL_INPUT_UNAVAILABLE';self.context=None
        self.decision=None;self.last_diagnostic=-math.inf;self.last_reason=None
        self.status=node.create_publisher(String,'/social_navigation/behavior_status',10)
        node.create_subscription(SocialAgentArray,'/social_navigation/observed_agents',self.receive,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT),callback_group=self.input_group)
        # P2 has no candidate coordinator. Reuse the existing BT route result
        # channel for bounded failure, without issuing replacement paths.
        if node.coordinator is None:
            node.create_service(ResolveRoute,'/navigation_policy/resolve_route',self.resolve,
                                callback_group=self.context_group)

    def resolve(self,req,res):
        res.evaluated_at=self.node.get_clock().now().to_msg()
        if (not req.session_id or not req.reference_path.poses or
            req.goal.pose!=req.reference_path.poses[-1].pose or
            req.goal.header.frame_id!=req.reference_path.header.frame_id):
            res.disposition=res.BLOCKED;res.reason='INVALID_EXECUTION_CONTEXT';return res
        key=(req.session_id,self.node.epoch)
        point=req.goal.pose.position;q=req.goal.pose.orientation
        goal=(req.goal.header.frame_id,point.x,point.y,
              math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        with self.context_lock:
            self.pending_context=(key,self.node.get_clock().now().nanoseconds,goal)
            snapshot=self.response_snapshot
        if snapshot and snapshot[0]==key and snapshot[1]:
            res.disposition=res.BLOCKED;res.reason=snapshot[1]
        else:res.reason='KEEP_CURRENT_PATH_SOCIAL_WAIT'
        return res

    def receive(self,message):
        # Receive independently of risk evaluation; a slow cycle must not drain
        # a queue of increasingly old observations on subsequent cycles.
        with self.input_lock:self.pending=message

    def observations(self,now):
        n=self.node;c=self.config
        if self.epoch!=now.epoch:
            self.sample=None;self.order=SampleOrder();self.policy.reset();self.epoch=now.epoch
            self.reason='SOCIAL_CLOCK_CHANGED'
        with self.input_lock:
            msg=self.pending;self.pending=None
        if msg is not None:
            age_ns=now.ns-stamp_ns(msg.header.stamp)
            if age_ns>=0 or not n.get_parameter('use_sim_time').value or age_ns < -50_000_000:
                error=validate_sample(msg,now.ns,c['observation_timeout_s'],self.allow_truth)
                if not error:
                    error=self.order.accept(msg)
                if not error:
                    try:
                        sample=(msg if msg.header.frame_id==n.profile.tracking_frame else
                            transform_sample(msg,n.tf.lookup_transform(n.profile.tracking_frame,
                                msg.header.frame_id,Time.from_msg(msg.header.stamp)),n.profile.tracking_frame))
                        self.sample=sample;self.reason=''
                    except Exception:self.sample=None;self.reason='SOCIAL_CAPTURE_TF_UNAVAILABLE'
                elif error!='DUPLICATE_OR_OUT_OF_ORDER':self.sample=None;self.reason=error
            else:
                with self.input_lock:
                    if self.pending is None:self.pending=msg
        if self.sample is None:return None,self.reason
        error=validate_sample(self.sample,now.ns,c['observation_timeout_s'],self.allow_truth)
        if error:return None,error
        age=(now.ns-stamp_ns(self.sample.header.stamp))*1e-9
        if len(self.sample.agents)>c['max_people']:return None,'SOCIAL_CAPACITY_EXCEEDED'
        tracks=[];p=n.profile
        steps=tuple((int(round(t*1e9)),float(t)) for t in np.arange(p.prediction_step_s,
                       p.prediction_horizon_s+p.prediction_step_s/2,p.prediction_step_s))
        for agent in self.sample.agents:
            if agent.tracking_state==agent.LOST:return None,'SOCIAL_TRACK_LOST'
            v=agent.velocity.twist.linear
            vx,vy=(float(v.x),float(v.y)) if agent.velocity_valid else (0.,0.)
            if math.hypot(vx,vy)>p.max_obstacle_speed_m_s:return None,'SOCIAL_SPEED_OUT_OF_MODEL'
            cov=np.asarray(agent.pose.covariance).reshape(6,6)[:3,:3].copy()
            velocity_var=(float(np.linalg.eigvalsh(np.asarray(agent.velocity.covariance).reshape(6,6)[:2,:2]).max())
                          if agent.velocity_valid else c['unobserved_velocity_variance'])
            cov[:2,:2]+=np.eye(2)*velocity_var*age*age
            position=agent.pose.pose.position
            geometry=MetricBox(Vec3(float(position.x)+vx*age,float(position.y)+vy*age,p.height_m/2),
                Vec3(2*agent.radius,2*agent.radius,p.height_m),Covariance3(tuple(float(x) for x in cov.ravel())))
            identity=f'{self.sample.source_id}:{self.sample.source_epoch}:{agent.track_id}'
            tracks.append(TrackedObstacle(identity,p.tracking_frame,now,geometry,(),
                (self.sample.source_id,),PredictionModel(Vec3(vx,vy,0.),velocity_var,steps)))
        return WorldSnapshot(n.last_world.version,now,p.tracking_frame,tuple(tracks),(),(),self.sample.sequence),''

    def goal_in_tracking_frame(self,goal,now):
        frame,x,y,heading=goal;n=self.node
        if frame==n.profile.tracking_frame:return x,y,heading
        try:
            tf=n.tf.lookup_transform(n.profile.tracking_frame,frame,Time())
            ns=stamp_ns(tf.header.stamp)
            if ns and not 0<=now.ns-ns<=int(n.profile.sensor_timeout_s*1e9):return None
            q=tf.transform.rotation;t=tf.transform.translation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return t.x+math.cos(yaw)*x-math.sin(yaw)*y,t.y+math.sin(yaw)*x+math.cos(yaw)*y,heading+yaw
        except Exception:return None

    def apply(self,selection,valid,now):
        n=self.node;seconds=now.ns*1e-9
        with self.context_lock:context=self.pending_context
        if context and context[0]!=self.context:
            self.context=context[0];self.policy.reset();self.decision=None
        world,error=self.observations(now)
        # No task means no wait budget; a later task must not inherit idle failure.
        active=bool(context and context[0][1]==now.epoch and
            0<=n.get_clock().now().nanoseconds-context[1]<=int(n.profile.planning_takeover['context_timeout_s']*1e9))
        if n.coordinator is not None:active=n.coordinator.context is not None
        if not active:self.policy.reset()
        goal=self.goal_in_tracking_frame(context[2],now) if active and context else None
        self.decision=self.policy.select(selection,world,n.last_robot,n.path,valid and not error,
                                          seconds,error or 'SOCIAL_INPUT_UNAVAILABLE',
                                          goal=goal)
        with self.context_lock:self.response_snapshot=(self.context,self.decision.failure)
        if n.coordinator is not None and self.decision.failure:n.coordinator.failure=self.decision.failure
        if seconds-self.last_diagnostic>=self.config['diagnostic_period_s'] or self.last_reason!=self.decision.reason:
            data=asdict(self.decision);data.update(stamp_ns=now.ns,active=active,
                source='none' if self.sample is None else self.sample.source_id,
                truth=bool(self.sample and self.sample.provenance==SocialAgentArray.SIMULATION_TRUTH),
                people=0 if world is None else len(world.tracks),input_error=error,
                sample_stamp_ns=None if self.sample is None else stamp_ns(self.sample.header.stamp),
                sample_age_s=None if self.sample is None else (now.ns-stamp_ns(self.sample.header.stamp))*1e-9)
            data['stopping_reference']=describe_stop_reference(n.profile,
                math.hypot(n.last_robot.vx,n.last_robot.vy) if n.last_robot else 0.)
            self.status.publish(String(data=json.dumps(data,allow_nan=False)))
            if self.last_reason!=self.decision.reason:
                n.get_logger().info('SOCIAL_BEHAVIOR '+json.dumps(data,allow_nan=False))
            self.last_diagnostic=seconds;self.last_reason=self.decision.reason
        return self.decision.selection
