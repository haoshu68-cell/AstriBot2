"""Bind start maneuvers to a live controller request, path and world generation."""
import math
import time
import threading
import numpy as np
from rclpy.time import Time
from astribot_navigation_msgs.msg import StartManeuver, StartManeuverRequest
from .behavior import Selection
from .path_evidence import path_identity
from .observer_node import yaw
from .world_geometry import prediction_rows
from .start_maneuver import StartManeuverPolicy, ManeuverScene, Maneuver


class StartManeuverAdapter:
    def __init__(self,node):
        self.node=node;self.policy=StartManeuverPolicy(node.profile)
        self.lock=threading.Lock();self.pending=None;self.key=None;self.request_id=None;self.last_error=''
        self.map_cache=None;self.map_cells=None;self.status={}
        node.create_subscription(StartManeuverRequest,'/path_tracking/start_maneuver_request',self.receive,1)
        self.publisher=node.create_publisher(StartManeuver,'/navigation_policy/start_maneuver',1)

    def receive(self,msg):
        with self.lock:self.pending=(msg,time.monotonic())

    def static_boxes(self,robot):
        n=self.node;m=n.map
        if m is None:raise ValueError('MAP_UNAVAILABLE')
        info=m.info
        if self.map_cache is not m:
            cells=np.asarray(m.data).reshape((info.height,info.width))
            self.map_cells=np.column_stack(np.nonzero((cells<0)|(cells>=65)))
            self.map_cache=m
        to_map=n.tf.lookup_transform('map',n.profile.tracking_frame,Time())
        to_odom=n.tf.lookup_transform(n.profile.tracking_frame,'map',Time())
        mx,my,_=n.point((robot.x,robot.y,0.),to_map)
        o=info.origin.position;angle=yaw(info.origin.orientation)
        c,s=math.cos(angle),math.sin(angle);dx,dy=mx-o.x,my-o.y
        gx,gy=c*dx+s*dy,-s*dx+c*dy
        radius=n.profile.start_maneuver['max_distance_m']+math.hypot(n.profile.half_length_m,n.profile.half_width_m)+n.profile.clearance_margin_m+n.profile.payload_extra_margin_m
        resolution=info.resolution
        low=np.floor((np.array((gy,gx))-radius)/resolution).astype(int)
        high=np.ceil((np.array((gy,gx))+radius)/resolution).astype(int)
        cells=self.map_cells
        cells=cells[np.all((cells>=low)&(cells<=high),axis=1)]
        # Outside-map cells remain unknown; never extrapolate free space.
        if np.any(low<0) or high[0]>=info.height or high[1]>=info.width:
            yy,xx=np.mgrid[low[0]:high[0]+1,low[1]:high[1]+1]
            outside=(yy<0)|(yy>=info.height)|(xx<0)|(xx>=info.width)
            cells=np.concatenate((cells,np.column_stack((yy[outside],xx[outside]))))
        x=(cells[:,1]+.5)*resolution;y=(cells[:,0]+.5)*resolution
        tx,ty,_=n.point((o.x,o.y,0.),to_odom)
        angle+=yaw(to_odom.transform.rotation);c,s=math.cos(angle),math.sin(angle)
        center=np.column_stack((tx+c*x-s*y,ty+s*x+c*y))
        half=.5*resolution*(abs(c)+abs(s))
        return center-half,center+half

    def advance(self,selection,valid):
        n=self.node;now=n.stamp();wall=time.monotonic()
        with self.lock:pending=self.pending
        if pending is None:return selection,False
        req,received=pending
        age=(now.ns-(req.stamp.sec*10**9+req.stamp.nanosec))*1e-9
        if not 0<=age<=n.profile.sensor_timeout_s or (n.profile.environment!='simulation' and wall-received>n.profile.sensor_timeout_s):
            self.status={'state':'IDLE','reason':'REQUEST_EXPIRED'}
            return selection,False
        # The controller request may arrive before its new active-path topic.
        # Even that WAIT must belong to the new request, not an old request's
        # expired/failed maneuver. Full world/path identity is still checked
        # below before any motion permit can be issued.
        if req.request_id!=self.request_id:
            self.request_id=req.request_id;self.key=None;self.policy.reset()
        try:
            identity=path_identity(req.reference_path)
            if identity!=n.active_path_key:raise ValueError('ACTIVE_PATH_PENDING')
            v=n.last_world.version
            key=(req.request_id,identity,v.goal_id,v.map_epoch,v.localization_epoch,v.envelope_epoch,v.clock_epoch)
            if key!=self.key:
                self.key=key;self.policy.reset()
            if not n.profile.start_maneuver['enabled']:
                decision=Maneuver('FAILED','START_HEADING_UNREACHABLE: MANEUVER_NOT_VALIDATED')
            elif not valid or n.last_robot is None:
                decision=self.policy.step(None,None,0.,now.ns*1e-9,False,False,False)
            else:
                transform=n.tf.lookup_transform(n.profile.tracking_frame,req.reference_path.header.frame_id,Time())
                heading=req.target_heading_rad+yaw(transform.transform.rotation)
                if not math.isfinite(heading):raise ValueError('INVALID_START_HEADING')
                r=n.last_robot
                pose=req.current_pose
                if pose.header.frame_id!=req.reference_path.header.frame_id:raise ValueError('REQUEST_FRAME_MISMATCH')
                px,py,_=n.point((pose.pose.position.x,pose.pose.position.y,0.),transform)
                if math.dist((px,py),(r.x,r.y))>.04:raise ValueError('REQUEST_POSE_MOVED')
                # Normal tracking forecasts have a shorter region of relevance.
                # Generate forecasts for the whole bounded exit search as well.
                world=n.fusion.snapshot(now,(r.x,r.y,n.profile.start_maneuver['max_distance_m']))
                rows=prediction_rows(world,include_current=True,swept=True)
                lower,upper=self.static_boxes(r)
                scene=ManeuverScene(np.concatenate((lower,rows.lower)),np.concatenate((upper,rows.upper)),n.profile)
                rear=n.health_registry.allows_motion(now,-.05,0.,0.)
                rotation=n.health_registry.allows_motion(now,0.,0.,.1 if abs(math.remainder(heading-r.yaw,2*math.pi))>.001 else 0.)
                motion_safe=n.last_risk is not None and not n.last_risk.immediate and not n.last_risk.uncertain
                decision=self.policy.step(scene,r,heading,now.ns*1e-9,valid and not world.unassociated,
                                          rear,rotation,motion_safe)
            self.last_error=''
        except Exception as error:
            self.last_error=str(error)
            decision=self.policy.step(None,None,0.,now.ns*1e-9,False,False,False)
            if decision.mode!='FAILED':decision=Maneuver('WAIT','START_MANEUVER_'+str(error))
        msg=StartManeuver();msg.stamp=n.get_clock().now().to_msg();msg.request_id=req.request_id
        msg.lease_s=min(.3,n.profile.constraint_lease_s);msg.mode=getattr(StartManeuver,decision.mode)
        msg.reason=decision.reason;msg.evaluated_pose=req.current_pose
        if decision.mode=='REVERSE':
            msg.command.linear.x=-decision.speed;msg.command.linear.y=decision.lateral
            msg.command.angular.z=decision.angular
        self.publisher.publish(msg)
        self.status=dict(state=self.policy.state,mode=decision.mode,reason=decision.reason,
                         request_id=req.request_id,evidence=self.policy.evidence)
        if decision.mode=='TURN':
            # A turn permit grants no ownership of route planning. A blocked
            # forward path must still be assessed/rejoined after the retreat.
            return selection,False
        if decision.mode=='REVERSE':
            return Selection('SLOW',math.hypot(decision.speed,decision.lateral),decision.reason,episode=selection.episode),True
        return Selection('HOLD',0.,decision.reason,episode=selection.episode),True
