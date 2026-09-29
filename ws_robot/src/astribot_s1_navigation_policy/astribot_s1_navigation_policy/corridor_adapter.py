"""Map annotations and fused metric observations feed the corridor domain policy."""
import json
import math
import numpy as np
from .world_geometry import prediction_rows
from pathlib import Path
from dataclasses import replace

from rclpy.time import Time
from .corridor import Corridor, CorridorPolicy, Passage
from .contracts import require
from .observer_node import yaw


class CorridorAdapter:
    def __init__(self, node):
        self.node = node
        node.declare_parameter('corridor_file', '')
        self.automatic=node.get_parameter('navigation_policy_stage').value=='p5'
        node.declare_parameter('automatic_corridor_max_width_m',1.8)
        node.declare_parameter('automatic_corridor_min_length_m',.8)
        self.auto_key=None
        filename = node.get_parameter('corridor_file').value
        require(bool(filename) or self.automatic, 'p4_requires_corridor_file')
        require(not self.automatic or node.profile.environment=='simulation','p5_requires_separate_hardware_validation')
        data = json.loads(Path(filename).read_text()) if filename else dict(schema_version=1,frame_id='map',environment='simulation',corridors=[])
        require(data.get('schema_version') == 1 and data.get('frame_id') == 'map', 'corridor.schema/frame')
        require(data.get('environment') == node.profile.environment, 'corridor.environment')
        self.annotations = tuple(Corridor(
            item['corridor_id'], tuple(item['entry']), tuple(item['exit']), item['width_m'],
            tuple(item['postures']), item.get('boundary_margin_m', .025),
            item.get('tracking_margin_m', .05), item.get('bidirectional', True))
            for item in data['corridors'])
        self.manual_annotations=self.annotations
        self.fixed=node.get_parameter('navigation_geometry_mode').value=='fixed_v2'
        from .fixed_corridor import FixedCorridorPolicy
        self.policy = (FixedCorridorPolicy if self.fixed else CorridorPolicy)(node.profile, self.annotations)
        if self.fixed:
            from astribot_navigation_msgs.msg import PassageAssessment
            self.assessment_pub=node.create_publisher(PassageAssessment,'/navigation/passage_assessment',10)
        self.invalid_since = None
        self.clock_epoch = None
        self.grid_cache = None
        self.grid_result = False
        self.last_error = ''
        self.evidence = {}

    def map_clear(self, corridor, to_map):
        n = self.node;m = n.map
        if m is None:return False
        key = (id(m), corridor, n.execution.version.envelope_epoch)
        if self.grid_cache == key and self.grid_message is m:return self.grid_result
        # Include cell extent, full body, margins and both entry/exit transitions.
        info = m.info;r = info.resolution
        margin = self.policy.margin(corridor)
        half_width = self.policy.half_projection(n.profile.narrow_heading_limit_rad)+margin
        shift=0.
        if self.fixed:
            _,_,right,left=self.policy.support_bounds()
            half_width=(left-right)/2+margin
            shift=self.policy.target_offset(corridor)+(left+right)/2
            # Bound all allowed yaw errors, including extrema between samples.
            half_width+=math.hypot(n.profile.half_length_m,n.profile.half_width_m)*n.profile.narrow_heading_limit_rad
        half_length = math.hypot(n.profile.half_length_m, n.profile.half_width_m)+margin
        origin = info.origin.position;theta = yaw(info.origin.orientation)
        c, s = math.cos(theta), math.sin(theta)
        step = r/2
        length = corridor.length+2*half_length
        okay = True
        for i in range(math.ceil(length/step)+1):
            along = -half_length+min(length, i*step)
            for j in range(math.ceil(2*half_width/step)+1):
                side = shift-half_width+min(2*half_width, j*step)
                x, y = corridor.point(along, side)
                x, y, _ = n.point((x,y,0.), to_map)
                dx, dy = x-origin.x, y-origin.y
                ix = math.floor((c*dx+s*dy)/r);iy = math.floor((-s*dx+c*dy)/r)
                if not (0 <= ix < info.width and 0 <= iy < info.height) or not 0 <= m.data[iy*info.width+ix] < 65:
                    okay = False;break
            if not okay:break
        self.grid_cache = key;self.grid_message = m;self.grid_result = okay
        return okay

    def geometry_clear(self, corridor, to_map):
        if self.fixed:
            offset=self.policy.target_offset(corridor)
            return self.fixed_sweep_clear(corridor,to_map,
                (*corridor.point(0.,offset),corridor.heading),
                (*corridor.point(corridor.length,offset),corridor.heading))
        if not self.map_clear(corridor, to_map):
            self.evidence = {'reason': 'UNKNOWN_OR_OCCUPIED_MAP_SWEEP'}
            return False
        n = self.node
        lateral = self.policy.half_projection(n.profile.narrow_heading_limit_rad)+self.policy.margin(corridor)
        shift=0.
        if self.fixed:
            _,_,right,left=self.policy.support_bounds()
            lateral=(left-right)/2+self.policy.margin(corridor)
            lateral+=math.hypot(n.profile.half_length_m,n.profile.half_width_m)*n.profile.narrow_heading_limit_rad
            shift=self.policy.target_offset(corridor)+(left+right)/2
        longitudinal = math.hypot(n.profile.half_length_m, n.profile.half_width_m)+self.policy.margin(corridor)
        c, s = abs(math.cos(corridor.heading)), abs(math.sin(corridor.heading))
        rows=prediction_rows(n.last_world,include_current=True)
        centers=(rows.lower+rows.upper)/2;half=(rows.upper-rows.lower)/2
        dx=centers[:,0]-corridor.entry[0];dy=centers[:,1]-corridor.entry[1]
        cosine,sine=math.cos(corridor.heading),math.sin(corridor.heading)
        along=cosine*dx+sine*dy;side=-sine*dx+cosine*dy
        axial=c*half[:,0]+s*half[:,1];side_bound=lateral+s*half[:,0]+c*half[:,1]
        hits=np.flatnonzero((along>=-longitudinal-axial)&
                           (along<=corridor.length+longitudinal+axial)&(np.abs(side-shift)<=side_bound))
        if hits.size:
            i=int(hits[0]);track=n.last_world.tracks[int(rows.owners[i])]
            self.evidence=dict(reason='PREDICTED_OCCUPANCY',track=track.fused_track_id,
                along_m=float(along[i]),lateral_m=float(side[i]),lateral_bound_m=float(side_bound[i]),
                obstacle_lower=rows.lower[i].tolist(),obstacle_upper=rows.upper[i].tolist())
            return False
        self.evidence = {'reason': 'MAP_AND_PREDICTED_SWEEP_CLEAR'}
        return True

    def fixed_sweep_clear(self, corridor, to_map, start, end):
        from astribot_s1_robot_geometry._geometry_native import passage_sweep,passage_grid_clear,box_distance
        n=self.node;m=n.map
        if m is None:return False
        sweep=passage_sweep(n.profile.footprint_xy,np.asarray(start),np.asarray(end),self.policy.margin(corridor))
        mapped=np.asarray([n.point((*p,0.),to_map)[:2] for p in sweep])
        info=m.info;o=info.origin
        if not passage_grid_clear(mapped,np.array([o.position.x,o.position.y,yaw(o.orientation)]),
                info.resolution,info.width,info.height,m.data):
            self.evidence={'reason':'UNKNOWN_OR_OCCUPIED_MAP_SWEEP'}
            return False
        rows=prediction_rows(n.last_world,include_current=True)
        boxes=np.column_stack((np.zeros((len(rows.lower),3)),rows.lower[:,:2],rows.upper[:,:2]))
        hits=np.flatnonzero(box_distance(sweep,boxes)<=0.)
        if hits.size:
            i=int(hits[0]);track=n.last_world.tracks[int(rows.owners[i])]
            self.evidence=dict(reason='PREDICTED_OCCUPANCY',track=track.fused_track_id,
                prediction_offset_ns=int(rows.offsets_ns[i]),obstacle_lower=rows.lower[i].tolist(),obstacle_upper=rows.upper[i].tolist())
            return False
        self.evidence={'reason':'MAP_AND_PREDICTED_SWEEP_CLEAR'}
        return True

    def rotation_clear(self, corridor, to_map, target=None):
        if self.fixed:
            r=self.node.last_robot
            end=(*target,r.yaw) if target is not None else (r.x,r.y,corridor.heading)
            return self.fixed_sweep_clear(corridor,to_map,(r.x,r.y,r.yaw),end)
        from .corridor_detection import Grid
        n=self.node;r=n.last_robot;m=n.map
        if r is None or m is None:return False
        info=m.info;o=info.origin
        grid=Grid(info.width,info.height,info.resolution,(o.position.x,o.position.y,yaw(o.orientation)),m.data)
        radius=math.hypot(n.profile.half_length_m,n.profile.half_width_m)+self.policy.margin(corridor)
        target=target or (r.x,r.y)
        distance=math.dist((r.x,r.y),target)
        steps=max(1,math.ceil(distance/(info.resolution/2)))
        centers=[(r.x+(target[0]-r.x)*k/steps,r.y+(target[1]-r.y)*k/steps) for k in range(steps+1)]
        extent=radius+info.resolution
        count=math.ceil(extent/info.resolution)
        for cx,cy in centers:
            for i in range(-count,count+1):
                for j in range(-count,count+1):
                    if math.hypot(i,j)*info.resolution>extent:continue
                    x,y,_=n.point((cx+i*info.resolution,cy+j*info.resolution,0.),to_map)
                    if not 0<=grid.at(x,y)<65:return False
        rows=prediction_rows(n.last_world,include_current=True)
        for cx,cy in centers:
            dx=np.maximum(np.maximum(rows.lower[:,0]-cx,0.),cx-rows.upper[:,0])
            dy=np.maximum(np.maximum(rows.lower[:,1]-cy,0.),cy-rows.upper[:,1])
            if np.any(np.hypot(dx,dy)<=radius+distance/(2*steps)):return False
        return True

    def advance(self, selection, valid, now):
        n = self.node
        self.translation_target=None
        if self.clock_epoch != n.execution.version.clock_epoch:
            self.clock_epoch = n.execution.version.clock_epoch
            self.invalid_since = None
        try:
            to_odom = n.tf.lookup_transform(n.profile.tracking_frame, 'map', Time())
            to_map = n.tf.lookup_transform('map', n.profile.tracking_frame, Time())
            if self.automatic and n.map is not None and n.path:
                from .corridor_detection import Grid,detect_corridors
                info=n.map.info;o=info.origin
                maximum=n.get_parameter('automatic_corridor_max_width_m').value
                minimum=n.get_parameter('automatic_corridor_min_length_m').value
                key=(n.execution.version.map_epoch,n.active_path_key,maximum,minimum)
                if key!=self.auto_key:
                    grid=Grid(info.width,info.height,info.resolution,(o.position.x,o.position.y,yaw(o.orientation)),n.map.data)
                    path=tuple(n.point((*point,0.),to_map)[:2] for point in n.path)
                    detected=detect_corridors(grid,path,'simulation_transport',maximum,minimum,n.profile.narrow_heading_limit_rad)
                    self.annotations=self.manual_annotations+detected;self.auto_key=key
            self.policy.corridors = tuple(replace(c,
                entry=n.point((*c.entry,0.), to_odom)[:2],
                exit=n.point((*c.exit,0.), to_odom)[:2]) for c in self.annotations)
            candidate = self.policy.active
            if candidate is None and n.last_robot is not None:
                candidate, _ = self.policy.choose(n.last_robot, n.path)
            translation_path=None
            if self.fixed and candidate is not None and n.active_path_message is not None:
                from astribot_s1_robot_geometry._geometry_native import narrow_translation_end
                source=n.active_path_message
                transform=n.tf.lookup_transform(n.profile.tracking_frame,source.header.frame_id,Time())
                heading=yaw(transform.transform.rotation)
                poses=np.asarray([(*n.point((p.pose.position.x,p.pose.position.y,0.),transform)[:2],
                    yaw(p.pose.orientation)+heading) for p in source.poses])
                end=narrow_translation_end(n.profile.footprint_xy,poses,np.asarray(candidate.entry),
                    np.asarray(candidate.exit),candidate.width_m,self.policy.margin(candidate))
                if end>=0:
                    translation_path=poses[:end+1]
                    self.translation_target=tuple(poses[end])
            self.evidence={}
            if translation_path is not None:
                robot=n.last_robot
                nearest=int(np.argmin(np.linalg.norm(translation_path[:,:2]-np.array([robot.x,robot.y]),axis=1)))
                remaining=[(robot.x,robot.y,translation_path[0,2]),*map(tuple,translation_path[nearest+1:])]
                clear=valid and all(self.fixed_sweep_clear(candidate,to_map,a,b) for a,b in zip(remaining,remaining[1:]))
            else:
                clear = valid and (candidate is None or self.geometry_clear(candidate, to_map))
            transit_evidence=dict(self.evidence)
            beyond_exit=bool(candidate is not None and n.last_robot is not None and
                candidate.coordinates(n.last_robot.x,n.last_robot.y)[0]>candidate.length)
            before_entry=bool(candidate is not None and n.last_robot is not None and
                candidate.coordinates(n.last_robot.x,n.last_robot.y)[0]<0)
            self.evidence={}
            rotation_clear=bool(valid and candidate is not None and
                (not self.policy.permit or beyond_exit or before_entry) and self.rotation_clear(candidate,to_map))
            alignment_evidence=dict(self.evidence)
            self.evidence={}
            centering_clear=False
            if valid and candidate is not None and n.last_robot is not None and before_entry:
                along,lateral=candidate.coordinates(n.last_robot.x,n.last_robot.y)
                target=self.policy.centering_target or candidate.point(along,self.policy.target_offset(candidate) if self.fixed else 0.)
                if abs(candidate.coordinates(*target)[1]-lateral)<=n.profile.narrow_centering_max_offset_m:
                    centering_clear=self.rotation_clear(candidate,to_map,target)
            centering_evidence=dict(self.evidence)
            self.invalid_since=None;self.last_error = ''
        except Exception as error:
            self.last_error = str(error)
            self.policy.clear_at=None
            if self.policy.active is not None:
                self.policy.state='HOLD' if self.policy.permit else 'WAIT'
            if self.invalid_since is None:self.invalid_since=now
            failure='CORRIDOR_BLOCKED: FRAME_OR_MAP_UNAVAILABLE' if now-self.invalid_since>=n.profile.planning_budget['episode_timeout_s'] else ''
            return Passage(replace(selection,motion='HOLD',speed=0.,reason=failure or 'CORRIDOR_FRAME_UNAVAILABLE'),
                           'WAIT',failure=failure)
        envelope = n.profile.envelope
        if self.translation_target is not None:
            self.policy.begin_evaluation(n.execution.version,now)
            robot=n.last_robot;target=self.translation_target
            self.policy.active=candidate
            if math.hypot(robot.x-target[0],robot.y-target[1])<=.008:
                self.policy.active=None;self.policy.permit=();self.policy.state='NORMAL'
                passage=Passage(selection)
            elif not valid or n.last_risk.immediate or n.last_risk.uncertain or not clear:
                passage=self.policy.hold('CORRIDOR_INPUT_UNAVAILABLE' if not valid else
                    (selection.reason if n.last_risk.immediate or n.last_risk.uncertain else
                     'CORRIDOR_TRANSLATION_SWEEP_BLOCKED'),now,selection)
            else:
                # The ordinary forecast rotates the body to the path tangent.
                # This phase instead checked the entire fixed-heading sweep;
                # current/braking collision and uncertain observations still hold.
                self.policy.state='TRANSLATE';self.policy.wait_at=None
                passage=Passage(replace(selection,motion='SLOW',speed=n.profile.narrow_speed_m_s,
                    reason='CORRIDOR_FIXED_HEADING_TRANSLATION'),'TRANSLATE',candidate.corridor_id,
                    n.profile.narrow_angular_speed_rad_s,tracking_heading=target[2])
        else:
            passage=self.policy.evaluate(selection, n.last_robot, n.path, n.execution.version,
                envelope.posture_id if envelope is not None else '', valid, clear, now, rotation_clear,centering_clear)
        if self.fixed:
            self.evidence=dict(transit=transit_evidence,alignment=alignment_evidence,centering=centering_evidence)
            if self.translation_target is not None:
                self.evidence['translation_target']=self.translation_target
            from astribot_navigation_msgs.msg import PassageAssessment
            evidence=getattr(self.policy,'assessment',{});self.evidence.update(evidence)
            msg=PassageAssessment();msg.header.stamp=n.get_clock().now().to_msg();msg.header.frame_id=n.profile.tracking_frame
            envelope=n.profile.v2
            if envelope:
                msg.envelope_epoch=envelope.epoch;msg.installed_geometry_hash=envelope.installed_geometry_hash
            msg.corridor_id=passage.corridor_id;msg.width_m=float(evidence.get('width',0.))
            if candidate is not None:
                msg.corridor_id=candidate.corridor_id;msg.width_m=candidate.width_m
                msg.entry.x,msg.entry.y=candidate.entry;msg.exit.x,msg.exit.y=candidate.exit
                msg.boundary_tracking_margin_m=candidate.boundary_margin_m+candidate.tracking_margin_m
                msg.bidirectional=candidate.bidirectional
            msg.left_clearance_m=float(evidence.get('left_clearance',0.));msg.right_clearance_m=float(evidence.get('right_clearance',0.))
            msg.lateral_target_m=float(evidence.get('target_offset',0.))
            msg.heading_min_rad=-n.profile.narrow_heading_limit_rad;msg.heading_max_rad=n.profile.narrow_heading_limit_rad
            msg.in_place_rotation_allowed=bool(evidence.get('in_place_rotation_allowed',False))
            msg.reverse_allowed=passage.state=='TRANSLATE'
            msg.decision=passage.state;msg.reason=passage.selection.reason;self.assessment_pub.publish(msg)
        return passage
