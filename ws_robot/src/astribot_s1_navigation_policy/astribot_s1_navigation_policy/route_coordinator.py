"""P3 asynchronous candidate arbitration; the behavior tree alone commits paths."""
import math
import time
import copy
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from dataclasses import replace
from rclpy.time import Time
from astribot_navigation_msgs.srv import PlanCandidate, ResolveRoute
from .contracts import Stamp, Version, Trigger, Planning
from .planning_session import PlanningBudget, PlanningSession, PlanningBudgetExhausted
from .path_evidence import path_identity
from .behavior import Selection
from .candidate_safety import candidate_clearance
from .candidate_variants import lateral_variants
from .obstruction_retry import ObstructionRetry


class RouteCoordinator:
    def __init__(self, node):
        self.node=node;self.profile=node.profile;self.takeover=self.profile.planning_takeover
        self.session=PlanningSession(str(node.boot),PlanningBudget(**self.profile.planning_budget))
        self.rpc_group=MutuallyExclusiveCallbackGroup()
        self.client=node.create_client(PlanCandidate,'/path_tracking/plan_candidate',callback_group=self.rpc_group)
        self.mailbox_lock=threading.Lock();self.pending_route=None;self.route_response=None
        self.service=node.create_service(ResolveRoute,'/navigation_policy/resolve_route',self.resolve,
                                        callback_group=self.rpc_group)
        self.context=None;self.reference=None;self.goal=None;self.key=None;self.version=None
        self.last_poll=0.;self.future=None;self.request=None;self.mode=None
        self.options=[];self.chosen=None;self.ready=None;self.blocked_since=None
        self.attempted_episode=None;self.failure=None;self.audit=[];self.holding=False
        self.variants=[];self.variant_keys=set();self.safety_evidence=[];self.retry=ObstructionRetry()

    def steady(self):return Stamp(time.monotonic_ns(),'steady',self.node.epoch)

    def cancel(self):
        if self.future is not None:
            self.client.remove_pending_request(self.future);self.future.cancel()
        self.future=None;self.request=None;self.options=[];self.chosen=None;self.ready=None;self.holding=False
        self.variants=[];self.variant_keys=set()

    @staticmethod
    def request_key(req):
        key=path_identity(req.reference_path)
        if (not req.session_id or not req.reference_path.poses or
            req.goal.pose!=req.reference_path.poses[-1].pose or
            req.goal.header.frame_id!=req.reference_path.header.frame_id):
            raise ValueError('INVALID_EXECUTION_CONTEXT')
        return req.session_id,key,req.allow_detour

    def resolve(self, req, res):
        # Transport never touches the planner state or evaluates geometry. The
        # processing callback is its sole writer; replies carry a bounded proof.
        res.evaluated_at=self.node.get_clock().now().to_msg()
        try:key=self.request_key(req)
        except (ValueError,TypeError) as error:
            res.disposition=res.BLOCKED;res.reason=str(error);return res
        now=time.monotonic()
        with self.mailbox_lock:
            self.pending_route=(key,req,now)
            cached=self.route_response
        if cached is not None and cached[0]==key and now-cached[1]<=.1:
            return copy.deepcopy(cached[2])
        res.reason='ROUTE_EVALUATION_PENDING'
        return res

    def process_route(self):
        with self.mailbox_lock:
            pending=self.pending_route;self.pending_route=None
        if pending is None:return
        key,req,received=pending
        if time.monotonic()-received>self.takeover['context_timeout_s']:return
        response=self.evaluate_route(req,ResolveRoute.Response())
        with self.mailbox_lock:self.route_response=(key,time.monotonic(),response)

    def evaluate_route(self, req, res):
        now=time.monotonic();res.reason='KEEP_CURRENT_PATH'
        try:
            key=path_identity(req.reference_path)
            if (not req.session_id or not req.reference_path.poses or
                req.goal.pose!=req.reference_path.poses[-1].pose or
                req.goal.header.frame_id!=req.reference_path.header.frame_id):
                raise ValueError('INVALID_EXECUTION_CONTEXT')
            if req.session_id!=self.context:
                # Execution identity, not a BT restart, owns the per-goal budget.
                self.cancel();self.context=req.session_id;self.context_epoch=self.node.epoch;self.blocked_since=None
                self.attempted_episode=None;self.failure=None;self.audit=[]
                self.safety_evidence=[];self.retry.reset()
                observed=self.node.execution.version
                self.version=replace(observed,goal_id=observed.goal_id if observed.goal_id!='idle' else self.context)
                self.session.activate(self.version,self.steady())
            if self.key!=key:
                self.cancel();self.key=key
                self.version=replace(self.version,path_revision=self.version.path_revision+1)
                self.session.activate(self.version,self.steady())
            observed=self.node.execution.version
            current=replace(self.version,goal_id=observed.goal_id if observed.goal_id!='idle' else self.version.goal_id,
                map_epoch=observed.map_epoch,
                localization_epoch=observed.localization_epoch,envelope_epoch=observed.envelope_epoch,
                clock_epoch=observed.clock_epoch)
            if current!=self.version:
                self.cancel();self.version=current;self.session.activate(current,self.steady())
            self.reference=req.reference_path;self.goal=req.goal;self.last_poll=now;self.allow_detour=req.allow_detour
            res.evaluated_at=self.node.get_clock().now().to_msg()
            if self.failure:
                res.disposition=res.BLOCKED;res.reason=self.failure
            elif self.ready is not None:
                safe,_,reason=self.safe(self.ready)
                if safe and self.session.response_current(self.request,self.version,self.steady()):
                    res.disposition=res.COMMIT;res.path=self.ready;res.request_id=self.request.request_id
                    res.reason='LOCAL_DETOUR' if self.chosen[0]==PlanCandidate.Request.LOCAL else 'GLOBAL_DETOUR'
                    # Remain held until the committed active path is observed.
                else:
                    self.audit.append(reason);self.finish_attempt()
        except (ValueError,TypeError) as error:
            res.disposition=res.BLOCKED;res.reason=str(error)
        return res

    def safe(self,path):
        n=self.node
        if (not n.last_inputs_valid or n.last_evaluation_epoch!=n.epoch or
            not hasattr(n,'last_world') or n.last_robot is None):return False,0.,'INPUT_UNAVAILABLE'
        if not 0<=n.get_clock().now().nanoseconds-n.last_world.stamp.ns<=int(n.profile.sensor_timeout_s*1e9):return False,0.,'WORLD_EXPIRED'
        world_version=n.last_world.version
        if world_version.goal_id not in ('idle',self.version.goal_id):return False,0.,'TASK_VERSION_CHANGED' 
        if any(getattr(world_version,f)!=getattr(self.version,f) for f in
               ('map_epoch','localization_epoch','envelope_epoch','clock_epoch')):
            return False,0.,'WORLD_VERSION_CHANGED'
        robot=n.last_robot
        if math.hypot(robot.vx,robot.vy)>self.takeover['linear_speed_m_s'] or abs(robot.wz)>self.takeover['angular_speed_rad_s']:return False,0.,'TAKEOVER_NOT_STOPPED'
        try:
            transform=n.tf.lookup_transform(n.profile.tracking_frame,path.header.frame_id,Time())
            # A transform discontinuity invalidates the candidate at commit.
            route=tuple(n.point((p.pose.position.x,p.pose.position.y,p.pose.position.z),transform)[:2] for p in path.poses)
            if not route or math.dist(route[0],(robot.x,robot.y))>self.takeover['position_tolerance_m']:return False,0.,'TAKEOVER_START_MOVED'
            evidence={}
            result=candidate_clearance(n.last_world,robot,route,n.profile,evidence)
            if evidence:
                self.safety_evidence.append(dict(reason=result[2],**evidence))
                self.safety_evidence=self.safety_evidence[-12:]
            return result
        except Exception:return False,0.,'TRANSFORM_UNAVAILABLE'

    def alternatives(self,mode,answer):
        path=answer.path
        route=tuple((p.pose.position.x,p.pose.position.y) for p in path.poses)
        for points in lateral_variants(route,min(.2,self.profile.local_max_deviation_m)):
            if mode==PlanCandidate.Request.LOCAL:
                reference=[(p.pose.position.x,p.pose.position.y) for p in self.reference.poses]
                if any(min(math.dist(p,q) for q in reference)>self.profile.local_max_deviation_m for p in points):continue
            if points in self.variant_keys:continue
            self.variant_keys.add(points)
            candidate=copy.deepcopy(path)
            for pose,(x,y) in zip(candidate.poses,points):pose.pose.position.x=x;pose.pose.position.y=y
            for a,b in zip(candidate.poses,candidate.poses[1:]):
                heading=math.atan2(b.pose.position.y-a.pose.position.y,b.pose.position.x-a.pose.position.x)
                a.pose.orientation.x=a.pose.orientation.y=0.
                a.pose.orientation.z=math.sin(heading/2);a.pose.orientation.w=math.cos(heading/2)
            candidate.poses[0]=copy.deepcopy(path.poses[0]);candidate.poses[-1]=copy.deepcopy(path.poses[-1])
            safe,clearance,_=self.safe(candidate)
            if safe:self.variants.append((mode,answer,clearance,candidate))
        self.variants=sorted(self.variants,key=lambda o:-min(o[2],1.))[:4]

    def next_variant(self):
        if not self.variants:self.finish_attempt();return
        mode,answer,clearance,path=self.variants.pop(0)
        self.chosen=(mode,answer,clearance)
        self.audit.append('BOUNDED_LATERAL_VARIANT')
        self.send(PlanCandidate.Request.VALIDATE,path)

    def send(self,mode,path=None):
        req=PlanCandidate.Request();req.mode=mode;req.goal=self.goal;req.reference_path=self.reference
        req.rejoin_distance_m=self.profile.local_rejoin_distance_m
        req.max_local_deviation_m=self.profile.local_max_deviation_m
        if path is not None:req.candidate_path=path
        self.mode=mode;self.future=self.client.call_async(req)

    def finish_attempt(self):
        request=self.request
        self.cancel()
        if request is not None:self.session.retire(request)

    def advance(self, selection, risk, valid):
        now=time.monotonic();n=self.node
        if selection.reason=='BLOCKED_CAPABILITY_NOT_ENABLED':
            selection=replace(selection,reason='WAITING_FOR_SAFE_ROUTE')
        if self.context is None or now-self.last_poll>self.takeover['context_timeout_s']:
            self.cancel();return selection
        if self.context_epoch!=n.epoch:
            self.cancel();self.failure='CLOCK_RESET_REQUIRES_NEW_GOAL'
        if self.failure:return Selection('HOLD',0.,self.failure,episode=selection.episode)
        if n.active_path_key!=self.key:
            self.cancel();return selection
        obstructed=valid and risk is not None and (risk.blocked or risk.immediate or risk.uncertain)
        if valid and risk is not None and not obstructed:
            self.cancel();self.blocked_since=None;self.attempted_episode=None
            self.retry.reset()
            self.session.clear_blockage(self.steady());return selection
        if (valid and risk is not None and selection.motion!='HOLD' and
            not risk.immediate and not risk.uncertain):
            # Safe progress on the current path takes precedence over an
            # outstanding detour, including a confirmed distant-risk slowdown.
            if self.request is not None or self.holding:
                self.audit.append('KEEP_CURRENT_PATH')
                self.finish_attempt()
                self.attempted_episode=None
                self.retry.reset()
            self.blocked_since=None
            self.session.clear_blockage(self.steady())
            return selection
        if self.request is None and selection.motion!='HOLD':self.blocked_since=None
        if selection.motion=='HOLD' and self.blocked_since is None:self.blocked_since=now
        if self.blocked_since is not None and now-self.blocked_since>=self.profile.planning_budget['episode_timeout_s']:
            self.cancel()
            self.failure=('TEMPORARILY_BLOCKED: obstruction deadline' if valid else
                          'INPUT_UNAVAILABLE: input deadline')
            return Selection('HOLD',0.,self.failure,episode=selection.episode)
        if not valid:
            self.cancel();return selection
        if self.request is not None:
            if not self.session.response_current(self.request,self.version,self.steady()):
                self.audit.append('REQUEST_EXPIRED');self.finish_attempt()
            elif self.future is not None and self.future.done():
                future=self.future;self.future=None
                try:answer=future.result()
                except Exception as error:answer=None;self.audit.append(str(error))
                if answer is not None:
                    self.audit.append(f'{self.mode}:{answer.reason}')
                    if answer.geometry_valid:
                        safe,clearance,reason=self.safe(answer.path);self.audit.append(reason)
                        if safe:
                            if self.mode==PlanCandidate.Request.VALIDATE:self.ready=answer.path
                            else:self.options.append((self.mode,answer,clearance))
                        elif self.mode!=PlanCandidate.Request.VALIDATE:
                            self.alternatives(self.mode,answer)
                if self.mode==PlanCandidate.Request.LOCAL:self.send(PlanCandidate.Request.GLOBAL)
                elif self.mode==PlanCandidate.Request.GLOBAL:
                    if self.options:
                        # Hard safety precedes quality; local stability wins ties.
                        self.chosen=min(self.options,key=lambda o:(-min(o[2],1.),o[1].curvature,
                            o[1].curvature_rate,o[0]!=PlanCandidate.Request.LOCAL))
                        self.send(PlanCandidate.Request.VALIDATE,self.chosen[1].path)
                    else:self.next_variant()
                elif self.ready is None:self.next_variant()
        if (self.request is None and self.allow_detour and obstructed and not risk.uncertain and self.blocked_since is not None and
            now-self.blocked_since>=self.profile.blocked_confirm_s and
            (self.attempted_episode!=selection.episode or self.retry.changed(n.last_world,now)) and
            (not risk.moving or now-self.blocked_since>=self.profile.wait_budget_s)):
            # Never replace the terminal refinement goal with a detour target.
            if n.path and math.dist(n.path[-1],(n.last_robot.x,n.last_robot.y))>self.takeover['terminal_exclusion_m'] and self.client.service_is_ready():
                self.holding=True
                if math.hypot(n.last_robot.vx,n.last_robot.vy)<=self.takeover['linear_speed_m_s'] and abs(n.last_robot.wz)<=self.takeover['angular_speed_rad_s']:
                    try:
                        self.request=self.session.request(self.version,Trigger.PATH_RISK,
                            frozenset((Planning.LOCAL,Planning.GLOBAL)),n.last_world.observation_seq,self.steady())
                        self.attempted_episode=selection.episode;self.retry.attempted(n.last_world,now)
                        self.send(PlanCandidate.Request.LOCAL)
                    except PlanningBudgetExhausted as error:self.failure=str(error)
        if self.request is not None or self.holding:
            return Selection('HOLD',0.,'ASSESSING_DETOURS',episode=selection.episode)
        return selection
