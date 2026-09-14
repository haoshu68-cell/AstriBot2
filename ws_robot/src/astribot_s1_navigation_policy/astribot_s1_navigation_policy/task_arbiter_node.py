"""Single navigation owner; replacement waits for the old backend action's terminal result."""
import threading
import time
from dataclasses import dataclass
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from action_msgs.msg import GoalStatus
from astribot_navigation_msgs.msg import NavigationExecutionStatus


@dataclass
class Task:
    handle: object
    client: object
    kind: object
    source: str
    priority: int
    backend: object = None
    preempted: bool = False
    cancel_sent: bool = False
    terminal: bool = False


class TaskArbiter(Node):
    def __init__(self):
        super().__init__('navigation_task_arbiter')
        self.declare_parameter('handover_timeout_s', 10.0)
        self.timeout = float(self.get_parameter('handover_timeout_s').value)
        if not 0 < self.timeout <= 60:raise ValueError('invalid handover_timeout_s')
        self.lock = threading.RLock()
        self.active = None
        self.pending = None
        self.reserved = False
        self.sequence = time.monotonic_ns()
        self.tasks = {}
        self.group = ReentrantCallbackGroup()
        self.status = self.create_publisher(NavigationExecutionStatus,
            '/navigation/execution_status', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.servers = []
        for kind, name in ((NavigateToPose, 'navigate_to_pose'), (NavigateThroughPoses, 'navigate_through_poses')):
            client = ActionClient(self, kind, '/navigation_executor/' + name, callback_group=self.group)
            for source, priority, front in (('operator',100,'/'+name),
                    ('route',50,'/route/'+name),('exploration',10,'/exploration/'+name)):
                self.servers.append(ActionServer(self, kind, front,
                    execute_callback=self.execute,
                    goal_callback=lambda request,p=priority: self.admit(p),
                    handle_accepted_callback=lambda h,c=client,k=kind,s=source,p=priority:self.accept(h,c,k,s,p),
                    cancel_callback=self.cancel, callback_group=self.group))

    def admit(self, priority):
        with self.lock:
            if (self.reserved or self.pending is not None or
                    (self.active is not None and priority < self.active.priority)):
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def emit(self, task, state, reason='', status=0):
        with self.lock:
            self.sequence += 1
            m = NavigationExecutionStatus()
            m.stamp = self.get_clock().now().to_msg()
            m.task_id = bytes(task.handle.goal_id.uuid).hex()
            m.source = task.source
            m.sequence = self.sequence
            m.state = state
            m.reason = reason
            m.action_status = status
            self.status.publish(m)

    def accept(self, handle, client, kind, source, priority):
        with self.lock:
            task = Task(handle,client,kind,source,priority)
            self.tasks[bytes(handle.goal_id.uuid)] = task
            self.pending = task
            self.reserved = False
            if self.active is not None:
                self.active.preempted = True
                self.request_cancel(self.active)
            self.emit(task,'ACCEPTED')
        handle.execute()

    def request_cancel(self, task):
        if task.backend is not None and not task.cancel_sent:
            task.cancel_sent = True
            task.backend.cancel_goal_async()
            self.emit(task,'CANCELING','PREEMPTED' if task.preempted else 'USER_CANCEL')

    def cancel(self, handle):
        with self.lock:
            task = self.tasks.get(bytes(handle.goal_id.uuid))
            if task is None:return CancelResponse.REJECT
            self.request_cancel(task)
        return CancelResponse.ACCEPT

    def wait(self, future, task):
        while rclpy.ok() and not future.done():
            with self.lock:
                if task.preempted or task.handle.is_cancel_requested:self.request_cancel(task)
            time.sleep(.01)
        return future.result() if future.done() else None

    def finish(self, task, state, reason, result=None, status=0):
        with self.lock:
            if task.terminal:return task.kind.Result() if result is None else result
            task.terminal = True
            if task.handle.is_cancel_requested:task.handle.canceled();state='CANCELED'
            elif state == 'SUCCEEDED':task.handle.succeed()
            else:task.handle.abort()
            self.emit(task,state,reason,status)
            if self.active is task:self.active = None
            if self.pending is task:self.pending = None
            self.tasks.pop(bytes(task.handle.goal_id.uuid),None)
        return task.kind.Result() if result is None else result

    def execute(self, handle):
        task = self.tasks[bytes(handle.goal_id.uuid)]
        deadline = time.monotonic() + self.timeout
        while rclpy.ok():
            with self.lock:
                if handle.is_cancel_requested:return self.finish(task,'CANCELED','USER_CANCEL')
                if self.active is None:
                    self.active = task
                    self.pending = None
                    break
            if time.monotonic() >= deadline:
                return self.finish(task,'FAILED','PREVIOUS_TASK_NOT_TERMINAL')
            time.sleep(.01)
        if not rclpy.ok():return self.finish(task,'FAILED','SHUTDOWN')
        if not task.client.wait_for_server(timeout_sec=self.timeout):
            return self.finish(task,'FAILED','EXECUTOR_UNAVAILABLE')
        if task.preempted or handle.is_cancel_requested:
            return self.finish(task,'PREEMPTED' if task.preempted else 'CANCELED','CANCELED_BEFORE_DISPATCH')
        def feedback(msg):
            if handle.is_active:handle.publish_feedback(msg.feedback)
        sent = task.client.send_goal_async(handle.request,feedback_callback=feedback)
        task.backend = self.wait(sent,task)
        if task.backend is None or not task.backend.accepted:
            return self.finish(task,'FAILED','EXECUTOR_REJECTED')
        with self.lock:
            if task.preempted or handle.is_cancel_requested:self.request_cancel(task)
            self.emit(task,'EXECUTING')
        # Do not release ownership on a cancel acknowledgement or timeout.
        # Only the terminal result proves that the old executor has relinquished the task.
        wrapped = self.wait(task.backend.get_result_async(),task)
        if wrapped is None:return self.finish(task,'FAILED','SHUTDOWN')
        if task.preempted:return self.finish(task,'PREEMPTED','HIGHER_OR_EQUAL_PRIORITY_TASK',wrapped.result,wrapped.status)
        state = 'SUCCEEDED' if wrapped.status == GoalStatus.STATUS_SUCCEEDED else 'FAILED'
        return self.finish(task,state,'EXECUTOR_RESULT',wrapped.result,wrapped.status)


def main():
    rclpy.init()
    node = TaskArbiter()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():rclpy.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
