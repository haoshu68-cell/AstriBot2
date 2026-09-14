"""Repository launch actions with one default log level policy."""
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as RosNode
from launch_ros.actions import ComposableNodeContainer as RosContainer
from . import log_level
from .output import configure_launch_logging

configure_launch_logging()


def _arguments(kwargs):
    arguments = list(kwargs.get('arguments') or [])
    ros_arguments = list(kwargs.get('ros_arguments') or [])
    # ROS severity/rosout/throttling remain native; console output is persisted by
    # launch's spdlog sink, avoiding a second unbounded native ROS file.
    ros_arguments.append('--disable-external-lib-logs')
    kwargs['ros_arguments'] = ros_arguments
    if '--log-level' not in arguments and '--log-level' not in ros_arguments:
        kwargs['ros_arguments'] = [
            '--log-level', LaunchConfiguration('log_level', default=log_level()),
            *ros_arguments]
    return kwargs


class Node(RosNode):
    def __init__(self, **kwargs):
        super().__init__(**_arguments(kwargs))


class ComposableNodeContainer(RosContainer):
    def __init__(self, **kwargs):
        super().__init__(**_arguments(kwargs))
