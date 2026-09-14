import os
import ast
import cv2
import time
import yaml
import json
import atexit
import numpy as np

from typing import List, Dict
from functools import wraps, partial

import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy


from std_msgs.msg import Int32MultiArray,Header
from astribot_msgs.srv import RawRequest
from astribot_msgs.srv import DoubleArrayRequest
from astribot_msgs.msg import RobotJointController, DoubleArray
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool

import astribot_ros_middleware as ast_ros_middleware

import sys, ctypes
quiet = os.getenv("ASTRIBOT_LOG", "").lower() not in ("1", "true", "on")

if quiet:  # >>> 静音开始
    sys.stdout.flush(); sys.stderr.flush(); ctypes.CDLL(None).fflush(None)
    _fd1, _fd2 = os.dup(1), os.dup(2)
    null_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null_fd, 1); os.dup2(null_fd, 2)
    os.close(null_fd)


from astribot_sdk.core.common.util.astribot_function import AstribotFunction
from astribot_sdk.core.common.robotics_library_py.robotics_library_base import robot_init, load_astribot, load_astribot_s1
from astribot_sdk.core.common.logger import default_logger

if quiet:
    os.dup2(_fd1, 1); os.dup2(_fd2, 2)
    os.close(_fd1); os.close(_fd2)

from astribot_sdk.core.common.astribot_base import AstribotBase


class AstribotInterface(AstribotBase):
    def __init__(self, freq=250.0, high_control_rights=False, node_name="use_astribot", logger=default_logger) -> None:
        super().__init__()
        robot_init()

        self._logger = logger
        node_name_with_pid = f"{node_name}_{os.getpid()}"

        if not ast_ros_middleware.is_inited():
            try:
                ast_ros_middleware.init(node_name=node_name_with_pid, anonymous=True)
            except RuntimeError:
                pass

        self.node = Node(node_name_with_pid)
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

        def _spin_with_exception_handler():
            while rclpy.ok():
                try:
                    self.executor.spin()
                except Exception:
                    pass
                time.sleep(0.008)

        self.spin_thread = threading.Thread(target=_spin_with_exception_handler, daemon=True, name=f"ROS2-spin-{node_name}")
        self.spin_thread.start()

        if self.robot_type == "S0":
            astribot_robot_dict = load_astribot()
        else:  # self.robot_type == 'S1'
            astribot_robot_dict = load_astribot_s1()

        self.__astribot_class: AstribotFunction = AstribotFunction(
            astribot_robot_dict, frequency=freq, node=self.node
        )
        self.logger = self.node.get_logger()

        self._desired_joint_subscribers = []
        self.__have_control_rights = None
        self.__high_control_rights = high_control_rights
        self.transfer_control_timer = None
        self.transfer_control_lock = threading.Lock()
        self._logger.info("acquiring control rights...")
        self.acquire_control_rights(high_control_rights)
        self._logger.info("acquired control rights.")

        self.__heartbeat_publisher = self.__astribot_class.heartbeat_publisher
        self.heartbeat_timer = self.node.create_timer(0.1, self._run_pub_heartbeat)
        self.flag_robot_driver_alive = True

        self._device_activate_service = self.node.create_client(
            RawRequest, "/astribot/device_activate_service"
        )
        self._kill_sdk = self.node.create_service(
            RawRequest, "/simu_real_switch_service", self.__kill_sdk, 
            callback_group=ReentrantCallbackGroup()
        )
        self.error_code_sub_list = list()
        self.error_code_timestamp = None
        self.last_reported_code = None
        self.error_map = self._load_error_mapping()
        self.heartbeat_timeout_count = 0  # 连续超时计数器
        self.heartbeat_timeout_threshold = 5  # 超时阈值
        self.error_code_sub_list.append(
            self.node.create_subscription(
                Int32MultiArray, "/astribot_error_code/control_driver", 
                self.drive_error_code, 10,
                callback_group=ReentrantCallbackGroup()
            )
        )
        self.error_code_sub_list.append(
            self.node.create_subscription(
                Int32MultiArray, "/astribot_error_code/device_driver", 
                self.show_error_code, 10,
                callback_group=ReentrantCallbackGroup()
            )
        )

        self.color_images_dict = dict()
        self.depth_images_dict = dict()
        self.ir_images_dict = dict()
        self.cam_timestamp = 0.0
        self.need_decode_ = False

    def is_alive(self):
        flag = True
        for robot_name in self.__astribot_class.robot_dict.keys():
            if robot_name == "robot_type":
                continue
            flag = self.__astribot_class.robot_dict[robot_name].joint_interface_.IsAlive()
            if flag is False:
                self._logger.error(f"{robot_name} is not alive")
                flag = False
        return flag

    def __kill_sdk(self, req=None):
        if not self.__high_control_rights:
            os._exit(0)

    @property
    def have_control_rights(self):
        return self.__have_control_rights

    @have_control_rights.setter
    def have_control_rights(self, value):
        if value != self.__have_control_rights:
            old_control_rights = self.__have_control_rights
            self.__have_control_rights = value
            if value:
                self.__astribot_class.restart_robot()
                self.__astribot_class.set_current_position_as_desired_joint_position()
            else:
                if old_control_rights:
                    self.__astribot_class.stop_robot()

    def check_control_rights(func):
        @wraps(func)
        def wrapper(self, *args, **kargs):
            if not self.__have_control_rights:
                self.logger.error("You don't have control rights of the robot.")
                return None
            else:
                return func(self, *args, **kargs)

        return wrapper

    def check_control_rights_and_is_robot_alive(func):
        @wraps(func)
        def wrapper(self, *args, **kargs):
            if not self.__have_control_rights:
                self.logger.error("You don't have control rights of the robot.")
            elif self.__astribot_class.is_stopped():
                self.logger.error("Robot is stopped, please restart it.")
            else:
                return func(self, *args, **kargs)
            return None

        return wrapper

    def __del__(self):
        self.stop_transfer_control_timer()
        if getattr(self, "_control_rights_srv", None) is not None:
            self.node.destroy_service(self._control_rights_srv)
            self._control_rights_srv = None
        for sub in self.error_code_sub_list:
            self.node.destroy_subscription(sub)
        self.node.destroy_node()

    def stop_transfer_control_timer(self, timeout=1.0):
        with self.transfer_control_lock:
            if hasattr(self, 'transfer_control_timer') and self.transfer_control_timer is not None:
                if self.transfer_control_timer.is_alive():
                    self.transfer_control_timer.join(timeout=timeout)

                    if self.transfer_control_timer.is_alive():
                        self.transfer_control_timer.cancel()

    def shutdown(self):
        """在 Python 退出时自动调用"""
        self.stop_transfer_control_timer()
                    
        req= RawRequest.Request()
        resp = RawRequest.Response()
        self.transfer_control_rights(req, resp)
        for sub in self.error_code_sub_list:
            self.node.destroy_subscription(sub)
        if getattr(self, "_control_rights_srv", None) is not None:
            self.node.destroy_service(self._control_rights_srv)

        self.node.destroy_timer(self.heartbeat_timer)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def acquire_control_rights(self, high_control_rights: bool) -> None:
        control_rights_name = "/astribot/control_rights"
        if self.__have_control_rights:
            return
        try:
            client = self.node.create_client(RawRequest, control_rights_name)
            service_exit_flag = client.wait_for_service(timeout_sec=0.2)
            
            if service_exit_flag is False:
                raise RuntimeError(f"Service {control_rights_name} not available")

            if high_control_rights:
                control_rights_request = self.node.create_client(RawRequest, control_rights_name)

                future = control_rights_request.call_async(RawRequest.Request())
                future.result()  # Wait for the service call to complete

                self._control_rights_srv = self.node.create_service(
                    RawRequest, control_rights_name, self.transfer_control_rights,
                    callback_group=ReentrantCallbackGroup()
                )

                self.have_control_rights = True
                self.logger.warning("Be careful that you seize control of other users")
            else:
                self.have_control_rights = False
                self.logger.warning(
                    "Another user currently controls the robot. "
                    "Acquiring control will immediately stop the robot's current motion. "
                    "Please ensure the robot is stationary and in a safe state before proceeding."
                    "Enter 'yes' to forcibly acquire control, or press 'Enter' to continue without control rights:"
                )
                user_input = input().strip().lower()
                if user_input == 'yes':
                    control_rights_request = self.node.create_client(
                        RawRequest, control_rights_name
                    )
                    future = control_rights_request.call_async(RawRequest.Request())
                    future.result()  # Wait for the service call to complete
                    self._control_rights_srv = self.node.create_service(
                        RawRequest, control_rights_name, self.transfer_control_rights,
                        callback_group=ReentrantCallbackGroup()
                    )
                    self.have_control_rights = True
                    self._logger.info("Be careful that you seize control of other users")

        except Exception as e:
            try:
                if getattr(self, "_control_rights_srv", None) is not None:
                    self.node.destroy_service(self._control_rights_srv)
                self._control_rights_srv = self.node.create_service(
                    RawRequest, control_rights_name, self.transfer_control_rights,
                    callback_group=ReentrantCallbackGroup()
                )
                self.have_control_rights = True
            except Exception as e:
                self._logger.error(f"Failed to grab control rights because of {e}")
                self.have_control_rights = False

    def transfer_control_rights(self, req, resp):
        if not self.have_control_rights:
            resp.response = "Internal failure"
            return resp
        self.transfer_control_timer = threading.Timer(0.05, self.shutdown_control_rights_srv)
        self.transfer_control_timer.start()
        self.logger.warning("Control rights will be released after 50ms")
        resp.response = "success"
        return resp

    def shutdown_control_rights_srv(self):
        with self.transfer_control_lock:
            if getattr(self, "_control_rights_srv", None) is not None:
                self.node.destroy_service(self._control_rights_srv)
                self._control_rights_srv = None
                self.have_control_rights = False

    def _pub_heartbeat(self, data):
        message = Int32MultiArray(data=data)
        self.__heartbeat_publisher.publish(message)

    def _run_pub_heartbeat(self):
        if self.__have_control_rights  and self.error_code_timestamp and time.time() - self.error_code_timestamp < 1:
            self._pub_heartbeat(data=[16000000])
            if self.heartbeat_timeout_count > 0:
                self.logger.info(f"Driver recovered. Resetting timeout count from {self.heartbeat_timeout_count} to 0.")
                self.heartbeat_timeout_count = 0
        if self.error_code_timestamp is not None:
            current_time = time.time()
            elapsed_time = current_time - self.error_code_timestamp
            if elapsed_time > 0.8:
                self.heartbeat_timeout_count += 1
                self.logger.warning(
                    f"Driver heartbeat timeout detected! "
                    f"Count: {self.heartbeat_timeout_count}/{self.heartbeat_timeout_threshold}, "
                    f"Elapsed: {elapsed_time:.3f}s, "
                    f"Last error_code_timestamp: {self.error_code_timestamp:.3f}, "
                    f"Current time: {current_time:.3f}"
                )
                if self.heartbeat_timeout_count >= self.heartbeat_timeout_threshold:
                    self.logger.error(
                        f"Driver crashes. Reached timeout threshold ({self.heartbeat_timeout_threshold}). Exiting..."
                    )
                    self.flag_robot_driver_alive = False
                    if not self.__high_control_rights:
                        os._exit(0)

    def _load_error_mapping(self):
        yaml_path = os.path.join(os.path.dirname(__file__), "error_mapping.yaml")
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
                return config.get("error_mapping", {})
        except Exception as e:
            self.logger.error(f"Failed to load error mapping: {e}")
            return {}
    
    def show_error_code(self, msg):
        if len(list(msg.data)) > 1:
            current_code = str(msg.data[1])

            if current_code == self.last_reported_code:
                return
            self.last_reported_code = current_code 
        
            third_digit = int(current_code[2])
            if third_digit == 2:
                self.logger.error(f"Error occur, error code: {msg.data}, error message: {self.error_map.get(current_code, 'Unknown error')}")
            elif third_digit == 1:
                self.logger.warning(f"Warning occur, error code: {msg.data}, warning message: {self.error_map.get(current_code, 'Unknown warning')}")

    def drive_error_code(self, msg):
        self.error_code_timestamp = time.time()
        if self.heartbeat_timeout_count > 0:
            self.logger.info(f"Received driver error code. Resetting timeout count from {self.heartbeat_timeout_count} to 0.")
            self.heartbeat_timeout_count = 0
            self.flag_robot_driver_alive = True
        self.show_error_code(msg)


    def get_robot_mode(self):
        try:
            client = self.node.create_client(RawRequest, '/astribot_safe_mode/state')
            if not client.wait_for_service(timeout_sec=0.1):
                return "simulation"

            request = RawRequest.Request()
            request.request = "quest"

            future = client.call_async(request)

            import time
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5:
                time.sleep(0.001)

            if future.done():
                response = future.result()
                return response.response
            else:
                return "simulation"
        except Exception as e:
            return "simulation"

    def get_general_info(self, info_type, names):
        if info_type == "get_dof":
            return ast.literal_eval(self.__astribot_class.get_dof({"name_list": names}))
        elif info_type == "get_joints_position_limit":
            joints_position_limit = json.loads(
                self.__astribot_class.get_joints_position_limit({"name_list": names})
            )
            return joints_position_limit["lower"], joints_position_limit["upper"]
        elif info_type == "get_joints_velocity_limit":
            return ast.literal_eval(
                self.__astribot_class.get_joints_velocity_limit({"name_list": names})
            )
        elif info_type == "get_joints_torque_limit":
            return ast.literal_eval(
                self.__astribot_class.get_joints_torque_limit({"name_list": names})
            )
        else:
            self._logger.warning(f"No info of: {info_type}")

    def get_self_closest_point(self, torso_joints_position, arm_left_joints_position, arm_right_joints_position):
        return (self.__astribot_class.get_self_closest_point(
                    torso_joints_position,
                    arm_left_joints_position,
                    arm_right_joints_position))

    def get_joints_info(self, function_name, names):
        if function_name == "get_current_joints_position":
            return self.__astribot_class.get_current_joint_position_list(names)
        elif function_name == "get_current_joints_velocity":
            return self.__astribot_class.get_current_joint_velocity_list(names)
        elif function_name == "get_current_joints_acceleration":
            return self.__astribot_class.get_current_joints_acceleration_list(names)
        elif function_name == "get_current_joints_torque":
            return self.__astribot_class.get_current_joint_torque_list(names)
        elif function_name == "get_desired_joints_position":
            return self.__astribot_class.get_desired_joint_position_list(names)
        elif function_name == "get_desired_joints_velocity":
            return self.__astribot_class.get_desired_joint_velocity_list(names)
        elif function_name == "get_desired_joints_torque":
            return self.__astribot_class.get_desired_joint_torque_list(names)
        else:
            self._logger.warning(f"No joint info for {function_name}")

    def get_cartesian_info(self, function_name, names, frame):
        result = self.__astribot_class.get_cartesian_pose(
            {"pose_type": f"{function_name}_from_{frame}", "name_list": names}
        )
        if not result:
            raise ValueError(
                f"Invalid argument. {function_name}_from_{frame} not found"
            )
        else:
            return result

    def get_forward_kinematics(self, names, joints_position):
        return json.loads(
            self.__astribot_class.get_forward_kinematics(
                {"name_list": names, "joints_list": joints_position}
            )
        )

    def get_inverse_kinematics(self, names, cartesian_pose_list):
        ik_result = json.loads(
            self.__astribot_class.get_inverse_kinematics(
                {"name_list": names, "pose_list": cartesian_pose_list}
            )
        )
        ik_flag = ik_result["ik_flag"]
        if ik_flag:
            ik_result.pop("ik_flag")
            return ik_flag, ik_result
        else:
            return ik_flag, None


    @check_control_rights
    def set_mode(self, mode: str, enable: bool, arm_name: str = "dual"):
        if mode == "set_head_follow_effector":
            self.__astribot_class.set_head_follow_effector({"enable": enable, "arm_name": arm_name})
        elif mode == "set_wbc_collision_avoidance":
            self.__astribot_class.set_wbc_collision_avoidance({"enable": enable})
        else:
            self.logger.warning(f"Not support for set mode: {mode}")

    @check_control_rights
    def set_filter_parameters(self, filter_scale, gripper_filter_scale):
        return self.__astribot_class.set_filter_parameters(
            {"filter_scale": filter_scale, "gripper_filter_scale": gripper_filter_scale}
        )

    @check_control_rights_and_is_robot_alive
    def move_to_joint_position(self, names, cmd, duration, use_wbc):
        return self.__astribot_class.move_to_joint_position(
            {
                "name_list": names,
                "command_list": cmd,
                "duration": duration,
                "use_wbc": use_wbc,
            }
        )

    @check_control_rights_and_is_robot_alive
    def move_joints_waypoints(
        self, names, waypoints, time_list, use_wbc, joy_controller
    ):
        return self.__astribot_class.move_joints_waypoints(
            {
                "name_list": names,
                "waypoints": waypoints,
                "time_list": time_list,
                "use_wbc": use_wbc,
                "joy_controller": joy_controller,
            }
        )

    @check_control_rights_and_is_robot_alive
    def move_to_cartesian_pose(self, names, cmd, duration, use_wbc):
        self.__astribot_class.move_to_cartesian_pose(
            {
                "name_list": names,
                "command_list": cmd,
                "duration": duration,
                "use_wbc": use_wbc,
            }
        )

    @check_control_rights
    def move_cartesian_waypoints(
        self, names, waypoints, time_list, use_wbc, joy_controller
    ):
        self.__astribot_class.move_cartesian_waypoints(
            {
                "name_list": names,
                "waypoints": waypoints,
                "time_list": time_list,
                "use_wbc": use_wbc,
                "joy_controller": joy_controller,
            }
        )

    @check_control_rights
    def stop_robot(self):
        self.__astribot_class.stop_robot()

    @check_control_rights
    def restart_robot(self):
        self.__astribot_class.restart_robot()

    @check_control_rights_and_is_robot_alive
    def set_joints_position(
        self,
        names: List[str],
        position: List[List[float]],
        control_way: str,
        use_wbc: bool,
        add_default_torso: bool,
    ):
        if control_way not in ["direct", "filter"]:
            raise ValueError("Invalid argument. control way options: [direct, filter]")

        command_list = position
        self.__astribot_class.set_joints_position(
            {
                "name_list": names,
                "command_list": command_list,
                "control_way": control_way,
                "use_wbc": use_wbc,
                "add_default_torso": add_default_torso,
            }
        )

    @check_control_rights
    def set_joints_velocity(self, names: List[str], velocity: List[List[float]]):
        self.__astribot_class.set_joints_velocity(
            {"name_list": names, "velocity": velocity}
        )

    @check_control_rights
    def set_joints_torque(self, names: List[str], torque: List[List[float]]):
        self.__astribot_class.set_joints_torque({"name_list": names, "torque": torque})

    @check_control_rights
    def set_cartesian_pose(
        self,
        names: List[str],
        cartesian_pose: List[List[float]],
        control_way: str,
        use_wbc: bool,
        add_default_torso: bool,
        remote_wbc_option: bool = True,
    ):
        self.__astribot_class.set_cartesian_pose(
            {
                "name_list": names,
                "cartesian_pose": cartesian_pose,
                "control_way": control_way,
                "use_wbc": use_wbc,
                "add_default_torso": add_default_torso,
                "remote_wbc_option": remote_wbc_option,
            }
        )
    
    @check_control_rights
    def set_different_type_command(
        self,
        names,
        types,
        command_list,
        control_way,
        use_wbc,
    ):
        self.__astribot_class.set_different_type_command(
            {
                "name_list": names,
                "type_list": types,
                "command_list": command_list,
                "control_way": control_way,
                "use_wbc": use_wbc,
            }
        )


    def activate_audio(self, audio_config: Dict[str, float]):
        """
        Activate audio by requesting system monitor.
        audio_config parameter is ignored now and will be support in later version.
        """
        try:
            client = self.node.create_client(SetBool, '/astribot_micspeaker_control')

            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service /astribot_micspeaker_control not available")

            try:
                req = SetBool.Request()
                req.data = True

                future = client.call_async(req)
                
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.001)

                if future.done():
                    response = future.result()
                else:
                    self.logger.error("Service call timeout")
                    return False

            except Exception as e:
                self.logger.error(f"System monitor service call failed: {e}")
                return False
        except Exception as e:
            self.logger.error(f"System monitor speaker service wait timeout: {e}")
            return False

    def deactivate_audio(self):
        """
        Deactivate audio by requesting system monitor.
        """
        try:
            client = self.node.create_client(SetBool, '/astribot_micspeaker_control')

            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service /astribot_micspeaker_control not available")

            try:
                req = SetBool.Request()
                req.data = False

                future = client.call_async(req)
                
                import time
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.001)

                if future.done():
                    response = future.result()
                else:
                    self.logger.error("Service call timeout")
                    return False

                if response.success:
                    self.logger.info(f"Successfully deactivate audio")
                    return True
                else:
                    self.logger.error(f"Failed to deactivate: {response.message}")
                    return False
            except Exception as e:
                self.logger.error(f"System monitor service call failed: {e}")
                return False
        except Exception as e:
            self.logger.error(f"System monitor audio service wait timeout: {e}")
            return False

    def activate_camera_abs(self, cameras_setting: Dict[str, Dict]):
        """
        Activate cameras which using astribot_camera_abs driver by requesting system monitor.
        cameras_setting parameter is ignored now and will be support in later version.
        """
        try:
            client = self.node.create_client(SetBool, '/astribot_camera_control')

            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service /camera_control_command not available")

            try:
                req = SetBool.Request()
                req.data = True

                future = client.call_async(req)
                
                import time
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.001)

                if future.done():
                    response = future.result()
                else:
                    self.logger.error("Service call timeout")
                    return False

                if response.success:
                    self.logger.info(f"Successfully activate cameras")
                    return True
                else:
                    self.logger.error(f"Failed to activate: {response.message}")
                    return False
            except Exception as e:
                self.logger.error(f"System monitor service call failed: {e}")
                return False
        except Exception as e:
            self.logger.error(f"System monitor camera service wait timeout: {e}")
            return False

    def deactivate_camera_abs(self):
        """
        Deactivate cameras which using astribot_camera_abs driver by requesting system monitor.
        """
        try:
            client = self.node.create_client(SetBool, '/astribot_camera_control')

            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service /camera_control_command not available")

            try:
                req = SetBool.Request()
                req.data = False

                future = client.call_async(req)
                
                import time
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.001)

                if future.done():
                    response = future.result()
                else:
                    self.logger.error("Service call timeout")
                    return False

                if response.success:
                    self.logger.info(f"Successfully deactivate cameras")
                    return True
                else:
                    self.logger.error(f"Failed to deactivate: {response.message}")
                    return False
            except Exception as e:
                self.logger.error(f"System monitor service call failed: {e}")
                return False
        except Exception as e:
            self.logger.error(f"System monitor camera service wait timeout: {e}")
            return False

    def camera_name_to_topic_name(self, camera_name: str, image_type: str) -> str:
        """
        Convert camera name and image type to topic name.
        :param camera_name: Name of the camera.
        :param image_type: Type of the image ('color', 'depth', 'grey', 'ir').
        :return: Topic name.
        """
        if image_type == "color" or image_type == "grey":
            return f"/astribot_camera/{camera_name}/{image_type}_compress"
        elif image_type == "depth" or image_type == "ir":
            return f"/astribot_camera/{camera_name}/{image_type}_compress"
        else:
            return f"/astribot_camera/{camera_name}/{image_type}_image"
    
    def is_camera_activated(self, camera_name, image_type) -> bool:
        topic_name = self.camera_name_to_topic_name(camera_name, image_type)
        real_topic_name = topic_name
        if image_type != "depth":
            real_topic_name = f"{topic_name}/compressed"
        
        try:
            message_received = threading.Event()
            
            def temp_callback(msg):
                message_received.set()

            temp_sub = self.node.create_subscription(
                CompressedImage, real_topic_name, temp_callback, 10
            )
            if message_received.wait(timeout=0.2):
                self.node.destroy_subscription(temp_sub)
                return True
            else:
                self.node.destroy_subscription(temp_sub)
                return False
                
        except Exception as e:
            return False

    def camera_callback(self, msg, topic_name):
        """
        ROS image call back
        :param msg: message of CompressedImage
        """
        try:
            if self.need_decode_ == True and msg.format.lower() == "jpeg":
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                self.user_callback_(topic_name, msg, cv_image.shape[1], cv_image.shape[0], cv_image)
            else:
                cv_image = np.frombuffer(msg.data, np.uint8)
                self.user_callback_(topic_name, msg, 0, 0, cv_image)
        except Exception as e:
                self.logger.error(f"Error processing compressed image: {e}")
    
    def register_image_callback(self, camera_name: str, image_type: str, callback, need_decode: bool):
        topic_name = self.camera_name_to_topic_name(camera_name, image_type)
        if self.is_camera_activated(camera_name, image_type) is False:
            self._logger.warning(f"Camera {camera_name}({image_type}) still not activated now.")

        if not topic_name.startswith('/astribot_camera/'):
            self._logger.error("Invalid topic format")
            return None

        parts = topic_name.split('/')
        camera_name = parts[2]

        image_type = parts[3]

        if image_type == 'color_compress':
            topic_name_ = topic_name + "/compressed"
        else:
            topic_name_ = topic_name
        self.user_callback_ = callback
        self.need_decode_ = need_decode
        callback = lambda msg: self.camera_callback(msg, topic_name_)
        return self.node.create_subscription(
            CompressedImage, topic_name_, callback, 10,
            callback_group=ReentrantCallbackGroup()
        )

    def activate_lidar(self):
        try:
            client = self.node.create_client(SetBool, '/astribot_lidar_control')

            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service /astribot_lidar_control not available")

            try:
                req = SetBool.Request()
                req.data = True

                future = client.call_async(req)

                import time
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.001)

                if future.done():
                    response = future.result()
                else:
                    self.logger.error("Service call timeout")
                    return False

                if response.success:
                    self.logger.info(f"Successfully activate lidar")
                    return True
                else:
                    self.logger.error(f"Failed to activate: {response.message}")
                    return False
            except Exception as e:
                self.logger.error(f"System monitor service call failed: {e}")
                return False
        except Exception as e:
            self.logger.error(f"System monitor lidar service wait timeout: {e}")
            return False

    def deactivate_lidar(self):
        try:
            client = self.node.create_client(SetBool, '/astribot_lidar_control')

            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("Service /astribot_lidar_control not available")

            try:
                req = SetBool.Request()
                req.data = False

                future = client.call_async(req)

                import time
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    time.sleep(0.001)

                if future.done():
                    response = future.result()
                else:
                    self.logger.error("Service call timeout")
                    return False

                if response.success:
                    self.logger.info(f"Successfully deactivate lidar")
                    return True
                else:
                    self.logger.error(f"Failed to deactivate: {response.message}")
                    return False
            except Exception as e:
                self.logger.error(f"System monitor service call failed: {e}")
                return False
        except Exception as e:
            self.logger.error(f"System monitor camera service wait timeout: {e}")
            return False


    def set_effector_max_force(self, max_force):
        if max_force[0] is not None:
            left_gripper_max_force_request = self.node.create_client(
                RawRequest, "/gripper_left_max_force"
            )

            if not left_gripper_max_force_request.wait_for_service(timeout_sec=5.0):
                self.logger.error("Left gripper max force service not available")
                return

            request = {
                "gripper_max_force": max_force[0],
            }
            request_json = json.dumps(request)
            req = RawRequest.Request()
            req.request = request_json
            future = left_gripper_max_force_request.call_async(req)
            import time
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                time.sleep(0.001)
            if future.done():
                response = future.result()

        if max_force[1] is not None:
            right_gripper_max_force_request = self.node.create_client(
                RawRequest, "/gripper_right_max_force"
            )

            if not right_gripper_max_force_request.wait_for_service(timeout_sec=5.0):
                self.logger.error("Right gripper max force service not available")
                return

            request = {
                "gripper_max_force": max_force[1],
            }
            request_json = json.dumps(request)
            req = RawRequest.Request()
            req.request = request_json
            future = right_gripper_max_force_request.call_async(req)
            import time
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                time.sleep(0.001)
            if future.done():
                response = future.result()

