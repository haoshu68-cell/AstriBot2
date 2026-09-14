import numpy as np
from sensor_msgs.msg import Joy
import astribot_ros_middleware as ast_ros_middleware

class XboxController:
    def __init__(self, mode='chassis_control'):
        if mode == 'chassis_control':
            self.subscription = ast_ros_middleware.create_subscriber(
                Joy,
                '/astribot_joy',
                self.joy_callback_for_chassis_vel,
                10)
            self.stop_flag = False
            self.last_key = 0
            self.vel = np.zeros(3)
        elif mode == 'traj_replay':
            self.subscription = ast_ros_middleware.create_subscriber(
                Joy,
                '/astribot_joy',
                self.joy_callback_for_traj_replay,
                10)
        
        self.have_value = False
        self.timer = ast_ros_middleware.create_timer(0.5, self.check_joy_value)

    def check_joy_value(self):
        if not self.have_value:
            self.vel = np.zeros(3)
            print("Note that no joystick message is received. " \
            "Please check whether rostopic '/astribot_joy' exists, " \
            "whether the joystick driver is started, and whether the joystick is powered.")

    def joy_callback_for_chassis_vel(self, joy):
        self.have_value = True
        x_vel = joy.axes[1] * 0.5
        y_vel = joy.axes[0] * 0.5
        yaw_vel = joy.axes[3] * 0.5
        self.vel[0] = x_vel
        self.vel[1] = y_vel
        self.vel[2] = yaw_vel

    def get_vel(self):
        return self.vel
    
    def joy_callback_for_traj_replay(self, joy):
        pass
