import os
import time
from pathlib import Path
import rclpy
from rclpy import qos
from rclpy.node import Node

from geometry_msgs.msg import Twist
from donkey_messages.msg import TwistSpeed

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from datetime import datetime

from .utils import constants as constants
from .environment.trajectory import Trajectory, ModelsList
from .models.unicycle import Unicycle
from .controllers.stanley import StanleyController, StanleyParams
from .utils.constants import MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, FREQUENCY
from .environment.rotations import Rotations
from .utils.tools import normalize_angle

timer_period = 0.5
DURATION = 30

show_animation = True

# Stanley controller parameters
K_THETA = 1.4
K_E = 0.25
EPSILON_V = 0.01
K_DELTA = 2.5


class BagNode(Node):

    def __init__(self, velocity_generator):
        super().__init__('bag_node')
        self.config_flag = True

        # ROS SETUP
        self.publisher_ = self.create_publisher(Twist, '/robo/cmd_vel', 10)
        self.subscription_vel = self.create_subscription(Twist, '/robo/cmd_vel', self.subscriber_callback_vel,
                                                     0)
        self.subscription_optitrack = self.create_subscription(TwistSpeed, '/robo/optiTrack', self.subscriber_callback_optitrack,
                                                     qos.qos_profile_sensor_data)
        # self.subscription
        # self.rate = self.create_rate(10)

        self.sub_x = 0.0
        self.sub_y = 0.0
        self.sub_z = 0.0
        self.sub_theta = 0.0
        self.timing = time.time()

        self.flag_optiTrack_ready = False

        # CONTROLLER SETUP
        self.velocity_generator = velocity_generator
        self.robot = None
        self.path = None
        self.reach_target = False
        self.goal_idx = 0
        self.controller = None

        # PLOT RESULT
        self.draw_x = []
        self.draw_y = []
        self.draw_theta = []
        self.draw_vel_controller = []
        self.draw_omega_controller = []
        self.draw_vel_optitrack = []
        self.draw_omega_optitrack = []
        self.draw_time = []
        self.draw_control_vel = []
        self.draw_control_omega = []

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # DEBUG: flags
        self.debug = 0
        self.append_it = False

    def subscriber_callback_optitrack(self, msg):
        self.flag_optiTrack_ready = True
        # x_world = z_optitrack
        # y_world = x_optitrack
        self.sub_x = round(msg.linear.z, 3)
        self.sub_y = round(msg.linear.x, 3)

        pitch = round(msg.angular.x, 3)
        yaw = round(msg.angular.y, 3)
        roll = round(msg.angular.z, 3)
        self.sub_theta = Rotations.compute_theta(pitch, yaw, roll)
        # print('OPT: x: %f y: %f z: %f' % (msg.linear.x, msg.linear.y, msg.linear.z))
        # print('OPT: pitch: %f yaw: %f roll: %f' % (pitch, yaw, roll))
        # print('SUB: x: %f y: %f theta: %f' % (self.sub_x, self.sub_y, np.rad2deg(self.sub_theta)))

    def subscriber_callback_vel(self, msg):
        self.draw_vel_controller.append(msg.linear.x)
        self.draw_omega_controller.append(msg.angular.z)

    def update_position(self):
        current_time = time.time()
        elapsed_time = current_time - self.timing

        # prendo ultime posizioni lette dal topic dell'optitrack
        current_x = self.sub_x
        current_y = self.sub_y
        current_theta = self.sub_theta
        # print('CURRENT: x: %f y: %f theta: %f' % (current_x, current_y, self.sub_theta))

        # TODO: controllo sul rumore -> dx e dy
        # sotto soglia di entrambi gli errori, velocità a zero

        dx = current_x - self.robot.x
        dy = current_y - self.robot.y
        dtheta = normalize_angle(current_theta - self.robot.theta)
        # dtheta = current_theta - self.robot.theta

        current_x_dot = dx / elapsed_time
        current_y_dot = dy / elapsed_time
        current_omega = dtheta / elapsed_time

        # update values
        self.robot.x = current_x
        self.robot.y = current_y
        self.robot.theta = current_theta
        self.robot.v = sqrt(current_x_dot ** 2 + current_y_dot ** 2)
        self.robot.omega = current_omega
        self.timing = current_time

        # if self.target_idx == self.goal_idx:
        #     self.reach_target = True
        #     self.send_velocity(0.0, 0.0)
        #     self.get_logger().info('Goal reached!')

        # UPDATE PLOT INFO
        self.draw_x.append(self.robot.x)
        self.draw_y.append(self.robot.y)
        self.draw_theta.append(self.robot.theta)
        self.draw_vel_optitrack.append(self.robot.v)
        self.draw_omega_optitrack.append(current_omega)
        self.draw_time.append(self.timing - self.draw_time[0])
        # in self.draw_time[0] viene salvato l'isatante di inizio del test

    def timer_callback(self):
        if not self.flag_optiTrack_ready:
            print("Waiting for Optitrack...")
            return

        if self.config_flag:
            # CONTROLLER SETUP
            self.robot = Unicycle(x=self.sub_x, y=self.sub_y, theta=self.sub_theta, v=0.0, omega=0.0)
            # self.path = Trajectory(model=ModelsList.UNICYCLE, start_x=self.sub_x, start_y=self.sub_y,
            #                        start_theta=self.sub_theta, velocity_generator=self.velocity_generator)
            _, _, self.s_test = self.velocity_generator()
            # self.goal_idx = len(self.path.x) - 1  # traguardo
            self.reach_target = False
            self.timing = 0.0
            # params = StanleyParams(K_THETA=K_THETA, K_E=K_E, EPSILON_V=EPSILON_V, K_DELTA=K_DELTA)
            # self.controller = StanleyController(self.path, params)

            # self.target_idx, _ = self.controller.calc_target_index(self.robot, self.path)

            self.draw_x.append(self.robot.x)
            self.draw_y.append(self.robot.y)
            self.draw_theta.append(self.robot.theta)
            self.draw_vel_controller.append(self.robot.v)
            self.draw_omega_controller.append(self.robot.omega)
            self.draw_vel_optitrack.append(self.robot.v)
            self.draw_omega_optitrack.append(self.robot.omega)
            self.draw_time.append(time.time())
            # self.draw_control_vel.append(0.0)
            # self.draw_control_omega.append(0.0)
            # self.draw_theta_error.append(0.0)
            # self.draw_cross_error.append(0.0)

            self.config_flag = False
            return

        self.debug += 1
        # DEBUG: time duration
        if (self.debug == (DURATION / timer_period)):
            self.reach_target = True
        # END DEBUG
        if not self.reach_target:  # if target is not reached yet
            # update position
            self.update_position()
            # controllers
            # di, target_idx = self.controller.control(self.robot)
            # ai = self.controller.longitudinal_velocity_controller(target_idx)
            # self.target_idx = target_idx
            #
            # # SAFE CHECK -> clipping velocities
            # ai = np.clip(ai, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
            #
            # di = np.clip(di, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
            #
            # self.draw_control_vel.append(ai)
            # self.draw_control_omega.append(di)
            # self.append_it = False

            # # send velocity
            # self.send_velocity(ai, di)


        else:

            print(len(self.draw_x))
            print(len(self.draw_y))
            print(len(self.draw_theta))
            print(len(self.draw_vel_controller))
            print(len(self.draw_omega_controller))
            print(len(self.draw_vel_optitrack))
            print(len(self.draw_omega_optitrack))
            print(len(self.draw_time))

            print("--------")

            print("controller:")
            print(self.draw_vel_controller)
            print("optitrack:")
            print(self.draw_vel_optitrack)


            # plot results and spinning forever
            now = datetime.now()
            dt_string = now.strftime("%d-%m-%H:%M:%S")
            home_path = str(Path.home())
            my_path = home_path + "/ros2_ws/src/doretta"

            self.draw_time[0] = 0.0  # istante di inizio viene aggiornato a 0.0

            # PATH
            plt.cla()
            # plt.plot(self.path.x, self.path.y, "-r", label="desired")
            # plt.plot(self.draw_x, self.draw_y, "-b", label="real")
            plt.plot(self.draw_time, self.draw_x, ".b", label="real")
            # plt.gcf().canvas.mpl_connect('key_release_event',
            #                             lambda event: [exit(0) if event.key == 'escape' else None])
            #plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
            plt.legend()
            plt.xlabel("time")
            plt.ylabel("x[m]")
            #plt.axis("equal")
            plt.grid(True)
            plt.savefig(os.path.join(my_path, 'test_results/BAG/BAG_pathX_%s.png' % (
             dt_string)))

            plt.cla()
            # plt.plot(self.path.x, self.path.y, "-r", label="desired")
            # plt.plot(self.draw_x, self.draw_y, "-b", label="real")
            plt.plot(self.draw_time, self.draw_y, ".b", label="real")
            # plt.gcf().canvas.mpl_connect('key_release_event',
            #                             lambda event: [exit(0) if event.key == 'escape' else None])
            # plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
            plt.legend()
            plt.xlabel("time")
            plt.ylabel("y[m]")
            #plt.axis("equal")
            plt.grid(True)
            plt.savefig(os.path.join(my_path, 'test_results/BAG/BAG_pathY_%s.png' % (
                dt_string)))

            # VELOCITY
            plt.subplot(2, 1, 1)
            plt.cla()
            plt.plot(self.draw_time, self.draw_vel_optitrack, "-r", label="REAL")
            plt.plot(self.draw_time, self.draw_vel_controller, "-b", label="CONTROLLER")
            # plt.gcf().canvas.mpl_connect('key_release_event',
            #                             lambda event: [exit(0) if event.key == 'escape' else None])
            #plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("linear velocity[m/s]")
            # plt.axis("equal")
            plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.cla()
            plt.plot(self.draw_time, self.draw_omega_optitrack, "-r", label="REAL")
            plt.plot(self.draw_time, self.draw_omega_controller, "-b", label="CONTROLLER")
            #plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("angular velocity[rad/s]")
            # plt.axis("equal")
            plt.grid(True)

            plt.savefig(os.path.join(my_path, 'test_results/BAG/BAG_vel_%s.png' % (
             dt_string)))

            # ERRORS
            # plt.subplot(2, 1, 1)
            # plt.cla()
            # plt.plot(self.draw_time, self.controller.theta_error, "-r", label="THETA")
            # plt.legend()
            # #plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
            # plt.xlabel("time[sec]")
            # plt.ylabel("theta error [rad]")
            # # plt.axis("equal")
            # plt.grid(True)
            #
            # plt.subplot(2, 1, 2)
            # plt.cla()
            # plt.plot(self.draw_time, self.controller.cross_error, "-r", label="CROSS")
            # plt.legend()
            # #plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
            # plt.xlabel("time[sec]")
            # plt.ylabel("cross error [m]")
            # # plt.axis("equal")
            # plt.grid(True)
            #
            # plt.savefig(os.path.join(my_path, 'test_results/BAG/BAG_%s_plot_err_%s.png' % (
            # self.s_test, dt_string)))

            print("FINISH!!!")
            self.destroy_node()
            rclpy.shutdown()
            while (True):
                pass


def main(args=None):
    rclpy.init(args=args)

    def velocity_test0():
        # linear velocity, angular velocity
        v = [1 / 20 * np.pi] * 15 * constants.FREQUENCY
        o = [np.pi / 15] * 15 * constants.FREQUENCY
        s = "velocity_test0"
        return v, o, s

    def velocity_test1():
        # linear velocity, angular velocity
        v = [3 / 32 * np.pi] * 8 * constants.FREQUENCY
        o = [np.pi / 8] * 8 * constants.FREQUENCY
        v = v + [3 / 32 * np.pi] * 8 * constants.FREQUENCY
        o = o + [-np.pi / 8] * 8 * constants.FREQUENCY
        s = "velocity_test1"
        return v, o, s

    def velocity_test2():
        # linear velocity, angular velocity
        v = [3 / 20 * np.pi] * 5 * constants.FREQUENCY
        o = [np.pi / 5] * 5 * constants.FREQUENCY
        v = v + [3 / 20 * np.pi] * 5 * constants.FREQUENCY
        o = o + [-np.pi / 5] * 5 * constants.FREQUENCY
        s = "velocity_test2"
        return v, o, s

    def velocity_test3():
        # linear velocity, angular velocity
        t_end = 60
        n_samples = t_end * constants.FREQUENCY
        radius = 0.8
        o = np.linspace(0.1, 1, n_samples).tolist()
        v = radius * o

        s = "velocity_test3"
        return v, o, s

    bag = BagNode(velocity_test0)

    rclpy.spin(bag)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stanley_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()