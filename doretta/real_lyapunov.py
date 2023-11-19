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
from .controllers.lyapunov import LyapunovController, LyapunovParams
from .utils.constants import MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY, FREQUENCY
from .environment.rotations import Rotations
from .utils.tools import normalize_angle

import doretta.velocity_tests as vt

timer_period = 0.05

# Lyapunov controller parameters
K_P = 1.6
K_THETA = 1.4

class LyapunovControllerNode(Node):

    def __init__(self, velocity_generator):
        super().__init__('lyapunov_controller_node')
        self.config_flag = True

        # ROS SETUP
        self.publisher_ = self.create_publisher(Twist, '/robo/cmd_vel', 10)
        self.subscription = self.create_subscription(TwistSpeed, '/robo/optiTrack', self.subscriber_callback,
                                                     qos.qos_profile_sensor_data)
        self.subscription
        # self.rate = self.create_rate(10)

        self.sub_x = 0.0
        self.sub_y = 0.0
        self.sub_z = 0.0
        self.sub_theta = 0.0
        self.timing = time.time()

        self.flag_optiTrack_ready = False

        # CONTROLLER SETUP
        _, _, self.s_test = velocity_generator()
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
        self.draw_vel = []
        self.draw_omega = []
        self.draw_time = []
        self.draw_control_vel = []
        self.draw_control_omega = []

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # DEBUG: flags
        self.debug = 0
        self.append_it = False

    def subscriber_callback(self, msg):
        self.flag_optiTrack_ready = True
        # x_world = z_optitrack
        # y_world = x_optitrack
        self.sub_x = round(msg.linear.z, 3)
        self.sub_y = round(msg.linear.x, 3)
        # TODO fix sub_time

        pitch = round(msg.angular.x, 3)
        yaw = round(msg.angular.y, 3)
        roll = round(msg.angular.z, 3)
        self.sub_theta = Rotations.compute_theta(pitch, yaw, roll)
        # print('OPT: x: %f y: %f z: %f' % (msg.linear.x, msg.linear.y, msg.linear.z))
        # print('OPT: pitch: %f yaw: %f roll: %f' % (pitch, yaw, roll))
        # print('SUB: x: %f y: %f theta: %f' % (self.sub_x, self.sub_y, np.rad2deg(self.sub_theta)))

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

        # DEBUG update print
        # print('UPDATE: x: %f y: %f theta: %f vel: %f current_idx: %d' % (current_x, current_y, np.rad2deg(current_theta), self.robot.v, self.target_idx))

        # UPDATE PLOT INFO
        self.draw_x.append(self.robot.x)
        self.draw_y.append(self.robot.y)
        self.draw_theta.append(self.robot.theta)
        self.draw_vel.append(self.robot.v)
        self.draw_omega.append(current_omega)
        self.draw_time.append(self.timing - self.draw_time[0])
        # in self.draw_time[0] viene salvato l'isatante di inizio del test

    def send_velocity(self, vel, omega):
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = omega
        self.publisher_.publish(msg)

    def timer_callback(self):
        if not self.flag_optiTrack_ready:
            print("Waiting for Optitrack...")
            return

        if self.config_flag:
            # CONTROLLER SETUP
            self.robot = Unicycle(x=self.sub_x, y=self.sub_y, theta=self.sub_theta, v=0.0, omega=0.0)
            self.path = Trajectory(model=ModelsList.UNICYCLE, start_x=self.sub_x, start_y=self.sub_y, start_theta=self.sub_theta, velocity_generator=self.velocity_generator)

            self.reach_target = False

            params = LyapunovParams(K_P=K_P, K_THETA=K_THETA)
            self.controller = LyapunovController(params)
            start_time = time.time()
            self.controller.config(start_x=self.sub_x, start_y=self.sub_y, start_theta=self.sub_theta, start_time=start_time,
                              velocity_generator=self.velocity_generator)

            self.draw_x.append(self.robot.x)
            self.draw_y.append(self.robot.y)
            self.draw_theta.append(self.robot.theta)
            self.draw_vel.append(self.robot.v)
            self.draw_omega.append(self.robot.omega)
            self.draw_time.append(start_time)
            self.draw_control_vel.append(0.0)
            self.draw_control_omega.append(0.0)
            # self.draw_theta_error.append(0.0)
            # self.draw_cross_error.append(0.0)

            self.config_flag = False
            return

        self.debug += 1
        # DEBUG: time duration
        # if (self.debug == (15 / timer_period)):
        #     self.reach_target = True
        # END DEBUG
        if not self.controller.goal_reached:  # if target is not reached yet
            # update position
            self.update_position()
            # controllers
            ai, di = self.controller.control(self.robot, time.time())

            # SAFE CHECK -> clipping velocities
            ai = np.clip(ai, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)

            di = np.clip(di, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)

            # DEBUG send print
            # print("SEND - v: %f om: %f" % (ai, di))
            # if self.append_it == True:
            self.draw_control_vel.append(ai)
            self.draw_control_omega.append(di)
            # self.append_it = False

            # # send velocity
            self.send_velocity(ai, di)


        else:
            # plot results and spinning forever
            now = datetime.now()
            dt_string = now.strftime("%d-%m-%H:%M:%S")

            home_path = str(Path.home())
            my_path = home_path + "/ros2_ws/src/doretta"
            self.draw_time[0] = 0.0  # istante di inizio viene aggiornato a 0.0

            # DEBUG
            # print("Time: " + str(self.draw_time))
            # print("Omega: " + str(self.draw_control_omega))

            # PATH
            plt.cla()
            plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
            plt.plot(self.path.x, self.path.y, "-r", label="desired")
            plt.plot(self.draw_x, self.draw_y, "-b", label="real")
            # plt.gcf().canvas.mpl_connect('key_release_event',
            #                             lambda event: [exit(0) if event.key == 'escape' else None])
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)
            plt.savefig(os.path.join(my_path, 'test_results/REAL_LYAPUNOV/REAL_LYAP_%s_plot_path_%s.png' % (self.s_test, dt_string)))

            # VELOCITY
            plt.subplot(2, 1, 1)
            plt.cla()
            plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
            # plt.plot(self.draw_time, self.draw_vel, "-r", label="REAL")
            plt.plot(self.draw_time, self.draw_control_vel, "-b", label="CONTROLLER")
            # plt.gcf().canvas.mpl_connect('key_release_event',
            #                             lambda event: [exit(0) if event.key == 'escape' else None])
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("linear velocity[m/s]")
            # plt.axis("equal")
            plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.cla()
            # plt.plot(self.draw_time, self.draw_omega, "-r", label="REAL")
            plt.plot(self.draw_time, self.draw_control_omega, "-b", label="CONTROLLER")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("angular velocity[rad/s]")
            # plt.axis("equal")
            plt.grid(True)

            plt.savefig(os.path.join(my_path, 'test_results/REAL_LYAPUNOV/REAL_LYAP_%s_plot_vel_%s.png' % (self.s_test, dt_string)))

            # ERRORS
            plt.subplots(1)
            plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
            plt.subplot(3, 1, 1)
            plt.cla()
            plt.plot(self.draw_time, self.controller.draw_e_x, "-r", label="e_x")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("e_x [m]")
            # plt.axis("equal")
            plt.grid(True)

            plt.subplot(3, 1, 2)
            plt.cla()
            plt.plot(self.draw_time, self.controller.draw_e_y, "-r", label="e_y")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("e_y [m]")
            # plt.axis("equal")
            plt.grid(True)

            plt.subplot(3, 1, 3)
            plt.cla()
            plt.plot(self.draw_time, self.controller.draw_e_theta, "-r", label="e_theta")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("e_theta [rad]")
            # plt.axis("equal")
            plt.grid(True)

            plt.savefig(os.path.join(my_path, 'test_results/REAL_LYAPUNOV/REAL_LYAP_%s_plot_err_%s.png' % (self.s_test, dt_string)))

            print("FINISH!!!")
            self.destroy_node()
            rclpy.shutdown()
            while (True):
                pass


def main(args=None):
    rclpy.init(args=args)

    lyapunov_controller = LyapunovControllerNode(vt.velocity_test1)

    rclpy.spin(lyapunov_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lyapunov_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()