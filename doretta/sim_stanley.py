from datetime import datetime
import time
import os
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from doretta.utils import constants as constants
from doretta.environment.trajectory import Trajectory, ModelsList
from doretta.models.unicycle import Unicycle
from doretta.controllers.stanley import StanleyController, StanleyParams

import doretta.velocity_tests as vt

save = True

SIMULATION = True
simulation_max = 100000.0

# Stanley controller parameters
K_THETA = 1.0
K_E = 6.0
EPSILON_V = 0.01
K_DELTA = 12



def main():
    vel_gen = vt.velocity_test5B

    robot = Unicycle(x=0, y=1.5, theta=np.radians(180))
    

    traj = Trajectory(ModelsList.UNICYCLE, 0, 1.5, np.radians(180), vel_gen)
    _, _, s_test = vel_gen()

    params = StanleyParams(K_THETA=K_THETA, K_E=K_E, EPSILON_V=EPSILON_V, K_DELTA=K_DELTA)
    controller = StanleyController(path=traj, params=params)

    # plot results
    time_ = [time.time()]
    draw_x = [robot.x]
    draw_y = [robot.y]
    draw_theta = [robot.theta]
    draw_vel = [robot.v]
    draw_omega = [robot.omega]

    t = 0.0

    while not controller.goal_reached:  # if target is not reached yet
        # update position for robot -> ONLY with real robot

        # controllers
        # prima control, così recupero indice da passare al controller di longitudinal velocity
        o, index = controller.control(robot=robot)
        v = controller.longitudinal_velocity_controller(index)

        # SAFE CHECK -> clipping velocities
        v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)


        # send / update velocity
        robot.update(v, o)
        state = robot.state()
        draw_x.append(state[0])
        draw_y.append(state[1])
        draw_theta.append(state[2])
        draw_vel.append(state[3])
        draw_omega.append(state[4])
        time_.append(time.time() - time_[0])

        t += constants.DT
        if t > simulation_max:
            print("MAXIMUM TIME ALLOWED REACHED")
            break

    time_[0] = 0  # istante di inizio viene aggiornato a 0.0

    # plot results and spinning forever
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%H:%M:%S")

    home_path = str(Path.home())
    my_path = home_path + "/ros2_ws/src/doretta"
    
    # DEBUG
    # print("Time: " + str(self.draw_time))
    # print("Omega: " + str(self.draw_control_omega))

    # PATHs
    plt.subplots(1)
    plt.cla()
    plt.plot(traj.x, traj.y, "-r", label="desired")
    plt.plot(draw_x, draw_y, "-b", label="real")
    plt.gcf().canvas.mpl_connect('key_release_event',
                                lambda event: [exit(0) if event.key == 'escape' else None])
    plt.legend()
    plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)
    if save:
        plt.savefig(os.path.join(my_path, 'test_results/SIM_STANLEY/SIM_STAN_%s_plot_path_%s.png' % (s_test, dt_string)))

    # # VELOCITY
    plt.subplots(1)
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.plot(time_, draw_vel, "-r", label="REAL")
    # plt.gcf().canvas.mpl_connect('key_release_event',
    #                             lambda event: [exit(0) if event.key == 'escape' else None])
    plt.legend()
    plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
    plt.xlabel("time[sec]")
    plt.ylabel("linear velocity[m/s]")
    # plt.axis("equal")
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.cla()
    plt.plot(time_, draw_omega, "-r", label="REAL")
    # plt.plot(self.draw_time, self.draw_control_omega, "-b", label="CONTROLLER")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("angular velocity[rad/s]")
    # plt.axis("equal")
    plt.grid(True)
    
    if save:
        plt.savefig(os.path.join(my_path, 'test_results/SIM_STANLEY/SIM_STAN_%s_plot_vel_%s.png' % (s_test, dt_string)))
    
    # # ERRORS
    plt.subplots(1)
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.plot(time_, controller.theta_error, "-r", label="THETA")
    plt.legend()
    plt.suptitle("K_THETA: %.2f  K_E: %.2f  K_DELTA: %.2f" % (K_THETA, K_E, K_DELTA))
    plt.xlabel("time[sec]")
    plt.ylabel("theta error [rad]")
    # plt.axis("equal")
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.cla()
    plt.plot(time_, controller.cross_error, "-r", label="CROSS")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("cross error [m]")
    # plt.axis("equal")
    plt.grid(True)
    
    if save:
        plt.savefig(os.path.join(my_path, 'test_results/SIM_STANLEY/SIM_STAN_%s_plot_err_%s.png' % (s_test, dt_string)))

    print("FINISH!!!")

    if not save:
        plt.show()


if __name__ == "__main__":
    main()
