#!/usr/bin/env python3

import casadi as ca
import numpy as np
import time
import math
import rospy
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from geometry_msgs.msg import Twist, Pose

class Draw_MPC_tracking(object):
    def __init__(self, robot_states: list, init_state: np.array, rob_diam=0.3, export_fig=False):
        self.init_state = init_state
        self.robot_states = robot_states
        self.rob_radius = rob_diam
        self.fig = plt.figure(figsize=(7,7))
        self.ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))
        # self.fig.set_size_inches(7, 6.5)
        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig:
            self.ani.save('tracking.gif', writer='imagemagick', fps=100)
        plt.show()

    def animation_init(self, ):
        # draw target line
        ##self.target_line = plt.plot([0, 20], [1, 1], '-r')
        # draw the initial position of the robot
        self.init_robot_position = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.init_robot_position)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        ## return self.target_line, self.init_robot_position, self.robot_body, self.robot_arr
        return self.init_robot_position, self.robot_body, self.robot_arr

    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]
        self.robot_body.center = position
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(position[0], position[1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.robot_arr, self.robot_body


def shift_movement(T, t0, x0, u, x_n, f):
    f_value = f(x0, u[0])
    st = x0 + T*f_value
    t = t0 + T
    u_end = np.concatenate((u[1:], u[-1:]))
    x_n = np.concatenate((x_n[1:], x_n[-1:]))
    return t, st, u_end, x_n

def forward_kinematic(v1, v2, v3, v4, theta):
    x = (-0.5)*(v1*math.sin(theta+a1)+v2*math.sin(theta+a2)+v3*math.sin(theta+a3)+v4*math.sin(theta+a4))
    y = (0.5)*(v1*math.cos(theta+a1)+v2*math.cos(theta+a2)+v3*math.cos(theta+a3)+v4*math.cos(theta+a4))
    yaw = (v1+v2+v3+v4)*(4*0.52)

    return x, y, yaw

def inverse_kinematic(x, y, theta):
    v1 = 0.52*theta-x*math.sin(theta+a1)+y*math.cos(theta+a1)
    v2 = 0.52*theta-x*math.sin(theta+a2)+y*math.cos(theta+a2)
    v3 = 0.52*theta-x*math.sin(theta+a3)+y*math.cos(theta+a3)
    v4 = 0.52*theta-x*math.sin(theta+a4)+y*math.cos(theta+a4)

    return v1, v2, v3, v4

def forward_kinematic(v1, v2, v3, v4, theta):
    x = (-0.5)*(v1*math.sin(theta+a1)+v2*math.sin(theta+a2)+v3*math.sin(theta+a3)+v4*math.sin(theta+a4))
    y = (0.5)*(v1*math.cos(theta+a1)+v2*math.cos(theta+a2)+v3*math.cos(theta+a3)+v4*math.cos(theta+a4))
    yaw = (v1+v2+v3+v4)*(4*0.52)

    return x, y, yaw

def inverse_kinematic(x, y, theta):
    v1 = 0.52*theta-x*math.sin(theta+a1)+y*math.cos(theta+a1)
    v2 = 0.52*theta-x*math.sin(theta+a2)+y*math.cos(theta+a2)
    v3 = 0.52*theta-x*math.sin(theta+a3)+y*math.cos(theta+a3)
    v4 = 0.52*theta-x*math.sin(theta+a4)+y*math.cos(theta+a4)

    return v1, v2, v3, v4

def desired_command_and_trajectory(t, T, x0_:np.array, N_):
    # initial state / last state
    x_ = np.zeros((N_+1, 3))
    x_[0] = x0_
    u_ = np.zeros((N_, 4))
    # states for the next N_ trajectories
    for i in range(N_):
        t_predict = t + T*i
        x_ref_ = 2.5*math.cos(math.pi/5*t_predict)
        y_ref_ = 2.5*math.sin(math.pi/5*t_predict)
        theta_ref_ = math.pi/5*t_predict
        v1_ref_, v2_ref_, v3_ref_, v4_ref_ = inverse_kinematic(x_ref_, y_ref_, theta_ref_)
        if x_ref_ == 1.84:
            v1_ref_ = 0
            v2_ref_ = 0
            v3_ref_ = 0
            v4_ref_ = 0
            break
        x_[i+1] = np.array([x_ref_, y_ref_, theta_ref_])
        u_[i] = np.array([v1_ref_, v2_ref_, v3_ref_, v4_ref_])
        ##plt.cla()
        ##plt.plot(x_ref_, y_ref_, "-r")
        ##plt.axis("equal")
        ##plt.pause(0.0005)
    # return pose and command
    return x_, u_



if __name__ == '__main__':
    pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    pub_pos = rospy.Publisher("robot_pose", Pose, queue_size=10)
    twist = Twist()
    pose = Pose()
    rospy.init_node("mpc_omni_casadi", anonymous=True)
    a1 = math.pi/4
    a2 = 3*math.pi/4
    a3 = 5*math.pi/4
    a4 = 7*math.pi/4
    T = 0.2
    N = 100
    rob_diam = 0.3 # [m]
    v_max = 0.4

    opti = ca.Opti()
    # control variables, linear velocity v and angle velocity omega
    opt_controls = opti.variable(N, 4)
    v1 = opt_controls[:, 0]
    v2 = opt_controls[:, 1]
    v3 = opt_controls[:, 2]
    v4 = opt_controls[:, 3]
    opt_states = opti.variable(N+1, 3)
    x = opt_states[:, 0]
    y = opt_states[:, 1]
    theta = opt_states[:, 2]

    # parameters, these parameters are the reference trajectories of the pose and inputs
    opt_u_ref = opti.parameter(N, 4)
    opt_x_ref = opti.parameter(N+1, 3)
    # create model
    f = lambda x_, u_: ca.vertcat(u_[0]*ca.sin(x_[2]+a1)+u_[1]*ca.sin(x_[2]+a2)+u_[2]*ca.sin(x_[2]+a3)+u_[3]*ca.sin(x_[2]+a4),
                                  u_[0]*ca.cos(x_[2]+a1)+u_[1]*ca.cos(x_[2]+a2)+u_[2]*ca.cos(x_[2]+a3)+u_[3]*ca.cos(x_[2]+a4),
                                  (u_[0]+u_[1]+u_[2]+u_[3])/(4*0.52))
    f_np = lambda x_, u_: np.array([u_[0]*np.sin(x_[2]+a1)+u_[1]*np.sin(x_[2]+a2)+u_[2]*np.sin(x_[2]+a3)+u_[3]*np.sin(x_[2]+a4),
                                    u_[0]*np.cos(x_[2]+a1)+u_[1]*np.cos(x_[2]+a2)+u_[2]*np.cos(x_[2]+a3)+u_[3]*np.cos(x_[2]+a4),
                                    (u_[0]+u_[1]+u_[2]+u_[3])/(4*0.52)])

    ## init_condition
    opti.subject_to(opt_states[0, :] == opt_x_ref[0, :])
    for i in range(N):
        x_next = opt_states[i, :] + f(opt_states[i, :], opt_controls[i, :]).T*T
        opti.subject_to(opt_states[i+1, :]==x_next)

    ## define the cost function
    ### some addition parameters
    Q = np.diag([1, 1, 1])
    R = np.diag([1, 1, 0.5, 0.5])
    #### cost function
    obj = 0 #### cost
    for i in range(N):
        state_error_ = opt_states[i, :] - opt_x_ref[i+1, :]
        control_error_ = opt_controls[i, :] - opt_u_ref[i, :]
        obj = obj + ca.mtimes([state_error_, Q, state_error_.T]) + ca.mtimes([control_error_, R, control_error_.T])
    opti.minimize(obj)

    #### boundrary and control conditions
    opti.subject_to(opti.bounded(-20, x, 20))
    opti.subject_to(opti.bounded(-20, y, 20))
    opti.subject_to(opti.bounded(-math.pi, theta, math.pi))
    opti.subject_to(opti.bounded(-v_max, v1, v_max))
    opti.subject_to(opti.bounded(-v_max, v2, v_max))
    opti.subject_to(opti.bounded(-v_max, v3, v_max))
    opti.subject_to(opti.bounded(-v_max, v4, v_max))

    opts_setting = {'ipopt.max_iter':2000, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}

    opti.solver('ipopt', opts_setting)

    t0 = 0
    init_state = np.array([0.0, 0.0, 0.0])
    current_state = init_state.copy()
    u0 = np.zeros((N, 4))
    next_trajectories = np.tile(init_state, N+1).reshape(N+1, -1) # set the initial state as the first trajectories for the robot
    next_controls = np.zeros((N, 4))
    next_states = np.zeros((N+1, 3))
    x_c = [] # contains for the history of the state
    u_c = []
    t_c = [t0] # for the time
    xx = []
    sim_time = 30.0

    ## start MPC
    mpciter = 0
    start_time = time.time()
    index_t = []
    while(mpciter-sim_time/T<0.0):
        ## set parameter, here only update initial state of x (x0)
        opti.set_value(opt_x_ref, next_trajectories)
        opti.set_value(opt_u_ref, next_controls)
        ## provide the initial guess of the optimization targets
        opti.set_initial(opt_controls, u0.reshape(N, 4))# (N, 2)
        opti.set_initial(opt_states, next_states) # (N+1, 3)
        ## solve the problem once again
        t_ = time.time()
        sol = opti.solve()
        index_t.append(time.time()- t_)
        ## obtain the control input
        u_res = sol.value(opt_controls)
        x_m = sol.value(opt_states)
        # print(x_m[:3])
        u_c.append(u_res[0, :])
        ##print(u_c)
        t_c.append(t0)
        x_c.append(x_m)
        t0, current_state, u0, next_states = shift_movement(T, t0, current_state, u_res, x_m, f_np)
        xx.append(current_state)
        vel_x, vel_y, vel_theta = forward_kinematic(u_res[0, 0], u_res[1, 1], u_res[2, 2], u_res[3, 3], x_m[2, 2])
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = vel_theta
        pub_vel.publish(twist)
        pose.position.x = x_m[0, 0]
        pose.position.y = x_m[1, 1]
        pose.position.z = x_m[2, 2]
        pub_pos.publish(pose)
        ## estimate the new desired trajectories and controls
        next_trajectories, next_controls = desired_command_and_trajectory(t0, T, current_state, N)
        #plt.cla()
        #plt.plot(next_trajectories[mpciter, 0], next_trajectories[mpciter, 1], "-b", label="trajectory")
        #plt.pause(0.0005)
        #print(next_trajectories)
        mpciter = mpciter + 1
        rospy.Rate(10)
        #print(mpciter)



    ## draw function
    draw_result = Draw_MPC_tracking(rob_diam=0.3, init_state=init_state, robot_states=xx )