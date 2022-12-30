#!/usr/bin/env python

import rospy
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import math

from geometry_msgs.msg import Twist, Pose

# Setup ros publisher

pub_pose = rospy.Publisher('Pose', Pose, queue_size=10)
pub_velocity = rospy.Publisher('cmd_vel', Twist, queue_size=10)


# Class call back

class state_omni():

    def __init__(self):
        # Params
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.sub_state = rospy.Subscriber('odom_mpc', Pose, self.callback)

    def callback(self, msg):
        self.current_state[0] = msg.position.x
        self.current_state[1] = msg.position.y
        self.current_state[2] = msg.orientation.z
        rospy.loginfo("Current State: {}".format(self.current_state))

# ROS callback function

def robot_pose(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = roll
    pose.orientation.y = pitch
    pose.orientation.z = yaw
    pub_pose.publish(pose)

def robot_velocity(vx, vy, vth):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = vth
    pub_velocity.publish(twist)

# Robot Params
# Setting matrix_weight's variables, works good with circle trajectories
Q_x = 5000
Q_y = 5000
Q_theta = 5000
R1 = 1
R2 = 1
R3 = 1
R4 = 1
dt = 0.1

# Model and MPC params
a1 = math.pi/4
a2 = 3*math.pi/4
a3 = 5*math.pi/4
a4 = 7*math.pi/4
dt = 0.1

# Define the boundary of the problem
x_max = 5.5
x_min = -x_max
y_max = 6
y_min = 0.975
theta_max = np.pi/2
theta_min = -theta_max
v_max = 1.5
v_min = -v_max
step_horizon = 0.1 # time between steps in seconds
N = 10 # the prediction horizontal
sim_time = 6.67 # simulation time

show_animation = True

class OMNI_ROBOT:

    def __init__(self, r_h=0.35, r_w=0.35, w_h=0.025, w_w=0.1, rot_angle=45.0, w_pos=0.35):

        # Default parameter for omni robot 4 wheeled
        ## Angle default
        self.a1 = math.pi/4
        self.a2 = 3*math.pi/4
        self.a3 = 5*math.pi/4
        self.a4 = 7*math.pi/4
        ## robot shape
        self.robot_h = r_h
        self.robot_w = r_w
        ## wheel shaspe
        self.wheel_h = w_h
        self.wheel_w = w_w
        ## wheel position
        self.w_pos = w_pos
        ## Rotation angle
        self.rotate_angle = rot_angle

    def rotation_angle(self, theta):
        matrix_transform = np.array([[np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]])
        return matrix_transform


    def robot_shape(self):
        matrix_shape = np.array([
            [-self.robot_w, self.robot_w, self.robot_w, -self.robot_w, -self.robot_w],
            [self.robot_h, self.robot_h, -self.robot_h, -self.robot_h, self.robot_h]
        ])
        return matrix_shape

    def robot_wheel(self):
        matrix_wheel = np.array([
            [-self.wheel_w, self.wheel_w, self.wheel_w, -self.wheel_w, -self.wheel_w],
            [self.wheel_h, self.wheel_h, -self.wheel_h, -self.wheel_h, self.wheel_h]
        ])
        return matrix_wheel

    def generate_each_wheel_and_draw(self, x, y, yaw):
        # Copy Matrix as robot_shape
        pos_wheel = self.robot_wheel()
        pos_wheel1 = pos_wheel.copy()
        pos_wheel2 = pos_wheel.copy()
        pos_wheel3 = pos_wheel.copy()
        pos_wheel4 = pos_wheel.copy()
        # Transpose wheel to each small length
        pos_wheel1 = np.dot(pos_wheel1.T, self.rotation_angle(-self.rotate_angle)).T
        pos_wheel2 = np.dot(pos_wheel2.T, self.rotation_angle(self.rotate_angle)).T
        pos_wheel3 = np.dot(pos_wheel3.T, self.rotation_angle(-self.rotate_angle)).T
        pos_wheel4 = np.dot(pos_wheel4.T, self.rotation_angle(self.rotate_angle)).T
        # Push each wheel to where it belong
        pos_wheel1[0, :] += self.w_pos
        pos_wheel1[1, :] -= self.w_pos
        pos_wheel2[0, :] += self.w_pos
        pos_wheel2[1, :] += self.w_pos
        pos_wheel3[0, :] -= self.w_pos
        pos_wheel3[1, :] += self.w_pos
        pos_wheel4[0, :] -= self.w_pos
        pos_wheel4[1, :] -= self.w_pos
        # Matrix Transforms each wheel 1, 2, 3, 4
        pos_wheel1 = np.dot(pos_wheel1.T, self.rotation_angle(yaw)).T
        pos_wheel2 = np.dot(pos_wheel2.T, self.rotation_angle(yaw)).T
        pos_wheel3 = np.dot(pos_wheel3.T, self.rotation_angle(yaw)).T
        pos_wheel4 = np.dot(pos_wheel4.T, self.rotation_angle(yaw)).T

        # Matrix Transforms robot shape
        robot_shape = self.robot_shape()
        robot_shaped = np.dot(robot_shape.T, self.rotation_angle(yaw)).T

        plt.plot(robot_shaped[0, :]+x, robot_shaped[1, :]+y, color="blue")
        plt.plot(pos_wheel1[0, :]+x, pos_wheel1[1, :]+y, color="black")
        plt.plot(pos_wheel2[0, :]+x, pos_wheel2[1, :]+y, color="black")
        plt.plot(pos_wheel3[0, :]+x, pos_wheel3[1, :]+y, color="black")
        plt.plot(pos_wheel4[0, :]+x, pos_wheel4[1, :]+y, color="black")

# Arrow
def plot_arrow(x, y, yaw, length=0.05, width=0.3, fc="b", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
            fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

# The forward kinematic model
def forward_kinematic(v1, v2, v3, v4, theta):
    v_x = (-0.5)*(v1*np.sin(a1+theta)+v2*np.sin(a2+theta)+v3*np.sin(a3+theta)+v4*np.sin(a4+theta))
    v_y = (0.5)*(v1*np.cos(a1+theta)+v2*np.cos(a2+theta)+v3*np.cos(a3+theta)+v4*np.cos(a4+theta))
    v_theta = (v1+v2+v3+v4)/(4*0.245)
    return v_x, v_y, v_theta

# The inverse kinematic model
def inverse_kinematic(v_x, v_y, v_theta, theta):
    v1 = 0.061*v_theta-v_x*np.sin(theta+a1)+v_y*np.cos(theta+a1)
    v2 = 0.061*v_theta-v_x*np.sin(theta+a2)+v_y*np.cos(theta+a2)
    v3 = 0.061*v_theta-v_x*np.sin(theta+a3)+v_y*np.cos(theta+a3)
    v4 = 0.061*v_theta-v_x*np.sin(theta+a4)+v_y*np.cos(theta+a4)

    return v1, v2, v3, v4

# Function to calculate velocity using discrete set points
def velocity_from_discrete_points(k, dt, x_ref, y_ref, theta_ref):
    vx = (x_ref[k]-x_ref[k-1])/dt
    vy = (y_ref[k]-y_ref[k-1])/dt
    vth = (theta_ref[k]-theta_ref[k-1])/dt

    return vx, vy, vth
# Function to calculate reference trajectory
def reference_state_and_control(t, step_horizon, x0, N, type="None"):
    # initial state
    x = x0.reshape(1, -1).tolist()[0]
    u = []
    # set points for calculating reference control
    x_ref, y_ref, theta_ref = [], [], []
    x_ref_, y_ref_, theta_ref_ = 0, 0, 0
    vx, vy, vth = 0, 0, 0
    # state for N predictions
    for k in range(N):
        t_predict = t + step_horizon * k
        if type == "circle":
            angle = math.pi/10*t_predict
            x_ref_ = 5*math.cos(angle)
            y_ref_ = 5*math.sin(angle)
            theta_ref_ = 0.0
            # Inserting all set points
            x.append(x_ref_)
            x.append(y_ref_)
            x.append(theta_ref_)
            x_ref.append(x_ref_)
            y_ref.append(y_ref_)
            theta_ref.append(theta_ref_)
            vx, vy, vth = velocity_from_discrete_points(k, 0.1, x_ref, y_ref, theta_ref)
            if t_predict >= 18.0:
                vx, vy = 0.0, 0.0
            v_ref1, v_ref2, v_ref3, v_ref4 = inverse_kinematic(vx, vy, vth, theta_ref[k])
            u.append(v_ref1)
            u.append(v_ref2)
            u.append(v_ref3)
            u.append(v_ref4)

        if type == "8shaped":
            x_ref_ = 3 + 3 * np.sin(2*np.pi*t_predict/25)
            y_ref_ = 2*np.sin(4*np.pi*t_predict/25)
            theta_ref_ = np.pi/2
            # Inserting all set points
            x.append(x_ref_)
            x.append(y_ref_)
            x.append(theta_ref_)
            x_ref.append(x_ref_)
            y_ref.append(y_ref_)
            theta_ref.append(theta_ref_)
            vx, vy, vth = velocity_from_discrete_points(k, 0.1, x_ref, y_ref, theta_ref)
            if t_predict >= 18.0:
                vx, vy = 0.0, 0.0
            v_ref1, v_ref2, v_ref3, v_ref4 = inverse_kinematic(vx, vy, vth, theta_ref[k])
            u.append(v_ref1)
            u.append(v_ref2)
            u.append(v_ref3)
            u.append(v_ref4)
        if type == "traj1":
            #if (t_predict <= 10.05):
            #    x_ref_ = 0.975
            #    y_ref_ = 6-0.5*t_predict
            #    theta_ref_ = 0.0
            #if ((t_predict > 10.05) and (t_predict <= 18.1)):
            #    x_ref_ = 0.5*t_predict-5.025+0.975
            #    y_ref_ = 0.975
            #    theta_ref_ = ((np.pi/2)/18.1*t_predict
            #if ((t_predict > 18.1) and (t_predict <= 20.9)):
            #    x_ref_ = 5
            #    y_ref_ = 0.5*t_predict - 9.05 + 0.975
            #    theta_ref_ = np.pi/2
            #if ((t_predict > 20.9) and (t_predict <= 24.2)):
            #    x_ref_ = 5-0.5*t_predict+10.45
            #    y_ref_ = 2.375
            #    theta_ref_ = np.pi/2
            #if ((t_predict > 24.2) and (t_predict <= 31.45)):
            #    x_ref_ = 3.35
            #    y_ref_ = 0.5*t_predict-12.1+2.375
            #    theta_ref_ = np.pi/2-((np.pi/2)/31.45*t_predict)
            if (t_predict <= 3.35):
                x_ref_ = 0.975
                y_ref_ = 6-1.5*t_predict
                theta_ref_ = 0.0
            if ((t_predict > 3.35) and (t_predict <= 6.3666)):
                x_ref_ = 1.5*t_predict - 5.025 +0.975
                y_ref_ = 0.975
                theta_ref_ = ((np.pi/2)/(6.3666))*t_predict
            if ((t_predict > 6.3666) and (t_predict <= 7.94933)):
                x_ref_ = 5.5
                y_ref_ = 1.5*t_predict+0.975-9.549
                theta_ref_ = np.pi/2
            if ((t_predict > 7.94933) and (t_predict <= 9.3826)):
                x_ref_ = 5.5-1.5*t_predict+11.9239
                y_ref_ = 2.375+0.975
                theta_ref_ = np.pi/2
            if ((t_predict > 9.3826) and (t_predict <= 11.79)):
                x_ref_ = 3.35
                y_ref_ = 1.5*t_predict-14.0739+2.375+0.975
                theta_ref_ = (np.pi/2)-((np.pi/2)/(11.79))*t_predict
            x.append(x_ref_)
            x.append(y_ref_)
            x.append(theta_ref_)
            x_ref.append(x_ref_)
            y_ref.append(y_ref_)
            theta_ref.append(theta_ref_)
            vx, vy, vth = velocity_from_discrete_points(k, 0.1, x_ref, y_ref, theta_ref)
            if (t_predict >= 11):
                vx, vy = 0.0, 0.0
            v_ref1, v_ref2, v_ref3, v_ref4 = inverse_kinematic(vx, vy, vth, theta_ref[k])
            u.append(v_ref1)
            u.append(v_ref2)
            u.append(v_ref3)
            u.append(v_ref4)
        if type=="traj2":
            x_ref_ = 0.75*t_predict
            y_ref_ = 0.0
            theta_ref_ = 0.2355
            x.append(x_ref_)
            x.append(y_ref_)
            x.append(theta_ref_)
            x_ref.append(x_ref_)
            y_ref.append(y_ref_)
            theta_ref.append(theta_ref_)
            vx, vy, vth = velocity_from_discrete_points(k, 0.1, x_ref, y_ref, theta_ref)
            if (t_predict >= 6.64):
                vx, vy = 0.0, 0.0
            v_ref1, v_ref2, v_ref3, v_ref4 = inverse_kinematic(vx, vy, vth, theta_ref[k])
            u.append(v_ref1)
            u.append(v_ref2)
            u.append(v_ref3)
            u.append(v_ref4)


    # reshaped state and control
    x = np.array(x).reshape(N+1, -1)
    u = np.array(u).reshape(N, -1)

    return x, u


# Shift timestep at each iteration
def shift_timestep(step_horizon, t0, x0, x_f, u, f):
    x0 = x0.reshape((-1,1))
    t = t0 + step_horizon
    f_value = f(x0, u[:, 0])
    st = ca.DM.full(x0 + (step_horizon) * f_value)
    u = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)
    x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)
    return t, st, x_f, u

#def reference_tracjectory(x_ref, y_ref, theta_ref, step_horizon, N):
#    states = np.zeros((3, N+1)
#    controls = np.zeros((4, N_))
#    for k in range(N):

def main():

    # State symbolic variables
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    states = ca.vertcat(x, y, theta)
    n_states = states.numel()

    # Control symbolic variables
    v1 = ca.SX.sym('v1')
    v2 = ca.SX.sym('v2')
    v3 = ca.SX.sym('v3')
    v4 = ca.SX.sym('v4')
    controls = ca.vertcat(v1, v2, v3, v4)
    n_controls = controls.numel()

    # Matrix containing all states
    X = ca.SX.sym('X', n_states, N+1)
    # Matrix containing all controls
    U = ca.SX.sym('U', n_controls, N)
    # Matrix containing all states_ref
    X_ref = ca.SX.sym('X_ref', n_states, N+1)
    # Matrix containing all controls_ref
    U_ref = ca.SX.sym('U_ref', n_controls, N)
    # State weight matrix
    Q = ca.diagcat(Q_x, Q_y, Q_theta)
    # Control weight matrix
    R = ca.diagcat(R1, R2, R3, R4)
    # Forward kinematic
    v_x = (-0.5)*(v1*ca.sin(a1+theta)+v2*ca.sin(a2+theta)+v3*ca.sin(a3+theta)+v4*ca.sin(a4+theta))
    v_y = (0.5)*(v1*ca.cos(a1+theta)+v2*ca.cos(a2+theta)+v3*ca.cos(a3+theta)+v4*ca.cos(a4+theta))
    v_theta = (v1+v2+v3+v4)/(4*0.245)
    RHS = ca.vertcat(v_x, v_y, v_theta)
    # nonlinear function for mpc model
    f = ca.Function('f', [states, controls], [RHS])
    # cost and constraint
    cost_fn = 0
    g = X[:, 0] - X_ref[:, 0]

    # Euler
    for k in range(N):
        st_err = X[:, k] - X_ref[:, k+1]
        con_err = U[:, k] - U_ref[:, k]
        cost_fn = cost_fn + st_err.T @ Q @ st_err + con_err.T @ R @ con_err
        st_next = X[:, k+1]
        f_value = f(X[:, k], U[:, k])
        st_next_euler = X[:, k] + (step_horizon*f_value)
        g = ca.vertcat(g, st_next-st_next_euler)

    optimal_var = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
    optimal_par = ca.vertcat(ca.reshape(X_ref, -1, 1), ca.reshape(U_ref, -1, 1))
    nlp_prob = {'f': cost_fn, 'x': optimal_var, 'p': optimal_par, 'g': g}
    opts = {
            'ipopt.max_iter': 2000,
            'ipopt.print_level': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6,
            'print_time': 0}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
    ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

    lbx[0: n_states*(N+1): n_states] = x_min
    lbx[1: n_states*(N+1): n_states] = y_min
    lbx[2: n_states*(N+1): n_states] = theta_min

    ubx[0: n_states*(N+1): n_states] = x_max
    ubx[1: n_states*(N+1): n_states] = y_max
    ubx[2: n_states*(N+1): n_states] = theta_max

    lbx[n_states*(N+1):] = v_min
    ubx[n_states*(N+1):] = v_max

    args = {
        'lbg': ca.DM.zeros((n_states*(N+1), 1)),
        'ubg': ca.DM.zeros((n_states*(N+1), 1)),
        'lbx': lbx,
        'ubx': ubx
    }

    # simulation
    t0 = 0
    mpciter = 0
    init_state = np.array([0.975, 6, 0.0]).reshape(1,-1)
    #current_state = init_state.copy()
    init_control = np.array([0.0, 0.0, 0.0, 0.0]).reshape(1, -1)
    state = np.tile(current_state.reshape(-1, 1), N+1).T
    control = np.tile(init_control.reshape(-1,1), N).T
    next_trajectories = state.copy()
    next_controls = control.copy()
    opt_x_x = []
    opt_x_y = []
    t = []
    vel_x, vel_y , vel_th = [], [], []
    vel_m1, vel_m2, vel_m3, vel_m4 = [], [], [], []
    th_cond = [0]
    w = np.linspace(0, 25, 100)
#ax = 3 + 3 * np.sin(2*np.pi/25*t)
#ay = np.sin(4*np.pi*t/25)
#ax = 5*np.cos(w)
#ay = 5*np.sin(w)
#ax = 3 + 3 * np.sin(2*np.pi*w/25)
#ay = 2*np.sin(4*np.pi*w/25)
    ax = [0.975, 0.975, 5.5, 5.5, 3.35, 3.35]
    ay = [6, 0.975, 0.975, 2.375+0.975, 2.375+0.975, 6]
    theta_bool = True
    #and (th_cond[-1] <= 13.80460871)
    omni_robot = OMNI_ROBOT()
    # ros rate
    rate = rospy.Rate(10) # 10Hz
    while ((mpciter  * step_horizon < sim_time)) and (rospy.is_shutdown == False):
            current_time = mpciter * step_horizon
            args['p'] = np.concatenate((
                next_trajectories.reshape(-1, 1),
                next_controls.reshape(-1, 1))
            )
            args['x0'] = np.concatenate(
                (state.reshape(-1,1),
                control.reshape(-1,1))
            )
            sol = solver(
                x0=args['x0'],
                p = args['p'],
                lbx=args['lbx'],
                ubx=args['ubx'],
                lbg=args['lbg'],
                ubg=args['ubg'],
            )
            sol_x = ca.reshape(sol['x'][:n_states*(N+1)], n_states, N+1)
            sol_u = ca.reshape(sol['x'][n_states*(N+1):], n_controls, N)
            current_state = state_omni.callback().reshape(1, -1)
            t0, current_state, state, control = shift_timestep(step_horizon, t0, current_state, sol_x, sol_u, f)
            next_trajectories, next_controls = reference_state_and_control(t0, step_horizon, current_state, N, type="traj2")
            #print(f" optimal state :x = {np.round(sol_x.full()[0, 0], 3)}, y = {np.round(sol_x.full()[1, 0], 3)}, theta = {np.round(sol_x.full()[2, 0], 3)}")
            #print(f" current state :x = {np.round(current_state[0], 3)}, y = {np.round(current_state[1], 3)}, theta = {np.round(current_state[2], 3)}")
            #print(f" next_trajectories :x = {np.round(next_trajectories[0], 3)}, y = {np.round(next_trajectories[1], 3)}, theta = {np.round(next_trajectories[2], 3)}")
            opt_x_x.append(sol_x[0, -1].full())
            opt_x_y.append(sol_x[1, -1].full())
            th_cond.append(sol_x.full()[2, -1])
            v_x, v_y, v_th = forward_kinematic(sol_u[0, -1].full(), sol_u[1, -1].full(), sol_u[2, -1].full(), sol_u[3, -1].full(), sol_x[2, -1].full())
            vel_x.append(v_x)
            vel_y.append(v_y)
            vel_th.append(v_th)
            vel_m1.append(sol_u[0, -1].full())
            vel_m2.append(sol_u[1, -1].full())
            vel_m3.append(sol_u[2, -1].full())
            vel_m4.append(sol_u[3, -1].full())
            # Publish necessary data
            robot_pose(sol_x.full()[0, 0], sol_x.full()[1, 0], 0.0, 0.0, 0.0, sol_x.full()[2, 0])
            robot_velocity(v_x, v_y, v_th)
            #next_trajectories, next_control = reference_trajectory(t0, step_horizon, init_state, N, dt=0.1)
            plt.clf()
            plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
            #plt.scatter(next_trajectories[0, :], next_trajectories[1, :], color="red")
            plt.plot(ax, ay, color="blue")
            #plt.plot([0, 3], [1, 1], color="green")
            plot_arrow(sol_x[0, -1].full(), sol_x[1, -1].full(), sol_x[2, -1].full())
            omni_robot.generate_each_wheel_and_draw(x=sol_x.full()[0, -1], y=sol_x.full()[1, -1], yaw=sol_x.full()[2, -1])
            #plt.plot(next_trajectories[0, -1:], next_trajectories[1, -1:])
            plt.plot(sol_x[0, :].full(), sol_x[1, :].full(), marker="*", color="red")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Velocity of robot [m/sec]:" + str(round(math.sqrt(v_x**2+v_y**2), 2)))
            plt.pause(0.0001)
            #print(f"x = {sol_x.full()[0, 0]}, y = {sol_x.full()[1, 0]}, theta = {sol_x.full()[2, 0]}")
            mpciter = mpciter + 1
            t.append(t0)
            rate.sleep()
            #print(t0)

    t = np.array(t).flatten()
    vel_x = np.array(vel_x).flatten()
    vel_y = np.array(vel_y).flatten()
    vel_th = np.array(vel_th).flatten()
    vel_m1 = np.array(vel_m1).flatten()
    vel_m2 = np.array(vel_m2).flatten()
    vel_m3 = np.array(vel_m3).flatten()
    vel_m4 = np.array(vel_m4).flatten()
    opt_x_x = np.array(opt_x_x).flatten()
    opt_x_y = np.array(opt_x_y).flatten()
    if show_animation == True:
        fig, axes = plt.subplots(3, 3, layout="constrained", figsize=(12, 7))
        axes[0, 0].plot(ax, ay, color="red")
        axes[0, 0].set_xlabel('x [m]')
        axes[0, 0].set_ylabel('y [m]')
        axes[0, 0].set_title("Reference trajectory")
        axes[0, 0].grid(True)
        axes[0, 1].plot(opt_x_x, opt_x_y, color="blue")
        axes[0, 1].set_xlabel('x [m]')
        axes[0, 1].set_ylabel('y [m]')
        axes[0, 1].set_title("Exact tracking")
        axes[0, 1].grid(True)
        axes[0, 2].plot(t, vel_x, color="red")
        axes[0, 2].set_xlabel('t (s)')
        axes[0, 2].set_ylabel('m/s')
        axes[0, 2].set_title("Horizontal velocity")
        axes[0, 2].grid(True)
        axes[1, 0].plot(t, vel_y, color="blue")
        axes[1, 0].set_xlabel('t (s)')
        axes[1, 0].set_ylabel('m/s')
        axes[1, 0].set_title("Vertical velocity")
        axes[1, 0].grid(True)
        axes[1, 1].plot(t, vel_th, color="green")
        axes[1, 1].set_xlabel('t (s)')
        axes[1, 1].set_ylabel('m/s')
        axes[1, 1].set_title("Angular velocity")
        axes[1, 1].grid(True)
        axes[1, 2].plot(t, vel_m1, color="green")
        axes[1, 2].set_xlabel('t (s)')
        axes[1, 2].set_ylabel('m/s')
        axes[1, 2].set_title("Velocity of motor 1")
        axes[1, 2].grid(True)
        axes[2, 0].plot(t, vel_m2, color="red")
        axes[2, 0].set_xlabel('t (s)')
        axes[2, 0].set_ylabel('m/s')
        axes[2, 0].set_title("Velocity of motor 2")
        axes[2, 0].grid(True)
        axes[2, 1].plot(t, vel_m3, color="blue")
        axes[2, 1].set_xlabel('t (s)')
        axes[2, 1].set_ylabel('m/s')
        axes[2, 1].set_title("Velocity of motor 3")
        axes[2, 1].grid(True)
        axes[2, 2].plot(t, vel_m4, color="orange")
        axes[2, 2].set_xlabel('t (s)')
        axes[2, 2].set_ylabel('m/s')
        axes[2, 2].set_title("Velocity of motor 4")
        axes[2, 2].grid(True)
        plt.show()
if __name__ == "__main__":
    rospy.init_node('MPC_OMNI', anonymous=False)
    main()