#!/usr/bin/env python3

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import math
import rospy
from time import time
from geometry_msgs.msg import Twist, Pose

a1 = math.pi/4
a2 = 3*math.pi/4
a3 = 5*math.pi/4
a4 = 7*math.pi/4
step_horizon = 0.1
N = 100
rob_diam = 0.3
sim_time = 200

x_goal = [1, 3, 5, 6]
y_goal = [1, 3, 5, 6]
theta_goal = [0, 0, 0, ca.pi/3]

def goal(x_target, y_target, theta_target):
    x = x_target
    y = y_target
    theta = theta_target
    return x, y, theta

for (i, j, k) in zip(x_goal, y_goal, theta_goal):
    x_target = i
    y_target = j
    theta_target = k

R = 0.52
Q_x = 1
Q_y = 1
Q_theta = 1
R1 = 1
R2 = 1
R3 = 1
R4 = 0.5

def forward_kinematic(v1, v2, v3, v4, theta):
    x = (-0.5)*(v1*math.sin(theta+a1)+v2*math.sin(theta+a2)+v3*math.sin(theta+a3)+v4*math.sin(theta+a4))
    y = (0.5)*(v1*math.cos(theta+a1)+v2*math.cos(theta+a2)+v3*math.cos(theta+a3)+v4*math.cos(theta+a4))
    yaw = (v1+v2+v3+v4)*(4*0.52)

    return x, y, yaw

def shift_timestep(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:,0])
    next_state = ca.DM.full(state_init + (step_horizon*f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:,-1], -1, 1)
    )
    return t0, next_state, u0

def DM2Arr(x):
    return np.array(x.full())

# state symbolic variables
x = ca.MX.sym('x')
y = ca.MX.sym('y')
theta = ca.MX.sym('theta')
states = ca.vertcat(
    x,
    y,
    theta
)
n_states = states.numel()

# control symbolic variables
v1 = ca.MX.sym('v1')
v2 = ca.MX.sym('v2')
v3 = ca.MX.sym('v3')
v4 = ca.MX.sym('v4')
controls = ca.vertcat(
    v1,
    v2,
    v3,
    v4
)
n_controls = controls.numel()

# Matrix containing all states over all time step + 1
X = ca.MX.sym('X', n_states, N+1)

# Matrix containing all control actions over all time step
U = ca.MX.sym('U', n_controls, N)

# column vector for storing initial states and target state
P = ca.MX.sym('P', n_states + n_states)

# state weights matrix Q
Q = ca.diagcat(Q_x, Q_y, Q_theta)

# control weights matrix R
R = ca.diagcat(R1, R2, R3, R4)

# discretization model
##rot_3d_matrix = ca.vertcat(
##    ca.horzcat(-math.sin(theta+a1), -math.sin(theta+a2), -math.sin(theta+a3), -math.sin(theta+a4)),
##    ca.horzcat(math.cos(theta+a1), math.cos(theta+a2), math.cos(theta+a3), math.cos(theta+a4)),
##    ca.horzcat(1/(2*0.52), 1/(2*0.52), 1/(2*0.52), 1/(2*0.52))
##)

##J = ca.DM([
##    [-1, -1, -1, -1],
##    [1, 1, 1, 1],
##    [1, 1, 1, 1]
##])

vel_x = -(0.5)*(v1*ca.sin(theta+a1)+v2*ca.sin(theta+a2)+v3*ca.sin(theta+a3)+v4*ca.sin(theta+a4))
vel_y = (0.5)*(v1*ca.cos(theta+a1)+v2*ca.cos(theta+a2)+v3*ca.cos(theta+a3)+v4*ca.cos(theta+a4))
vel_theta = (1/(4*0.52))*(v1+v2+v3+v4)
RHS = ca.vertcat(
    vel_x,
    vel_y,
    vel_theta
)

# mapped function control
f = ca.Function('f', [states, controls], [RHS])

cost_fn = 0
g = X[:, 0] - P[:n_states]

# runge kutta
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    cost_fn = cost_fn + (st-P[n_states:]).T @ Q @(st-P[n_states:]) + con.T @ R @ con
    st_next = X[:, k+1]
    k1 = f(st, con)
    k2 = f(st + step_horizon/2*k1, con)
    k3 = f(st + step_horizon/2*k2, con)
    k4 = f(st + step_horizon * k3, con)
    st_next_RK4 = st + (step_horizon / 6) * (k1+k2+k3+k4)
    g = ca.vertcat(g, st_next - st_next_RK4)

OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),
    U.reshape((-1, 1))
)

nlp_prob = {
    'f': cost_fn,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

lbx[0: n_states*(N+1): n_states] = -ca.inf
lbx[1: n_states*(N+1): n_states] = -ca.inf
lbx[2: n_states*(N+1): n_states] = -ca.inf

ubx[0: n_states*(N+1): n_states] = ca.inf
ubx[1: n_states*(N+1): n_states] = ca.inf
ubx[2: n_states*(N+1): n_states] = ca.inf

lbx[n_states*(N+1):] = -1
ubx[n_states*(N+1):] = 1

args = {
    'lbg': ca.DM.zeros((n_states*(N+1), 1)),
    'ubg': ca.DM.zeros((n_states*(N+1), 1)),
    'lbx': lbx,
    'ubx': ubx
}

def mpc_run(x_init, y_init, theta_init, x_target, y_target, theta_target):
    t0 = 0
    state_init = ca.DM([x_init, y_init, theta_init])
    state_target = ca.DM([x_target, y_target, theta_target])

    t = ca.DM(t0)

    u0 = ca.DM.zeros((n_controls, N))
    X0 = ca.repmat(state_init, 1, N+1)
    times = np.array([[0]])

    mpciter = 0
    cat_states = DM2Arr(X0)
    cat_controls = DM2Arr(u0[:,0])

    main_loop = time()  # return time in sec
    while (ca.norm_2(state_init - state_target) > 1e-1) and (mpciter * step_horizon < sim_time):
        t1 = time()
        args['p'] = ca.vertcat(
            state_init,    # current state
            state_target   # target state
        )

        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )

        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))

        cat_controls = np.vstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))

        t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)

        # print(X0)
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time()
        print(mpciter)
        print(t2-t1)
        times = np.vstack((
            times,
            t2-t1
        ))
        ##print(f"Optimal states{X0[0 , mpciter]}")
        ##print(f"Optimal controls{u[:, mpciter]}")
        vel_x, vel_y, vel_theta = forward_kinematic(u[0, mpciter], u[1, mpciter], u[2, mpciter], u[3, mpciter], X0[2, mpciter])
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = vel_theta
        vel_pub.publish(twist)
        pose.position.x = X0[0, mpciter]
        pose.position.y = X0[1, mpciter]
        pose.position.z = 0.0
        pos_pub.publish(pose)
        ##print("V_x", vel_x)
        mpciter = mpciter + 1

    main_loop_time = time()
    ss_error = ca.norm_2(state_init - state_target)

    print('\n\n')
    ##print("optimal states", cat_states)
    ##print("optimal contros", cat_controls)
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)
    simulate(cat_states, cat_controls, times, step_horizon, N,
             np.array([x_init, y_init, theta_init, x_target, y_target, theta_target]), save=False)


if __name__=="__main__":
    rospy.init_node("omni_publisher", anonymous=True)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    pos_pub = rospy.Publisher("robot_pose", Pose, queue_size=10)
    twist = Twist()
    pose = Pose()
    x_init = 0
    y_init = 0
    theta_init = 0
    x_target = 0
    y_target = 0
    theta_target = 0
    for (i, j, k) in zip(x_goal, y_goal, theta_goal):
        x_init = i
        y_init = j
        theta_init = k
        x_target = x_target + i
        y_target = y_target + j
        theta_target = theta_target + k
        mpc_run(x_init, y_init, theta_init, x_target, y_target, theta_target)