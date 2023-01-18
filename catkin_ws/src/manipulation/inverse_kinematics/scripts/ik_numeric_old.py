#!/usr/bin/env python
import math
import string
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from manip_msgs.srv import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

global joint_traj_msg

def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = {'left': [None for i in range(8)], 'right': [None for i in range(8)]}
    transforms = {'left':[], 'right':[]}
    for joint in robot_model.joints:
        for i in range(1,8):
            joints['left' ][i-1] = joint if joint.name == ('la_'+str(i)+'_joint') else joints['left' ][i-1]
            joints['right'][i-1] = joint if joint.name == ('ra_'+str(i)+'_joint') else joints['right'][i-1]
        joints['left' ][7] = joint if joint.name == 'la_grip_center_joint' else joints['left' ][7]
        joints['right'][7] = joint if joint.name == 'ra_grip_center_joint' else joints['right'][7]
    for joint in joints['left']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['left'].append(tft.concatenate_matrices(T,R))
    for joint in joints['right']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['right'].append(tft.concatenate_matrices(T,R))

def angles_in_joint_limits(q, arm):
    for i in range(len(q)):
        if q[i] < joints[arm][i].limit.lower or q[i] > joints[arm][i].limit.upper:
            #print("InverseKinematics.->Articular position out of joint bounds")
            return False
    return True
    
def forward_kinematics(q, arm):
    global transforms, joints
    H = tft.identity_matrix()
    for i in range(len(q)):
        H  = tft.concatenate_matrices(H, transforms[arm][i], tft.rotation_matrix(q[i], joints[arm][i].axis))
    H  = tft.concatenate_matrices(H, transforms[arm][7])
    return numpy.asarray([H[0,3], H[1,3], H[2,3]] + list(tft.euler_from_matrix(H)))

def jacobian(q, arm):
    delta_q = 0.000001   
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])##numpy, ndarray
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q)) 
    for i in range(len(q)):
        J[:,i] = (forward_kinematics(qn[i], arm) - forward_kinematics(qp[i], arm))/delta_q/2.0
     
    return J

def jacobian_3x7(q, arm):   
    delta_q = 0.000001   
    J = numpy.asarray([[0.0 for a in q] for i in range(3)])   # 3x7
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))  # 7x7
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q))  # 7x7
    for i in range(len(q)):
        x, y,z, R, P, Y = (forward_kinematics(qn[i], arm) - forward_kinematics(qp[i], arm))/delta_q/2.0
        J[:,i] = x, y, z
    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, arm,q = numpy.asarray([-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3])):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    p  = forward_kinematics(q, arm)
    iterations = 0
    q = [qi for qi in q]
    err = p-pd
    while numpy.linalg.norm(err) > 0.00001 and iterations < 20:
        J = jacobian(q, arm)
        err = p - pd
        err[3:6] = (err[3:6] + math.pi)%(2*math.pi) - math.pi
        q = (q - numpy.dot(numpy.linalg.pinv(J), err) + math.pi)%(2*math.pi) - math.pi
        p = forward_kinematics(q, arm)
        err = p - pd
        iterations +=1
        
    if iterations < 20 and angles_in_joint_limits(q, arm):
        #print("InverseKinematics.->IK for " + arm + " arm solved after " + str(iterations) + " iterations: " + str(q))
        return q
    else:
        #print("InverseKinematics.->Cannot solve IK for " + arm + " arm. Max attempts exceeded. ")
        return False



def inverse_kinematics_xyz(x, y, z, arm, q):
    pd = numpy.asarray([x,y,z])
    p  = forward_kinematics(q, arm)
    iterations = 0
    q_J3x7 = [qi for qi in q]
    #print("Estimacion inicial: " + str(q_J3x7))
    err_xyz = p[0:3]-pd[0:3]
    while numpy.linalg.norm(err_xyz) > 0.00001 and iterations < 20:
        J_3x7 = jacobian_3x7(q, arm)
        err_xyz = p[0:3]-pd[0:3]
        q_J3x7 = q_J3x7 - numpy.dot(numpy.linalg.pinv(J_3x7), err_xyz)
        p = forward_kinematics(q_J3x7, arm)
        err_xyz = p[0:3]-pd[0:3]
        iterations +=1
        
    if iterations < 20 and angles_in_joint_limits(q_J3x7, arm):
        #print("InverseKinematics.->IK for " + arm + " arm solved after " + str(iterations) + " iterations: " + str(q_J3x7))
        return q_J3x7
    else:
        #print("InverseKinematics.->Cannot solve IK for " + arm + " arm. Max attempts exceeded. ")
        return False

def get_polynomial_trajectory(q_initial, q_final, qp_initial=0, qp_final=0, qpp_initial=0, qpp_final=0, duration=1.0, time_step=0.05):
    #
    # This function calculates a polynomial trajectory for a single variable.
    # It is intended to be called N times for a N-DoF trajectory. 
    #
    t = duration
    A = [[   t**5,    t**4,   t**3, t**2, t, 1],
         [ 5*t**4,  4*t**3, 3*t**2,  2*t, 1, 0],
         [20*t**3, 12*t**2,    6*t,    2, 0, 0],
         [      0,       0,      0,    0, 0, 1],
         [      0,       0,      0,    0, 1, 0],
         [      0,       0,      0,    2, 0, 0]]
    A = numpy.asarray(A)
    B = [[q_final  ],
         [qp_final ],
         [qpp_final],
         [q_initial  ],
         [qp_initial ],
         [qpp_initial]]
    B = numpy.asarray(B)
    X = numpy.dot(numpy.linalg.inv(A),B)
    a5, a4, a3, a2, a1, a0 = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0], X[5,0]
    T = numpy.arange(0, t, time_step)
    Q = numpy.zeros(len(T))
    for i in range(len(T)):
        Q[i] = a5*T[i]**5 + a4*T[i]**4 + a3*T[i]**3 + a2*T[i]**2 + a1*T[i] + a0

    return T, Q
    
def get_articular_trajectory(Q_initial, Q_final, t, dt):
    Q = []
    T = []
    for i in rante(len(Q_initial)):
        T, Qi = get_polynomial_trajectory(Q_initial[i], Q_final[i], duration=t, time_step=dt)
        Q.append(Qi)
    Q = numpy.asarray(Q)
    Q = Q.transpose()
    

def calcula_tray(tt, pi, pf, vi, vf, ai, af):  # Calculate 5th order polynomial for one variable
    tm = 0.1
    #print("sampling time tm: ",tm)
    for p in range(0,len(tt)-1):
        # Time range in which the polynomial is evaluated
        t=numpy.arange(tt[p],tt[p+1],tm)
        c = numpy.ones(len(t))

        M = numpy.array([[1, tt[p], tt[p]**2, tt[p]**3, tt[p]**4, tt[p]**5],
            [0, 1, 2*tt[p], 3*tt[p]**2, 4*tt[p]**3, 5*tt[p]**4],
            [0, 0, 2, 6*tt[p], 12*tt[p]**2, 20*tt[p]**3],
            [1, tt[p+1], tt[p+1]**2, tt[p+1]**3, tt[p+1]**4, tt[p+1]**5],
            [0, 1, 2*tt[p+1], 3*tt[p+1]**2, 4*tt[p+1]**3, 5*tt[p+1]**4],
            [0, 0, 2, 6*tt[p+1], 12*tt[p+1]**2, 20*tt[p+1]**3]])
        # Vector of initial conditions
        b=[pi, vi, ai, pf, vf, af]
        b = numpy.vstack(b)
        # Solution to the matrix equation
        Minv = numpy.linalg.inv(M)
        a = numpy.dot(Minv,b) 
        # 5th degree polynomial
        xpos = a[0][0]*c + a[1][0]*t +a[2][0]*t**2 + a[3][0]*t**3 + a[4][0]*t**4 + a[5][0]*t**5
        xvel = a[1][0]*c +2*a[2][0]*t +3*a[3][0]*t**2 +4*a[4][0]*t**3 +5*a[5][0]*t**4
        xacel = 2*a[2][0]*c + 6*a[3][0]*t +12*a[4][0]*t**2 +20*a[5][0]*t**3
    
    #print("number of points", len(xpos))

    return [t, xpos, xvel, xacel]

    
def cartesian_traj(tt, pi, pf, vi, vf, ai, af):
    p6, v6, a6, t = numpy.empty(0), numpy.empty(0), numpy.empty(0), numpy.empty(0)
 
    for i in range(6): 
        t, p, v, a = calcula_tray(tt, pi[i], pf[i], vi[i], vf[i], ai[i], af[i])
        p6 ,v6,a6 = numpy.append(p6, [p]), numpy.append(v6, [v]), numpy.append(a6, [a])

    p6, v6 , a6 = numpy.reshape(p6, (6, len(p))), numpy.reshape(v6, (6, len(v))), numpy.reshape(a6, (6, len(a)))
    traj = JointTrajectory()   
    traj.joint_names = ["x","y","z","row","pitch","yaw"]
    i = 0
    tfs = 0
    for i in range(len(p)):    # For each point of the trajectory
        point = JointTrajectoryPoint() 
        point.positions = p6[0,i], p6[1,i],p6[2,i], (p6[3,i]), (p6[4,i]), (p6[5,i])
        #point.velocities = 0,0,0,0,0,0#v6[0,i],v6[1,i],v6[2,i],v6[3,i],v6[4,i],v6[5,i]
        #point.accelerations = 0,0,0,0,0,0#a6[0,i],a6[1,i],a6[2,i],a6[3,i],a6[4,i],a6[5,i]
        point.time_from_start.secs = tfs
        traj.points.append(point)
        tfs += tm

    return traj

def callback_trajectory_3d(req):  
    angles1 = tft.euler_from_quaternion([req.p1.orientation.x , req.p1.orientation.y ,req.p1.orientation.z, req.p1.orientation.w])
    angles2 = tft.euler_from_quaternion([req.p2.orientation.x , req.p2.orientation.y ,req.p2.orientation.z, req.p2.orientation.w])
    ang1, ang2 = list(angles1), list(angles2)
    t = req.t
    tt = numpy.array([0,t])
    pi = [req.p1.position.x, req.p1.position.y, req.p1.position.z, ang1[0], ang1[1], ang1[2]]
    pf = [req.p2.position.x, req.p2.position.y, req.p2.position.z, ang2[0], ang2[1], ang2[2]]
    vi, vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai, af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    c_tr = cartesian_traj(tt, pi, pf, vi, vf, ai, af)
    resp = GetCartesianTrajectoryResponse()
    resp.trajectory = c_tr
	
    callback_trajectory_q( c_tr, 'left')
    

    return resp    # Returns a JointTrajectory object

def callback_trajectory_q(req, arm, q_estim, jacob):  # Trajectory in joint space: receives a JointTrajectory
    qs = numpy.empty(0)
    # Form the first estimate with current position
    n_p = len(req.points)   # Number of points on the path
    #print("numero de puntos traj*****", n_p)
    # Form the successive guesses with the point before the target
    i = 0
    for i in range(n_p):
        if jacob is 3:
            q_obt = inverse_kinematics_xyz(req.points[i].positions[0], req.points[i].positions[1], req.points[i].positions[2], arm,q_estim)
            qs = numpy.append(qs, [q_obt]) 
            q_estim = q_obt  # Update the estimate
        else:
            q_obt = inverse_kinematics_xyzrpy(req.points[i].positions[0], req.points[i].positions[1], req.points[i].positions[2], req.points[i].positions[3], req.points[i].positions[4], req.points[i].positions[5], arm,q_estim)
            qs = numpy.append(qs, [q_obt]) 
            q_estim = q_obt  # Update the estimate

    
    qs = numpy.reshape(qs, (n_p,7)) # Resize the array
    traj_q = JointTrajectory()    #trajectory with points in joint space
    traj_q.joint_names = ["q1","q2","q3","q4","q5","q6","q7"]
    i=0
    tfs = 0
  
    for element in qs:
        point = JointTrajectoryPoint()  # We create an object that stores the data of 1 point
        point.positions = element[0], element[1], element[2], element[3], element[4], element[5], element[6]
        traj_q.points.append(point)
        point.time_from_start = rospy.Duration(tfs)
        tfs += tm
        
    return traj_q


def callback_LA_ik_for_trajectory(req):
    #req.roll, req.pitch, req.yaw = numpy.nan, numpy.nan, numpy.nan
    print("Trying to calculate inverse kinematics for " + str([req.x, req.y, req.z, req.roll, req.pitch, req.yaw]))
    initial_guess = rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray, 5.0)
    initial_guess = initial_guess.data
    tt = numpy.array([0,t])
    pi = forward_kinematics(initial_guess, 'left')
    pf = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    vi, vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai, af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    c_tr = cartesian_traj(tt, pi, pf, vi, vf, ai, af)
    if req.roll  is numpy.nan or req.pitch is numpy.nan or req.yaw is numpy.nan:
        traj_q = callback_trajectory_q(c_tr, 'left',initial_guess, 3)
    else:
        traj_q = callback_trajectory_q(c_tr, 'left',initial_guess, 6)
    resp = InverseKinematicsResponse()
    resp = traj_q
    joint_traj_msg = JointTrajectory()
    joint_traj_msg = traj_q
    return resp
    


def callback_RA_ik_for_trajectory(req):

    init_estim = rospy.wait_for_message("/hardware/right_arm/current_pose", Float64MultiArray, 0.5)
    init_estim = init_estim.data
    tt = numpy.array([0,t])
    pi = forward_kinematics(init_estim, 'right')
    pf = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    vi, vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai, af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    c_tr = cartesian_traj(tt, pi, pf, vi, vf, ai, af)
    if req.roll  is numpy.nan or req.pitch is numpy.nan or req.yaw is numpy.nan:
        traj_q = callback_trajectory_q(c_tr, 'right',intial_guess, 3)
    else:
        traj_q = callback_trajectory_q(c_tr, 'right',intial_guess, 6)
    resp = InverseKinematicsResponse()
    resp = traj_q
    joint_traj_msg = JointTrajectory()
    joint_traj_msg = traj_q
    return resp

def callback_la_ik_single_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, 'left')
    if q is None:
        return None
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_ra_ik_single_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, 'right')
    if q is None:
        return False
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_la_fk(req):
    x = forward_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], 'left')
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def callback_ra_fk(req):
    x = forward_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], 'left')
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("INITIALIZING INVERSE KINEMATIC NODE BY MARCOSOFT...")
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/la_forward_kinematics", ForwardKinematics, callback_la_fk)
    rospy.Service("/manipulation/ra_forward_kinematics", ForwardKinematics, callback_ra_fk)
    # Service that generates trajectory in Cartesian space
    rospy.Service("/manipulation/cartesian_traj", GetCartesianTrajectory, callback_trajectory_3d)
    # Service that generates a cartesian trajectory and solves the IK for each point
    rospy.Service("/manipulation/la_inverse_kinematics", InverseKinematics, callback_LA_ik_for_trajectory)
    rospy.Service("/manipulation/ra_inverse_kinematics", InverseKinematics, callback_RA_ik_for_trajectory)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()


