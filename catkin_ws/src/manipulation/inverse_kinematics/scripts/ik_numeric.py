#!/usr/bin/env python3
import math
import sys
import rospy
import numpy
import tf
import tf.transformations as tft
import urdf_parser_py.urdf
from std_msgs.msg import Float64MultiArray
from manip_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

prompt = ""

def get_model_info(joint_names):
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = []
    transforms = []
    for name in joint_names:
        for joint in robot_model.joints:
            if joint.name == name:
                joints.append(joint)
    for joint in joints:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms.append(tft.concatenate_matrices(T,R))
    return joints, transforms

def angles_in_joint_limits(q):
    for i in range(len(q)):
        if q[i] < joints[i].limit.lower or q[i] > joints[i].limit.upper:
            #print(prompt+"Articular position out of joint bounds")
            return False
    return True
    
def forward_kinematics(q):
    global transforms, joints
    H = tft.identity_matrix()
    for i in range(len(q)):
        H  = tft.concatenate_matrices(H, transforms[i], tft.rotation_matrix(q[i], joints[i].axis))
    H  = tft.concatenate_matrices(H, transforms[7])
    return numpy.asarray([H[0,3], H[1,3], H[2,3]] + list(tft.euler_from_matrix(H)))

def jacobian(q):
    delta_q = 0.000001   
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])##numpy, ndarray
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q)) 
    for i in range(len(q)):
        J[:,i] = (forward_kinematics(qn[i]) - forward_kinematics(qp[i]))/delta_q/2.0
     
    return J

def inverse_kinematics(x, y, z, roll, pitch, yaw, initial_guess=numpy.zeros(7), max_iterations=20):
    only_xyz = math.isnan(roll) or math.isnan(pitch) or math.isnan(yaw)
    q = [qi for qi in initial_guess]
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    p  = forward_kinematics(q)
    iterations = 0
    err = p-pd
    if only_xyz:
        err = err[0:3]
    while numpy.linalg.norm(err) > 0.00001 and iterations < max_iterations:
        J = jacobian(q)
        err = p - pd
        if only_xyz:
            J = J[0:3]
            err = err[0:3]
        else:
            err[3:6] = (err[3:6] + math.pi)%(2*math.pi) - math.pi
        q = (q - numpy.dot(numpy.linalg.pinv(J), err) + math.pi)%(2*math.pi) - math.pi
        p = forward_kinematics(q)
        err = p - pd
        if only_xyz:
            err = err[0:3]
        iterations +=1
    return iterations < max_iterations and angles_in_joint_limits(q), q

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
    
def get_polynomial_trajectory_multi_dof(Q_start, Q_end, Qp_start=[], Qp_end=[], Qpp_start=[], Qpp_end=[], duration=1.0, time_step=0.05):
    Q = []
    T = []
    if Qp_start == []:
        Qp_start = numpy.zeros(len(Q_start))
    if Qpp_start == []:
        Qpp_start = numpy.zeros(len(Q_start))
    if Qp_end == []:
        Qp_end = numpy.zeros(len(Q_end))
    if Qpp_end == []:
        Qpp_end = numpy.zeros(len(Q_end))
    for i in range(len(Q_start)):
        T, Qi = get_polynomial_trajectory(Q_start[i], Q_end[i], Qp_start[i], Qp_end[i], Qpp_start[i], Qpp_end[i], duration, time_step)
        Q.append(Qi)
    Q = numpy.asarray(Q)
    Q = Q.transpose()
    return Q,T

def callback_forward_kinematics(req):
    if len(req.q) != 7:
        print(prompt+"By the moment, only 7-DOF arm is supported")
        return False
    resp = ForwardKinematicsResponse()
    resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw = forward_kinematics(req.q)
    return resp

def get_trajectory_time(p1, p2, speed_factor):
    p1 = numpy.asarray(p1)
    p2 = numpy.asarray(p2)
    m = max(numpy.absolute(p1 - p2))
    return m/speed_factor + 0.5

def callback_ik_for_trajectory(req):
    global max_iterations
    print(prompt+"Calculating inverse kinematics and trajectory for " + str([req.x, req.y, req.z, req.roll, req.pitch, req.yaw]))
    if len(req.initial_guess) <= 0 or req.initial_guess == None:
        initial_guess = rospy.wait_for_message("/hardware/arm/current_pose", Float64MultiArray)
        initial_guess = initial_guess.data
    else:
        initial_guess = req.initial_guess
    p1 = forward_kinematics(initial_guess)
    p2 = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    t  = req.duration if req.duration > 0 else get_trajectory_time(p1, p2, 0.25)
    dt = req.time_step if req.time_step > 0 else 0.05
    X,T = get_polynomial_trajectory_multi_dof(p1, p2, duration=t, time_step=dt)
    trj = JointTrajectory()
    trj.header.stamp = rospy.Time.now()
    q = initial_guess
    for i in range(len(X)):
        x, y, z, roll, pitch, yaw = X[i]
        success, q = inverse_kinematics(x, y, z, roll, pitch, yaw, q, max_iterations)
        if not success:
            return False
        p = JointTrajectoryPoint()
        p.positions = q
        p.time_from_start = rospy.Duration.from_sec(T[i])
        trj.points.append(p)
    resp = InverseKinematicsPose2TrajResponse()
    resp.articular_trajectory = trj
    return resp
    
        
def callback_ik_for_pose(req):
    global max_iterations
    print(prompt+"Calculating inverse kinematics for pose: " + str([req.x, req.y, req.z, req.roll, req.pitch, req.yaw]))
    if len(req.initial_guess) <= 0 or req.initial_guess == None:
        initial_guess = rospy.wait_for_message("/hardware/arm/current_pose", Float64MultiArray, 5.0)
        initial_guess = initial_guess.data
    else:
        initial_guess = req.initial_guess
    resp = InverseKinematicsPose2PoseResponse()
    success, q = inverse_kinematics(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, initial_guess, max_iterations)
    if not success:
        return False
    resp.q = q
    return resp        
    

def main():
    global joint_names, max_iterations, joints, transforms, prompt
    print("INITIALIZING INVERSE KINEMATIC NODE BY MARCOSOFT...")
    rospy.init_node("ik_geometric")
    prompt = rospy.get_name().upper() + ".->"
    joint_names    = rospy.get_param("~joint_names", [])
    max_iterations = rospy.get_param("~max_iterations", 20)
    print(prompt+"Joint names: " + str(joint_names))
    print(prompt+"max_iterations: " + str(max_iterations))

    joints, transforms = get_model_info(joint_names)
    if not (len(joints) > 6 and len(transforms) > 6):
        print("Inverse kinematics.->Cannot get model info from parameter server")
        sys.exit(-1)

    rospy.Service("/manipulation/forward_kinematics"   , ForwardKinematics, callback_forward_kinematics)    
    rospy.Service("/manipulation/ik_trajectory"        , InverseKinematicsPose2Traj, callback_ik_for_trajectory)
    rospy.Service("/manipulation/ik_pose"              , InverseKinematicsPose2Pose, callback_ik_for_pose)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()


