#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
import tf2_ros
import tf
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, Vector3, Vector3Stamped
import ros_numpy
from vision_msgs.srv import *
from manip_msgs.srv import *
from std_msgs.msg import String
import geometry_msgs.msg


"""
    Node that receives the position and orientation of an object in the base_link coordinate system, 
    in cartesian coordinates, and returns the orientation that the gripper must have to pick up the object, 
    in Cartesian coordinates.

     Reglas para la construccion del sistema coordenado del gripper(frame 'base_link') :

        1: Se toma como eje x a la componente principal resultante de PCA
        2. Se define la ecuacion parametrica de un circulo en en plano Z = Z_centroide cuyo radio es epsilon
            x = epsilon * np.cos(theta)
            y = epsilon * np.sin(theta)
            z = z_centroid 

        3. Se define tamanio de paso para la generacion de puntos de candidatos de agarre y se almacenan en una lista.
        4. Para cada punto se genera un sistema coordenado y se almacena en una lista:
            eje x = eje x del objeto trasladado al punto.
            eje z = vector formado por el punto de centroide y punto, en direccion externa a la circunferencia.
            eje y = producto cruz entre eje x y eje z.

        5. Se verifican las orientaciones de forma grafica.
        
        6. Se prueba la validez de los candidatos de agarre enviandolos a la cinematica inversa, en el frame shoulders. 
            Los puntos validos se almacenan en una nueva lista.
            
        

        7. Se elije el mejor candidato de agarre conforme a un criterio y se envia para la pose de gripper.

"""


def box(obj_pose):
    print("take a box")
    



def cube_or_sphere(obj_pose):
    print("take cube or sphere")




def cylinder_or_prism(obj_pose):
    print("take cylinder or prism")
   
    q1, q2, q3, q4 = obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w
    R, P, Y =tft.euler_from_quaternion([ q1, q2, q3, q4])

    R = tft.quaternion_matrix([q1, q2, q3, q4])
    #print("R obj")
    #print(R)
    #Matriz de rotacion deseada para el gripper



    Rd = np.zeros((4,4)) 
    Rd[:,0] = -1*R[:,1]   
    Rd[:,3] = R[:,3]


    #print("Rgd")
    #print(Rd)

    rd,pd,yd = tft.euler_from_matrix(R)
    #print("Orientacion deseada ... eje X gripper")
    #print(rd, pd, yd)
    



def pose_actual_to_pose_target(pose, f_actual, f_target):
    global pose_in, listener
    #q1, q2, q3, q4 = tft.quaternion_from_euler(R, P, Y) # conversion a quaternion

    # Empaqueta msg, convierte orientacion de frame 'realsense_link' a frame 'base_link'
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = f_actual   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time(0)  # la ultima transformacion

    poseStamped_msg.pose = pose
    new_poseStamped = listener.transformPose(f_target, poseStamped_msg)
    new_pose = new_poseStamped.pose

    return new_pose





def angle_pca_floor(obj_pose):
    MT = tft.quaternion_matrix([ obj_pose.orientation.x , obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w])
    R,P,Y = tft.euler_from_quaternion([obj_pose.orientation.x , obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w])
    pc1 = np.asarray( [MT[0,1], MT[1,1], MT[2,1]])
    # componente principal esta en eje x
    if pc1[0] < 0:
        pc1[0] = pc1[0]*-1

    eje_z = np.asarray([0, 0, 1], dtype=np.float64 )# vector normal al plano xy
    # angulo entre pc1 y plano xy frame base_link
    ang_pc1_pixy = np.arcsin( np.dot(pc1 , eje_z) / (np.linalg.norm(pc1) * 1) )
    print("angulo pc1", np.rad2deg( ang_pc1_pixy ) )

    return ang_pc1_pixy, pc1 # en radianes





def grip_rules(obj_pose, type_obj):
    """
    Esta funcion calcula la pose adecuada de la pinza para tomar el objeto,
    recibe como argumento la pose del objeto en el frame base_link 
    """
    if type_obj == 1:
        print("cube_or_sphere")

    if type_obj == 2:
        print("box")

    if type_obj ==3:
        print("cylinder_or_prism")
    print("type object", type_obj)

    
    y_object = 0.03 # ancho del objeto
    epsilon = y_object # radio de la circunferencia
    coord_centroid = np.asarray([obj_pose[0] , obj_pose[1], obj_pose[2]]) # origen de la circunferencia
    x_centroid = obj_pose[0] 
    y_centroid = obj_pose[1]
    z_centroid = obj_pose[2]

    grasp_candidates_quaternion = []
    grasp_candidates_rpy = []
    
    # Step 1:
    MT = tft.euler_matrix( obj_pose[3] , obj_pose[4], obj_pose[5]) 

    axis_x_obj = np.asarray( [MT[0,0], MT[1,0], MT[2,0]])   # eje principal

    # Step 2: rango (y,-y), (0,x)
    # Step 3:

    #offset_theta = np.deg2rad(120)
    offset_theta = np.deg2rad(0)    # donde inicia la generacion de agarres candidatos
    theta = 0 + offset_theta
    step_size = np.deg2rad(10)  
    #range_points = np.pi    
    range_points = np.pi *2           # rango dentro del cual se generan los candidatos 360 grados
    num_points = int(range_points / step_size)
    print("number points", num_points)
    temp = 0
    
    
    for i in range( num_points + 1):     # (-pi/2, pi/2)
        
        # obtencion de origen de frame candidato
        point = np.asarray([ epsilon*np.cos(theta) + x_centroid , epsilon*np.sin(theta) + y_centroid , z_centroid ])
        """
        # obtencion de eje x
        axis_x_point = axis_x_obj / np.linalg.norm( (axis_x_obj) )
        # Step 4: obtencion de eje z
        axis_z_point = (point - coord_centroid ) / np.linalg.norm( (point - coord_centroid ) )
        # obtencion de eje y
        axis_y_point = np.cross(axis_z_point , axis_x_point) / np.linalg.norm( np.cross(axis_z_point , axis_x_point) )
        
        # los cuaterniones necesarios para generar el frame del objeto
        comb = [[axis_x_point, axis_y_point, axis_z_point],[ axis_y_point , axis_x_point , axis_z_point],
                [ axis_z_point , axis_y_point , axis_x_point ],[ axis_x_point , axis_z_point , axis_y_point ],
                [ axis_z_point , axis_x_point , axis_y_point ]]
        
        # 1
        RM = np.asarray( comb[0] )
        RM = RM.T
        TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
            [RM[1,0], RM[1,1] , RM[1,2], 0],
            [RM[2,0], RM[2,1] , RM[2,2], 0], 
            [0, 0, 0, 1]]

        r,p,y = tft.euler_from_matrix(np.asarray(TM))
        """
        # Se extrae la pose original***************************************************
        R = obj_pose[3] + temp#np.deg2rad(180) + temp
        P = obj_pose[4]
        Y = obj_pose[5]

        q_gripper = tft.quaternion_from_euler( R , P , Y )
        
        #same_tf = tft.is_same_transform(np.asarray(TM), np.asarray( tft.quaternion_matrix(q_gripper)))
        #print("same tf?", same_tf )
        
        # lista de poses para la cinematica inversa
        candidate_rpy = np.asarray([point[0], point[1] , point[2]  ,R , P , Y])
        # lista de poses para graficos
        candidate_grasp = Pose()
        d = np.sqrt(q_gripper[0]**2 + q_gripper[1]**2 + q_gripper[2]**2 + q_gripper[3]**2)
        candidate_grasp.position.x = point[0] 
        candidate_grasp.position.y = point[1] 
        candidate_grasp.position.z = point[2] 
        candidate_grasp.orientation.x = q_gripper[0]/d
        candidate_grasp.orientation.y = q_gripper[1]/d
        candidate_grasp.orientation.z = q_gripper[2]/d
        candidate_grasp.orientation.w = q_gripper[3]/d
        grasp_candidates_quaternion.append(candidate_grasp)
        grasp_candidates_rpy.append( candidate_rpy )

        theta = theta + step_size
        temp = temp + step_size

    return grasp_candidates_quaternion



def broadcaster_frame_object(frame, child_frame, pose):
    """
        Step 5:
    """
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    
    br.sendTransform(t)




def callback(req):
    global pose_in, listener, ik_srv
    resp = InverseKinematicsPose2TrajResponse()
    cartesian_pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    type_obj = req.initial_guess
    # retorna el ang entre piso y pc1
    #ang_pc1_pixy, pc1 = angle_pca_floor(obj_pose_in_base_link)  
    pose_list_q = grip_rules(cartesian_pose , type_obj) # solo para cilindro 
    # Step 5:
    

    offset = 0
    broadcaster_frame_object('base_link', 'candidate1' , pose_list_q[0] )
    broadcaster_frame_object('base_link', 'candidate2' , pose_list_q[1] )
    broadcaster_frame_object('base_link', 'candidate3' , pose_list_q[17] )
    #broadcaster_frame_object('base_link', 'candidate4' , pose_list_q[2+offset] )
    #broadcaster_frame_object('base_link', 'candidate5' , pose_list_q[3+offset] )
    #broadcaster_frame_object('base_link', 'candidate6' , pose_list_q[4+offset] )
    

    # Step 6:
    new_pose_q_list = []
    new_pose_rpy_list = []
    i = 0
    for pos in pose_list_q:
        new_pose = pose_actual_to_pose_target(pos, 'base_link' , 'shoulders_left_link')
        new_pose_q_list.append(new_pose)

        x , y, z = new_pose.position.x , new_pose.position.y , new_pose.position.z
        roll,pitch,yaw = tft.euler_from_quaternion( [new_pose.orientation.x , new_pose.orientation.y , 
                                          new_pose.orientation.z , new_pose.orientation.w ])
        
        pose_shoulder_frame = np.asarray([x ,y ,z , roll, pitch , yaw])
        new_pose_rpy_list.append(pose_shoulder_frame)
        
    # Prepara msg request para el servicio /manipulation/ik_trajectory
    ik_msg = InverseKinematicsPose2TrajRequest()
    # Rellenar msg pose to pose
    successful_candidate_trajectories = []
    successful_candidate_pose_list = []
    indx_list = []
    print("Evaluating the possibility of grip given the position of the object...")

    for pose in new_pose_rpy_list:  
        ik_msg.x = pose[0]
        ik_msg.y = pose[1]
        ik_msg.z = pose[2]
        ik_msg.roll = pose[3]
        ik_msg.pitch = pose[4]
        ik_msg.yaw = pose[5]
    
        try: 
            successful_candidate_trajectories.append(ik_srv(ik_msg) )   #trajectory_msgs/JointTrajectory
            i = i + 1
            indx_list.append(i)

        except :
            i = i + 1
            print("..............................")
            continue


    print("there are" , len(successful_candidate_trajectories) , " possible ways to grab an object with left arm")
    #print("indices de poses aprobadas", indx_list)

    # Prepara la respuesta al servicio /vision/gripper_orientation_grasping
    resp = InverseKinematicsPose2TrajResponse()

    resp = successful_candidate_trajectories[0]
    #resp.articular_trajectory.articular_trajectory.joint_names = "la"
    print("graficando frames en Rviz...")

    """
    broadcaster_frame_object('base_link', 'candidate1' , pose_list_q[ indx_list[0]] )
    broadcaster_frame_object('base_link', 'candidate2' , pose_list_q[ indx_list[1] ] )
    broadcaster_frame_object('base_link', 'candidate3' , pose_list_q[ indx_list[2] ] )
    broadcaster_frame_object('base_link', 'candidate4' , pose_list_q[ indx_list[3] ] )
    broadcaster_frame_object('base_link', 'candidate5' , pose_list_q[ indx_list[4] ] )
    """

    return resp




def main():
    global pose_obj_frame_base, image_seg_service, pose_in, listener , ik_srv
    print("Node to grab objects based on their orientation...")
    rospy.init_node("gripper_orientation_for_grasping")
    rospy.Service("/vision/gripper_orientation_grasping", InverseKinematicsPose2Traj, callback)
    listener = tf.TransformListener()
    # se suscribe al servicio /manipulation/ik_trajectory
    rospy.wait_for_service( '/manipulation/la_ik_trajectory' )
    ik_srv = rospy.ServiceProxy( '/manipulation/la_ik_trajectory' , InverseKinematicsPose2Traj )
    

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        loop.sleep()

if __name__ == '__main__':
    main()


