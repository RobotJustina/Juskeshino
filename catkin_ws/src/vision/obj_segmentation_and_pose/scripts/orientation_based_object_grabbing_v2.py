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
     Reglas para la construccion del sistema coordenado del gripper(frame 'base_link') :

        1: Se toma como eje x a la componente principal resultante de PCA

        2. Se define tamanio de paso para la generacion de puntos de candidatos de agarre y se almacenan en una lista.
        
        3. Se define el rango dentro del cual se generan los candidatos.
         
        4. Se define la ecuacion parametrica de una circunferencia, dada por la interseccion de un plano pi normal a 
           la componente principal resultante del PCA y una esfera de radio r y centro en centroide.

           Ecuacion del plano: a(x-x1) + b(y-y1) + c(z-z1) + d = 0
           vector N: 1PCA
           punto perteneciente al plano: se obtiene de la 2PCA

           Ecuacion de la esfera: (x-h)² + (y-k)² + (z-l)² = r²
            centroide = (h,k,l)

           Ecuacion de la circunferencia:  
            x = epsilon * np.cos(theta)
            y = epsilon * np.sin(theta)
            z = z_centroid 
        
        5. Para cada punto se genera un sistema coordenado y se almacena en una lista:
            eje x = componente principal resultante del PCA.
            eje z = vector formado por el punto de centroide y punto, en direccion externa a la circunferencia.
            eje y = producto cruz entre eje x y eje z.

        6. Se verifican las orientaciones de forma grafica.
        
        7. Se prueba la validez de los candidatos de agarre enviandolos a la cinematica inversa, en el frame shoulders. 
            Los puntos validos se almacenan en una nueva lista.

        8. Se elije el mejor candidato de agarre conforme a un criterio y se envia para la pose de gripper.
"""


def box(obj_pose):
    print("take a box")
    



def cube_or_sphere(obj_pose):
    print("take cube or sphere")   



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





def angle_pca_floor(cartesian_pose):  
    MT = tft.euler_matrix( cartesian_pose[3] , cartesian_pose[4], cartesian_pose[5]) 
    pc1 = np.asarray( [MT[0,0], MT[1,0], MT[2,0]])   # eje principal

    # componente principal esta en eje x
    if pc1[0] < 0:
        pc1[0] = pc1[0]*-1

    eje_z = np.asarray([0, 0, 1], dtype=np.float64 )# vector normal al plano xy
    # angulo entre pc1 y plano xy frame base_link
    ang_pc1_pixy = np.arcsin( np.dot(pc1 , eje_z) / (np.linalg.norm(pc1) * 1) )
    print("angulo pc1", np.rad2deg( ang_pc1_pixy ) )

    return ang_pc1_pixy # en radianes





def grip_rules(obj_pose, type_obj, angle_obj):
    """
    Esta funcion calcula la pose adecuada de la pinza para tomar el objeto,
    recibe como argumento la pose del objeto en el frame base_link 
    """
    if type_obj == 1:
        print("cube_or_sphere")

    if type_obj == 2:
        print("box")

    if type_obj == 3:
        print("take cylinder or prism") 
        grasp_candidates_quaternion = cylinder_or_prism(obj_pose, angle_obj)
        return grasp_candidates_quaternion
    
    return 




def cylinder_or_prism(obj_pose, angle_obj ):   
    y_object = 0.03 # ancho del objeto
    epsilon = y_object # radio de la circunferencia

    coord_centroid = np.asarray([obj_pose[0] , obj_pose[1], obj_pose[2]]) # origen de la circunferencia
    x_centroid = obj_pose[0] 
    y_centroid = obj_pose[1]
    z_centroid = obj_pose[2]
    grasp_candidates_quaternion = []
    grasp_candidates_rpy = []
    
    MT = tft.euler_matrix( obj_pose[3] , obj_pose[4], obj_pose[5]) 

    # Step 1: *****************************************************************************************
    axis_x_obj = np.asarray( [MT[0,0], MT[1,0], MT[2,0]])   # eje principal

    if (angle_obj < np.deg2rad(30)) or (angle_obj > np.deg2rad(150)):
        print("Prisma horizontal*****************************")
        # Step 2: *****************************************************************************************
        offset_theta = np.deg2rad(0)    # donde inicia la generacion de agarres candidatos
        theta = 0 + offset_theta
        step_size = np.deg2rad(10)   
        
        # Step 3:
        range_points = np.deg2rad(3600)          # rango dentro del cual se generan los candidatos 360 grados
        num_points = int(range_points / step_size)
        print("number points", num_points)
        temp = 0

    else: 
        print("Prisma vertical*******************************")
        # Step 2: *****************************************************************************************
        offset_theta = np.deg2rad(90)    # donde inicia la generacion de agarres candidatos
        theta = 0 + offset_theta
        step_size = np.deg2rad(10)   
        
        # Step 3:
        range_points = np.deg2rad(270)          # rango dentro del cual se generan los candidatos 360 grados
        num_points = int(range_points / step_size)
        print("number points", num_points)
        temp = 0


    
    # obtencion de origen de frame candidato
    for i in range( num_points + 1):   
    # Step 3: *****************************************************************************************
        point = np.asarray([ epsilon*np.cos(theta) + x_centroid , epsilon*np.sin(theta) + y_centroid , z_centroid ])
        # obtencion de eje x
        axis_x_point = axis_x_obj / np.linalg.norm( (axis_x_obj) )

        # Step 4:*******************************************************************************
        # obtencion de eje z
        axis_z_point = (point - coord_centroid ) / np.linalg.norm( (point - coord_centroid ) )
        
        # obtencion de eje y
        axis_y_point = np.cross(axis_z_point , axis_x_point) / np.linalg.norm( np.cross(axis_z_point , axis_x_point) )

        # los cuaterniones necesarios para generar el frame del objeto
        comb = [[axis_x_point, axis_y_point, axis_z_point],[ axis_y_point , axis_x_point , axis_z_point],
                [ axis_z_point , axis_y_point , axis_x_point ],[ axis_x_point , axis_z_point , axis_y_point ],
                [ axis_z_point , axis_x_point , axis_y_point ],[ axis_y_point , axis_z_point , axis_x_point ]]
        
        RM = np.asarray( comb[0] ) 
        RM = RM.T
        TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
              [RM[1,0], RM[1,1] , RM[1,2], 0],
              [RM[2,0], RM[2,1] , RM[2,2], 0], 
              [      0,        0,       0, 1]]

        r,p,y = tft.euler_from_matrix(np.asarray(TM))
        R, P, Y = r, p ,y

        q_gripper = tft.quaternion_from_euler( R , P , Y )
        q_gripper = tft.quaternion_from_matrix ( TM )
        
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
    cartesian_pose = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    type_obj = req.initial_guess[0]

    ang_pc1_pixy = angle_pca_floor(cartesian_pose) 
    pose_list_q = grip_rules(cartesian_pose , type_obj, ang_pc1_pixy) # solo para cilindro 
    # retorna el ang entre piso y pc1
     
    
    # Step 5:****************************************************************
    offset = 0
    broadcaster_frame_object('base_link', 'candidate1' , pose_list_q[0+offset] )
    broadcaster_frame_object('base_link', 'candidate2' , pose_list_q[2] )
    broadcaster_frame_object('base_link', 'candidate3' , pose_list_q[4] )
    broadcaster_frame_object('base_link', 'candidate4' , pose_list_q[6] )
    broadcaster_frame_object('base_link', 'candidate5' , pose_list_q[8] )
    broadcaster_frame_object('base_link', 'candidate6' , pose_list_q[10] )
    broadcaster_frame_object('base_link', 'candidate7' , pose_list_q[12] )
    broadcaster_frame_object('base_link', 'candidate7' , pose_list_q[15] )
    broadcaster_frame_object('base_link', 'candidate9' , pose_list_q[17] )
    """
    broadcaster_frame_object('base_link', 'candidate10' , pose_list_q[20] )
    broadcaster_frame_object('base_link', 'candidate11' , pose_list_q[22] )
    broadcaster_frame_object('base_link', 'candidate12' , pose_list_q[26] )
    broadcaster_frame_object('base_link', 'candidate13' , pose_list_q[30] )
    broadcaster_frame_object('base_link', 'candidate14' , pose_list_q[34] )
    """

    # Step 6:***************************************************************
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

    # Step 7 **************************************************************
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
            print("..............")
        except :
            i = i + 1
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


