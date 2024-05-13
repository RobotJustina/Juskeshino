#!/usr/bin/env python3
import math
import rospy
import numpy as np
import tf.transformations as tft
import tf2_ros
import ros_numpy
import cv2
from geometry_msgs.msg import PointStamped,  Point, Pose, Vector3
from visualization_msgs.msg import Marker
from vision_msgs.srv import *
import geometry_msgs.msg
from cv_bridge import CvBridge


MAXIMUM_GRIP_LENGTH = 0.16
MAXIMIUN_CUBE_SIZE  = 0.15


def get_cv_mats_from_cloud_message(cloud_msg):
    img_xyz = ros_numpy.point_cloud2.pointcloud2_to_array(cloud_msg)  # dim 480 x 640 
    rgb_array = img_xyz['rgb'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    if debug:
        cv2.imshow("imagen reconstruida", img_bgr) #*****************
        cv2.waitKey(0)
    cv_mats = np.zeros((480, 640, 3))
    for i in range(480):
        for j in range(640):
            cv_mats[i,j][0] = img_xyz[i,j][0]
            cv_mats[i,j][1] = img_xyz[i,j][1]
            cv_mats[i,j][2] = img_xyz[i,j][2]
    return cv_mats


def get_object_xyz(cloud_xyz, mask):
    mask = imgmsg_to_cv2(mask)
    th, mask = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)
    # Take xyz points only in mask and remove points with zero X
    obj_xyz = cloud_xyz[(mask == 255) & (cloud_xyz[:,:,0] > 0.1)].copy()
    
    return obj_xyz


def imgmsg_to_cv2(img_array): # Entra imagen en forma de array
    global debug 
    bridge = CvBridge()
    obj_cv = bridge.imgmsg_to_cv2(img_array)
    if debug:
        cv2.imshow("Show image was called....", obj_cv) #*****************
        cv2.waitKey(0)
    return obj_cv


def get_centroid(img_bgr):
    centroid = 1
    return centroid


def pca(xyz_points):    # pc del contorno mas cercano
    eig_val, eig_vect = np.linalg.eig(np.cov(np.transpose(xyz_points))) # Eigenvalues and eigenvectors from Point Cloud Cov Matrix
    idx = eig_val.argsort()
    eig_val  = eig_val[idx]
    eig_vect = np.transpose(np.transpose(eig_vect)[idx])
    pts_frame_PCA = np.transpose(np.dot(eig_vect, np.transpose(xyz_points)))
    #pt_frame_PCA = np.transpose(np.dot(eig_vect, np.transpose(centrid)))
    
    #print("EigVal...........", eig_val)
    H = np.max(pts_frame_PCA[:, 2]) - np.min(pts_frame_PCA[:, 2])
    L = np.max(pts_frame_PCA[:, 1]) - np.min(pts_frame_PCA[:, 1])
    W = np.max(pts_frame_PCA[:, 0]) - np.min(pts_frame_PCA[:, 0])
    
    size_obj = np.asarray([H, L,W])
    idx = size_obj.argsort()  # entrega el orden de las 3 componentes principale de menor a mayor 
    size_obj = size_obj[idx]
    print("size obj", size_obj)         # menor a mayor 
    # Los vectores de salida estan en el frame base_link
    size_object = Vector3(x = size_obj[2] , z = size_obj[1], y = size_obj[0])
    return [eig_vect[:,2], eig_vect[:,1] , eig_vect[:,0]], [eig_val[2], eig_val[1], eig_val[0]], size_object


def object_pose_without_PCA(centroid):
    # Asignacion de ejes del objeto
    eje_x_obj = np.asarray([1, 0, 0], dtype=np.float64 )# coordenadas en 'base_link' 
    eje_z_obj = np.asarray([0, 0, 1], dtype=np.float64 )# coordenadas en 'base_link'
    eje_y_obj = np.asarray([0, 1, 0], dtype=np.float64 )# coordenadas en 'base_link'
    # Se forma la matriz de rotacion (columnas) del objeto, a partir de ella se obtienen los cuaterniones necesarios para generar el frame del objeto
    RM = np.asarray([eje_x_obj, eje_y_obj , eje_z_obj])
    RM = RM.T
    TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
         [RM[1,0], RM[1,1] , RM[1,2], 0],
         [RM[2,0], RM[2,1] , RM[2,2], 0], 
         [0, 0, 0, 1]]

    r,p,y = tft.euler_from_matrix(np.asarray(TM))
    quaternion_obj = tft.quaternion_from_euler(r, p, y)
    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = centroid[0], centroid[1], centroid[2]
    obj_pose.orientation.x = quaternion_obj[0]
    obj_pose.orientation.y = quaternion_obj[1]
    obj_pose.orientation.z = quaternion_obj[2]
    obj_pose.orientation.w = quaternion_obj[3]

    broadcaster_frame_object("base_link", "object", obj_pose)
    # Retorna la pose del objeto en 'base_link'
    return obj_pose



def object_pose(centroid, principal_component, second_component, size_obj):  # vectores de entrada estan en frame base_link
    """
    Construction of the coordinate system of the object,the construction of the frame depends on the coordinate system of the robot gripper 
    Ademas se imponen restricciones para que la pose sea realizable por el gripper
    """
    principal_component = np.asarray(principal_component)
    if principal_component[2] < 0: 
        principal_component = -1 * principal_component

    # calculo de angulo de primera componente respecto de la superficie
    eje_z = np.asarray([0, 0, 1], dtype=np.float64 )# coordenadas en 'base_link'
    angle_obj = np.abs(np.arcsin( np.dot(principal_component , eje_z ) / (np.linalg.norm(principal_component ) * 1) ))
    print("angulo del objeto respecto de la superficie: ", np.rad2deg(angle_obj))


    """
    p = Point()
    p.x, p.y, p.z = second_component[0], second_component[1], second_component[2] 
    print("s c", second_component)
    publish_arow_marker(centroid, p ,"base_link", 6, "z_obj")
    """
    
    # ************************************************************************************************
    if ((angle_obj < np.deg2rad(30)) or (angle_obj > np.deg2rad(150))):  # Si el objeto es horizontal
        if (size_obj.x >= MAXIMIUN_CUBE_SIZE):        # Si el objeto es
            # Realiza agarre superior
            obj_state = 'horizontal'
            print("Eje principal horizontal")
            # Angulo respecto de x_base_link
            angle_obj_x_bl =  math.atan2(principal_component[1], principal_component[0]) 
            print("Angulo respecto de eje X de BASE_LINK:", np.rad2deg(angle_obj_x_bl))


            if angle_obj_x_bl > np.deg2rad(60) or angle_obj_x_bl < np.deg2rad(-130):
                print("Angulo 1pc fuera de limites corregido...........")
                principal_component[0] = -1*principal_component[0]
                principal_component[1] = -1*principal_component[1]
                                                        
            if second_component[2] < 0: 
                print("se corrigio sc")
                second_component = -1 * second_component    # sc no puede ser un vector negativo

            
            if ((size_obj.x /size_obj.z) > 1.333):    # Objeto prismatico
                print("cylinder..........")
                # Asignacion de ejes del objeto
                eje_x_obj = principal_component 
                eje_z_obj = eje_z
                eje_y_obj = np.cross(eje_z_obj , eje_x_obj )

            else:
                print("BOWL.................")
                eje_x_obj = np.asarray([1, 0, 0], dtype=np.float64 )# coordenadas en 'base_link'
                eje_z_obj = eje_z
                eje_y_obj = np.cross(eje_z_obj , eje_x_obj )
            
        
        else:   # Sie el objeto es vertical
            # Si el objeto es pequenio se construye un frame que permita el agarre superior
            print("se construye un frame que permita el agarre superior")
            obj_state = 'horizontal'
            eje_x_obj = np.asarray([1, 0, 0], dtype=np.float64)
            eje_z_obj = eje_z
            eje_y_obj = np.cross(eje_z_obj , eje_x_obj )


    # ********************************************
    else: # Realiza agarre lateral
        obj_state = 'vertical'
        print("Eje principal vertical")
        if (size_obj.x >= MAXIMIUN_CUBE_SIZE):
            # Mide el angulo de la proyeccion del vector z_obj sobre el plano xy de base_link y el eje x_base_link
            angle_2pc_x_bl =  math.atan2(second_component[1], second_component[0])  
            print("angulo de la proyeccion del vector z_obj sobre el plano xy de base_link y el eje x_base_link", np.rad2deg(angle_2pc_x_bl))

            print("eje_x_obj before:_____", principal_component)
            print("eje_y_obj before:_____")
            print("eje_z_obj before:_____", second_component)

            if angle_2pc_x_bl < np.deg2rad(23):
                if angle_2pc_x_bl > np.deg2rad(-157):
                    print("angulo fuera de rango de agarre ,se invirtio sentido de 2pc")
                    p = Point()
                    p.x, p.y, p.z = second_component[0], second_component[1], second_component[2] 
                    print("s c", second_component)
                    publish_arow_marker(centroid, p ,"base_link", 6, "z_obj")
                    second_component[0] = -1*second_component[0]
                    second_component[1] = -1*second_component[1]
                    angle_2pc_x_bl =  math.atan2(second_component[1], second_component[0]) 
                    print("angulo de vector z_obj respecto del eje x_base_link despues", np.rad2deg(angle_2pc_x_bl))

                    # Asignacion de ejes del objeto
                    eje_z_obj = second_component 
                    eje_x_obj = principal_component 
                    eje_y_obj = np.cross(eje_z_obj , eje_x_obj )



            else:
                # Asignacion de ejes del objeto
                eje_z_obj = second_component 
                eje_x_obj = principal_component 
                eje_y_obj = np.cross(eje_z_obj , eje_x_obj )

        else:
            # Si el objeto es pequenio se construye un frame que permita el agarre superior
            print("SMALL VERTICAL.................")
            print("se construye un frame que permita el agarre superior")
            eje_x_obj = np.asarray([1, 0, 0], dtype=np.float64)
            eje_z_obj = eje_z
            eje_y_obj = np.cross(eje_z_obj , eje_x_obj )

    # **************************************************************************************************
    print("eje_x_obj:_____", eje_x_obj)
    print("eje_y_obj:_____", eje_y_obj)
    print("eje_z_obj:_____", eje_z_obj)

    axis_x_obj = Point(x = eje_x_obj[0], y = eje_x_obj[1], z = eje_x_obj[2])    # Vector x_obj
    # Se forma la matriz de rotacion (columnas) del objeto, a partir de ella se obtienen los cuaterniones necesarios para generar el frame del objeto
    RM = np.asarray([eje_x_obj, eje_y_obj , eje_z_obj])
    RM = RM.T
    TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
         [RM[1,0], RM[1,1] , RM[1,2], 0],
         [RM[2,0], RM[2,1] , RM[2,2], 0], 
         [0, 0, 0, 1]]

    r,p,y = tft.euler_from_matrix(np.asarray(TM))
    quaternion_obj = tft.quaternion_from_euler(r, p, y)
    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = centroid[0], centroid[1], centroid[2]
    obj_pose.orientation.x = quaternion_obj[0]
    obj_pose.orientation.y = quaternion_obj[1]
    obj_pose.orientation.z = quaternion_obj[2]
    obj_pose.orientation.w = quaternion_obj[3]

    broadcaster_frame_object("base_link", "object", obj_pose)
    # Retorna la pose del objeto en 'base_link'
    return obj_pose , axis_x_obj, obj_state



def broadcaster_frame_object(frame, child_frame, pose):   # Emite la transformacion en el frame base_link,
    #br = tf2_ros.TransformBroadcaster()
    br =  tf2_ros.StaticTransformBroadcaster()
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



def publish_arow_marker(centroide_cam, p1,frame_id, id, name):  
    p0 = PointStamped()
    p0.header.frame_id = frame_id
    p0.point.x , p0.point.y, p0.point.z = centroide_cam[0], centroide_cam[1], centroide_cam[2]
    frame = frame_id
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = Marker.ARROW
    marker.ns = name#'principal_component'
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.id = 1#+10
    # body radius, Head radius, hight head
    marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.1, 0.04 
    marker.color.r , marker.color.g , marker.color.b, marker.color.a = 20, 0.0, 100.0, 1.0
    point = Point()
    point.x, point.y, point.z = p0.point.x + p1.x , p0.point.y + p1.y , p0.point.z + p1.z
    marker.lifetime = rospy.Duration(10.0)
    marker.points = [p0.point, point]
    marker.pose.orientation.w = 1.0
    marker_pub.publish(marker)



def get_obj_pose_response(obj_state, c_obj, size_obj, obj_pose, graspable):
    resp = RecognizeObjectResponse()
    resp.recog_object.object_state = obj_state
    resp.recog_object.category = c_obj
    resp.recog_object.graspable  = graspable 
    resp.recog_object.size = size_obj
    resp.recog_object.pose = obj_pose
    return resp




def object_category(size_obj, obj_state):  # estima la forma geometrica del objeto. (x,z,y)

    if (size_obj.z <= MAXIMUM_GRIP_LENGTH) and (size_obj.y <= MAXIMUM_GRIP_LENGTH) and (size_obj.x >= 0.13):

        if((size_obj.x /size_obj.z) < 1.4) and (obj_state == 'horizontal'):
            print("The object is a bowl")
            return "BOWL", True
        else:
            print("The object will be GRABBED AS PRISM..................")
            return "PRISM", True


    else:
        if(size_obj.z <= MAXIMUM_GRIP_LENGTH) and (size_obj.y <= MAXIMUM_GRIP_LENGTH) and (size_obj.x < MAXIMIUN_CUBE_SIZE) and (size_obj.x > 0.07):
            print("Object spheric or cubic SMALL, SUPERIOR GRIP will be made")
            return "CUBIC", True
            
        else:
            if((size_obj.x /size_obj.z) < 1.4) and (obj_state == 'horizontal'):
                print("The object is a bowl")
                return "BOWL", True
            else:
                if (size_obj.z >= MAXIMUM_GRIP_LENGTH) and (size_obj.y >= MAXIMUM_GRIP_LENGTH) and (size_obj.x >= MAXIMUM_GRIP_LENGTH):
                    return "BOX", True#"BIG", False
                
                print("size object > MAX LENGHT GRIP")
                print("The object will be GRABBED as BOX....................")
                return "BOX", True
    


def callbackPoseObjectOrientation(req):  # Request is a PointCloud2
    cv_mats= get_cv_mats_from_cloud_message(req.point_cloud)
    obj_xyz = get_object_xyz(cv_mats , req.obj_mask)

    print("******************************")
    print("**    OBJECT INFORMATION    **")
    print("******************************")

    centroid = np.mean(obj_xyz, axis=0)
    pca_vectors, eig_val, size_obj = pca(obj_xyz)
    obj_pose, axis_x_obj, obj_state = object_pose(centroid, pca_vectors[0], pca_vectors[1], size_obj )
    c_obj, graspable = object_category(size_obj, obj_state)

    print("CENTROID:____", centroid)
    #publish_arow_marker(centroid, axis_x_obj, 'base_link', ns ="principal_component", id=22)
    print("SIZE:________", )
    print(size_obj)
    print("OBJECT TYPE", c_obj)
    print("STATE", obj_state)
    
    resp = get_obj_pose_response( obj_state, c_obj, size_obj, obj_pose, graspable)
    return resp


def callbackPoseObject(req):
    cv_mats= get_cv_mats_from_cloud_message(req.point_cloud)
    obj_xyz = get_object_xyz(cv_mats , req.obj_mask)

    print("**    OBJECT INFORMATION    **********************")

    centroid = np.mean(obj_xyz, axis=0)
    graspable = True
    obj_pose = object_pose_without_PCA(centroid)
    print("CENTROID:____", centroid)
    resp = RecognizeObjectResponse()
    resp.recog_object.graspable  = graspable 
    resp.recog_object.pose = obj_pose
    resp.recog_object.category = "CUBIC"
    return resp




def main():
    print("Obj_segmentation_and_pose --> obj_pose by Iby *******************(✿◠‿◠)7**")
    rospy.init_node("object_pose")

    global pub_point, marker_pub, debug 
    debug = False
    rospy.Service("/vision/obj_segmentation/get_obj_pose_with_orientation", RecognizeObject, callbackPoseObjectOrientation) 
    rospy.Service("/vision/obj_segmentation/get_obj_pose_without_orientation", RecognizeObject, callbackPoseObject) 
    pub_point = rospy.Publisher('/vision/detected_object', PointStamped, queue_size=10)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10) 

    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = 0, 0, 0
    obj_pose.orientation.x = 0
    obj_pose.orientation.y = 0
    obj_pose.orientation.z = 0
    obj_pose.orientation.w = 1.0
    broadcaster_frame_object("base_link", "object", obj_pose)

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
