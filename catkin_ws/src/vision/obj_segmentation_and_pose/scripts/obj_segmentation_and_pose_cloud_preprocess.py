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


MAXIMUM_GRIP_LENGTH = 0.14

def get_cv_mats_from_cloud_message(cloud_msg):
    img_xyz = ros_numpy.point_cloud2.pointcloud2_to_array(cloud_msg)  # dim 480 x 640, 
    rgb_array = img_xyz['rgb'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    #cv2.imshow("imagen sin plano #1", img_bgr) #*****************
    #cv2.waitKey(0)
    xyz = np.zeros((480, 640, 3))
    for i in range(480):
        for j in range(640):
            xyz[i,j][0] = img_xyz[i,j][0]
            xyz[i,j][1] = img_xyz[i,j][1]
            xyz[i,j][2] = img_xyz[i,j][2]
    return [img_bgr, xyz]


def segment_by_contour(img_bgr, pointCloud_array, original_cloud):
    #print("Segmenting image by contours")
    values=img_bgr.reshape((-1,3))  # (M,3=BGR), M = num of pixels
    values= np.float32(values)
    # Image color quantification with k-medias
    criteria= ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,100,0.1)    # centroid convergence criterion
    k=7     # num of colors in image
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS) 
    # convert into uint8, and make original image
    cc=np.uint8(cc) # Image with reduced colors, conversion to the format with which cv2 works
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(img_bgr.shape)
    #cv2.imshow("imagen con clusterización por color #3", segmented_image) #*****************
    #cv2.waitKey(0)
    # Change the color space from BGR to grayscale
    image_gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("imagen en escala de grises #4", image_gray) #*****************
    #cv2.waitKey(0)
    img_bin = cv2.adaptiveThreshold(image_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,15)
    #cv2.imshow("imagen umbralizada #5", img_bin) #*****************
    #cv2.waitKey(0)
    kernel = np.ones((3,3),np.uint8)    # Define a kernel for erosion
    img_erode=cv2.erode(img_bin , kernel,iterations=3)  #4 eroded image
    #cv2.imshow("imagen erosionada #6", img_erode) #*****************
    #cv2.waitKey(0)
    img_dilate = cv2.dilate(img_erode , kernel, iterations=2) # apertura y cierre
    #cv2.imshow("imagen dilatada #7", img_dilate) #*****************
    #cv2.waitKey(0)
    edged = cv2.Canny(img_dilate, 100, 20)
    #cv2.imshow("imagen con filtro Canny #8", edged) #*****************
    #cv2.waitKey(0)
    #Finding contours in the image, each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object
    contours, hierarchy = cv2.findContours(edged.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    #if original_cloud:
     #   contours, hierarchy = cv2.findContours(edged.astype('uint8'),cv2.RETR_TREE ,cv2.CHAIN_APPROX_NONE)

    if not len(contours) > 0:
        return False ,None, None   

    min_distance = float("inf")
    nearest_obj_bgr = None
    nearest_obj_xyz = None
    nearest_centroid = None
    print("Found " + str(len(contours)) + " countours")

    for j, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < 1000 or area > 25000: continue # discarding contours by area
        
        print("no descartado")
        mask = np.zeros((img_bgr.shape[0], img_bgr.shape[1]),np.uint8)
        cv2.drawContours(mask, contours, j, 255 , thickness = cv2.FILLED)  # llena contorno para mascara
        # the mask is eroded to ensure that we only get the point cloud of the object
        mask =cv2.erode(mask , kernel,iterations=3)
        obj_bgr, obj_xyz = get_object_bgr_and_xyz(img_bgr, pointCloud_array, mask)
        cv2.imshow("obj", obj_bgr) #*****************
        cv2.waitKey(0)
        # Basic threhold example 
        th, mask_bin = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY); 
        cv2.imshow("mask", mask) #*****************
        cv2.waitKey(0)
        print("TYPE IMAGE",type(obj_bgr))
        obj_centroid = np.mean(obj_xyz, axis=0)
        print("centroide " , obj_centroid)
        distance = math.sqrt(obj_centroid[0]**2 + obj_centroid[1]**2)
        if distance < min_distance:
            min_distance = distance
            nearest_obj_bgr = obj_bgr.copy()
            nearest_obj_xyz = obj_xyz.copy()
            nearest_centroid= obj_centroid.copy()

        bridge = CvBridge()
        image_obj = bridge.cv2_to_imgmsg(obj_bgr , "bgr8")
        image_mask = bridge.cv2_to_imgmsg(mask , "bgr8")

    return True, nearest_centroid, nearest_obj_xyz, image_obj, image_mask
    




def get_object_bgr_and_xyz(img_bgr, img_xyz, mask):
    obj_bgr = img_bgr.copy()
    obj_bgr[mask == 0] = 0
    # Take xyz points only in mask and remove points with zero X
    obj_xyz = img_xyz[(mask == 255) & (img_xyz[:,:,0] > 0.1)].copy()
    return obj_bgr, obj_xyz



def pca(xyz_points,centrid):    # pc del contorno mas cercano
    eig_val, eig_vect = np.linalg.eig(np.cov(np.transpose(xyz_points))) # Eigenvalues and eigenvectors from Point Cloud Cov Matrix
    idx = eig_val.argsort()
    eig_val  = eig_val[idx]
    eig_vect = np.transpose(np.transpose(eig_vect)[idx])
    pts_frame_PCA = np.transpose(np.dot(eig_vect, np.transpose(xyz_points)))
    #pt_frame_PCA = np.transpose(np.dot(eig_vect, np.transpose(centrid)))
    
    print("EigVal...........", eig_val)
    H = np.max(pts_frame_PCA[:, 2]) - np.min(pts_frame_PCA[:, 2])
    L = np.max(pts_frame_PCA[:, 1]) - np.min(pts_frame_PCA[:, 1])
    W = np.max(pts_frame_PCA[:, 0]) - np.min(pts_frame_PCA[:, 0])
    
    size_obj = np.asarray([H, L,W])
    idx = size_obj.argsort()
    size_obj = size_obj[idx]
    print("size obj", size_obj)
    # Los vectores de salida estan en el frame base_link
    size_object = Vector3(x = size_obj[2] , z = size_obj[1], y = size_obj[0])
    return [eig_vect[:,2], eig_vect[:,1] , eig_vect[:,0]], [eig_val[2], eig_val[1], eig_val[0]], size_object



def object_pose(centroid, principal_component, second_component):  # vectores de entrada estan en frame base_link
    """
    Construction of the coordinate system of the object,the construction of the frame depends on the coordinate system of the robot gripper 
    Ademas se imponen restricciones para que la pose sea realizable por el gripper
    """
    principal_component = np.asarray(principal_component)
    if principal_component[2] < 0: 
        principal_component = -1 * principal_component

    # calculo de angulo de primera componente respecto de la superficie
    eje_z = np.asarray([0, 0, 1], dtype=np.float64 )# vector normal al plano xy
    angle_obj = np.abs(np.arcsin( np.dot(principal_component , eje_z ) / (np.linalg.norm(principal_component ) * 1) ))
    print("angulo del objeto respecto de la superficie: ", np.rad2deg(angle_obj))

    if (angle_obj < np.deg2rad(30)) or (angle_obj > np.deg2rad(150)):   
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
        # Asignacion de ejes del objeto
        eje_x_obj = principal_component 
        eje_z_obj = eje_z
        eje_y_obj = np.cross(eje_z_obj , eje_x_obj )

    else: # Realiza agarre lateral
        obj_state = 'vertical'
        print("Eje principal vertical")
        angle_2pc_x_bl =  math.atan2(second_component[1], second_component[0]) 
        print("angulo de z_obj respecto de eje x base link", np.rad2deg(angle_2pc_x_bl))
        if angle_2pc_x_bl < np.deg2rad(23):
            if angle_2pc_x_bl > np.deg2rad(-157):
                print("angulo fuera de rango de agarre ,se invirtio sentido de 2pc")
                second_component[0] = -1*second_component[0]
                second_component[1] = -1*second_component[1]
                angle_2pc_x_bl =  math.atan2(second_component[1], second_component[0]) 
                print("angulo de z_obj respecto de eje x base link despues", np.rad2deg(angle_2pc_x_bl))

                #sec_comp = Point(x = second_component[0], y =second_component[1], z=second_component[2])
                #publish_arow_marker(centroid, sec_comp, 'base_link', ns ="second_component", id=23)
                #publish_arow_marker( centroid, sec_comp, 'base_link')

        # Asignacion de ejes del objeto
        eje_z_obj = second_component 
        eje_x_obj = principal_component 
        eje_y_obj = np.cross(eje_z_obj , eje_x_obj )


    axis_x_obj = Point()
    axis_x_obj.x, axis_x_obj.y, axis_x_obj.z = eje_x_obj[0], eje_x_obj[1], eje_x_obj[2]
    # Se forma la matriz de rotacion (columnas) del objeto, a partir de ella se obtienen los cuaterniones necesarios para generar el frame del objeto
    RM = np.asarray([eje_x_obj, eje_y_obj , eje_z_obj])
    RM = RM.T
    TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
         [RM[1,0], RM[1,1] , RM[1,2], 0],
         [RM[2,0], RM[2,1] , RM[2,2], 0], 
         [0, 0, 0, 1]]

    r,p,y = tft.euler_from_matrix(np.asarray(TM))
    q_obj = tft.quaternion_from_euler(r, p, y)
    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = centroid[0], centroid[1], centroid[2]
    obj_pose.orientation.x = q_obj[0]
    obj_pose.orientation.y = q_obj[1]
    obj_pose.orientation.z = q_obj[2]
    obj_pose.orientation.w = q_obj[3]
    return obj_pose , axis_x_obj



def broadcaster_frame_object(frame, child_frame, pose):   # Emite la transformacion en el frame base_link,
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



def publish_arow_marker(centroide_cam, p1,frame_id, ns, id):  
    p0 = PointStamped()
    p0.header.frame_id = frame_id
    p0.point.x , p0.point.y, p0.point.z = centroide_cam[0], centroide_cam[1], centroide_cam[2]
    frame = frame_id
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = Marker.ARROW
    marker.ns = 'principal_component'
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



def object_category(fpc, spc, thpc):  # estima la forma geometrica del objeto. (x,z,y)
    print("1pca, 2pca, 3pca", fpc,spc,thpc)
    # coeficiente de similitud entre aristas de bounding box del objeto
    c21, c31, c32 =  spc * ( 100 / fpc),   thpc * ( 100 / fpc),    thpc * ( 100 / spc)  
    print("c21, c31, c32", c21, c31, c32)
    if c21 >= 60:    # Means 1PC and 2PC are similar
        if (c31 > 70) or (c32 > 70):    # Means 2PC and 3PC are similar
            print("cubic bounding box") # debe verificarse grapabilidad
            return "cube"
        if c31 <= 70 or c32 <= 70:
            print("box")  #se puede confundir con prism
            return "box"
    
    elif c21 < 60:    # Means 1PC is much bigger than 2PC        
        if c31 <= 70 or c32 <= 70:
            print("box")
            return "box"
        if c32 > 70 and spc < MAXIMUM_GRIP_LENGTH:    # Means 2PC and 3PC are similar
            print("prismatic bounding box") # debe verificarse grapabilidad
            return "prism"
        else: return 'box'
    else:
        print("Could not determine the shape of the object...")
        return "0"
    



def callback_RecognizeObject(req):  # Request is a PointCloud2
    global clt_get_points
    print("ObjSegmenter.->The service has been requested **************")
    req_ppc = PreprocessPointCloudRequest()
    req_ppc.input_cloud = req.point_cloud
    try:
        resp_clt_get_points = clt_get_points(req_ppc)
        print("ObjSegmenter.->Preprocessed pc received")
        msg = resp_clt_get_points.output_cloud
        original_cloud = False
    except:
        msg = req.point_cloud
        print("ObjSegmenter.->Cannot get preprocessed cloud. Using original cloud.")
        original_cloud = True
    
    img_bgr, img_xyz = get_cv_mats_from_cloud_message(msg)
    
    found_object, centroid, obj_xyz, image_obj, image_mask = segment_by_contour(img_bgr, img_xyz, original_cloud)
    resp = RecognizeObjectResponse()
    
    if found_object:
        print("An object was detected...............")
        print("object position" , centroid)
        pca_vectors, eig_val, size_obj = pca(obj_xyz, centroid)
        #obj_state = object_state(pca_vectors[0], pca_vectors[1])

        #print("OBJECT STATE", obj_state[0])

        c_obj = object_category(size_obj.x, size_obj.z, size_obj.y)

        obj_pose, axis_x_obj = object_pose(centroid, pca_vectors[0], pca_vectors[1])
        publish_arow_marker(centroid, axis_x_obj, 'base_link', ns ="principal_component", id=22)
        broadcaster_frame_object("base_link", "object", obj_pose)
        print("size object i frame object", size_obj)
        print("object category", c_obj)
        # Rellenando msg 
        resp.image = image_mask     # mascara del objeto detectado
        resp.recog_object.image = image_obj
        resp.recog_object.category = c_obj
        resp.recog_object.header = req.point_cloud.header
        resp.recog_object.size = size_obj
        resp.recog_object.pose = obj_pose
        resp.recog_object.graspable = found_object
        return resp
    else:
        print("no object detected on stage")
        resp.recog_object.graspable = False
        return resp



def main():
    print("Node to segment objects in a image from camera by Iby..ʕ•ᴥ•ʔ")
    rospy.init_node("object_pose")

    global pub_point, marker_pub, clt_get_points
    rospy.wait_for_service('/vision/get_points_above_plane')
    clt_get_points = rospy.ServiceProxy('/vision/get_points_above_plane', PreprocessPointCloud)
    rospy.Service("/vision/obj_segmentation/get_obj_pose", RecognizeObject, callback_RecognizeObject) 
    pub_point = rospy.Publisher('/vision/detected_object', PointStamped, queue_size=10)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10) 

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
