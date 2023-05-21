#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import tf.transformations as tft
import tf2_ros
import ros_numpy
import cv2
import pandas as pd
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, Vector3, Vector3Stamped
from numpy.linalg import eig
from visualization_msgs.msg import Marker
from vision_msgs.srv import *
import geometry_msgs.msg

ROBOTIC_ARM = 0.7

def segment_by_contour(msg):
    objects_on_stage = False
    pointCloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)  # dim 480 x 640, 
    rgb_array = pointCloud_array['rgb'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    img_bgr_copy = img_bgr
    #cv2.imshow('Imagen original', img_bgr)  #***************************************
    #cv2.waitKey(1)
    img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)   # Change the color space from BGR to HSV
    #cv2.imshow('Imagen hsv', img_hsv)   #*******************************************
    #cv2.waitKey(1)
    values=img_bgr.reshape((-1,3))  # (M,3=BGR), M = num of pixels
    values= np.float32(values)
    # Image color quantification with k-medias
    criteria= ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,100,0.1)    # centroid convergence criterion
    k=7     # num of colors in image
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS) 
    # convert into uint8, and make original image
    cc=np.uint8(cc) # Image with reduced colors, conversion to the format with which cv2 works
    #cv2.imshow('Imagen cuantizada', cc)
    #cv2.waitKey(1)
    segmented_image= cc[labels.flatten()]
    #cv2.imshow('Imagen aplanada', segmented_image)
    #cv2.waitKey(1)
    segmented_image=segmented_image.reshape(img_hsv.shape)
    #cv2.imshow('Imagen aplanada2', segmented_image)
    #cv2.waitKey(1)
    # Change the color space from BGR to grayscale
    image_gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('Imagen escala grises', image_gray )
    #cv2.waitKey(1)
    img_bin = cv2.adaptiveThreshold(image_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,15)
    kernel = np.ones((3,3),np.uint8)    # Define a kernel for erosion
    #cv2.imshow('Imagen binaria', img_bin)
    #cv2.waitKey(1)
    img_erode=cv2.erode(img_bin , kernel,iterations=4)  #4 eroded image
    #cv2.imshow('Imagen erosionada', img_erode)
    #cv2.waitKey(1) 
    img_dilate = cv2.dilate(img_erode , kernel, iterations=2) 
    #cv2.imshow('dilacion', img_dilate)
    #cv2.waitKey(1) 
    edged = cv2.Canny(img_dilate, 100, 20)
    #cv2.imshow('contours', edged)
    #cv2.waitKey(1) 
    #edged = img_dilate
    #Finding contours in the image, each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object
    contours, hierarchy = cv2.findContours(edged.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    cents , normas, new_contour_list, contour_index, temp = [], [], [], [], []
    desired_idx = np.nan
    #print("num contours", len(contours))
    if len(contours) < 1:
        print("no contours candidates")
        return(objects_on_stage ,0 ,0, 0, 0,0,0,0) 

    if len(contours) > 600:
        return(objects_on_stage ,0 ,0, 0, 0,0,0,0)  
    
    if len(contours) < 600:
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 1500 and area < 17000:    # discarding contours by area
                new_contour_list.append(contour)

        if len(new_contour_list) <= 0:
            print("no contours candidates")
            return(objects_on_stage ,0 ,0, 0, 0,0,0,0) 
        
        else:
            mask_contours_list = []
            X_array, Y_array, Z_array= [], [], []
            i = 0
        
            for i in range(len(new_contour_list)): 
                mask = np.zeros(img_bgr.shape,np.uint8)
                cv2.drawContours(mask, new_contour_list, i ,(255, 255, 255), thickness = cv2.FILLED)  # llena contorno para mascara
                # the mask is eroded to ensure that we only get the point cloud of the object
                mask_ero=cv2.erode(mask , kernel,iterations=4)   
                pixels_array, img_with_mask = contour_point_cloud(img_bgr, mask_ero)
                if len(pixels_array) <= 0:
                    continue
            
                x_c, y_c, z_c = 0,0,0
                for pixel in pixels_array:  # extract xyz from each pixel 
                    x, y, z = pointCloud_array[pixel[1],pixel[0]][0] , pointCloud_array[pixel[1],pixel[0]][1], pointCloud_array[pixel[1],pixel[0]][2]
                    x_c += pointCloud_array[pixel[1],pixel[0]][0]   # calculate the centroid of the point cloud
                    y_c += pointCloud_array[pixel[1],pixel[0]][1]
                    z_c += pointCloud_array[pixel[1],pixel[0]][2]
                    X_array.append(x), Y_array.append(y), Z_array.append(z)

                    centroid_x = x_c/len(pixels_array)
                    centroid_y = y_c/len(pixels_array)
                    centroid_z = z_c/len(pixels_array)
                    cloud_msg = cloud_obj(pixels_array, X_array, Y_array, Z_array,0)
                    norma = np.sqrt(centroid_x** + centroid_y** + centroid_z**)
                    if norma > ROBOTIC_ARM: # descarta contornos lejanos
                        continue

                    X_array, Y_array, Z_array = np.asarray(X_array), np.asarray(Y_array), np.asarray(Z_array)
                    # falta guardar contorno

                    #si hay contornos alcanzables actualiza bandera object_on_estage a True

        #retorna el contorno mas cercano

        cv2.imshow('Objeto', img_with_mask)
        cv2.waitKey(1)

        return(objects_on_stage ,[centroid_x, centroid_y, centroid_z] ,X_array, Y_array, Z_array, img_with_mask, cloud_msg)
        
    else:
        return(objects_on_stage ,0 ,0, 0, 0,0,0,0)   



def contour_point_cloud(img_bgr, mask_ero):  
    mask_ero = cv2.cvtColor(mask_ero, cv2.COLOR_BGR2GRAY)
    img_with_mask = cv2.bitwise_and(img_bgr,img_bgr, mask=mask_ero)
    img_non_zero = cv2.findNonZero(mask_ero)
    pixels_array= []

    for cord in img_non_zero:
        x,y = cord[0]          
        pixels_array.append(cord[0])

    return (pixels_array, img_with_mask)




def cloud_obj(pixels_array, x_points, y_points, z_points, rgb_array):
    lista_coord = []
    i = 0
    for i in range(len(pixels_array)):
        lista_coord.append(tuple( [x_points[i] , y_points[i], z_points[i], 0]))

    cloud_obj = np.array(lista_coord,
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb','f4')])
    
    stamp = rospy.Time.now()
    frame_id = 'realsense_link'
    cloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_obj, stamp, frame_id)

    return cloud_msg




def pca(x_points, y_points, z_points, frame_id, centroide_cam):    
    """
        Expects as argument one point cloud of the contoured object in the frame_id and return
        eigenvectors and eigenvalues.
    """
    point_cloud =  {"x": x_points, "y": y_points,"z": z_points}
    # sin estandar scaler
    point_cloud = pd.DataFrame(pd.DataFrame(point_cloud), columns=["x","y","z"])
    eig_val, eig_vect = eig(point_cloud.cov())  # Eigenvalues and eigenvectors from Point Cloud Covariance Matrix

    # Ordering from largest to smallest eigenvalues
    mayor, menor = np.amax(eig_val), np.amin(eig_val)
    eig_value_list = eig_val.tolist()
    ind_M = eig_value_list.index(mayor)
    ind_m = eig_value_list.index(menor)
    ind_sec = 3 - ind_m - ind_M

    #print("M,s,m", ind_M, ind_sec, ind_m)
    # each column of eig_vect corresponds to an eigenvector
    Eig_vect = [eig_vect[:,ind_M]* np.sqrt(eig_val[ind_M]), 
                eig_vect[:,ind_sec]*np.sqrt(eig_val[ind_sec]),  
                eig_vect[:,ind_m]*np.sqrt(eig_val[ind_m])]

    m = eig_vect[:,ind_m]
    M = eig_vect[:,ind_M]
    s = eig_vect[:,ind_sec]

    #MT = np.asarray([M, s, m])  # Matriz de transformacion con eigenvectores ordenados de mayor a menor
    MT = np.asarray(eig_vect) 
    # ESTIMACION DEL TAMANIO DEL OBJETO SEGMENTADO  object_size_estimation******************
    pts_frame_PCA = pd.DataFrame(point_cloud.values @ MT,#eig_vect.T, 
                                 columns=["x","y","z"])
    
    pts_frame_PCA = pts_frame_PCA.to_numpy()
    h_min, h_max = pts_frame_PCA[0, 0] , pts_frame_PCA[-1, 0] #H[0] , H[-1]
    l_min, l_max = pts_frame_PCA[0, 1] , pts_frame_PCA[-1, 1]
    w_min, w_max = pts_frame_PCA[0, 2] , pts_frame_PCA[-1, 2]
    H = abs( h_max - h_min )
    L = abs( l_min - l_max )
    W = 2*abs( w_max - w_min )
    # Approximate size of the object is H, L, W
    size_obj = Vector3()
    size_obj.x = H
    size_obj.y = W
    size_obj.z = L
    
    return [M, s, m], [eig_val[ind_M], eig_val[ind_sec], eig_val[ind_m]], size_obj



def object_pose(centroid, principle_axis, frame_id_camera):  # modificar a pointStamped cada punto
    """
        Construction of the coordinate system of the object
        All points is in the frame_id
    
        Pp = a random point on principle axis
        Ps = position of the frame in which the cloud points is obtained = (0,0,0) 

        Oc_y = Oc_Ps x Oc_Pp , ahora eje x sera el de la componente principal
        Oc_z = Oc_y x Oc_x 
    """
    global listener 
    principle_axis  = np.asarray(principle_axis)    # vector in frame "realsense_link"
    # transformacion de vector en frame camera a frame base_link*************************

    v_msg = Vector3Stamped()
    v_msg.header.frame_id = frame_id_camera   # Sistema coordenado de vector
    v_msg.header.stamp = rospy.Time(0) 
    v_msg.vector.x, v_msg.vector.y , v_msg.vector.z = principle_axis[0], principle_axis[1], principle_axis[2]
    target_frame = 'base_link'
    vector_principal_base_link = listener.transformVector3(target_frame, v_msg)

    # Cambiar punto de centroide a frame base_link*********************************************
    centroid = points_actual_to_points_target( centroid , frame_id_camera, 'base_link')
    principle_axis = np.asarray([vector_principal_base_link.vector.x , vector_principal_base_link.vector.y , vector_principal_base_link.vector.z ])
    
    if principle_axis[2] < 0: 
        principle_axis = -1 * principle_axis
    
    Oc_Pp  = np.asarray(principle_axis) 
    # axis x,y,z quedan en el frame base_link
    Oc_Ps = np.asarray([ centroid[0] , centroid[1] , centroid[2] ])

    Obj_y = np.cross(Oc_Ps , Oc_Pp)

    Obj_y = Obj_y /np.linalg.norm( np.cross(Oc_Ps , Oc_Pp ) )
    Obj_x = principle_axis /np.linalg.norm( principle_axis )    # Eje principal en x
    Obj_z = np.cross(Obj_y , Obj_x ) /np.linalg.norm( np.cross(Obj_x , Obj_y ) )

    axis_x_obj, axis_y_obj, axis_z_obj = Point(), Point(), Point()
    axis_x_obj.x, axis_x_obj.y, axis_x_obj.z = Obj_x[0], Obj_x[1], Obj_x[2]
    axis_y_obj.x, axis_y_obj.y, axis_y_obj.z = Obj_y[0], Obj_y[1], Obj_y[2]
    axis_z_obj.x, axis_z_obj.y, axis_z_obj.z = Obj_z[0], Obj_z[1], Obj_z[2]
    
    # con los vectores que representan a los ejes se forma la matriz de rotacion (columnas) y a partir de ella se obtienen
    # los cuaterniones necesarios para generar el frame del objeto
    comb = [[Obj_x, Obj_y , Obj_z],[Obj_y, Obj_x , Obj_z],[Obj_z, Obj_y , Obj_x],[Obj_x, Obj_z , Obj_y],[Obj_z, Obj_x , Obj_y]]
    RM = np.asarray( comb[3])
    RM = RM.T
    TM = [[RM[0,0], RM[0,1] , RM[0,2], 0],
         [RM[1,0], RM[1,1] , RM[1,2], 0],
         [RM[2,0], RM[2,1] , RM[2,2], 0], 
         [0, 0, 0, 1]]

    r,p,y = tft.euler_from_matrix(np.asarray(TM))
    q_obj = tft.quaternion_from_euler(r, p, y)
    same_tf = tft.is_same_transform(np.asarray(TM), np.asarray( tft.quaternion_matrix(q_obj)))
    print("same tf?", same_tf )

    d = np.sqrt(q_obj[0]**2 + q_obj[1]**2 + q_obj[2]**2 + q_obj[3]**2)

    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = centroid[0], centroid[1], centroid[2]
    obj_pose.orientation.x = q_obj[0]/d
    obj_pose.orientation.y = q_obj[1]/d
    obj_pose.orientation.z = q_obj[2]/d
    obj_pose.orientation.w = q_obj[3]/d
    return obj_pose , axis_x_obj, axis_y_obj, axis_z_obj , centroid



def broadcaster_frame_object(frame, child_frame, pose):
    """
        Emite la transformacion en el frame base_link, entre pose en frame base_link
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



def centroid_marker(centroide_cam, frame_id): 
    global pub_point
    centroide = PointStamped()
    centroide.header.frame_id = frame_id
    centroide.point.x = centroide_cam[0]
    centroide.point.y = centroide_cam[1]
    centroide.point.z = centroide_cam[2]
    frame_id = centroide.header.frame_id
    hdr = Header(frame_id=frame_id, stamp=rospy.Time(0))
    pub_point.publish(PointStamped(header=hdr, point=Point(x = centroide.point.x , y = centroide.point.y, z = centroide.point.z)))



def arow_marker(centroide_cam, p1 ,p2,p3, frame_id_point_cloud):  # P1 y P2
    #centroide
    p0 = PointStamped()
    p0.header.frame_id = frame_id_point_cloud
    p0.point.x , p0.point.y, p0.point.z = centroide_cam[0], centroide_cam[1], centroide_cam[2]
    frame = frame_id_point_cloud
    ns = ['axis_obj_x','axis_obj_y','axis_obj_z']
    points = [p0,p1,p2,p3]
    marker = Marker()
 
    for i in range(3):
        marker.header.frame_id = frame
        marker.type = Marker.ARROW
        marker.ns = ns[i]
        marker.header.stamp = rospy.Time.now()
        marker.action = marker.ADD
        marker.id = i#+10
        # body radius, Head radius, hight head
        marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.1, 0.04 
        marker.color.r , marker.color.g , marker.color.b, marker.color.a = 0, 100.0, 100.0, 1.0
        point = Point()
        point.x = points[0].point.x + (points[i+1].x) 
        point.y = points[0].point.y + (points[i+1].y)
        point.z = points[0].point.z + (points[i+1].z)

        marker.points = [points[0].point, point]
        marker_pub.publish(marker)



def object_category(fpc, spc, thpc):
    """
        esta funcion pide como argumento los eigenvalores obtenidos mendiante PCA y atraves de 
        la relacion entre ellos estima la forma geometrica del objeto y regresa una cadena.

        Establish similarity coefficients (sc) between the principal components
    """
    c21 =  spc * ( 100 / fpc)
    c31 = thpc * ( 100 / fpc)
    c32 = thpc * ( 100 / spc)

    # Examine the main components and decide how to grab the object
    if c21 >= 70:    # Means 1PC and 2PC are similar
        if c31 > 70:    # Means 2PC and 3PC are similar
            print("cube_or_sphere")
            return "1"

        if c32 < 40:    # Means 2PC is much bigger than 3PC
            print("box")
            return "2"

    elif c21 < 70:    # Means 1PC is much bigger than 2PC
        print("cylinder_or_prism")
        return "3"

    else:
        print("Could not determine the shape of the object...")
        return "0"



def frame_actual_to_frame_target(pos_in, f_actual, f_target):
    """
        Dado un frame y una pose de entrada, retorna una Pose en el frame objetivo
    """
    global pose_in, listener_base

    # Empaqueta msg, convierte orientacion de frame 'realsense_link' a frame 'base_link'
    poseStamped_msg = PoseStamped()  
    poseStamped_msg.header.frame_id = f_actual   # frame de origen
    poseStamped_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
    pose_in = pos_in
   
    #poseStamped_msg.pose = Pose()
    poseStamped_msg.pose = pose_in
    pose_frame_base_link = listener_base.transformPose(f_target, poseStamped_msg)
    new_pose = pose_frame_base_link.pose
    return new_pose



def points_actual_to_points_target(point_in, f_actual, f_target):
    """
        Dado un frame y una pose de entrada, retorna una Pose en el frame objetivo
    """
    global listener_base

    # Empaqueta msg, convierte orientacion de frame 'realsense_link' a frame 'base_link'
    point_msg = PointStamped()  
    point_msg.header.frame_id = f_actual   # frame de origen
    point_msg.header.stamp = rospy.Time(0)  # la ultima transformacion
   
    point_msg.point.x = point_in[0]
    point_msg.point.y = point_in[1]
    point_msg.point.z = point_in[2]
    point_target_frame = listener_base.transformPoint(f_target, point_msg)
    new_point = point_target_frame.point

    return [ new_point.x , new_point.y , new_point.z ]




def callback_RecognizeObject(req):  # Request is a PointCloud2
    global pca1
    msg = req.point_cloud
    print("the service has been requested **************")
    # centroide cam in 
    obj_in_stage, centroide_cam, x_points, y_points, z_points, img_with_mask, cloud_msg = segment_by_contour(msg)
    
    resp = RecognizeObjectResponse()

    if obj_in_stage:
        print("An object was detected")

        pca_vectors, eig_val, size_obj = pca(x_points, y_points, z_points, msg.header.frame_id, centroide_cam)
        c_obj = object_category(eig_val[0], eig_val[1], eig_val[2])
        
        obj_pose, axis_x_obj, axis_y_obj, axis_z_obj, centroid = object_pose(centroide_cam, pca_vectors[0], msg.header.frame_id)
        arow_marker( centroid, axis_x_obj, axis_y_obj, axis_z_obj, 'base_link')
        broadcaster_frame_object("base_link", "object", obj_pose)
        print("size object", size_obj)

        # Rellenando msg
        resp.recog_object.category = c_obj
        resp.recog_object.header = req.point_cloud.header
        resp.recog_object.point_cloud = cloud_msg
        resp.recog_object.size = size_obj
        resp.recog_object.pose = obj_pose  # envia la pose en el frame 'base_link'      
        resp.recog_object.image.data = img_with_mask.flatten().tolist()
        resp.recog_object.image.height = 480
        resp.recog_object.image.width = 640
        resp.recog_object.graspable = obj_in_stage
        #resp.image.data = recog_obj_img_list
        resp.image.height = 480
        resp.image.width = 640
        return resp

    else:
        print("no object detected on stage")
        resp.recog_object.graspable = False
        return resp



def main():
    print("Node to segment objects in a image from camera...ʕ•ᴥ•ʔ")
    rospy.init_node("object_pose")
    global pub_point, marker_pub, listener_base, listener, pca1
    
    rospy.Service("/vision/obj_reco/recognize_object", RecognizeObject, callback_RecognizeObject) 
    pub_point = rospy.Publisher('/vision/detected_object', PointStamped, queue_size=10)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 10) 
    listener_base = tf.TransformListener()
    listener = tf.TransformListener()

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():

        loop.sleep()

if __name__ == '__main__':
    main()
