#!/usr/bin/env python3
import math
import rospy
import numpy as np
import tf.transformations as tft
import tf2_ros
import ros_numpy
import cv2
import pandas as pd
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped,  Point, Pose, Vector3
from numpy.linalg import eig
from visualization_msgs.msg import Marker
from vision_msgs.srv import *
import geometry_msgs.msg


def segment_by_contour(msg):
    objects_on_stage = False
    pointCloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)  # dim 480 x 640, 
    rgb_array = pointCloud_array['rgba'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)   # Change the color space from BGR to HSV
    values=img_bgr.reshape((-1,3))  # (M,3=BGR), M = num of pixels
    values= np.float32(values)
    # Image color quantification with k-medias
    criteria= ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,100,0.1)    # centroid convergence criterion
    k=7     # num of colors in image
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS) 
    # convert into uint8, and make original image
    cc=np.uint8(cc) # Image with reduced colors, conversion to the format with which cv2 works
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(img_hsv.shape)
    # Change the color space from BGR to grayscale
    image_gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
    img_bin = cv2.adaptiveThreshold(image_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,15)
    kernel = np.ones((3,3),np.uint8)    # Define a kernel for erosion
    img_erode=cv2.erode(img_bin , kernel,iterations=4)  #4 eroded image
    img_dilate = cv2.dilate(img_erode , kernel, iterations=2) # apertura y cierre
    edged = cv2.Canny(img_dilate, 100, 20)
    #Finding contours in the image, each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object
    contours, hierarchy = cv2.findContours(edged.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    normas, point_cloud_contour_list, centroids_list, j = [], [] , [] , 0

    if len(contours) > 0: objects_on_stage = True
    
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < 1000 and area > 20000:    # discarding contours by area
            contours.pop(i)
       
    if objects_on_stage:
        for contour in contours:
            mask = np.zeros(img_bgr.shape,np.uint8)
            cv2.drawContours(mask, contours, j,(255, 255, 255), thickness = cv2.FILLED)  # llena contorno para mascara
            # the mask is eroded to ensure that we only get the point cloud of the object
            mask_er=cv2.erode(mask , kernel,iterations=8)   
            pixels_array, img_with_mask = contour_point_cloud(img_bgr, mask_er)
            X_array, Y_array, Z_array= [], [], []
            x_c, y_c, z_c = 0,0,0
            for pixel in pixels_array:  # extract xyz from each pixel 
                x, y, z = pointCloud_array[pixel[1],pixel[0]][0] , pointCloud_array[pixel[1],pixel[0]][1], pointCloud_array[pixel[1],pixel[0]][2]
                if abs(x) < 0.1 : continue
                x_c += pointCloud_array[pixel[1],pixel[0]][0]   # calculate the centroid of the point cloud
                y_c += pointCloud_array[pixel[1],pixel[0]][1]
                z_c += pointCloud_array[pixel[1],pixel[0]][2]
                X_array.append(x), Y_array.append(y), Z_array.append(z)
            if len(pixels_array) == 0:
                print("WARNING: NO POINT CLOUD")
            centroid_x ,  centroid_y, centroid_z = x_c/len(X_array), y_c/len(Y_array),z_c/len(Z_array)
            centroid = (np.asarray((centroid_x , centroid_y , centroid_z ))) 
            centroids_list.append(centroid)
            # calcula la distancia hacia cada contorno
            norma = math.sqrt(centroid_x**2 + centroid_y**2 + centroid_z**2)
            normas.append(norma)
            X_array, Y_array, Z_array = np.asarray(X_array), np.asarray(Y_array), np.asarray(Z_array)
            pc_contour=(np.asarray((X_array , Y_array , Z_array ))) 
            point_cloud_contour_list.append(pc_contour)

        index = normas.index(min(normas))   # envia la nube del objeto mas cercano
        return(objects_on_stage ,centroids_list[index].tolist() , point_cloud_contour_list[index][0] , point_cloud_contour_list[index][1] , point_cloud_contour_list[index][2])
    else:
        return(objects_on_stage ,0 ,0, 0, 0,0,0,0)   

    

def contour_point_cloud(img_bgr, mask_ero):  
    mask_ero = cv2.cvtColor(mask_ero, cv2.COLOR_BGR2GRAY)
    img_with_mask = cv2.bitwise_and(img_bgr,img_bgr, mask=mask_ero)
    img_non_zero = cv2.findNonZero(mask_ero)
    pixels_array= []
    for cord in img_non_zero:         
        pixels_array.append(cord[0])
    return (pixels_array, img_with_mask)



def pca(x_points, y_points, z_points):    # pc del contorno mas cercano
    point_cloud =  {"x": x_points, "y": y_points,"z": z_points}
    point_cloud = pd.DataFrame(pd.DataFrame(point_cloud), columns=["x","y","z"])
    eig_val, eig_vect = eig(point_cloud.cov())  # Eigenvalues and eigenvectors from Point Cloud Covariance Matrix
    # Ordering from largest to smallest eigenvalues
    eig_value_list = eig_val.tolist()
    ind_M = eig_value_list.index(max(eig_value_list))
    ind_m = eig_value_list.index(min(eig_value_list))
    ind_sec = 3 - ind_m - ind_M
    # object_size_estimation   MT = np.asarray(eig_vect)   ******************
    pts_frame_PCA = pd.DataFrame(point_cloud.values @ np.asarray(eig_vect) , columns=["x","y","z"])
    pts_frame_PCA = pts_frame_PCA.to_numpy()
    H = abs(  pts_frame_PCA[-1, 0] - pts_frame_PCA[0, 0])
    L = abs(  pts_frame_PCA[-1, 1] - pts_frame_PCA[0, 1])
    W = 2*abs(pts_frame_PCA[-1, 2] - pts_frame_PCA[0, 2])
    size_obj = Vector3()
    size_obj.x, size_obj.z , size_obj.y = H, W, L
    
    return [eig_vect[:,ind_M], eig_vect[:,ind_sec] , eig_vect[:,ind_m]], [eig_val[ind_M], eig_val[ind_sec], eig_val[ind_m]], size_obj



def object_pose(centroid, principal_component, second_component): 
    """
    Construction of the coordinate system of the object,the construction of the frame depends on the coordinate system of the robot gripper 
    """
    principal_component = np.asarray(principal_component)
    if principal_component[2] < 0: 
        principal_component = -1 * principal_component
    
    eje_y_obj = np.asarray(second_component) /np.linalg.norm( np.asarray(second_component) )
    eje_x_obj = principal_component /np.linalg.norm( principal_component )    # Eje principal en x
    eje_z_obj = np.cross(eje_y_obj , eje_x_obj ) / np.linalg.norm(np.cross(eje_y_obj , eje_x_obj))
    # se almacenan como puntos las coordenadas de eje x,y,...................................
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
    d = np.sqrt(q_obj[0]**2 + q_obj[1]**2 + q_obj[2]**2 + q_obj[3]**2)
    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = centroid[0], centroid[1], centroid[2]
    obj_pose.orientation.x = q_obj[0]/d
    obj_pose.orientation.y = q_obj[1]/d
    obj_pose.orientation.z = q_obj[2]/d
    obj_pose.orientation.w = q_obj[3]/d
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



def centroid_marker(xyz, frame_id): 
    global pub_point
    centroide = PointStamped()
    centroide.header.frame_id = frame_id
    centroide.point.x = xyz[0]
    centroide.point.y = xyz[1]
    centroide.point.z = xyz[2]
    frame_id = centroide.header.frame_id
    hdr = Header(frame_id=frame_id, stamp=rospy.Time(0))
    pub_point.publish(PointStamped(header=hdr, point=Point(x = centroide.point.x , y = centroide.point.y, z = centroide.point.z)))



def arow_marker(centroide_cam, p1, frame_id_point_cloud):  
    p0 = PointStamped()
    p0.header.frame_id = frame_id_point_cloud
    p0.point.x , p0.point.y, p0.point.z = centroide_cam[0], centroide_cam[1], centroide_cam[2]
    frame = frame_id_point_cloud

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
    point.x = p0.point.x + p1.x 
    point.y = p0.point.y + p1.y 
    point.z = p0.point.z + p1.z 
    marker.lifetime = rospy.Duration(10.0)
    marker.points = [p0.point, point]
    marker_pub.publish(marker)



def object_category(fpc, spc, thpc):  # estima la forma geometrica del objeto.
    c21, c31, c32 =  spc * ( 100 / fpc), thpc * ( 100 / fpc),thpc * ( 100 / spc)
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



def callback_RecognizeObject(req):  # Request is a PointCloud2
    global pca1, clt_get_points
    print("the service has been requested **************")
    req_ppc = PreprocessPointCloudRequest()
    req_ppc.input_cloud = req.point_cloud
    resp_clt_get_points = clt_get_points(req_ppc)
    msg = resp_clt_get_points.output_cloud
    obj_in_stage, centroide_cam, x_points, y_points, z_points= segment_by_contour(msg)
    frame_id_point_cloud = msg.header.frame_id
    resp = RecognizeObjectResponse()

    if obj_in_stage:
        print("An object was detected...............")
        print("object position" , centroide_cam)
        pca_vectors, eig_val, size_obj = pca(x_points, y_points, z_points)
        c_obj = object_category(eig_val[0], eig_val[1], eig_val[2])
        obj_pose, axis_x_obj = object_pose(centroide_cam, pca_vectors[0], pca_vectors[1])
        arow_marker( centroide_cam, axis_x_obj, 'base_link')
        broadcaster_frame_object("base_link", "object", obj_pose)
        print("size object", size_obj)
        # Rellenando msg
        resp.recog_object.category = c_obj
        resp.recog_object.header = req.point_cloud.header
        resp.recog_object.size = size_obj
        resp.recog_object.pose = obj_pose
        resp.recog_object.graspable = obj_in_stage
        return resp
    else:
        print("no object detected on stage")
        resp.recog_object.graspable = False
        return resp



def main():
    print("Node to segment objects in a image from camera...ʕ•ᴥ•ʔ")
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
