#!/usr/bin/env python3

import math
import rospy
import numpy as np
import tf
import tf.transformations as tft
import tf2_ros
import ros_numpy
import cv2
import pandas as pd
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, Vector3
from sklearn.preprocessing import StandardScaler
from numpy.linalg import eig
from visualization_msgs.msg import Marker
from vision_msgs.srv import *
import geometry_msgs.msg



"""
 node to recognize the closest object from the recognition of edges and contours, 
 in addition an approximation of the object's pose is obtained through principal 
 component analysis techniques.
"""

def segment_by_contour(msg):
    objects_on_stage = False
    pointCloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)  # dim 480 x 640, 
    rgb_array = pointCloud_array['rgb'].copy()     # Pass a copy of rgb float32, 480 x 640
    rgb_array.dtype = np.uint32       # Config data type of elements from array
    r,g,b = ((rgb_array >> 16) & 255), ((rgb_array >> 8) & 255), (rgb_array & 255)  # 480 x 640 c/u
    img_bgr = cv2.merge((np.asarray(b,dtype='uint8'),np.asarray(g,dtype='uint8'),np.asarray(r,dtype='uint8')))
    img_bgr_copy = img_bgr

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
    img_dilate = cv2.dilate(img_erode , kernel, iterations=3) 
    edged = cv2.Canny(img_dilate, 100, 20)
    #Finding contours in the image, each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object
    contours, hierarchy = cv2.findContours(edged.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    cents , normas, new_contour_list, contour_index, temp = [], [], [], [], []
    desired_idx = np.nan
    
    if len(contours) > 600:
        return(objects_on_stage ,0 ,0, 0, 0,0,0,0)  

    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 1500 and area < 16000:    # discarding contours by area
            x1,y1,w,h = cv2.boundingRect(contour) 
            xyz=[]

            for c in range (x1, (x1+w)):   # Go through the rectangle from minimum to maximum.
                for r in range(y1, (y1 + h)):  
                    aux=(np.asarray((pointCloud_array[r,c][0] , pointCloud_array[r,c][1], pointCloud_array[r,c][2]))) # Guarda 1 coordenada array 1x3
                    if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):'reject point'
                    else:
                        xyz.append(aux) 
            if np.size(xyz) < 1:
                continue
            xyz=np.asarray(xyz)       
            cent_rect = xyz.mean(axis=0) 
            cents.append(cent_rect) 
            # Calculation of the norm for the criterion of the centroid to choose
            norma =  math.sqrt(cent_rect[0]**2 + cent_rect[1]**2 + cent_rect[2]**2)    
            normas.append(norma)
            objects_on_stage = True
            new_contour_list.append(contour)
            contour_index.append(i)
            inf_contour = [contour, i, norma]
            temp.append(inf_contour)
       
    if objects_on_stage:
        cents, normas=np.asarray(cents), np.asarray(normas)
        min_dist = np.amin(normas)
        index = 0
        for element in normas:      # search for the nearest contour index
            if min_dist == element: 
                desired_idx = index
                break
            index += 1

        num_chosen_contour = temp[desired_idx][1]
        # it is examined if the chosen contour has a parent***************************************
        while hierarchy[0, num_chosen_contour ,3] > -1:     # as long as there is a father
            num_parent_contour = hierarchy[0, num_chosen_contour ,3]
            # it is examined if the parent contour belongs to the new list of contours
            if num_parent_contour in contour_index: 
                desired_idx = contour_index.index(num_parent_contour)
                num_chosen_contour = temp[desired_idx][1]
            
        cv2.drawContours(img_bgr_copy, new_contour_list, desired_idx,(200, 20, 233), 3)  # solo contorno en img original
        recog_obj_img = img_bgr_copy 
        mask = np.zeros(img_bgr.shape,np.uint8)
        cv2.drawContours(mask, new_contour_list, desired_idx,(255, 255, 255), thickness = cv2.FILLED)  # llena contorno para mascara

        # the mask is eroded to ensure that we only get the point cloud of the object
        mask_ero=cv2.erode(mask , kernel,iterations=4)   
        pixels_array, img_with_mask = contour_point_cloud(img_bgr, mask_ero)
        X_array, Y_array, Z_array= [], [], []
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
        X_array, Y_array, Z_array = np.asarray(X_array), np.asarray(Y_array), np.asarray(Z_array)

        return(objects_on_stage ,[centroid_x, centroid_y, centroid_z] ,X_array, Y_array, Z_array, recog_obj_img, img_with_mask, cloud_msg)
        
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




def pca(x_points, y_points, z_points, frame_id, centroide_cam):    # Expects as argument ONE point cloud of the contoured object separated in x,y,z
    point_cloud =  {"x": x_points, "y": y_points,"z": z_points}
    point_cloud = pd.DataFrame(StandardScaler().fit_transform(pd.DataFrame(point_cloud)), columns=["x","y","z"])
    eig_val, eig_vect = eig(point_cloud.cov())  # Eigenvalues and eigenvectors from Point Cloud Covariance Matrix

    # Ordering from largest to smallest eigenvalues
    mayor, menor = np.amax(eig_val), np.amin(eig_val)
    index, ind_M , ind_m = 0, 0, 0 

    for element in eig_val:
        if mayor == element: ind_M = index
        if menor == element: ind_m = index
        else: ind_sec = index 
        index = index+1

    # each column of eig_vect corresponds to an eigenvector
    Eig_vect1 = [ eig_vect[:,ind_sec] , eig_vect[:,ind_M] , eig_vect[:,ind_m]]

    Eig_vect = [eig_vect[:,ind_M]* np.sqrt(eig_val[ind_M]), eig_vect[:,ind_sec]*np.sqrt(eig_val[ind_sec]) ,  eig_vect[:,ind_m]*np.sqrt(eig_val[ind_m])]
    R = [[Eig_vect1[0][0], Eig_vect1[1][0], Eig_vect1[2][0], 0], [Eig_vect1[0][1], Eig_vect1[1][1], Eig_vect1[2][1], 0],[Eig_vect1[0][2], Eig_vect1[1][2], Eig_vect1[2][2], 0], [0, 0, 0, 1]]

    r,p,y = tft.euler_from_matrix(np.asarray(R))
    q1 = tft.quaternion_from_euler(r, p, y)
    pose_obj = object_pose(centroide_cam, q1)   

    point_1, point_2, point_3 = Point(), Point(), Point()
    array_points = [point_1, point_2, point_3]

    for i in range(3):
        array_points[i].x = Eig_vect[i][0]
        array_points[i].y = Eig_vect[i][1]
        array_points[i].z = Eig_vect[i][2]

    return ([array_points[0], array_points[1], array_points[2]], pose_obj, [eig_val[ind_M] , eig_val[ind_sec], eig_val[ind_m]])  # Regresa las componentes pricipales




def object_pose(centroid, PCA_q):  # Requests as arguments coordinates of the centroid and quaternions of the PCA frame    global tfBuffer
    # construct frame object with respect to realsense frame
    # normalize quaternions
    d = np.sqrt(PCA_q[0]**2 + PCA_q[1]**2 + PCA_q[2]**2 + PCA_q[3]**2)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "realsense_link"
    t.child_frame_id = "object" 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = centroid[0]
    t.transform.translation.y = centroid[1]
    t.transform.translation.z = centroid[2]
    
    t.transform.rotation.x = PCA_q[0]/d
    t.transform.rotation.y = PCA_q[1]/d
    t.transform.rotation.z = PCA_q[2]/d
    t.transform.rotation.w = PCA_q[3]/d
    
    br.sendTransform(t)

    obj_pose = Pose()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = centroid[0], centroid[1], centroid[2]
    obj_pose.orientation.x = PCA_q[0]/d
    obj_pose.orientation.y = PCA_q[1]/d
    obj_pose.orientation.z = PCA_q[2]/d
    obj_pose.orientation.w = PCA_q[3]/d

    return obj_pose




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




def arow_marker(centroide_cam, p1,p2,p3, frame_id_point_cloud):  # P1 y P2
    #centroide
    p0 = PointStamped()
    p0.header.frame_id = frame_id_point_cloud
    p0.point.x , p0.point.y, p0.point.z = centroide_cam[0], centroide_cam[1], centroide_cam[2]
    # Arrow
    frame = frame_id_point_cloud
    ns = ['orientation_obj_1','orientation_obj_2','orientation_obj_3']
    points = [p0,p1,p2,p3]

    #marker_1, marker_2, marker_3 = Marker(), Marker(), Marker()
    #marker_1.id = 0
    marker = Marker()
    #marker_array = [marker_1, marker_2, marker_3]

    #marker_pub_array = [marker_pub_1, marker_pub_2, marker_pub_3]

    for i in range(3):
        marker.header.frame_id = frame
        marker.type = Marker.ARROW
        marker.ns = ns[i]
        marker.header.stamp = rospy.Time.now()
        marker.action = marker.ADD
        marker.id = i+10

        # body radius, Head radius, hight head
        marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.1, 0.04 
        # Set the color
        marker.color.r , marker.color.g , marker.color.b, marker.color.a = 0, 100.0, 100.0, 1.0
        point = Point()
        point.x = points[0].point.x + (points[i+1].x) 
        point.y = points[0].point.y + (points[i+1].y)
        point.z = points[0].point.z + (points[i+1].z)

        marker.points = [points[0].point, point]
        marker_pub.publish(marker)




def callback_RecognizeObject(req):  # Request is a PointCloud2
    msg = req.point_cloud
    print("the service has been requested **************")
    obj_in_stage, centroide_cam, x_points, y_points, z_points, recog_obj_img, img_with_mask, cloud_msg = segment_by_contour(msg)
    frame_id_point_cloud = msg.header.frame_id
    resp = RecognizeObjectResponse()

    if obj_in_stage:
        print("An object was detected")

        pca_vectors, obj_pose, eig_val = pca(x_points, y_points, z_points, msg.header.frame_id, centroide_cam)
        arow_marker( centroide_cam, pca_vectors[0], pca_vectors[1], pca_vectors[2], frame_id_point_cloud)
        centroid_marker(centroide_cam, frame_id_point_cloud)

        resp.recog_object.header = req.point_cloud.header
        resp.recog_object.point_cloud = cloud_msg
        resp.recog_object.size.x, resp.recog_object.size.y, resp.recog_object.size.z = eig_val[0], eig_val[1], eig_val[2]
        resp.recog_object.pose = obj_pose
        print(obj_pose)
        resp.recog_object.image.data = img_with_mask.flatten().tolist()
        resp.recog_object.image.height = 480
        resp.recog_object.image.width = 640

        resp.recog_object.graspable = True
        #resp.image.data = recog_obj_img_list
        resp.image.height = 480
        resp.image.width = 640

        #cv2.imshow('final obj', img_with_mask)
        #cv2.waitKey(2)
        #cv2.imshow('detected object', recog_obj_img)
        #cv2.waitKey(2) 
        return resp

    else:
        print("no object detected on stage")
        resp.recog_object.graspable = False
        return resp



def main():
    print("Node to segment objects in a image from camera...ʕ•ᴥ•ʔ")
    rospy.init_node("object_pose")
    global pub_point, marker_pub, marker_pub_2, marker_pub_3
    
    rospy.Service("/vision/obj_reco/recognize_object", RecognizeObject, callback_RecognizeObject) 
    pub_point = rospy.Publisher('/vision/detected_object', PointStamped, queue_size=10)
    marker_pub = rospy.Publisher("/vision/object_recognition/markers", Marker, queue_size = 2) 

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():

        loop.sleep()

if __name__ == '__main__':
    main()
