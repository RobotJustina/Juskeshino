#! /usr/bin/env python3
import tf
import cv2
import rospy  
import tf2_ros                                    
from human_detector.srv import Point_detector ,Point_detectorResponse 
from human_detector.srv import Human_detector ,Human_detectorResponse 

from cv_bridge import CvBridge
from object_classification.srv import *
import tf2_ros    
from segmentation.msg import *
import numpy as np
import ros_numpy
import os
from glob import glob
import matplotlib.pyplot as plt
import cv2 
from collections import Counter
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped, Pose
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#from utils.misc_utils import TF_MANAGER

#-----------------------------------------------------------------
global tf_listener, ptcld_lis, broadcaster , bridge , net , b_tf, b_st


rospy.init_node('human_pointing_detector') 
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
b_tf=tf2_ros.TransformBroadcaster()
b_st=tf2_ros.StaticTransformBroadcaster()
_tfbuff = tf2_ros.Buffer()

usr_url=os.path.expanduser( '~' )
protoFile = usr_url+"/openpose/models/pose/body_25/pose_deploy.prototxt"
weightsFile = usr_url+"/openpose/models/pose/body_25/pose_iter_584000.caffemodel"
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)

bridge=CvBridge()

#-----------------------------------------------------------------
def write_tf(pose, q, child_frame , parent_frame='map'):
    t= TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id =parent_frame
    t.child_frame_id =  child_frame
    t.transform.translation.x = pose[0]
    t.transform.translation.y = pose[1]
    t.transform.translation.z = pose[2]
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

#-----------------------------------------------------------------
def read_tf(t):
    pose=np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z
        ))
    quat=np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
        ))
    
    return pose, quat

#-----------------------------------------------------------------
def getTF(target_frame='', ref_frame='map'):
        try:
            tf = tfBuffer.lookup_transform(
                ref_frame, target_frame, rospy.Time(0), rospy.Duration(1.5))
            return tf2_obj_2_arr(tf)
        except:
            return [False, False]

#-----------------------------------------------------------------
def tf2_obj_2_arr(transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)

        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]

#-----------------------------------------------------------------
def change_ref_frame_tf(point_name='', rotational=[0, 0, 0, 1], new_frame='map'):
        try:
            traf = tfBuffer.lookup_transform(
                new_frame, point_name, rospy.Time(0))
            translation, _ = tf2_obj_2_arr(traf)
            print("TRANSLATION",translation)
            t=write_tf(pose=translation,q=rotational,child_frame=point_name,parent_frame=new_frame)
            b_st.sendTransform(t)
            return True
        except:
            return False

#-----------------------------------------------------------------
def probmap_to_3d_mean(points_data,probMap, thres_prob=0.3):
    ##Prob map to 3d point

    mask=np.where(probMap>thres_prob)
    npmask=np.asarray(mask).T

    npmask.shape
    xyz=[]
    if len (npmask)>1:
        for a in npmask:
            ix,iy=a[0],a[1]
            aux=(np.asarray((points_data['x'][ix,iy],points_data['y'][ix,iy],points_data['z'][ix,iy])))
            #print (aux)
            if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                    'reject point'
            else:
                xyz.append(aux)

    xyz=np.asarray(xyz)
    if (len(xyz)!=0):
        cent=xyz.mean(axis=0)
    else:
        cent=np.zeros(3)
    return cent

#-----------------------------------------------------------------
def detect_human(points_msg,dist = 6):
    points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]

    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()
    i = 0 #Face
    #i = 1# Neck
    probMap = output[0, i, :, :]
    probMap = cv2.resize(probMap, (inWidth, inHeight))
    cent= probmap_to_3d_mean(points_data,probMap)
    print (cent)
    if np.isnan(cent.any()):return Human_detectorResponse()
    print (cent)
    if np.isnan(cent.any()):cent=np.zeros(3)
    res=Human_detectorResponse()
    res.x= cent[0]
    res.y= cent[1]
    res.z= cent[2]

    return res    

#-----------------------------------------------------------------
def detect_pointing(points_msg,dist = 6):

    pts= ros_numpy.numpify(points_msg)  
    #---
    human, _ =getTF(target_frame='human',ref_frame='camera_rgb_optical_frame') 
    distToTF = np.linalg.norm(human) if human[0] else 2
    print("DISTANCIA AL HUMANO ",distToTF)

    # <<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><
    # HARDCODEADA LA DISTANCIA A 2 METROS (+ 0.3 m) 
    # NECESARIO MODIFICAR PARA PRUEBAS EN ROBOCUP DE ACUERDO A ESPACIOS Y 
    # DISTANCIAS DE LA PRUEBA 
    # SE IBA A UTILIZAR -> distToTF DE ACUERDO A LA TF DE LA PERSONA DETECTADA
    # PERO NO ES MUY EXACTO
    # <<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><
    image, masked_image = removeBackground(points_msg, dist)
    data = len(glob(os.path.join(os.path.expanduser( '~' )+"/Documentos","*"))) # cambiar a Documents si esta en ingles 
    cv2.imwrite(os.path.expanduser( '~' )+"/Documentos/maskedImage_"+str(data + 1)+".jpg",masked_image)
    
    #---
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(masked_image, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)
    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()
    thresh= 0.45
    poses=[]
    deb_imgr=image[:,:,0]
    deb_imgg=image[:,:,1]
    deb_imgb=image[:,:,2]
    res=Point_detectorResponse()
    for i in np.asarray((3,4,6,7)):
        probMap = output[0, i, :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        if len(np.where(probMap>=thresh)[0]) ==0: pose=[0,0,0]  
        else:pose=  [np.nanmean(pts['x'][np.where(probMap>=thresh)]),
                     np.nanmean(pts['y'][np.where(probMap>=thresh)]),
                     np.nanmean(pts['z'][np.where(probMap>=thresh)])]
        poses.append(pose)
        #deb_img= probMap+ deb_img*0.3  #DEBUG IMAGE
        deb_imgr[np.where(probMap>=thresh)]= 255
        deb_imgg[np.where(probMap>=thresh)]= 255
        deb_imgb[np.where(probMap>=thresh)]= 255
    deb_img_rgb=cv2.merge((deb_imgr, deb_imgg, deb_imgb))
    res.debug_image.append(bridge.cv2_to_imgmsg(deb_img_rgb))
    #right elbow       ####
    # right wrist      ####
    # left elbow
    # left wrist
    ##############################
    try:
            tt = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                        
            trans,rot=read_tf(tt)
            #print ("############head",trans,rot)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ( 'No head TF FOUND')

    found_joints={}
    for i, name in enumerate(['right_elbow','right_wrist','left_elbow','left_wrist']):
        if np.sum(np.asarray(poses[i]))==0:print(f'no {name} points')
        else:           
            print (poses[i],f' {name} pose wrt head')
            found_joints[name]=poses[i]
            
    print("FOUND JOINTS: ",found_joints)
    vd=[0,0,0]
    if 'right_wrist' in found_joints.keys() and 'right_elbow' in found_joints.keys():
        
        pc_np_array = np.array([(found_joints['right_elbow'][0], found_joints['right_elbow'][1], found_joints['right_elbow'][2]),
                                 (found_joints['right_wrist'][0],found_joints['right_wrist'][1],found_joints['right_wrist'][2])]
         , dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        #pc_np_array = np.array([found_joints['right_elbow'], found_joints['right_wrist'], (0.0, 0.0, 0.0)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_msg=ros_numpy.msgify(PointCloud2,pc_np_array,rospy.Time.now(),'camera_rgb_optical_frame')
        cloud_out = do_transform_cloud(points_msg, tt)
        np_corrected=ros_numpy.numpify(cloud_out)
        corrected=np_corrected.reshape(pc_np_array.shape)
        print(corrected,'########################### Left', found_joints)

        elbow_xyz,wrist_xyz=[corrected['x'][0],corrected['y'][0],corrected['z'][0]],[corrected['x'][1],corrected['y'][1],corrected['z'][1]]
        v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)

        vd = [-(wrist_xyz[0] - elbow_xyz[0]), -(wrist_xyz[1]-elbow_xyz[1]),-1-(wrist_xyz[2]-elbow_xyz[2])]
        
        vectD = [wrist_xyz[0]-elbow_xyz[0],wrist_xyz[1]-elbow_xyz[1],wrist_xyz[2]-elbow_xyz[2]]
        alfa = -wrist_xyz[2]/vectD[2]
        y=wrist_xyz[1]+alfa*vectD[1]
        x=wrist_xyz[0]+alfa*vectD[0]

        print(x,y,'x,y')
        t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
        b_st.sendTransform(t)
        res.x_r=x
        res.y_r=y
        res.z_r=0
        #
    else:
        res.x_r=0.0
        res.y_r=0.0
        res.z_r=0.0
        t=write_tf((0,0,0),(0,0,0,1),'pointing_right')
        b_st.sendTransform(t)

    vi=[0,0,0]
    if 'left_wrist'  in found_joints.keys() and 'left_elbow'  in found_joints.keys():
        pc_np_array = np.array([(found_joints['left_elbow'][0], found_joints['left_elbow'][1], found_joints['left_elbow'][2]),(found_joints['left_wrist'][0],found_joints['left_wrist'][1],found_joints['left_wrist'][2])]
         , dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        #pc_np_array = np.array([found_joints['right_elbow'], found_joints['right_wrist'], (0.0, 0.0, 0.0)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_msg=ros_numpy.msgify(PointCloud2,pc_np_array,rospy.Time.now(),'camera_rgb_optical_frame')
        cloud_out = do_transform_cloud(points_msg, tt)
        np_corrected=ros_numpy.numpify(cloud_out)
        corrected=np_corrected.reshape(pc_np_array.shape)
        print(corrected,'###########################Right', found_joints)

        elbow_xyz,wrist_xyz=[corrected['x'][0],corrected['y'][0],corrected['z'][0]],[corrected['x'][1],corrected['y'][1],corrected['z'][1]]

        v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
        vi = [-(wrist_xyz[0] - elbow_xyz[0]), -(wrist_xyz[1]-elbow_xyz[1]),-1-(wrist_xyz[2]-elbow_xyz[2])]
        vectD = [wrist_xyz[0]-elbow_xyz[0],wrist_xyz[1]-elbow_xyz[1],wrist_xyz[2]-elbow_xyz[2]]
        alfa = -wrist_xyz[2]/vectD[2]
        y=wrist_xyz[1]+alfa*vectD[1]
        x=wrist_xyz[0]+alfa*vectD[0]
        
        print(x,y, v,'x,y')
        t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
        b_st.sendTransform(t)
        res.x_l=x
        res.y_l=y
        res.z_l=0
        
    else:
        res.x_l=0.0
        res.y_l=0.0
        res.z_l=0.0
        t=write_tf((0,0,0),(0,0,0,1),'pointing_left')
        b_st.sendTransform(t)

    if np.linalg.norm(vd)>np.linalg.norm(vi):
        print("Mano DERECHA levantada")
        res.x_l = -1.0
        res.y_l = -1.0
        res.z_l = -1.0

    else:
        print("Mano IZQUIERDA levantada")
        res.x_r = -1.0
        res.y_r = -1.0
        res.z_r = -1.0
        
    
    return res
    #print (poses[0])
    #if np.sum(np.asarray(poses[0]))==0:print('no r.e.')
    #else:
    #    t=write_tf(poses[0],(0,0,0,1),'right_elbow','head_rgbd_sensor_rgb_frame') 
    #    b_tf.sendTransform(t)
    #    rospy.sleep(0.25)
    #    tt=tfBuffer.lookup_transform('map','right_elbow',rospy.Time(0))
    #    pose,quat= read_tf(tt)
    #    t=write_tf(pose,(0,0,0,1),'right_elbow')
    #    b_st.sendTransform(t)
    
    """rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'right_elbow')
                b_st.sendTransform(t)
                rospy.sleep(0.2)
                t=write_tf(poses[1],(0,0,0,1),'right_wrist','head_rgbd_sensor_rgb_frame') 
                b_tf.sendTransform(t)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','right_wrist',rospy.Time(0))
                rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'right_wrist')
                b_st.sendTransform(t)
            
                t=write_tf(poses[2],(0,0,0,1),'left_elbow','head_rgbd_sensor_rgb_frame') 
                b_tf.sendTransform(t)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','left_elbow',rospy.Time(0))
                rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'left_elbow')
                b_st.sendTransform(t)
            
                t=write_tf(poses[3],(0,0,0,1),'left_wrist','head_rgbd_sensor_rgb_frame') 
                b_tf.sendTransform(t)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','left_wrist',rospy.Time(0))
                rospy.sleep(0.2)
                pose,quat= read_tf(tt)
                t=write_tf(pose,(0,0,0,1),'left_wrist')
                b_st.sendTransform(t)
            
            
            
            
            
                tt=tfBuffer.lookup_transform('map','right_wrist',rospy.Time(0))
                wrist_xyz,_= read_tf(tt)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','right_elbow',rospy.Time(0))
                rospy.sleep(0.2)
                elbow_xyz,_= read_tf(tt)
                
                v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
                print (v,elbow_xyz)
                t= elbow_xyz[2]-   v[2]
                x= elbow_xyz[0]+ t*v[0]
                y= elbow_xyz[1]+ t*v[1]
                t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
                b_st.sendTransform(t)
                res.x_r=x
                res.y_r=y
                res.z_r=0
            
                tt=tfBuffer.lookup_transform('map','left_wrist',rospy.Time(0))
                wrist_xyz,_= read_tf(tt)
                rospy.sleep(0.2)
                tt=tfBuffer.lookup_transform('map','left_elbow',rospy.Time(0))
                rospy.sleep(0.2)
                elbow_xyz,_= read_tf(tt)
                
                v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
                print (v,elbow_xyz)
                t= elbow_xyz[2]-   v[2]
                x= elbow_xyz[0]+ t*v[0]
                y= elbow_xyz[1]+ t*v[1]
                t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
                b_st.sendTransform(t)
                res.x_l=x
                res.y_l=y
                res.z_l=0"""



    print (pose,quat)
    #tf_man.pub_static_tf(pos=poses[0],point_name='right_elbow', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='right_elbow')
    #tf_man.pub_static_tf(pos=poses[1],point_name='right_wrist', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='right_wrist')
    #tf_man.pub_static_tf(pos=poses[2],point_name='left_elbow', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='left_elbow')
    #tf_man.pub_static_tf(pos=poses[3],point_name='left_wrist', ref='head_rgbd_sensor_rgb_frame')
    #tf_man.change_ref_frame_tf(point_name='left_wrist')
    #wrist_xyz=tf_man.getTF(target_frame='right_wrist')
    #elbow_xyz=tf_man.getTF(target_frame='right_elbow')
    #res.x_r= x
    #res.y_r= y
    #res.z_r= 0
    #wrist_xyz=tf_man.getTF(target_frame='left_wrist')
    #elbow_xyz=tf_man.getTF(target_frame='left_elbow')
    #v= np.asarray(wrist_xyz[0])-np.asarray(elbow_xyz[0])
    #t=elbow_xyz[0][2]-v[2]
    #x= elbow_xyz[0][0]+ t*v[0]
    #y= elbow_xyz[0][1]+ t*v[1]
    #res.x_l= x
    #res.y_l= y
    #res.z_l= 0
    #tf_man.pub_static_tf(pos=[x,y,0],point_name='point_left')

    return res    

#-----------------------------------------------------------------
def detect_pointing2(points_msg,dist = 6):
    #tf_man = TF_MANAGER()
    res=Point_detectorResponse()
    points_data = ros_numpy.numpify(points_msg)

    image, masked_image = removeBackground(points_msg,distance = dist)
    data = len(glob(os.path.join(os.path.expanduser( '~' )+"/Documents","*"))) # cambiar a Documents si esta en ingles 
    cv2.imwrite(os.path.expanduser( '~' )+"/Documents/maskedImage_"+str(data + 1)+".jpg",masked_image)

    print (image.shape)
    frame=image
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(masked_image, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)
    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()
    try:
        poses = getconectionJoints(output,inHeight,inWidth)
        imageDraw = drawSkeletons(image,poses,plot=False)
        cv2.imwrite(os.path.expanduser( '~' )+"/Documents/maskedImageWithOP.jpg",imageDraw)
    
    except Exception as e:
        print("Ocurrio un error al construir el esqueleto",e,type(e).__name__)
        raise Exception("Ocurrio un error al construir el esqueleto ")


    res.debug_image.append(bridge.cv2_to_imgmsg(imageDraw))
    dists=[]
    for i,pose in enumerate(poses):
        if pose[0,0] != 0:
            print(pose[0,0],pose[0,1],pose[0])
            pose_xyz =[points_data['x'][int(pose[0,1]), int(pose[0,0])],
                       points_data['y'][int(pose[0,1]), int(pose[0,0])],
                       points_data['z'][int(pose[0,1]), int(pose[0,0])]]

            dists.append(np.linalg.norm(pose_xyz)) 
            t=write_tf((pose_xyz[0],pose_xyz[1],pose_xyz[2]),(0,0,0,1),'person_'+str(i),parent_frame='camera_rgb_optical_frame')
            b_st.sendTransform(t)
            rospy.sleep(0.3)
        elif pose[0,0] == 0 and pose[1,0] != 0:
            print(pose[1,0],pose[1,1])
            pose_xyz =[points_data['x'][int(pose[1,1]), int(pose[1,0])],
                       points_data['y'][int(pose[1,1]), int(pose[1,0])],
                       points_data['z'][int(pose[1,1]), int(pose[1,0])]]

            dists.append(np.linalg.norm(pose_xyz))  
            t=write_tf((pose_xyz[0],pose_xyz[1],pose_xyz[2]),(0,0,0,1),'person_'+str(i),parent_frame='camera_rgb_optical_frame')
            b_st.sendTransform(t)
            rospy.sleep(0.3)
        else:
            print("NO HAY DATOS PARA PUBLICAR")   
                    # PENDIENTE DE TERMINAR Y PROBAR
            #raise Exception("Error, datos en zero para TF")


    print(np.min(dists),np.argmin(dists))
    k=0
    if len(dists)>1:
        # DE TODAS LAS DISTANCIAS OBTENGO EL INDICE DE LA MAS PEQUEÑA
        k= np.argmin(dists)
    # PUBLICO CODOS Y MANOS DE LA PERSONA k Y OBTENGO COORDENADAS RESPECTO A MAPA    
    codoD =[points_data['x'][int(poses[k,3,1]), int(poses[k,3,0])],
            points_data['y'][int(poses[k,3,1]), int(poses[k,3,0])],
            points_data['z'][int(poses[k,3,1]), int(poses[k,3,0])]]
    codoI =[points_data['x'][int(poses[k,6,1]), int(poses[k,6,0])],
            points_data['y'][int(poses[k,6,1]), int(poses[k,6,0])],
            points_data['z'][int(poses[k,6,1]), int(poses[k,6,0])]]
    manoD =[points_data['x'][int(poses[k,4,1]), int(poses[k,4,0])],
            points_data['y'][int(poses[k,4,1]), int(poses[k,4,0])],
            points_data['z'][int(poses[k,4,1]), int(poses[k,4,0])]]
    manoI =[points_data['x'][int(poses[k,7,1]), int(poses[k,7,0])],
            points_data['y'][int(poses[k,7,1]), int(poses[k,7,0])],
            points_data['z'][int(poses[k,7,1]), int(poses[k,7,0])]]
            
    t=write_tf((codoD[0],codoD[1],codoD[2]),(0,0,0,1),'codoD',parent_frame='camera_rgb_optical_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.3)
    t=write_tf((manoD[0],manoD[1],manoD[2]),(0,0,0,1),'manoD',parent_frame='camera_rgb_optical_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.3)
    t=write_tf((codoI[0],codoI[1],codoI[2]),(0,0,0,1),'codoI',parent_frame='camera_rgb_optical_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.3)
    t=write_tf((manoI[0],manoI[1],manoI[2]),(0,0,0,1),'manoI',parent_frame='camera_rgb_optical_frame')
    b_st.sendTransform(t)
    rospy.sleep(0.7)
    change_ref_frame_tf(point_name='codoD')
    change_ref_frame_tf(point_name='codoI')
    change_ref_frame_tf(point_name='manoD')
    change_ref_frame_tf(point_name='manoI')
    codoD, _ =getTF(target_frame='codoD')
    codoI, _ =getTF(target_frame='codoI')
    manoD, _ =getTF(target_frame='manoD')
    manoI, _ =getTF(target_frame='manoI')
    ds=[manoD[2]-codoD[2],manoI[2]-codoI[2]]
    v1=[-(manoD[0]-codoD[0]),-(manoD[1]-codoD[1]),ds[np.argmin(ds)]-(manoD[2]-codoD[2])]
    v2=[-(manoI[0]-codoI[0]),-(manoI[1]-codoI[1]),ds[np.argmin(ds)]-(manoI[2]-codoI[2])]
    vectD = [manoD[0]-codoD[0],manoD[1]-codoD[1],manoD[2]-codoD[2]]
    alfa = -manoD[2]/vectD[2]
    y=manoD[1]+alfa*vectD[1]
    x=manoD[0]+alfa*vectD[0]
    t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
    b_st.sendTransform(t)
    res.x_r=x
    res.y_r=y
    res.z_r=0
    rospy.sleep(0.3)
    vectD = [manoI[0]-codoI[0],manoI[1]-codoI[1],manoI[2]-codoI[2]]
    alfa = -manoI[2]/vectD[2]
    y=manoD[1]+alfa*vectD[1]
    x=manoD[0]+alfa*vectD[0]
    t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
    b_st.sendTransform(t)
    res.x_l=x
    res.y_l=y
    res.z_l=0
    
    if np.linalg.norm(v1) > np.linalg.norm(v1):
        print("Mano DERECHA levantada")
        res.x_l = -1.0
        res.y_l = -1.0
        res.z_l = -1.0
        #
    else:
        print("Mano IZQUIERDA levantada")
        res.x_r = -1.0
        res.y_r = -1.0
        res.z_r = -1.0
    return res


# FUNCIONES PARA DETECTAR TODOS LOS KEYPOINTS
#--------------------------------------
def getKeypoints(output,inWidth, inHeight,numKeys=8):
    # se obtiene primero los keypoints 
    keypoints=[]
    for i in range (numKeys):
        probMap = output[0, i, :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        mapSmooth = cv2.GaussianBlur(probMap,(3,3),0,0)
        thresh =-256 if mapSmooth.max() < 0.1 else 256
        minthresh = mapSmooth.max()*thresh/2 if thresh == 256 else mapSmooth.min()*thresh/2
        if minthresh >15:
            _,mapMask = cv2.threshold(mapSmooth*thresh,minthresh-12,255,cv2.THRESH_BINARY)
        else:
            _,mapMask = cv2.threshold(mapSmooth*thresh,minthresh-1,255,cv2.THRESH_BINARY)
        contours,_ = cv2.findContours(np.uint8(mapMask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            blobMask = np.zeros(mapMask.shape)
            blobMask = cv2.fillConvexPoly(blobMask, cnt, 1)
            maskedProbMap = mapSmooth * blobMask
            _, maxVal, _, maxLoc = cv2.minMaxLoc(maskedProbMap)
            keypoints.append(maxLoc + (probMap[maxLoc[1], maxLoc[0]],) +(i,))
    return keypoints

#----------------------
def getGroups(output,conections):
    
    fullMAP=np.zeros((inHeight,inWidth),np.float32)
    for con in conections:
        probMap = output[0, con, :, :]
        probMap = cv2.resize(probMap, (inWidth, inHeight))
        mapSmooth = cv2.GaussianBlur(probMap,(3,3),0,0)
        thresh =-256 if mapSmooth.max() < 0.1 else 256
        minthresh = mapSmooth.max()*thresh/2 if thresh == 256 else mapSmooth.min()*thresh/2
        if minthresh >15:
            _,mapMask = cv2.threshold(mapSmooth*thresh,minthresh-12,255,cv2.THRESH_BINARY)
        else:
            _,mapMask = cv2.threshold(mapSmooth*thresh,minthresh-1,255,cv2.THRESH_BINARY)

        fullMAP += mapMask
    contours,_ = cv2.findContours(np.uint8(fullMAP), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    listOfGroups=[]
    for i in range(len(contours)):
        tmpo=np.zeros((inHeight,inWidth),np.float32)
        cv2.drawContours(tmpo, [contours[i]], -1, (255,255,0), thickness=cv2.FILLED)
        listOfGroups.append(np.transpose(np.nonzero(tmpo == 255)))
    return listOfGroups

#----------------------
# FUNCION PRINCIPAL
# version 2
def getconectionJoints(output,inHeight,inWidth,numKeyPoints = 8 ):
    conections = [57,40,43,45,48,51,53] #,27,32,34,37]   # SOLO se utilzan 7 conexiones por simplicidad
    # se obtiene primero los keypoints 
    
    keypoints = getKeypoints(output,inWidth, inHeight,numKeys = numKeyPoints)
    # cuento max personas encontradas
    conteo=Counter([i[-1] for i in keypoints])
    maxPeople = max(list(conteo.values()))
    avgPeople = round(sum(list(conteo.values()))/len(list(conteo.values())))
    if maxPeople > avgPeople :
        maxPeople = avgPeople
    
    groups = getGroups(output,conections)
    sk = np.zeros([len(groups),numKeyPoints,2])

    #print(maxPeople,len(groups))
    if maxPeople != len(groups):
        raise Exception("Distinto numero de KP que esqueletos detectados ")
    
    for k in keypoints:
        for i, group in enumerate(groups):
            if [True for item in group if (item == [k[1],k[0]]).all()]:     
                sk[i,k[3],0] = k[0]
                sk[i,k[3],1] = k[1]
    return sk

#----------------------
def drawSkeletons(frame,sk,plot=False):
    colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),
                    (148,241,4),(239,255,1),(253,145,31),(253,80,31),(248,8,207),(248,8,76)]
    conections = [[0,1],[1,2],[2,3],[3,4],[1,5],[5,6],[6,7]] #primero puede ser NO necesario
    
    rgbbkg = np.copy(frame)
    for s in sk:
        rgbbkg = cv2.circle(rgbbkg,(int(s[0][0]),int(s[0][1])),6,(255,255,0),-1)
        rgbbkg = cv2.circle(rgbbkg,(int(s[1][0]),int(s[1][1])),6,(255,255,0),-1)
        rgbbkg = cv2.line(rgbbkg,
                              (int(s[conections[0][0],0]),int(s[conections[0][0],1])),
                              (int(s[conections[0][1],0]),int(s[conections[0][1],1])),
                              colors[0],
                              2)
    for i in range(1,len(conections)):
        for s in sk:
            if int(s[conections[i][1],0]) != 0:
                rgbbkg = cv2.circle(rgbbkg,(int(s[conections[i][1],0]),int(s[conections[i][1],1])),6,(255,255,0),-1)
            if int(s[conections[i][0],0]) != 0 and int(s[conections[i][1],0]) != 0:
                rgbbkg = cv2.line(rgbbkg,
                              (int(s[conections[i][0],0]),int(s[conections[i][0],1])),
                              (int(s[conections[i][1],0]),int(s[conections[i][1],1])),
                              colors[i],
                              2)
    
    if plot:
        plt.imshow(rgbbkg)
    return rgbbkg

#--------------------------
# para quitar fondo, es necesario rgbd de la camara del robot
def removeBackground(points_msg,distance = 2):
    # Obtengo rgb
    points_data = ros_numpy.numpify(points_msg)
    image = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)    

    # Quito todos los pixeles que esten a una distancia mayor y/o a una distancia menor
    # Para poder obtener una mascara con ceros y unos
    zs_no_nans=np.where(~np.isnan(points_data['z']),points_data['z'],10)
    img_corrected = np.where((zs_no_nans < distance + 0.3),zs_no_nans,0)
    #img_corrected = np.where((img_corrected >1.5),img_corrected,0)
    img_corrected = np.where((img_corrected == 0),img_corrected,1)

    # operacion AND entre la imagen original y la mascara para quitar fondo (background)
    #img_corrected = img_corrected.astype(np.uint8)
    masked_image = cv2.bitwise_and(rgb_image, rgb_image, mask=img_corrected.astype(np.uint8))
    return rgb_image, masked_image