#!/usr/bin/env python3

import numpy as np
import sys
import cv2
from os import path

from cv_bridge import CvBridge
from rospkg import RosPack
import ros_numpy
from sensor_msgs.msg import Image , LaserScan , PointCloud2
from geometry_msgs.msg import TransformStamped,Point, Quaternion
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rospy
import tf2_ros 


#---
try:
    usr_url=path.expanduser( '~' )
    # CHANGE THIS PATH TO WHERE OPENPOSE DIR WAS INSTALLED,
    #       'openpose/build/python' <-- do not change it, just the first part
    sys.path.append(usr_url+'/openpose/build/python');
    from openpose import pyopenpose as op
    
except ImportError as e:
	print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
	raise e
#---
global rospack,bridge,net,tf_man


#---------------------------------------------------  
def init_openPose(n_people=-1,net_res="-1x208",model="BODY_25",heatmap=False):
    try:
        usr_url=path.expanduser( '~/' )
        params = dict()
		# OJO a este path, cambiarlo desde donde esté la carpeta openpose
        #print(usr_url+"openpose/models/")
        params["model_folder"] = usr_url+"openpose/models/"
        params["model_pose"] = model
        params["net_resolution"]= net_res
        # -1 -> toda persona detectable
        params["number_people_max"]= n_people
        if heatmap:
            params["heatmaps_add_parts"] = True
            params["heatmaps_add_bkg"] = True
            params["heatmaps_add_PAFs"] = True
            params["heatmaps_scale"] = 2
        
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()
        datum = op.Datum()

    except Exception as e:
        print("exception:",e)
        sys.exit(-1)
    return opWrapper,datum

  
#---------------------------------------------------
def human_detection(cloud_msg):

    points_data = ros_numpy.numpify(cloud_msg) 

    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    frame=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)

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

    return [False,False,False] if np.isnan(cent.any()) else cent

#------------------------------------------
def detectWaving(dataout):
    #print(dataout)
    # El primero que detecte lo retorna,
    # PRIMER CASO, BRAZO MUY LEVANTADO
    for i,sk in enumerate(dataout):
        if (sk[4,1] <= sk[0,1] and sk[4,1]!= 0)  or (sk[7,1] <= sk[0,1] and sk[7,1]!= 0):
            return i
    # SEGUNDO CASO, waving 'BAJO', mano(4,7) entre cabeza (0) y hombro (1)    
    for i,sk in enumerate(dataout):
        if (sk[4,1]!= 0 and sk[1,1]!=0 and sk[0,1]!=0 and (sk[4,1] <= sk[1,1] and sk[4,1] >= sk[0,1]))  \
        or (sk[7,1]!= 0 and sk[1,1]!=0 and sk[0,1]!=0 and (sk[7,1] <= sk[1,1] and sk[7,1] <= sk[0,1])):
            return i
        
    return False
   
#---------------------------------------------------
def waving_detection(cloud_msg,opWrapper,datum):
    cloud_data = ros_numpy.numpify(cloud_msg)
    frame=cloud_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h,w,_=frame.shape
    #dataout=np.zeros((25,2))
    datum.cvInputData = frame
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))
    if datum.poseKeypoints is None:
        
        print("No se encontro esqueleto/persona")
        return False,frame
    
    dataout=np.copy(datum.poseKeypoints[:,:,:2])

    for i in range(dataout.shape[0]):
        image_drawn=draw_skeleton(dataout[i],h,w,frame,bkground=True)
        draw_text_bkgn(image_drawn,text="Person "+str(i)+": ",pos=(int(dataout[i,0,0]), int(dataout[i,0,1])-40),
                        font_scale=1.3,text_color=(32, 255, 255))
        
    person_id = detectWaving(dataout)
    if not(person_id):return False,image_drawn

    head_mean = np.concatenate((dataout[person_id,0:1,:],dataout[person_id,15:19,:]),axis=0)
    head_mean = np.sum(head_mean,axis=0)/np. count_nonzero(head_mean,axis=0)[0] 
    

    head_xyz =[cloud_data['x'][int(head_mean[1]), int(head_mean[0])],
                cloud_data['y'][int(head_mean[1]), int(head_mean[0])],
                cloud_data['z'][int(head_mean[1]), int(head_mean[0])]]
    print("HEAD XYZ of waving person", head_xyz)
    return head_xyz,image_drawn

#---------------------------------------------------
def pointing_detection(cloud_msg,opWrapper,datum):
    dataout=np.zeros((25,2))
    cloud_data= ros_numpy.numpify(cloud_msg)
    frame=cloud_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
    frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h,w = frame.shape[:2]
    # se pasa imagen a openpose
    datum.cvInputData = frame
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))

    
    if datum.poseKeypoints is None:
        return False,False,frame # Pendiente de ver que retorna
    
    # si se obtuvo un resultado de openpose
    dataout=np.copy(datum.poseKeypoints[0,:,:2])
    # dibuja el resultado
    im_t=draw_skeleton(dataout,h,w,frame,cnt_person=0)


    # mano_levantada == 0 -> mano izquierda apuntando, mano_levantada == 1 -> mano derecha
    # en otro caso retorna False,False, False
    mano,codo,m_levantada = detect_pointing_arm(dataout,cloud_data)   
    if not(m_levantada):
        return False,False,im_t
    
    #tf_man.pub_static_tf(pos=codo,point_name='CODO',ref='head_rgbd_sensor_link')
    #tf_man.pub_static_tf(pos=mano,point_name='MANO',ref='head_rgbd_sensor_link')#print("cambiando referencia")
    #tf_man.change_ref_frame_tf(point_name='CODO',new_frame='map')
    #tf_man.change_ref_frame_tf(point_name='MANO',new_frame='map')
    #rospy.sleep(0.8)
    #manoM,_ = tf_man.getTF(target_frame='MANO',ref_frame='map')
    #codoM,_ = tf_man.getTF(target_frame='CODO',ref_frame='map')
    #ob_xyz = get_extrapolation(manoM,codoM)
    
    return mano, codo, im_t

    #return ob_xyz, m_levantada, im_t

#-----------------------------------------------------------------
def detect_pointing_OPENCV(points_msg):
    """points_data = ros_numpy.numpify(points_msg)    
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    image=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    
    print (image.shape)
    """
    pts= ros_numpy.numpify(points_msg)  
    
    image_data = pts['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]   
    frame=cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)

    #---
    
    inHeight = frame.shape[0]
    inWidth = frame.shape[1]
    # Prepare the frame to be fed to the network
    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)
    # Set the prepared object as the input blob of the network
    net.setInput(inpBlob)
    output = net.forward()
    thresh= 0.45
    poses=[]
    deb_imgr=frame[:,:,0]
    deb_imgg=frame[:,:,1]
    deb_imgb=frame[:,:,2]
    #res=Point_detectorResponse()
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
    #res.debug_image.append(bridge.cv2_to_imgmsg(deb_img_rgb))
    #right elbow       ####
    # right wrist      ####
    # left elbow
    # left wrist
    ##############################
    try:
            tt = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_link', rospy.Time())
                        
            #trans,rot=read_tf(tt)
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
        points_msg=ros_numpy.msgify(PointCloud2,pc_np_array,rospy.Time.now(),'head_rgbd_sensor_rgb_frame')
        cloud_out = do_transform_cloud(points_msg, tt)
        np_corrected=ros_numpy.numpify(cloud_out)
        corrected=np_corrected.reshape(pc_np_array.shape)
        print(corrected,'########################### Left', found_joints)
        #t=write_tf((corrected['x'][0],corrected['y'][0],corrected['z'][0]),(0,0,0,1),'right_elbow')
        #print (t)
        #b_st.sendTransform(t)
        #t=write_tf((corrected['x'][1],corrected['y'][1],corrected['z'][1]),(0,0,0,1),'right_wrist')
        #print (t)
        #b_st.sendTransform(t)
        elbow_xyz,wrist_xyz=[corrected['x'][0],corrected['y'][0],corrected['z'][0]],[corrected['x'][1],corrected['y'][1],corrected['z'][1]]
        v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
        #print("ELBOW RIGHT",elbow_xyz)
        #print("WRIST RIGHT",wrist_xyz)
        vd = [-(wrist_xyz[0] - elbow_xyz[0]), -(wrist_xyz[1]-elbow_xyz[1]),-1-(wrist_xyz[2]-elbow_xyz[2])]
        
        vectD = [wrist_xyz[0]-elbow_xyz[0],wrist_xyz[1]-elbow_xyz[1],wrist_xyz[2]-elbow_xyz[2]]
        alfa = -wrist_xyz[2]/vectD[2]
        y=wrist_xyz[1]+alfa*vectD[1]
        x=wrist_xyz[0]+alfa*vectD[0]
        
        #t= elbow_xyz[2]-   v[2]
        #x= elbow_xyz[0]+ t*v[0]
        #y= elbow_xyz[1]+ t*v[1]
        print(x,y,'x,y')
        #t=write_tf((x,y,0),(0,0,0,1),'pointing_right')
        #b_st.sendTransform(t)
        #res.x_r=x
        #res.y_r=y
        #res.z_r=0
        #
    

    vi=[0,0,0]
    if 'left_wrist'  in found_joints.keys() and 'left_elbow'  in found_joints.keys():
        pc_np_array = np.array([(found_joints['left_elbow'][0], found_joints['left_elbow'][1], found_joints['left_elbow'][2]),(found_joints['left_wrist'][0],found_joints['left_wrist'][1],found_joints['left_wrist'][2])]
         , dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        #pc_np_array = np.array([found_joints['right_elbow'], found_joints['right_wrist'], (0.0, 0.0, 0.0)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
        points_msg=ros_numpy.msgify(PointCloud2,pc_np_array,rospy.Time.now(),'head_rgbd_sensor_rgb_frame')
        cloud_out = do_transform_cloud(points_msg, tt)
        np_corrected=ros_numpy.numpify(cloud_out)
        corrected=np_corrected.reshape(pc_np_array.shape)
        #print(corrected,'###########################Right', found_joints)
        #t=write_tf((corrected['x'][0],corrected['y'][0],corrected['z'][0]),(0,0,0,1),'right_elbow')
        #print (t)
        #b_st.sendTransform(t)
        #t=write_tf((corrected['x'][1],corrected['y'][1],corrected['z'][1]),(0,0,0,1),'right_wrist')
        #print (t)
        #b_st.sendTransform(t)
        elbow_xyz,wrist_xyz=[corrected['x'][0],corrected['y'][0],corrected['z'][0]],[corrected['x'][1],corrected['y'][1],corrected['z'][1]]
        #print("ELBOW LEFT",elbow_xyz)
        #print("WRIST LEFT",wrist_xyz)
        v= np.asarray(wrist_xyz)-np.asarray(elbow_xyz)
        vi = [-(wrist_xyz[0] - elbow_xyz[0]), -(wrist_xyz[1]-elbow_xyz[1]),-1-(wrist_xyz[2]-elbow_xyz[2])]
        vectD = [wrist_xyz[0]-elbow_xyz[0],wrist_xyz[1]-elbow_xyz[1],wrist_xyz[2]-elbow_xyz[2]]
        alfa = -wrist_xyz[2]/vectD[2]
        y=wrist_xyz[1]+alfa*vectD[1]
        x=wrist_xyz[0]+alfa*vectD[0]
        
        #t= elbow_xyz[2]-   v[2]
        #x= elbow_xyz[0]+ t*v[0]
        #y= elbow_xyz[1]+ t*v[1]
        #print(x,y, v,'x,y')
        #t=write_tf((x,y,0),(0,0,0,1),'pointing_left')
        #b_st.sendTransform(t)
        #res.x_l=x
        #res.y_l=y
        #res.z_l=0
        
    #else:

        #res.x_l=0.0
        #res.y_l=0.0
        #res.z_l=0.0
        #t=write_tf((0,0,0),(0,0,0,1),'pointing_left')
        #b_st.sendTransform(t)

    
    if np.linalg.norm(vd)>np.linalg.norm(vi):
        print("Mano DERECHA levantada")
        #res.x_l = -1.0
        #res.y_l = -1.0
        #res.z_l = -1.0
        return [x,y,0],1

    else:
        print("Mano IZQUIERDA levantada")
        #res.x_r = -1.0
        #res.y_r = -1.0
        #res.z_r = -1.0
        return [x,y,0],1
        
    
#---------------------------------------------------
def handling_detection(cloud_msg,opWrapper,datum):
    return 0

#---------------------------------------------------
def recieving_detection(cloud_msg,opWrapper,datum):
    return 0
					  

#---------------------------------------------------
def draw_skeleton(joints,hh,wh,im,cnt_person=0,norm=False,bkground=True,centroid=False):

    """
    conections 15 (of 25) joints:
        0-1   <-> centro cabeza         - cuello-torso
        0-11  <-> centro cabeza         - ojo D ?
        0-12  <-> centro cabeza         - ojo I ?
        1-2   <-> cuello-torso          - hombro D
        1-5   <-> cuello-torso          - hombro I
        1-8   <-> cuello-torso          - tronco-cadera ombligo
        2-3   <-> hombro D              - codo D
        3-4   <-> codo D                - muñeca D
        5-6   <-> hombro I              - codo I
        6-7   <-> codo I                - muñeca I
        8-9   <-> tronco-cadera ombligo - tronco-cadera D 
        8-10  <-> tronco-cadera ombligo -tronco-cadera I 
        11-13 <-> ojo D ?               - oreja D
        12-14 <-> ojo I ?               - oreja I

    conections 14 (of 18) joints:

        0-1  <-> centro cabeza  - cuello-torso
        0-10 <-> centro cabeza  - ojo D
        0-11 <-> centro cabeza  - ojo I
        1-2  <-> cuello-torso   - hombro D
        1-5  <-> cuello-torso   - hombro I
        1-8  <-> cuello-torso   - tronco-cadera D
        1-9  <-> cuello-torso   - tronco-cadera I
        2-3  <-> hombro D       - codo D
        3-4  <-> codo D         - muneca D
        5-6  <-> hombro I       - codo I
        6-7  <-> codo I         - muneca I
        10-12<-> ojo D          - oreja D
        11-13<-> ojo I          - oreja I
 

    """
    h=1
    w=1
    lineThick=2
    circleSize=3

    if norm:
        h=hh
        w=wh

    if bkground:
        bkgn=im.astype(np.uint8)
    else:
        bkgn=np.zeros((hh,wh,3),np.uint8)
    
    if centroid:
        lnCnt=int(joints.shape[0]/2)
        frame=np.zeros((lnCnt,2))
        frame[:,0]=joints[:lnCnt]
        frame[:,1]=joints[lnCnt:]
        if frame.shape[0]==15:
            conections=[[0,1],[0,11],[0,12],[1,2],[1,5],[1,8],
                [2,3],[3,4],[5,6],[6,7],[8,9],[8,10],
                [11,13],[12,14]]
        else:
            conections=[[0,1],[0,14],[0,15],[1,2],[1,5],[1,8],
                [1,11],[2,3],[3,4],[5,6],[6,7],[8,9],
                [9,10],[11,12],[12,13],[14,16],[15,17]]

        
        for conect in conections:
            if frame[conect[0]][0]!=0 and frame[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(frame[conect[0]][0]*h),int(frame[conect[0]][1]*w)),(int(frame[conect[1]][0]*h),int(frame[conect[1]][1]*w)),(0,255,255),lineThick)
        for i in range(frame.shape[0]):
                    if frame[i][0]!=0.0 and frame[i][1]!=0.0:
                        bkgn=cv2.circle(bkgn,(int(frame[i][0]*h),int(frame[i][1]*w)),circleSize,(190,152,253),-1)

        return bkgn

    else:

        if joints.shape[0]==15:
            conections=[[0,1,0],[0,11,1],[0,12,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,10,11],
                        [11,13,12],[12,14,13]]
            # 0-1|0-11|0-12|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-10 
            # 11-13|12-14
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),   
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),   
                    (248,8,207),(248,8,76)]            

        elif joints.shape[0]==18:
            conections=[[0,1,0],[0,14,1],[0,15,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,11,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],
                        [9,10,11],[11,12,12],[12,13,13],[14,16,1],[15,17,1]]
            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        elif joints.shape[0]==25:
            conections=[[0,1,0],[0,15,1],[0,16,2],[1,2,3],[1,5,4],[1,8,5],
                        [2,3,6],[3,4,7],[5,6,8],[6,7,9],[8,9,10],[8,12,11],
                        [9,10,12],[10,11,13],[11,22,13],[11,24,13],[12,13,14],[13,14,15],
                        [14,19,15],[14,21,15],[15,17,16],[16,18,17],[19,20,15],[22,23,13]]
            # 0-1|0-15|0-16|1-2|1-5|1-8
            # 2-3|3-4|5-6|6-7|8-9|8-12 
            # 9-10|<10-11|11-22|11-24|22-23>|12-13|<13-14|14-19|14-21|19-20>|15-17|16-18
            colors=[(41,23,255),(99,1,249),(251,10,255),(10,75,255),(41,243,186),(10,10,255),
                    (25,136,253),(40,203,253),(0,218,143),(0,218,116),(78,218,0),(253,183,31),
                    (148,241,4),(239,255,1),(253,145,31),(253,80,31),(248,8,207),(248,8,76)]

        else:  #18 to less joints
            conections=[[0,1,0],[0,10,1],[0,11,2],[1,2,3],[1,5,4],[1,8,5],
                        [1,9,5],[2,3,6],[3,4,7],[5,6,8],[6,7,9],
                        [10,12,1],[11,13,1]]

            colors=[(253,45,31),(253,31,104),(253,31,184),(3,14,250),(15,104,252),(72,219,0),
                    (192,219,0),(18,170,255),(50,220,255),(50,255,152),(50,255,82),(113,219,0),
                    (167,251,77),(219,171,0),(219,113,0),(253,31,159),(159,31,253)]

        for i in range(joints.shape[0]):
            if joints[i][0]!=0.0 and joints[i][1]!=0.0:
                bkgn=cv2.circle(bkgn,(int(joints[i][0]*h),int(joints[i][1]*w)),circleSize,(255,255,255),-1)

        for conect in conections:
            if joints[conect[0]][0]!=0 and joints[conect[1]][1]!=0:
                bkgn=cv2.line(bkgn,(int(joints[conect[0]][0]*h),int(joints[conect[0]][1]*w)),(int(joints[conect[1]][0]*h),int(joints[conect[1]][1]*w)),colors[conect[2]],lineThick)
        
        draw_text_bkgn(bkgn,text="Person:"+str(cnt_person),pos=(int(joints[0,0]), int(joints[0,1])-40),
                   font_scale=1.3,text_color=(255, 255, 32))
        return bkgn
            
#---------------------------------------------------
# Para escribir texto en imagen con fondo negro
def draw_text_bkgn(img, text,
          font=cv2.FONT_HERSHEY_PLAIN,
          pos=(0, 0),
          font_scale=1,
          font_thickness=2,
          text_color=(0, 255, 0),
          text_color_bg=(0, 0, 0)
          ):

    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (x + text_w, y + text_h), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + int(font_scale) - 1), font, font_scale, text_color, font_thickness)

    return text_size

#---------------------------------------------------        
def detect_pointing_arm(lastSK,cld_points):
    area=10
    # CodoD -> 3, CodoI -> 6 
    codoD = [cld_points['x'][round(lastSK[3,1]), round(lastSK[3,0])],
            cld_points['y'][round(lastSK[3,1]), round(lastSK[3,0])],
            0]
    codoD[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[3,1])-area:round(lastSK[3,1])+area+1, 
                                                round(lastSK[3,0])-area:round(lastSK[3,0])+area+1]))
   
    codoI = [cld_points['x'][round(lastSK[6,1]), round(lastSK[6,0])],
            cld_points['y'][round(lastSK[6,1]), round(lastSK[6,0])],
            0]
    codoI[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[6,1])-area:round(lastSK[6,1])+area+1, 
                                                round(lastSK[6,0])-area:round(lastSK[6,0])+area+1]))
    
    # ManoD -> 4, ManoI -> 7
    manoD = [cld_points['x'][round(lastSK[4,1]), round(lastSK[4,0])],
            cld_points['y'][round(lastSK[4,1]), round(lastSK[4,0])],
            0]
    manoD[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[4,1])-area:round(lastSK[4,1])+area+1, 
                                                round(lastSK[4,0])-area:round(lastSK[4,0])+area+1]))
    
    manoI = [cld_points['x'][round(lastSK[7,1]), round(lastSK[7,0])],
            cld_points['y'][round(lastSK[7,1]), round(lastSK[7,0])],
            0]
    manoI[2]=np.nanmean(np.array(cld_points['z'][round(lastSK[7,1])-area:round(lastSK[7,1])+area+1, 
                                                round(lastSK[7,0])-area:round(lastSK[7,0])+area+1]))
   
    if ~np.isnan(manoD).any() or ~np.isnan(codoD).any() or ~np.isnan(manoI).any() or ~np.isnan(codoI).any():
        #tf_man.pub_tf(pos=codoD,point_name='codoD_t',ref='head_rgbd_sensor_link')
        #tf_man.pub_tf(pos=codoI,point_name='codoI_t',ref='head_rgbd_sensor_link')
        #tf_man.pub_tf(pos=manoD,point_name='manoD_t',ref='head_rgbd_sensor_link')
        #tf_man.pub_tf(pos=manoI,point_name='manoI_t',ref='head_rgbd_sensor_link')

        # resta entre [0,-k,0] y vectores de codo a mano 
        # k == min(manoD, manoI)
        k_min = abs(min(manoD[1],codoD[1]))
        print('K_MIN:',k_min)

        v_derecha=[-(manoD[0]-codoD[0]),-k_min-(manoD[1]-codoD[1]),-(manoD[2]-codoD[2])]
        v_izquierda=[-(manoI[0]-codoI[0]),-k_min-(manoI[1]-codoI[1]),-(manoI[2]-codoI[2])]
        
        #return ("Mano izquierda levantada") else  ("Mano derecha levantada")
        return manoI,codoI,0 if np.linalg.norm(v_derecha)>np.linalg.norm(v_izquierda) else manoD,codoD,1
        
    else:
        return False,False,False

#---------------------------------------------------
def get_extrapolation(mano,codo,z=0):

    vectD=[mano[0]-codo[0],mano[1]-codo[1],mano[2]-codo[2]]
    alfa=z-mano[2]/vectD[2]
    y=mano[1]+alfa*vectD[1]
    x=mano[0]+alfa*vectD[0]
    
    return [x,y,z]
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


def change_tf_reference():
    return 0

#---------------------------------------------------
"""
class TF_MANAGER:
    def __init__(self):
        self._tfbuff = tf2_ros.Buffer()
        self._lis = tf2_ros.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2_ros.StaticTransformBroadcaster()
        self._broad = tf2_ros.TransformBroadcaster()

    def _fillMsg(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation = Point(*pos)
        TS.transform.rotation = Quaternion(*rot)
        return TS

    def pub_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos=[0, 0, 0], rot=[0, 0, 0, 1], point_name='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name='', rotational=[0, 0, 0, 1], new_frame='map'):
        try:
            traf = self._tfbuff.lookup_transform(
                new_frame, point_name, rospy.Time(0))
            translation, _ = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos=translation, rot=rotational,
                               point_name=point_name, ref=new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(
                ref_frame, target_frame, rospy.Time(0), rospy.Duration(1.5))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False, False]

    def tf2_obj_2_arr(self, transf):
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
"""
#========================================================================
#tf_man = TF_MANAGER()
rospack = RosPack()
bridge = CvBridge()
tfBuffer = tf2_ros.Buffer()

# PARA USAR OPENCV (NO openpose)
usr_url = path.expanduser( '~' )
#protoFile = usr_url+"/openpose/models/pose/body_25/pose_deploy.prototxt"
#weightsFile = usr_url+"/openpose/models/pose/body_25/pose_iter_584000.caffemodel"
net = cv2.dnn.readNetFromCaffe(usr_url+"/openpose/models/pose/body_25/pose_deploy.prototxt",
                                usr_url+"/openpose/models/pose/body_25/pose_iter_584000.caffemodel")
