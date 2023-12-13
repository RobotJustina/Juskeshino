Example  notebooks in calling the service, 
https://drive.google.com/drive/folders/1MUQPld1xVBBCDLR2Lff1W8ZQ30m4WnC6?usp=sharing

https://youtu.be/EFBdmaPLaz0


######################
Running FACE DETECTOR DEMO UBUNTU 20 ros noetic.... DLIB version 
( catkin_extras)

SOURCE YOUR CATKIN!!!!!!!!!!!!!!!!!!!

$ roslaunch hsrb_wrs_gazebo_launch takeshi_extras.launch

Moveit (with no gui should start, also, hmm nav (beta ) and face recog
IF a Map is provided , use navigation server (toyota nav)

publish a nav goal pose on the client, or use the arrow on rviz to test/use 
:D


If not there is an excelent navigation based on HMM’s ;) (beta)

publish a nav goal pose on the hmm_navclient, or use the PUBLISH POINTon rviz to test/use 
:D

MOVING ON  

For debugging face detector we will use a ros pkg called usb cam…
The extras launch already launched face recog server… 
you can launch it the same way
as the object detection in case is needed…..

CHECK README IN FACE RECOGNITION 
CATKIN FOR  WEIGHTS IMAGES ETC

Lets open jupyter notebook and go to ….
	How_to_use_face_recog_dlib_server 

















