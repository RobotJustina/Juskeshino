#!/usr/bin/env python
import rospy
import rospkg
import sensor_msgs.msg
import cv2
import cv_bridge
import numpy as np

method = {1: "Kaze", 2: "Orb", 3: "Akaze"}


def simpleFeatureExtractorCallback(msg):
    method_selector = 3

    img_path = rospkg.RosPack().get_path('cv_shapes') + "/scripts/Data/ref1.jpg"
    img1 = cv2.imread(filename=img_path, flags=cv2.IMREAD_GRAYSCALE)
    #  Do something
    rospy.loginfo("callback test")
    img2 = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    out = compare_features(img1, img2, method[method_selector])
    cv2.imshow(method[method_selector], out)
    cv2.waitKey(1)


def compare_features(image1, image2, selector):

    descriptor = None
    if (selector == "Kaze"):
        # Initiate KAZE descriptor
        descriptor = cv2.AKAZE_create()
    elif (selector == "Orb"):
        # Initiate ORB descriptor
        descriptor = cv2.ORB_create()
    elif (selector == "Akaze"):
        # Initiate A-KAZE descriptor
        descriptor = cv2.AKAZE_create()
    else:
        descriptor = None

    keypoints1, descriptors1 = descriptor.detectAndCompute(image1, None)
    keypoints2, descriptors2 = descriptor.detectAndCompute(image2, None)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE,
                        trees=5)
    search_params = dict(checks=50)
    # Convert to float32
    descriptors1 = np.float32(descriptors1)
    descriptors2 = np.float32(descriptors2)
    # Create FLANN object
    FLANN = cv2.FlannBasedMatcher(
        indexParams=index_params, searchParams=search_params)
    # Matching descriptor vectors using FLANN Matcher
    matches = FLANN.knnMatch(queryDescriptors=descriptors1,
                             trainDescriptors=descriptors2, k=2)
    # Lowe's ratio test
    ratio_thresh = 0.7
    # "Good" matches
    good_matches = []
    # Filter matches
    for m, n in matches:
        if m.distance < ratio_thresh * n.distance:
            good_matches.append(m)

    # Draw only "good" matches
    output = cv2.drawMatches(img1=image1, keypoints1=keypoints1, img2=image2, keypoints2=keypoints2,
                             matches1to2=good_matches, outImg=None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    return output


def main():
    rospy.init_node('node_name')
    rospy.loginfo("Debug message")

    rospy.Subscriber("hsrb/head_center_camera/image_raw", sensor_msgs.msg.Image,
                     simpleFeatureExtractorCallback, queue_size=2)

    rate = rospy.Rate(2)  # Hz
    while not rospy.is_shutdown():
        message_str = "message at time: %s" % rospy.get_time()
        rospy.loginfo(message_str)

        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
