#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from hmm_nav2.srv import ViterbiObservationState, ViterbiObservationStateRequest


if __name__ == "__main__":

    rospy.wait_for_service("viterbi_hmm2")

    try:
        viterbi_service = rospy.ServiceProxy("viterbi_hmm2", ViterbiObservationState)
        print("response ")
        print(viterbi_service([1, 2, 3, 12 ,4, 50, 30, 30, 16, 0, 2, 80]))
    except rospy.ServiceException as service_ex:
        rospy.logerr("Error %s", service_ex)


    
