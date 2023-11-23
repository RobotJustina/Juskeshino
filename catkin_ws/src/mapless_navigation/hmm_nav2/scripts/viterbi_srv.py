#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
import numpy as np
from hmm_nav2.srv import ViterbiObservationState, ViterbiObservationStateResponse

hmm = None

class HMM():
    def __init__(self, A, B, PI):
        self.A = np.array(A)
        self.B = np.array(B)
        self.PI = np.array(PI)


def viterbi_callback(request):
    n_obs = len(request.observations)
    rospy.loginfo("request-> " + str(request))
    rospy.loginfo("obs count: %d", n_obs)
    
    # Viterbi algorithm
    delta = np.zeros((n_obs + 1, len(hmm.A)))
    phi = np.zeros((n_obs + 1, len(hmm.A))) + 666
    # print("phi")
    # print(phi)
    # print(phi.shape)
    path_estimate = np.zeros(n_obs + 1)

    delta[0,:] = hmm.PI * hmm.B[:, request.observations[0]]
    # print("delta")
    # print(delta)
    # print(delta.shape)

    for i in range(n_obs):
        for j in range(delta.shape[1]):
            delta[i+1, j] = np.max(delta[i] * hmm.A[:, j]) * hmm.B[j, request.observations[i]]
            phi[i+1, j] = np.argmax(delta[i] * hmm.A[:, j])

    path_estimate[n_obs] = int(np.argmax(delta[n_obs, :]))
    for i in np.arange(n_obs-1, 0, -1):
        path_estimate[i] = phi[i+1, int(path_estimate[i+1])]


    list_states = []
    for state in path_estimate:
        list_states.append(int(state))
    
    print(type(list_states))
    print(list_states)
    
    #return ViterbiObservationStateResponse([3, 2, 1])
    return ViterbiObservationStateResponse(list_states)



if __name__ == "__main__":
    rospy.init_node("viterbi_srv_node")
    path = rospkg.RosPack().get_path('create_dataset') + '/HMM/'

    try:
        hmm = HMM(np.load(path + 'A.npy'), np.load(path + 'B.npy'), np.load(path + 'PI.npy'))
        rospy.loginfo("HMM loaded")  
    except OSError as error:
        rospy.logerr('Error1 %s', error)
    
    rospy.loginfo("A.shape: %s", np.shape(hmm.A))
    rospy.loginfo("B.shape: %s", np.shape(hmm.B))
    rospy.loginfo("PI.shape: %s", np.shape(hmm.PI))

    viterbi_srv = rospy.Service("viterbi_hmm2", ViterbiObservationState, viterbi_callback)
    
    rospy.spin()
    
    