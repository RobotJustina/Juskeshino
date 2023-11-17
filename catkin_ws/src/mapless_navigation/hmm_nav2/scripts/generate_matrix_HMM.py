#!/usr/bin/env python
# -*- coding: utf-8 -*-
from sklearn.cluster import MiniBatchKMeans
import pandas as pd
import numpy as np
import rospkg
from general_utils import files_utils

if __name__ == '__main__':
    """Read the data"""
    path = rospkg.RosPack().get_path('create_dataset')
    print('Reading path:')
    print(path)
    print(path + '/Dataset/laser_odom_data.csv')
    data = pd.read_csv(path + '/Dataset/laser_odom_data.csv')
    print("shape: ", data.shape)
    print(data.head())
    path_created = files_utils.DirectoryUtils.replaceDir(
        path + '/HMM/', verbose=True)

    if not path_created:
        print("Error!")
        exit()

    n_lectures = data.shape[1]-3
    """Cleaning the data"""
    # Limit the values: clip(min, max)
    data.iloc[:, 1:n_lectures] = data.iloc[:, 1:n_lectures].clip(0, 10)
    print(data.describe())
    nans_count = data.isnull().sum().sum()

    if nans_count > 0:
        print('There are ', nans_count, 'nans')
        data.fillna(0, inplace=True)
    else:
        print("There are no nans")

    """Processing centroids"""
    K_corpus = 256  # Hyperparam: Number of symbols of observations
    n_states = 50   # Hyperparam: Number of states

    print('Calculating ccvk ...')  # Symbols
    km_corpus = MiniBatchKMeans(init='k-means++', n_clusters=K_corpus, batch_size=1000,
                                n_init=10, max_no_improvement=10, verbose=1)
    km_corpus.fit(data.iloc[:, 1:n_lectures])
    ccvk = km_corpus.cluster_centers_

    print('Calculating cc ...')  # States
    km_states = MiniBatchKMeans(init='k-means++', n_clusters=n_states, batch_size=1000,
                                n_init=10, max_no_improvement=10, verbose=1)
    km_states.fit(data[['Px', 'Py', 'e-th']])

    cc = km_states.cluster_centers_
    ccxyth = pd.DataFrame(cc)
    ccxyth['norm'] = np.linalg.norm(cc, axis=1)
    # Sort by proximity to origin
    cc = ccxyth.sort_values('norm').iloc[:, :3].values
    np.save(path + '/HMM/ccxyth.npy', cc)
    np.save(path + '/HMM/ccvk.npy', ccvk)

    """Calculate distances to centroids"""
    states = []  # Distance to centroids x,y,th,
    observations = []  # Distance to lecture
    for center, lec in zip(data[['Px', 'Py', 'e-th']].values, data.iloc[:, 1:n_lectures].values):
        states.append(np.argmin(np.linalg.norm(center - cc, axis=1)))
        observations.append(
            np.power(lec.T - ccvk, 2).sum(axis=1, keepdims=True).argmin())

    states = np.array(states)
    observations = np.array(observations)

    """Generate tuple(A, B, pi)"""
    mat_A = np.zeros((n_states, n_states))
    mat_B = np.zeros((n_states, K_corpus))
    mat_PI = np.ones(n_states)/n_states

    for i in range(n_states):
        from_to = []
        for state_i in np.where(states == i)[0]:
            if (int(state_i) == len(states) - 1):
                state_i = state_i - 1
            from_to.append(states[state_i + 1])
        # Set states from 0 to n-1
        from_to.append(0)  # initial state
        from_to.append(n_states - 1)  # final state
        from_to = np.bincount(from_to)
        # Discount initial and final occurrences
        from_to[0] = from_to[0] - 1
        from_to[n_states-1] = from_to[n_states-1] - 1
        mat_A[i, :] = from_to

    mat_A = mat_A/mat_A.sum(axis=1)
    for vk in range(K_corpus):
        index = np.where(observations == vk)[0]
        vk_state = states[index]
        vk_state = np.bincount(vk_state)

        if (n_states - len(vk_state) > 0):
            vk_state = np.append(vk_state, np.zeros(n_states - len(vk_state)))
        mat_B[:, vk] = (vk_state/np.bincount(states) + .00001)

    np.save(path + '/HMM/A.npy', mat_A)
    np.save(path + '/HMM/B.npy', mat_B)
    np.save(path + '/HMM/PI.npy', mat_PI)
    print("Model saved")
