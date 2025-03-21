#!/usr/bin/env python3

import numpy as np
import os
import pandas as pd
import rospy
from scipy.spatial.distance import cdist, euclidean

DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset_cube_test/"

def get_dataset_frame(dataset_path):
    filelist = os.listdir(dataset_path)
    filelist.sort()
    filelist = np.array(filelist)
    data_frames = [pd.DataFrame(pd.read_pickle(os.path.join(dataset_path,file))['grasp'],index=['x','y','z','ox','oy','oz','w']) for file in filelist]
    dataset_frame = pd.concat(data_frames,axis=1).T

    
    return dataset_frame

def get_stats(df):
    mean = df.mean()
    min = df.min(axis=0)
    max = df.max(axis=0)
    median = df.median()
    mad = abs(df - mean).mean()
    MADN = 2.5*1.4826* abs(df-median).median()
    mmad = MADN.mean()
    std = df.std()

    stat = pd.concat([mean, std, min, max, median, mad, MADN], axis=1)
    stat = stat.rename({0:'mean',1:'std',2:'min',3:'max',4:'median',5:'mad',6:'MADN'},axis=1)
    print(stat)
    
def geometric_mean(df,eps):
    """
    Computes weighted geometric median
    :param X: the list of sample points, a 2D ndarray
    :param eps: acceptable error margin
    :return: first estimate meeting eps
    """
    X = df.to_numpy()
    #print(X)
    y = np.mean(X,0) # the geometric mean is a fare start
    while True:
        while np.any(cdist(X,[y])==0): # Euclidean distances, let's move away to avoid any null
            y +=0.1*np.ones(len(y))
        # set of weights that are the inverse of the distances from current estimate to the observations
        W = 1/cdist(X,[y]) # element-wise
        # new estimate is the weighted average of the observations
        y1 = np.sum(W*X,0)/np.sum(W) # sum along axis 0
        if euclidean(y,y1) < eps:
            print(len(y1),y1)
            return y1
        y = y1

def madgm(df, m):
    npdf = df.to_numpy()
    dist = [euclidean(sample,m) for sample in npdf]
    dist = pd.DataFrame(dist)
    print(dist.median())
    return dist.median()

def main():
    DF = get_dataset_frame(DATASET_PATH)
    #print(DF)
    get_stats(DF)
    gm = geometric_mean(DF,1e-6)
    print(madgm(DF,gm))
    get_stats(get_dataset_frame("/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset"))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass