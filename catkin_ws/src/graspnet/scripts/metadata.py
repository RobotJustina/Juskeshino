#!/usr/bin/env python3

import numpy as np
import os
import pandas as pd
import rospy

DATASET_PATH = "/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset_test"

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
    print(stat, mmad)
    

def main():
    DF = get_dataset_frame(DATASET_PATH)
    #print(DF)
    get_stats(DF)

    get_stats(get_dataset_frame("/home/robocup/Juskeshino/catkin_ws/src/graspnet/dataset"))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass