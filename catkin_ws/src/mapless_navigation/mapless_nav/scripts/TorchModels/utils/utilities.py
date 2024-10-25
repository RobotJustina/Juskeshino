#! /usr/bin/env python3

import numpy as np
import pandas as pd

import glob


def load_data(files_path='/Datasets/'):
    files_names =  files_path+'/*.npz'
    files = glob.glob(files_names)
    print("Data.npz files found:", len(files))
    print("Loading data ...")
    data , temp = None, None

    count = 0
    for file in files:
        arr = np.load(file)
        temp = arr['data'][:,:]
        if(count==0):
            data=temp
            count+=1
        else:
            data=np.concatenate((data, temp), axis=0)

    return data