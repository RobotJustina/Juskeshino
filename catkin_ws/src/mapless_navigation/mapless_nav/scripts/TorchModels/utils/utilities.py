#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
#from PIL import Image
from sklearn.preprocessing import OneHotEncoder

import glob


def load_data(files_path='/data/'):
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


def load_data_matrix(files_path='/data/', shape=[100, 100]):
    files = glob.glob(files_path)
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
    
    print("Data shape:", data.shape)
    # --- prepare X(data), Y(labels) ---
    data_X = []
    data_Y = []
    count = 0
    for info in data:
        #print("image shape: ", info.shape)
        # print(info[-1, :])  # target (d, th)
        # print(info[-1, 2:4])  # DATA Y
        data_Y.append(np.copy(info[-1, 2:4]))
        info[-1, 2:4] = [0.0, 0.0]
        # print(data_Y[-1])
        data_X.append([info])  # img(80x80) ch0, vet(2) ch2
        
    data_X = np.asarray(data_X, dtype=np.float32)
    data_Y = np.asarray(data_Y, dtype=np.float32)

    return data_X, data_Y


def show_image_gray(img):   
    plt.imshow(img, cmap='gray') 
    plt.show()


def removePattern(np_arr, condition):
    index = np.where( condition ) [0]
    np_arr = np.delete(np_arr, index, axis=0)
    return np_arr


def one_hot_encode(data_Y, verbose=False):
    categories = np.unique(data_Y, axis=0, return_counts=True)
    if verbose:
        print("Categories:")
        print(categories)
    Y_labels = []
    for cat in categories[0]:
        Y_labels.append([str(cat)])

    # Creating the encoder
    enc = OneHotEncoder(handle_unknown='ignore')
    enc.fit(Y_labels)  # Fitting the encoder to the data
    y_one_hot = []
    if verbose:
        print("Processing data ...")
    for cat in data_Y:
        result = enc.transform([[str(cat)]]).toarray()[0]
        y_one_hot.append(result)
        
    if verbose:
        categories = np.unique(y_one_hot, axis=0, return_counts=True)
        print("OneHot encoding:")
        print(categories)

    return y_one_hot