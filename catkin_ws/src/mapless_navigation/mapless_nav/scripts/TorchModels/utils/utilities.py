#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
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