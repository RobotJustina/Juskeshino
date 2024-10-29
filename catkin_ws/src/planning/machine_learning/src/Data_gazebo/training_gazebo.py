#! /usr/bin/env python3


import numpy as np
import torch as th
import matplotlib.pyplot as plt
from torch import nn
from torch.optim import SGD, AdamW
from torch.utils.data import TensorDataset, DataLoader
from numpy import genfromtxt
import sys
import rospy
import rospkg
from sklearn.model_selection import train_test_split

import time

rospack = rospkg.RosPack()
Redes_folder = rospack.get_path("machine_learning") + "/src"
sys.path.append(Redes_folder)

rospy.init_node("training_gazebo")
np.random.seed(42)
th.manual_seed(42)
th.cuda.manual_seed(42)
th.backends.cudnn.deterministic = True
# th.backends.cudnn.benchmark = False
from Redes import training_functions
from Redes import architecture

def main():
    # Get centroids
    C = np.asarray([[0.3, 0.0], [0.0, 0.5], [0.0, -0.5]], dtype=np.float32)
    data_folder = rospack.get_path("machine_learning") + "/src/Data_gazebo"
    data = training_functions.get_data(data_folder)
    # temporary code
    index = training_functions.index_data(data, C)
    # define index data
    index = index.astype(int)
    data_ent = data[:, :6402]
    # finding number of classes
    n_index = int(np.max(index)+1)
    # One hot matrix with labels
    n_class = len(index)
    M_sal = training_functions.class_one_hot(index, n_index, n_class)
	
    # M_sal=data[:, 6402:]

    # Getting number of maximum examples per class
    # max_n=training_functions.examples_per_class(n_index, M_one)
    # Erase data if the examples for one class are more than the smallest one
    # data_ent, M_sal = training_functions.clean_data(data, M_one, index,max_n)

    x_ent, x_pru, y_ent, y_pru = train_test_split(
        data_ent, M_sal, test_size=0.3, shuffle=True, random_state=42)
    x_pru, x_vad, y_pru, y_vad = train_test_split(
        x_pru, y_pru, test_size=0.5, shuffle=True, random_state=42)
    
    print(type(x_ent))
    print(type(y_ent))

    disp = 'cuda' if th.cuda.is_available() else 'cpu'
    x_ent = th.tensor(x_ent, dtype=th.float32, device=disp)
    y_ent = th.tensor(y_ent, dtype=th.float32, device=disp)

    x_pru = th.tensor(x_pru, dtype=th.float32, device=disp)
    y_pru = th.tensor(y_pru, dtype=th.float32, device=disp)

    x_vad = th.tensor(x_vad, dtype=th.float32, device=disp)
    y_vad = th.tensor(y_vad, dtype=th.float32, device=disp)

    batch = 32
    learn_r = 4e-6
    epochs = 1
    print("batch:", batch)
    print("lr:", learn_r)
    print("epochs:", epochs)

    print(type(x_ent))
    print(type(y_ent))
    entdl = DataLoader(TensorDataset(x_ent, y_ent),
                       batch_size=batch, shuffle=True, drop_last=True)
    valdl = DataLoader(TensorDataset(x_vad, y_vad),
                       batch_size=batch, shuffle=False, drop_last=False)
    prudl = DataLoader(TensorDataset(x_pru, y_pru),
                       batch_size=batch, shuffle=False, drop_last=False)

    # Last fully connected

    # mired = architecture.CNN_A()  # Red_conv(3)
    # # mired = architecture.Reg()
    # mired.to(disp)
    # t0 = time.time()  # ><<<<<<<<<<<<
    # ecm = nn.MSELoss()
    # opt = AdamW(mired.parameters(), lr=learn_r)  # 4e-3, 40e-6
    # # opt = AdamW(mired.parameters(), lr = 4e-5)

    # # hist = training_functions.entrena(mired, ecm, nn.functional.mse_loss, opt, entdl, valdl, n_epocas=19)
    # hist = training_functions.entrena(
    #     mired, ecm, training_functions.exactitud, opt, entdl, valdl, n_epocas=epochs)  # 50
    # tf = time.time()  # ><<<<<<<<<<<<
    # print("time: ", tf - t0, "[s]")
    # training_functions.graficar(hist, entdl, valdl, "Red1")

    # # print("Prueba")
    # # training_functions.dataloader_r2(prudl,mired)
    # # print("validación")
    # # training_functions.dataloader_r2(valdl,mired)
    # # print("entrenamiento")
    # # training_functions.dataloader_r2(entdl,mired)

    # print("prueba")
    # training_functions.dataloader_eval(prudl, mired)
    # print("validación")
    # training_functions.dataloader_eval(valdl, mired)
    # print("entrenamiento")
    # entdl = DataLoader(TensorDataset(x_ent, y_ent),
    #                    batch_size=1, shuffle=False, drop_last=False)
    # training_functions.dataloader_eval(entdl, mired)

    # th.save(mired.state_dict(), data_folder+"/CNN_A.pth")
    # plt.show()


if __name__ == "__main__":
    main()
