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

rospack = rospkg.RosPack()
Redes_folder = rospack.get_path("machine_learning") + "/src"
sys.path.append(Redes_folder)
#sys.path.append('/home/sergio/Juskeshino/catkin_ws/src/planning/machine_learning/src')
from Redes import architecture
from Redes import training

#def divide_data(data_ent, M_sal,porcentaje_entr, porcentaje_val, procentaje_pr):
#	x_ent=th.tensor(data_ent[:int(n_class*porcentaje_entr), :])
#	y_ent=th.tensor(M_sal[:int(n_class*porcentaje_entr), :])

def main():
	rospy.init_node("training_gazebo")
	np.random.seed(42)
	th.manual_seed(42)
	#Get centroids
	C=np.asarray([[0.3, 0.0],[0.0, 0.5],[0.0, -0.5]], dtype=np.float32)
	data_folder = rospack.get_path("machine_learning") + "/src/Data_gazebo"
	data=training.get_data(data_folder)
	#define index data
	index=data[:, 6402:]
	index=index.astype(int)
	data=data[:, :6402]
	#finding number of classes
	n_index=int(3)
	#One hot matrix with labels
	n_class=len((index))
	print(data.shape)
	index=index.reshape(-1)
	print(index.shape)
	M_one=np.eye(n_class)[index]
	M_one=M_one[:,:n_index] ##Taking the n_index numbers
	##Smoothed labels
	M_one[M_one==1]=0.994
	M_one[M_one==0]=0.003
	##Getting number of examples per class
	max_n=training.examples_per_class(n_index, M_one)
	##Permutation data
	perm=np.random.permutation(n_class)
	data_ent=data.copy()
	M_sal=M_one.copy()
	data_ent=data_ent[perm]
	M_sal=M_sal[perm]
	index=index[perm]
	##Erase data if the examples for one class are more than the smallest one
	data_ent, M_sal = training.clean_data(data_ent, M_sal, index,max_n)
	training.examples_per_class(n_index, M_sal)
	n_class=3*int(max_n)

	x_ent=th.tensor(data_ent[:n_class*7//10, :])
	y_ent=th.tensor(M_sal[:n_class*7//10, :])

	x_vad=th.tensor(data_ent[n_class*7//10:n_class*85//100, :])
	y_vad=th.tensor(M_sal[n_class*7//10:n_class*85//100, :])

	x_pru=th.tensor(data_ent[n_class*85//100:, :])
	y_pru=th.tensor(M_sal[n_class*85//100:, :])

	disp = 'cuda' if th.cuda.is_available() else 'cpu'
	x_ent=x_ent.to(th.device(disp), th.float32)
	y_ent=y_ent.to(th.device(disp), th.float32)

	x_pru=x_pru.to(th.device(disp), th.float32)
	y_pru=y_pru.to(th.device(disp), th.float32)

	x_vad=x_vad.to(th.device(disp), th.float32)
	y_vad=y_vad.to(th.device(disp), th.float32)

	entdl = DataLoader(TensorDataset(x_ent, y_ent), batch_size=8, shuffle=True, drop_last=True)
	valdl = DataLoader(TensorDataset(x_vad, y_vad), batch_size=1, shuffle=False, drop_last=False)
	prudl = DataLoader(TensorDataset(x_pru, y_pru), batch_size=1, shuffle=False, drop_last=False)

	#mired = Red1_drop_normal(200, 200, 200) ##Activación tanh, 3 capas ocultas
	#mired = Red2(200, 200, 200, 200)
	###Este modelo funciona bien
	#mired = Red3(300, 300, 200, 200, 100)

	##Last fully connected
	#mired = arch.Red3_div(300, 300, 200, 200, 100)

	mired = architecture.Red_conv()
	mired.to(disp)
	#ecm = nn.L1Loss()
	ecm = nn.MSELoss()
	opt = AdamW(mired.parameters(), lr = mired.lr) #4e-3, 40e-6
	#opt = AdamW(mired.parameters(), lr = 40e-6)

	#hist = training.entrena(mired, ecm, nn.functional.mse_loss, opt, entdl, valdl, n_epocas=5)
	hist = training.entrena(mired, ecm, nn.functional.mse_loss, opt, entdl, valdl, n_epocas=mired.epoch) #50
	#hist = entrena(mired, ecm, nn.functional.l1_loss, opt, entdl, valdl, n_epocas=4)

	training.graficar(hist, entdl, valdl,"Red1")

	print("prueba")
	training.dataloader_eval(prudl, mired)
	print("validación")
	training.dataloader_eval(valdl, mired)
	print("entrenamiento")
	entdl = DataLoader(TensorDataset(x_ent, y_ent), batch_size=1, shuffle=False, drop_last=False)
	training.dataloader_eval(entdl, mired)

	th.save(mired.state_dict(), data_folder+"/modelo_gazebo.pth")
	plt.show()

if __name__=="__main__":
	main()
