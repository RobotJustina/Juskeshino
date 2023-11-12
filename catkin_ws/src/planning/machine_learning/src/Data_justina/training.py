#! /usr/bin/env python3
import numpy as np
import glob
import torch as th
import matplotlib.pyplot as plt
from torch import nn
from torch.optim import SGD, AdamW
from torch.utils.data import TensorDataset, DataLoader
import sys
import rospy
import rospkg
import sklearn.metrics as metrics
from sklearn.model_selection import train_test_split
from numpy import genfromtxt

rospack = rospkg.RosPack()
Redes_folder = rospack.get_path("machine_learning") + "/src"
sys.path.append(Redes_folder)

from Redes import architecture
from Redes import training_functions

rospy.init_node("training_justina")
np.random.seed(42)
th.manual_seed(42)
th.cuda.manual_seed(42)
th.backends.cudnn.deterministic = True

def main():
	data_folder = rospack.get_path("machine_learning") + "/src/Data_justina"

	#Read files
	C = genfromtxt(data_folder+'/Centroid.csv', delimiter=',')
	index= genfromtxt(data_folder+'/Index.csv', delimiter=',')
	#finding number of classes
	n_index=int(max(index))
	n_class=len(index)
	#in MATLAB index starts with 1
	index=index-1
	index=index.astype(int) ##Change index type
	##Get matrix with data (grid+distance)
	data=training_functions.get_data(data_folder)
	data=data[:, :6402] ###this is different from training_gazebo in classification
	##Permutation data
	perm=np.random.permutation(n_class)
	##Regresion changes
	data=training_functions.get_data(data_folder)
	##Clear some data

	#data[(data[:,6402]==0) & (data[:,6403]==0), 6400]=0.1
	##Split data in M_sal and data
	M_sal=data[:, 6402:]
	data=data[:, :6402]

	#Split data x_ent=70%, x_pru=15%, x_vad=15%
	x_ent, x_pru, y_ent, y_pru = train_test_split(data, M_sal, test_size = 0.3, shuffle=True, random_state=42)
	x_pru, x_vad, y_pru, y_vad = train_test_split(x_pru, y_pru, test_size = 0.5, shuffle=True, random_state=42)

	disp = 'cuda' if th.cuda.is_available() else 'cpu'
	x_ent = th.tensor(x_ent, dtype=th.float32, device=disp)
	y_ent = th.tensor(y_ent, dtype=th.float32, device=disp)

	x_pru = th.tensor(x_pru, dtype=th.float32, device=disp)
	y_pru = th.tensor(y_pru, dtype=th.float32, device=disp)

	x_vad = th.tensor(x_vad, dtype=th.float32, device=disp)
	y_vad = th.tensor(y_vad, dtype=th.float32, device=disp)

	entdl = DataLoader(TensorDataset(x_ent, y_ent), batch_size=8, shuffle=True, drop_last=True)
	valdl = DataLoader(TensorDataset(x_vad, y_vad), batch_size=1, shuffle=False, drop_last=False)
	prudl = DataLoader(TensorDataset(x_pru, y_pru), batch_size=1, shuffle=False, drop_last=False)

	#mired = architecture.Red_conv(8)
	mired = architecture.Reg()
	mired.to(disp)
	ecm = nn.MSELoss()
	#ecm = nn.L1Loss()
	opt = AdamW(mired.parameters(), lr = 3e-5) #4e-3
	#plt.figure()
	#hist = training_functions.entrena(mired, ecm, nn.functional.mse_loss, opt, entdl, valdl, n_epocas=4)
	hist = training_functions.entrena(mired, ecm,training_functions.exactitud, opt, entdl, valdl, n_epocas=4)
	#hist = training_functions.entrena(mired, ecm, nn.functional.l1_loss, opt, entdl, valdl, n_epocas=18)
	training_functions.graficar(hist, entdl, valdl,"Red1")

	print("Prueba")
	training_functions.dataloader_r2(prudl,mired)
	print("validación")
	training_functions.dataloader_r2(valdl,mired)
	print("entrenamiento")
	training_functions.dataloader_r2(entdl,mired)
	#print("prueba")
	#training.dataloader_eval(prudl, mired)
	#print("validación")
	#training.dataloader_eval(valdl, mired)
	#print("entrenamiento")
	#entdl = DataLoader(TensorDataset(x_ent, y_ent), batch_size=1, shuffle=False, drop_last=False)
	#training.dataloader_eval(entdl, mired)

	th.save(mired.state_dict(), data_folder+"/modelo.pth")
	plt.show()

if __name__=="__main__":
	main()
