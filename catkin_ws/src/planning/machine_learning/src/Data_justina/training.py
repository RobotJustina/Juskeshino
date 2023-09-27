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

rospack = rospkg.RosPack()
Redes_folder = rospack.get_path("machine_learning") + "/src"
sys.path.append(Redes_folder)

from numpy import genfromtxt
from Redes import architecture
from Redes import training_functions
import sklearn.metrics as metrics
#mse = metrics.mean_squared_error(y_test, y_pred)
#r2 = metrics.r2_score(y_test, y_pred)

class Red_conv(nn.Module):
	def __init__(self):
		f1=32   ##Mejor configuración f1 =32, l1=64, lr=8.1e-3, epoch=14
		l1=64
		expand=32
		super(Red_conv, self).__init__()
		self.conv1 = nn.Conv2d(1, f1, 3)
		self.dropout1 = nn.Dropout2d(p=0.5)
		self.norm1 = nn.GroupNorm(1, f1)

		self.c1 = nn.Linear(int(39*39*f1), l1) #27380
		self.norm3 = nn.LayerNorm(l1)
		self.dropout3 = nn.Dropout(p=0.5)

		self.c2 = nn.Linear(l1+expand, 8)

		self.lr = 8.1e-3
		self.epoch = 14

		self.extra = nn.Linear(2,expand)
		self.extra_norm = nn.LayerNorm(expand)

	def forward(self,x):
		pos=x[:, 6400:]
		pos= self.extra(pos)
		print(pos.shape)
		pos = nn.functional.relu(self.extra_norm(pos))
		x=x[:,0:6400]
		x = x.view(x.size(0),1,80,80)

		x = self.conv1(x)
		x = nn.functional.relu(self.norm1(x))
		x = nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
		x = self.dropout1(x)

		x = th.flatten(x,1)
		x = self.c1(x)
		x = nn.functional.relu(self.norm3(x))
		x = self.dropout3(x)

		x = th.cat((x, pos), 1)
		x = self.c2(x)
		return nn.functional.softmax(x,dim=1)

def main():
	rospy.init_node("training_justina")
	data_folder = rospack.get_path("machine_learning") + "/src/Data_justina"

	np.random.seed(42)
	th.manual_seed(42)
	#Read files
	C = genfromtxt(data_folder+'/Centroid.csv', delimiter=',')
	index= genfromtxt(data_folder+'/Index.csv', delimiter=',')
	#finding number of classes
	n_index=int(max(index))
	#in MATLAB index starts with 1
	index=index-1
	index=index.astype(int) ##Change index type
	#One hot matrix with labels
	n_class=len((index))
	M_one=np.eye(n_class)[index]
	M_one=M_one[:,:n_index] ##Taking the n_index numbers
	##Smoothed labels
	M_one[M_one==1]=0.985
	M_one[M_one==0]=0.001
	##Getting number of examples per class
	training_functions.examples_per_class(n_index, M_one)
	##Get matrix with data (grid+distance)
	data=training_functions.get_data(data_folder)
	data=data[:, :6402] ###this is different from training_gazebo
	##Permutation data
	perm=np.random.permutation(n_class)
	data_ent=data.copy()
	M_sal=M_one.copy()
	data_ent=data_ent[perm]
	M_sal=M_sal[perm]

	##Regresion changes
	data=training_functions.get_data(data_folder)
	M_sal=data[:, 6402:]
	data=data[:, :6402]
	data_ent=data_ent[perm]
	M_sal=M_sal[perm]

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

	#mired = Red_conv()
	mired = architecture.Reg()
	mired.to(disp)
	ecm = nn.MSELoss()
	opt = AdamW(mired.parameters(), lr = 1e-4) #4e-3

	hist = training_functions.entrena(mired, ecm, nn.functional.mse_loss, opt, entdl, valdl, n_epocas=4)

	training_functions.graficar(hist, entdl, valdl,"Red1")

	entdl = DataLoader(TensorDataset(x_ent, y_ent), batch_size=len(x_ent)//3, shuffle=False, drop_last=False)
	valdl = DataLoader(TensorDataset(x_vad, y_vad), batch_size=len(x_vad), shuffle=False, drop_last=False)
	prudl = DataLoader(TensorDataset(x_pru, y_pru), batch_size=len(x_pru), shuffle=False, drop_last=False)

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
