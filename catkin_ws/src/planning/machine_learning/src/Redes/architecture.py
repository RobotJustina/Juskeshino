#! /usr/bin/env python3
import torch as th
from torch import nn

class Reg(nn.Module):
	def __init__(self):
		super(Reg, self).__init__()
		f1=32   ##Mejor configuración f1 =32, l1=64, lr=8.1e-3, epoch=14
		l1=64
		expand=32
		self.conv1 = nn.Conv2d(1, f1, 3)
		self.dropout1 = nn.Dropout2d(p=0.5)
		self.norm1 = nn.GroupNorm(1, f1)

		self.c1 = nn.Linear(int(39*39*f1), l1) #27380
		self.norm3 = nn.LayerNorm(l1)
		self.dropout3 = nn.Dropout(p=0.5)

		self.c2 = nn.Linear(l1+expand, 100)

		self.c3 = nn.Linear(100, 2)
		#self.c4 = nn.Linear(512, 2)
		self.lr = 8.1e-1
		self.epoch = 50

		self.extra = nn.Linear(2,expand)
		self.extra_norm = nn.LayerNorm(expand)

	def forward(self,x):
		pos=x[:, 6400:]
		pos= self.extra(pos)
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
		x = nn.functional.tanh(x)
		x = self.c3(x)
		#x = self.c4(x)
		return x

class Red_lin(nn.Module):
	def __init__(self):
		super(Red_lin, self).__init__()
		f1=32   ##Mejor configuración f1 =32, l1=64, lr=8.1e-3, epoch=14
		l1=64
		expand=32
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

class Red_conv(nn.Module):
	def __init__(self, salida):
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

		self.c2 = nn.Linear(l1+expand, salida)

		self.lr = 8.1e-3
		self.epoch = 14

		self.extra = nn.Linear(2,expand)
		self.extra_norm = nn.LayerNorm(expand)

	def forward(self,x):
		pos=x[:, 6400:]
		pos= self.extra(pos)
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

class Red_conv2(nn.Module):
	def __init__(self):
		f1=64
		f2=32
		l1=50
		super(Red_conv2, self).__init__()
		self.conv1 = nn.Conv2d(1, f1, 3)
		self.conv2 = nn.Conv2d(f1, f2, 3)
		self.dropout1 = nn.Dropout2d(p=0.5)
		self.dropout2 = nn.Dropout2d(p=0.5)
		self.norm1 = nn.GroupNorm(1, f1)
		self.norm2 = nn.GroupNorm(1, f2)
		self.c1 = nn.Linear(int(18*18*f2), l1) #27380
		self.norm3 = nn.LayerNorm(l1)
		self.dropout3 = nn.Dropout(p=0.5)
		self.c2 = nn.Linear(l1+2, 3)

	def forward(self,x):
		pos=x[:, 6400:]
		x=x[:,0:6400]
		x = x.view(x.size(0),1,80,80)

		x = self.conv1(x)
		x = nn.functional.relu(self.norm1(x))
		x = nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
		x = self.dropout1(x)

		x = self.conv2(x)
		x = nn.functional.relu(self.norm2(x))
		x = nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
		x = self.dropout2(x)

		x = th.flatten(x,1)
		x = self.c1(x)
		x = nn.functional.relu(self.norm3(x))
		x = self.dropout3(x)

		x = th.cat((x, pos), 1)
		x = self.c2(x)
		return nn.functional.softmax(x,dim=1)

class Red_conv3(nn.Module):
	def __init__(self):
		super(Red_conv3, self).__init__()
		self.conv1 = nn.Conv2d(1, 10, 3)
		self.conv2 = nn.Conv2d(10, 20, 3)
		self.dropout1 = nn.Dropout2d(p=0.5)
		self.dropout2 = nn.Dropout2d(p=0.5)
		self.norm1 = nn.GroupNorm(1, 10)
		self.norm2 = nn.GroupNorm(1, 20)
		self.c1 = nn.Linear(27380, 50)
		self.norm3 = nn.LayerNorm(50)
		self.dropout3 = nn.Dropout(p=0.5)
		self.c2 = nn.Linear(52, 3)

	def forward(self,x):
		pos=x[:, 6400:]
		x=x[:,0:6400]
		#print(x.shape[0])
		x = x.view(x.size(0),1,80,80)

		x = nn.functional.relu(self.conv1(x))
		x = self.dropout1(x)
		x = self.norm1(x)
		x = nn.functional.max_pool2d(x, 2)

		x = nn.functional.relu(self.conv2(x))
		x = self.dropout2(x)
		x = self.norm2(x)

		x = th.flatten(x,1)
		x = nn.functional.relu(self.c1(x))
		x = self.dropout3(x)
		x = self.norm3(x)

        #x = nn.functional.relu(self.avg(x))
        #x = th.flatten(x,1)
		x = th.cat((x, pos), 1)
		x = self.c2(x)
		return nn.functional.softmax(x,dim=1)

class Red1_drop_normal(nn.Module):
	def __init__(self, capa_1, capa_2, capa_3):
		super(Red1_drop_normal, self).__init__()
		self.c1 = nn.Linear(6402, capa_1) #Capa densa
		self.b1 = nn.LayerNorm(capa_1) #Capa de normalización
		self.d1 = nn.Dropout(p=0.5) #Capa de normalización
		self.c2 = nn.Linear(capa_1, capa_2) #Capa densa
		self.b2 = nn.LayerNorm(capa_2) #Capa de normalización
		self.d2 = nn.Dropout(p=0.5) #Capa de normalización
		self.c3 = nn.Linear(capa_2, capa_3) #Capa densa
		self.b3 = nn.LayerNorm(capa_3) #Capa de normalización
		self.d3 = nn.Dropout(p=0.5) #Capa de normalización
		self.salida = nn.Linear(capa_3, 3)

	def forward(self, x):
		x = nn.functional.relu(self.c1(x)) #Activaciones tanh
		x = self.b1(x)
		x = self.d1(x)
		x = nn.functional.relu(self.c2(x))
		x = self.b2(x)
		x = self.d2(x)
		x = nn.functional.relu(self.c3(x))
		x = self.b3(x)
		x = self.d3(x)
		x = self.salida(x)
		return nn.functional.softmax(x,dim=1)

class Red2(nn.Module):
	def __init__(self, capa_1, capa_2, capa_3, capa_4):
		super(Red2, self).__init__()
		self.c1 = nn.Linear(6402, capa_1) #Capa densa
		self.b1 = nn.LayerNorm(capa_1) #Capa de normalización
		self.d1 = nn.Dropout(p=0.5) #Capa de normalización
		self.c2 = nn.Linear(capa_1, capa_2) #Capa densa
		self.b2 = nn.LayerNorm(capa_2) #Capa de normalización
		self.d2 = nn.Dropout(p=0.5) #Capa de normalización
		self.c3 = nn.Linear(capa_2, capa_3) #Capa densa
		self.b3 = nn.LayerNorm(capa_3) #Capa de normalización
		self.d3 = nn.Dropout(p=0.5) #Capa de normalización
		self.c4 = nn.Linear(capa_3, capa_4) #Capa densa
		self.b4 = nn.LayerNorm(capa_4) #Capa de normalización
		self.d4 = nn.Dropout(p=0.5) #Capa de normalización
		self.salida = nn.Linear(capa_4, 3)

	def forward(self, x):
		x = nn.functional.relu(self.c1(x)) #Activaciones tanh
		x = self.b1(x)
		x = self.d1(x)
		x = nn.functional.relu(self.c2(x))
		x = self.b2(x)
		x = self.d2(x)
		x = nn.functional.tanh(self.c3(x))
		x = self.b3(x)
		x = self.d3(x)
		x = nn.functional.tanh(self.c4(x))
		x = self.b4(x)
		x = self.d4(x)
		x = self.salida(x)
		return nn.functional.softmax(x,dim=1)

class Red3(nn.Module):
	def __init__(self, capa_1, capa_2, capa_3, capa_4, capa_5):
		super(Red3, self).__init__()
		print("Se creo una NN")
		self.c1 = nn.Linear(6402, capa_1) #Capa densa
		self.b1 = nn.LayerNorm(capa_1) #Capa de normalización
		self.d1 = nn.Dropout(p=0.5) #Capa de normalización
		self.c2 = nn.Linear(capa_1, capa_2) #Capa densa
		self.b2 = nn.LayerNorm(capa_2) #Capa de normalización
		self.d2 = nn.Dropout(p=0.5) #Capa de normalización
		self.c3 = nn.Linear(capa_2, capa_3) #Capa densa
		self.b3 = nn.LayerNorm(capa_3) #Capa de normalización
		self.d3 = nn.Dropout(p=0.5) #Capa de normalización
		self.c4 = nn.Linear(capa_3, capa_4) #Capa densa
		self.b4 = nn.LayerNorm(capa_4) #Capa de normalización
		self.d4 = nn.Dropout(p=0.5) #Capa de normalización
		self.c5 = nn.Linear(capa_4, capa_5) #Capa densa
		self.b5 = nn.LayerNorm(capa_5) #Capa de normalización
		self.d5 = nn.Dropout(p=0.5) #Capa de normalización
		self.salida = nn.Linear(capa_5, 3)

	def forward(self, x):
		x = nn.functional.relu(self.c1(x)) #Activaciones tanh
		x = self.b1(x)
		x = self.d1(x)
		x = nn.functional.relu(self.c2(x))
		x = self.b2(x)
		x = self.d2(x)
		x = nn.functional.relu(self.c3(x))
		x = self.b3(x)
		x = self.d3(x)
		x = nn.functional.relu(self.c4(x))
		x = self.b4(x)
		x = self.d4(x)
		x = nn.functional.tanh(self.c5(x))
		x = self.b5(x)
		x = self.d5(x)
		x = self.salida(x)
		return nn.functional.softmax(x,dim=1)

class Red3_div(nn.Module):
	def __init__(self, capa_1, capa_2, capa_3, capa_4, capa_5):
		super(Red3_div, self).__init__()
		print("Se creo una Red3_div")
		self.c1 = nn.Linear(6400, capa_1) #Capa densa
		self.b1 = nn.LayerNorm(capa_1) #Capa de normalización
		self.d1 = nn.Dropout(p=0.5) #Capa de normalización
		self.c2 = nn.Linear(capa_1, capa_2) #Capa densa
		self.b2 = nn.LayerNorm(capa_2) #Capa de normalización
		self.d2 = nn.Dropout(p=0.5) #Capa de normalización
		self.c3 = nn.Linear(capa_2, capa_3) #Capa densa
		self.b3 = nn.LayerNorm(capa_3) #Capa de normalización
		self.d3 = nn.Dropout(p=0.5) #Capa de normalización

		self.c4 = nn.Linear(capa_3, capa_4) #Capa densa
		self.b4 = nn.LayerNorm(capa_4) #Capa de normalización
		self.d4 = nn.Dropout(p=0.5) #Capa de normalización

		self.c5 = nn.Linear(capa_4, capa_5) #Capa densa
		self.b5 = nn.LayerNorm(capa_5) #Capa de normalización
		self.d5 = nn.Dropout(p=0.5) #Capa de normalización

		self.salida = nn.Linear(capa_5+20, 3)

	def forward(self, x):
		pos=x[:, 6400:]
		x=x[:,0:6400]

		x = nn.functional.relu(self.c1(x)) #Activaciones tanh
		x = self.b1(x)
		x = self.d1(x)
		x = nn.functional.relu(self.c2(x))
		x = self.b2(x)
		x = self.d2(x)
		x = nn.functional.relu(self.c3(x))
		x = self.b3(x)
		x = self.d3(x)
		x = nn.functional.relu(self.c4(x))
		x = self.b4(x)
		x = self.d4(x)

		x = nn.functional.tanh(self.c5(x))
		x = self.b5(x)
		x = self.d5(x)

		x = th.cat((pos, pos, pos, pos, pos, x, pos, pos, pos, pos, pos), 1)

		x = self.salida(x)
		return nn.functional.softmax(x,dim=1)
