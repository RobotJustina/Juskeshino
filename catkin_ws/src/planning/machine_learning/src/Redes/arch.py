#! /usr/bin/env python3
import torch as th
from torch import nn

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
