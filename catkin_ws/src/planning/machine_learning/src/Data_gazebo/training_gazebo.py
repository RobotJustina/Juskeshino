#! /usr/bin/env python3

import numpy as np
import glob
import torch as th
import matplotlib.pyplot as plt
from torch import nn
from torch.optim import SGD, AdamW
from torch.utils.data import TensorDataset, DataLoader
from numpy import genfromtxt

##Activación tanh, 3 capas ocultas
##Se aplican capas de dropout y normalización
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

def paso_ent(modelo, fp, opt, X, y):
	opt.zero_grad() # se ponen los gradientes asociados a los parámetros
                  # a actualizaren en cero
	y_hat = modelo(X) # se propagan las entradas para obtener las predicciones
	perdida = fp(y_hat, y) # se calcula la pérdida
	perdida.backward() # se obtienen los gradientes
	opt.step() # se actualizan todos los parámetros del modelo

	with th.no_grad():
		perdida_paso = perdida.cpu().numpy() # convertimos la pérdida (instancia de
                                         # Tensor de orden 0) a NumPy, para
                                         # lo que es necesario moverla a CPU
	return perdida_paso

def entrena(modelo, fp, metrica, opt, entdl, valdl, n_epocas = 100):
	hist = {'perdida_ent': np.zeros(n_epocas, dtype = np.float32),
			'perdida_val': np.zeros(n_epocas, dtype = np.float32)}
	for e in range(n_epocas):
		# bucle de entrenamiento
		modelo.train()
		for lote, (Xlote, ylote) in enumerate(entdl):
			hist['perdida_ent'][e] += paso_ent(modelo, fp, opt, Xlote, ylote)

			# bucle de evaluación
		modelo.eval()
		for (Xlote, ylote) in valdl:
			with th.no_grad():
				y_pred = modelo(Xlote)
				hist['perdida_val'][e] += metrica(y_pred, ylote).cpu().numpy()

	return hist

def graficar(hist, entdl, valdl, opt): ##Función para graficar y ahorrar líneas de código
	plt.plot(hist['perdida_ent'] / len(entdl), label='Entrenamiento '+opt)
	plt.plot(hist['perdida_val'] / len(valdl), label='Validación '+opt)
	plt.xlabel('Época')
	plt.ylabel('Pérdida')
	plt.legend()
  	#plt.show()

def examples_per_class(n_index, M_one):
	number=np.argmax(M_one, axis=1)
	lista=[]
	for i in range(n_index):
		x=number[number==int(i)]
		lista.append(len(x))
	print(lista)
	min_lista=min(lista)
	print(f'El numero minimo de etiquetas en una clase es {min_lista}')
	return min_lista

def get_data():
	files=glob.glob('*.npz')
	count=0
	for file in files:
		arr=np.load(file)
		temp=arr['data'][:,:]
		if(count==0):
			data=temp
			count+=1
		else:
			data=np.concatenate((data, temp), axis=0)
	#data[:, 6400:6402]=data[:, 6402:6404]
	#data=data[:, :6402]
    #print(data.shape)
	return data

def clean_data(data_ent, M_sal, index, N):
	index0=np.where(index==0)[0][:N]
	index1=np.where(index==1)[0][:N]
	index2=np.where(index==2)[0][:N]
	idx=np.concatenate((index0, index1), axis=0)
	idx=np.concatenate((idx, index2), axis=0)
	perm=np.random.permutation(idx)
	return data_ent[perm], M_sal[perm]

def dataloader_eval(prudl,modelo):
	corr=0
	incorr=0

	modelo.eval()
	for (Xlote, ylote) in prudl:
		with th.no_grad():
			y_pred = modelo(Xlote)
			y_pred =y_pred.cpu().numpy()

			ylote = ylote.cpu().numpy()
			clase_real = np.argmax(ylote)
			clase_pred = np.argmax(y_pred)
			if(clase_real==clase_pred):
				corr+=1
			else:
				incorr+=1
            #print(f'La clase predicha es {clase_pred} y la clase real es {clase_real}')
	p=corr/(corr+incorr)*100
	print(f'Porcentaje de respuestas de prueba correctas {p}%')

def main():
	np.random.seed(42)
	th.manual_seed(42)
	#Get centroids
	C=np.asarray([[0.3, 0.0],[0.0, 0.5],[0.0, -0.5]], dtype=np.float32)
	data=get_data()
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
	max_n=examples_per_class(n_index, M_one)
	##Permutation data
	perm=np.random.permutation(n_class)
	data_ent=data.copy()
	M_sal=M_one.copy()
	data_ent=data_ent[perm]
	M_sal=M_sal[perm]
	index=index[perm]
	##Erase data if the examples for one class are more than the smallest one
	data_ent, M_sal = clean_data(data_ent, M_sal, index,max_n)
	examples_per_class(n_index, M_sal)
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
	mired = Red3(300, 300, 200, 200, 100)
	mired.to(disp)
	ecm = nn.MSELoss()
	opt = AdamW(mired.parameters(), lr = 100e-6) #4e-3

	hist = entrena(mired, ecm, nn.functional.mse_loss, opt, entdl, valdl, n_epocas=5)

	graficar(hist, entdl, valdl,"Red1")

	print("prueba")
	dataloader_eval(prudl, mired)
	print("validación")
	dataloader_eval(valdl, mired)
	print("entrenamiento")
	entdl = DataLoader(TensorDataset(x_ent, y_ent), batch_size=1, shuffle=False, drop_last=False)
	dataloader_eval(entdl, mired)

	th.save(mired.state_dict(), "./modelo_gazebo.pth")
	plt.show()

if __name__=="__main__":
	main()
