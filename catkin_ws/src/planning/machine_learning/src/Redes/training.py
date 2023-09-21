import glob
import numpy as np
import torch as th
import matplotlib.pyplot as plt

def graficar(hist, entdl, valdl, opt): ##Función para graficar y ahorrar líneas de código
	plt.plot(hist['perdida_ent'] / len(entdl), label='Entrenamiento '+opt)
	plt.plot(hist['perdida_val'] / len(valdl), label='Validación '+opt)
	plt.xlabel('Época')
	plt.ylabel('Pérdida')
	plt.legend()
	#plt.show()

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
	#print(data.shape)
	return data

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
	p=corr/(corr+incorr)*100
	print(f'Porcentaje de respuestas correctas {p}%')

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

def clean_data(data_ent, M_sal, index, N):
	idx_max=np.max(index)
	print(f'El indice máximo es {idx_max}')
	index0=np.where(index==0)[0][:N]
	index1=np.where(index==1)[0][:N]
	idx=np.concatenate((index0, index1), axis=0)
	for i in range(2,idx_max+1):
        #print(i)
		temp=np.where(index==int(i))[0][:N]
		idx=np.concatenate((idx, temp), axis=0)
	perm=np.random.permutation(idx)
	return data_ent[perm], M_sal[perm]
