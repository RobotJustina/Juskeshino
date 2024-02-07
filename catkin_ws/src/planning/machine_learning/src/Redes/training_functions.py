import glob
import numpy as np
import torch as th
import matplotlib.pyplot as plt
import sklearn.metrics as metrics

def index_data(data, C):
	index = data[:, 6402] ##Se eliminan datos con velociades negativas
	x,y=C.shape
	#print(x,y)
	for i in range(x):
		index[(data[:,6402]==C[i,0]) & (data[:,6403]==C[i,1])]=i
	return index

def class_one_hot(index, n_index, n_class):
	#One hot matrix with labels
	index=index.reshape(-1)
	print(index.shape)
	M_one=np.eye(n_class)[index]
	M_one=M_one[:,:n_index] ##Taking the n_index numbers
	##Smoothed labels
	M_one[M_one==1]=0.994
	M_one[M_one==0]=0.003
	return M_one

def graficar(hist, entdl, valdl, opt): ##Función para graficar y ahorrar líneas de código
	f,axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))
	axs[0].plot(hist['perdida_ent'] / len(entdl), label='Entrenamiento '+opt)
	axs[0].plot(hist['perdida_val'] / len(valdl), label='Validación '+opt)
	axs[0].set_xlabel('Época')
	axs[0].set_ylabel('Pérdida')
	axs[0].legend()

	axs[1].plot(hist['metrica_ent']/ len(entdl), label='Entrenamiento '+opt)
	axs[1].plot(hist['metrica_val']/ len(valdl), label='Validación '+opt)
	axs[1].set_xlabel('Época')
	axs[1].set_ylabel('Exactitud')
	axs[1].legend()

def get_data(folder):
	folder=folder+'/*.npz'
	#files=glob.glob('*.npz')
	files=glob.glob(folder)
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

def dataloader_r2(prudl,modelo):
	modelo.eval()
	count=0
	for (Xlote, ylote) in prudl:
		with th.no_grad():
			y_pred = modelo(Xlote)
			y_pred =y_pred.cpu().numpy()
			ylote = ylote.cpu().numpy()
			if(count==0):
				total_pred=y_pred
				total_lote=ylote
				count+=1
			else:
				total_pred=np.concatenate((total_pred, y_pred), axis=0)
				total_lote=np.concatenate((total_lote, ylote), axis=0)
			#print(ylote.shape)
	r2 = metrics.r2_score(total_lote, total_pred)
	mse = metrics.mean_squared_error(total_lote, total_pred)
	print(f'R2: {r2}, MSE: {mse}')

def entrena(modelo, fp, metrica, opt, entdl, valdl, n_epocas = 100):
	hist = {'perdida_ent': np.zeros(n_epocas, dtype = np.float32),
			'perdida_val': np.zeros(n_epocas, dtype = np.float32),
			'metrica_ent': np.zeros(n_epocas, dtype = np.float32),
			'metrica_val': np.zeros(n_epocas, dtype = np.float32)}
	for e in range(n_epocas):
		# bucle de entrenamiento
		modelo.train()
		for lote, (Xlote, ylote) in enumerate(entdl):
			perdida_paso, metrica_paso = paso_ent(modelo, fp, metrica, opt, Xlote, ylote)
			hist['perdida_ent'][e] += perdida_paso
			hist['metrica_ent'][e] += metrica_paso
		# bucle de evaluación
		modelo.eval()
		for (Xlote, ylote) in valdl:
			with th.no_grad():
				y_hat = modelo(Xlote)
				hist['perdida_val'][e] += fp(y_hat, ylote).cpu().numpy()
				hist['metrica_val'][e] += metrica(y_hat, ylote).cpu().numpy()

	return hist

def paso_ent(modelo, fp, metrica, opt, X, y):
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
	#return perdida_paso
	metricas_paso = metrica(y_hat, y)

	return perdida_paso, metricas_paso

def exactitud(y_hat, y):
	cmp = y_hat.argmax(dim=-1) == y.argmax(dim=-1)
	#cmp = y_hat.argmax(dim=-1) == y
	aciertos = th.count_nonzero(cmp)
	return aciertos / cmp.shape[0]

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
