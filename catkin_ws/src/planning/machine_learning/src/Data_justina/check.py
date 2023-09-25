#! /usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import glob
from datetime import datetime

#files=["./data_train2023-09-19_13:58:28.305694.npz"]
files=glob.glob('*.npz')
total_data=0

def print_path():
	global files, total_data
	plt.figure()
	for file in files:
		arr=np.load(file)
		print(file)
		print(arr['data'].shape)
		temp=arr['data'][:,6400:]
		plt.plot(temp[0:,0], temp[0:,1], label=file) ##Changed for the first files
		print(temp.shape)
		total_data+=len(temp)
	plt.ylabel('Distancia')
	plt.xlabel('Ángulo')
	plt.legend()

def print_speed():
	global files
	plt.figure()
	count=0
	for file in files:
		arr=np.load(file)
		temp=arr['data'][:,6402:] #Changed for first files
		plt.scatter(temp[:,0], temp[:,1], label=file)
		if(count==0):
			csv_arr=temp[:, :2]
			count+=1
		else:
			csv_arr=np.concatenate((csv_arr, temp[:, :2]), axis=0)
	plt.ylabel('Velocidad angular')
	plt.xlabel('Velocidad linear')
	plt.legend()
	return csv_arr

def main():
	#string=str(datetime.now())
	#string=string.replace(" ", "_")
	#print(string)
	#a=np.load("./data_head_1.npz")
	#temp=a['data'][:,6400:]
	#print(a['data'].shape)
	#print(temp.shape)
	global total_data
	print_path()
	cmd=print_speed()
	np.savetxt('Velocidades.csv', cmd, delimiter=',')
	#print(total_data)
	plt.show()

if __name__=="__main__":
	main()
