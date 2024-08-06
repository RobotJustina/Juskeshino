import numpy as np
import torch
import matplotlib.pyplot as plt

# Cargar los datos desde un archivo de NumPy
data = np.load('./DRL_files/r_episode.npy')

# Convertir los datos a un tensor de PyTorch
#tensor_data = torch.tensor(data, dtype=torch.float32)

# Graficar los datos
plt.figure(figsize=(8, 6))
plt.plot(data, label='Datos')
plt.title('Gráfico de los datos')
plt.xlabel('Índice')
plt.ylabel('Valor')
plt.legend()
plt.grid(True)
plt.show()
