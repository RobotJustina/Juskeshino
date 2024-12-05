#!/usr/bin/env python3

import rospkg
import torch
import torchvision.transforms as transforms
from torch.utils.data import TensorDataset, DataLoader
from torch.optim import SGD, AdamW
from sklearn.model_selection import train_test_split
import numpy as np
import sys

import utils.utilities as l_util 
import utils.models as nn_models
from datetime import datetime
import time


"""
Enable GPU mode
"""
if torch.cuda.is_available():
    device = torch.device('cuda:0')
    print("device", device, "name: ", torch.cuda.get_device_name())
else:
    device = torch.device('cpu')


"""
Dataset
"""
np.set_printoptions(threshold=sys.maxsize)

# --- load files ---
pkg_name = 'mapless_nav'
pkg_path = rospkg.RosPack().get_path(pkg_name)
files_path = pkg_path + '/scripts/TorchModels/data/'
files_path += "data_dep1_*.npz"
data_X, data_Y = l_util.load_data_matrix(files_path, shape=[81, 80])  # 81x80



print("Data_X shape:", data_X.shape)
print("Data_Y shape:", data_Y.shape)
## print(data_X[0, 1, 0, :2])  # show vector
# show random sample
#l_util.show_image_gray(data_X[np.random.randint(0,len(data_X)), 0])

# CLASSIFICATION problem
y_one_hot = l_util.one_hot_encode(data_Y, verbose=True)
y_one_hot = np.asarray(y_one_hot, dtype=np.float32)

# Normalization
data_X[:,0] = data_X[:,0] /100.
data_X = np.array(data_X)
# Split data
X_train, X_val , y_train, y_val= train_test_split(data_X, y_one_hot, train_size=0.7, shuffle=True)
X_val, X_test, y_val, y_test= train_test_split(X_train, y_train, train_size=0.5, shuffle=True)

X_train = torch.tensor(X_train, dtype=torch.float32, device=device)
X_val = torch.tensor(X_val, dtype=torch.float32, device=device)
X_test = torch.tensor(X_test, dtype=torch.float32, device=device)
y_train = torch.tensor(y_train, dtype=torch.float32, device=device)
y_val = torch.tensor(y_val, dtype=torch.float32, device=device)
y_test = torch.tensor(y_test, dtype=torch.float32, device=device)


"""
Hyperparameters
"""
batch_size = 16
learn_r = 4e-6 # 1e-3
epochs = 2


"""
Model
"""
model = nn_models.CNN_B()
model.to(device)

optimizer = AdamW(model.parameters(), lr=learn_r)
loss_fn = torch.nn.MSELoss() # CrossEntropyLoss()

"""
Training
"""

train_loader = DataLoader(TensorDataset(X_train, y_train), batch_size=batch_size, shuffle=True)
valid_loader = DataLoader(TensorDataset(X_val, y_val), batch_size=batch_size, shuffle=True)
test_loader = DataLoader(TensorDataset(X_test, y_test), batch_size=batch_size, shuffle=True)

# Initializing in a separate cell so we can easily add more epochs to the same run
timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

t0 = time.time()
best_vloss = 1_000_000.
print("\nTraining model")
for epoch in range(epochs):
    print(f'Epoch: {epoch+1}/{epochs}')

    # Make sure gradient tracking is on, and do a pass over the data
    model.train(True)
    running_loss = 0.
    train_loss = 0.

    for i, t_data in enumerate(train_loader):
        # Every data instance is an input + label pair
        x_inputs, y_labels = t_data[0].to(device), t_data[1].to(device)
        # Zero your gradients for every batch!
        optimizer.zero_grad()
        # Make predictions for this batch
        predictions = model(x_inputs)
        # Compute the loss and its gradients
        loss_val = loss_fn(predictions, y_labels)
        loss_val.backward()
        # Adjust learning weights
        optimizer.step()
        # Gather data and report
        running_loss += loss_val.item()
        if i % (batch_size*10) == (batch_size*10)-1:
            train_loss = running_loss / batch_size # loss per batch
            print('   batch {} loss: {:.4f}'.format(i + 1, train_loss), end='\r')
            tb_x = epoch * len(train_loader) + i + 1
            running_loss = 0.
    print()

    running_vloss = 0.0
    # Set the model to evaluation mode, disabling dropout and using population
    # statistics for batch normalization.
    model.eval()

    # Disable gradient computation and reduce memory consumption.
    with torch.no_grad():
        for i, vdata in enumerate(valid_loader):
            #vinputs, vlabels = vdata 
            vinputs, vlabels = vdata[0].to(device), vdata[1].to(device)
            voutputs = model(vinputs)
            vloss = loss_fn(voutputs, vlabels)
            running_vloss += vloss

    avg_vloss = running_vloss / (i + 1)
    print('   train loss: {:.3f}, valid loss: {:.3f}'.format(train_loss, avg_vloss))
    # Track best performance, and save the model's state
    if avg_vloss < best_vloss:
        best_vloss = avg_vloss
        model_path = 'model_{}_{}'.format(timestamp, epoch)
        ##torch.save(model.state_dict(), model_path)

tf = time.time()
from datetime import timedelta
t =  str(timedelta(seconds=tf - t0))[:-4]
print('\nTraining completed in:', t)

"""
Save model info
"""
# # --- save the model ---
save_path = './'+ model.name + '.pth'
torch.save(model.state_dict(), save_path)


"""
Evaluation
"""

data_train_iter = iter(train_loader)
images, labels = next(data_train_iter)
model.eval()

correct = 0
total = 0
with torch.no_grad():
    for data in train_loader:
        images, labels = data
        pred_out = model(images)
        _, predicted = torch.max(pred_out.data, 0)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

print(f'Accuracy train: {100 * correct // total}%')