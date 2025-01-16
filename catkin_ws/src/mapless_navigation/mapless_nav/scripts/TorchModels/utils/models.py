#! /usr/bin/env python3
import torch

class Red_conv(torch.nn.Module):
    def __init__(self, salida):
        f1 = 32  # Mejor configuración f1 =32, l1=64, lr=8.1e-3, epoch=14
        l1 = 128
        expand = 32
        super(Red_conv, self).__init__()
        self.conv1 = torch.nn.Conv2d(1, f1, 3)
        self.dropout1 = torch.nn.Dropout2d(p=0.5)
        self.norm1 = torch.nn.GroupNorm(1, f1)

        self.c1 = torch.nn.Linear(int(39*39*f1), l1)  # 27380
        self.norm3 = torch.nn.LayerNorm(l1)
        self.dropout3 = torch.nn.Dropout(p=0.5)

        self.c2 = torch.nn.Linear(l1+expand, salida)

        self.lr = 8.1e-3
        self.epoch = 14

        self.extra = torch.nn.Linear(2, expand)
        self.extra_norm = torch.nn.LayerNorm(expand)

    def forward(self, x):
        pos = x[:, 6400:]
        pos = self.extra(pos)
        pos = torch.nn.functional.relu(self.extra_norm(pos))
        x = x[:, 0:6400]
        x = x.view(x.size(0), 1, 80, 80)

        x = self.conv1(x)
        x = torch.nn.functional.relu(self.norm1(x))
        x = torch.nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
        x = self.dropout1(x)

        x = torch.flatten(x, 1)
        x = self.c1(x)
        x = torch.nn.functional.relu(self.norm3(x))
        x = self.dropout3(x)

        x = torch.cat((x, pos), 1)
        x = self.c2(x)
        return torch.nn.functional.softmax(x, dim=1)
    

class SingleConvModel(torch.nn.Module):
    def __init__(self):
        super(SingleConvModel, self).__init__()
        f1 = 32
        l1 = 64
        expand = 32

        self.conv1 = torch.nn.Conv2d(1, f1, 3)
        self.dropout1 = torch.nn.Dropout2d(p=0.5)
        self.norm1 = torch.nn.GroupNorm(1, f1)

        self.lin1 = torch.nn.Linear(int(39*39*f1), l1)
        self.norm2 = torch.nn.LayerNorm(l1)
        self.dropout2 = torch.nn.Dropout(p=0.5)

        self.lin2 = torch.nn.Linear(l1+expand, 100)

        self.lin3 = torch.nn.Linear(100, 2)
        self.lr = 8.1e-1
        self.epoch = 50

        self.extra = torch.nn.Linear(2, expand)
        self.extra_norm = torch.nn.LayerNorm(expand)

    def foward(self, x):
        # TODO: normalize values instead flat normalization
        pos = x[:, 6400:]
        pos = self.extra(pos)
        pos = torch.nn.functional.relu(self.extra_norm(pos))
        x = x[:, 0:6400]
        x = x.view(x.size(0), 1, 80, 80)

        x = self.conv1(x)
        x = torch.nn.functional.relu(self.norm1(x))
        x = torch.nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
        x = self.dropout1(x)

        x = torch.flatten(x, 1)
        x = self.lin1(x)
        x = torch.nn.functional.relu(self.norm2(x))
        x = self.dropout2(x)

        x = torch.cat((x, pos), 1)  # CATEGORY?
        x = self.lin2(x)
        x = torch.nn.functional.tanh(x)
        x = self.lin3(x)
        return x


class CNN_A(torch.nn.Module):
    def __init__(self):
        super(CNN_A, self).__init__()

        self.conv1 = torch.nn.Conv2d(1, 3, 3)
        self.conv2 = torch.nn.Conv2d(3, 15, 5)
        self.dropout_10 = torch.nn.Dropout(p=0.1)
        self.conv3 = torch.nn.Conv2d(15, 80, 5)
        self.dropout_40 = torch.nn.Dropout(p=0.4)
        self.norm_l3 = torch.nn.GroupNorm(1, 80)
        self.flat1 = torch.nn.Linear(70, 100)

        # second input
        self.vector = torch.nn.Linear(2, 2)

        # merge
        self.flat2 = torch.nn.Linear(560002, 150)
        self.dropout_20 = torch.nn.Dropout(p=0.2)
        self.flat3 = torch.nn.Linear(150, 60)
        self.flat4 = torch.nn.Linear(60, 3)

    def forward(self, x):
        # vector
        vec = x[:, 6400:]
        vec = self.vector(vec)
        vec = torch.nn.functional.relu(self.vector(vec))
        # image
        x = x[:, 0:6400]
        x = x.view(x.size(0),1,80,80)

        x = self.conv1(x)
        x = torch.nn.functional.relu(x)

        x = self.conv2(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_40(x)

        x = self.conv3(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_40(x)
        x = self.norm_l3(x)

        x = self.flat1(x)
        x = self.dropout_20(x)

        x = torch.flatten(x, 1)
        # concat inputs
        x = torch.cat((x, vec ), 1)
        x = self.flat2(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_10(x)
        x = self.flat3(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_10(x)
        x = self.flat4(x)
        x = torch.nn.functional.relu(x)

        x = torch.nn.functional.softmax(x, dim=1)
        return x
    

class CNN_B(torch.nn.Module):
    def __init__(self):
        super(CNN_B, self).__init__()

        self.conv1 = torch.nn.Conv2d(1, 6, 5)
        # self.conv2 = torch.nn.Conv2d(3, 8, 5)
        self.dropout_50 = torch.nn.Dropout2d(p=0.5)
        self.conv3 = torch.nn.Conv2d(6, 18, 3)
        self.dropout_40 = torch.nn.Dropout(p=0.4)
        self.norm_l3 = torch.nn.GroupNorm(1, 18)
        self.flat1 = torch.nn.Linear(74, 32)

        # second input
        self.vector = torch.nn.Linear(2, 2)

        # merge
        self.flat2 = torch.nn.Linear(42626, 16)
        self.dropout_20 = torch.nn.Dropout(p=0.2)
        # self.flat3 = torch.nn.Linear(60, 32)
        self.flat4 = torch.nn.Linear(16, 3)

    def forward(self, x):
        # vector
        vec = x[:, 6400:]
        vec = self.vector(vec)
        vec = torch.nn.functional.relu(self.vector(vec))
        # image
        x = x[:, 0:6400]
        x = x.view(x.size(0), 1, 80, 80)

        x = self.conv1(x)
        x = torch.nn.functional.relu(x)

        # x = self.conv2(x)
        # x = torch.nn.functional.relu(x)
        # x = self.dropout_50(x)

        x = self.conv3(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_50(x)
        x = self.norm_l3(x)

        x = self.flat1(x)
        x = self.dropout_40(x)

        x = torch.flatten(x, 1)
        # concat inputs
        x = torch.cat((x, vec), 1)
        x = self.flat2(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_20(x)
        # x = self.flat3(x)
        # x = torch.nn.functional.relu(x)
        # x = self.dropout_20(x)
        x = self.flat4(x)
        x = torch.nn.functional.relu(x)

        x = torch.nn.functional.softmax(x, dim=1)
        return x