#! /usr/bin/env python3
import torch

class SingleConvModel(torch.nn.Module):
    def __init__(self):
        super(SingleConvModel, self).__init__()
        f1=32
        l1=64
        expand=32

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
        x = torch.nn.functional.relu(self.norm1(x)) #TODO: verify if is correct
        x = torch.nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
        x = self.dropout1(x)

        x = torch.flatten(x, 1)
        x = self.lin1(x)
        x = torch.nn.functional.relu(self.norm2(x))
        x = self.dropout2(x)

        x = torch.cat((x, pos), 1) #CATEGORY?
        x = self.lin2(x)
        x = torch.nn.functional.tanh(x)
        x = self.lin3(x)
        
        return x
    
    def printMessage(self):
        print(">>>>>>>>>>>>>>Test message")


class Reg(torch.nn.Module):
	def __init__(self):
		super(Reg, self).__init__()
		f1=32   ##Mejor configuración f1 =32, l1=64, lr=8.1e-3, epoch=14
		l1=64
		expand=32
		self.conv1 = torch.nn.Conv2d(1, f1, 3)
		self.dropout1 = torch.nn.Dropout2d(p=0.5)
		self.norm1 = torch.nn.GroupNorm(1, f1)

		self.c1 = torch.nn.Linear(int(39*39*f1), l1) #27380
		self.norm3 = torch.nn.LayerNorm(l1)
		self.dropout3 = torch.nn.Dropout(p=0.5)

		self.c2 = torch.nn.Linear(l1+expand, 100)

		self.c3 = torch.nn.Linear(100, 2)
		#self.c4 = nn.Linear(512, 2)
		self.lr = 8.1e-1
		self.epoch = 50

		self.extra = torch.nn.Linear(2,expand)
		self.extra_norm = torch.nn.LayerNorm(expand)

	def forward(self,x):
		pos=x[:, 6400:]
		pos= self.extra(pos)
		pos = torch.nn.functional.relu(self.extra_norm(pos))
		x=x[:,0:6400]
		x = x.view(x.size(0),1,80,80)

		x = self.conv1(x)
		x = torch.nn.functional.relu(self.norm1(x))
		x = torch.nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
		x = self.dropout1(x)

		x = torch.nn.flatten(x,1)
		x = self.c1(x)
		x = torch.nn.functional.relu(self.norm3(x))
		x = self.dropout3(x)

		x = torch.cat((x, pos), 1)
		x = self.c2(x)
		x = torch.nn.functional.tanh(x)
		x = self.c3(x)
		#x = self.c4(x)
		return x
