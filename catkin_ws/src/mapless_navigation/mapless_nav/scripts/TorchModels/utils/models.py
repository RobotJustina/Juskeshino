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


### TODO: delete---------------------------------------------->
class SNet(torch.nn.Module):
    def __init__(self, channels=5, img_size=80):
        f1 = 32  # Mejor configuración f1 =32, l1=64, lr=8.1e-3, epoch=14
        l1 = 128
        expand = 32
        
        super(SNet, self).__init__()
        self.name = "SNet"
        self.channels = channels
        
        #self.conv1 = torch.nn.Conv2d(1, f1, 3)
        self.conv1 = torch.nn.Conv2d(self.channels, f1, 3)

        self.dropout1 = torch.nn.Dropout2d(p=0.5)
        self.norm1 = torch.nn.GroupNorm(1, f1)

        self.c1 = torch.nn.Linear(int(39*39*f1), l1)  # 27380
        self.norm3 = torch.nn.LayerNorm(l1)
        self.dropout3 = torch.nn.Dropout(p=0.5)

        #self.c2 = torch.nn.Linear(l1+expand, 2)
        self.c2 = torch.nn.Linear(l1+img_size*2, 2)


        #self.extra = torch.nn.Linear(2, expand)
        #self.extra_norm = torch.nn.LayerNorm(expand)
        #self.extra_norm = torch.nn.LayerNorm(img_size)

    def forward(self, x):

        ##print("forward")
        #pos = x[:, 6400:]
        vect = x[:, 0, -2:, :]  # [batch_s, channel, col, row]
        ##print("vect", vect.shape)
        #print(vect)
        
        #pos = self.extra(pos)
        #pos = torch.nn.functional.relu(self.extra_norm(pos))
        #vect = torch.nn.functional.relu(self.extra_norm(vect))
        #print(vect.shape)
        #print(vect)

        vect = torch.flatten(vect, 1)
        # print(vect.shape)
        # print(vect)
        
        # IMAGE
        #x = x[:, 0:6400]
        ##print(x.shape)
        #print(x)
        x = x[:, :, :80, :]
        ##print(x.shape)
        #print(x)
        #x = x.view(x.size(0), 1, 80, 80)


        x = self.conv1(x)
        ##print("conv1", x.shape)
        x = torch.nn.functional.relu(self.norm1(x))
        #print("norm1", x.shape)
        x = torch.nn.functional.avg_pool2d(x, kernel_size=2, stride=2)
        ##print("avg_pool2d", x.shape)
        x = self.dropout1(x)

        x = torch.flatten(x, 1)
        ##print("flatten", x.shape)
        x = self.c1(x)
        ##print("c1", x.shape)
        x = torch.nn.functional.relu(self.norm3(x))
        x = self.dropout3(x)

        #x = torch.cat((x, pos), 1)
        x = torch.cat((x, vect), 1)
        ##print("cat", x.shape)
        x = self.c2(x)
        ##print("c2", x.shape)
        #return torch.nn.functional.softmax(x, dim=1)
        x = torch.nn.functional.tanh(x)
        ##print("out.shape", x.shape)

        return x


class CNN_B(torch.nn.Module):
    def __init__(self):
        super(CNN_B, self).__init__()
        self.name = 'CNN_B'

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


class CNN_2feat(torch.nn.Module):
    def __init__(self):
        super(CNN_2feat, self).__init__()
        self.name = 'CNN_2feat'
        
        # layers
        self.dropout_60 = torch.nn.Dropout(p=0.6)
        self.dropout_40 = torch.nn.Dropout(p=0.4)
        self.norm_l3 = torch.nn.GroupNorm(1, 32)
        self.dropout_20 = torch.nn.Dropout(p=0.2)
        
        # second input
        self.vector = torch.nn.Linear(2, 32)

        # merge
        self.flat2 = torch.nn.Linear(32, 16)
        self.out = torch.nn.Linear(16, 2)

    def forward(self, x):
        # vector: d, th
        vect = x[:, 0, -1, :2]  # [batch_s, channel, col, row]
        vect = self.vector(vect)
        vect = torch.nn.functional.relu(vect)

        x = self.flat2(vect)
        x = torch.nn.functional.relu(x)

        x = self.out(x)
        x = torch.nn.functional.tanh(x)

        #x = torch.nn.functional.softmax(x, dim=1)
        return x


class NN_82_80(torch.nn.Module):
    def __init__(self):
        super(NN_82_80, self).__init__()
        self.name = 'NN_82_80'
        
        # layers
        self.conv1 = torch.nn.Conv2d(5, 32, 3)
        self.conv2 = torch.nn.Conv2d(32, 64, 3)
        self.dropout_60 = torch.nn.Dropout(p=0.6)
        self.dropout_40 = torch.nn.Dropout(p=0.4)
        self.norm_l3 = torch.nn.GroupNorm(1, 32)
        self.dropout_20 = torch.nn.Dropout(p=0.2)
        
        # second input
        self.flat1 = torch.nn.Linear(40, 120)

        # merge
        self.flat2 = torch.nn.Linear(120, 80)
        self.out = torch.nn.Linear(80, 2)

    def forward(self, x):
        # vector: d, th
        # Get last 2 cols: [[80(distance)], [80(angle)]
        vect = x[:, 0, -2:, :]  # [batch_s, channel, col, row]
        vect = torch.flatten(vect, 1)
        print("vect.shape", vect.shape)
        #print("vect", vect)
        # image
        # [batch_s, channel, col, row]
        x = x[:, :, :-2]

        x = self.conv1(x)
        x = torch.nn.functional.relu(x)
        #x = self.conv2(x)
        #x = torch.nn.functional.relu(x)
        x = torch.flatten(x, 1)
        # print("x.shape", x.shape)
        # print("vect.shape", vect.shape)
        # concat inputs

        vect = self.flat1(vect)
        vect = torch.nn.functional.relu(vect)
        x = torch.cat((x, vect), 1)
        #print("cat.shape", x.shape)

        x = self.flat2(vect)
        x = torch.nn.functional.relu(x)

        x = self.out(x)
        x = torch.nn.functional.tanh(x)
        print("out.shape", x.shape)

        #x = torch.nn.functional.softmax(x, dim=1)
        return x
#--------------------------------------------<


class CNN_RegTanh(torch.nn.Module):
    def __init__(self):
        super(CNN_RegTanh, self).__init__()
        self.name = 'CNN_RegTanh'
        # layers
        self.conv1 = torch.nn.Conv2d(1, 16, 3)
        # self.conv2 = torch.nn.Conv2d(3, 8, 5)
        self.dropout_50 = torch.nn.Dropout2d(p=0.5)
        self.conv3 = torch.nn.Conv2d(16, 32, 3)
        self.dropout_40 = torch.nn.Dropout(p=0.4)
        self.norm_l3 = torch.nn.GroupNorm(1, 32)
        
        self.flat1 = torch.nn.Linear(184834, 120)

        # second input
        self.vector = torch.nn.Linear(2, 2)

        # merge
        self.flat2 = torch.nn.Linear(120, 32)
        self.dropout_20 = torch.nn.Dropout(p=0.2)
        self.flat3 = torch.nn.Linear(32, 6)
        self.out = torch.nn.Linear(6, 2)

    def forward(self, x):
        # vector: d, th
        vect = x[:, 0, -1, :2]  # [batch_s, channel, col, row]
        # print(vect.size())
        # print(vect)
        vect = self.vector(vect)

        vect = torch.nn.functional.relu(self.vector(vect))


        # image
        # [batch_s, channel, col, row]
        x = x[:, :, :-1]    
        # print(x.size())
        # print(x)

        # Architecture
        x = self.conv1(x)
        x = torch.nn.functional.relu(x)

        # x = self.conv2(x)
        # x = torch.nn.functional.relu(x)
        # x = self.dropout_50(x)

        x = self.conv3(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_50(x)
        x = self.norm_l3(x)



        x = torch.flatten(x, 1)
        x = torch.cat((x, vect), 1)
        x = self.flat1(x)
        x = self.dropout_40(x)
        # concat inputs
        #x = torch.cat((x, vect), 1)
        x = self.flat2(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_20(x)
        
        x = self.flat3(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_20(x)
        x = self.out(x)
        x = torch.nn.functional.tanh(x)

        #x = torch.nn.functional.softmax(x, dim=1)
        return x



class Param_CNN(torch.nn.Module):
    def __init__(self, channels=5, img_size=80):
        super(Param_CNN, self).__init__()
        """
        A fully convolutional NN
        """
        self.name = "Param_CNN"
        self.channels = channels
        self.img_shape = (img_size, img_size)
        #print(self.name, self.img_shape)
        
        # layers
        self.conv1 = torch.nn.Conv2d(self.channels, 16, 3)
        self.conv2 = torch.nn.Conv2d(16, 32, 3)
        self.batch_norm_32 = torch.nn.BatchNorm2d(32)
        self.max_pool = torch.nn.MaxPool2d(5, 3)
        self.conv3 = torch.nn.Conv2d(32, 8, 3)

        #width=((W-F+2*P )/S)+1 
        width = (((img_size-4-5)/3)+1) // 1
        height = (((img_size-2-5)/3)+1) // 1
        width = (((width-2-5)/3)+1) // 1
        height = (((height-2-5)/3)+1) // 1
        
        print("width", width)
        print("height", height)
        n_chan = int(8 * height * width) +  2*img_size
        print("n_chan", n_chan)
        #n_chan *= channels
        #print("n_chan", n_chan)


        self.flat1 = torch.nn.Linear(n_chan, 120)
        self.flat2 = torch.nn.Linear(120, 80)
        self.out = torch.nn.Linear(80, 2)

        # self.conv1x1_1 = torch.nn.Conv1d(n_chan, 120, 1)

        # self.conv1x1_2 = torch.nn.Conv1d(120, 32, 1)
        
        #self.out = torch.nn.Conv1d(32, 2, 1)
        #self.out = torch.nn.Linear(32, 2)


    def forward(self, x):
        # vector: d, th
        vect = x[:, 0, -2:, :]  # [batch_s, channel, col, row]
        #vect = torch.reshape(vect, (5, 1, 2*vect.shape[2]))
        vect = torch.flatten(vect, 1)
        #print("vect.shape", vect.shape)
        
        #vect.shape torch.Size([5, 2, 80])
        #print(vect)

        # image
        # [batch_s, channel, col, row]
        #x = x[:, :, :-2]
        #print("x.shape", x.shape)

        x = self.conv1(x)
        x = torch.nn.functional.relu(x)
        #print("x1.shape", x.shape)

        x = self.conv2(x)
        x = self.batch_norm_32(x)
        x = self.max_pool(x)
        x = torch.nn.functional.relu(x)
        #print("x2.shape", x.shape)
  
        x = self.conv3(x)
        x = self.max_pool(x)
        x = torch.nn.functional.relu(x)
        #print("x3.shape", x.shape)
        
        x = torch.flatten(x, 1)
        #print("flat.shape", x.shape)
        #print(x[:,-20])

        x = torch.cat((x, vect), 1)
        #print("cat.shape", x.shape)
        #print(x[0])

        x = self.flat1(x)
        x = torch.nn.functional.relu(x)
        #print("flat1.shape", x.shape)

        x = self.flat2(x)
        x = torch.nn.functional.relu(x)
        #print("flat2.shape", x.shape)

        x = self.out(x)
        x = torch.nn.functional.tanh(x)
        #print("out.shape", x.shape)

        return x


class Param_CNN_B(torch.nn.Module):
    def __init__(self, channels=5, img_size=80):
        super(Param_CNN_B, self).__init__()
        """
        A fully convolutional NN
        """
        self.name = "Param_CNN_B"
        self.channels = channels
        self.img_shape = (img_size, img_size)
        #print(self.name, self.img_shape)
        
        # layers
        self.conv1 = torch.nn.Conv2d(self.channels, 16, 3)
        self.conv2 = torch.nn.Conv2d(16, 32, 3)
        self.batch_norm_32 = torch.nn.BatchNorm2d(32)
        self.max_pool = torch.nn.MaxPool2d(5, 3)
        self.conv3 = torch.nn.Conv2d(32, 8, 3)

        #width=((W-F+2*P )/S)+1 
        width = (((img_size-4-5)/3)+1) // 1
        height = (((img_size-2-5)/3)+1) // 1
        width = (((width-2-5)/3)+1) // 1
        height = (((height-2-5)/3)+1) // 1
        
        print("width", width)
        print("height", height)
        n_chan = int(8 * height * width) +  2*img_size
        print("n_chan", n_chan)
        #n_chan *= channels
        #print("n_chan", n_chan)


        self.flat1 = torch.nn.Linear(n_chan, 120)
        self.flat2 = torch.nn.Linear(120, 80)
        self.out = torch.nn.Linear(80, 3)

        self.dropout_20 = torch.nn.Dropout(p=0.2)
        self.dropout_15 = torch.nn.Dropout(p=0.15)
        self.dropout_10 = torch.nn.Dropout(p=0.1)


    def forward(self, x):
        # vector: d, th
        vect = x[:, 0, -2:, :]  # [batch_s, channel, col, row]
        #vect = torch.reshape(vect, (5, 1, 2*vect.shape[2]))
        vect = torch.flatten(vect, 1)
        #print("vect.shape", vect.shape)
        
        #vect.shape torch.Size([5, 2, 80])
        #print(vect)

        # image
        # [batch_s, channel, col, row]
        #x = x[:, :, :-2]
        #print("x.shape", x.shape)

        x = self.conv1(x)
        x = torch.nn.functional.relu(x)
        #print("x1.shape", x.shape)

        x = self.conv2(x)
        x = self.batch_norm_32(x)
        x = self.max_pool(x)
        x = torch.nn.functional.relu(x)
        #print("x2.shape", x.shape)
  
        x = self.conv3(x)
        x = self.max_pool(x)
        x = torch.nn.functional.relu(x)
        #print("x3.shape", x.shape)
        
        x = torch.flatten(x, 1)
        #print("flat.shape", x.shape)
        #print(x[:,-20])

        x = torch.cat((x, vect), 1)
        #print("cat.shape", x.shape)
        #print(x[0])

        x = self.flat1(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_20(x)
        #print("flat1.shape", x.shape)

        x = self.flat2(x)
        x = torch.nn.functional.relu(x)
        x = self.dropout_15(x)
        #print("flat2.shape", x.shape)

        x = self.out(x)
        x = torch.nn.functional.tanh(x)
        #print("out.shape", x.shape)

        return x
