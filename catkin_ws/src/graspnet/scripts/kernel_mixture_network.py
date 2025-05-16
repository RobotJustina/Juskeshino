#!/home/robocup/venvs/python3_9/bin/python

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
import geomstats.backend as gs
from joblib import Parallel, delayed, parallel_config
from matplotlib.colors import Normalize
from geomstats.geometry.hypersphere import Hypersphere
from geomstats.learning import kmeans, online_kmeans
#from botorch.utils.sampling import sample_hypersphere

from scipy.stats import vonmises_fisher
from scipy.stats import uniform_direction

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class Kernel_Mixture_Network(nn.Module):
    def __init__(self, dim_in, hidden_dim, n_components, component_centers, space_dim, component_kappa):
        super().__init__()
        self.wi_network = nn.Sequential(
            nn.Linear(dim_in, hidden_dim),
            nn.SiLU(),
            nn.Dropout(0.25),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Dropout(0.25),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Dropout(0.25),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Dropout(0.25),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Dropout(0.25),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Dropout(0.25),
            nn.Linear(hidden_dim, n_components),
            nn.ReLU(),
        ).float()
        self.Kcenters = component_centers
        self.vmfdist = [vonmises_fisher(mu ,kappa=component_kappa) for mu in self.Kcenters]
        self.space_dim = space_dim
        self.workers = Parallel(n_jobs=8, backend='loky', return_as='generator_unordered')
        self.space = Hypersphere(space_dim-1)
    
    def forward(self, x):
        x = self.wi_network(x)
        x = F.softmax(x,dim=-1,dtype=torch.float64)

        return x
    
    def parallel_kernel_sum(self, y):
        ksum = np.fromiter(self.workers(delayed(k.pdf)(y) for k in self.vmfdist),dtype=np.float64,count=-1)
        ksum = torch.from_numpy(ksum).T.to(DEVICE)
        #ksum = torch.tensor(np.asarray(ksum)).T.to(DEVICE)
        return ksum

    def loss(self, x, y):
        wi = self.forward(x)
        #ksum = np.asarray([k.pdf(y) for k in self.vmfdist])
        ksum = torch.tensor(np.asarray([k.pdf(y) for k in self.vmfdist]),device=DEVICE).T
        #ksum = torch.from_numpy(np.fromiter([k.pdf(y) for k in self.vmfdist],dtype=np.float64,count=-1)).T.to(DEVICE)
        #ksum = self.parallel_kernel_sum(y)
        #ksum = torch.from_numpy(ksum).T
        #print(ksum.shape)
        #mixture = torch.bmm(wi.unsqueeze(dim=1),ksum.unsqueeze(dim=-1)).squeeze()
        
        mixture = torch.einsum('bi,bi->b',wi,ksum)
        #print(mixture.shape)
        log_likelihood = torch.log(mixture)
        return -log_likelihood
    
    def sample(self, x):
        #print(x.shape)
        wi = self.forward(x)
        #print(wi.shape)
        #y = vonmises_fisher.rvs([0,0,0,1],kappa=0,size=100)
        #y = uniform_direction.rvs(self.space_dim,(x.shape[0],256))
        #y = [self.space.random_uniform(256) for i in range(x.shape[0])]
        #y = list(self.workers(delayed(self.space.random_uniform)(256) for i in range(x.shape[0])))
        y = np.asarray(list(self.workers(delayed(uniform_direction.rvs)(self.space_dim,512) for i in range(x.shape[0]))))
        y = torch.from_numpy(y)
        #y = torch.tensor(np.asarray(y),device='cpu')
        #y = sample_hypersphere(self.space_dim,x.shape[0]*256).view(x.shape[0],256,self.space_dim)
        #y = self.space.random_uniform(x.shape[0]*256).view(x.shape[0],256,self.space_dim)

        #print(y.shape)
        #ksum = np.asarray([k.pdf(y) for k in self.vmfdist])
        #print(ksum)
        #ksum = torch.stack(ksum,dim=1)
        #ksum = torch.from_numpy(ksum).to(DEVICE)
        #ksum = self.parallel_kernel_sum(y)
        ksum = torch.tensor(np.asarray([k.pdf(y) for k in self.vmfdist]),device=DEVICE)
        #print(ksum.shape)
        ksum = torch.permute(ksum,(1,2,0))
        #print(ksum.shape)
        #mixture = torch.bmm(wi.unsqueeze(dim=1),ksum.unsqueeze(dim=2)).squeeze()
        #mixture = torch.tensordot(wi,ksum,dims=1)
        mixture = torch.einsum('bi,bji->bj',wi,ksum)
        #print(mixture.shape)
        #print(mixture)
        idx = torch.argmax(mixture,dim=-1)
        y = y.cuda()
        #y[:] = y[:,idx]
        #y = torch.tensor(y,device=DEVICE)
        #return y[:,torch.argmax(mixture,dim=-1),:]
        #y = torch.tensor(y,device=DEVICE)
        y = y[range(len(idx)),idx]
        #y = torch.take_along_dim(y,idx,dim=1)
        #print(y.shape)
        return y
    
def sphere_test():
    #kcenters = [[0.0,0.0,1.0],[0.0,1.0,0.0],[1.0,0.0,0.0]]
    kcenters = [[0.0,0.0,0.0,1.0],[0.0,0,1.0,0.0],[0.0,1.0,0.0,0.0],[1.0,0.0,0.0,0.0]]
    vmfdist = [vonmises_fisher(kc,kappa=128) for kc in kcenters]
    y1 = vmfdist[0].rvs(800)
    y2 = vmfdist[1].rvs(800)
    y3 = vmfdist[2].rvs(800)
    y = torch.from_numpy(np.concatenate((y1,y2,y3),axis=0)).to(DEVICE)

    x1 = torch.ones(800,dtype=torch.float32).unsqueeze(-1)*-1
    x2 = torch.ones(800,dtype=torch.float32).unsqueeze(-1)*200
    x3 = torch.ones(800,dtype=torch.float32).unsqueeze(-1)*3000
    
    print(x1.shape)
    x = torch.cat((x1,x2,x3),dim=0).to(DEVICE)
    print(x.shape)
    print(x)

    rdist = torch.from_numpy(uniform_direction(4).rvs(10000))
    print(rdist)
    km_centers = find_kmeans_centers(256,4,rdist)

    torch.set_default_dtype(torch.float32)
    model = Kernel_Mixture_Network(1,64,256,km_centers,4,350).to(DEVICE)
    optimizer = optim.AdamW(model.wi_network.parameters(),lr=0.01,weight_decay=0.02)
    num_epochs = 200
    for epoch in range(num_epochs):
        #out = model.sample(x)
        loss = model.loss(x,y.cpu()).mean()
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print ('Epoch [{}/{}], Training Loss: {:.4f}'.format(epoch+1, num_epochs, loss.item()))

    with torch.no_grad():
        model.eval()
        
        xt = torch.tensor([[-1]],dtype=torch.float32).to(DEVICE)
        p = model(xt)
        u = model.sample(xt)
        print(p)
        print(u)
        xt = torch.tensor([[200]],dtype=torch.float32).to(DEVICE)
        p = model(xt)
        u = model.sample(xt)
        print(p)
        print(u)
        xt = torch.tensor([[3000]],dtype=torch.float32).to(DEVICE)
        p = model(xt)
        u = model.sample(xt)
        print(p)
        print(u)

def find_kmeans_centers(k,dim,data = None,max_iter=100000):
    space = Hypersphere(dim)
    #kmeans_clustering = kmeans.RiemannianKMeans(space,k,'kmeans++',max_iter=max_iter)
    kmeans_clustering = online_kmeans.OnlineKMeans(space=space,n_clusters=k,n_repetitions=300,max_iter=max_iter)
    if data is None:
        data = torch.from_numpy(uniform_direction(dim).rvs(10000))
    kmeans_clustering.fit(data)

    return kmeans_clustering.cluster_centers_

def main():
    #kcenters = [[0.0,0.0,0.0,1.0],[0.0,0,1.0,0.0],[0.0,1.0,0.0,0.0],[1.0,0.0,0.0,0.0]]
    # kcenters = [[0.0,0.0,1.0],[0.0,1.0,0.0],[1.0,0.0,0.0]]
    # model = Kernel_Mixture_Network(1,5,3,kcenters,3,30).double().cuda()
    # x = torch.ones(1).unsqueeze(0).cuda()
    # y = torch.tensor(uniform_direction.rvs(3)).unsqueeze(0)
    # print(y)
    # #y = uniform_direction.rvs(4)
    # print(model(x))
    # #print(model.sample(x))
    # print(model.loss(x,y))
    # print(model.sample(x))
    sphere_test()

if __name__ == '__main__':
    main()