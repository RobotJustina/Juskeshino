#! /usr/bin/env python3

import rospkg

import torch

from utils.utilities import load_data



if torch.cuda.is_available():
    device = torch.device('cuda:0')
else:
    device = torch.device('cpu')



# --- load files ---
pkg_name = 'machine_learning' #'mapless_nav'
pkg_path = rospkg.RosPack().get_path(pkg_name)
files_path = pkg_path + '/src/Data_gazebo/'
data = load_data(files_path)
print(data.shape)