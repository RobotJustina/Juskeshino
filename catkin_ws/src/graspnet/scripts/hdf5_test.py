#!/usr/bin/env python3

import numpy as np
import os
import pandas as pd
import h5py

FILE_PATH = '/home/robocup/Downloads/egad310320.hdf5'
EGAD_PATH = '/home/robocup/egad_object_set/'

f = h5py.File(FILE_PATH,'r')

objects = f['datasets/train/objects']

a = np.array([1,2,3,4,5,6,7,8,9,10])

b = a[a>5]

print(b)

# for key in objects.keys():
#     print(key) #Names of the root level object names in HDF5 file - can be groups or datasets.
#     object_path = os.path.join(EGAD_PATH, key)
#     #print(object_path)
#     if not os.path.exists(object_path):
#         os.makedirs(object_path)
#     #print(type(objects[key])) # get the object type: usually group or dataset

# a0_sdf = objects['A00_0/sdf']
# for key in a0_sdf.keys():
#     print(key) #Names of the root level object names in HDF5 file - can be groups or datasets.
# print(a0_sdf['data'])
# print(a0_sdf['data'][:])
# f.close()
#a0_sdf.visit(print)
# #Get the HDF5 group; key needs to be a group name from above
# group = f[key]

# #Checkout what keys are inside that group.
# for key in group.keys():
#     print(key)
#     print(type(group[key]))

# # This assumes group[some_key_inside_the_group] is a dataset, 
# # and returns a np.array:
# data = group['train']
# #Do whatever you want with data
# print(data)
# print(type(data))

# #Checkout what keys are inside that group.
# for key in data.keys():
#     print(key)
#     print(type(data[key]))

# # metrics = data['metrics']
# # #Checkout what keys are inside that group.
# # for key in metrics.keys():
# #     print(key)
# #     print(type(metrics[key]))

# # metrics.visititems(print)

# objects = data['objects']
# # #Checkout what keys are inside that group.
# # for key in objects.keys():
# #     print(key)
# #     print(type(objects[key]))

# s190 = objects['S19_0']
# #Checkout what keys are inside that group.
# for key in s190.keys():
#     print(key)
#     print(type(s190[key]))
# #print(data.items())
# #data.visit(print)

#After you are done
#f.close()

# def print_grp_name(grp_name, object):
# #  print ('object = ' , object)
# #  print ('Group =', object.name)

#   try:
#     n_subgroups = len(object.keys())
#     #print ('Object is a Group')
#   except:
#     n_subgroups = 0
#     #print ('Object is a Dataset')
#     dataset_list.append (object.name)

# #  print ('# of subgroups = ', n_subgroups )

# if __name__ ==  '__main__' :  
#     with h5py.File(FILE_PATH,'r') as h5f:

#         print ('visting group = ', h5f)
#         dataset_list = []
#         h5f.visititems(print_grp_name)

#     print (dataset_list)    

# def h5_tree(val, pre=''):
#     items = len(val)
#     for key, val in val.items():
#         items -= 1
#         if items == 0:
#             # the last item
#             if type(val) == h5py._hl.group.Group:
#                 print(pre + '└── ' + key)
#                 h5_tree(val, pre+'    ')
#             else:
#                 try:
#                     print(pre + '└── ' + key + ' (%d)' % len(val))
#                 except TypeError:
#                     print(pre + '└── ' + key + ' (scalar)')
#         else:
#             if type(val) == h5py._hl.group.Group:
#                 print(pre + '├── ' + key)
#                 h5_tree(val, pre+'│   ')
#             else:
#                 try:
#                     print(pre + '├── ' + key + ' (%d)' % len(val))
#                 except TypeError:
#                     print(pre + '├── ' + key + ' (scalar)')

# with h5py.File(FILE_PATH, 'r') as hf:
#     print(hf)
#     h5_tree(hf)

#df = pd.read_hdf(FILE_PATH,mode='r')
#print(df.info())
