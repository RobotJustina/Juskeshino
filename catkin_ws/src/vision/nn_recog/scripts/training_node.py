#!/usr/bin/env python3

import cv2
import random
import numpy
import rospy
import rospkg
import os
from FCNN import FullyConnectedNeuralNetwork

def load_dataset(dataset_folder, resize_w, resize_h):
    labels  = os.listdir(dataset_folder)
    images  = []
    targets = []
    n = len(labels)
    for i in range(len(labels)):
        files = os.listdir(dataset_folder + "/" + labels[i])
        for f in files:
            img_file = dataset_folder  + labels[i] + "/" + f
            print("Loading image file: " + img_file)
            img = cv2.resize(cv2.imread(img_file), (resize_w, resize_h)).astype(numpy.float32)/255.0            
            img = numpy.reshape(img, [resize_w*resize_h*3,1])
            t   = numpy.zeros((n,1), dtype=numpy.float32)
            t[i]= 1.0
            images.append(img)
            targets.append(t)
    return images, targets, labels

def main():
    print("INITIALIZING NEURAL-NETWORK TRAINER FOR OBJECT RECOGNIZION BY MARCOSOFT...")
    rospy.init_node("nn_recog")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("obj_reco") + "/training_dir/"
    model_file     = dataset_folder + "network.npz";
    epochs         = 10
    batch_size     = 3
    learning_rate  = 3.0
    
    if rospy.has_param("~epochs"):
        epochs = rospy.get_param("~epochs")
    if rospy.has_param("~batch_size"):
        batch_size = rospy.get_param("~batch_size")
    if rospy.has_param("~learning_rate"):
        learning_rate = rospy.get_param("~learning_rate")

    images, targets, labels = load_dataset(dataset_folder, 80,80)
    print("Loaded " + str(len(labels)) + " labels: " + str(labels))
    print("Loaded " + str(len(images)) + " images")

    # try:
    #     saved_data = numpy.load(dataset_folder+"network.npz",allow_pickle=True)
    #     layers = [saved_data['w'][0].shape[1]] + [b.shape[0] for b in saved_data['b']]
    #     nn = NeuralNetwork(layers, weights=saved_data['w'], biases=saved_data['b'])
    #     print("Loading data from previously trained model with layers " + str(layers))
    # except:
    #     print("Initializing new fully connected neural network")
    #     nn = FullyConnectedNeuralNetwork([784,30,10])
    #     pass

    #print(images.shape)
    #print(targets.shape)
    print(images[0].shape)
    print(targets[0].shape)
    input_size = len(images[0])
    number_of_labels = len(targets[0])
    layers = [input_size, 30, number_of_labels]
    print("Creating neural network with layers: " + str(layers))
    nn = FullyConnectedNeuralNetwork(layers)
    nn.train_by_SGD(images, targets, epochs, batch_size, learning_rate)
    #numpy.savez(dataset_folder + "network",w=nn.weights, b=nn.biases)
    
    print("\nPress key to test network or ESC to exit...")
    numpy.set_printoptions(formatter={'float_kind':"{:.3f}".format})
    cmd = cv2.waitKey(0)
    while cmd != 27 and not rospy.is_shutdown():
        idx = numpy.random.randint(0, len(images))
        img,target = images[idx], targets[idx]
        y = nn.feedforward(img).transpose()
        print("\nPerceptron output: " + str(y))
        print("Expected output  : "   + str(target.transpose()))
        print("Recognized digit : "   + str(labels[numpy.argmax(y)]))
        cv2.imshow("Digit", numpy.reshape(img, (80,80,3)))
        cmd = cv2.waitKey(0)
    

if __name__ == '__main__':
    main()
