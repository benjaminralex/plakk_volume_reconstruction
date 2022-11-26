# import libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import scipy.spatial
import scipy.io as sio
from scipy import signal
from scipy.optimize import minimize
import transformations
from numpy import genfromtxt
from optparse import OptionParser
import os
import h5py
import hdf5storage

# This code saves all ultrasound images as an npy
# need to update this so that an npy is accepted or bypass to saveUltrasoundTimes435.py
if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    # Input files
    parser.add_option("-i", "--input", dest="input", help="Location of saved images", metavar="INPUT")

    # output file location
    parser.add_option("-o", "--output", dest="output", help="Output location (including filename) for ultrasound images", metavar="OUTPUT")

    # parse the arguments
    (options, args) = parser.parse_args()

    temp = sio.loadmat(options.input)
    # print(temp    .keys())
    if 'masks' in temp:
        labels = temp['masks']   # NOTE: change to 'masks' if there's error with 'labels'
    else:
        labels = temp['labels']

    usLabels = np.zeros((labels.shape[2], labels.shape[0], labels.shape[1]))

    # Save usImages
    for i in range(labels.shape[2]):
        curr = np.uint8(255*labels[:,:,i])
        curr = cv2.Canny(curr,10,20)
        usLabels[i,:,:] = curr/255.0

    # save numpy array
    np.save(options.output, usLabels)

    print("ultrasound images saved.")
