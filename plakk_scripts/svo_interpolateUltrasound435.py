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

if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    # Input files
    # ultrasound is also substitute for infrared
    parser.add_option("-u", "--usTimes", dest="usTimes", help="Location of saved ultrasound times", metavar="USTIMES")
    parser.add_option("-c", "--camTimes", dest="camTimes", help="Location of saved camera times", metavar="CAMTIMES")
    parser.add_option("-i", "--camPoses", dest="camPoses", help="Location of saved camera poses", metavar="CAMPOSES")
    parser.add_option("-e", "--cov", dest="cov", help="Location of covariances", metavar="COV")
    parser.add_option("-a", "--camTimesInt", dest="camTimesInt", help="Location of interpolated camera times", metavar="CAMTIMESINT")
    parser.add_option("-b", "--camPosesInt", dest="camPosesInt", help="Location of interpolated camera poses", metavar="CAMPOSESINT")
    parser.add_option("-d", "--covInt", dest="covInt", help="Location of interpolated covariances", metavar="COVINT")
    

    # parse the arguments
    (options, args) = parser.parse_args()

    # load all values
    usTimes = np.load(options.usTimes)
    camTimes = np.load(options.camTimes)
    camPoses = np.load(options.camPoses)
    cov = np.load(options.cov)

    # alternative if more poses than images
    camTimesInt = np.zeros((len(usTimes)))
    camPosesInt = np.zeros((usTimes.shape[0], 3, 4))
    covInt = np.zeros((len(usTimes), 6, 6))

    for i in range(len(usTimes)):
        idx1 = np.abs(camTimes - usTimes[i]).argmin()
        camTimesInt[i] = camTimes[idx1]
        camPosesInt[i,:,:] = camPoses[idx1, :,:]
        covInt[i,:,:] = cov[idx1, :, :]

    # Save
    np.save(options.camTimesInt, camTimesInt)
    np.save(options.camPosesInt, camPosesInt)
    np.save(options.covInt, covInt)

    print("poses interpolated.")
