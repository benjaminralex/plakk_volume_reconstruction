# import libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
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
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    # Input files
    parser.add_option("-c", "--images", dest="images", help="Location of file containing images", metavar="IMAGES")

    # output file location
    parser.add_option("-t", "--times", dest="times", help="Output location for us image times", metavar="TIMES")
    parser.add_option("--ct", "--camTimes", dest="camTimes", help="location of camera times", metavar="CAMTIMES")

    # parse the arguments
    (options, args) = parser.parse_args()

    # Read bag file
    images = np.load(options.images)
    camTimes = np.load(options.camTimes)
    times = []

    # Start and end time
    posesStart = camTimes[0]
    posesEnd = camTimes[-1]
    duration = posesEnd - posesStart
    dt = duration/(images.shape[0] - 1)

    # alternative if poses ran longer
    #posesStart = camTimes[0]
    #fr = 28.0
    #dt = 1.0/fr
    #duration = dt*(images.shape[0])
    #posesEnd = posesStart + duration

    # create times
    times = np.arange(posesStart, posesEnd, dt)

    # Save
    np.save(options.times, times)

    print("ultrasound times created.")
