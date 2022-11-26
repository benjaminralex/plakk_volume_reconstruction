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
from numpy import genfromtxt
from optparse import OptionParser
import os
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    # Input files
    parser.add_option("-b", "--bag", dest="bag", help="Location of bag file", metavar="BAG")
    parser.add_option("-t", "--times", dest="times", help="Output location for times", metavar="TIMES")

    # parse the arguments
    (options, args) = parser.parse_args()

    # Read bag file
    bag = rosbag.Bag(options.bag, "r")

    # Poses start and time (only capture in that time frame; this is variable)
    times = []
    
    for topic, msg, t in bag.read_messages(topics='/odometry/filtered'):
        times.append(msg.header.stamp.to_sec())

    bag.close()
    times = np.asarray(times)

    # Save times
    np.save(options.times, times)

    print("camera times created.")
