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
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion as q
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    # Input files
    parser.add_option("-b", "--bag", dest="bag", help="Location of bag file containing poses", metavar="BAG")
    parser.add_option("-t", "--times", dest="times", help="Input location of times for which poses should be saved", metavar="TIMES")
    parser.add_option("-c", "--covariances", dest="covariances", help="Output location of covariances", metavar="COVARIANCES")

    # parse the arguments
    (options, args) = parser.parse_args()

    # Read bag file
    bag = rosbag.Bag(options.bag, "r")

    # Create empty arrays
    times = []

    # Times
    times = np.load(options.times)
    
    # covariances
    covariances = np.zeros((times.shape[0], 6, 6))

    # Create Pose matrix

    # Topics
    for count in range(times.shape[0]):
        for topic, msg, t in bag.read_messages(topics='/odometry/filtered'):
            if msg.header.stamp.to_sec() == times[count]:
                covariances[count, :, :] = np.reshape(np.array(msg.pose.covariance), (-1, 6))
    bag.close()
    np.save(options.covariances, covariances)


    print("covariances saved.")
