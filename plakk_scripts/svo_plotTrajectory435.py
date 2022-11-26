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
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    # Input files
    parser.add_option("-b", "--bag", dest="bag", help="Location of bag file containing poses", metavar="BAG")

    # parse the arguments
    (options, args) = parser.parse_args()

    # Read bag file
    bag = rosbag.Bag(options.bag, "r")

    # Create Pose matrix
    scale = 1.0
    xa = []
    ya = []
    za = []
    x = []
    y = []
    z = []
    times = []
    cx = []
    cy = []
    cz = []
    cxa = []
    cya = []
    cza = []

    # Topics
    for topic, msg, t in bag.read_messages(topics='/odometry/filtered'):
        times.append(t.to_sec())
        x.append(scale*msg.pose.pose.position.x)
        y.append(scale*msg.pose.pose.position.y)
        z.append(scale*msg.pose.pose.position.z)
        xa.append(msg.pose.pose.orientation.x)
        ya.append(msg.pose.pose.orientation.y)
        za.append(msg.pose.pose.orientation.z)
        cx.append(1000*np.sqrt(msg.pose.covariance[0]))
        cy.append(1000*np.sqrt(msg.pose.covariance[6]))
        cz.append(1000*np.sqrt(msg.pose.covariance[12]))

    # close bag
    bag.close()

    # limits
    max_range = np.array([np.array(x).max()-np.array(x).min(), np.array(y).max()-np.array(y).min(), np.array(z).max()-np.array(z).min()]).max() / 2.0
    mid_x = (np.array(x).max()+np.array(x).min()) * 0.5
    mid_y = (np.array(y).max()+np.array(y).min()) * 0.5
    mid_z = (np.array(z).max()+np.array(z).min()) * 0.5


    # Positions
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=True)
    ax1.plot(times, x)

    ax1.set_title('position')
    ax1.set_ylabel('x (m)')
    ax2.plot(times, y)
    ax2.set_ylabel('y (m)')
    ax3.plot(times, z)
    ax3.set_ylabel('z (m)')

    # Fine-tune figure; make subplots close to each other and hide x ticks for
    # all but bottom plot.
    f.subplots_adjust(hspace=0.2)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)

    # Orientations
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=True)
    ax1.plot(times, xa)

    ax1.set_title('orientation')
    ax1.set_ylabel('xa')
    ax2.plot(times, ya)
    ax2.set_ylabel('ya)')
    ax3.plot(times, za)
    ax3.set_ylabel('za')

    # Fine-tune figure; make subplots close to each other and hide x ticks for
    # all but bottom plot.
    f.subplots_adjust(hspace=0.2)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)


    # trajectory
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x, y, z, label='trajectory (m)')
    ax.legend()
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # covariance
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=False)
    ax1.plot(times, cx)
    ax1.set_title('covariances')
    ax1.set_ylabel('cx (mm)')
    ax2.plot(times, cy)
    ax2.set_ylabel('cy (mm)')
    ax3.plot(times, cz)
    ax3.set_ylabel('cz (mm)')

    # Fine-tune figure; make subplots close to each other and hide x ticks for
    # all but bottom plot.
    f.subplots_adjust(hspace=0.2)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)


    plt.show()
