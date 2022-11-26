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
    parser.add_option("-p", "--pointCloud", dest="pointCloud", help="Location of point cloud", metavar="POINTCLOUD")
	
    # parse the arguments
    (options, args) = parser.parse_args()

    # Read bag file
    pointCloud = np.load(options.pointCloud)

    # Create Pose matrix    
    x = pointCloud[:,0]
    y = pointCloud[:,1]
    z = pointCloud[:,2]
    perror = pointCloud[:,3]
    oerror = pointCloud[:,4]
    
    # color 
    col = np.where(1000*perror < 1,'g')
    col = np.where(1000*perror > 1, 'r')
    
    # point cloud    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x, y, z, label='volume', c = col)
    ax.legend()
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')   

    plt.show()
