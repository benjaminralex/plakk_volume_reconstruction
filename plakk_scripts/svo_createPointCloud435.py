# import libraries
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import time
import scipy.spatial
import scipy.io as sio
from scipy import signal
from scipy.optimize import minimize
import transformations
from numpy import genfromtxt
import serial
from optparse import OptionParser
import roslaunch
import rospy

if __name__ == "__main__":

    # parse the arguments
    parser = OptionParser()

    parser.add_option("-i", "--usImages", dest="usImages", help="Location of interpolated ultrasound images", metavar="USIMAGES")
    parser.add_option("-c", "--camPoses", dest="camPoses", help="Location of interpolated camera poses", metavar="CAMPOSES")
    parser.add_option("-d", "--cov", dest="cov", help="Location of interpolated covariances", metavar="COV")
    parser.add_option("-p", "--pointCloud", dest="pointCloud", help="Location for storing point cloud as npy", metavar="POINTCLOUD")
    parser.add_option("-t", "--text", dest="text", help="Saving location of point cloud text file", metavar="TEXT")

    # parse the arguments
    (options, args) = parser.parse_args()

    # Load images (number of images, rows, cols)
    usImages = np.load(options.usImages)
    camPoses = np.load(options.camPoses)
    cov = np.load(options.cov)

    # room imaging (user input)
    # need to update this based on CAD and DICOM information
    ppmm = 0.0238/100.0
    us2Cam = np.array([[0.0, 0.0, -28.56/1000.0], [-ppmm, 0.0, 0.0/1000.0], [0.0, -ppmm, -75.62/1000.0], [0, 0, 1.0]])


    # poses
    cam2World = camPoses

    # initilaize slices
    slices = usImages

    # Create point cloud array of non zeros only
    pointCloud = np.zeros((np.count_nonzero(slices),5))
    # NOTE: I don't know why but flatten() will break this program. Since there is a count_nonzero(), the result would not change without flatten()
    count = 0

    # Fill it in
    for i in range(0,slices.shape[0]-1,1):
	# print(i)

        # current covariance
        currentCov = cov[i,:,:]

        # current pose
        currentPose = cam2World[i,:,:]

        #current image
        selected = slices[i,:,:]

        # States
        states = selected > 0
        row, col = np.where(states)

        for r, c in zip(row, col):

            # offsets for room
            co = 425.0
            ro = 64.0
            ref = 26.0
            ro = ro + ref

            # subtract offsets
            c = c - co
            r = r - ro
            pixel = np.array([c, r, 1])

            # The current pixel location (x, y, 1) --> (col, row, 1)
            imagePixel = np.reshape(pixel, (3,1))

            # Transform the current pixel

            worldPixel = np.matmul(currentPose, np.matmul(us2Cam,imagePixel))

            pointCloud[count, 0] =  worldPixel[0]
            pointCloud[count, 1] =  worldPixel[1]
            pointCloud[count, 2] =  worldPixel[2]
            count = count + 1


    np.save(options.pointCloud, pointCloud)

    # Write to text file
    allPoints = pointCloud
    text_file1 = open(options.text, "w")

    # Header file
    text_file1.write("# .PCD v0.7 - Point Cloud Data file format" + "\n")
    text_file1.write("VERSION 0.7" + "\n")
    text_file1.write("FIELDS x y z" + "\n")
    text_file1.write("SIZE 4 4 4" + "\n")
    text_file1.write("TYPE F F F" + "\n")
    text_file1.write("COUNT 1 1 1" + "\n")
    text_file1.write("WIDTH" + " " + str(allPoints.shape[0]) + "\n")
    text_file1.write("HEIGHT 1" + "\n")
    text_file1.write("VIEWPOINT 0 0 0 1 0 0 0" + "\n")
    text_file1.write("POINTS" + " " + str(allPoints.shape[0]) + "\n")
    text_file1.write("DATA ascii" + "\n")

    # Write to files
    for j in range(allPoints.shape[0]):
        text_file1.write(str(allPoints[j,0]) + " " + str(allPoints[j,1])  + " " + str(allPoints[j,2])  + "\n")

    text_file1.close()

    print("point cloud created.")
