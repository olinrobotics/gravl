#!/usr/bin/env python
#     _/    _/                                _/                _/      _/
#    _/  _/      _/_/_/  _/      _/      _/      _/_/_/        _/_/    _/
#   _/_/      _/    _/  _/      _/      _/  _/  _/    _/      _/  _/  _/
#  _/  _/    _/    _/    _/  _/  _/  _/    _/  _/    _/      _/    _/_/
# _/    _/    _/_/_/      _/      _/      _/  _/    _/      _/      _/
# Supposed to display a transformed image from the camera, using the
# recognize_road function.

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import numpy as np


class road_recognition:
    def __init__(self):
        self.image = rospy.Subscriber(
            '/camera/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('trapezoid', Image, queue_size=10)

    def norm(self, x):
        '''Finds the y-value of a normal curve centered at the mean
        (mu) road width and standard deviation sigma. This is used to
        make some road locations more prominent than others.'''
        mu = 550
        sigma = 100
        return np.e**((-(x - mu) ** 2) / (2 * sigma ** 2)) / (np.sqrt(2 * np.pi * sigma ** 2))

    def trap(self, img, line, m, n):
        '''Returns mean intensity within a trapezoid with the base on
        the bottom of the image. Takes an image, a number of pixels
        above the bottom of the image (line) the top of the trapezoid
        should be, a left point (m) an right (n) on that line, and
        calculates the average pixel intensity of the region inside
        the trapezoid'''
        white = 0
        total = 0
        for r in range(line, img.shape[0], 10):  # check every 10
            # pixels, otherwise this will take too long to run
            for c in range(int(r * m / (line - (img.shape[0])) + m * (img.shape[0]) / ((img.shape[0]) - line)),
                           int((r * (n - img.shape[1]) + line * img.shape[1] -
                                n * img.shape[0]) / (line - img.shape[0])),
                           10):
                white += img[r][c]
                total += 1
        return (white + 0.0) / total * self.norm(n - m)  # multiply by
        # normal curve to favor m and n some distance apart

    def recognize_road(self, img):
        '''The function used by the node to find the road.'''

        # Look for a road 1/3 of the image from the bottom
        line = int(img.shape[0] * 2 / 3)

        maxmn = 0
        maxm = 0
        maxn = 0

        # Sweep possible top vertices for the trapezoid in the trap method, and keep the most intense one.
        # check every 100 pixels for speed.
        for m in range(0, img.shape[1], 100):
            for n in range(m, img.shape[1], 100):
                val = self.trap(img, line, m, n)
                if val > maxmn:
                    maxmn = val
                    maxm = m
                    maxn = n

        cv2.line(img, (maxm, line), (0, img.shape[0]), 128, 10)
        cv2.line(img, (maxn, line), (img.shape[1], img.shape[0]), 128, 10)
        cv2.line(img, (maxm, line), (maxn, line), 128, 10)
        return img

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'mono8')
        except CvBridgeError as e:
            print(e)

        road = self.recognize_road(cv_image)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(road, "mono8"))
        except CvBridgeError as e:
            print(e)
        cv2.imshow('Image window', road)
        cv2.waitKey(1)


def main():
    road_recognition()
    rospy.init_node('road_recognition', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')


if __name__ == '__main__':
    main()
