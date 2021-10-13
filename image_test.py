#-*- coding:utf-8 -*-

''' References
    https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
    https://github.com/markwsilliman/turtlebot/blob/master/take_photo.py 
    
    by Hoeun Lee (202011353) 2020.11.15.                                        '''

from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def left_ROI(img):
    poly = np.array([
        [(0,260),(0,520),(110,520),(110,260)]
        ])
    BG = np.zeros_like(img)
    cv2.fillConvexPoly(BG,poly,255)
    masked_img = cv2.bitwise_and(img,BG)
    return masked_img

def right_ROI(img):
    poly = np.array([
        [(655,260),(655,490),(745,490),(745,260)]
        ])
    BG = np.zeros_like(img)
    cv2.fillConvexPoly(BG,poly,255)
    masked_img = cv2.bitwise_and(img,BG)
    return masked_img


def HSV_yellow(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low = np.array([70, 210, 220])
    high = np.array([90, 230, 255])
    yellowmask = cv2.inRange(hsv, low, high)
    return yellowmask


class CompressedImages:
    ''' Constructor '''
    def __init__(self):
        
        # Bridge Instance
        self.bridge = CvBridge()
        
        # Get Compressed Image Topic from Subscriber
        img_topic = "/raspicam_node/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback)

    ''' Callback Method '''
    def callback(self, data):
        try:
            # Convert Image to OpenCV Format (Compressed Image Message to CV2)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Rotate 180 deg.
        height, width = cv_image.shape[0], cv_image.shape[1]
        rotatedMatrix = cv2.getRotationMatrix2D((width // 2, height // 2), 180, 1)
        dst = cv2.warpAffine(cv_image, rotatedMatrix, (width, height))
        
        ''' Image Processing '''
        # 빨간색 필터
        low1 = np.array([0, 130, 20])
        high1 = np.array([10, 255, 255])
	low2 = np.array([165, 130, 20])
        high2 = np.array([179, 255, 255])
	lower_mask = cv2.inRange(dst, low1, high1)
        higher_mask = cv2.inRange(dst, low2, high2)
        redmask = lower_mask + higher_mask
        #yellowmask = cv2.inRange(dst, low, high)
        # Canny Edge Detection
        redCanny = cv2.Canny(redmask, 50, 500, apertureSize=5, L2gradient=True)

        # 흰색 필터
        low = np.array([248, 250, 248])
        high = np.array([255, 255, 255])
        whitemask = cv2.inRange(dst, low, high)
        # Canny Edge Detection
        whiteCanny = cv2.Canny(whitemask, 50, 500, apertureSize=5, L2gradient=True)
        
        # 합치기
        #lanedtc = cv2.bitwise_or(yellowCanny, whiteCanny)

        # 허프 변환으로 선형 도형 검출
        lines = cv2.HoughLinesP(redCanny, 1, np.pi/180, 100, np.array([]), minLineLength=1, maxLineGap=20)
	


        # 라인들의 평균 계산
        averaged_lines = average_slope(dst,lines)
        line_image = display_square(dst,averaged_lines)
        combo_image = cv2.addWeighted(dst, 0.6, line_image, 1, 1)

        # Show Image
        cv2.imshow("/raspicam_node/image/compressed", combo_image)	
	#cv2.imwrite('~/capture.jpg', combo_image)
        cv2.waitKey(1)



def display_square(img,lines):
    BG = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line.reshape(4)
            cv2.line(BG, (x1,y1), (x2,y2), (0, 255, 0), 5)
            
    return BG


def average_slope(img,lines):
    left_fit = []
    right_fit = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
        
            if slope < 0:
                left_fit.append((slope,intercept))
            else:
                right_fit.append((slope,intercept))

        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)

        left_line = make_coordinates(img, left_fit_average)
        right_line = make_coordinates(img, right_fit_average)

        return np.array([left_line, right_line])


def make_coordinates(img,coordinate):
    slope,intercept = coordinate
    y1 = img.shape[0]
    y2 = int(y1*(1/5))
    x1 = int((y1-intercept)/slope)
    x2 = int((y2-intercept)/slope)
    print(x1,y1,x2,y2)
    return np.array([x1, y1, x2, y2])


if __name__ == '__main__':
    # Initialize
    camera = CompressedImages()
    rospy.init_node('CompressedImages', anonymous=False)
    rospy.spin()
