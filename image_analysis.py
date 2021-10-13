
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class ImageAnalysis:
    def __init__(self, img_size=(480, 640), min_blob=50):
        self.image = np.empty(img_size, np.uint8)
        self.min_blob = min_blob
        self.sub = rospy.Subscriber("rpi_image", CompressedImage, self.analyze_image)

    def imfill(self, bw_image):
        h, w = bw_image.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        im_filled = bw_image.copy()
        cv2.floodFill(im_filled, mask, (0, 0), 255)
        im_filled_inv = cv2.bitwise_not(im_filled)
        return bw_image | im_filled_inv

    def refine_mask(self, raw_mask):
        filled_mask = self.imfill(raw_mask)
        kernel = cv2.getStructingElement(shape=cv2.MORPH_ELLIPSE, ksize=(self.min_blob, self.min_blob))
        opened_mask = cv2.morphologyEx(filled_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(opened_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.mask = np.zeros(raw_mask.shape, np.uint8)
        if len(contours) != 0:
            largest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(self.mask, [largest_contour], -1, 255, cv2.FILLED)
            m = cv2.moments(largest_contour)
            self.disease_cx = int(m['m10'] / m['m00'])
            self.disease_cy = int(m['m01'] / m['m00'])
            self.disease_found = True
        else:
            self.disease_found = False

    def make_mask(self):
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 130, 20])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([165, 130, 20])
        upper2 = np.array([179, 255, 255])
        lower_mask = cv2.inRange(hsv_image, lower1, upper1)
        upper_mask = cv2.inRange(hsv_image, lower2, upper2)
        raw_mask = lower_mask + upper_mask
        self.refine_mask(raw_mask)

    def analyze_image(self, msg):
        rospy.loginfo(msg.header)
        bridge = CvBridge()
        try:
            self.image = bridge.compressed_imgmsg_cv2(msg, "bgr8")
            rospy.loginfo("Image was received")

        except CvBridgeError as e:
            print(e)
        self.make_mask()
        masked_image = cv2.bitwise_and(self.image, self.image, mask=self.mask)
        if self.disease_found:
            rospy.loginfo('Area:{self.disease_area:.0f},'
                          'cx:{self.disease_cx:.2f},'
                          'cy:{self.disease_cy:.2f},')
        cv2.imshow('orginal', self.image)
        cv2.imshow("masked", masked_image)
        key = cv2.waitKey(0)
        if key == 27 or key == ord('q'):
            cv2.destoryALLWindows()
        rospy.loginfo("image was analyze")

if __name__ == "__main__":
    rospy.init_node("image_analysis")
    ImageAnalysis()
    rospy.spin()
