#!/usr/bin/env python
import cv2
import cv
import numpy as np
import math
import time
from random import randint

# Hardcoded values for green android folder
LOW_HSV = [60, 50, 50]

HIGH_HSV = [90, 255, 255]

#GLOBAL IMAGES
hsv_img = None
bin_img = None

# Get binary thresholded image
# low_HSV, hi_HSV - low, high range values for threshold as a list [H,S,V]
# debug= True to display the binary image generated
def get_binary(src_img, low_HSV, hi_HSV, debug=False):
    global hsv_img, bin_img
    #convert to HSV
    hsv_img = cv2.cvtColor(src_img, cv.CV_BGR2HSV)
    #generate binary image
    lower = np.array(low_HSV)
    higher = np.array(hi_HSV)
    bin_img = cv2.inRange(hsv_img, lower, higher)
    if debug:
        cv2.namedWindow("Binary")
        cv2.imshow("Binary",bin_img)
        cv2.waitKey(0)
        cv2.destroyWindow("Binary")
        
    return bin_img

def main():
    global tempImg, tempImgForReal, hsv_img, bin_img, binaryImg

    # TODO: hook in camera using cv.VideoCapture(); cam.open(deviceId)
    M = cv.LoadImageM('straight.jpg')
    cv2.namedWindow("Test",0)

    a = np.asarray(M)
    b = get_binary(a, LOW_HSV, HIGH_HSV)

    imgSize = np.shape(M)

    # NOTE: Assumes an accurate color/marker detection at the very top row of the image
    start = -1
    end = -1
    row = 0

    for j in range(imgSize[1]):
        if start < 0 and b[row,j] != 0:
            start = j
        if end < 0 and start >= 0 and b[row,j] == 0:
            end = j
        if (start >= 0 and end >= 0):
            break

    det_width = (end - start) / 2
    tgt_width = imgSize[1] / 2
    error_px  = (start + det_width) - tgt_width
    error_rad = math.tan(float(error_px)/imgSize[0])
    print start, end
    print det_width, tgt_width
    print error_px, error_rad
    cv2.imshow("Test", b)                                    
    c = cv2.waitKey(100000)

if __name__ == '__main__':
    main()
