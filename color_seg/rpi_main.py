import cv2
import numpy as np
from collections import deque
import pymodbus
import modbus.pymodbus_server as mbs


from multiprocessing import Queue, Process
import threading
import time

# # Red params
low_red = [179, 120, 0]  
high_red = [180, 255, 255]

# # Yellow params
low_yellow = [10, 120, 0] # 20 120 0
high_yellow = [35, 255, 255] # 35 255 255

# # Blue params
low_blue = [100, 120, 0]
high_blue = [140, 255, 255]

def process_img_binary(src, low, high, k_erosion=3, k_dilation=5, erosion_iterations=5, dilation_iterations=2):

    filter_size = 29
    src = cv2.GaussianBlur(src, (filter_size, filter_size), cv2.BORDER_DEFAULT)
    src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    src = cv2.inRange(src, (low[0], low[1], low[2]), (high[0], high[1], high[2]))
    str_el_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_erosion, k_erosion))
    str_el_dilation = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_dilation, k_dilation))
    src = cv2.dilate(src, str_el_dilation, iterations=dilation_iterations)
    src = cv2.erode(src, str_el_erosion, iterations=erosion_iterations)
    binary = src

    return binary


def process_img_contours(binary_img, color_name):

    if color_name == "red":
        color_code = (0, 0, 255)
        min_threshold_area = 2500
        max_threshold_area = 3500
    if color_name == "yellow":
        color_code = (0, 255, 255)
        min_threshold_area = 6500
        max_threshold_area = 10000
    if color_name == "blue":
        color_code = (255, 0, 0)
        min_threshold_area = 1100
        max_threshold_area = 1800
    
    drawing = np.zeros(binary_img.shape + (3,))

    src = np.uint8(binary_img)
    thresh = 50
    canny = cv2.Canny( src, thresh, thresh*2, 3 )
    contours, hierarchy = cv2.findContours( canny, cv2.CHAIN_APPROX_SIMPLE, cv2.RETR_TREE)[-2:]

    minRect = [cv2.minAreaRect(contour) for contour in contours]
    
    # for i in range(len(contours)):
    #     # Should the -1 be i or not?
    #     cv2.drawContours(drawing, contours, i, color_code, 2, 8)# , hierarchy[:][0]) 
        
    #     box = cv2.boxPoints(minRect[i])
    #     box = np.int0(box)
    #     for j in range(4):
    #         cv2.line( drawing, tuple(box[j]), tuple(box[(j+1)%4]), (128,0,128), 3, 8)
    #     cv2.putText(drawing, color_name, tuple(box[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color_code)

    
    
    if (len(contours) > 0):
        area = cv2.contourArea(contours[0])
        # print(area)
        if (color_name == "red" and area > min_threshold_area and area < max_threshold_area):
            cv2.drawContours(drawing, contours[0], -1, color_code, 2, 8)#  , hierarchy[0][0]) 
            box = cv2.boxPoints(minRect[0])
            box = np.int0(box)
            for j in range(4):
                cv2.line( drawing, tuple(box[j]), tuple(box[(j+1)%4]), (128,0,128), 3, 8)
            cv2.putText(drawing, color_name, tuple(box[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color_code)
            print("{} brick deteced!".format(color_name))
            mbs.set_register_value(0x00, 1)

        elif (color_name == "yellow" and area > min_threshold_area and area < max_threshold_area):
            cv2.drawContours(drawing, contours[0], -1, color_code, 2, 8)#  , hierarchy[0][0]) 
            box = cv2.boxPoints(minRect[0])
            box = np.int0(box)
            for j in range(4):
                cv2.line( drawing, tuple(box[j]), tuple(box[(j+1)%4]), (128,0,128), 3, 8)
            cv2.putText(drawing, color_name, tuple(box[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color_code)
            print("{} brick deteced!".format(color_name))
            mbs.set_register_value(0x00, 2)
        elif (color_name == "blue" and area > min_threshold_area and area < max_threshold_area):
            cv2.drawContours(drawing, contours[0], -1, color_code, 2, 8)#  , hierarchy[0][0]) 
            box = cv2.boxPoints(minRect[0])
            box = np.int0(box)
            for j in range(4):
                cv2.line( drawing, tuple(box[j]), tuple(box[(j+1)%4]), (128,0,128), 3, 8)
            cv2.putText(drawing, color_name, tuple(box[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color_code)
            print("{} brick deteced!".format(color_name))
            mbs.set_register_value(0x00, 3)
        else:
            print("No bricks detected")
            mbs.set_register_value(0x00, 4)


    return drawing

if __name__ == "__main__":

    server = threading.Thread(target=mbs.run_sync_server, args=())
    server.daemon = True
    server.start()
    mbs.set_register_value(0x00, 0)

    cap = cv2.VideoCapture(0)
    if not cap:
        print("Nothing captured")

    red_deque = deque(maxlen=10)
    yellow_deque = deque(maxlen=10)
    blue_deque = deque(maxlen=10)
    
    while(True):
        red_count = 0
        yellow_count = 0
        blue_count = 0

        ret, frame = cap.read()
        frame = frame[200:400, 150:450]
        drawing = np.zeros_like(frame)

        binary_red = process_img_binary(frame, low_red, high_red)
        # binary_red = cv2.bitwise_not(binary_red)
        binary_yellow = process_img_binary(frame, low_yellow, high_yellow)
        binary_blue = process_img_binary(frame, low_blue, high_blue)
        
        red_deque.append(binary_red)
        yellow_deque.append(binary_yellow)
        blue_deque.append(binary_blue)
        binary_red = np.average(red_deque, axis=0)
        binary_yellow = np.average(yellow_deque, axis=0)
        binary_blue = np.average(blue_deque, axis=0)

        dst_red = process_img_contours(binary_red, "red")
        dst_yellow = process_img_contours(binary_yellow, "yellow")
        dst_blue = process_img_contours(binary_blue, "blue")

        binary_all = binary_red + binary_yellow + binary_blue
        dst_all = dst_red + dst_yellow + dst_blue


#         cv2.namedWindow('Feed',cv2.WINDOW_NORMAL)
#         cv2.resizeWindow('Feed', 400,400)
#         cv2.moveWindow ('Feed', 0, 0)
#         cv2.imshow('Feed', frame)

#         cv2.namedWindow('binary',cv2.WINDOW_NORMAL)
#         cv2.resizeWindow('binary', 400,400)
#         cv2.moveWindow ('binary', 0, 400)
#         cv2.imshow('binary', binary_all)

#         cv2.namedWindow('processed',cv2.WINDOW_NORMAL)
#         cv2.resizeWindow('processed', 800, 800)
#         cv2.moveWindow ('processed', 450, 0)
#         cv2.imshow('processed', dst_all)

#         key = cv2.waitKey(1)
#         while(key & 0xFF == ord('q')):
#             break
    
#     cap.release()
#     cv2.destroyAllWindows()
