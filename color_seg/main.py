import cv2
import numpy as np



# # Red params
low_red = [160, 120, 120]  
high_red = [180, 255, 255]

# # Yellow params
low_yellow = [10, 70, 170] # 20 120 0
high_yellow = [30, 200, 255] # 35 255 255

# # Blue params
low_blue = [100, 120, 120]
high_blue = [140, 255, 255]

def process_img(src, color_name, low, high, k_erosion=3, k_dilation=11, erosion_iterations=5, dilation_iterations=5):

    if color_name == "red":
        color_code = (0, 0, 255)
    if color_name == "yellow":
        color_code = (0, 255, 255)
    if color_name == "blue":
        color_code = (255, 0, 0)
    
    drawing = np.zeros_like(src)

    filter_size = 29
    src = cv2.GaussianBlur(src, (filter_size, filter_size), cv2.BORDER_DEFAULT)
    src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    src = cv2.inRange(src, (low[0], low[1], low[2]), (high[0], high[1], high[2]))
    str_el_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_erosion, k_erosion))
    str_el_dilation = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_dilation, k_dilation))
    src = cv2.dilate(src, str_el_dilation, iterations=dilation_iterations)
    src = cv2.erode(src, str_el_erosion, iterations=erosion_iterations)
    binary = src

    thresh = 50
    canny = cv2.Canny( src, thresh, thresh*2, 3 )
    contours, hierarchy = cv2.findContours( canny, cv2.CHAIN_APPROX_SIMPLE, cv2.RETR_TREE)[-2:]

    minRect = [cv2.minAreaRect(contour) for contour in contours]

    for i in range(len(contours)):
        cv2.drawContours(drawing, contours, i, color_code, 2, 8, hierarchy, ) 
        
        box = cv2.boxPoints(minRect[i])
        box = np.int0(box)
        for j in range(4):
            cv2.line( drawing, tuple(box[j]), tuple(box[(j+1)%4]), (128,0,128), 3, 8)
        # cv2.drawContours(drawing,[box],0,(0,255,255),2)
        cv2.putText(drawing, color_name, tuple(box[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color_code)

    return binary, drawing

if __name__ == "__main__":
    img = cv2.imread("./lego_images/4.jpg")
    img = img[1000:2000, 700:2500]
    # binary, dst = process_img(img, "red", low_red, high_red)
    # binary, dst = process_img(img, "yellow", low_yellow, high_yellow)
    binary, dst = process_img(img, "blue", low_blue, high_blue)

    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 400,400)
    cv2.moveWindow ('image', 0, 0)
    cv2.imshow('image', img)

    cv2.namedWindow('binary',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('binary', 400,400)
    cv2.moveWindow ('binary', 0, 400)
    cv2.imshow('binary', binary)

    cv2.namedWindow('processed',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('processed', 800, 800)
    cv2.moveWindow ('processed', 450, 0)
    cv2.imshow('processed', dst)
    

    key = cv2.waitKey(30)
    while(key != 27 or key != "q"):
        pass