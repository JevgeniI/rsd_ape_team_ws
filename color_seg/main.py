import cv2
import numpy as np


# void process_img(cv::Mat& src, cv::Mat& drawing, std::string color_name, int low[], int high[], int k_erosion=5, int k_dilation=5, int erosion_iterations = 5, int dilation_iterations = 5)
# {

#     /// Draw contours
#     // cv::Mat drawing = cv::Mat::zeros( canny.size(), CV_8UC3 );
#     for( int i = 0; i< contours.size(); i++ )
#     {
#         // draw contour    
#         cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
#         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
#         // rotated rectangle
#         cv::Point2f rect_points[4]; minRect[i].points( rect_points );
#         for( int j = 0; j < 4; j++ )
#             line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
#         cv::putText(drawing, color_name, rect_points[1], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
#     }


# }


# # Red params
low_red = [160, 120, 0]  
high_red = [180, 255, 255]

# # Yellow params
low_yellow = [10, 120, 0] # 20 120 0
high_yellow = [30, 255, 255] # 35 255 255

# # Blue params
low_blue = [100, 120, 0]
high_blue = [140, 255, 255]

def process_img(src, color_name, low, high, k_erosion=5, k_dilation=5, erosion_iterations=5, dilation_iterations=5):

    filter_size = 29
    src = cv2.GaussianBlur(src, (filter_size, filter_size), cv2.BORDER_DEFAULT)
    src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    src = cv2.inRange(src, (low[0], low[1], low[2]), (high[0], high[1], high[2]))
    str_el_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_erosion, k_erosion))
    str_el_dilation = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_dilation, k_dilation))
    src = cv2.dilate(src, str_el_dilation, iterations=dilation_iterations)
    src = cv2.erode(src, str_el_erosion, iterations=erosion_iterations)

    thresh = 50
    canny = cv2.Canny( src, thresh, thresh*2, 3 )
    contours, hierarchy = cv2.findContours( canny, cv2.CHAIN_APPROX_SIMPLE, cv2.RETR_EXTERNAL)

    minRect = [cv2.minAreaRect(contour) for contour in contours]

    drawing = np.zeros_like(src)

    # for i in range(len(contours)):
    cv2.drawContours(drawing, contours, -1, (255, 255, 255), 2, 8, hierarchy) 

        # box = cv2.cv.BoxPoints(minRect[i])
        # box = np.int0(box)
        # cv2.drawContours(drawing,[box],0,(0,0,255),2)
        # cv2.putText(drawing, color_name, box[1], cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    return drawing

if __name__ == "__main__":
    img = cv2.imread("/home/mohamed/rsd_ape_team_ws/color_seg/lego_images/1.jpg")
    dst = process_img(img, "yellow", low_yellow, high_yellow)

    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 600,600)
    cv2.imshow('image', dst)
    

    key = cv2.waitKey(30)
    while(key != 27 or key != "q"):
        pass