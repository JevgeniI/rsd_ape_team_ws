#include <iostream>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"

const int max_H_value = 360/2;
const int max_S_value = 255;
const int max_V_value = 255;

int low_H = 0;
int high_H = max_H_value;
int low_S = 0;
int high_S = max_S_value;
int low_V = 0;
int high_V = max_V_value;

const char* window_original_name = "Original Image";
const char* window_processed_name = "Processed Image";

const char* trackerbar_low_H_name = "Low H value";
const char* trackerbar_high_H_name = "High H value";
const char* trackerbar_low_S_name = "Low S value";
const char* trackerbar_high_S_name = "High S value";
const char* trackerbar_low_V_name = "Low V value";
const char* trackerbar_high_V_name = "High V value";


static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = std::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_processed_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = std::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_processed_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = std::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_processed_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = std::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_processed_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = std::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_processed_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = std::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_processed_name, high_V);
}

int main(int argc, char** argv)
{

    std::string img_name;
    
    if (argc ==1)
    {
        std::cout << "Please provide the path to the image as an argument!" << std::endl;
        \
        return -1;
    }
    else if (argc > 1)
    {
        img_name = argv[1];
        // img_name = img_name + ".jpg";
    }


    cv::Mat src= cv::imread(img_name, cv::IMREAD_COLOR);

    if (src.empty())
    {
        std::cout << "Could not open/find the image" << std::endl;
        
        return -1;
    }


    cv::Mat src_hsv;
    cv::cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);

    cv::Mat dst;




    cv::namedWindow(window_original_name, cv::WINDOW_NORMAL);
    cv::namedWindow(window_processed_name, cv::WINDOW_NORMAL);

    cv::createTrackbar(trackerbar_low_H_name, window_processed_name, &low_H, max_H_value, on_low_H_thresh_trackbar);
    cv::createTrackbar(trackerbar_high_H_name, window_processed_name, &high_H, max_H_value, on_high_H_thresh_trackbar);
    cv::createTrackbar(trackerbar_low_S_name, window_processed_name, &low_S, max_S_value, on_low_S_thresh_trackbar);
    cv::createTrackbar(trackerbar_high_S_name, window_processed_name, &high_S, max_S_value, on_high_S_thresh_trackbar);
    cv::createTrackbar(trackerbar_low_V_name, window_processed_name, &low_V, max_V_value, on_low_V_thresh_trackbar);
    cv::createTrackbar(trackerbar_high_V_name, window_processed_name, &high_V, max_V_value, on_high_V_thresh_trackbar);

    while(true)
    {

        cv::inRange(src_hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), dst);

        imshow(window_original_name, src);
        cv::resizeWindow (window_original_name, 600, 600);
        cv::moveWindow (window_original_name, 0, 0);

        imshow(window_processed_name, dst);
        cv::resizeWindow (window_processed_name, 600, 600);
        cv::moveWindow (window_processed_name, 700, 0);

        char key = (char) cv::waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }

    }
         

    return 0;
}
