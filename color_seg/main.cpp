#include <iostream>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <unistd.h>



// // Red params
int low_red[] = {20, 0, 0};    
int high_red[] = {180, 255, 255};
// int k_erode = 5;
// int k_dilate = 11;
// int erosion_iterations = 5;
// int dilation_iterations = 2;

// // Yellow params
int low_yellow[] = {10, 70, 0};    
int high_yellow[] = {30, 255, 255};
// int k_erode = 5;
// int k_dilate = 11;
// int erosion_iterations = 5;
// int dilation_iterations = 2;

// // Blue params
int low_blue[] = {100, 120, 0};    
int high_blue[] = {140, 255, 255};
// int k_erode = 5;
// int k_dilate = 11;
// int erosion_iterations = 5;
// int dilation_iterations = 2;

cv::RNG rng(12345);

const int max_H_value = 360/2;
const int max_S_value = 255;
const int max_V_value = 255;

int low_H = 0;
int high_H = max_H_value;
int low_S = 0;
int high_S = max_S_value;
int low_V = 0;
int high_V = max_V_value;
int kernel_size_erosion = 1;
int kernel_max_erosion = 29;
int kernel_size_dilation = 1;
int kernel_max_dilation = 29;

const char* window_original_name = "Original Image";
const char* window_processed_name = "Processed Image";
const char* window_eroded_name = "Eroded";
const char* window_dilated_name = "Eroded";

const char* trackerbar_low_H_name = "Low H value";
const char* trackerbar_high_H_name = "High H value";
const char* trackerbar_low_S_name = "Low S value";
const char* trackerbar_high_S_name = "High S value";
const char* trackerbar_low_V_name = "Low V value";
const char* trackerbar_high_V_name = "High V value";
const char* trackerbar_erosion_kernel_size = "Kernel Size Erosion";
const char* trackerbar_dilation_kernel_size = "Kernel Size Dilation";


cv::Mat src_hsv;
cv::Mat dst;
cv::Mat eroded;
cv::Mat dilated;

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

void process_img(cv::Mat& src, int low[], int high[], int k_erosion=5, int k_dilation=5, int erosion_iterations = 5, int dilation_iterations = 5)
{
    int filter_size = 9;
    cv::Mat src_hsv;
    cv::GaussianBlur(src, src, cv::Size(filter_size, filter_size), 0);
    cv::cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);
    cv::Mat dst;
    cv::inRange(src_hsv, cv::Scalar(low[0], low[1], low[2]), cv::Scalar(high[0], high[1], high[2]), dst);
    cv::Mat structuring_element_erosion = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k_erosion, k_erosion));
    cv::Mat structuring_element_dilation = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k_dilation, k_dilation));
    cv::dilate(dst, dst, structuring_element_dilation, cv::Point(-1, -1), dilation_iterations);
    cv::erode(dst, dst, structuring_element_erosion, cv::Point(-1, -1), erosion_iterations);    
    // cv::dilate(dst, dst, structuring_element_dilation, cv::Point(-1, -1), 3);

    src = dst;
}


bool is_yellow_brick(cv::Mat yellow_img)
{
    for (int i = 0; i < yellow_img.rows; i++)
    {
        for (int j = 0; j < yellow_img.cols; j++)
        {
            return -1;
        }
    }
}

int main(int argc, char** argv)
{

    // std::cout << "\nNote that the reflections of the bricks off the walls \
    //     of the container are also counted!!" << std::endl;
    // std::vector<cv::String> fn;
    // cv::glob("../lego_images/*.jpg", fn, false);

    // std::vector<cv::Mat> images;
    // size_t count = fn.size(); //number of png files in images folder
    // for (size_t i=0; i<count; i++)
    //     images.push_back(cv::imread(fn[i], cv::IMREAD_COLOR));


    // for (size_t i=0; i< images.size(); i++)
    // {
    //     if (images[i].empty())
    //         {
    //             std::cout << "Could not open/find the image" << std::endl;
                
    //             return -1;
    //         }

        
    //     process_img(images[i], low_red, high_red); //, k_erode, k_dilate, erosion_iterations, dilation_iterations);
    //     std::string dst_path = "../processed_images/" + std::to_string(i+1);
    //     cv::imwrite(dst_path + ".jpg", images[i]);
    //     std::cout << dst_path << std::endl;

    // }

    cv::namedWindow(window_original_name, cv::WINDOW_NORMAL);
    cv::namedWindow(window_processed_name, cv::WINDOW_NORMAL);

    // cv::createTrackbar(trackerbar_low_H_name, window_processed_name, &low_H, max_H_value, on_low_H_thresh_trackbar);
    // cv::createTrackbar(trackerbar_high_H_name, window_processed_name, &high_H, max_H_value, on_high_H_thresh_trackbar);
    // cv::createTrackbar(trackerbar_low_S_name, window_processed_name, &low_S, max_S_value, on_low_S_thresh_trackbar);
    // cv::createTrackbar(trackerbar_high_S_name, window_processed_name, &high_S, max_S_value, on_high_S_thresh_trackbar);
    // cv::createTrackbar(trackerbar_low_V_name, window_processed_name, &low_V, max_V_value, on_low_V_thresh_trackbar);
    // cv::createTrackbar(trackerbar_high_V_name, window_processed_name, &high_V, max_V_value, on_high_V_thresh_trackbar);



    cv::VideoCapture vcap;

    auto capture = vcap.open("http://10.42.0.96:8000/stream.mjpg");
    if (!capture)
    {
        std::cout << "Video not captured!" << std::endl;
        return EXIT_FAILURE;
    }

    cv::Mat frame;
    cv::Mat src, red_img, yellow_img, blue_img;
    // x 150-500
    // y 60-360
    cv::Range rows(60, 360);
    cv::Range cols(150, 500);
    while(capture)
    {
        // std::cout << "Finally.." << std::endl;
        vcap.read(frame);
        frame = frame(rows, cols);
        src = frame.clone();
        red_img = frame.clone();
        yellow_img = frame.clone();
        blue_img = frame.clone();
        int low_new[] = {low_H, low_S, low_V};
        int high_new[] = {high_H, high_S, high_V};
        process_img(red_img, low_red, high_red); // , k_erode, k_dilate, erosion_iterations, dilation_iterations);
        bitwise_not(red_img, red_img);
        process_img(yellow_img, low_yellow, high_yellow);
        process_img(blue_img, low_blue, high_blue);
        
        cv::bitwise_or(red_img, yellow_img, frame);
        cv::bitwise_or(frame, blue_img, frame);
        
        // Cotours
        cv::Mat canny_output;
        int thresh = 50;
        cv::Canny( frame, canny_output, thresh, thresh*2, 3 );
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        cv::minAreaRect(contours);
        
        /// Draw contours
        cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
            {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
            }


        // Display
        cv::imshow(window_original_name, src);
        cv::resizeWindow (window_original_name, 600, 600);
        cv::moveWindow (window_original_name, 0, 0);

        cv::imshow(window_processed_name, frame);
        cv::resizeWindow (window_processed_name, 600, 600);
        cv::moveWindow (window_processed_name, 700, 0);

        /// Show in a window
        cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        cv::imshow( "Contours", drawing );


        char key = (char) cv::waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
        // cv::imwrite("/home/mohamed/Desktop/frame.png", frame);
        // usleep(3000);
    }
    

    return 0;

}