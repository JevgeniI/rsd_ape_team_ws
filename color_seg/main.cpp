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

void process_img(cv::Mat& src, int low[], int high[], int k_erosion=5, int k_dilation=21, int erosion_iterations = 5, int dilation_iterations = 3)
{
    int filter_size = 51;
    cv::Mat src_hsv;
    cv::GaussianBlur(src, src, cv::Size(filter_size, filter_size), 0);
    cv::cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);
    cv::Mat dst;
    cv::inRange(src_hsv, cv::Scalar(low[0], low[1], low[2]), cv::Scalar(high[0], high[1], high[2]), dst);
    cv::Mat structuring_element_erosion = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_erosion, k_erosion));
    cv::Mat structuring_element_dilation = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_dilation, k_dilation));
    cv::erode(dst, dst, structuring_element_erosion, cv::Point(-1, -1), 5);    
    cv::dilate(dst, dst, structuring_element_dilation, cv::Point(-1, -1), 3);

    src = dst;
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


    // // Red params
    // // int low[] = {160, 120, 120};    
    // // int high[] = {180, 255, 255};
    // // int k_erode = 5;
    // // int k_dilate = 11;
    // // int erosion_iterations = 5;
    // // int dilation_iterations = 2;

    // // Yellow params
    // // int low[] = {10, 70, 150};    
    // // int high[] = {30, 255, 255};
    // // int k_erode = 5;
    // // int k_dilate = 11;
    // // int erosion_iterations = 5;
    // // int dilation_iterations = 2;

    // // Blue params
    // int low[] = {100, 120, 120};    
    // int high[] = {140, 255, 255};
    // int k_erode = 5;
    // int k_dilate = 11;
    // int erosion_iterations = 5;
    // int dilation_iterations = 2;

    // for (size_t i=0; i< images.size(); i++)
    // {
    //     if (images[i].empty())
    //         {
    //             std::cout << "Could not open/find the image" << std::endl;
                
    //             return -1;
    //         }

        
    //     process_img(images[i], low, high, k_erode, k_dilate, erosion_iterations, dilation_iterations);
    //     std::string dst_path = "../processed_images/" + std::to_string(i+1);
    //     cv::imwrite(dst_path + ".jpg", images[i]);
    //     std::cout << dst_path << std::endl;

    // }


    cv::VideoCapture vcap;

    // std::cout << vcap.open(10.42.0.96:8000/index.html/?action=stream?dummy=param.mjpg) << std::endl;

    // if(!vcap.open("http://@10.42.0.96:8000"))
    auto capture = vcap.open("http://10.42.0.96:8000/stream.mjpg");
    if (!capture)
    {
        std::cout << "Video not captured!" << std::endl;
        return EXIT_FAILURE;
    }


    cv::Mat frame;


    while(capture)
    {
        std::cout << "Finally.." << std::endl;
        vcap.read(frame);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        cv::threshold(frame, frame, 150, 255, 0);
        cv::imwrite("/home/mohamed/Desktop/frame.png", frame);
        usleep(3000);
    }
    

    return 0;

}