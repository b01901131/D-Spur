#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <sstream>
#include <math.h>
// #include <sys/time.h>
#include <ctime>
#include <chrono>

using namespace cv;
using namespace std;




int main(){


	VideoCapture cap(1);
    if (!cap.isOpened()){
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    while(1){

        Mat frame;
        bool success = cap.read(frame);
        if(!success) return -1;


        Mat bgr_image = frame;
        cv::Mat lab_image;
	    cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

	    // Extract the L channel
	    std::vector<cv::Mat> lab_planes(3);
	    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

	    // apply the CLAHE algorithm to the L channel
	    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	    clahe->setClipLimit(4);
	    cv::Mat dst;
	    clahe->apply(lab_planes[0], dst);

	    // Merge the the color planes back into an Lab image
	    dst.copyTo(lab_planes[0]);
	    cv::merge(lab_planes, lab_image);

	   // convert back to RGB
	   cv::Mat image_clahe;
	   cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

	   // display the results  (you might also want to see lab_planes[0] before and after).
	   cv::imshow("image original", bgr_image);
	   cv::imshow("image CLAHE", image_clahe);
	   // cv::waitKey();
    }

    
}