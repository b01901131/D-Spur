#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat img, img_grey, img_canny;

int main( int argc, char** argv ){
    

    img = imread("../img/puzzle2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    //cout << img.cols/2 <<" , "<<img.rows/2<<endl;
    //cv::resize( img, img, cv::Size(img.cols / 2, img.rows / 2) );
    if(! img.data ){
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    imshow("img",img);
    //waitKey(0);

    cvtColor( img, img_grey, CV_BGR2GRAY );
    imshow("grey",img_grey);
    waitKey(0);  
    //threshold(img_grey, img_grey, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    //imshow("grey",img_grey);
    //waitKey(0);  

    Canny(img_grey, img_canny, 50, 200, 5);
    imshow("canny",img_canny);
	waitKey(0); 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    findContours(img_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros( img_grey.size(), CV_8UC3 );

    int idx = -1;
    int max = 0;
    int limit = 30;
    vector<Point> approx;
    //vector<Point> square;
    Scalar color;
    for( int i = 0; i< contours.size(); i++ )
    {	
    	//get largest contour
    	if (contours[i].size() > max){
    		max = contours[i].size();
    		idx = i;
    	}

    	if(contours[i].size() > limit){
    		drawContours( drawing, contours, i, Scalar(255,0,0), 1);	
    	}

    	//
    	//imshow("contour",drawing);
		//waitKey(0); 
    } 
    imshow("contour",drawing); 
	waitKey(0); 
    drawContours( drawing, contours, idx, Scalar(0,0,255), 2 );
    imshow("contour",drawing);
	waitKey(0); 


}