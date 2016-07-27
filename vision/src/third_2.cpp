#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

int lo_diff = 1;
int up_diff = 5;
Mat img, img_canny,img_3,img_4, img_hsv, img_thresh;
Rect ccomp;


int main( int argc, char** argv )
{
    // if( argc != 2)
    // {
    //  cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    //  return -1;
    // }

    
    img = imread("../img/cifar1.png", 0);   // Read the file
    cout << img.cols <<" , "<<img.rows<<endl;
    //cv::resize( img, img, cv::Size(img.cols / 2, img.rows / 2) );
    if(! img.data ){
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    
    

    namedWindow( "image", 0 );
    imshow( "image", img );


    Canny( img, img_canny, 50, 200, 3 );
    cvtColor( img_canny, img_3, CV_GRAY2BGR );
    
    vector<Vec4i> lines;
    HoughLinesP( img_canny, lines, 1, CV_PI/180, 80, 30, 10 );
    
    /*for( size_t i = 0; i < lines.size(); i++ ){
        line( img_3, Point(lines[i][0], lines[i][1]),
        Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }*/

    namedWindow( "canny", 0 );
    imshow( "canny", img_canny );

    namedWindow( "bgr", 0 );
    imshow( "bgr", img_3 );



    // while(1){
    // 	//on_trackbar();	
    // 	img_2 = img.clone();
    // 	img_3 = img.clone();
    // 	img_4 = img.clone();
    // 	floodFill(img_2, Point(1,1), Scalar(155, 255,55), &ccomp, Scalar(lo_diff, lo_diff, lo_diff),Scalar(up_diff, up_diff, up_diff)); 
    // 	floodFill(img_3, Point(359,479), Scalar(155, 255,55), &ccomp, Scalar(lo_diff, lo_diff, lo_diff),Scalar(up_diff, up_diff, up_diff)); 
    // 	floodFill(img_4, Point(0,240), Scalar(155, 255,55), &ccomp, Scalar(lo_diff, lo_diff, lo_diff),Scalar(up_diff, up_diff, up_diff)); 
    // 	imshow( "filled1", img_2 );
    // 	imshow( "filled2", img_3 );
    // 	imshow( "filled3", img_4 );
    // 	int iKey = waitKey(1000);
    // }
    
    //cvtColor(img, img_hsv, COLOR_BGR2HSV);
	//cvtColor(img, img_grey, COLOR_BGR2GRAY);   

	
	//cv::threshold(img_grey, img_thresh, 20, 255, THRESH_BINARY);
   	//inRange(img_hsv, Scalar(0,0,100,0), Scalar(180,255,255,0), threshold); 
    

    //vector<Vec4i> lines;  
 	//HoughLinesP(img, lines, 1, CV_PI/180, 80, 100, 10);  



    //namedWindow( "Img", WINDOW_AUTOSIZE );
    //namedWindow( "Img_hsv", WINDOW_AUTOSIZE );// Create a window for display.
    //namedWindow( "threshold", WINDOW_AUTOSIZE );
    //for(;;){
	    
	    //imshow( "threshold", img_thresh );
	    //imshow( "Img_hsv", img_hsv );
		
	    waitKey(0);                                          // Wait for a keystroke in the window	
    //}
    
    return 0;
}