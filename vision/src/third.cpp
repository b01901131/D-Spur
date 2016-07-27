#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int lo_diff = 1;
int up_diff = 5;
int th =170; 
int mode = 2;
Mat img, img_2,img_3,img_4, img_hsv,img_grey,img_grey2, img_thresh,img_canny;
Rect ccomp;
void on_trackbar(){
	
	floodFill(img, Point(1,1), Scalar(155, 255,55), &ccomp, Scalar(lo_diff, lo_diff, lo_diff),Scalar(up_diff, up_diff, up_diff)); 
	imshow("fill", img);
}

int main( int argc, char** argv )
{
    // if( argc != 2)
    // {
    //  cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    //  return -1;
    // }

    
    img = imread("../img/cifar2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    cout << img.cols/4 <<" , "<<img.rows/4<<endl;
    cv::resize( img, img, cv::Size(img.cols / 4, img.rows / 4) );
    if(! img.data ){
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    
    
    //imshow("morph",img);
    cvtColor( img, img_grey, CV_BGR2GRAY );
    threshold(img_grey, img_grey, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    //imshow("grey",img_grey);

    
    dilate(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(15,15)));
    erode(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(15,15)));
    //imshow("morph grey1",img_grey);    

    //dilate(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(15,15)));
    //erode(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(15,15)));
    //dilate(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(3,3)) );
    //erode(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(3,3)));
    
	//imshow("morph grey2",img_grey);    
	//waitKey(0);

	//threshold(img_grey, img_grey2, 1, 255, CV_THRESH_BINARY);
    Canny(img_grey, img_grey2, 100, 200, 3);
    //imshow("grey2",img_grey2);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    findContours(img_grey2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros( img_grey.size(), CV_8UC3 );

    int idx = -1;
    int max = 0;
    vector<Point> approx;
    vector<Point> square;
    Scalar color;
    for( int i = 0; i< contours.size(); i++ )
    {	
    	//get largest contour
    	if (contours[i].size() > max){
    		max = contours[i].size();
    		idx = i;
    	}
    } 
    drawContours( drawing, contours, idx, Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)));
    approxPolyDP(Mat(contours[idx]), approx, arcLength(Mat(contours[idx]), true)*0.02, true);
    cout<<"Approx size:"<< approx.size()<<endl;
    for (int n=0; n<approx.size(); n++){
    	bool cont = false;
    	for(int m=0; m<n; m++){
    		double dist = cv::norm(approx[n]-approx[m]);
    		if (dist < 10){ //ignore points that are close with eachother
    			cont = true;
    			break;
    		}
    	}
    	if(cont)
    		continue;
    	else{
    		circle(drawing, approx[n], 2, Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)), 2 );
    		square.push_back(approx[n]);		
    		//imshow("draw",drawing);
    		//waitKey(0);
    	}
    		
    }
    
   

   	/*Draw boundary*/
    // int n = (int)square.size();
    // const Point* p = &square[0];
    // polylines(img, &p, &n, 1, true, Scalar(0, 0, 255), 3, 0);
    // imshow("result",img);
    // waitKey(0);


    vector<Point2f> square2f;
    for(int n=0; n<square.size(); n++){
    	float x = (float)square[n].x;
    	float y = (float)square[n].y;
    	cout << x <<" , " <<y<<endl;
    	square2f.push_back(Point2f(x,y));
    }

    Mat tranformed(Size(512,512),CV_8UC3);

    vector<Point2f> dst;
    dst.push_back(Point2f(0, 0));
    dst.push_back(Point2f(511, 0));
    dst.push_back(Point2f(511, 511));
    dst.push_back(Point2f(0, 511));
    

    Mat h = findHomography(square2f, dst, 0);
    cout <<"H"<<h<<endl;
    Mat im_out;
    // Warp source image to destination based on homography
    warpPerspective(img, im_out, h, tranformed.size());
 	
    // Display images
    imshow("Source Image", img);
    //waitKey(0);
    //imshow("Destination Image", tranformed);
    imshow("Warped Source Image", im_out);
    waitKey(0);
    Mat im_out_2 = im_out.clone();
    vector<Rect> crops;
    for(int i=0; i<16; i++){
    	for(int j=0; j<16; j++){
    		rectangle(im_out_2, Point(32*i,32*j), Point(32*(i+1),32*(j+1)), Scalar(0,0,255), 2);
    		crops.push_back(Rect(32*i,32*j,32,32));			
    	}
    }
    imshow("Warped Source Image", im_out_2);
    waitKey(0);    		

    for(int n=0; n<crops.size(); n++){
    	//Mat src(img);
    	Mat cropped(Size(32,32),CV_8UC3);// = image(crops[n]);
    	cropped = im_out(crops[n]).clone();//.copyTo(cropped);

    	
    	imshow("crop", cropped);
    	waitKey(0);    		
    }






    
    return 0;
}

