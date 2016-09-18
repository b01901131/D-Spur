#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <sstream>

using namespace cv;
using namespace std;


#define FEAT_ORB (0)
#define FEAT_SURF (1)
#define SAVE_IMG (1)

enum COLOR { RED, GREEN, PURPLE, BLUE};

bool detect_key(Mat, Mat*);
void detect_puzzle(Mat,int);
void test_orb(Mat);
void match_orb(Mat database, Mat img);
bool draw_color_contour(Mat*, Mat*, COLOR);
double getOrientation(vector<Point>&, Mat&);

Mat img, img_hsv, img_grey, img_canny;
double alpha = 3.5; 
int beta = -650;
int main( int argc, char** argv ){
    
    VideoCapture cap(1); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while(1){
        Mat frame;
        bool success = cap.read(frame);
        if(!success){
            return 1;
        }

        imshow("Frame", frame);
        
        // Mat object = imread("../database/brj.jpg",CV_LOAD_IMAGE_COLOR);
        
        // Mat mask;
        // bool ret = detect_key(frame, &mask);

        // draw_color_contour(&frame, &mask, COLOR(RED));
        // Mat test;
        // frame.copyTo(test,mask);

        // imshow("video", mask);

        // waitKey();
        // test_orb(mask);
        // match_orb(object,test);
        // detect_puzzle(frame, 3);
        // imshow("video", frame);
        // resize( frame, frame, cv::Size(frame.cols / 2, frame.rows / 2) );
        // detect_key(frame);

        #if SAVE_IMG
            int key = cvWaitKey(50);
            // if (key == 97 && ret){
                // cout<< "FUCK!!" <<endl;
                // imwrite("../database/key/i.jpg", mask);
                // return 0;
            // } // ESC
        #endif
            


    }
    


}

bool detect_key(Mat img, Mat* mask){
    int SENSITIVITY1 = 10;
    int SENSITIVITY2 = 10;
    int VALUE = 150;
    // erode(img, img, getStructuringElement(MORPH_RECT,Size(3,3)));
    // dilate(img, img, getStructuringElement(MORPH_RECT,Size(3,3)));
    
    img.convertTo(img, -1, 1.0, -20);
    

    cvtColor(img, img_hsv, COLOR_BGR2HSV);
    // Mat mask;
    Mat mask_red, mask_red1, mask_red2;
    Mat mask_green, mask_purple, mask_blue;


    // Reg and Orange and Yellow detection
    SENSITIVITY1 = 40;
    SENSITIVITY2 = 10;
    inRange(img_hsv, Scalar(0, 30, 150), Scalar(0+SENSITIVITY1,255,255), mask_red1);
    inRange(img_hsv, Scalar(180-SENSITIVITY2, 30, 150), Scalar(180,255,255), mask_red2);
    mask_red = mask_red1 | mask_red2;

    // Green Detection
    inRange(img_hsv, Scalar(40, 50, 150), Scalar(76,255,255), mask_green); //80~157.5
    // imshow("Mask", mask_green);
    // waitKey();

    // Purple 
    SENSITIVITY1 = 20;
    inRange(img_hsv, Scalar(150-SENSITIVITY1, 50, 150), Scalar(150+SENSITIVITY1,255,255), mask_purple); // H:300d S:67~100% V:100%

    // Blue detection
    inRange(img_hsv, Scalar(86, 50, 150), Scalar(132,255,255), mask_blue); // 172.5~262.5

    
    // imshow("Mask1", mask1);
    // waitKey();
    // imshow("Mask2", mask2);
    // waitKey();
    *mask = mask_red1 | mask_red2 | mask_green | mask_purple | mask_blue;

    // imshow("Mask", mask);
    // // waitKey();
    int MORPH_SIZE = 7;
    // int MORPH_SIZE2 = 5;
    erode(*mask, *mask, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    dilate(*mask, *mask, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));

    imshow("morph",*mask);
    // waitKey(0); 


    bool ret = draw_color_contour(&img, mask, COLOR(RED));
    // draw_color_contour(&img, &mask_green, COLOR(GREEN));
    // draw_color_contour(&img, &mask_purple, COLOR(PURPLE));
    // draw_color_contour(&img, &mask_blue, COLOR(BLUE));
    imshow("color", img);

    return ret;
}

void test_orb(Mat img){

    // cvtColor(img, img, cv::COLOR_RGB2GRAY);
    vector<KeyPoint> keypoints, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    ORB orb;

    orb.detect(img, keypoints);
    orb.compute(img, keypoints, descriptors_object);

    drawKeypoints(img, keypoints, img, Scalar(0,0,255), 2);

    imshow("keypoint", img);
    // waitKey();

}

void match_orb(Mat database, Mat img){

    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    Mat homography;


    #if FEAT_ORB
    ORB orb;

    orb.detect(database, keypoints_object);
    orb.compute(database, keypoints_object, descriptors_object);

    orb.detect(img, keypoints_scene);
    orb.compute(img, keypoints_scene, descriptors_scene);

    #elif FEAT_SURF
    int minHessian = 400;
    SurfFeatureDetector surf( minHessian );
    SurfDescriptorExtractor extractor;

    surf.detect( database, keypoints_object );
    extractor.compute( database, keypoints_object, descriptors_object );

    
    surf.detect( img, keypoints_scene );
    extractor.compute( img, keypoints_scene, descriptors_scene );
    #endif


    Mat draw1 = database.clone();
    Mat draw2 = img.clone();
    drawKeypoints(database, keypoints_object, draw1, Scalar(0,0,255), 2);
    drawKeypoints(img, keypoints_scene, draw2, Scalar(0,0,255), 2);
    imshow("draw1", draw1);
    imshow("draw2", draw2);

    // waitKey();
    // return;

    FlannBasedMatcher matcher;
    vector< DMatch > matches;
    Mat img_matches;

    if(descriptors_object.type()!=CV_32F) {
        descriptors_object.convertTo(descriptors_object, CV_32F);
    }
    if(descriptors_scene.type()!=CV_32F) {
        descriptors_scene.convertTo(descriptors_scene, CV_32F);
    }
    if(!descriptors_object.empty() && !descriptors_scene.empty()) {

        // cout << "Start matching!!" <<endl;
        matcher.match(descriptors_object, descriptors_scene, matches);
        // cout << "Start matching!!" <<endl;
            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min idstance between keypoints
            for( int i = 0; i < descriptors_object.rows; i++)
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            //printf("-- Max dist : %f \n", max_dist );
            //printf("-- Min dist : %f \n", min_dist );
            //-- Draw only good matches (i.e. whose distance is less than 3*min_dist)
            std::vector< cv::DMatch >good_matches;

            for( int i = 0; i < descriptors_object.rows; i++ )
            { if( matches[i].distance < 3*min_dist )
                { good_matches.push_back( matches[i]); }
            }

            drawMatches(database, keypoints_object, img, keypoints_scene, \
                    good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cout << "Feature match size:" << good_matches.size() << endl;
            imshow("match result", img_matches );
            // waitKey();

    //         //-- localize the object
    //         // std::vector<cv::Point2f> obj;
    //         // std::vector<cv::Point2f> scene;

    //         // for( size_t i = 0; i < good_matches.size(); i++) {
    //         //     //-- get the keypoints from the good matches
    //         //     obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    //         //     scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    //         // }
    //         // if( !obj.empty() && !scene.empty() && good_matches.size() >= 4) {
    //         //     cv::Mat H = cv::findHomography( obj, scene, cv::RANSAC );

    //         //     //-- get the corners from the object to be detected
    //         //     std::vector<cv::Point2f> obj_corners(4);
    //         //     obj_corners[0] = cv::Point(0,0);
    //         //     obj_corners[1] = cv::Point(database.cols,0);
    //         //     obj_corners[2] = cv::Point(database.cols,database.rows);
    //         //     obj_corners[3] = cv::Point(0,database.rows);

    //         //     std::vector<cv::Point2f> scene_corners(4);

    //         //     cv::perspectiveTransform( obj_corners, scene_corners, H);

    //         //     //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    //         //     line( img_matches, \
    //         //             scene_corners[0] + Point2f(database.cols, 0), \
    //         //             scene_corners[1] + Point2f(database.cols, 0), \
    //         //             Scalar(0,255,0), 4 );
    //         //     line( img_matches, \
    //         //             scene_corners[1] + Point2f(database.cols, 0), \
    //         //             scene_corners[2] + Point2f(database.cols, 0), \
    //         //             Scalar(0,255,0), 4 );
    //         //     line( img_matches, \
    //         //             scene_corners[2] + Point2f(database.cols, 0), \
    //         //             scene_corners[3] + Point2f(database.cols, 0), \
    //         //             Scalar(0,255,0), 4 );
    //         //     line( img_matches, \
    //         //             scene_corners[3] + Point2f(database.cols, 0), \
    //         //             scene_corners[0] + Point2f(database.cols, 0), \
    //         //             Scalar(0,255,0), 4 );
    //         // }
    }

    // imshow("match result", img_matches );
    // if(cv::waitKey(30) >= 0) break;


    // drawKeypoints(img, keypoints, img, Scalar(0,0,255), 2);

    // imshow("keypoint", img);
}

bool draw_color_contour(Mat* img, Mat* mask, COLOR c){

    Mat canny;
    Scalar color;
    switch(c){
        case RED: color = Scalar(0,0,255); break;
        case GREEN: color = Scalar(0,150,0); break;
        case PURPLE: color = Scalar(150,0,150); break;
        case BLUE: color = Scalar(150,0,0); break;    
    }
    

    
    // int MORPH_SIZE2 = 5;
    // int MORPH_SIZE = 3;
    // erode(*mask, *mask, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    // dilate(*mask, *mask, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    // imshow("morph", *mask);
    // waitKey();

    Canny(*mask, canny, 50, 200, 3);
    // imshow("canny",canny);
    // waitKey(0); 


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    vector<Moments> mu;
    vector<Point2f> mc;
    bool valid = false;
    for( int i = 0; i< contours.size(); i++ )
    {   
        //get largest contour
        // if (contours[i].size() > max){
        //     max = contours[i].size();
        //     idx = i;
        // }
        // cout << contours[i].size() << endl;
        int limit = 20;
        if(contourArea(contours[i]) >= 4000){ //(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;
            valid = true;
            drawContours( *img, contours, i, color, 2);//CV_FILLED); 
            double angle = getOrientation(contours[i], *img);


            mu.push_back(moments(contours[i], false));
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));
            // cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
            cout<< "Mass area:" << contourArea(contours[i]) << endl;
            cout<< "Mass peripheral:" << contours[i].size() << endl;


            imshow("contour",*img);
            // waitKey();
        }
    } 
    return valid;
}

double getOrientation(vector<Point>& pts, Mat& img){
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    // drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    line( img, cntr, p1, Scalar(0,255,0), 1);
    // drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
    line( img, cntr, p2, Scalar(255,255,0), 1);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
}

void detect_puzzle(Mat img, int morph){

    // img = imread("../img/key3.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
    // cout << img.cols/2 <<" , "<<img.rows/2<<endl;
    // cv::resize( img, img, cv::Size(img.cols / 2, img.rows / 2) );
    
    // if(! img.data ){
        // cout <<  "Could not open or find the image" << std::endl ;
        // return -1;
    // }

    // imshow("img",img);
    // waitKey(0);


    img.convertTo(img, -1, alpha, beta);
    // imshow("img",img);
    // waitKey(0); 


    erode(img, img, getStructuringElement(MORPH_RECT,Size(morph,morph)));
    dilate(img, img, getStructuringElement(MORPH_RECT,Size(morph,morph)));

    // imshow("img",img);
    // waitKey(0);  


    // cvtColor(img, img_hsv, COLOR_BGR2HSV);
    // Mat mask, mask1, mask2;
     

    cvtColor( img, img_grey, CV_BGR2GRAY);
    // imshow("grey",img_grey);
    // waitKey(0);  

    /* Hist Equalizer
    vector<Mat> channels;
    Mat img_hist;

    cvtColor( img, img_grey, CV_BGR2YCrCb);//CV_BGR2GRAY
    imshow("grey",img_grey);
    waitKey(0);  

    split(img_grey,channels);
    equalizeHist(channels[0], channels[0]);
    merge(channels,img_grey); 
    cvtColor(img_grey, img_grey, CV_YCrCb2BGR);
    imshow("grey",img_grey);
    waitKey(0);  
    */


    // equalizeHist( img_grey, img_grey );
    // imshow("grey",img_grey);
    // waitKey(0); 
    

    //threshold(img_grey, img_grey, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    //imshow("grey",img_grey);
    //waitKey(0);  

    // erode(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(9,9)));
    // dilate(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(9,9)));
    // imshow("grey",img_grey);
    // waitKey(0);  

    Canny(img_grey, img_canny, 50, 200, 3);
    imshow("canny",img_canny);
    // waitKey(0); 


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    findContours(img_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    Mat drawing = Mat::zeros( img_canny.size(), CV_8UC3 );
    
    int idx = -1;
    int max = 0;
    int limit = 80;
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

        if(contourArea(contours[i]) >= 500 && contourArea(contours[i]) < 10000){//(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;
            // cout << contours[i].size() << endl;
            drawContours( img, contours, i, Scalar(0,0,255), CV_FILLED);    
            ostringstream convert, convert2;
            convert << contourArea(contours[i]); 
            convert2 << contours[i].size();
            String toput = convert.str();
            String toput2 = convert2.str();
            Moments mu = moments(contours[i], false);
            Point2f cntr = Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
            Point2f cntr2 = Point2f(mu.m10/mu.m00 , mu.m01/mu.m00+20);

            
            putText(img, toput, cntr, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, Scalar(255,0,0), 1, 8);
            putText(img, toput2, cntr2, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, Scalar(255,0,0), 1, 8);
            imshow("contour",img);
            // waitKey();
        }
        
        //
        
    } 
    imshow("img2",img);
    // waitKey(0); 
}
