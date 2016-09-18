#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <sstream>
#include <math.h>

using namespace cv;
using namespace std;


#define SAVE_IMG 0

Mat calc_profile_n_crop(Mat* img, String, Point2f*, double*);
bool calc_profile(Mat* img, String, Point2f*, double*, bool);
// Mat calc_profile2(Mat* img, String, double);
Mat warp_transform(Mat* img, Point2f center, double angle, double scale);
double compare_contour(Mat* answer, Mat* object, double);

int main( int argc, char** argv ){
    
    // namedWindow("bitch", WINDOW_NORMAL);
    while(1){
        Mat answer, object, warp_object;
        bool VERBOSE = true;
        
        answer = imread("../database/brj_grey.jpg",CV_LOAD_IMAGE_COLOR);
        object = imread("../database/test_j_grey.jpg",CV_LOAD_IMAGE_COLOR);
        
        double min = 1.0;
        double min_deg = -1.0;

        Point2f ans_center, obj_center;
        double ans_area, obj_area;


        // Process Answer
        Mat cropped_ans = calc_profile_n_crop(&answer, "answer", &ans_center, &ans_area);
        calc_profile(&cropped_ans, "answer", &ans_center, &ans_area, VERBOSE);

        double tot_compare = 0.0;
        int total_test = 0;
        for (int i=0; i<36; i++){
            // Process Object
            bool ret = calc_profile(&object, "object", &obj_center, &obj_area, true);
            if (ret == false)
                continue;
            
            warp_object = warp_transform(&object, obj_center, 0.0+i*10 , sqrt(ans_area/obj_area));
            Mat cropped_obj = calc_profile_n_crop(&warp_object, "object", &obj_center, &obj_area);
            cout<<"HI!" <<endl;
            calc_profile(&cropped_obj, "object", &obj_center, &obj_area, VERBOSE);



            imshow("cropped_obj", cropped_obj);
            imshow("cropped_answer", cropped_ans);
            // waitKey();

            // Compare Object
            double compare = compare_contour(&cropped_ans, &cropped_obj, i);
            if (compare != -1.0){
                tot_compare += compare;
                total_test += 1;
            }
        }

        cout << "Average compare : " << tot_compare/total_test << endl;
          
        /*
        for(int i=0; i<360; i++){
            
            
            double match_score = compare_contour(&answer, &scaled, 0.0+i);    
            waitKey();
            if(match_score < min){
                min = match_score;
                min_deg = 0.0+i;
            }
        }

        cout << min_deg << ", " << min << endl;
        

        calc_profile(&answer, "answer");
        Mat scaled = calc_profile2(&object, "object", min_deg);
        double match_score = compare_contour(&answer, &scaled, min_deg);

        */

        
        
        // waitKey();
        return 0;
        #if SAVE_IMG
            int key = cvWaitKey(50);
            if (key == 97){
                cout<< "FUCK!!" <<endl;
                // imwrite("../database/test_j_grey.jpg", mask);
                return 0;
            } // ESC
        #endif
            


    }
}

Mat calc_profile_n_crop(Mat* img, String name, Point2f* center, double* area){

    Mat ret;
    Mat canny = Mat();
    Scalar color = Scalar(0,0,255);
    Canny(*img, canny, 50, 200, 3);
    // imshow("img",*img);
    // waitKey(0); 
    // imshow("canny",canny);
    // waitKey(0); 

    bool valid = false;
    vector<vector<Point> > contours;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    vector<Moments> mu;
    vector<Point2f> mc;
    vector<Rect> boundRect;


    for( int i = 0; i< contours.size(); i++ ){   
        
        // cout << contours[i].size() << endl;
        // cout << contourArea(contours[i]) << endl;
        int limit = 20;
        if(contourArea(contours[i]) >= 1500){ //(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;
            // drawContours( *img, contours, i, color, 2);//CV_FILLED); 
            //double angle = getOrientation(contours[i], *img);


            mu.push_back(moments(contours[i], false));
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));
            *center = mc.back();
            *area = contourArea(contours[i]);
            // cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
            // cout<< "Mass area:" << contourArea(contours[i]) << endl;
            // cout<< "Mass peripheral:" << contours[i].size() << endl;

            // approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            // boundRect.push_back(boundingRect( Mat(contours_poly[i]) ));
            
            boundRect.push_back(boundingRect(contours[i]));
            Point new_tl = Point((boundRect.back().tl().x - 5), (boundRect.back().tl().y - 5));
            Point new_br = Point((boundRect.back().br().x + 5), (boundRect.back().br().y + 5));
            boundRect[i] = Rect(new_tl, new_br);
            // rectangle( *img, new_tl, new_br, Scalar(0,0,255), 2, 8, 0 );                
        


            // imshow(name,*img);
            // waitKey();
        }
    } 

    // if(crop)
    ret = (*img)(boundRect.back());

    return ret;

    // warp_transform(img, mc.back(), 0.0, 0.8);
}

bool calc_profile(Mat* img, String name, Point2f* center, double* area, bool verbose){

    // Mat ret;
    Mat canny = Mat();
    Scalar color = Scalar(0,0,255);
    Canny(*img, canny, 50, 200, 3);
    bool valid = false;

    vector<vector<Point> > contours;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    vector<Moments> mu;
    vector<Point2f> mc;


    for( int i = 0; i< contours.size(); i++ ){   
        
        // cout << contours[i].size() << endl;
        // cout << contourArea(contours[i]) << endl;
        int limit = 20;
        if(contourArea(contours[i]) >= 5000){ //(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;
            // drawContours( *img, contours, i, color, 2);//CV_FILLED); 
            //double angle = getOrientation(contours[i], *img);

            valid = true;
            mu.push_back(moments(contours[i], false));
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));
            *center = mc.back();
            *area = contourArea(contours[i]);
            if (verbose){
                cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
                cout<< "Mass area:" << contourArea(contours[i]) << endl;
                cout<< "Mass peripheral:" << contours[i].size() << endl;    
            }
            

            // approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            // boundRect.push_back(boundingRect( Mat(contours_poly[i]) ));


            // imshow(name,*img);
            // waitKey();
        }
    }

    if (contours.size() == 0)
        return false;
    else 
        return true; 

}



double compare_contour(Mat* answer, Mat* object, double deg){

    // waitKey();

    // cout<< "answer size: " << answer->rows << "," << answer->cols << endl;
    // cout<< "object size: " << object->rows << "," << object->cols << endl;
    cout << "Test:" << deg<<endl;
    Mat canny_answer, canny_object;
    Scalar color = Scalar(0,0,255);
    Canny(*answer, canny_answer, 50, 200, 3);
    Canny(*object, canny_object, 50, 200, 3);

    vector<vector<Point> > contours_answer, contours_object;
    findContours(canny_answer, contours_answer, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    findContours(canny_object, contours_object, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    // Mat R = Mat::zeros( canny_answer.size(), CV_32F );
    // Mat T = Mat::zeros( canny_object.size(), CV_32F );
    // drawContours(R, contours_answer, 0, Scalar(255,255,255), 1);
    // drawContours(T, contours_object, 0, Scalar(255,255,255), 1);

    // if(R.type()!=CV_32F) {
    //     R.convertTo(R, CV_32F);
    //     T.convertTo(T, CV_32F);
    // }
    // imshow("Contour_ans", R);
    // imshow("Contour_obj", T);
    // waitKey();
    double compare = -1.0;
    cout << contours_object.size() << endl;
    if(contours_object.size() != 0){
        compare = matchShapes(contours_answer[0], contours_object[0], CV_CONTOURS_MATCH_I1, 0);
        cout << "compare:" << deg <<","<< compare << endl;

    }
    else{
        cout << "Cannot find contour...." <<endl;
    } 
        
    // double compare = matchShapes(R, T, CV_CONTOURS_MATCH_I1, 1);

    
    // waitKey();
    return compare;
    
}

Mat warp_transform(Mat* img, Point2f center, double angle, double scale){
    
    Mat warp = Mat::zeros(img->rows, img->cols, img->type());
    // double angle = 0.0;
    // double scale = 0.8;

    Mat rot_mat = getRotationMatrix2D(center, angle, scale);
    warpAffine(*img, warp, rot_mat, warp.size());

    // imshow("warp", warp);
    // calc_profile(&warp, "Scaled");

    return warp;
}