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
using namespace chrono;


#define SAVE_IMG 0

enum COLOR { RED, GREEN, PURPLE, BLUE};

struct match_result{
    int pivot;
    int compare;
};

match_result rough_search(Mat*, Mat*, double*, int);
void complete_search(Mat*, Mat*, double*, int, int);
Mat calc_profile_n_crop(Mat* img, String, Point2f*, double*);
void calc_profile(Mat* img, String, Point2f*, double*, bool);
bool detect_key(Mat, Mat*);
bool draw_color_contour(Mat*, Mat*, COLOR);
Mat warp_transform(Mat* img, Point2f center, double angle, double scale);
double compare_contour(Mat* answer, Mat* object, double);

int main( int argc, char** argv ){
    Mat frame;

    // Setup Camera
    VideoCapture cap(1);
    if (!cap.isOpened()){
        cout << "Cannot open the video cam" << endl;
        return -1;
    }
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    cout << "Frame size : " << dWidth << " x " << dHeight << endl;


    //Process Database
    vector<Mat> ans_data;
    vector<Point2f> ans_centers(10);
    vector<double> ans_areas(10);
    String fn[] = {"a.jpg","b.jpg","c.jpg","d.jpg","e.jpg","f.jpg","g.jpg","h.jpg","w.jpg","m.jpg"};

    for(int i=0; i<sizeof(fn)/sizeof(String); i++){
        ans_data.push_back(imread(("../database/key/"+fn[i]), CV_LOAD_IMAGE_GRAYSCALE));

        calc_profile(&(ans_data[i]), "answer", &(ans_centers[i]), &(ans_areas[i]), false);
        Mat w = warp_transform(&(ans_data[i]), (ans_centers[i]), 0.0, 1.0);
        Mat cropped_ans = calc_profile_n_crop(&w, "answer", &(ans_centers[i]), &(ans_areas[i]));
        calc_profile(&cropped_ans, "answer", &(ans_centers[i]), &(ans_areas[i]), true);

        imshow(fn[i], cropped_ans);
        ans_data[i] = cropped_ans;
        // waitKey();  
    }


    char cmd; // = waitKey();


    //Match with Database
    while(1){

        
        bool success = cap.read(frame);
        if(!success) return 1;
        // imshow("Video", frame);
        cmd = waitKey(50);
        
        Mat mask;
        bool ret = detect_key(frame, &mask);
        imshow("mask", mask);

        if(cmd == 'm' && ret){

            cout << "Start matching !..." <<endl;

            for(int i=0; i<sizeof(fn)/sizeof(String); i++){
                // imshow("ans", (ans_data[i]));
                // imshow("key", frame);
                // waitKey();

                match_result res = rough_search(&(ans_data[i]), &mask, &(ans_areas[i]), 5);


                cout << fn[i][0] << ": " << res.pivot << "," << res.compare << endl;
            }
            waitKey();
            return 0;
        }

        else{
            continue;
        }
        
    }

    /*while(1){
        Mat answer, object;
        
        
        answer = imread("../database/brj_grey.jpg",CV_LOAD_IMAGE_GRAYSCALE);
        object = imread("../database/test_R_grey.jpg",CV_LOAD_IMAGE_GRAYSCALE);
        

        // Process Answer
        Point2f ans_center;
        double ans_area;

        calc_profile(&answer, "answer", &ans_center, &ans_area, false);
        Mat w = warp_transform(&answer, ans_center, 4  , 1.0);
        Mat cropped_ans = calc_profile_n_crop(&w, "answer", &ans_center, &ans_area);
        calc_profile(&cropped_ans, "answer", &ans_center, &ans_area, true);

        time_t start, end;
        time(&start);

        
        int pivot = rough_search(&cropped_ans, &object, &ans_area, 5);
        complete_search(&cropped_ans, &object, &ans_area, pivot, 30);

        time(&end);
        cout << "Time: " << (double)difftime(end,start) << "sec." <<endl;
        waitKey();
    }*/
}

void complete_search(Mat* cropped_ans, Mat* object, double* ans_area, int pivot, int range){

    Mat warp_object, cropped_obj;
    double obj_area;
    Point2f obj_center;

    double min = 9999999;
    double min_deg = -1.0;
    double total = 0.0;
    int total_index = 0;

    // time_t start, time_2,end;
    // time(&start);
    
    int SCALE = 1;
    // for (int deg=(pivot-range); deg<(pivot+range); deg++){
    for (int deg=(30); deg<(50); deg++){

        
        // struct timeval start;
        // gettimeofday(&start, NULL);
        // long int start_ms = start.tv_sec * 1000 + start.tv_usec / 1000;
        // milliseconds start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());

        // Process Object
        calc_profile(object, "object", &obj_center, &obj_area, false);
        warp_object = warp_transform(object, obj_center, deg*SCALE , sqrt(*ans_area/obj_area));
        cropped_obj = calc_profile_n_crop(&warp_object, "object", &obj_center, &obj_area);
        if(cropped_obj.rows == 1)
            continue;
        calc_profile(&cropped_obj, "object", &obj_center, &obj_area, false);

        // struct timeval time_2;
        // gettimeofday(&time_2, NULL);
        // long int time_2_ms = time_2.tv_sec * 1000 + time_2.tv_usec / 1000;
        // cout << time_2_ms - start_ms << "ms" << endl;
        // milliseconds time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        // cout << (time_2 - start).count() << "ms" << endl;

        imshow("cropped_obj", cropped_obj);
        imshow("cropped_answer", *cropped_ans);
        // waitKey();

        // Compare Object
        double val = compare_contour(cropped_ans, &cropped_obj, deg*SCALE);   
        if(val < min){
            min = val;
            min_deg = deg*SCALE;
        }
        total += val;
        total_index += 1;


        // struct timeval end;
        // gettimeofday(&end, NULL);
        // long int end_ms = end.tv_sec * 1000 + end.tv_usec / 1000;
        // cout << end_ms - time_2_ms << "ms" << endl;
    }

    cout << min_deg <<","<< min << endl;
    cout << total/total_index << endl;
    


    double to_turn = min_deg*SCALE;
    to_turn = tan((90-to_turn)*M_PI/180);
    cout << "To turn :" << to_turn << endl;
    calc_profile(object, "object", &obj_center, &obj_area, false);
    warp_object = warp_transform(object, obj_center, to_turn , 1.0);
    cropped_obj = calc_profile_n_crop(object, "object", &obj_center, &obj_area);
    calc_profile(&cropped_obj, "object", &obj_center, &obj_area, false);
    Mat draw;
    cvtColor(cropped_obj, draw, CV_GRAY2RGB);
    cout << obj_center << endl;
    cout <<Point2f((obj_center.x+50),(obj_center.y-50*(to_turn))) << endl;
    cv::line(draw, obj_center, Point2f((obj_center.x+50),(obj_center.y-50*(to_turn))), Scalar(0,0,255), 1, 8, 0);
    // line(draw, Point(150,150), Point(150, 300), Scalar(0,0,255), 1, 8, 0);
    imshow("result", draw);
}

match_result rough_search(Mat* cropped_ans, Mat* object, double* ans_area, int SCALE){
    Mat warp_object, cropped_obj;
    double obj_area;
    Point2f obj_center;

    double min = 9999999;
    double min_deg = -1.0;
    double total = 0.0;
    int total_index = 0;

    for (int deg=0; deg<(360/SCALE); deg++){

        // Process Object
        calc_profile(object, "object", &obj_center, &obj_area, false);
        warp_object = warp_transform(object, obj_center, deg*SCALE , sqrt(*ans_area/obj_area));
        cropped_obj = calc_profile_n_crop(&warp_object, "object", &obj_center, &obj_area);
        if(cropped_obj.rows == 1)
            continue;
        calc_profile(&cropped_obj, "object", &obj_center, &obj_area, false);

        imshow("cropped_obj", cropped_obj);
        imshow("cropped_answer", *cropped_ans);
        waitKey();
        // Compare Object
        double val = compare_contour(cropped_ans, &cropped_obj, deg*SCALE);   
        if(val < min){
            min = val;
            min_deg = deg*SCALE;
        }
        total += val;
        total_index += 1;
    }

    // cout << min_deg <<","<< min << endl;
    // cout << total/total_index << endl;

    match_result ret;
    ret.pivot = min_deg;
    ret.compare = min;

    return ret;
}


Mat calc_profile_n_crop(Mat* img, String name, Point2f* center, double* area){

    Mat ret = Mat::zeros( Size(1,1), CV_32F );
    Mat canny = Mat();
    Scalar color = Scalar(0,0,255);
    Canny(*img, canny, 50, 200, 3);
    // imshow("img",*img);
    // waitKey(0); 
    // imshow("canny",canny);
    // waitKey(0); 


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


            mu.push_back(moments(contours[i], false));
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));
            *center = mc.back();
            *area = contourArea(contours[i]);
            // cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
            // cout<< "Mass area:" << contourArea(contours[i]) << endl;
            // cout<< "Mass peripheral:" << contours[i].size() << endl;
            
            boundRect.push_back(boundingRect(contours[i]));
            Point new_tl = Point((boundRect.back().tl().x - 5), (boundRect.back().tl().y - 5));
            Point new_br = Point((boundRect.back().br().x + 5), (boundRect.back().br().y + 5));
            boundRect[i] = Rect(new_tl, new_br);
            // rectangle( *img, new_tl, new_br, Scalar(0,0,255), 2, 8, 0 );                
        


            // imshow(name,*img);
            // waitKey();
        }
    } 

    if(boundRect.size())
        // if(crop)
        ret = (*img)(boundRect.back());

    return ret;

    // warp_transform(img, mc.back(), 0.0, 0.8);
}

void calc_profile(Mat* img, String name, Point2f* center, double* area, bool verbose){

    // Mat ret;
    Mat canny = Mat();
    Scalar color = Scalar(0,0,255);
    Canny(*img, canny, 50, 200, 3);


    vector<vector<Point> > contours;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    vector<Moments> mu;
    vector<Point2f> mc;


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
}

double compare_contour(Mat* answer, Mat* object, double deg){

    
    // imshow("Contour_ans", *answer);
    // imshow("Contour_obj", *object);
    // waitKey();
    double compare = 0.0;
    double compare2 = 0.0;


    int row_m = min(answer->rows,object->rows);
    int col_m = min(answer->cols,object->cols);

    // Mat crop_answer = ;
    // Mat crop_object = ;

    // cout << crop_answer.rows << "x" << crop_answer.cols <<endl;

    // uint16_t* input = (uint16_t*)(answer->data);
    
    for(int i=0; i< row_m; i++){

        // double *M = (answer->ptr<double>(i));
        for(int j=0; j< col_m; j++){
            compare += abs(answer->at<uint16_t>(i,j) - object->at<uint16_t>(i,j));
            // cout << answer->at<uint16_t>(i,j) << "," << M[j] <<endl;
            // waitKey();
        }
    }




    // compare2 = sum(norm((*answer)(Rect(Point(0,0),Point(col_m, row_m))) , (*object)(Rect(Point(0,0),Point(col_m, row_m)))))[0];
    // Mat test;
    // cv::absdiff(*answer, *object, test);
    // cout << test << endl;
    compare /= (col_m * row_m);
    // cout << "compare:" << deg <<","<< compare<< endl;
    // waitKey(10);

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

bool detect_key(Mat img, Mat* mask){
    Mat img_hsv;    
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
        if(contourArea(contours[i]) >= 1500){ //(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;
            valid = true;
            drawContours( *img, contours, i, color, 2);//CV_FILLED); 
            // double angle = getOrientation(contours[i], *img);


            mu.push_back(moments(contours[i], false));
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));
            // cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
            // cout<< "Mass area:" << contourArea(contours[i]) << endl;
            // cout<< "Mass peripheral:" << contours[i].size() << endl;


            // imshow("contour",*img);
            // waitKey();
        }
    } 
    return valid;
}