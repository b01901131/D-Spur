 #include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
// #include "opencv2/opencv.hpp"
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

int show_x, show_y;
int SIZE_LIMIT = 1000;
bool print = false;

enum COLOR { RED, GREEN, PURPLE, BLUE, NONE};

struct match_result{
    int pivot;
    int idx;
    double compare;
    Point2f center;
};

void onMouse(int event,int x,int y,int flag,void* param){
    if(flag==CV_EVENT_FLAG_LBUTTON && event == CV_EVENT_LBUTTONUP){
          show_x = x;
          show_y = y;
          print = true;
    }
}

match_result rough_search(Mat*, Mat*, double*, int);
void complete_search(Mat*, Mat*, double*, int, int);
Mat calc_profile_n_crop(Mat* img, String, Point2f*, double*, int limit = 1000);
void calc_profile(Mat* img, String, Point2f*, double*, bool, int limit = 1000);
void detect_key(Mat, Mat*, COLOR, vector<Mat>*);
int draw_color_contour(Mat*, Mat*, COLOR, vector<Mat>*);
Mat warp_transform(Mat* img, Point2f center, double angle, double scale);
double compare_contour(Mat* answer, Mat* object, double);
Mat tan_triggs_preprocessing(InputArray src, float alpha = 0.1, float tau = 10.0, float gamma = 0.2, int sigma0 = 1, int sigma1 = 2);
Mat norm_0_255(const Mat& src);
void detect_puzzle(Mat img, int morph);
vector<Point> contoursConvexHull( vector<Point>, Mat* img=0);
// void update_detect_list(Point2f, Mat, vector<Mat>*, vector<Point2f>*);
// void show_detect_list(vector<Mat>*);



int main( int argc, char** argv ){
    Mat frame;
    cvNamedWindow("Frame",0);
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
    vector<Point2f> ans_centers(4);
    vector<double> ans_areas(4);
    String fn[] = {"a.jpg","i.jpg","q.jpg","y.jpg"};
    // String fn[] = {"d.jpg", "e.jpg", "m.jpg"};

    for(int i=0; i<sizeof(fn)/sizeof(String); i++){
        ans_data.push_back(imread(("../database/key/"+fn[i]), CV_LOAD_IMAGE_GRAYSCALE));

        calc_profile(&(ans_data[i]), "answer", &(ans_centers[i]), &(ans_areas[i]), false);
        Mat w = warp_transform(&(ans_data[i]), (ans_centers[i]), 0.0, 0.5);
        Mat cropped_ans = calc_profile_n_crop(&w, "answer", &(ans_centers[i]), &(ans_areas[i]));
        calc_profile(&cropped_ans, "answer", &(ans_centers[i]), &(ans_areas[i]), true);

        imshow(fn[i], cropped_ans);
        ans_data[i] = cropped_ans;
        // waitKey();  
    }


    char cmd; // = waitKey();

    cap.read(frame);
    sleep(2);

    Mat overlap = Mat::zeros(Size(640,480), CV_8UC1);
    vector<Mat> detected_obj;
    //Match with Database
    while(1){

    	
        
        bool success = cap.read(frame);
        if(!success) return 1;
        imshow("Video", frame);

        // Mat processed_frame = tan_triggs_preprocessing(frame);
        // processed_frame =  norm_0_255(processed_frame);
        // imshow("Frame", processed_frame);
        Mat mask;
        detect_key(frame, &mask, COLOR(RED), &detected_obj);
        overlap = overlap | mask;
        
        imshow("mask", mask);
        imshow("overlap", overlap);
        
        // waitKey();
        // show_detect_list(&detected_obj);
        
        


        // waitKey();
        cmd = waitKey(10);

        if(cmd == 'm' ){

            cout << "Start matching !..." <<endl;
        	
            erode(overlap, overlap, getStructuringElement(MORPH_RECT,Size(3,3)));
    		dilate(overlap, overlap, getStructuringElement(MORPH_RECT,Size(3,3)));
    		// medianBlur(overlap, overlap, 11);
    		imshow("overlap", overlap);    
    		// waitKey();

            int det_count = draw_color_contour(&frame, &overlap, COLOR(RED), &detected_obj);
        	cout<<"Detected : " << det_count << endl;
        	// waitKey();

        	vector<double> minimum;
        	vector<int> prediction;
        	vector<match_result> pred;
        	for(int n=0; n<sizeof(fn)/sizeof(String); n++){
        		match_result m;
        		m.pivot = -1;
        		m.compare = 1000;
        		m.center = Point2f(0,0);
        		minimum.push_back(1000);
        		prediction.push_back(-1);
        		pred.push_back(m);
        	}

            for(int i=0; i<det_count; i++){
	            for(int j=0; j<sizeof(fn)/sizeof(String); j++){

	                match_result res = rough_search(&(ans_data[j]), &(detected_obj[i]), &(ans_areas[j]), 5);
	                res.idx = i;
	                cout << fn[j][0] << ": " << res.pivot << "," << res.compare << endl;

	                if( res.compare < pred[j].compare ){
	                	// minimum[j] = res.compare;
	                	// prediction[j] = i;
	                	pred[j] = res;
	                }
	            }
            }

            for(int n=0; n<sizeof(fn)/sizeof(String); n++){

            	cout << fn[n][0] << ":" <<"IDX:"<< pred[n].idx <<" SCORE:" << pred[n].compare <<" DEGREE:" << pred[n].pivot << " CENTER:"<< pred[n].center<< endl;
        		imshow("Prediction", detected_obj[pred[n].idx]);
        		waitKey();
        	}


            
            return 0;
        }

        else{
            continue;
        }
        
    }
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
    Point2f min_center;
    double total = 0.0;
    int total_index = 0;

    for (int deg=0; deg<(360/SCALE); deg++){

        // Process Object
        calc_profile(object, "object", &obj_center, &obj_area, false, SIZE_LIMIT);
        warp_object = warp_transform(object, obj_center, deg*SCALE , sqrt(*ans_area/obj_area));
        cropped_obj = calc_profile_n_crop(&warp_object, "object", &obj_center, &obj_area, SIZE_LIMIT*(*ans_area/obj_area));
        if(cropped_obj.rows == 1)
            continue;
        // imshow("cropped_obj", cropped_obj);
        // Compare Object
        double val = compare_contour(cropped_ans, &cropped_obj, deg*SCALE);   
        if(val < min){
            min = val;
            min_deg = deg*SCALE;
            min_center = obj_center;
        }
        total += val;
        total_index += 1;
    }

    match_result ret;
    ret.pivot = min_deg;
    ret.compare = min;
    ret.center = min_center;

    return ret;
}

Mat calc_profile_n_crop(Mat* img, String name, Point2f* center, double* area, int limit){

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
    Mat toTest;

    for( int i = 0; i< contours.size(); i++ ){   
        
        // cout << contours[i].size() << endl;
        // cout << contourArea(contours[i]) << endl;
        // if(!varify_pic(*mask)) break;

        vector<Point> convexHullPoints =  contoursConvexHull(contours[i], img);
        if (convexHullPoints.empty()) continue;
    	contours[i] = convexHullPoints;

        //int limit = 20;
        if(contourArea(contours[i]) >= limit){ //(contours[i].size()>=limit ){ //


        	
            mu.push_back(moments(contours[i], false));
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));
            *center = mc.back();
            *area = contourArea(contours[i]);
            // cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
            // cout<< "Mass area:" << contourArea(contours[i]) << endl;
            // cout<< "Mass peripheral:" << contours[i].size() << endl;
            
            boundRect.push_back(boundingRect(contours[i]));
            
            // cout << "FUCKYOU!" <<endl;
            // Point new_tl = Point((boundRect.back().tl().x - 5), (boundRect.back().tl().y - 5));
            // Point new_br = Point((boundRect.back().br().x + 5), (boundRect.back().br().y + 5));
            /*
				x = 1~cols
				y = 1~rows
            */
            Point new_tl = Point(((boundRect.back().tl().x>=6)?(boundRect.back().tl().x - 5):(boundRect.back().tl().x)), ((boundRect.back().tl().y>=6)?(boundRect.back().tl().y - 5):(boundRect.back().tl().y)));
            Point new_br = Point(((boundRect.back().br().x<=(img->cols-5))?(boundRect.back().br().x + 5):(boundRect.back().br().x)), ((boundRect.back().br().y<=(img->rows-5))?(boundRect.back().br().y + 5):(boundRect.back().br().y)));
            assert(new_tl.x >=0);
            assert(new_tl.y >=0);
            assert(new_tl.x < img->cols);
            assert(new_tl.y < img->rows);
            boundRect.back() = Rect(new_tl, new_br);
            // rectangle( *img, new_tl, new_br, Scalar(0,0,255), 2, 8, 0 );                
        	// Rect boundary = Rect(0,0, img->cols, img->rows);
        	// if( (boundRect.back() & boundary) != boundRect.back()){
        	// 	cout << "FUCK WRONG!!!!!!!!" <<endl;
        	// }

        	// cout << boundRect.back().tl() << boundRect.back().br() << endl;
        	// cout << boundary.tl() << boundary.br() << endl;

        	toTest = (*img)(boundRect.back());
        	threshold(toTest, toTest, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        	// cout<<"1-4 "<< toTest.rows <<"," << toTest.cols <<endl;
        	// cout << toTest <<endl;
        	// cout << (toTest.data == NULL) << endl;
            // imshow("DUmmy", toTest);
            // cout<<"1-5"<<endl;
            // waitKey();
            break;
            
        }
    } 

    if(boundRect.size())
        ret = toTest;//(*img)(boundRect.back());

    return ret;

    // warp_transform(img, mc.back(), 0.0, 0.8);
}

void calc_profile(Mat* img, String name, Point2f* center, double* area, bool verbose, int limit){

    // Mat ret;
    Mat canny;// = Mat();
    Scalar color = Scalar(0,0,255);
    Canny(*img, canny, 50, 200, 3);


    vector<vector<Point> > contours;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    vector<Moments> mu;
    vector<Point2f> mc;


    for( int i = 0; i< contours.size(); i++ ){   
        
        vector<Point> convexHullPoints =  contoursConvexHull(contours[i]);
    	contours[i] = convexHullPoints;
        if(contourArea(contours[i]) >= limit){ //(contours[i].size()>=limit ){ //
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
            break;
            // imshow(name,*img);
            // waitKey();
        }
    } 
}

double compare_contour(Mat* answer, Mat* object, double deg){


	// if(object==0 || answer==0){
 //    	cout << "empty!" << endl;
 //    	return 100.0;
 //    }
    
 //    imshow("Contour_ans", *answer);
 //    imshow("Contour_obj", *object);
 //    waitKey();
    double compare = 0.0;
    double compare2 = 0.0;

	int row_m = 0;
	int col_m = 0;    

    row_m = min(answer->rows,object->rows);
    col_m = min(answer->cols,object->cols);

    
    for(int i=0; i< row_m; i++){

        // double *M = (answer->ptr<double>(i));
        for(int j=0; j< col_m; j++){

        	// cout << "answer"<< answer->at<uint16_t>(i,j) << endl; 
        	// cout << "object" << object->at<uint16_t>(i,j) << endl;

        	uint16_t valu = abs(answer->at<uint16_t>(i,j) - object->at<uint16_t>(i,j));
        	// cout << i<<","<<j<<":"<<  answer->at<uint16_t>(i,j) << "," << object->at<uint16_t>(i,j) << endl;
        	if(valu > 65536/2)
				compare += 100.0;
				// compare += abs(answer->at<uint16_t>(i,j) - object->at<uint16_t>(i,j));
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

void detect_key(Mat img, Mat* mask, COLOR color, vector<Mat>* detected_obj){
    Mat img_hsv;    
    int SENSITIVITY1 = 10;
    int SENSITIVITY2 = 10;
    int VALUE = 150;
    // erode(img, img, getStructuringElement(MORPH_RECT,Size(3,3)));
    // dilate(img, img, getStructuringElement(MORPH_RECT,Size(3,3)));
    
    img.convertTo(img, -1, 1.5, -60);
    

    cvtColor(img, img_hsv, COLOR_BGR2HSV);
    // Mat mask;
    Mat mask_red, mask_red1, mask_red2;
    Mat mask_green, mask_purple, mask_blue;


    // Reg and Orange and Yellow detection
    SENSITIVITY1 = 40;
    SENSITIVITY2 = 10;
    inRange(img_hsv, Scalar(0, 60, 150), Scalar(0+SENSITIVITY1,255,255), mask_red1);
    inRange(img_hsv, Scalar(180-SENSITIVITY2, 30, 150), Scalar(180,255,255), mask_red2);
    mask_red = mask_red1 | mask_red2;

    // Green Detection
    inRange(img_hsv, Scalar(40, 10, 130), Scalar(76,255,255), mask_green); //80~157.5
    // imshow("Mask", mask_green);
    // waitKey();

    // Purple 
    SENSITIVITY1 = 20;
    inRange(img_hsv, Scalar(130, 30, 140), Scalar(165,255,255), mask_purple); // H:300d S:67~100% V:100%
    //262.5~330
    // Blue detection
    inRange(img_hsv, Scalar(76, 50, 140), Scalar(140,255,255), mask_blue); // 172.5~262.5

    
    // imshow("Mask1", mask1);
    // waitKey();
    // imshow("Mask2", mask2);
    // waitKey();
    
    mask_red = mask_red1 | mask_red2;
    // *mask = mask_blue; 

    // imshow("Mask", mask);
    // // waitKey();
    int MORPH_SIZE = 7;
    // int MORPH_SIZE2 = 5;
    erode(mask_red, mask_red, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    dilate(mask_red, mask_red, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    erode(mask_green, mask_green, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    dilate(mask_green, mask_green, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    erode(mask_purple, mask_purple, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    dilate(mask_purple, mask_purple, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    erode(mask_blue, mask_blue, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    dilate(mask_blue, mask_blue, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));


    // *mask = mask_red | mask_green | mask_purple | mask_blue;

    switch (color){
    	case RED: *mask = mask_red; break;
    	case GREEN: *mask = mask_green; break;
    	case PURPLE: *mask = mask_purple; break;
    	case BLUE: *mask = mask_blue; break;
    	case NONE: *mask = mask_red | mask_green | mask_purple | mask_blue; break;
    }

    // imshow("morph",*mask);
    // waitKey(0); 

    
    // int ret = draw_color_contour(&img, mask, COLOR(RED), detected_obj);
    // draw_color_contour(&img, &mask_green, COLOR(GREEN));
    // draw_color_contour(&img, &mask_purple, COLOR(PURPLE));
    // draw_color_contour(&img, &mask_blue, COLOR(BLUE));
    // imshow("color", img);
    // waitKey();


    // return ret;
}

int draw_color_contour(Mat* img, Mat* mask, COLOR c, vector<Mat>* detect_list){

    Mat canny;
    Scalar color;
    switch(c){
        case RED: color = Scalar(0,0,255); break;
        case GREEN: color = Scalar(0,150,0); break;
        case PURPLE: color = Scalar(150,0,150); break;
        case BLUE: color = Scalar(150,0,0); break;    
        case NONE: color = Scalar(0,0,255); break;
    }
    

    
    // int MORPH_SIZE2 = 5;
    // int MORPH_SIZE = 3;
    // erode(*mask, *mask, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    // dilate(*mask, *mask, getStructuringElement(MORPH_RECT,Size(MORPH_SIZE,MORPH_SIZE)));
    // imshow("morph", *mask);
    // waitKey();

    Canny(*mask, canny, 50, 200, 3);
    // imshow("canny",canny);
    // waitKey(); 


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    vector<Moments> mu;
    vector<Point2f> mc;
    bool valid = false;


    for( int i = 0; i< contours.size(); i++ ){   
    	

    	vector<Point> convexHullPoints =  contoursConvexHull(contours[i]);
    	// if (convexHullPoints.empty()) {
    	// 	cout << "empty!" << endl;
    	// 	continue;
    	// }
    	contours[i] = convexHullPoints;

    	drawContours( *img, contours, i, Scalar(255,0,0), 1);//CV_FILLED); 
    	// cout << contourArea(contours[i]) << endl;

 		// imshow("contour",*img);
 		// waitKey();

        if(contourArea(contours[i]) >= 1000){ //(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;



    		cout << (moments(contours[i], false)).m00 << endl;
            valid = true;
            drawContours( *img, contours, i, color, 2);//CV_FILLED); 
 			// imshow("img", *img);
 			// waitKey();
            // double angle = getOrientation(contours[i], *img);
            mu.push_back(moments(contours[i], false));
            // Point2f center = ;
            mc.push_back(Point2f(mu.back().m10/mu.back().m00 , mu.back().m01/mu.back().m00));

            ostringstream convert, convert2;
            convert << contourArea(contours[i]); 
            convert2 << contours[i].size();
            String toput = convert.str();
            String toput2 = convert2.str();
            Moments mu = moments(contours[i], false);
            Point2f cntr = Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
            Point2f cntr2 = Point2f(mu.m10/mu.m00 , mu.m01/mu.m00+20);
            putText(*img, toput, cntr, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, Scalar(255,0,0), 1, 8);
            putText(*img, toput2, cntr2, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, Scalar(255,0,0), 1, 8);

            Rect rect = boundingRect(contours[i]);
            Point new_tl = Point(((rect.tl().x>4)?(rect.tl().x - 4):(rect.tl().x)), ((rect.tl().y>5)?(rect.tl().y - 5):(rect.tl().y)));
            Point new_br = Point(((rect.br().x<=(mask->cols-4))?(rect.br().x + 4):(rect.br().x)), ((rect.br().y<=(mask->rows-5))?(rect.br().y + 5):(rect.br().y)));
            // cout << "old point:" << rect.tl()<<" " << rect.br() << endl;
            // cout << "new point:" << new_tl <<" " << new_br << endl;
            // assert(new_tl.x >=0);
            // assert(new_tl.y >=0);
            // assert(new_tl.x < img->cols);
            // assert(new_tl.y < img->rows);
            rect = Rect(new_tl, new_br);
            const Mat toadd = Mat::zeros(mask->size(), mask->type());
            Mat crop = (*mask)(rect);
            // Mat crop = Mat::zeros(Size((*mask)(rect).rows, (*mask)(rect).cols), mask->type());
            // threshold(crop, crop, 0, 65535, CV_THRESH_BINARY | CV_THRESH_OTSU);
            (*mask)(rect).copyTo(toadd(rect));

            // imshow("before", crop);
            // cout << crop.rows << "x" << crop.cols << endl;
            // for(int n=-1; n<10; ++n){
            // 	// for(int m=-1; m<crop.cols; m++){
            // 		cout << crop.at<uint16_t>(n,crop.cols-1) << " ";
            // 		crop.at<uint16_t>(n,crop.cols-1) = 65535;
            // 	// }
            // 	// cout << endl;
            // 	// cout << crop.row(i) << endl;
            // 	// waitKey();	
            // }
            // imshow("after", crop);
            
            // bool edge = false;
            // int edge_count = 0;
            // for(int n=0; n<crop.rows; n++){
            // 	cout << n << ":" << crop.at<uint16_t>(n,0) << endl;
            // 	cout << n << "_:" << crop.at<uint16_t>(n,crop.cols-1) << endl;
            // 	if(crop.at<uint16_t>(n,0) == 65535){
            // 		cout<< "white edge at left" <<endl;
            // 		edge_count ++;
            // 		if(edge_count == 10){
            // 			edge = true;
            // 			break;
            // 		}
            // 	}
            // 	if(crop.at<uint16_t>(n,crop.cols-1) == 65535){
            // 		cout<< "white edge at right" <<endl;
            // 		edge_count ++;
            // 		if(edge_count == 10){
            // 			edge = true;
            // 			break;
            // 		}
            // 	}
            // }
            // for(int m=0; m<crop.cols; m++){
            // 	if(crop.at<uint16_t>(0,m) == 65535){
            // 		cout<< "white edge at top" <<endl;
            // 		edge_count ++;
            // 		if(edge_count == 10){
            // 			edge = true;
            // 			break;
            // 		}
            // 	}
            // 	if(crop.at<uint16_t>(crop.rows-1,m) == 65535){
            // 		cout<< "white edge at bottom" <<endl;
            // 		edge_count ++;
            // 		if(edge_count == 10){
            // 			edge = true;
            // 			break;
            // 		}
            // 	}
            // }


            // imshow("Test", crop);
            // waitKey();            

            // if(!edge)
			detect_list->push_back(toadd);
            // update_detect_list(center, toadd, detect_list, center_list);
			// show_detect_list(detect_list);

            // cout<< "Mass center:" << mu.back().m10/mu.back().m00 << ", " << mu.back().m01/mu.back().m00 <<endl;
            // cout<< "Mass area:" << contourArea(contours[i]) << endl;
            // cout<< "Mass peripheral:" << contours[i].size() << endl;
            
            // waitKey();
        }
    } 
    imshow("contour",*img);
    return detect_list->size();
}

// void update_detect_list(Point2f center, Mat toadd, vector<Mat>* detect_list){

// 	double MIN_THRESHOLD = 20.0;
// 	bool add = true;

// 	for(int i=0; i<detect_list->size(); i++){
// 		double dist = norm(center - center_list->at(i));
// 		cout << dist << endl;
// 		if( dist <= MIN_THRESHOLD){
// 			add = false;
// 			break;
// 		}
// 	}
// 	if(add){
// 		detect_list->push_back(toadd);
// 		center_list->push_back(center);
// 	}
// }

// void show_detect_list(vector<Mat>* detect_list){
// 	Mat detect_img;
// 	for(int i=0; i<detect_list->size(); i++){
// 		detect_img = detect_img | detect_list->at(i);
// 	}

// 	imshow("detect_list", detect_img);
// 	waitKey();
// }


Mat tan_triggs_preprocessing(InputArray src, float alpha, float tau, float gamma, int sigma0, int sigma1) {

    // Convert to floating point:
    Mat X = src.getMat();
    X.convertTo(X, CV_32FC1);
    // Start preprocessing:
    Mat I;
    pow(X, gamma, I);
    // Calculate the DOG Image:
    {
        Mat gaussian0, gaussian1;
        // Kernel Size:
        int kernel_sz0 = (3*sigma0);
        int kernel_sz1 = (3*sigma1);
        // Make them odd for OpenCV:
        kernel_sz0 += ((kernel_sz0 % 2) == 0) ? 1 : 0;
        kernel_sz1 += ((kernel_sz1 % 2) == 0) ? 1 : 0;
        GaussianBlur(I, gaussian0, Size(kernel_sz0,kernel_sz0), sigma0, sigma0, BORDER_REPLICATE);
        GaussianBlur(I, gaussian1, Size(kernel_sz1,kernel_sz1), sigma1, sigma1, BORDER_REPLICATE);
        subtract(gaussian0, gaussian1, I);
    }

    {
        double meanI = 0.0;
        {
            Mat tmp;
            pow(abs(I), alpha, tmp);
            meanI = mean(tmp).val[0];

        }
        I = I / pow(meanI, 1.0/alpha);
    }

    {
        double meanI = 0.0;
        {
            Mat tmp;
            pow(min(abs(I), tau), alpha, tmp);
            meanI = mean(tmp).val[0];
        }
        I = I / pow(meanI, 1.0/alpha);
    }

    // Squash into the tanh:
    {
        Mat exp_x, exp_negx;
	exp( I / tau, exp_x );
	exp( -I / tau, exp_negx );
	divide( exp_x - exp_negx, exp_x + exp_negx, I );
        I = tau * I;
    }
    return I;
}

Mat norm_0_255(const Mat& src) {
    // Create and return normalized image:
    Mat dst;
    switch(src.channels()) {
    case 1:
        cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
        break;
    case 3:
        cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC3);
        break;
    default:
        src.copyTo(dst);
        break;
    }
    return dst;
}

vector<Point> contoursConvexHull( vector<Point> contours, Mat* img ){
    vector<Point> result;
    vector<Point> pts;
    // for ( size_t i = 0; i< contours.size(); i++)
    for ( size_t i = 0; i< contours.size(); i++){
    	if( img != 0 ){
    		if( contours[i].x == 0 || contours[i].x == img->cols-1 || contours[i].y == 0 || contours[i].y == img->rows-1){
    			result.clear();
    			cout << "coutour on boundary...!!" << endl;
    			return result;
    		}	
    	}
        pts.push_back(contours[i]);
    }
    convexHull( pts, result );
    return result;
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

	Mat img_canny, img_grey;
    img.convertTo(img, -1, 1.5, -40);
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

    Canny(img_grey, img_canny, 80, 240, 3);
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

        if(contourArea(contours[i]) >= 1500){//(contours[i].size()>=limit ){ //
            // cout << contourArea(contours[i]) << endl;
            // cout << contours[i].size() << endl;
            drawContours( img, contours, i, Scalar(0,0,255), 2);    //CV_FILLED
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
            // imshow("contour",img);
            // waitKey();
        }
        
        //
        
    } 
    imshow("img2",img);
    // waitKey(0); 
}