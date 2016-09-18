#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
#include <math.h>
#include "SerialPortController.h"
#include "WheelController.h"
// #include "utils.h"

using namespace std;
using namespace cv;




enum{
    SCAN_STEP = 5,			  // in pixels
	LINE_REJECT_DEGREES = 5, // in degrees
    BW_TRESHOLD = 250,		  // edge response strength to recognize for 'WHITE'
    BORDERX = 10,			  // px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,	  // px
	
	CANNY_MIN_TRESHOLD = 1,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 100, // edge detector maximum hysteresis threshold

	HOUGH_TRESHOLD = 50,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 50,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 100,   // join lines to one with smaller than this gaps

	CAR_DETECT_LINES = 4,    // minimum lines for a region to pass validation as a 'CAR'
	CAR_H_LINE_LENGTH = 10,  // minimum horizontal line length from car body in px

	MAX_VEHICLE_SAMPLES = 30,      // max vehicle detection sampling history
	CAR_DETECT_POSITIVE_SAMPLES = MAX_VEHICLE_SAMPLES-2, // probability positive matches for valid car
	MAX_VEHICLE_NO_UPDATE_FREQ = 15 // remove car after this much no update frames
};

struct Lane {
	Lane(){}
	Lane(CvPoint a, CvPoint b, float angle, float kl, float bl): p0(a),p1(b),angle(angle),
		votes(0),visited(false),found(false),k(kl),b(bl) { }

	CvPoint p0, p1;
	int votes;
	bool visited, found;
	float angle, k, b;
};


Mat img, img1;
Mat img_grey, img_canny, img_hough;

SerialPortController* serial_port_controller;
WheelController* wheel;

const double Kx = 1.0;
const double Ky = 1.0;
const double Kw = 0.125;
double vx, vy, w0;

// int WIDTH = 1000/2;
// int HEIGHT =  1334/2;
int WIDTH = 640;
int HEIGHT =  480;


void processLanes(vector<Vec4i> lines, Mat* img_hough, Mat*, Mat*);
Mat color_detection(Mat*);
Mat white_detection(Mat*);
void draw_forward_line(Mat* img, double deg, Scalar);
int check_lane_position(Mat*, LineIterator);
void lane_detection(Mat* img, Mat*, Mat*);
void detect_bridge(Mat* img, String label);

int main(){

	serial_port_controller = new SerialPortController("/dev/cu.wchusbserial1410");
	wheel = new WheelController(serial_port_controller);
	
	VideoCapture cap("../road_0915.mov"); //400*222
	// VideoCapture cap(1);

	if (!cap.isOpened()){
		std::cout << "!!! Failed to open file: " << endl;
		return -1;
	}

	WIDTH = cap.get(CV_CAP_PROP_FRAME_WIDTH)/4; //get the width of frames of the video
	HEIGHT = cap.get(CV_CAP_PROP_FRAME_HEIGHT)/4; //get the height of frames of the video

	cout << "Frame size : " << WIDTH << " x " << HEIGHT << endl;
	
	int count = 0;

	while(1){

		Mat frame;


		// frame = imread("../img/road1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
		// WIDTH = frame.rows/4;
		// HEIGHT = frame.cols/4;

		
        bool success = cap.read(frame);
        if(!success){
            return 1;
        }
        count++;
        // cout << count << endl;
		// if (count%50 != 0) continue;
		// if (count< 1000) continue;
		

        cv::resize( frame, frame, cv::Size(frame.cols/4, frame.rows/4) );
        // imshow("video", frame);

        Mat half_frame(frame, cvRect(0,HEIGHT*0.3,WIDTH,HEIGHT*0.7));
        // imshow("half", half_frame);

        // Mat green_mask = color_detection(&half_frame);
        Mat white_mask = white_detection(&half_frame);
        // imshow("green",green_mask);
        
        
		lane_detection(&half_frame, 0, &white_mask);
		imshow("white",white_mask);
		imshow("lane",half_frame);
		// waitKey();
		// detect_bridge(&img, "img");
		// detect_bridge(&img1, "img1");

		// waitKey(0);
		char key = cvWaitKey(20);
		if (key == 27) // ESC
			break;	
	    
    }
}

void detect_bridge(Mat* img, String label){
	
	// imshow("img", *img);
	// waitKey(0);

	cvtColor(*img, img_grey, CV_BGR2GRAY);

	vector<KeyPoint> keyPoints;

	SurfFeatureDetector surf(2500);
	surf.detect(img_grey, keyPoints);

	cout << "Size : " << keyPoints.size() << endl; 
	for(size_t i=0; i<keyPoints.size(); i++){
		Point center;
		// cout << keyPoints[i].pt <<endl;
		center = keyPoints[i].pt;
		circle(*img, center, 4, Scalar(0,0,255), 2);
	}
	imshow(label, *img);
	// waitKey(0);

}

Mat color_detection(Mat* img){
	Mat img_hsv;
	// Mat half_img(*img, cvRect(0,HEIGHT*0.3,WIDTH,HEIGHT*0.7));

	cvtColor(*img, img_hsv, COLOR_BGR2HSV);
    Mat mask, mask1, mask2;

    // inRange(img_hsv, Scalar(30, 100, 100), Scalar(90,255,255), mask1); //GREEN
    inRange(img_hsv, Scalar(40, 10, 130), Scalar(76,255,255), mask1); //GREEN
    
    // imshow("Mask1", mask1);
    // waitKey(0);

    mask = mask1 | mask2;

    erode(mask, mask, getStructuringElement(MORPH_RECT,Size(9,9)));
    dilate(mask, mask, getStructuringElement(MORPH_RECT,Size(9,9)));
    // imshow("mask",mask);
    // waitKey(0); 

    return mask;
}

Mat white_detection(Mat* img){
	Mat img_grey;
	// Mat half_img(*img, cvRect(0,HEIGHT*0.3,WIDTH,HEIGHT*0.7));
	cvtColor(*img, img_grey, CV_BGR2GRAY);
	threshold(img_grey, img_grey, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

	erode(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(3,3)));
    dilate(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(3,3)));
	// imshow("white",img_grey);
    // waitKey(0); 
    return img_grey;
}

void lane_detection(Mat* img, Mat* green, Mat* white){

	
	// Mat half_img;
	// half_img = img->clone();
	// imshow("crop",half_img);
	    
	// cvtColor(*img, img_grey, CV_BGR2GRAY);
	// imshow("grey",img_grey);
	// waitKey(0);

	// threshold(img_grey, img_grey, 100, 255, cv::THRESH_OTSU);
	// imshow("thresh",img_grey);
	// waitKey(0);
	// GaussianBlur(img_grey, img_grey, Size(5,5), 0);
	// imshow("blur",img_grey);
	// erode(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(9,9)));
	// dilate(img_grey, img_grey, getStructuringElement(MORPH_RECT,Size(9,9)));
    
    // imshow("blur",img_grey);
    // waitKey(0);

	// Canny(img_grey, img_canny, 1, 300, 3);
	Canny(*white, img_canny, 1, 300, 3);
	// imshow("canny",img_canny);
	// waitKey(0);

	vector<Vec4i> lines;
	double rho = 1;
	double theta = CV_PI/180;
	HoughLinesP(img_canny, lines, rho, theta, 30, 50, 30);
		

	Mat img_hough(Size(img_canny.cols,img_canny.rows),CV_8UC3);
	// cout << "lines : "<< lines.size() <<endl;

	processLanes(lines, img, green, white);
	// for( size_t i = 0; i < lines.size(); i++ )
	// {
	//     line( img_hough, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
	// imshow("img",*img);
	// imshow("canny",img_canny);

}


void draw_forward_line(Mat* img, Point2f A, double deg, Scalar c= Scalar(0,255,255)){

	Point2f B;
	double k, b;
	double sign = (-1)*deg/abs(deg);
	// A = Point2f(img->cols/2, img->rows);
	k = tan( sign*(90 - abs(deg))/180*CV_PI );
	b = A.y - k * A.x;

	B.y = 1;
	B.x = (1 - b)/k;

	line(*img, A, B, c, 3, 8 );

	// imshow("MIDDLE", *img);
	// waitKey();



}



void processLanes(vector<Vec4i> lines, Mat* img_hough, Mat* green, Mat* white) {

	// classify lines to left/right side
	vector<Lane> left, right, mid;
	// vector<vector<Lane> > left_

	draw_forward_line(img_hough, Point2f(img_hough->cols*3/10, img_hough->rows), -5.0);

	for(size_t i = 0; i < (lines).size(); i++ ){

		Point2f A, B;
		A = Point((lines)[i][0],(lines)[i][1]);
		B = Point((lines)[i][2],(lines)[i][3]);
		if(A.x > B.x){
			Point2f temp = A;
			A = B;
			B = temp;
		}

		double dy = A.y - B.y;
		double dx = A.x - B.x;
		
		// assume that vanishing point is close to the image horizontal center
		// calculate line parameters: y = kx + b;
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0!  
		double k = dy/(double)dx;
		float angle = atan(dy/dx) * 180/CV_PI;
		float b = lines[i][1] - k * lines[i][0];

		// cout << "Angle:" << angle << ","<< k << ","<<atan(dy/dx) << endl;

		if (fabs(angle) <= LINE_REJECT_DEGREES) { // reject near horizontal lines
			continue;
		}

		if( angle < 0){
			angle = (1)*(90-abs(angle));
		}
		else if(angle >=0){
			angle = (-1)*(90-abs(angle));
		}

		// LineIterator it(*img_hough, Point((lines)[i][0],(lines)[i][1]), Point((lines)[i][2],(lines)[i][3]), 8, true);

		// check_lane_position(img_hough, it); //(lines)[i][0],(lines)[i][1], (lines)[i][2],(lines)[i][3]);

		int midx = (lines[i][0] + lines[i][2]) / 2;
		
		Point2f C, D, E, F, G;
		C.x = (1 - b) / k;//B.x + (B.x - A.x) / norm(A-B) * length; // (y-b)/k
		C.y = 1;
		// cout << "Test" << C << endl;
		if(C.x > img_hough->cols){
			C.x = img_hough->cols;
			C.y = k * C.x + b;
		}
		else if(C.x < 1 && C.x < C.y){
			C.x = 1;
			C.y = k * C.x + b;
		}
		// else if(C.y > img_hough->rows){
		// 	C.y = img_hough->rows;
		// 	C.x = (y - b) / k;
		// }
		// else if(C.y < 1){
		// 	C.y = 1;
		// 	C.x = (y - b) / k;
		// }

		D.x = (img_hough->rows - b) / k;
		D.y = img_hough->rows;
		// cout << "Test" << D << endl;
		if(D.x < 1){
			D.x = 1;
			D.y = k + b;
		}
		else if(D.x > img_hough->cols && (D.x-img_hough->cols) > (D.y-img_hough->rows)){
			D.x = img_hough->cols;
			D.y = k * D.x + b;
		}

		E.x = (C.x+D.x)/2;
		E.y = (C.y+D.y)/2;
		int LENGTH = 100;
		float new_k;


		F.x = E.x + k/sqrt(1+pow(k,2))*LENGTH;
		F.y = E.y + (-1)*(LENGTH/sqrt(1+pow(k,2)));
		if(F.x > img_hough->cols){ // y = -(1/k)*x + b => b = y - (-1/k)*x
			float new_length = (img_hough->cols - E.x)/k;
			F.x = img_hough->cols;
			F.y = E.y + (-1)*new_length;
		}
		else if(F.x < 1 && F.x < F.y){
			float new_length = (E.x - 1)/k;
			F.x = 1;
			F.y = E.y + (-1)*new_length;	
		}
		else if(F.y > img_hough->rows){
			float new_length = (img_hough->rows - E.y)/(-1);
			F.y = img_hough->rows;
			F.x = E.x + k*new_length;
		}
		else if(F.y < 1){
			float new_length = (E.y - 1);
			F.y = 1;
			F.x = E.x + k*new_length;
		}


		G.x = E.x - k*LENGTH/sqrt(1+pow(k,2));
		G.y = E.y - (-1)*LENGTH/sqrt(1+pow(k,2));
		if(G.x > img_hough->cols){ // y = -(1/k)*x + b => b = y - (-1/k)*x
			float new_length = (img_hough->cols - E.x)/k;
			G.x = img_hough->cols;
			G.y = E.y + (-1)*new_length;
		}
		else if(G.x < 1){
			float new_length = (E.x - 1)/k;
			G.x = 1;
			G.y = E.y + (-1)*new_length;	
		}
		else if(G.y > img_hough->rows){
			float new_length = (img_hough->rows - E.y)/(-1);
			G.y = img_hough->rows;
			G.x = E.x + k*new_length;
		}
		else if(G.y < 1){
			float new_length = (E.y - 1)/(-1);
			G.y = 1;
			G.x = E.x + k*new_length;
		}

		if(!(F.x <= G.x)){
			Point2f temp = F;
			F = G;
			G = temp;
			new_k = (-1.0)*k;
		}
		else
			new_k = k;
		// cout << A << " , " << B << ","<< k << ", " << new_k<< endl;
		// cout << C << " , " << D << "," << E << endl;
		// cout << E << " , " << F << endl;
		// imshow("LINES", *img_hough);
		// waitKey();
		// line( *img_hough, E, F, Scalar(0,255,255), 3, 8 );
		// imshow("LINES", *img_hough);
		// waitKey();
		// line( *img_hough, E, G, Scalar(0,255,0), 3, 8 );
		// imshow("LINES", *img_hough);
		// waitKey();
		// bool left_white = (int)white->at<uchar>(G) == 255;
		// bool right_white = (int)white->at<uchar>(F) == 255;

		// vote_color()
		Point2f temp = E;
		double vote = 0.0;
		double idx = 0;
		float limit = 2.0;
		double avg_left, avg_right;
		// cout << "k:" << new_k << endl;
		while((temp.x >= F.x)){
			// cout << temp << endl;
			vote += (int)white->at<uchar>(temp);
			idx ++;
			temp.x = temp.x + new_k;
			temp.y = temp.y + (-1);
			if (temp.x > img_hough->cols || temp.x < 1) break;
			if (temp.y > img_hough->rows || temp.y < 1) break;
		}
		avg_left = (vote/idx > 255/2)? 255 : 0;
		// cout << "average left : " << vote/idx << endl;
		temp = E;
		vote = 0.0;
		idx = 0;
		while((temp.x <= G.x)){
			vote += (int)white->at<uchar>(temp);
			idx ++;
			temp.x = temp.x - new_k;
			temp.y = temp.y + 1;
			if (temp.x > img_hough->cols || temp.x < 1) break;
			if (temp.y > img_hough->rows || temp.y < 1) break;
		}
		avg_right = (vote/idx > 255/2)? 255 : 0;
		// cout << "average right : " << vote/idx << endl;

		
		if(avg_left == 255 && avg_right == 0){
			// cout << "left lane!" <<endl;
			line( *img_hough, C, D, Scalar(0,0,255), 3, 8 );
			right.push_back(Lane(C, D, angle, k, b));
		}
		else if(avg_left == 0 && avg_right == 255){
			// cout << "right lane!" <<endl;
			line( *img_hough, C, D, Scalar(255,0,0), 3, 8 );
			left.push_back(Lane(C, D, angle, k, b));
		}
		else if(avg_left == 255 && avg_right == 255){
			// cout << "mid lane!" << endl;
			line( *img_hough, C, D, Scalar(255,0,255), 3, 8 );
			mid.push_back(Lane(C, D, angle, k, b));

		}
		// imshow("LINES", *img_hough);
		// waitKey();

		// if (midx < WIDTH/2 ) {
		// 	left.push_back(Lane(C, D, angle, k, b));
			
		// } else if (midx >= WIDTH/2) {
		// 	right.push_back(Lane(C, D, angle, k, b));
		// }
        
    }


    /* FSM: FORWARD---(mid.size()>0)--->DETECT_MID_LANE---(mid_lane_disappear)--_>DETECT_TURN---(sleep for n sec)--->
    		DETECT_TURN_END ---(mid_lane_disappear)----> FORWARD
    	FOWARD: 0
    	DETECT_MID_LANE: 1
    	DETECT_TURN: 2
    	DETECT_TURN_END: 3
    	STOP: 4
    */





    // if(!trigger_middle & mid.size()){

    // }
    // else if(trigger_middle & !mid.size()){

    // }
    if(right.size() && left.size()){
    	Point2f far;
	    double a = right[0].k;
	    double b = right[0].b;
	    double c = left[0].k;
	    double d = left[0].b;
	    
	    far.y =  (b*c-a*d)/(c-a);// (right[0].p1.x + left[0].p1.x)/2;
	    far.x =  (d-b)/(a-c);
	    // cout << "Far:" << far<< endl;


	    Point2f near_r, near_l, near;
	    near_r.y = img_hough->rows;
	    near_r.x = (near_r.y - b)/a;
	    near_l.y = img_hough->rows;
	    near_l.x = (near_l.y - d)/c;
	    near.x = (near_r.x + near_l.x)/2;
	    near.y = (near_r.y + near_l.y)/2;

	    double current_deg = tan((far.x - near.x) / (near.y - far.y) )*180/CV_PI;
	    double translation = (near.x - far.x);
	    cout << "delta deg: " << (5 - current_deg) << endl;
	    cout << "cur trans: " << translation << endl;

	    // cout << "theta:" << (-5 - current_deg) << endl;
	    draw_forward_line(img_hough, near, current_deg, Scalar(238,130,238));
	    vx = translation * Kx;
	    vy = 0.03 * Ky;
	    w0 = (5 - current_deg) * Kw;  
	    // wheel->setMotionRect(vx, vy, w0);



    }
    else if(!right.size() && left.size()){
    	cout << "Turn right a bit!" << endl;
    	// vx = translation * Kx;
	    // vy = 0.03 * Ky;
	    // w0 = (5 - current_deg) * Kw;  
    }
    else if(!left.size() && right.size()){
    	cout << "Turn left a bit" << endl;
    }
    else if(!left.size() && !right.size()){
    	// cout << "WHAT THE FUCK??" << endl;
    }
    
}


int check_lane_position(Mat* img, LineIterator it){
	/*
		0 : middle
		1 : right
		2 : left
	*/
	const int GREEN = 0;
	const int WHITE = 1;
	const int DEPTH = 50;
	
	for(int i=0; i<it.count; i++, ++it){
		int x = it.pos().x;
		int y = it.pos().y;
		vector <float> color(3);
		for(int n=x; n>x-DEPTH; n--){
			color[0] += (*img).at<Vec3f>(y,n).val[0];
			color[1] += (*img).at<Vec3f>(y,n).val[1];
			color[2] += (*img).at<Vec3f>(y,n).val[2];
			cout << (int)(*img).at<Vec3f>(y,n).val[0] << "," << (int)(*img).at<Vec3f>(y,n).val[1] << "," << (int)(*img).at<Vec3f>(y,n).val[2] << endl;

		}
		cout << x << ","<< y << ":" << color[0]/DEPTH << " " << color[1]/DEPTH << " " << color[2]/DEPTH << endl;
		int dummy;
		cin >> dummy;
	}


}



