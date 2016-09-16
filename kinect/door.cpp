#include "libfreenect.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <map>





using namespace cv;
using namespace std;



int lowThreshold = 50;
int edge_ratio = 3;
int kernel_size = 3;


float t_gamma[2048];



class Point3{
public:
	Point3(double xi=0, double yi=0, double zi=0):x(xi),y(yi),z(zi)
	{}
	~Point3(){}

	bool operator==( Point3 o){
		return(x == o.x && y == o.y && z == o.z);
	}

	Point3 operator+(Point3 other){
		return Point3(x+other.x, y+other.y, z+other.z);
	}

	friend ostream &operator<<(ostream& output, const Point3 &pt)
	{
		output << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")" << endl;
		return output; 
	}

	double x;
	double y;
	double z;
};



class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {
			
			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			//std::cout << "RGB callback" << std::endl;
			m_rgb_mutex.lock();
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			rgbMat.data = rgb;
			m_new_rgb_frame = true;
			m_rgb_mutex.unlock();
		};
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			//std::cout << "Depth callback" << std::endl;
			m_depth_mutex.lock();
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			m_depth_mutex.unlock();
		}
		
		bool getVideo(Mat& output) {
			m_rgb_mutex.lock();
			if(m_new_rgb_frame) {
				cv::cvtColor(rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				m_rgb_mutex.unlock();
				return true;
			} else {
				m_rgb_mutex.unlock();
				return false;
			}
		}
		
		bool getDepth(Mat& output) {
				m_depth_mutex.lock();
				if(m_new_depth_frame) {
					depthMat.copyTo(output);
					m_new_depth_frame = false;
					m_depth_mutex.unlock();
					return true;
				} else {
					m_depth_mutex.unlock();
					return false;
				}
			}
		
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};



float RawDepthToMeters(int depthValue)
{
    if (depthValue < 2047)
    {
        return float(1.0 / (double(depthValue) * -0.0030711016 + 3.3309495161));
    }
    return 0.0f;
}

Point3 DepthToWorld(int x, int y, int depthValue) // (x,y) ! -> (row,col)
{
    static const double fx_d = 1.0 / 5.5664659286646634e+02;
    static const double fy_d = 1.0 / 5.5440902782330022e+02;	
    static const double cx_d = 3.0414055153733733e+02;
    static const double cy_d = 2.3977154993635511e+02;

    Point3 result;
    const double depth = RawDepthToMeters(depthValue);
    //const double depth = 1.05;
    result.x = -double((x - cx_d) * depth * fx_d);
    result.y = -double((y - cy_d) * depth * fy_d);
    result.z = (depth);
    return result;
}

Point3 inner_product(double R[3][3], Point3 p3d){
	double result[3];
	for(int i=0; i<3; ++i){
		result[i] = R[i][0] * p3d.x + R[i][1] * p3d.y + R[i][2] * p3d.z;
	}
	return Point3(result[0], result[1], result[2]);
}

Point2i WorldToColor(Point3 point_3d){   // (x,y)->(row,col)

	static const double fx_rgb = 5.2921508098293293e+02;
	static const double fy_rgb = 5.2556393630057437e+02;
	static const double	cx_rgb = 3.2894272028759258e+02;
	static const double	cy_rgb = 2.6748068171871557e+02;


	double R[3][3] =   {{9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02},
						{-1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02},
						{1.7470421412464927e-02, 1.2275341476520762e-02,	9.9977202419716948e-01}};

	Point3 T =  Point3(1.9985242312092553e-02, -7.4423738761617583e-04, -1.0916736334336222e-02);

	Point3 p3d_tran = inner_product(R, point_3d) + T;

	int x = p3d_tran.x * fx_rgb / p3d_tran.z + cx_rgb;
	int y = p3d_tran.y * fy_rgb / p3d_tran.z + cy_rgb;

	return Point2i(y,x);
}
 



int main(int argc, char **argv) {

	for (size_t i=0; i<2047; i++)
	{
		const float k1 = 1.1863;
		const float k2 = 2842.5;
		const float k3 = 0.1236;
		const float depth = k3 * tanf(i/k2 + k1);
		t_gamma[i] = depth;
	}
	t_gamma[2047] = 0;

	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	
	Mat gray;
	Mat depthMat(Size(640,480),CV_16UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));

	Mat depth3 (Size(640,480),CV_8UC3);
	Mat detected_edges_c;
	Mat detected_edges_d;

	Mat drawing_c, drawing_d, merge;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> approxShape;


	map<pair<int,int>,Point3> map_c2w;
	
	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:
	
	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	
	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);	
	namedWindow("merge",CV_WINDOW_AUTOSIZE);	
	//namedWindow( "Edge Map Color", CV_WINDOW_AUTOSIZE );
	//namedWindow( "Edge Map Depth", CV_WINDOW_AUTOSIZE );
	device.startVideo();
	device.startDepth();
	device.setTiltDegrees(0);
	while (!die) {
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		depthMat.convertTo(depth3, CV_8UC3, 255.0/2048.0 * 50, -4400);
		

		/*

		//////////////////////////////////////////////////////////////////////
		map_c2w.clear();
		for(int y=0;y<depthMat.rows;y++)
		{
			for(int x=0;x<depthMat.cols;x++)
			{
				Point3 p3d = DepthToWorld(x,y,depthMat.at<uint16_t>(x,y));
				Point2i pixel_color = WorldToColor(p3d);
				if(pixel_color.x<0 || pixel_color.y<0 || pixel_color.x>=rgbMat.rows || pixel_color.y>=rgbMat.cols)
					continue;
				//map_c2w[pair<int,int>(pixel_color.x,pixel_color.y)] = p3d;
			}
		}
		//////////////////////////////////////////////////////////////////////


		*/




		/// Convert the image to grayscale
		cvtColor( rgbMat, gray, CV_BGR2GRAY );

		blur( gray, detected_edges_c, Size(3,3) );
		blur( depth3, detected_edges_d, Size(3,3) );

		/// Canny detector
		Canny( detected_edges_c, detected_edges_c, 10, lowThreshold * edge_ratio, kernel_size );
		Canny( detected_edges_d, detected_edges_d, 10, lowThreshold * edge_ratio, kernel_size );

		contours.clear();
		hierarchy.clear();
		approxShape.clear();

		//RNG rng(12345);
		findContours( detected_edges_c, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		/// Draw contours
		drawing_c = Mat::zeros( detected_edges_c.size(), CV_8UC3 );
		


		for(size_t i = 0; i < contours.size(); i++){
			if(contourArea( contours[i],false)<200 || contourArea( contours[i],false)>1200)
				continue;
			approxPolyDP(contours[i], approxShape, arcLength(Mat(contours[i]), true)*0.04, true);
			drawContours(drawing_c, contours, i, Scalar(255, 0, 0), CV_FILLED);   // fill BLUE
		}
		
		imshow( "rgb", drawing_c );

		contours.clear();
		hierarchy.clear();
		approxShape.clear();

		findContours( detected_edges_d, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		/// Draw contours
		drawing_d = Mat::zeros( detected_edges_d.size(), CV_8UC3 );

		
		for(size_t i = 0; i < contours.size(); i++){
			if(contourArea( contours[i],false)<200 || contourArea( contours[i],false)>800)
				continue;
			approxPolyDP(contours[i], approxShape, arcLength(Mat(contours[i]), true)*0.04, true);
			drawContours(drawing_d, contours, i, Scalar(0, 0, 255), CV_FILLED);   // fill BLUE
			Moments mu = moments( contours[i], false );;
  			///  Get the mass centers:
  			Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
  			circle( drawing_d, mc, 4, Scalar(255, 255, 255), -1, 8, 0 );
  			//cout<<DepthToWorld(mc.y, mc.x, depthMat.at<uint16_t>(mc.y, mc.x))<<endl;
  			cout<<DepthToWorld(mc.x, mc.y, depthMat.at<uint16_t>(mc.y, mc.x))<<endl;
		}

		
		
		imshow( "depth", drawing_d );

		merge = Mat::zeros( detected_edges_d.size(), CV_8UC3 );

		


		imshow( "merge", depth3 );
		

		char k = cvWaitKey(5);
		
		if( k == 27 ){
			cvDestroyWindow("rgb");
			cvDestroyWindow("depth");
			cvDestroyWindow("merge");
			break;
		}
	}
	
	device.stopVideo();
	device.stopDepth();
	return 0;
}

