#ifndef _IMGPROC_H_
#define _IMGPROC_H_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
using namespace std;
#include <math.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <imgproc.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ibvs_formation_bearing/bearing.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <map>
#include <fstream>
#define pi 3.141592653589
//#define show_images
#define RANSAC_ellipse

class getCircle : public nodelet::Nodelet
{
 public:
	  double fx;
	  double fy;
	  double cx;
	  double cy;
	  double d0;
	  double d1;
	  double d2;
	  double d3;
	  double d4;
	  double d5;
	  double d6;
	  double d7;
	  int RANSAC_iterations;
	  int erosion_size;
	  int dilation_size;
	  int dilation_elem;
	  int erosion_elem;
	  double cylinder_size;
	  string color1;
	  string color2;
	  int color11_low;
	  int color12_low;
	  int color13_low;
	  int color21_low;
	  int color22_low;
	  int color23_low;
	  int color31_low;
	  int color32_low;
	  int color33_low;
	  int color11_high;
	  int color12_high;
	  int color13_high;
	  int color21_high;
	  int color22_high;
	  int color23_high;
	  int color31_high;
	  int color32_high;
	  int color33_high;

	  RNG rng;
	  ros::Publisher ellipse_pos_pub_;
	  ibvs_formation_bearing::bearing ellipses;
      image_transport::Publisher image_ellipse;

      void Erosion(const Mat& src);
      void Dilation(const Mat& src);
      void Moprh(const Mat& src);
      Mat FilterColors(const Mat& src);
      void getColor(cv::Mat &srchsv, cv::Mat &src, cv::Mat &contour_img, string color, vector<RotatedRect>& minEllipse, vector<double>* bearing);
      void ellipsePublisher(Mat* src, vector<Point2f>* P1, vector<Point2f>* P2, RotatedRect* minEllipse, vector<double>* bearing);
      void RANSAC_thread(vector<Point> contours, RotatedRect* minEllipse, vector<Point2f>* P1, vector<Point2f>* P2, int sample_num);

 private:
  void onInit(void);

  void camera_callback(const sensor_msgs::Image::ConstPtr &img);
  //,
	//	  const sensor_msgs::CameraInfo::ConstPtr &c_info);

  image_transport::CameraSubscriber sub_camera_;
  ros::Subscriber sub_camera_imu_;


};

void get_5_random_num(int max_num, int* rand_num);




#endif
