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

#define pi 3.141592653589

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
	  int RANSAC_iterations;
	  image_transport::Publisher image_thresholded_pub_;
	  ros::Publisher ellipse_pos_pub_;
      ibvs_formation_bearing::bearing ellipse_direction;

 private:
  void onInit(void);

  void camera_callback(const sensor_msgs::Image::ConstPtr &img);
  //,
	//	  const sensor_msgs::CameraInfo::ConstPtr &c_info);

  image_transport::CameraSubscriber sub_camera_;
  ros::Subscriber sub_camera_imu_;


};

void get_5_random_num(int max_num, int* rand_num);
Mat Erosion(const Mat& src);
Mat Dilation(const Mat& src);
Mat FilterColors(const Mat& src);
Mat getColor(cv::Mat &srcBGR);

void RANSAC_thread(vector<Point> contours, RotatedRect* minEllipse, vector<Point2f>* P1, vector<Point2f>* P2, int sample_num);


#endif
