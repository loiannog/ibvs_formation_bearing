#ifndef _IMGPROC_H_
#define _IMGPROC_H_

#include <iostream>
#include "opencv2/core/core.hpp"
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMeEllipse.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImageConvert.h>
#include <visp/vpNurbs.h>
#include <visp/vpMeNurbs.h>
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
#define pi 3.141592653589

class getCircle : public nodelet::Nodelet
{
 public:
	  int image_threshold;
	  double opt_sizePrecision;
	  double opt_grayLevelPrecision;
	  double opt_ellipsoidShapePrecision;
	  int height_min;
	  int height_max;
	  int width_min;
	  int width_max;
	  int GrayLevelMin;
	  int GrayLevelMax;
	  int Surface;
	  double fx;
	  double fy;
	  double cx;
	  double cy;
	  double d0;
	  double d1;
	  double d2;
	  double d3;
	  image_transport::Publisher image_thresholded_pub_;
	  ros::Publisher cylinder_pos_pub_;


 private:
  void onInit(void);

  void camera_callback(const sensor_msgs::Image::ConstPtr &img);
  //,
	//	  const sensor_msgs::CameraInfo::ConstPtr &c_info);

  image_transport::CameraSubscriber sub_camera_;
  ros::Subscriber sub_camera_imu_;


};


Mat Erosion(const Mat& src);
Mat Dilation(const Mat& src);
Mat FilterColors(const Mat& src);
Mat getColor(cv::Mat &srcBGR);
#endif
