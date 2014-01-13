#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <imgproc.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <iostream>

// Set dot characteristics for the auto detection

using namespace std;




void getCircle::onInit(void)
{
  ros::NodeHandle priv_nh(getMTPrivateNodeHandle());
  std::string path_file;
  //priv_nh.param<string>("path_file", path_file, "/home/prisma-airobots/AIRobots_Unina_workspace/AIRobots_UNINA/vision_perching/");
	priv_nh.param<double>("fx", fx, 621.755015);//Surface of a dot to search in an area.
	priv_nh.param<double>("fy", fy, 617.402184);//Surface of a dot to search in an area.
	priv_nh.param<double>("cx", cx, 395.913754);//Surface of a dot to search in an area.
	priv_nh.param<double>("cy", cy, 60);//Surface of a dot to search in an area.
	priv_nh.param<double>("d0", d0, -0.406827);//Surface of a dot to search in an area.
	priv_nh.param<double>("d1", d1,  0.173936);//Surface of a dot to search in an area.
	priv_nh.param<double>("d2", d2, -6.1e-05);//Surface of a dot to search in an area.
	priv_nh.param<double>("d3", d3, -0.002139);//Surface of a dot to search in an area.
	priv_nh.param<int>("RANSAC_iterations", RANSAC_iterations, 4400);//RANSAC iterations
	priv_nh.param<int>("erosion_size", erosion_size, 3);//Surface of a dot to search in an area.
	priv_nh.param<int>("dilation_size", dilation_size, 3);//Surface of a dot to search in an area.
	priv_nh.param<int>("dilation_elem", dilation_elem, 0);//Surface of a dot to search in an area.
	priv_nh.param<int>("erosion_elem", erosion_elem, 0);//Surface of a dot to search in an area.
	priv_nh.param<string>("color1", color1, "green");//Surface of a dot to search in an area.
	priv_nh.param<string>("color2", color2, "red");//Surface of a dot to search in an area.

    image_transport::ImageTransport it(priv_nh);
    image_ellipse = it.advertise("/QuadrotorGolf/ellipse", 1);

    ellipse_pos_pub_ = priv_nh.advertise<ibvs_formation_bearing::bearing>("/bearings",
                                                                    5);
    ros::Subscriber sub = priv_nh.subscribe("image", 1,  &getCircle::camera_callback, this);

   ros::spin();
}

void getCircle::camera_callback(const sensor_msgs::Image::ConstPtr &img)
	//	const sensor_msgs::CameraInfo::ConstPtr &c)
{

  cv::Mat src(cv::Size(img->width, img->height), CV_8UC3,
              const_cast<uchar*>(&img->data[0]), img->step);//3 channels image
  

     //GaussianBlur(src, src, Size(5,5), 0);//smooth the image
    double secs = ros::Time::now().toSec();
    //convert the image to hsv
    cv::Mat hsv(src.rows, src.cols, CV_8UC1);
    cv::Mat hsv2(src.rows, src.cols, CV_8UC1);
    cv::Mat contour_img1(src.rows, src.cols, CV_8UC3);
    cv::Mat contour_img2(src.rows, src.cols, CV_8UC3);

    cvtColor(src, hsv, CV_BGR2HSV);

    //getColor(src, getColor_from_img);//get the color red
    vector<RotatedRect> minEllipse_color1;
    vector<RotatedRect> minEllipse_color2;
    boost::thread thread_getColor_1(&getCircle::getColor, this, hsv, src, contour_img1, "violet", minEllipse_color1);
    //boost::thread thread_getColor_2(&getCircle::getColor, this, hsv, src, contour_img2, "green", minEllipse_color2);
    thread_getColor_1.join();
    //thread_getColor_2.join();
    //publish bearings
    ibvs_formation_bearing::bearing ellipses;
    ellipses.bearings.push_back(ellipse_direction);
    ellipse_pos_pub_.publish(ellipses);
    //publish the image
    cv_bridge::CvImage cv_ptr;
    cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
    cv_ptr.image = src.clone();
    image_ellipse.publish(cv_ptr.toImageMsg());
    cout<<"total time:"<<1/(ros::Time::now().toSec()-secs)<<endl;

 #ifdef show_images
     //namedWindow( "Color Extraction", CV_WINDOW_AUTOSIZE );
     //cv::imshow("Color Extraction", getColor_from_img);
     namedWindow( "Ellipse Fitting", CV_WINDOW_AUTOSIZE );
     imshow( "Ellipse Fitting", src );
     namedWindow( "Contours1", CV_WINDOW_AUTOSIZE );
     cv::imshow("Contours1", contour_img1);
    // namedWindow( "Contours2", CV_WINDOW_AUTOSIZE );
    // cv::imshow("Contours2", contour_img2);
     cv::waitKey(1);
    #endif


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(circle_detection, getCircle,
                        getCircle, nodelet::Nodelet);
