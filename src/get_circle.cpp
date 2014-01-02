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

#define show_images
#define RANSAC_ellipse
// Set dot characteristics for the auto detection

using namespace std;




void getCircle::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
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


    image_transport::ImageTransport it(priv_nh);

    ellipse_pos_pub_ = priv_nh.advertise<ibvs_formation_bearing::bearing>("/bearing",
                                                                    5);
    ros::Subscriber sub = priv_nh.subscribe("image", 1,  &getCircle::camera_callback, this);


   image_thresholded_pub_ = it.advertise("/image_thresholded",1);


   ros::spin();
}


void getCircle::camera_callback(const sensor_msgs::Image::ConstPtr &img)
	//	const sensor_msgs::CameraInfo::ConstPtr &c)
{
	RNG rng;

  std::vector<cv::KeyPoint> myBlobs;
  cv::Mat src(cv::Size(img->width, img->height), CV_8UC3,
              const_cast<uchar*>(&img->data[0]), img->step);//3 channels image

    Mat getColor_from_img;
    GaussianBlur(src, src, Size(5,5), 0);//smooth the image
    getColor_from_img = getColor(src);//get the color red


    //Contour definiton
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    double secs = ros::Time::now().toSec();

    Mat contour_img;
    contour_img = getColor_from_img.clone();//copy the image
    findContours( contour_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    //ellipse fitting problem
    vector<RotatedRect> minEllipse( contours.size() );
    for( int i = 0; i < contours.size() ; i++ )
       {
         if( contours[i].size() > 5 )
           {
        	 minEllipse[i] = fitEllipse( Mat(contours[i]) );//give the ellipse fitting points
        	 cout<<"width:"<<minEllipse[i].size.width<<endl;
        	 if(minEllipse[i].size.width  < 0){
				minEllipse.erase(minEllipse.begin() + i);
				contours.erase(contours.begin() + i);
        	 }
           }
       }

    //matrix of points
    vector<Point2f> P1;
    vector<Point2f> P2;
    P1.resize(2);
    P2.resize(2);

    //RANSAC thread for each ellipse
    boost::thread thread_ellipse_detection(RANSAC_thread,contours[0], &minEllipse[0], &P1, &P2, this->RANSAC_iterations);
    thread_ellipse_detection.join();

    //compensate distortion and change coordinates
    vector<Point2f> P;
    vector<Point2f> dst_P;
    P.resize(1);
    dst_P.resize(1);
    P[0] = minEllipse[0].center;
    const cv:: Mat cM = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    const cv:: Mat Dl = (cv::Mat_<double>(4,1) << d0, d1, d2, d3);
    undistortPoints(P, dst_P, cM, Dl);
    ellipse_direction.x = dst_P[0].x;
    ellipse_direction.y = dst_P[0].y;
    ellipse_direction.z = 0;
    ellipse_direction.scale = 0;
    ellipse_pos_pub_.publish(ellipse_direction);
    // Draw contours + rect + ellipse
    for( int i = 0; i< 1; i++ )
       {
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         Scalar color_max = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         Scalar color_min = Scalar( rng.uniform(0, 100), rng.uniform(0,100), rng.uniform(0,255) );

         //ellipse( src, minEllipse[i], color, 2, 8 );//draw ellipse
         ellipse(src, minEllipse[i].center, minEllipse[i].size*0.5f, minEllipse[i].angle, 0, 360, Scalar(0,255,255), 1, CV_AA);

         Point2f rect_points[4]; minEllipse[i].points( rect_points );
         for( int j = 0; j < 4; j++ )
            line( src, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
         Point2f PM;//major axis points
         Point2f Pm;//minor axis points
         if((sqrt(pow(P1[1].x - minEllipse[0].center.x,2)) + sqrt(pow(P1[1].y - minEllipse[0].center.y,2))) >= (sqrt(pow(P2[1].x - minEllipse[0].center.x,2)) + sqrt(pow(P2[1].y - minEllipse[0].center.y,2)))){
         PM = Point2f(P1[1].x, P1[1].y);
         Pm = Point2f(P2[1].x, P2[1].y);
         }
         else{
         PM = Point2f(P2[1].x, P2[1].y);
         Pm = Point2f(P1[1].x, P1[1].y);
         }
         line( src, minEllipse[i].center, PM, color_max, 1, 8 );
         line( src, minEllipse[i].center, Pm, color_min, 1, 8 );
       }


     //Show your results
     #ifdef show_images
     namedWindow( "Color Extraction", CV_WINDOW_AUTOSIZE );
     cv::imshow("Color Extraction", getColor_from_img);
     namedWindow( "Ellipse Fitting", CV_WINDOW_AUTOSIZE );
     imshow( "Ellipse Fitting", src );
     namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
     cv::imshow("Contours", contour_img);
     cv::waitKey(1);
	 #endif

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(circle_detection, getCircle,
                        getCircle, nodelet::Nodelet);
