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
#include <iostream>
#include <opencv2/nonfree/features2d.hpp>
#define show_images
// Set dot characteristics for the auto detection

using namespace std;

void getCircle::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  string path_file;
  //priv_nh.param<string>("path_file", path_file, "/home/prisma-airobots/AIRobots_Unina_workspace/AIRobots_UNINA/vision_perching/");
    priv_nh.param<int>("image_threshold", image_threshold, 240);//Surface of a dot to search in an area.
    priv_nh.param<double>("opt_sizePrecision", opt_sizePrecision, 0.25);
	priv_nh.param<double>("opt_grayLevelPrecision", opt_grayLevelPrecision, 0.55);
	priv_nh.param<double>("opt_ellipsoidShapePrecision", opt_ellipsoidShapePrecision, 0.3);
	priv_nh.param<int>("height_min", height_min, 100);//Coordinate (row) of the upper-left area corner.
	priv_nh.param<int>("height_max", height_max, 200);// Height or the area in which a dot is searched.
	priv_nh.param<int>("width_min", width_min, 200);//Coordinate (column) of the upper-left area corner.
	priv_nh.param<int>("width_max", width_max,300);//Width or the area in which a dot is searched.
	priv_nh.param<int>("GrayLevelMin", GrayLevelMin, 0);//GrayLevel min.
	priv_nh.param<int>("GrayLevelMax", GrayLevelMax, 250);//GrayLevel max.
	priv_nh.param<int>("Surface", Surface, 124);//Surface of a dot to search in an area.
	priv_nh.param<double>("fx", fx, 621.755015);//Surface of a dot to search in an area.
	priv_nh.param<double>("fy", fy, 617.402184);//Surface of a dot to search in an area.
	priv_nh.param<double>("cx", cx, 395.913754);//Surface of a dot to search in an area.
	priv_nh.param<double>("cy", cy, 60);//Surface of a dot to search in an area.
	priv_nh.param<double>("d0", d0, -0.406827);//Surface of a dot to search in an area.
	priv_nh.param<double>("d1", d1,  0.173936);//Surface of a dot to search in an area.
	priv_nh.param<double>("d2", d2, -6.1e-05);//Surface of a dot to search in an area.
	priv_nh.param<double>("d3", d3, -0.002139);//Surface of a dot to search in an area.
    image_transport::ImageTransport it(priv_nh);
	cout<<image_threshold<<endl;
	//cvNamedWindow("thresholded");
	/*cvNamedWindow("Original image");

	cvNamedWindow("Reduced");
	cvNamedWindow("Blurred");
	cvStartWindowThread();*/
  //sub_camera_ = it.subscribeCamera("image", 2, &getCylinderPosition::camera_callback,
    //                               this);
  ros::Subscriber sub = priv_nh.subscribe("image", 1,  &getCircle::camera_callback, this);


  image_thresholded_pub_ = it.advertise("/image_thresholded",1);
  cylinder_pos_pub_ = priv_nh.advertise<geometry_msgs::Vector3Stamped>("/cylinder_position",
                                                                   5);

  ros::spin();
}


void getCircle::camera_callback(const sensor_msgs::Image::ConstPtr &img)
//,
	//	const sensor_msgs::CameraInfo::ConstPtr &c)
{
	RNG rng(12345);
  static bool initialized = false;
  static ros::Time initial_timestamp;
  if(!initialized)
  {
    initial_timestamp = img->header.stamp;
    initialized = true;
  }

  vector<Mat> layers;

  std::vector<cv::KeyPoint> myBlobs;
  cv::Mat src(cv::Size(img->width, img->height), CV_8UC3,
              const_cast<uchar*>(&img->data[0]), img->step);//3 channels image

   //Patch patch = imgproc(mvbegin, this);
    Mat imgThresh;
    Mat getColor_from_img;
    GaussianBlur(src, src, Size(5,5), 0);//smooth the image
    getColor_from_img = getColor(src);//get the color red

    // Two methods for finding circle/ellipse
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

       double secs = ros::Time::now().toSec();

        Mat contour_img;
        contour_img = getColor_from_img.clone();//copy the image
        //second ellipse fitting
    findContours( contour_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<RotatedRect> minEllipse( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
       {
         if( contours[i].size() > 5 )
           {
        	      	 minEllipse[i] = fitEllipse( Mat(contours[i]) );//give the ellipse fitting points

           }
       }

    /// Draw contours + rotated rects + ellipses
    int threshold_width_ellipse = 20;
    RotatedRect tempEllip;
    float ellipse_major, ellipse_minor, ellipse_angle;
    Point2f ellipse_center, ellipse_top, ellipse_side;
    for( int i = 0; i< contours.size() && minEllipse[i].size.width>threshold_width_ellipse; i++ )
       {
    	tempEllip = minEllipse[i];
         Scalar color = Scalar(0,0,255);
         ellipse( src, tempEllip, color, 2, 8 );//draw ellipse

         // Extract ellipse information
         ellipse_major = tempEllip.size.height/2;
         ellipse_minor = tempEllip.size.width/2;
         ellipse_angle = tempEllip.angle;
         ellipse_center = tempEllip.center;
         // Draw axes
         ellipse_top.x = -ellipse_major * sin(ellipse_angle) + ellipse_center.x;
         ellipse_top.y = ellipse_major * cos(ellipse_angle) + ellipse_center.y;
         ellipse_side.x = ellipse_minor * cos(ellipse_angle) + ellipse_center.x;
         ellipse_side.y = ellipse_minor * sin(ellipse_angle) + ellipse_center.y;
         line(src, ellipse_center, ellipse_side, Scalar(255,255,255),2);
         line(src, ellipse_center, ellipse_top, Scalar(0,0,0),2);

       }
    cout<<"Frequency [Hz]:"<<1/(ros::Time::now().toSec() - secs)<<endl;

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




/*
    myBlobDetector.detect(getColor_from_img, myBlobs);
    cout<<"number of detected blobs:"<<myBlobs.size()<<endl;
    cv::Mat blobImg;
    for( size_t i = 0; i < myBlobs.size(); i++ )
    {
   	 //circle drawing parameters
   	 int thickness = -1;
   	 int lineType = 8;
   	 int shift = 0;
        Point center_blob(cvRound(myBlobs[i].pt.x), cvRound(myBlobs[i].pt.y));
        //double radius_blob = myBlobs[i].size;
        // circle center
        circle( src, center_blob, 20, Scalar(240, 150, 50), thickness, lineType, shift );
     }
    //cv::drawKeypoints(src, myBlobs, blobImg);
    //cv::imshow("layer", imgThresh);
    cv::imshow("Blobs", src);
    cv::waitKey(1);*/
   //cout<<"Frequency [Hz]:"<<1/(ros::Time::now().toSec() - secs)<<endl;
  //geometry_msgs::Vector3Stamped::Ptr pos(new geometry_msgs::Vector3Stamped);


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(circle_detection, getCircle,
                        getCircle, nodelet::Nodelet);
