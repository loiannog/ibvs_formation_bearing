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
#define show_images
// Set dot characteristics for the auto detection

using namespace std;

void getCircle::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  std::string path_file;
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
/// Global variables
Mat erosion_dst, dilation_dst;
int dilation_elem = 2;
int erosion_elem = 2;
int erosion_size = 1;
int dilation_size = 1;
int const max_elem = 2;
/**  @function Erosion  */
Mat Erosion(const Mat& src)
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( src, erosion_dst, element );
  return erosion_dst;
  //imshow( "Erosion Demo", erosion_dst );
}

/** @function Dilation */
Mat Dilation(const Mat& src)
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( src, dilation_dst, element );
  return dilation_dst;

}

Mat FilterColors(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    cv::Mat imR(src.rows, src.cols, CV_8UC1);
    cv::Mat imG(src.rows, src.cols, CV_8UC1);
    cv::Mat imB(src.rows, src.cols, CV_8UC1);

    Mat out[] = {imR, imG, imB};
    int from_to[] = {0, 2, 1, 1,  2, 0};
    cv::mixChannels(&src, 1, out, 3, from_to, 3);
    cv::bitwise_not(imG, imG);
    cv::bitwise_not(imB, imB);



    //cv::multiply(imR, imG, imGboost, (double)1/255);
    //cv::multiply(imGboost, imB, imGboost, (double)1/255);
    //Erosion(imG);//filtering
    //Dilation(imG);//filtering
    return imG;
}

Mat getColor(cv::Mat &srcBGR)
{
  cv::Mat mask(srcBGR.rows, srcBGR.cols, CV_8UC1);
  cv::Mat hsv(srcBGR.rows, srcBGR.cols, CV_8UC3);
  cvtColor(srcBGR, hsv, CV_BGR2HSV);
  inRange(hsv, Scalar(0,10, 20),
	                Scalar(9, 255, 255), mask);

//red good Scalar(0,20, 20), Scalar(10, 255, 255)

  mask = Erosion(mask);
  mask = Dilation(mask);
  GaussianBlur(mask, mask, Size(3,3), 0);//smooth the image

  return mask;
}

void getCircle::camera_callback(const sensor_msgs::Image::ConstPtr &img)
//,
	//	const sensor_msgs::CameraInfo::ConstPtr &c)
{

  static bool initialized = false;
  static ros::Time initial_timestamp;
  if(!initialized)
  {
    initial_timestamp = img->header.stamp;
    initialized = true;
  }

  vector<Mat> layers;
  SimpleBlobDetector::Params params;
  //params.filterByArea = false;
  params.filterByCircularity = true;
  params.filterByConvexity = false;
  params.filterByInertia = false;
  params.filterByColor = true;
  params.blobColor = 255;
  params.maxCircularity = 10;
  params.maxCircularity = 100;
  //params.minArea = 100.0f;
  //params.maxArea = 500.0f;
  SimpleBlobDetector myBlobDetector(params);

  std::vector<cv::KeyPoint> myBlobs;
  cv::Mat src(cv::Size(img->width, img->height), CV_8UC3,
              const_cast<uchar*>(&img->data[0]), img->step);//3 channels image

   //Patch patch = imgproc(mvbegin, this);
    Mat imgThresh;
    Mat getColor_from_img;
    GaussianBlur(src, src, Size(5,5), 0);//smooth the image
    getColor_from_img = getColor(src);//get the color red


    //Apply the Hough Transform to find the circles
    vector<Vec3f> circles;
    //parameters for the Hough transform
    int dp = 2;
    int params1 = 200;
    int params2 = 40;
    int minRadius = 5;
    int maxRadius = 0;
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    HoughCircles( getColor_from_img, circles, CV_HOUGH_GRADIENT, dp, getColor_from_img.rows/4, params1, params2, minRadius, maxRadius );


    cout<<"number of circles:"<<circles.size()<<endl;
    if(circles.size()>0){
    cout<<"radius:"<<circles[0][2]<<endl;
    cout<<"center::"<<circles[0][0]<<" "<<circles[0][1]<<endl;
    }

    //Draw the circles detected
     for( size_t i = 0; i < circles.size(); i++ )
     {
    	 //circle drawing parameters
    	 int thickness = -1;
    	 int lineType = 8;
    	 int shift = 0;
         Point center(circles[i][0], circles[i][1]);//center of the circle
         double radius = circles[i][2];//radius of the circle
         // circle center
         circle( src, center, 3, Scalar(255, 0, 0), thickness, lineType, shift );
         // circle outline
         circle( src, center, radius, Scalar(255, 100, 50), thickness+4, lineType, shift );
      }


     Mat contour_img;
     contour_img = getColor_from_img.clone();//copy the image

     findContours( contour_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


     int thresh = 100;
     int max_thresh = 255;
     RNG rng(12345);

     /// Approximate contours to polygons + get bounding rects and circles
     vector<vector<Point> > contours_poly( contours.size() );
     vector<Rect> boundRect( contours.size() );
     vector<Point2f>center_c( contours.size() );
     vector<float>radius_c( contours.size() );


     for( int i = 0; i < contours.size(); i++ )
           { approxPolyDP( Mat(contours[i]), contours_poly[i], 0.1, true );
             //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
             //minEnclosingCircle( (Mat)contours_poly[i], center_c[i], radius_c[i] );
           }
     /// Draw polygonal contour + bonding rects + circles
     Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
     for( int i = 0; i< contours.size(); i++ )
        {
          Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
          //circle( drawing, center_c[i], (int)radius_c[i], color, 2, 8, 0 );
        }


     //Show your results
     #ifdef show_images
     namedWindow( "Color Extraction", CV_WINDOW_AUTOSIZE );
     cv::imshow("Color Extraction", getColor_from_img);
     namedWindow( "Hough Circle", CV_WINDOW_AUTOSIZE );
     imshow( "Hough Circle", src );
     //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
     //cv::imshow("Contours", contour_img);
     cv::waitKey(0);
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
