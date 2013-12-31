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


    //Apply the Hough Transform to find the circles
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

       double secs = ros::Time::now().toSec();

    Mat contour_img;
    contour_img = getColor_from_img.clone();//copy the image
    findContours( contour_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    //ellipse fitting problem
    vector<RotatedRect> minEllipse( contours.size() );
    vector<RotatedRect> minRect( contours.size() );
    for( int i = 0; i < contours.size() ; i++ )
       {
         if( contours[i].size() > 5 )
           {
        	 minEllipse[i] = fitEllipse( Mat(contours[i]) );//give the ellipse fitting points
        	 if(minEllipse[i].size.width  < 40){
				minEllipse.erase(minEllipse.begin() + i);
				contours.erase(contours.begin() + i);
        	 }
           }
       }

#ifdef RANSAC_ellipse
    //Ransac adaptive version
    int ep_num = 5;//minimum point number to estimate the ellipse
    vector<int> inliers_index;
    vector<int> max_inliers_index;
    int ninliers = 0;
    int max_inliers = 0;
    int sample_num = 4400;        //number of sample
    int ransac_count = 0;
    double mean = 0;
    double sigma = 0;
    double dis_scale;
    double dis_threshold;
    bool first_cycle = true;

    vector<double> dis_error;
    double ellipse_par[5] = {0};
	 int rand_index[5] = {0};

		for(int i = 0; i< contours[0].size(); i++)
		mean = mean + sqrt(pow(contours[0][i].x,2) + pow(contours[0][i].y,2) );
		mean = mean/contours[0].size();
		vector <double> sqrt_variance;
		sqrt_variance.clear();
		double standard_deviation = 0;
		for(int i = 0; i< contours[0].size(); i++){
			sqrt_variance.resize(i+1);
			sqrt_variance[i] =  (sqrt(pow(contours[0][i].x,2) + pow(contours[0][i].y,2) ) - mean)*(sqrt(pow(contours[0][i].x,2) + pow(contours[0][i].y,2) ) - mean);
	         sigma = sigma + sqrt_variance[i];
		}

		sigma = sqrt(sigma/contours[0].size());
		cout<<"sigma:"<<sigma<<endl;
        dis_threshold = sqrt(3.84)*sigma;
        cout<<"dis_threshold:"<<dis_threshold<<endl;



//main cycle
    while (sample_num > ransac_count) {
      get_5_random_num(contours[0].size(), rand_index);
      vector<Point> contours_ransac;
      for (int j = 0; j < ep_num; j++){
    	  contours_ransac.resize(j+1);
    	  contours_ransac[j] = contours[0][rand_index[j]];//copy random points
      }
 	 minEllipse[0] = fitEllipse( Mat(contours_ransac) );//give the ellipse fitting points


 	double f = sqrt(fmax(minEllipse[0].size.width/2*minEllipse[0].size.width/2, minEllipse[0].size.height/2*minEllipse[0].size.height/2)-fmin(minEllipse[0].size.width/2*minEllipse[0].size.width/2, minEllipse[0].size.height/2*minEllipse[0].size.height/2));//focus norm

 	//compute the axis of the major axis
 	double m = tan(minEllipse[0].angle*pi/180);
 	double b = minEllipse[0].center.y - m*minEllipse[0].center.x;
 	Point2f c0(minEllipse[0].center.x, minEllipse[0].center.y);
 	Point2f c1((c0.x + 2),(c0.x + 2)*m + b);
 	Point2f diff_c_norm = (c1 - c0);

 	Point2f F1 = diff_c_norm*f + minEllipse[0].center;//estimate focus of the ellipses
 	Point2f F2 = -diff_c_norm*f + minEllipse[0].center;

 	ninliers = 0;
 	bool new_index;
 	int dis_error_counter = 0;
 	dis_error.clear();
    //check how many points are in the consensus set out of the possible inliers
    for (int i = 0; i < contours[0].size(); i++) {
    	new_index = true;
    	for (int j = 0; j < contours_ransac.size(); j++)
    		if(contours_ransac[j] == contours[0][i]){
    			new_index =  false;
    			break;
    		}
    	if (new_index){
      dis_error.resize(dis_error_counter + 1);
      dis_error[dis_error_counter] = sqrt(pow(F1.x - contours[0][i].x,2) + pow(F1.y - contours[0][i].y,2)) + sqrt(pow(F2.x - contours[0][i].x,2) + pow(F2.y - contours[0][i].y,2)) - minEllipse[0].size.width; //compute error for each point
      dis_error_counter++;
    	if (fabs(dis_error[i]) < dis_threshold) {
    	  inliers_index.resize(ninliers + 1);
        inliers_index[ninliers] = i;
        ninliers++;
    			}
    		}
    	}

    if (ninliers > max_inliers) {
    	max_inliers_index.resize(inliers_index.size());
    	for(int i = 0; i<inliers_index.size(); i++)
    		max_inliers_index[i] = inliers_index[i];
            max_inliers = ninliers;
            sample_num = (int)(log((double)(1-0.99))/log(1.0-pow(ninliers*1.0/contours[0].size(), 5)))*10;
          }
    ransac_count++;//increment ransac counter iterations

    if (ransac_count > 1500) {
      printf("Error! ransac_count exceed! ransac break! sample_num=%d, ransac_count=%d\n", sample_num, ransac_count);
      break;
    }
    }

 //estimate again the model based on all the inliers and original inlier set
    vector<Point> contours_opt;
    for (int j = 0; j < max_inliers_index.size(); j++){
    	contours_opt.resize(j+1);
    	contours_opt[j] = contours[0][max_inliers_index[j]];//copy random points
    }
    for(int j = 0; j < 5; j++){
    	contours_opt.resize(max_inliers_index.size() + j +1);
    	contours_opt[max_inliers_index.size() + j] = contours[0][rand_index[j]];;//copy random points
    }
    if(max_inliers_index.size()>=5){
    minEllipse[0] = fitEllipse( Mat(contours_opt) );//give the ellipse fitting points
    cout<<"inliers:"<<max_inliers + 5<<" "<<"of"<<" "<<contours[0].size()<<endl;

    }
    else{
    	minEllipse[0] = fitEllipse( Mat(contours[0]) );//give the ellipse fitting points
    	cout<<"no_inliers"<<endl;
    }
#endif
//compute the axis of the major axis
double m0 = tan(minEllipse[0].angle*pi/180);
double b0 = minEllipse[0].center.y - m0*minEllipse[0].center.x;
Point2f c00(minEllipse[0].center.x, minEllipse[0].center.y);
Point2f c01((c00.x + 2),(c00.x + 2)*m0 + b0);
Point2f diff_c_norm_w = (c01 - c00);
diff_c_norm_w.x = (diff_c_norm_w.x/sqrt((c01.x - c00.x)*(c01.x - c00.x) + (c01.y - c00.y)*(c01.y - c00.y)))*minEllipse[0].size.width/2;
diff_c_norm_w.y = (diff_c_norm_w.y/sqrt((c01.x - c00.x)*(c01.x - c00.x) + (c01.y - c00.y)*(c01.y - c00.y)))*minEllipse[0].size.width/2;

//compute the axis of the minor axis
double m1;
if(minEllipse[0].angle<=pi/2)
m1 = tan(pi/2 - minEllipse[0].angle*pi/180);
else
m1 = tan(minEllipse[0].angle*pi/180 - pi/2);

double b1 = minEllipse[0].center.y - m1*minEllipse[0].center.x;
Point2f c10(minEllipse[0].center.x, minEllipse[0].center.y);
Point2f c11((c10.x + 2),(c10.x + 2)*m1 + b1);
Point2f diff_c_norm_h = (c11 - c10);
diff_c_norm_h.x = (diff_c_norm_h.x/sqrt((c11.x - c10.x)*(c11.x - c10.x) + (c11.y - c10.y)*(c11.y - c10.y)))*minEllipse[0].size.height/2;
diff_c_norm_h.y = (diff_c_norm_h.y/sqrt((c11.x - c10.x)*(c11.x - c10.x) + (c11.y - c10.y)*(c11.y - c10.y)))*minEllipse[0].size.height/2;

//matrix of points
vector<Point2f> P1;
vector<Point2f> P2;
P1.resize(1);
P2.resize(1);
P1[0] = diff_c_norm_w + minEllipse[0].center;//upper point
P1[1] = -diff_c_norm_w + minEllipse[0].center;
P2[0] = diff_c_norm_h + minEllipse[0].center;//upper point
P2[1] = -diff_c_norm_h + minEllipse[0].center;
//cout<<"angle:"<<minEllipse[0].angle<<"number of ellipse detected:"<<minEllipse.size()<<endl;



// Draw contours + rect + ellipse
    int threshold_width_ellipse = 20;
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
         Point2f PM;
         Point2f Pm;
         //line( src, minEllipse[0].center, Point2f(P[0].x, P[0].y), color, 1, 8 );
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
         //line( src, Point2f(P[2].x, P[2].y), Point2f(P[3].x, P[3].y), color, 1, 8 );

       }

    //cout<<"difference:"<<(minEllipse[0].size.width*0.5f - norm(minEllipse[0].center - Point2f(P1[1].x, P1[1].y)))<<endl;
    //cout<<"Frequency [Hz]:"<<1/(ros::Time::now().toSec() - secs)<<endl;
     /*
     //method based on contour detection
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

*/
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
