#include "imgproc.h"
#include <cstdio>
// Set dot characteristics for the auto detection

bool points_init = false;
//#define GUI
  vpDisplayOpenCV d;
  vpMeNurbs tracking_rect;
  vpMe me;
  vpDot tracker_rectangle[1];
// any pixel value lower than threshold1 becomes black.  I think threshold2 just needs to be > 1.  JT
  double pitch;
  bool semaphore_imu = true;
  void getCircle::imu_callback(const geometry_msgs::Vector3& imu_msg)
  {
	  while(semaphore_imu==false) usleep(10);
		  semaphore_imu=false;
		  pitch = imu_msg.x;
		  semaphore_imu = true;

  }

Patch imgproc(Mat &src, getCircle* obj_ptr)
{
  Mat blurred, thresholded, reduced, src_gray, countour_image;
  int threshold1 = obj_ptr->image_threshold;
  /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src, src_gray, Size(5, 5), 2, 2 );
    threshold(src_gray, thresholded, threshold1, 255, THRESH_TOZERO);//threshold the image
    vpImage<unsigned char> I;
    vpImageConvert::convert(thresholded, I);

     //tracker_rectangle[0].setComputeMoments(1);
     d.init(I, 0, 0, "") ;
     vpDisplay::display(I);
     if (points_init == false){
   	  d.init(I, 0, 0, "") ;

   	  vpDisplay::display(I);
   	  std::list<vpDot2> list_d;//list of elements in constrast respect ot the background
   	  vpDot2 dot_search;

   	  dot_search.setGraphics(true);

   	  //dot_search.setSurface(obj_ptr->Surface);
   	  dot_search.setGrayLevelMin(obj_ptr->GrayLevelMin);
   	  dot_search.setGrayLevelMax(obj_ptr->GrayLevelMax);
   	  dot_search.setGrayLevelPrecision(obj_ptr->opt_grayLevelPrecision);
   	  //dot_search.setSizePrecision(obj_ptr->opt_sizePrecision);
   	  dot_search.setEllipsoidShapePrecision(obj_ptr->opt_ellipsoidShapePrecision);
   	  try
   	   {
   		  while(list_d.size() == 0){
   		  dot_search.searchDotsInArea(I, obj_ptr->width_min,  obj_ptr->height_min,  obj_ptr->width_max, obj_ptr->height_max, list_d) ;
   		  cout<<"searching the first dot"<<endl;
   		  }
   		  //dot_search.searchDotsInArea(I, list_d);
   	   }
   	  catch (int e)
   	   {
   	     cout << "An exception occurred. Exception Nr. " << e << endl;
   	   }
   	  vpImagePoint init_point;
   	  if(list_d.size()!=0){
   	  init_point.set_i(list_d.front().getCog().get_i());
   	  init_point.set_j(list_d.front().getCog().get_j());
   	  tracker_rectangle[0].initTracking(I, init_point);
   	  }
   	  else{
   		  cout<<"no regions have been found select it on the image"<<endl;
   	  }
   	 /* try
   	   {
           tracker_rectangle[0].track(I);
   	   }
     	  catch (const std::exception &e)
     	   {
     		  cout<<" tracking failed start to send vicon message recovery"<<endl;
     		  while(ros::ok()){
     			  geometry_msgs::Vector3Stamped pos;
     			   pos.vector.z = ros::Time::now().toSec();
     			   pos.vector.x = 1000; // x1 point start (highest point)
     			   pos.vector.y = 1000; // x2 point (lowest point)
     			   obj_ptr->cylinder_pos_pub_.publish(pos);
     		  }
     	 }
   	  tracker_rectangle[0].display(I, vpColor::red);
*/
   	  points_init = true;

     }
   else{
   	d.init(I, 0, 0, "") ;
   	vpDisplay::display(I);
   	  try
   	   {
           tracker_rectangle[0].track(I);
   	   }
     	  catch (const std::exception &e)
     	   {
     		  cout<<" tracking failed start to send vicon message recovery"<<endl;
     		  while(ros::ok()){
     			  geometry_msgs::Vector3Stamped pos;
     			   pos.vector.z = ros::Time::now().toSec();
     			   pos.vector.x = 1000; // x1 point start (highest point)
     			   pos.vector.y = 1000; // x2 point (lowest point)
     			   obj_ptr->cylinder_pos_pub_.publish(pos);
     		  }
     	   }
       tracker_rectangle[0].display(I, vpColor::green);
   }
     vpDisplay::flush(I);
#ifdef GUI
	//imshow("Original image", src);
	//imshow("Blurred", blurred);
	namedWindow( "Hough Circle Transform", CV_WINDOW_AUTOSIZE );
	imshow( "Hough Circle Transform", src_gray );
	namedWindow( "Threshold", CV_WINDOW_AUTOSIZE );
	imshow("Threshold", thresholded);
	namedWindow( "drawing", CV_WINDOW_AUTOSIZE );
	imshow("drawing", drawing);
	cv::waitKey(1);
#endif
_Patch patch;
  return patch;
}
