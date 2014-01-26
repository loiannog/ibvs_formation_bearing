#include "imgproc.h"
#include <algorithm>
#include <cstdio>



/// Global variables
/**  @function Erosion  */
void getCircle::Erosion(const Mat& src)
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
  double secserosion = ros::Time::now().toSec();

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  /// Apply the erosion operation
  erode( src, src, element );


}

/** @function Dilation */
void getCircle::Dilation(const Mat& src)
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

  /// Apply the dilation operation
  dilate( src, src, element );
}

void getCircle::Moprh(const Mat& src)
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  
  morphologyEx( src, src, MORPH_OPEN, element );


}


void getCircle::getColor(cv::Mat &srchsv, cv::Mat &src, cv::Mat &contour_img, string color, vector<RotatedRect>& minEllipse, vector<double>* bearing)
{


    map<string, int> numbers;
    numbers["green"] = 1;
    numbers["violet"] = 2;
    numbers["orange"] = 3;
double secs_morph = ros::Time::now().toSec();

  switch (numbers[color])
  {

  case 1:
	  //inRange(srchsv, Scalar(28, 30, 30),
		//               Scalar(40, 240, 240), contour_img);//green values
	  inRange(srchsv, Scalar(color11_low, color12_low, color13_low),
		               Scalar(color11_high, color12_high, color13_high), contour_img);//green values
	  //inRange(srchsv, Scalar(28, 70, 38),
	//	               Scalar(40, 116, 76), contour_img);//green values
	  //green good Scalar(38, 30, 30), Scalar(50, 240, 240)//handheld
	//inRange(hsv, Scalar(30, 30, 30), Scalar(40, 240, 240), mask); green circle;
      break;
  case 2:

	 // inRange(srchsv, Scalar(8,40, 8),
		//                Scalar(20, 126, 86), contour_img);//violet values 8-17
	  inRange(srchsv, Scalar(color21_low, color22_low, color23_low),
		                Scalar(color21_high, color22_high, color23_high), contour_img);//violet values 8-17
	 //Scalar(0,20, 20), Scalar(10, 255, 255);//red good
     break;
  case 3:
	  inRange(srchsv, Scalar(color31_low, color32_low, color33_low),
		                Scalar(color31_high, color32_high, color33_high), contour_img);//brown values
      break;
	default:
		cout << "Invalid Selection. Please try Again." << endl;
  }

  //cv::imshow("color_extraction", contour_img);
   Moprh(contour_img);
    cout<<"morph time:"<<(ros::Time::now().toSec()-secs_morph)<<endl;
   //cv::imshow("after cleaning", contour_img);

double secs_gaussian = ros::Time::now().toSec();

   GaussianBlur(contour_img, contour_img, Size(3,3), 0, 0);//smooth the image
    //cout<<"gaussian time:"<<(ros::Time::now().toSec()-secs_gaussian)<<endl;

   //Contour definiton
   vector<vector<Point> > contours;
   vector<Vec4i> hierarchy;
double secs_find_contour = ros::Time::now().toSec();

   findContours( contour_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    //cout<<"find contour time:"<<(ros::Time::now().toSec()-secs_find_contour)<<endl;

   //ellipse fitting problem
   minEllipse.resize( contours.size() );
   for( int i = 0; i < contours.size() ; i++ )
      {
        if( contours[i].size() >= 5 )
          {
       	 minEllipse[i] = fitEllipse( Mat(contours[i]) );//give the ellipse fitting points
       	 //cout<<"width:"<<minEllipse[i].size.width<<endl;
       	 if(minEllipse[i].size.width  < 15){//inser threshold size
				minEllipse.erase(minEllipse.begin() + i);
				contours.erase(contours.begin() + i);
       	 }
          }
      }

   //matrix of points
   vector<Point2f> P1(2);
   vector<Point2f> P2(2);

   //RANSAC thread for each ellipse
   if(minEllipse.size()>0 && contours[0].size()>=5){
#ifdef RANSAC_ellipse
 	RANSAC_thread(contours[0], &minEllipse[0], &P1, &P2, RANSAC_iterations);
#endif
	ellipsePublisher(&src, &P1, &P2, &minEllipse[0], bearing);
   }



}

//this is not used for the moment
Mat getCircle::FilterColors(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    cv::Mat imR(src.rows, src.cols, CV_8UC1);
    cv::Mat imG(src.rows, src.cols, CV_8UC1);
    cv::Mat imB(src.rows, src.cols, CV_8UC1);

    Mat out[] = {imR, imG, imB};
    int from_to[] = {0, 2, 1, 1,  2, 0};
    cv::mixChannels(&src, 1, out, 3, from_to, 3);
    cv::bitwise_not(imR, imR);
    cv::bitwise_not(imB, imB);



    cv::multiply(imG, imR, imG, (double)1/255);
    cv::multiply(imG, imB, imG, (double)1/255);
    //Erosion(imG);//filtering
    //Dilation(imG);//filtering
    return imG;
}

// Randomly select 5 indeics
void get_5_random_num(int max_num, int* rand_num)
{
  int rand_index = 0;
  int r;
  int i;
  bool is_new = 1;

  if (max_num == 4) {
    for (i = 0; i < 5; i++) {
      rand_num[i] = i;
    }
    return;
  }

  while (rand_index < 5) {
    is_new = 1;
    r = (int)((rand()*1.0/RAND_MAX) * max_num);
    for (i = 0; i < rand_index; i++) {
      if (r == rand_num[i]) {
        is_new = 0;
        break;
      }
    }
    if (is_new) {
      rand_num[rand_index] = r;
      rand_index++;
    }
  }
}
void getCircle::ellipsePublisher(Mat* src, vector<Point2f>* P1, vector<Point2f>* P2, RotatedRect* minEllipse, vector<double>* bearing){
    Point2f PM1;//major axis points
    Point2f PM2;//major axis points
    Point2f Pm1;//minor axis points
    Point2f Pm2;//minor axis points
    if((sqrt(pow(P1->at(0).x - minEllipse[0].center.x,2)) + sqrt(pow(P1->at(1).y - minEllipse[0].center.y,2))) >= (sqrt(pow(P2->at(1).x - minEllipse[0].center.x,2)) + sqrt(pow(P2->at(1).y - minEllipse[0].center.y,2)))){
    PM1 = Point2f(P1->at(0).x, P1->at(0).y);
    Pm1 = Point2f(P2->at(0).x, P2->at(0).y);
    PM2 = Point2f(P1->at(1).x, P1->at(1).y);
    Pm2 = Point2f(P2->at(1).x, P2->at(1).y);
    }
    else{
    PM1 = Point2f(P2->at(0).x, P2->at(0).y);
    Pm1 = Point2f(P1->at(0).x, P1->at(0).y);
    PM2 = Point2f(P2->at(1).x, P2->at(1).y);
    Pm2 = Point2f(P1->at(1).x, P1->at(1).y);
    }

    //compensate distortion and change coordinates
    vector<Point2f> P;
    vector<Point2f> dst_P;
    P.resize(3);
    dst_P.resize(3);
    P[0] = minEllipse[0].center;
    P[1] = PM1;
    P[2] = PM2;
    const cv:: Mat cM = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    const cv:: Mat Dl = (cv::Mat_<double>(4,1) << d0, d1, d2, d3);
    undistortPoints(P, dst_P, cM, Dl);
    Point2f diff_dstP = dst_P[2] - dst_P[1];
    double norm_dstP = sqrt(pow(diff_dstP.x,2) + pow(diff_dstP.y,2));
    double ellipse_direction_scale = norm_dstP/cylinder_size;
    bearing->at(0) = dst_P[0].x/sqrt(pow(dst_P[0].x,2) + pow(dst_P[0].y,2) + 1)/ellipse_direction_scale;
    bearing->at(1) = dst_P[0].y/sqrt(pow(dst_P[0].x,2) + pow(dst_P[0].y,2) + 1)/ellipse_direction_scale;
    bearing->at(2) = 1/sqrt(pow(dst_P[0].x,2) + pow(dst_P[0].y,2) + 1)/ellipse_direction_scale;

    //cout<<"position_z:"<<ellipse_direction.vector<<endl;

    //Show your results
      // Draw contours + rect + ellipse
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           Scalar color_max = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           Scalar color_min = Scalar( rng.uniform(0, 100), rng.uniform(0,100), rng.uniform(0,255) );

           //ellipse( src, minEllipse[i], color, 2, 8 );//draw ellipse
           ellipse(*src, minEllipse[0].center, minEllipse[0].size*0.5f, minEllipse[0].angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
           Point2f rect_points[4]; minEllipse[0].points( rect_points );
           for( int j = 0; j < 4; j++ )
              line( *src, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );



       	line( *src, minEllipse[0].center, PM1, color_max, 1, 8 );
       	line( *src, minEllipse[0].center, Pm1, color_min, 1, 8 );
       }



//class RANSAC thread functions
void getCircle::RANSAC_thread(vector<Point> contours, RotatedRect* minEllipse, vector<Point2f>* P1, vector<Point2f>* P2, int sample_num)
{


		//Ransac adaptive version just defined in the first ellipse
		    int ep_num = 5;//minimum point number to estimate the ellipse
		    vector<int> inliers_index;
		    vector<int> max_inliers_index;
		    int ninliers = 0;
		    int max_inliers = 0;
		    //int sample_num = 4400;        //number of sample
		    int ransac_count = 0;
		    double mean = 0;
		    double sigma = 0;
		    double dis_scale;
		    double dis_threshold;

		    vector<double> dis_error;
		    double ellipse_par[5] = {0};
			 int rand_index[5] = {0};

				for(int i = 0; i< contours.size(); i++)
				mean = mean + sqrt(pow(contours[i].x,2) + pow(contours[i].y,2) );
				mean = mean/contours.size();
				vector <double> sqrt_variance;
				sqrt_variance.clear();
				double standard_deviation = 0;
				for(int i = 0; i< contours.size(); i++){
					sqrt_variance.resize(i+1);
					sqrt_variance[i] =  (sqrt(pow(contours[i].x,2) + pow(contours[i].y,2) ) - mean)*(sqrt(pow(contours[i].x,2) + pow(contours[i].y,2) ) - mean);
			         sigma = sigma + sqrt_variance[i];
				}

				sigma = sqrt(sigma/contours.size());
		        dis_threshold = sqrt(3.84)*sigma;



		    //main cycle
		    while (sample_num > ransac_count) {
		      get_5_random_num(contours.size(), rand_index);
		      vector<Point> contours_ransac;
		      for (int j = 0; j < ep_num; j++){
		    	  contours_ransac.resize(j+1);
		    	  contours_ransac[j] = contours[rand_index[j]];//copy random points
		      }
		 	 *minEllipse = fitEllipse( Mat(contours_ransac) );//give the ellipse fitting points


		 	double f = sqrt(fmax(minEllipse->size.width/2*minEllipse->size.width/2, minEllipse->size.height/2*minEllipse->size.height/2)-fmin(minEllipse->size.width/2*minEllipse->size.width/2, minEllipse->size.height/2*minEllipse->size.height/2));//focus norm

		 	//compute the axis of the major axis
		 	double m = tan(minEllipse->angle*pi/180);
		 	double b = minEllipse->center.y - m*minEllipse->center.x;
		 	Point2f c0(minEllipse->center.x, minEllipse->center.y);
		 	Point2f c1((c0.x + 2),(c0.x + 2)*m + b);
		 	double norm_c = sqrt((c1.x - c0.x)*(c1.x - c0.x) + (c1.y - c0.y)*(c1.y - c0.y));
		 	Point2f diff_c_norm = (c1 - c0);
		 	diff_c_norm.x = diff_c_norm.x/norm_c;
		 	diff_c_norm.y = diff_c_norm.y/norm_c;


		 	Point2f F1 = diff_c_norm*f + minEllipse->center;//estimate focus of the ellipses
		 	Point2f F2 = -diff_c_norm*f + minEllipse->center;

		 	ninliers = 0;
		 	bool new_index;
		 	int dis_error_counter = 0;
		 	dis_error.clear();
		    //check how many points are in the consensus set out of the possible inliers
		    for (int i = 0; i < contours.size(); i++) {
		    	new_index = true;
		    	for (int j = 0; j < contours_ransac.size(); j++)
		    		if(contours_ransac[j] == contours[i]){
		    			new_index =  false;
		    			break;
		    		}
		    	if (new_index){
		      dis_error.resize(dis_error_counter + 1);
		      dis_error[dis_error_counter] = sqrt(pow(F1.x - contours[i].x,2) + pow(F1.y - contours[i].y,2)) + sqrt(pow(F2.x - contours[i].x,2) + pow(F2.y - contours[i].y,2)) - minEllipse->size.width; //compute error for each point
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
		            sample_num = (int)(log((double)(1-0.99))/log(1.0-pow(ninliers*1.0/contours.size(), 5)))*10;
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
		    	contours_opt[j] = contours[max_inliers_index[j]];//copy random points
		    }
		    for(int j = 0; j < 5; j++){
		    	contours_opt.resize(max_inliers_index.size() + j +1);
		    	contours_opt[max_inliers_index.size() + j] = contours[rand_index[j]];;//copy random points
		    }

		    *minEllipse = fitEllipse( Mat(contours_opt) );//give the ellipse fitting points


		    //compute the axis of the major axis
		    double m0 = tan(minEllipse->angle*pi/180);
		    double b0 = minEllipse->center.y - m0*minEllipse->center.x;
		    Point2f c00(minEllipse->center.x, minEllipse->center.y);
		    Point2f c01((c00.x + 2),(c00.x + 2)*m0 + b0);
		    Point2f diff_c_norm_w = (c01 - c00);
		    diff_c_norm_w.x = (diff_c_norm_w.x/sqrt((c01.x - c00.x)*(c01.x - c00.x) + (c01.y - c00.y)*(c01.y - c00.y)))*minEllipse->size.width/2;
		    diff_c_norm_w.y = (diff_c_norm_w.y/sqrt((c01.x - c00.x)*(c01.x - c00.x) + (c01.y - c00.y)*(c01.y - c00.y)))*minEllipse->size.width/2;

		    //compute the axis of the minor axis
		    double m1;
		    if(minEllipse->angle<=pi/2)
		    m1 = tan(pi/2 - minEllipse->angle*pi/180);
		    else
		    m1 = tan(minEllipse->angle*pi/180 - pi/2);

		    double b1 = minEllipse->center.y - m1*minEllipse->center.x;
		    Point2f c10(minEllipse->center.x, minEllipse->center.y);
		    Point2f c11((c10.x + 2),(c10.x + 2)*m1 + b1);
		    Point2f diff_c_norm_h = (c11 - c10);
		    diff_c_norm_h.x = (diff_c_norm_h.x/sqrt((c11.x - c10.x)*(c11.x - c10.x) + (c11.y - c10.y)*(c11.y - c10.y)))*minEllipse->size.height/2;
		    diff_c_norm_h.y = (diff_c_norm_h.y/sqrt((c11.x - c10.x)*(c11.x - c10.x) + (c11.y - c10.y)*(c11.y - c10.y)))*minEllipse->size.height/2;

		    //matrix of points
		    P1->at(0) = diff_c_norm_w + minEllipse->center;//upper point
		    P1->at(1) = -diff_c_norm_w + minEllipse->center;
		    P2->at(0) = diff_c_norm_h + minEllipse->center;//upper point
		    P2->at(1) = -diff_c_norm_h + minEllipse->center;








	}

