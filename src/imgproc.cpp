#include "imgproc.h"
#include <algorithm>
#include <cstdio>
#define PI 3.14159265
#define NUM_INF 999999.9
#define GUI
// Set dot characteristics for the auto detection



Patch imgproc(Mat &src, getCircle* obj_ptr)
{

  Mat blurred, thresholded, reduced, src_gray, countour_image;

  int threshold1 = obj_ptr->image_threshold;
  //GaussianBlur(src, blurred, Size(3,3), 0);//clean the image
  //threshold(src, thresholded, threshold1, 255, THRESH_BINARY_INV);//threshold the image
  //threshold(src, thresholded, threshold1, 255, THRESH_TOZERO);//threshold the image
  /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src, src_gray, Size(5, 5), 2, 2 );
    threshold(src_gray, thresholded, threshold1, 255, THRESH_TOZERO);//threshold the image
    //Canny(src_gray, thresholded, threshold1, threshold1, 3, false);

    /// Apply the Hough Transform to find the circles
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat thresholdedcont = thresholded.clone();
    findContours(thresholdedcont, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros( thresholdedcont.size(), CV_8UC3 );

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    vector<float>radius( contours.size() );
    vector<double> norm_ellipse_center;
    vector<double> angle_ellipse_center;
    RNG rng(12345);
 int j = 0;
    for( int i = 0; i < contours.size(); i++ )
       {
    	 //approxPolyDP( Mat(contours[i]), contours_poly[i], 0.1, true );//input vector of Mat
         //boundRect[i] = boundingRect( Mat(contours_poly[i]) );//find boundaries for each contour
         //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
         if( contours[i].size() > 4 )
            {
        	 //find ellipse
        	 minEllipse[j] = fitEllipse( Mat(contours[i]));
            //compute the center norm
            norm_ellipse_center.push_back(sqrt(minEllipse[j].center.x*minEllipse[j].center.x+minEllipse[j].center.y*minEllipse[j].center.y));
            //compute the angle respect to the center
            angle_ellipse_center.push_back(atan2 (minEllipse[j].center.y, minEllipse[j].center.x));
            j++;//increase the ellispe contour size
       }
   }

    //compute for each ellipse the distance to the others
    vector<vector<double> > mat_distance(minEllipse.size());
    vector<vector<double> > mat_angle(minEllipse.size());
    for(int i = 0; i<minEllipse.size(); i++)
    	for(int j = 0; j<minEllipse.size(); j++){
    		mat_distance[i].push_back(fabs(norm_ellipse_center[i]-norm_ellipse_center[j]));
    		mat_angle[i].push_back(fabs(angle_ellipse_center[i]-angle_ellipse_center[j]));
    	 }

     //find the minimum distance for each row and store the index
    vector<double> min_distance(minEllipse.size());
    vector<double> min_angle(minEllipse.size());
    vector<int> index_min(minEllipse.size());
    for(int i = 0; i<minEllipse.size(); i++){
    	 double min = NUM_INF;
    	for(int j = 0; j<minEllipse.size(); j++){
    		if(mat_distance[i][j] < min){
    			min_distance[i] = mat_distance[i][j];
    			min = min_distance[i];
    			index_min[i] = j;
    		}
    	}
    }

    //check the minimum in the column and check the corresponding angle
         double threshold_distance= 0.3;
         double threshold_angle = 0.1;//low threshld since the noise is compensated
         vector<RotatedRect> finalEllipse(2);
         for(int i = 0; i<minEllipse.size(); i++){
        	 	if(min_distance[i]<threshold_distance && min_distance[i]!=0 && mat_angle[i][index_min[i]]<threshold_angle)
        	 		finalEllipse[0] = minEllipse[i];
    	 		    finalEllipse[1] = minEllipse[index_min[i]];


         }
    // Draw polygonal contour + bonding rects + circles
    for( int i = 0; i< finalEllipse.size(); i++ )
       {
    	cout<<finalEllipse[0].center<<" "<<finalEllipse[1].center<<endl;
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
         ellipse( drawing, finalEllipse[i], color, 2, 8 );
         //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
         //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
}


    // Show your result

#ifdef GUI
	//imshow("Original image", src);
	//imshow("Blurred", blurred);
	namedWindow( "Hough Circle Transform", CV_WINDOW_AUTOSIZE );
	imshow( "Hough Circle Transform", src_gray );
	namedWindow( "Threshold", CV_WINDOW_AUTOSIZE );
	imshow("Threshold", thresholded);
	namedWindow( "drawing", CV_WINDOW_AUTOSIZE );
	cv::imshow("drawing", drawing);
	namedWindow( "thresholdedcont", CV_WINDOW_AUTOSIZE );
	cv::imshow("thresholdedcont", thresholdedcont);
	cv::waitKey(1);
#endif
_Patch patch;
  return patch;
}
