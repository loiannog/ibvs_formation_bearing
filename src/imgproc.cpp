#include "imgproc.h"
#include <algorithm>
#include <cstdio>



/// Global variables
Mat erosion_dst, dilation_dst;
int dilation_elem = 2;
int erosion_elem = 2;
int erosion_size = 2;
int dilation_size = 2;
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

Mat getColor(cv::Mat &srcBGR)
{
  cv::Mat mask(srcBGR.rows, srcBGR.cols, CV_8UC1);
  cv::Mat hsv(srcBGR.rows, srcBGR.cols, CV_8UC1);
  cvtColor(srcBGR, hsv, CV_BGR2HSV);
  inRange(hsv, Scalar(0,20, 20),
	                Scalar(10, 255, 255), mask);
  //cvtColor(hsv, hsv, CV_BGR2GRAY);
//red good Scalar(0,20, 20), Scalar(10, 255, 255)


  mask = Erosion(mask);
  mask = Dilation(mask);
  medianBlur(mask, mask, 11);//smooth the image
  //GaussianBlur(mask, mask, Size(11,11), 0);//smooth the image

  return mask;
}

//this is not used for the moment
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



