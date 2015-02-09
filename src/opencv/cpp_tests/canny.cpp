#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
using namespace std;
using namespace cv;

cv::Mat frame, shi_dest;
cv::Mat gray_frame;
RNG rng(12345);

cv::Mat shi_tomasi(){
  vector<Point2f> corners;
  double quality = 0.01;
  double min_dist = 10;
  int block_size = 3;
  bool use_harris = false;
  double k = 0.04;
  Mat copy;
  copy = frame.clone();
  goodFeaturesToTrack( gray_frame,
                      corners,
                      10,
                      quality,
                      min_dist,
                      Mat(),
                      block_size,
                      use_harris,
                      k );
  int r = 4;
  for(int i=0; i<corners.size(); i++){
    circle(copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0);
  }
  namedWindow("shi");
  imshow("shi", copy);
}

int main()
{
  cv::VideoCapture camera("/home/maps/Videos/hallway.webm");
  if(!camera.isOpened()){
    return 1;
  }
  cv::Mat canny_out;
  cv::Mat canny_color;
  cv::Mat hough_img;
  cv::Mat small;
  cv::Mat gray_frame_C1(320, 240, CV_8UC3);
  cv::Mat hue(320, 240, CV_8UC1);
  cv::Mat sat(320, 240, CV_8UC1);
  cv::Mat val(320, 240, CV_8UC1);
  std::vector<cv::Point2f> corners_dst;
  std::vector<cv::Mat> hsv_planes;
  cv::namedWindow("frame");
  cv::namedWindow("canny");
  cv::namedWindow("hough transform");
  vector<cv::Vec4i> lines;
  while(true){
    if(!camera.read(frame)){
      break;
    }
    cv::cvtColor(frame, gray_frame, CV_BGR2GRAY);
    cv::pyrDown(gray_frame, small, cv::Size(gray_frame.cols/2, gray_frame.rows/2));
    cv::pyrUp(small, gray_frame, cv::Size(small.cols*2, small.rows*2));
    cv::cvtColor(frame, gray_frame_C1, CV_BGR2HSV);
    cv::split(gray_frame_C1, hsv_planes);
    cv::threshold(hsv_planes[2], hsv_planes[2], 254, 255, CV_THRESH_BINARY);
    cv::Canny(gray_frame, canny_out, 100, 300);
    //cv::threshold(canny_out, canny_out, 128, 255, cv::THRESH_BINARY_INV);
    cv::goodFeaturesToTrack(hsv_planes[2], corners_dst, 10, .95, 10.0, cv::noArray(), 3, true);
    cv::cvtColor(canny_out, canny_color, CV_GRAY2BGR);
    cv::HoughLinesP(canny_out, lines, 1, CV_PI/180, 80, 100, 10);
    for( size_t i=0; i<lines.size(); i++){
          cv::Vec4i l = lines[i];
          if (abs(l[0] - l[2]) < 30 || abs(l[1] - l[3]) < 30){
                cv::line(canny_color, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
          }
    }
    for( size_t i=0; i<corners_dst.size(); i++){
        cv::circle(canny_color, corners_dst.at(i), 10, cv::Scalar(255,0,0), 2);
    }
    //shi_tomasi();
    cv::imshow("frame", hsv_planes[2]);
    cv::imshow("canny", canny_out);
    cv::imshow("hough transform", canny_color);
    if(cv::waitKey(10)>=0){
          break;
    }
  }
}
