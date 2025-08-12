//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h> 
#include <string.h> 
#include <stdio.h>
#include <unistd.h>   
#include <stdint.h>   
#include <stdlib.h>  
#include <errno.h>

using namespace cv;
using namespace std;

#define IMG_Width     640
#define IMG_Height    480

#define USE_DEBUG  1   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용  0 CAMERA 미사용

#define NO_LINE 300
#define MAX_STEER_ANGLE  30 

Mat mat_image_org_color;  // Image 저장하기 위한 변수
Mat mat_image_org_color_crop;
Mat mat_image_org_gray;
Mat mat_image_roi_crop;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge1;
//Mat mat_image_canny_edge_roi;
Mat mat_image_roi_gray;
Mat mat_image_roi_gray_crop;
Mat mat_image_roi_threshold;

Scalar GREEN(0, 255, 0);
Scalar RED(0, 0, 255);
Scalar BLUE(255, 0, 0);
Scalar YELLOW(0, 255, 255);
Scalar ORANGE(0,165,255);


int img_width    = 640;
int img_height   = 480;


//////////////////////////////////////// PID control ////////////////////////////////////////////////

double  Kp = 0.4;  double Ki = 0; double  Kd = 0.1;

double  error_lane    = 0; 
double  error_lane_delta    = 0; 
double  error_lane_old = 0;

bool   lane_control_run_flag = false;

double lane_width = 300;

struct Rect_Region
{
	int left;
	int right;
	int top;
	int bottom;
	
};

struct Rect_Region ROI_lane;

void CamImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	
}

void run_lane_control_flagCallback(const std_msgs::Bool& msg)
{
	lane_control_run_flag = msg.data;
	
}

Mat Region_of_Interest(Mat image, Point* points)
{
    Mat img_mask = Mat::zeros(image.rows, image.cols, CV_8UC1);

    Scalar mask_color = Scalar(255, 255, 255);
    const Point* pt[1] = { points };
    int npt[] = { 4 };
    cv::fillPoly(img_mask, pt, npt, 1, Scalar(255, 255, 255), LINE_8);
    Mat masked_img;
    bitwise_and(image, img_mask, masked_img);

    return masked_img;
}

Mat Region_of_Interest_crop(Mat image, Point* points)
{
    Mat img_roi_crop_temp;
    Mat image2;
    image.copyTo(image2);

    Rect bounds(0, 0, image2.cols, image2.rows);
    Rect r(points[0].x, points[0].y, points[2].x-points[0].x, points[2].y - points[0].y);
   // Rect r(points[0].x, points[0].y, points[2].x, points[2].y);
    //Rect r(points[0].x, points[0].y, points[2].x,300);
   // printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x, points[2].y);
    //printf("%d  %d\n", image.cols, points[2].y-points[0].y);
    img_roi_crop_temp = image2(r & bounds);

    return img_roi_crop_temp;
}



double line_blob_detection(Mat mat_img_crop)
{
	int max_area   = -1;
	int max_area_i = -1;  
	cvtColor(mat_img_crop, mat_image_roi_gray, cv::COLOR_RGB2GRAY);        // color to gray conversion
	 
	erode(mat_image_roi_threshold, mat_image_roi_threshold, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),1);
	
	Mat img_labels,stats, centroids;  
	  
	int numOfLables = connectedComponentsWithStats(mat_image_roi_threshold, img_labels, stats, centroids, 8,CV_32S);  
    
    
	for (int i = 1; i < numOfLables; i++) 
	{ 
		 int left  = stats.at<int>(i,CC_STAT_LEFT);
		 int width = stats.at<int>(i,CC_STAT_WIDTH); 
		 int area  = stats.at<int>(i,CC_STAT_AREA);
		  
		 if(area > max_area)
		 {
			max_area   = area;
			max_area_i = i; 
		 }  
	 } 
	 
	if(max_area_i != -1)
	{ 
		double x = centroids.at<double>(max_area_i, 0); //중심좌표
		double y = centroids.at<double>(max_area_i, 1); 
	  
		printf("Max Blob location : [%4.1lf ,%4.1lf]\n", x,y) ;
		return x;
	}
	else
	{
		return -1;
	}
	
		    //printf("R Line Center(Blob) : %.d %d %d %d \n\n", j,left,width,left+width/2 );
		 
}

int main(int argc, char** argv)
{
	int i;
	
	double gradient[NO_LINE]        = { 0, };
	double intersect_x[NO_LINE]     = { 0, };
	double intersect_left[NO_LINE]  = { 0, };
	double intersect_right[NO_LINE] = { 0, };
	
	double c_x_sum = 0;

	int capture_width  = 640; //1280 ;
	int capture_height = 480; //720 ;
	int display_width  = 640;
	int display_height = 480;
	int framerate     = 60;
	int flip_method   = 2;
	int left_line_no  = 0;
	int right_line_no = 0;

	double  c[NO_LINE] = { 0.0, };
	double  d[NO_LINE] = { 0.0, };
	
	double  valid_c[NO_LINE] = { 0.0, };
	double  valid_d[NO_LINE] = { 0.0, };
	
	double  line_left_x  = 0.0;
	double  line_right_x = 0.0;

	double steer_angle_old, steer_angle_new ;
	double *euclidean_distance_matrix;    // euclidean matrix 
	double line_center = -1;
	
	int perspective_upper = 137;
	int perspective_lower = 319;
	
	cv::Mat gray_image_yellow;
	cv::Mat gray_image_blue;
	
	cv::Mat binary_image_yellow;
	cv::Mat binary_image_blue;
	
	cv::Mat binary_image_yellow_erode;
	cv::Mat binary_image_blue_erode;
			

	steer_angle_old =  steer_angle_new = 0.0;
	ros::init(argc, argv, "ros_color_detect");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle nh;

	int img_width  = 640;
	int img_height = 480;
	
	int blue_b1,blue_g1,blue_r1;
	int blue_b2,blue_g2,blue_r2;


	blue_b1 =  80; blue_g1 = 20; blue_r1 = 10;
	blue_b2 = 255; blue_g2 = 70; blue_r2 = 60;
	
		
	std::string input_image_topic           = "/usb_cam/image_raw";
	    
	ros::param::get("~img_width",     img_width);     
	ros::param::get("~img_height",    img_height);
	
	ros::param::get("~blue_b1",       blue_b1);
	ros::param::get("~blue_g1",       blue_g1);
	ros::param::get("~blue_r1",       blue_r1);
	
	ros::param::get("~blue_b2",       blue_b2);
	ros::param::get("~blue_g2",       blue_g2);
	ros::param::get("~blue_r2",       blue_r2);

	
  
    
	ros::Subscriber image_sub                      = nh.subscribe(input_image_topic, 1, &CamImageCallback);
	ros::Subscriber run_lane_control_flag_sub      = nh.subscribe("/flag/lane_control_run",1, run_lane_control_flagCallback);
	
	ros::Publisher cross_track_error_pub           = nh.advertise<std_msgs::Float32>("/cross_track_error/lane",1);
	ros::Publisher vision_car_control_pub          = nh.advertise<std_msgs::Int16>("/Car_Control_cmd/Vision_SteerAngle_Int16", 1);
	
	std_msgs::Float32 cross_track_error_msg;
	cross_track_error_msg.data = 0;

	std::cout << "OpenCV version : " << CV_VERSION << std::endl;

	float  track_offset_error = 0.0;

	
	printf("\n\n\n");
	
	namedWindow("Camera Image", WINDOW_NORMAL);
	resizeWindow("Camera Image", IMG_Width / 2, IMG_Height / 2);
	moveWindow("Camera Image",10, 250); 
	
	namedWindow("Yellow Detection", WINDOW_NORMAL);
    resizeWindow("Yellow Detection", img_width/2,img_height/2);
	moveWindow("Yellow Detection", 400, 250);
	
	namedWindow("Blue Detection", WINDOW_NORMAL);
    resizeWindow("Blue Detection", img_width/2,img_height/2);
	moveWindow("Blue Detection", 730, 250);

	
    /*
     * 
	namedWindow("Camera Image", WINDOW_NORMAL);
	resizeWindow("Camera Image", IMG_Width / 2, IMG_Height / 2);
	moveWindow("Camera Image", 10, 10); 

    namedWindow("Gray Image window", WINDOW_NORMAL);
    resizeWindow("Gray Image window", img_width,img_height);
    moveWindow("Gray Image window", 700, 10);

   
    namedWindow("Edge Image window", WINDOW_NORMAL);
    resizeWindow("Edge Image window", img_width,img_height);
    moveWindow("Edge Image window", 700, 500);
   
    namedWindow("Bird eye Image window", WINDOW_NORMAL);
	resizeWindow("Bird eye Image window", 400,300);
    moveWindow("Bird eye Image window", 700, 10);
	 */
	sensor_msgs::ImageConstPtr image_topic_ptr;
	
	ros::Rate loop_rate(30);
	int count = 0;

	
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
		
		image_topic_ptr  = ros::topic::waitForMessage<sensor_msgs::Image>(input_image_topic,nh,ros::Duration(2) )  ; 
		
		if(image_topic_ptr == NULL) ROS_WARN("Image topic does not exit!");
		else
		{
			mat_image_org_color = cv_bridge::toCvShare(image_topic_ptr, "bgr8")->image;
					
			int width  = mat_image_org_color.size().width;
			int height = mat_image_org_color.size().height;
			
			printf("이미지의 너비: %d 픽셀, 높이: %d 픽셀\n", width, height);
			
			cv::imshow("Camera Image", mat_image_org_color);
			
			cv::Mat hsv;
			cv::cvtColor(mat_image_org_color, hsv, cv::COLOR_BGR2HSV);

			// 노란색 범위 (HSV에서) 여기 조정할 것
			
			
			cv::Scalar lower_yellow(20, 100, 100);
			cv::Scalar upper_yellow(70, 255, 255);

			// 노란색 마스크 생성
			cv::Mat yellow_mask;
			cv::inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

			// 노란색 픽셀 표시 (노란색 부분을 흰색으로 만듭니다.)
			cv::Mat result_yellow;
			cv::bitwise_and(mat_image_org_color, mat_image_org_color, result_yellow, yellow_mask);
			
			cv::cvtColor(result_yellow, gray_image_yellow, cv::COLOR_BGR2GRAY);
			cv::threshold(gray_image_yellow, binary_image_yellow, 60, 255, cv::THRESH_BINARY);
			
			
			
			/////////// blue con detect /////////////////////////////////

			cv::Scalar lower_blue = cv::Scalar(blue_b1, blue_g1, blue_r1);  // 파란색 범위의 하한 값
			cv::Scalar upper_blue = cv::Scalar(blue_b2, blue_g2, blue_r2); // 파란색 범위의 상한 값		
		
			
			// 파란색 픽셀 표시 (파란색 부분을 흰색으로 만듭니다.)
			cv::inRange(mat_image_org_color, lower_blue, upper_blue, binary_image_blue);
			
			////////////////////////////////////////////////////////////////////
			
			erode(binary_image_yellow, binary_image_yellow_erode, Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),4);
			erode(binary_image_blue,   binary_image_blue_erode,   Mat::ones(Size(3,3),CV_8UC1),Point(-1,-1),4);


			// 결과 이미지 표시
			
			cv::imshow("Yellow Detection",binary_image_yellow_erode);
			cv::imshow("Blue Detection", binary_image_blue_erode);
			
			if (cv::waitKey(25) >= 0)
				break;;
		}			 
			

        ros::spinOnce();
        
        loop_rate.sleep();
        ++count;
    }

    destroyWindow("Camera Image");
    destroyWindow("Yellow Detection");
    destroyWindow("Blue Detection");
    //destroyWindow("Bird eye Image window");
    //destroyWindow("Gray Image window");
    //destroyWindow("ROI Image window");
    //destroyWindow("Edge Image window");
    return 0;
}

