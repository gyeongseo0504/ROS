//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#define LEFT            -1
#define CENTER           0
#define RIGHT            1

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
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
int ROI_CENTER_Y = 180;
int ROI_WIDTH    =  30;
int canney_low   =  50;
int canney_high  = 170;


//////////////////////////////////////// PID control ////////////////////////////////////////////////

bool   lane_control_run_set = false;
bool   avoid_control_enable_flag = false;
int    car_avoid_steer_direction = CENTER;

double lane_width = 300;
double vision_xte_offset = 0.0; 


struct Rect_Region
{
	int left;
	int right;
	int top;
	int bottom;
	
};

struct Rect_Region ROI_lane;


void avoid_control_enable_Callback(const std_msgs::Bool& msg)
{
	avoid_control_enable_flag = msg.data;
}

void car_avoid_steer_directoin_Callback(const std_msgs::Int8& msg)
{
	car_avoid_steer_direction = msg.data;
}
void CamImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	
}


void vision_xte_offset_topic_Callback(const std_msgs::Float32& msg)
{
	vision_xte_offset = msg.data;	
}

void run_lane_control_flagCallback(const std_msgs::Bool& msg)
{
	lane_control_run_set = msg.data;
	
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

Mat Canny_Edge_Detection(Mat img)
{
    Mat mat_blur_img, mat_canny_img;
    blur(img, mat_blur_img, Size(3, 3));
    Canny(mat_blur_img, mat_canny_img, canney_low, canney_high, 3);

    return mat_canny_img;
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
	/////////////////// 카메라 image는 640x480으로 고정 ////////////////////////
	
	int i;

	double gradient[NO_LINE]        = { 0, };
	double intersect_x[NO_LINE]     = { 0, };
	double intersect_left[NO_LINE]  = { 0, };
	double intersect_right[NO_LINE] = { 0, };
	
	double c_x_sum = 0;

	int capture_width        = 640; //1280 ;
	int capture_height       = 480; //720 ;
	int display_width        = 640;
	int display_height       = 480;
	int framerate            = 60;
	int left_line_no         = 0;
	int right_line_no         = 0;

	double  c[NO_LINE]       = { 0.0, };
	double  d[NO_LINE]       = { 0.0, };
	
	double  valid_c[NO_LINE] = { 0.0, };
	double  valid_d[NO_LINE] = { 0.0, };
	
	double  line_left_x  = 0.0;
	double  line_right_x = 0.0;

	double steer_angle_old, steer_angle_new ;
	double *euclidean_distance_matrix;    // euclidean matrix 
	double line_center       = -1;
	
	int perspective_upper    = 137;
	int perspective_lower    = 319;
	
	int left_pos             = IMG_Width*0.25;
	int right_pos            = IMG_Width*0.75;
	
	int left_window_size     = 50;
	int right_window_size    = 50;
	

	steer_angle_old =  steer_angle_new = 0.0;
	ros::init(argc, argv, "ros_lane_control");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle nh;

	int img_width  = 640;
	int img_height = 480;

	std::string input_image_topic           = "/usb_cam/image_raw";
	std::string vision_xte_offset_topic     = "/vision_xte_offset";
	std::string vision_xte_topic            = "/vision_xte";
	ros::param::get("~input_image_topic",input_image_topic);
	
	ros::param::get("~img_width",     img_width);     
	ros::param::get("~img_height",    img_height);
	ros::param::get("~ROI_CENTER_Y",  ROI_CENTER_Y);  
	ros::param::get("~ROI_WIDTH",     ROI_WIDTH);  
	
	ros::param::get("~perspective_upper",     perspective_upper);  
	ros::param::get("~perspective_lower",     perspective_lower);  
	
	ros::param::get("~lane_width",    lane_width);
	
	ros::param::get("~canney_low",    canney_low);
	ros::param::get("~canney_high" ,  canney_high);
	ros::param::get("~vision_xte_topic",vision_xte_topic);
	ros::param::get("~vision_xte_offset_topic",vision_xte_offset_topic);
	
    ros::Subscriber image_sub                       = nh.subscribe(input_image_topic, 1, &CamImageCallback);
	ros::Subscriber run_lane_control_flag_sub       = nh.subscribe("/flag/lane_control_set",1, &run_lane_control_flagCallback);
	ros::Subscriber cmd_vel_vision_xte__sub         = nh.subscribe(vision_xte_offset_topic,1, &vision_xte_offset_topic_Callback);
	ros::Subscriber avoid_control_enable_sub        = nh.subscribe("/flag/avoid_control_lane_enable",1, &avoid_control_enable_Callback); 
	
	ros::Subscriber car_avoid_steer_directoin_sub   = nh.subscribe("/Car_Control_Cmd/avoid_direction", 1,&car_avoid_steer_directoin_Callback);
	
	
	
	ros::Publisher cross_track_error_pub            = nh.advertise<std_msgs::Float32>(vision_xte_topic,1);
	
	std_msgs::Float32 cross_track_error_msg;
	cross_track_error_msg.data = 0;

	std::cout << "OpenCV version : " << CV_VERSION << std::endl;

	float  track_offset_error = 0.0;

	vector<Point2f> srcPts, dstPts ;
	Point points[4];
	Point Perspect[4];
	
	
	//   p1    p2
	//   p3    p4
	
	Mat  perspective_mat;
	
	Point p1s  = Point2f( img_width / 2 - perspective_upper ,   2);
	Point p2s  = Point2f( img_width / 2 + perspective_upper ,   2);
	Point p3s  = Point2f( img_width / 2 - perspective_lower ,   ROI_WIDTH*2 - 3);
	Point p4s  = Point2f( img_width / 2 + perspective_lower ,   ROI_WIDTH*2 - 3);
	
	
	Point p1d  = Point2f(0 ,0);
	Point p2d  = Point2f(300,0);
	Point p3d  = Point2f(0,250);
	Point p4d  = Point2f(300,250);
	
	srcPts = { p1s, p2s, p3s, p4s};
	dstPts = { p1d, p2d, p3d, p4d};
	
	perspective_mat = getPerspectiveTransform(srcPts, dstPts);
	
	
	printf("\n\n\n");
	
	//namedWindow("Camera Image", WINDOW_NORMAL);
	//resizeWindow("Camera Image", IMG_Width / 2, IMG_Height / 2);
	//moveWindow("Camera Image", 10, 10); 
	
	namedWindow("ROI Image window", WINDOW_NORMAL);
    resizeWindow("ROI Image window", img_width/2,img_height/2);
	moveWindow("ROI Image window", 10, 50);

	namedWindow("Edge Image window", WINDOW_NORMAL);
    resizeWindow("Edge Image window", img_width,img_height);
    moveWindow("Edge Image window", 700, 500);
    
    /*
     * 
	namedWindow("Camera Image", WINDOW_NORMAL);
	resizeWindow("Camera Image", IMG_Width / 2, IMG_Height / 2);
	moveWindow("Camera Image", 10, 10); 

    namedWindow("Gray Image window", WINDOW_NORMAL);
    resizeWindow("Gray Image window", img_width,img_height);
    moveWindow("Gray Image window", 700, 10);

   
    
    */
    //namedWindow("Bird eye Image window", WINDOW_NORMAL);
	//resizeWindow("Bird eye Image window", 400,300);
    //moveWindow("Bird eye Image window", 700, 10);
	
	sensor_msgs::ImageConstPtr image_topic_ptr;
	
	ros::Rate loop_rate(30);
	int count = 0;
	
	points[0] = Point(0, ROI_CENTER_Y - ROI_WIDTH);
	points[1] = Point(0, ROI_CENTER_Y + ROI_WIDTH);
	points[2] = Point(img_width, ROI_CENTER_Y + ROI_WIDTH);
	points[3] = Point(img_width, ROI_CENTER_Y - ROI_WIDTH);
			
	Perspect[0]  = Point2f( img_width / 2 - perspective_upper ,   0);
	Perspect[1]  = Point2f( img_width / 2 - perspective_lower ,   ROI_WIDTH*2 - 0);
	Perspect[2]  = Point2f( img_width / 2 + perspective_lower ,   ROI_WIDTH*2 - 0);
	Perspect[3]  = Point2f( img_width / 2 + perspective_upper ,   0);
			
	
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
		
		if(lane_control_run_set == true)
		{
						image_topic_ptr  = ros::topic::waitForMessage<sensor_msgs::Image>(input_image_topic,nh,ros::Duration(1) ); 
			
			if(avoid_control_enable_flag == true)
			{
				printf("avoid control enable !!\n");
				printf("Stop Image Capture\n");
				printf("car_avoid_steer_direction : %1d\n",car_avoid_steer_direction);
				
				if(car_avoid_steer_direction == CENTER)
				{
					cross_track_error_msg.data = 0 ;
					
				}
				else if (car_avoid_steer_direction == LEFT)
				{
					cross_track_error_msg.data =  600;
				}
				else if (car_avoid_steer_direction == RIGHT)
				{
					cross_track_error_msg.data = -600;
				}
				else
				{
					cross_track_error_msg.data = 0;
				}
				
				printf("Lane Cross Track Error  : %6.3lf\n",cross_track_error_msg.data);
				printf("Lane Cross Track Offset : %6.3lf\n",vision_xte_offset);
				cross_track_error_pub.publish(cross_track_error_msg);
				
				image_topic_ptr = NULL;
			}
			
			if(image_topic_ptr == NULL)
			{
				 ROS_WARN("Image topic does not exit!");		
				 
			}		
			else
			{
				mat_image_org_color = cv_bridge::toCvShare(image_topic_ptr, "bgr8")->image;
						
						 
				int width  = mat_image_org_color.size().width;
				int height = mat_image_org_color.size().height;
						 
				if( (width !=640) || (height !=480 ) )   
				{
					cv::resize(mat_image_org_color, mat_image_org_color, cv::Size(480, 640), 1);
				}
				
				width  = mat_image_org_color.size().width;
				height = mat_image_org_color.size().height;
				
				printf("image size : [%3d %3d]\n", width, height);
				
				
				//imshow("Camera Image", mat_image_roi_crop);
						  
				/*
				int roi_left  =  (line_center_blob - 200) < 0     ?     0 : (line_center_blob - 200) ;
				int roi_right =  (line_center_blob + 200) > width ? width : (line_center_blob + 200) ;
				*/
				int roi_left  = 0;
				int roi_right = 640;
							
				
				mat_image_roi_crop  = Region_of_Interest_crop(mat_image_org_color, points);    // ROI 영역을 추출함 blob 추출 한 것에서 수정해서 진행함
				//width  = mat_image_roi_crop.size().width;
				//height = mat_image_roi_crop.size().height;
				
					
				int line_center_blob =  width/2 ; //line_blob_detection(mat_image_roi_crop);
				
				line(mat_image_roi_crop,Point(line_center_blob,0),Point(line_center_blob,height), Scalar(0,255,0), 1, LINE_AA);
				
				cvtColor(mat_image_roi_crop, mat_image_roi_gray_crop, cv::COLOR_RGB2GRAY);        // color to gray conversion
				
			  
				mat_image_canny_edge   = Canny_Edge_Detection(mat_image_roi_gray_crop);
				
				imshow("Edge Image window", mat_image_canny_edge);
				
				mat_image_canny_edge1  = Region_of_Interest(mat_image_canny_edge, Perspect);
				
				vector<Vec4i> linesP;
				HoughLinesP(mat_image_canny_edge1, linesP, 1, CV_PI / 180, 40, 40, 40);
				//printf("Line Number : %3d\n", (int)linesP.size());

				line_left_x = 0.0;
				line_right_x = 0.0;

				int valid_line_no       = 0;
				int valid_line_left_no  = 0;
				int valid_line_right_no = 0;
				
				Mat mat_image_bird_eye = cv::Mat::zeros(400,300,CV_8U);
				
				
				for(int i = 0; i < linesP.size(); i++)
				{
					double intersect = 0.0;

					if (i >= NO_LINE) break;
					Vec4i L = linesP[i];

					if (fabs((L[3] - L[1])) > 1.0e-7)
					{
						c[i] = ((double)L[2] - (double)L[0]) /( (double)L[3] - (double)L[1]  );
						//printf("[%3d]line slope : %6.3lf \n",i,c[i]);
					}
					else
					{
						c[i] = 1.0e7;
					}

					if ( fabs(c[i]) < 6)
					{ 
						
						d[i]      = (double)L[0] - (double)c[i] * (double)L[1];
						intersect = (double)c[i] * (double) ROI_WIDTH + (double)d[i];
						
						valid_d[valid_line_no] = d[i];
						valid_c[valid_line_no] = c[i];
						
						intersect_x[valid_line_no] = intersect;
										
						valid_line_no++;
										

						//line(mat_image_roi_crop, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(0, 255, 255), 1, LINE_AA);
						
						
						//printf("%3d, %3d  | %6.3lf,%6.3lf\n",L[0],L[1],x1/w1,y1/w1);
						//printf("%3d, %3d  | %6.3lf,%6.3lf\n",L[2],L[3],x2/w2,y2/w2);
						
						//printf("L[%d] :[%3d, %3d] , [%3d , %3d] \n", i, L[0], L[1], L[2], L[3]);
						
						//printf("x[%d] = [%f] *y + [%f] \n", i, c[i], d[i]);
						//printf("instersect left =[%f] [%f] [%f] \n\n", intersect, line_left_x, line_right_x);
						
						//printf("H :[%3d , %3d] , [%3d , %3d] \n", cx1,cy1, cx2,cy2);
					   
					}
				}
				
				//printf("Valid line No : %d \n",valid_line_no);
				// Clustering two group;
				//double *euclidean_distance_matrix;    // euclidean matrix 
				double max_dissimilarity   = -1.0;
				int    max_dissimilarity_i = -1;
				int    no_line_class = 0;
				if(valid_line_no >= 1)
				{
					euclidean_distance_matrix = new double[valid_line_no];
									
					for(int j = 0; j < valid_line_no; j++)
					{
						euclidean_distance_matrix[j] =  fabs(intersect_x[0] - intersect_x[j]);
						if( max_dissimilarity < euclidean_distance_matrix[j] )
							{
								max_dissimilarity = euclidean_distance_matrix[j];
								max_dissimilarity_i = j;
							}
							printf("%6.3lf ",euclidean_distance_matrix[j] );
					}
					printf("\n");
					//printf("max_dissimilarity_i : %d    /    max_dissimilarity : %6.3lf\n", max_dissimilarity_i ,max_dissimilarity);
					if( max_dissimilarity > 300)  no_line_class = 2;
				}
				
				printf("no_line_class %d\n",no_line_class);
				
				double left_line_c = 0;    double left_line_d = 0;    int left_line_cnt = 0;
				double right_line_c = 0;   double right_line_d = 0;   int right_line_cnt = 0;
				
				//printf("right :  %d %6.3lf %6.3lf\n", right_line_cnt, right_line_c, right_line_d);

				left_line_cnt = right_line_cnt =0;			
				for(i = 0; i < valid_line_no; i++)
				{
					if(euclidean_distance_matrix[i] < 30) 
					{
						left_line_c += valid_c[i];
						left_line_d += valid_d[i];
						left_line_cnt++;
						//printf("left  :  %d %6.3lf %6.3lf\n", left_line_cnt, left_line_c, left_line_d);
					}						
				}
				if(left_line_cnt!=0)
				{
						left_line_c /= left_line_cnt;
						left_line_d /= left_line_cnt;
				}	
				
				if(no_line_class == 2)
				{
					for(i = 0; i < valid_line_no; i++)
					{
						if(euclidean_distance_matrix[i] > 300) 
						{
							right_line_c += valid_c[i];
							right_line_d += valid_d[i];
							right_line_cnt++;	
						}
					}
					if(right_line_cnt!=0)
					{
						right_line_c /= right_line_cnt;
						right_line_d /= right_line_cnt;
					}
				}	
				
				
				
				if(left_line_cnt !=0)
				{
					double x1,y1, x2,y2;
					double x3,y3, x4,y4, w3,w4;
				
					y1 = 10; 
					x1 = left_line_c * y1 + left_line_d;
					y2 = ROI_WIDTH*2 -10; 
					x2 = left_line_c * y2 + left_line_d;
					line(mat_image_roi_crop, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 2, LINE_AA);
					line_left_x = x2;
					/*
					double x3,y3, x4,y4, w3,w4;
					x3 = perspective_mat.at<double>(0,0)*x1 + perspective_mat.at<double>(0,1)*y1 + perspective_mat.at<double>(0,2)*1;
					y3 = perspective_mat.at<double>(1,0)*x1 + perspective_mat.at<double>(1,1)*y1 + perspective_mat.at<double>(1,2)*1;
					w3 = perspective_mat.at<double>(2,0)*x1 + perspective_mat.at<double>(2,1)*y1 + perspective_mat.at<double>(2,2)*1;

					x4 = perspective_mat.at<double>(0,0)*x2 + perspective_mat.at<double>(0,1)*y2 + perspective_mat.at<double>(0,2)*1;
					y4 = perspective_mat.at<double>(1,0)*x2 + perspective_mat.at<double>(1,1)*y2 + perspective_mat.at<double>(1,2)*1;
					w4 = perspective_mat.at<double>(2,0)*x2 + perspective_mat.at<double>(2,1)*y2 + perspective_mat.at<double>(2,2)*1;
					*/
					//printf("left  : [%6.3lf %6.3lf]  [%6.3lf %6.3lf] \n", x1,y1,x2,y2);
					//printf("left  : [%6.3lf %6.3lf]  [%6.3lf %6.3lf] \n", x3/w3, y3/w3,x4/w4,y4/w4);
					
					line(mat_image_bird_eye,Point(x3/w3, y3/w3), Point(x4/w4,y4/w4), Scalar(255), 5, LINE_AA);
				}
				
				if(right_line_cnt !=0)
				{
					double x1,y1, x2,y2;
					double x3,y3, x4,y4, w3,w4;
					y1 = 10; 
					x1 = right_line_c * y1 + right_line_d;
					y2 = ROI_WIDTH*2 -10; 
					x2 = right_line_c * y2 + right_line_d;
					line_right_x = x2;
					
					line(mat_image_roi_crop, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 2, LINE_AA);
					//printf("right : [%6.3lf %6.3lf]  [%6.3lf %6.3lf] \n", x1,y1,x2,y2);
					
					
					x3 = perspective_mat.at<double>(0,0)*x1 + perspective_mat.at<double>(0,1)*y1 + perspective_mat.at<double>(0,2)*1;
					y3 = perspective_mat.at<double>(1,0)*x1 + perspective_mat.at<double>(1,1)*y1 + perspective_mat.at<double>(1,2)*1;
					w3 = perspective_mat.at<double>(2,0)*x1 + perspective_mat.at<double>(2,1)*y1 + perspective_mat.at<double>(2,2)*1;

					x4 = perspective_mat.at<double>(0,0)*x2 + perspective_mat.at<double>(0,1)*y2 + perspective_mat.at<double>(0,2)*1;
					y4 = perspective_mat.at<double>(1,0)*x2 + perspective_mat.at<double>(1,1)*y2 + perspective_mat.at<double>(1,2)*1;
					w4 = perspective_mat.at<double>(2,0)*x2 + perspective_mat.at<double>(2,1)*y2 + perspective_mat.at<double>(2,2)*1;

					line(mat_image_bird_eye,Point(x3/w3, y3/w3), Point(x4/w4,y4/w4), Scalar(255), 5, LINE_AA);
				}
						
				printf("Valid Line Number : %3d %3d\n", left_line_cnt, right_line_cnt );
				
				if( (left_line_cnt!=0) && (right_line_cnt !=0))
				{
					line_center = (line_left_x + line_right_x)/2;
					printf("Lane Center : %4.1lf  %4.1lf  %4.1lf %4.1lf\n", line_left_x, line_right_x, line_center,fabs(line_left_x-line_right_x) );
					
					line(mat_image_roi_crop, Point(line_center,0), Point(line_center, ROI_WIDTH*2),  Scalar(147, 20,255), 1, LINE_AA);
				}
				
				if( right_line_cnt == 0 ) 
				{
					printf( "line_left_x :%lf \n",line_left_x);
					
					if( line_left_x < (img_width/2) )
					{
						line_center = (line_left_x + line_left_x + lane_width)/2;
						
					}
					else
					{
						line_center = (line_left_x + line_left_x - lane_width)/2;
						printf("right line : %lf %lf\n", line_left_x , line_left_x - lane_width);
					}
					printf("Lane Center : %4.1lf  %4.1lf  %4.1lf \n", line_left_x, line_right_x, line_center);
					line(mat_image_roi_crop, Point(line_center,0), Point(line_center, ROI_WIDTH*2),  Scalar( 147, 20,255), 1, LINE_AA);
				}
				
				if( left_line_cnt == 0 ) 
				{
					if( line_right_x < (img_width/2) )
					{
						line_center = (line_right_x + line_right_x+lane_width)/2;
						
					}
					else
					{
						line_center = (line_right_x + line_right_x-lane_width)/2;
					}
					printf("Lane Center : %4.1lf  %4.1lf  %4.1lf \n", line_left_x, line_right_x, line_center);
					line(mat_image_roi_crop, Point(line_center,0), Point(line_center, ROI_WIDTH*2),  Scalar( 147, 20,255), 1, LINE_AA);
				}
				if(valid_line_no >= 1)	delete[] euclidean_distance_matrix; 
			
			
				////////////////////////////////// lane control ///////////////////////////////////////
					
				cross_track_error_msg.data =  img_width/2 - line_center + vision_xte_offset;
				
				printf("Lane Cross Track Error  : %6.3lf\n",cross_track_error_msg.data);
				printf("Lane Cross Track Offset : %6.3lf\n",vision_xte_offset);
				cross_track_error_pub.publish(cross_track_error_msg);
				
										
				line(mat_image_roi_crop, p1s, p2s,  Scalar(0, 255, 0), 1, LINE_AA);
				line(mat_image_roi_crop, p3s, p4s,  Scalar(0, 255, 0), 1, LINE_AA);
				line(mat_image_roi_crop, p1s, p3s,  Scalar(0, 255, 0), 1, LINE_AA);
				line(mat_image_roi_crop, p2s, p4s,  Scalar(0, 255, 0), 1, LINE_AA);
				
				line(mat_image_roi_crop, Point(0,ROI_WIDTH), Point(640, ROI_WIDTH),              Scalar(255, 255,   0), 1, LINE_AA);
				line(mat_image_roi_crop, Point(img_width/2,0), Point(img_width/2, ROI_WIDTH*2),  Scalar(  0,   0, 255), 1, LINE_AA);
				
				//Mat canney_bird_eye;			
				//warpPerspective(mat_image_canny_edge, canney_bird_eye,perspective_mat, Size(300,250), INTER_LINEAR);
				
				imshow("ROI Image window",   mat_image_roi_crop );
				//imshow("Edge Image window",mat_image_canny_edge1);
				//imshow("Bird eye Image window",mat_image_bird_eye);
				
				printf("\n\n\n");
			   /*imshow("Gray Image window", mat_image_org_gray);
				 imshow("ROI Image window",mat_image_roi);			 
			   */
			   if (waitKey(25) >= 0)
			   {
				   
				   
			   }
				

			}
			
		}
		
		else
		{
			ROS_ERROR("Waiting for lane_control_run_set is true");
		}
		

        ros::spinOnce();
        
        loop_rate.sleep();
        ++count;
    }

    //destroyWindow("Camera Image");
    destroyWindow("ROI Image window");
    destroyWindow("Edge Image window");
    //destroyWindow("Gray Image window");
    
    return 0;
}

