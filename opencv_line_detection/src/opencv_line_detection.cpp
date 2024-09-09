#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <math.h>

using namespace cv;
using namespace std;

#define ROI_CENTER_Y 120
#define ROI_WIDTH 20
#define NO_LINE 20

#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

Mat cv_image;

int img_width = 320;
int img_height = 240;

Mat mat_image_org_color;
Mat mat_image_org_gray;
Mat mat_image_roi;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge_roi;

int m_roi_center = 120;
int m_roi_height = 20;
int m_roi_width = 320;
int m_roi_width_large = 3;

float line_center_x = 0.0;
float line_center_x_old = 0.0;

struct Rect_Region
{
    int left;
    int right;
    int top;
    int bottom;
};

struct Rect_Region ROI_line_center;

Mat Region_of_Interest_crop(Mat image, Point *points)
{
    Mat img_roi_crop;
    Rect bounds(0, 0, image.cols, image.rows);
    Rect r(points[0].x, points[0].y, points[2].x - points[0].x, points[2].y - points[0].y);
    img_roi_crop = image(r & bounds);
    return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat img)
{
    Mat mat_blur_img, mat_canny_img;
    blur(img, mat_blur_img, Size(3, 3));
    Canny(mat_blur_img, mat_canny_img, 70, 150, 3);
    return mat_canny_img;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    img_width = cv_image.size().width;
    img_height = cv_image.size().height;
    
    mat_image_org_color = cv_image.clone();
    
    cvtColor(mat_image_org_color, mat_image_org_gray, COLOR_BGR2GRAY);
    
    Point points[4];
    points[0] = Point(img_width/2 - m_roi_width, m_roi_center - m_roi_height);
    points[1] = Point(img_width/2 - m_roi_width, m_roi_center + m_roi_height);
    points[2] = Point(img_width/2 + m_roi_width, m_roi_center + m_roi_height);
    points[3] = Point(img_width/2 + m_roi_width, m_roi_center - m_roi_height);
    
    //mat_image_roi = Region_of_Interest_crop(mat_image_org_gray, points);
    mat_image_roi = Region_of_Interest_crop(mat_image_org_color, points);
    
    mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);
    
    rectangle(mat_image_org_color, points[0], points[2], Scalar(0, 255, 0), 2);
    
    imshow("Original Image", mat_image_org_color);
    imshow("ROI", mat_image_roi);
    //imshow("Canny Edge", mat_image_canny_edge);
    
    waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_lane_detection_node");
    ros::NodeHandle nh;
    
    std::string camera_topic 				= "/camera/image_raw";
    
	ros::param::get("~camera_topic",		camera_topic);
	ros::param::get("~m_roi_center",		m_roi_center);
	ros::param::get("~m_roi_height",		m_roi_height);
	ros::param::get("~m_roi_width",			m_roi_width);
	ros::param::get("~img_width",			img_width);
	ros::param::get("~img_height",			img_height);
	
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(camera_topic, 10, imageCallback);
    
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
