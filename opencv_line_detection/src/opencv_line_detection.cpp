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

#define DEG2RAD(x) ((M_PI / 180.0) * (x))
#define RAD2DEG(x) ((180.0 / M_PI) * (x))

Mat cv_image;
int img_width = 320;
int img_height = 240;

Mat mat_image_org_color;
Mat mat_image_org_gray;
Mat mat_image_roi;
Mat mat_image_canny_edge;

int m_roi_center = 120;
int m_roi_height = 20;
int m_roi_width = 320;

struct Rect_Region {
    int left;
    int right;
    int top;
    int bottom;
};

Mat Region_of_Interest_crop(Mat image, Point* points) {
    Mat img_roi_crop;
    Rect bounds(0, 0, image.cols, image.rows);
    Rect r(points[0].x, points[0].y, points[2].x - points[0].x, points[2].y - points[0].y);
    img_roi_crop = image(r & bounds); 
    return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat img) {
    Mat mat_blur_img, mat_canny_img;
    blur(img, mat_blur_img, Size(3, 3));
    Canny(mat_blur_img, mat_canny_img, 70, 150, 3);
    return mat_canny_img;
}

void detectLines(Mat& canny_img, vector<Vec4i>& lines) {
    HoughLinesP(canny_img, lines, 1, CV_PI / 180, 50, 50, 10);
}

void lane_detection(Mat& img, Mat& mat_image_canny_edge, Mat& mat_image_roi) {
    Point points[4];
    points[0] = Point(img.cols / 2 - m_roi_width, m_roi_center - m_roi_height);
    points[1] = Point(img.cols / 2 - m_roi_width, m_roi_center + m_roi_height);
    points[2] = Point(img.cols / 2 + m_roi_width, m_roi_center + m_roi_height);
    points[3] = Point(img.cols / 2 + m_roi_width, m_roi_center - m_roi_height);

    rectangle(img, points[0], points[2], Scalar(0, 255, 0), 2);

    mat_image_roi = Region_of_Interest_crop(img, points);

    mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);

    vector<Vec4i> lines;
    detectLines(mat_image_canny_edge, lines);

    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(mat_image_roi, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }

    imshow("ROI", mat_image_roi);
    imshow("Canny Edge", mat_image_canny_edge);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_image = cv_ptr->image;

        img_width = cv_image.size().width;
        img_height = cv_image.size().height;

        mat_image_org_color = cv_image.clone();

        cvtColor(mat_image_org_color, mat_image_org_gray, COLOR_BGR2GRAY);

        mat_image_canny_edge = Mat::zeros(cv_image.size(), CV_8UC1);
        mat_image_roi = Mat::zeros(cv_image.size(), CV_8UC3);

        lane_detection(mat_image_org_color, mat_image_canny_edge, mat_image_roi);

        // 원본 이미지와 차선 검출 결과를 출력
        imshow("Original Image", mat_image_org_color);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "opencv_lane_detection_node");
    ros::NodeHandle nh;

    std::string camera_topic = "/camera/image_raw";

    ros::param::get("~camera_topic", camera_topic);
    ros::param::get("~m_roi_center", m_roi_center);
    ros::param::get("~m_roi_height", m_roi_height);
    ros::param::get("~m_roi_width", m_roi_width);
    ros::param::get("~img_width", img_width);
    ros::param::get("~img_height", img_height);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(camera_topic, 10, imageCallback);

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
