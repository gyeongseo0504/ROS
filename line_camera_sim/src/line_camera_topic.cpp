#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>

#include <math.h>


using namespace cv; 
using namespace std; 


#define ROI_CENTER_Y  120
#define ROI_WIDTH     10

#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

Mat cv_image;

int     img_width  = 320; 
int     img_height = 240; 
int     tsl1401cl_size = 320;
double  *tsl11401cl_data;


Mat mat_image_org_color;
Mat mat_image_org_gray;
Mat mat_image_roi;


struct Rect_Region
{
    int left;
	int right;
    int top;
	int bottom;		
};


struct Rect_Region ROI_line_center;


Mat Region_of_Interest(Mat image, Point *points)
{
  Mat img_mask =Mat::zeros(image.rows,image.cols,CV_8UC1);	 
  
  Scalar mask_color = Scalar(255,255,255);
  const Point* pt[1]={ points };	    
  int npt[] = { 4 };
  cv::fillPoly(img_mask,pt,npt,1,Scalar(255,255,255),LINE_8);
  Mat masked_img;	
  bitwise_and(image,img_mask,masked_img);
  
  return masked_img;
}

Mat Region_of_Interest_crop(Mat image, Point *points)
{
   Mat img_roi_crop;	

   Rect bounds(0,0,image.cols,image.rows);	 
   Rect r(points[0].x, points[0].y, points[2].x-points[0].x, points[2].y-points[0].y);    
 
   img_roi_crop = image(r & bounds);
   
   return img_roi_crop;
}

double *TSC1401CL(void)
{
	Point points[4];  // ROI(Region of Interest) 
	static double data[320] = {0.0, };
	
	int i,j;
	ROI_line_center.top    = ROI_CENTER_Y - ROI_WIDTH;
	ROI_line_center.bottom = ROI_CENTER_Y + ROI_WIDTH;
	ROI_line_center.left   = 0;
	ROI_line_center.right  = img_width;
	points[0] = Point(ROI_line_center.left  , ROI_line_center.top);
	points[1] = Point(ROI_line_center.left  , ROI_line_center.bottom);
	points[2] = Point(ROI_line_center.right , ROI_line_center.bottom);
	points[3] = Point(ROI_line_center.right , ROI_line_center.top);

	// color to gray conversion  
	cvtColor(mat_image_org_color, mat_image_org_gray, cv::COLOR_RGB2GRAY);
	resize(mat_image_org_gray, mat_image_org_gray, Size(img_width, img_height));

	// ROI 영역 추출      
	mat_image_roi = Region_of_Interest_crop(mat_image_org_gray,points);

	cv::imshow("imge", mat_image_roi);
	cv::waitKey(30);
	for(i = ROI_line_center.left ; i < ROI_line_center.right ; i++)
	{
		for(j = 0 ; j < ROI_WIDTH*2 ; j++)
		{
			
			data[i] += mat_image_roi.at<uchar>(j, i);
		}
		data[i] /= ROI_WIDTH*2;
	}
    
    /*
	for(i = ROI_line_center.left ; i < ROI_line_center.right ; i++)
	{
		printf("%3d : %6.3lf \n", i,data[i]);
	}
	*/
	//printf("size of array %ld \n", sizeof(data)/sizeof(double));
	return data;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
	cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

	img_width = cv_image.size().width ;
	img_height = cv_image.size().height;

	mat_image_org_color = cv_image.clone(); 
	
	tsl11401cl_data = TSC1401CL();
}



int main(int argc, char **argv)
{
	Point points[4];  // ROI(Region of Interest) 

	ros::init(argc, argv, "tsl1401cl_camera");
	ros::NodeHandle nh;

	ros::Publisher pub_tsl1401cl_pub = nh.advertise<std_msgs::Float32MultiArray>("/tsl1401cl", 10);

	ros::Duration(0.5).sleep();   // sleep for 0.5 senods

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1000, imageCallback);
	tsl11401cl_data = new double[320];
	
	printf("tsl1401cl_camera start!\n");
	ros::Rate rate(30);
	while (ros::ok())
	{	
			 
		std_msgs::Float32MultiArray data_msg;
		for(int i = ROI_line_center.left ; i < ROI_line_center.right ; i++)
		{
		//	printf("%3d : %6.3lf \n", i,tsl11401cl_data[i]);
			data_msg.data.push_back(tsl11401cl_data[i]);
			
		}
		
		pub_tsl1401cl_pub.publish(data_msg);
		
		
		ros::spinOnce();
		rate.sleep();       
	}
	delete []tsl11401cl_data;
	
}





