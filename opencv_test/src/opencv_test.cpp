#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

 
using namespace cv;
using namespace std;


class Ball_detect
{
	private:	
		cv_bridge::CvImagePtr cv_ptr,processed;	
		
	public:
		ros::Publisher image_out;
		int min_dist;
		int hough_thresh;
		int hough_c_thresh;
		int minRadius;
		int maxRadius;

	Ball_detect() //constructor 
	{
		min_dist=100;
		hough_thresh=500;
		hough_c_thresh=8;
		minRadius=10;
		maxRadius=70;	
		namedWindow("Color_scale");
		/// Create Trackbars for Manual HSV Filter Setup
 		createTrackbar("minDist","Color_scale",&min_dist,300,NULL);
 		createTrackbar("minRadius","Color_scale",&minRadius,100,NULL);
 		createTrackbar("maxRadius","Color_scale",&maxRadius,200,NULL);
 		createTrackbar("HoughCanny","Color_scale",&hough_thresh,3000,NULL);
 		createTrackbar("HoughCenter","Color_scale",&hough_c_thresh,20,NULL);	
 	}
		
	~Ball_detect() //destructor
	{
		destroyWindow("Color_scale");
	} 	
	
	void readImage(const sensor_msgs::Image::ConstPtr &msg)
	{
		//write callback topic  to private opencv variable
		try
		{ 
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			processed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);	
			
			processed = processImage(cv_ptr);
			image_out.publish(processed->toImageMsg());	
		}
			
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}	
	}
	
	cv_bridge::CvImagePtr processImage(cv_bridge::CvImagePtr input)
	{
		cv_bridge::CvImagePtr output=input;
		
		//GaussianBlur(input->image,output->image,Size(21,21),0);	
		//filter lab color space with l,a,b min max
		int a_min,a_max, l_min,l_max,b_min,b_max;
		l_min=1;
		a_min=150;
		b_min=1;
		a_max=255;
		l_max=255;
		b_max=255;
		
		Mat out,lab;
		lab = input->image.clone();
		//convert image to lab color space
		cvtColor(lab, lab, COLOR_BGR2Lab);	
		//create binary image out of lab limits	
		inRange(lab,Scalar(l_min,a_min,b_min), Scalar(l_max,a_max,b_max), out);
		// find all round object in the Binary Image
		vector<Vec3f> circles;
			
		HoughCircles(out,circles,CV_HOUGH_GRADIENT,1, min_dist,hough_thresh,hough_c_thresh,minRadius,maxRadius);	//Attention-> The Max r should not be too small	
		//draw circles into output image
		cvtColor(out, output->image, COLOR_GRAY2BGR);
		 for( size_t i = 0; i < circles.size(); i++ )
		  {
			  cout<<"ball detected!"<<endl;
			  Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			  int radius = cvRound(circles[i][2]);
			  // circle center
			  circle(output->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
			  // circle outline
			  circle(output->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
		   }
		  cout<<"new_image"<<endl;
		 imshow("Color_scale", output->image);
		waitKey(3);	
		
		return output;
	}
	
	

};


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "ball detector node");
			
	ros::NodeHandle n("~");
	Ball_detect object;	
	
	string image_raw,out;
	n.param<string>("image_raw",image_raw,"/raw");  
	n.param<string>("image_out",out,"/image"); 
	
	ros::Subscriber imageSub=n.subscribe(image_raw.c_str(),10,&Ball_detect::readImage,&object);
	
	object.image_out=n.advertise<sensor_msgs::Image>(out.c_str(),10);
	
	ros::spin();	
	return 0;	
	
}
