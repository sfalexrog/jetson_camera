#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
	return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
	       std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
	       "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
	       std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "jetson_camera");
	ros::NodeHandle nh, nh_priv("~");
	image_transport::ImageTransport it(nh);
	image_transport::ImageTransport it_priv(nh_priv);

	int cap_width = nh_priv.param<int>("cap_width", 1280);
	int cap_height = nh_priv.param<int>("cap_height", 720);

	int width = nh_priv.param<int>("width", 640);
	int height = nh_priv.param<int>("height", 480);
	int framerate = nh_priv.param<int>("fps", 60);

	int flip_method = nh_priv.param<int>("flip_method", 0);

	std::string frame_id = nh_priv.param<std::string>("frame_id", "main_camera_optical");
	double delay = nh_priv.param<double>("capture_delay", 0.0);

	std::string topic_name = nh_priv.param<std::string>("topic_name", "image_raw");

	std::string url = nh_priv.param<std::string>("camera_info_url", "");

	camera_info_manager::CameraInfoManager info_manager(nh, "camera", url);

	ROS_INFO("jetson_camera: Starting node with the following parameters:\n"\
	         "capture width: %d, capture height: %d\n"\
	         "published width: %d, published height: %d\n"\
	         "requested framerate: %d, flip method: %d\n",
	         cap_width, cap_height, width, height, framerate, flip_method);

	auto pipeline = gstreamer_pipeline(cap_width, cap_height, width, height, framerate, flip_method);

	ROS_INFO("jetson_camera: Creating cv capture with the following pipeline: %s", pipeline.c_str());
	cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

	auto pub = it_priv.advertiseCamera(topic_name, 1);

	ROS_INFO("jetson_camera: Showtime");
	cv_bridge::CvImage img;
	
	img.encoding = sensor_msgs::image_encodings::BGR8;
	img.header.frame_id = frame_id;
	auto cam_info = info_manager.getCameraInfo();
	cam_info.header.frame_id = frame_id;

	double width_coeff = static_cast<double>(width) / cam_info.width;
	double height_coeff = static_cast<double>(height) / cam_info.height;
	cam_info.width = width;
	cam_info.height = height;

	cam_info.K[0] *= width_coeff;
	cam_info.K[2] *= width_coeff;
	cam_info.K[4] *= height_coeff;
	cam_info.K[5] *= height_coeff;

	cam_info.P[0] *= width_coeff;
	cam_info.P[2] *= width_coeff;
	cam_info.P[5] *= height_coeff;
	cam_info.P[6] *= height_coeff;

	while(!ros::isShuttingDown())
	{
		if (cap.read(img.image))
		{
			img.header.stamp = ros::Time::now() - ros::Duration(delay);
			cam_info.header.stamp = img.header.stamp;
			pub.publish(*img.toImageMsg(), cam_info);
		}
	}
}
