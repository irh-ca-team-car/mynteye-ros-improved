// Main node for Mynteye
//

#include <mynteye/api/api.h>

#ifdef _WIN32

#pragma warning(push)
//Severity	Code	Description	Project	File	Line	Suppression State
//Error(active)	E0145	member "boost::chrono::system_clock::is_steady" may not be initialized	MyntEye_ROS	C : \opt\rosdeps\x64\include\boost - 1_66\boost\chrono\system_clocks.hpp	134
#pragma warning(disable \
				: E0415)
#pragma warning(disable : 0415)
//unkown attribute no_init_all in winnt.h
#pragma warning(disable \
				: E1097)
#pragma warning(disable : 1097)
//It's a bug in visual studio, the project compiles even with these errors
//DLL warning in mynteye API
#pragma warning(disable \
				: C4251)
#pragma warning(disable : 4251)
//An error caused by changing the project name
#pragma warning(disable \
				: MSB8028)
#pragma warning(disable : 8028)

// On windows, ROS need the Windows.h
#include "Windows.h"
#endif
#ifdef _DEBUG
#define ISDEBUG
#undef __MSVC_RUNTIME_CHECKS
#undef _DEBUG
#endif

#ifndef NDEBUG
#define NDEBUG
#endif
//ROS HEADERS CONTAINS DEBUG SYMBOLS BUT LIB FILE ARE NOT AVAILABLE ON WINDOWS

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>
#include <string>
#include <mutex>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#ifdef ISDEBUG
#define _DEBUG
#endif

#include "Matrix.hpp"

#ifdef _WIN32
#include <conio.h>
#endif
#define Deg2Rad(A) ((A) * (3.14 / 180))
#define Rad2Deg(A) ((A) * (180 / 3.14))
#define LEFT_IDX 0
#define RIGHT_IDX 1
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono;	  // nanoseconds, system_clock, seconds

double theta_x = 0, theta_y = 0, theta_z = 0;
std::string topic_left, topic_right, topic_imu, frame_id, left_frame_id, right_frame_id;

rclcpp::Node::SharedPtr n;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p[2];
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pimu;

using namespace mynteye;
using namespace cv;
using byte = unsigned char;
typedef cv::Point3_<uint8_t> Pixel;
void HandleCaptureAndROS(std::shared_ptr<mynteye::API> api);
void publishPoints(std::shared_ptr<mynteye::API> api, const mynteye::api::StreamData &data, cv::Mat left);
bool isFirst = true;
sensor_msgs::msg::Imu firstIMUmsg;
double currentSpeedX = 0, currentSpeedY = 0, currentSpeedZ = 0;
double currentLocationX = 0, currentLocationY = 0, currentLocationZ = 0;
bool initialized_ = false;
double constant_dt_;
rclcpp::Time lastTime_;

std::string to_string(double data) //Convert a mm value to a meter string
{
	return std::to_string(data / 1000) + "m";
}
//Store ros elements

//Send a cv::Mat over ROS
void send(int idx, Mat m, std::string format, std::string frame)
{
	cv_bridge::CvImage img;
	img.image = m;
	img.header.frame_id = frame;
	img.header.stamp = n->now();
	img.encoding = format;
	auto msg = img.toImageMsg();
	p[idx]->publish(*msg);
}
//Main loop
void HandleCaptureAndROS(std::shared_ptr<mynteye::API> api)
{
	//Do not enable Stream::POINTS since we create the point cloud using the depth map
	api->EnableMotionDatas();
    api->SetOptionValue(Option::ACCELEROMETER_RANGE, 32);
    // GYROSCOPE_RANGE values: 500, 1000, 2000, 4000
    api->SetOptionValue(Option::GYROSCOPE_RANGE, 4000);
    // FRAME_RATE values: 10, 15, 20, 25, 30, 35, 40, 45, 50, 55
    api->SetOptionValue(Option::FRAME_RATE, 60);
    // IMU_FREQUENCY values: 100, 200, 250, 333, 500
    api->SetOptionValue(Option::IMU_FREQUENCY, 500);
	
	//Store the left image to apply the correct gray on the point cloud
	cv::Mat left;

	std::mutex depth_mtx, left_mtx;
	
	RCLCPP_INFO(n->get_logger(), "Enabling MOTION stream");
	auto begin_time = n->now();
	api->SetMotionCallback([&begin_time](const api::MotionData &data) {
		if (pimu->get_subscription_count() > 0)
		{
			auto time = n->now();
			auto imu_msg = sensor_msgs::msg::Imu();
			imu_msg.header.frame_id = frame_id;
			imu_msg.header.stamp = time;
			//auto timeSecSinceLast = (time-begin_time).toSec();
			begin_time = time;
			auto gyro = geometry_msgs::msg::Vector3();
			auto acc = geometry_msgs::msg::Vector3();
			if (data.imu->gyro)
			{
				gyro.x = Deg2Rad(data.imu->gyro[2]);
				gyro.y = Deg2Rad(data.imu->gyro[0]);
				gyro.z = Deg2Rad(data.imu->gyro[1]);
				imu_msg.angular_velocity = gyro;
			}
			if (data.imu->accel)
			{
                //Mynt: Z forward, X left, Y up
                //ROS:x forwars, y lateral, z up
				acc.x = data.imu->accel[2] * 9.81;
				acc.y = data.imu->accel[0]* 9.81;
				acc.z = data.imu->accel[1]* 9.81;
				imu_msg.linear_acceleration = acc;
			}
			//Integrate the Gyro to obtain a lookin angle and location
			//SendAccelerationToTransform(msg, timeSecSinceLast);
			//Publish the IMU data to ROS
		
			// apply yaw offsets
			imu_msg.orientation.w = 1;
			imu_msg.orientation.x = 0;
			imu_msg.orientation.y = 0;
			imu_msg.orientation.z = 0;

			imu_msg.orientation_covariance[0] = 0;
			imu_msg.orientation_covariance[1] = 0.0;
			imu_msg.orientation_covariance[2] = 0.0;
			imu_msg.orientation_covariance[3] = 0.0;
			imu_msg.orientation_covariance[4] = 0;
			imu_msg.orientation_covariance[5] = 0.0;
			imu_msg.orientation_covariance[6] = 0.0;
			imu_msg.orientation_covariance[7] = 0.0;
			imu_msg.orientation_covariance[8] = 0;

			pimu->publish(imu_msg);
		}
		//Optionally print data from the IMU

		//double G = 9.81;
		//printf("IMU: Acc : x=%5.2lf m/s2 y=%5.2lf m/s2 z=%5.2lf m/s2 ", data.imu->accel[0] * G, data.imu->accel[1] * G, data.imu->accel[2] * G);
		//printf(" Gyro: x=%8.1lf d/s y=%8.1lf d/s z=%8.1lf d/s ", data.imu->gyro[0], data.imu->gyro[1], data.imu->gyro[2]);
	});
	RCLCPP_INFO(n->get_logger(), "MOTION stream ENABLED");
	RCLCPP_INFO(n->get_logger(), "Starting camera");
	api->Start(Source::ALL);
	RCLCPP_INFO(n->get_logger(), "Camera Started");
#ifdef _WIN32
	//Enable the following line to run as an invisible console in Windows
	//FreeConsole();
#endif
	//Loop until CTRL-C
	while (rclcpp::ok())
	{
		api->WaitForStreams();
		auto &&left_data = api->GetStreamData(Stream::LEFT);
		auto &&right_data = api->GetStreamData(Stream::RIGHT);

		left = left_data.frame;

		//Send left and right camera
		send(LEFT_IDX, left_data.frame, "mono8", left_frame_id);
		send(RIGHT_IDX, right_data.frame, "mono8", right_frame_id);

		char key = static_cast<char>(cv::waitKey(1));
		if (key == 27 || key == 'q' || key == 'Q')
		{ // ESC/Q
			break;
		}
		
	}
	RCLCPP_INFO(n->get_logger(), "Closing camera");
	api->Stop(Source::ALL);
	RCLCPP_INFO(n->get_logger(), "Camera closed");
}
int main(int argc, char *argv[])
{
	printf("Created empty map\r\n");
	rclcpp::init(argc, argv);

	printf("Ros has been initialized\r\n");
	n = std::make_shared<rclcpp::Node>("mynteye_ros");
	RCLCPP_INFO(n->get_logger(), "Obtained Node");

	topic_left = n->declare_parameter<std::string>("topic_left", "/mynteye/image/left");
	topic_right = n->declare_parameter<std::string>("topic_right", "/mynteye/image/right");
	topic_imu = n->declare_parameter<std::string>("topic_imu", "/mynteye/imu");
	frame_id = n->declare_parameter<std::string>("imu_frame_id", "mynteye_imu");
	left_frame_id = n->declare_parameter<std::string>("left_frame_id", "mynteye_left");
	right_frame_id = n->declare_parameter<std::string>("right_frame_id", "mynteye_right");

	p[LEFT_IDX] = n->create_publisher<sensor_msgs::msg::Image>(topic_left, 1);
	p[RIGHT_IDX] = n->create_publisher<sensor_msgs::msg::Image>(topic_right, 1);
	pimu = n->create_publisher<sensor_msgs::msg::Imu>(topic_imu, 100);

	RCLCPP_INFO(n->get_logger(), "Advertised topics");
	RCLCPP_INFO(n->get_logger(), "\t topic_left=%s", topic_left.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_right=%s", topic_right.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_imu=%s", topic_imu.c_str());
	RCLCPP_INFO(n->get_logger(), "\t frame_id=%s", frame_id.c_str());
	RCLCPP_INFO(n->get_logger(), "\t left_frame_id=%s", left_frame_id.c_str());
	RCLCPP_INFO(n->get_logger(), "\t right_frame_id=%s", right_frame_id.c_str());
	RCLCPP_INFO(n->get_logger(), "Obtaining MyntEYE API");
	auto api = API::Create(argc, argv);
	RCLCPP_INFO(n->get_logger(), "MyntEYE API obtained");

	if (api)
		HandleCaptureAndROS(api);
	else
		RCLCPP_ERROR(n->get_logger(), "There is no MyntEye camera to output");
}

#ifdef __WIN32
#pragma warning(pop)
#endif
