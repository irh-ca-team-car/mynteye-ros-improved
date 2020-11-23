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
#include "imu_filter_madgwick/imu_filter_ros.h"
#include "imu_filter_madgwick/stateless_orientation.h"
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

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono;	  // nanoseconds, system_clock, seconds

double theta_x = 0, theta_y = 0, theta_z = 0;
std::string topic_left, topic_right, topic_depth, topic_color_map, topic_imu, topic_point_cloud, frame_id, parent_frame_id;
ImuFilter filter_;
rclcpp::Node::SharedPtr n;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr p[4];
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pimu;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ppcl;
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

sensor_msgs::msg::Imu madgwick_filter(sensor_msgs::msg::Imu imu_msg_raw)
{
	const geometry_msgs::msg::Vector3 &ang_vel = imu_msg_raw.angular_velocity;
	const geometry_msgs::msg::Vector3 &lin_acc = imu_msg_raw.linear_acceleration;

	rclcpp::Clock steady_clock(RCL_STEADY_TIME); // for throttle logger message

	rclcpp::Time time = imu_msg_raw.header.stamp;
	std::string imu_frame_ = imu_msg_raw.header.frame_id;

	if (!initialized_)
	{
		filter_.setWorldFrame(WorldFrame::ENU);
		filter_.setAlgorithmGain(0.1);
  		filter_.setDriftBiasGain(0.0);
		geometry_msgs::msg::Quaternion init_q;
		if (!StatelessOrientation::computeOrientation(WorldFrame::ENU, lin_acc, init_q))
		{
			RCLCPP_WARN_THROTTLE(n->get_logger(), steady_clock, 5.0, "The IMU seems to be in free fall, cannot determine gravity direction!");
			return imu_msg_raw;
		}
		filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
	}

	if (!initialized_)
	{
		RCLCPP_INFO(n->get_logger(), "First IMU message received.");

		// initialize time
		lastTime_ = time;
		initialized_ = true;
	}

	// determine dt: either constant, or from IMU timestamp
	float dt;

	dt = (time - lastTime_).seconds();
	if (0 == time.nanoseconds())
	{
		RCLCPP_WARN_STREAM_THROTTLE(n->get_logger(), steady_clock, 5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!"
																			<< " The filter will not update the orientation.");
	}

	lastTime_ = time;

	filter_.madgwickAHRSupdateIMU(
		ang_vel.x, ang_vel.y, ang_vel.z,
		lin_acc.x, lin_acc.y, lin_acc.z,
		dt);

	return imu_msg_raw;
}

//Transform camera centric IMU to world centric IMU to calculate camera position
void SendAccelerationToTransform(sensor_msgs::msg::Imu &msg, double seconds)
{
	auto acc = msg.linear_acceleration;
	auto gyro = msg.angular_velocity;

	theta_x += (gyro.x * seconds);
	theta_y += (gyro.y * seconds);
	theta_z += (gyro.z * seconds);

	auto rx = (theta_x);
	auto ry = (theta_y);
	auto rz = (theta_z);

	CMatrix A = CMatrix(4, 1);
	A(0, 0) = acc.x;
	A(1, 0) = acc.y;
	A(2, 0) = acc.z;
	A(3, 0) = 1;
	CMatrix R = CMatrix(4, 4);

	R(0, 0) = std::cos(rx) * std::cos(rz);
	R(1, 0) = std::cos(ry) * std::sin(rz);
	R(2, 0) = -std::sin(ry);

	R(0, 1) = std::sin(rx) * std::sin(ry) * std::cos(rz) -
			  std::cos(rx) * std::sin(rz);
	R(1, 1) = std::sin(rx) * std::sin(ry) * std::sin(rz) -
			  std::cos(rx) * std::cos(rz);
	R(2, 1) = std::sin(rx) * std::cos(ry);

	R(0, 2) = std::cos(rx) * std::sin(ry) * std::sin(rz) -
			  std::sin(rx) * std::cos(rz);
	R(1, 2) = std::cos(rx) * std::sin(ry) * std::sin(rz) -
			  std::sin(rx) * std::cos(rz);
	R(2, 2) = std::cos(rx) * std::cos(ry);

	R(3, 3) = 1;

	if (isFirst) //First frame is gravity, no movement
	{
		firstIMUmsg = msg;
		isFirst = false;
	}
	CMatrix g = CMatrix(4, 1);

	g(0, 0) = firstIMUmsg.linear_acceleration.x;
	g(1, 0) = firstIMUmsg.linear_acceleration.y;
	g(2, 0) = firstIMUmsg.linear_acceleration.z;

	g(3, 0) = 1;

	//Rotate acceleration and remove gravity
	CMatrix AF = R * A + -g;

	auto AccX = AF(0, 0);
	auto AccY = AF(1, 0);
	auto AccZ = AF(2, 0);

	double dt = seconds;

	//Calculate change in location

	double instantXSpeed = AccX * dt;

	auto dx = currentSpeedX * dt + AccX * dt * dt / 2;
	currentSpeedX += instantXSpeed;

	double instantYSpeed = AccY * dt;

	auto dy = currentSpeedY * dt + AccY * dt * dt / 2;
	currentSpeedY += instantYSpeed;

	double instantZSpeed = AccZ * dt;

	auto dz = currentSpeedZ * dt + AccZ * dt * dt / 2;
	currentSpeedZ += instantZSpeed;

	//Calculate new location
	currentLocationX += (dx);
	currentLocationY += (dy);
	currentLocationZ += (dz);

	//auto gy = geometry_msgs::Quaternion();

	//gy.x = sin(ry)*sin(rz)*cos(rx) + cos(ry)*cos(rz)*sin(rx);
	//gy.y = sin(ry)*cos(rz)*cos(rx) + cos(ry)*sin(rz)*sin(rx);
	//gy.z = cos(ry)*sin(rz)*cos(rx) - sin(ry)*cos(rz)*sin(rx);
	//gy.w = cos(ry)*cos(rz)*cos(rx) - sin(ry)*sin(rz)*sin(rx);

	//msg.orientation = gy;
}
double average(Mat frame) //Return the average distance in mm
{
	double d = 0;

	int max = frame.cols * frame.rows;
	int ct = 0;
	ushort *dataPtr = (ushort *)(frame.data);
	for (int i = 0; i < max; i++)
	{
		if (*dataPtr != 0)
			ct++;
		d += *dataPtr++;
	}
	if (ct > 0)
		d /= ct;
	return d;
}

struct Operator //Remove objects that are less than 0.1m from the camera since it is mostly false detections
{
	void operator()(ushort &pixel, const int *position) const
	{
		position++;
		if (pixel < 100)
			pixel = UINT16_MAX;
	}
};
struct OperatorRange //Apply a range over a cv mat
{
public:
	OperatorRange(ushort a, ushort b)
	{
		mi = a;
		ma = b;
		r = ma - mi;
		//if (r < 255 * 25)
		//	r = 255 * 25;

		//if (mi + r > UINT16_MAX)
		//	mi = UINT16_MAX - r;
	}
	void operator()(ushort &pixel, const int *position) const
	{
		position++;
		if (pixel > ma) //If over max, set it to max
		{
			pixel = UINT16_MAX;
			return;
		}
		if (pixel < mi) //If less than min set it to zero
		{
			pixel = 0;
			return;
		}
		pixel = (ushort)((pixel - mi) / r * UINT16_MAX); //Uniformely space
	}

private:
	double mi, ma;
	double r;
};

std::string to_string(double data) //Convert a mm value to a meter string
{
	return std::to_string(data / 1000) + "m";
}
//Store ros elements

//ros::Publisher* p[6];
#define DEPTH_IDX 1
#define DEPTH_CM_IDX 3
#define LEFT_IDX 0
#define RIGHT_IDX 2
//Find range of mat
void analyzeRange(Mat m, ushort *min, ushort *max)
{
	ushort lmin = UINT16_MAX;
	ushort lmax = 0;
	for (auto x = 0; x < m.cols; x++)
		for (auto y = 0; y < m.rows; y++)
		{
			ushort pixel = m.at<ushort>(y, x);
			if (pixel > 100)
			{
				if (pixel > lmax)
					lmax = pixel;
				if (pixel < lmin)
					lmin = pixel;
			}
		}
	if (min)
		*min = lmin;
	if (max)
		*max = lmax;
}
//Send a cv::Mat over ROS
void send(int idx, Mat m, std::string format)
{
	cv_bridge::CvImage img;
	img.image = m;
	img.header.frame_id = frame_id;
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
	api->EnableStreamData(Stream::DEPTH);
	//Some debug messages when we start the node
	RCLCPP_INFO(n->get_logger(), "Enabling DEPTH stream");

	//Store the left image to apply the correct gray on the point cloud
	cv::Mat left;

	std::mutex depth_mtx, left_mtx;
	RCLCPP_INFO(n->get_logger(), "Enabling DEPTH processing");
	api->SetStreamCallback(
		Stream::DEPTH,
		[&api, &depth_mtx, &left](const api::StreamData &data) {
			std::lock_guard<std::mutex> _(depth_mtx);

			auto mat = data.frame;
			//Generate the point cloud
			publishPoints(api, data, left);
			//Send /image/depth/raw

			send(DEPTH_IDX, mat, "mono16");
			//Enable this line if you desire to disable colormap when no subscribers
			//if (p[DEPTH_CM_IDX]->getNumSubscribers() > 0)
			{
				Mat mat2;

				ushort min, max;
				analyzeRange(mat, &min, &max);
				mat.forEach<ushort>(Operator());
				mat.forEach<ushort>(OperatorRange(min, max));

				cv::convertScaleAbs(mat, mat2, 1 / 255.0, 0);

				applyColorMap(mat2, mat2, COLORMAP_JET);
				//0m = white
				//16m= black

				send(DEPTH_CM_IDX, mat2, "bgr8");
			}
		});
	RCLCPP_INFO(n->get_logger(), "DEPTH processing ENABLED");
	RCLCPP_INFO(n->get_logger(), "Enabling MOTION stream");
	auto begin_time = n->now();
	api->SetMotionCallback([&begin_time](const api::MotionData &data) {
		if (pimu->get_subscription_count() > 0)
		{
			auto time = n->now();
			auto msg = sensor_msgs::msg::Imu();
			msg.header.frame_id = frame_id;
			msg.header.stamp = time;
			//auto timeSecSinceLast = (time-begin_time).toSec();
			begin_time = time;
			auto gyro = geometry_msgs::msg::Vector3();
			auto acc = geometry_msgs::msg::Vector3();
			if (data.imu->gyro)
			{
				gyro.x = Deg2Rad(data.imu->gyro[0]);
				gyro.y = Deg2Rad(data.imu->gyro[1]);
				gyro.z = Deg2Rad(data.imu->gyro[2]);
				msg.angular_velocity = gyro;
			}
			if (data.imu->accel)
			{
				acc.x = data.imu->accel[0];
				acc.y = data.imu->accel[1];
				acc.z = data.imu->accel[2];
				msg.linear_acceleration = acc;
			}
			//Integrate the Gyro to obtain a lookin angle and location
			//SendAccelerationToTransform(msg, timeSecSinceLast);
			//Publish the IMU data to ROS
			msg = madgwick_filter(msg);
			sensor_msgs::msg::Imu imu_msg = msg;
			double q0, q1, q2, q3;
			filter_.getOrientation(q0, q1, q2, q3);
			// apply yaw offsets
			imu_msg.orientation.w = q0;
			imu_msg.orientation.x = q1;
			imu_msg.orientation.y = q2;
			imu_msg.orientation.z = q3;

			imu_msg.orientation_covariance[0] = 0;
			imu_msg.orientation_covariance[1] = 0.0;
			imu_msg.orientation_covariance[2] = 0.0;
			imu_msg.orientation_covariance[3] = 0.0;
			imu_msg.orientation_covariance[4] = 0;
			imu_msg.orientation_covariance[5] = 0.0;
			imu_msg.orientation_covariance[6] = 0.0;
			imu_msg.orientation_covariance[7] = 0.0;
			imu_msg.orientation_covariance[8] = 0;

			if (true)
			{
				float gx, gy, gz;
				filter_.getGravity(gx, gy, gz);
				imu_msg.linear_acceleration.x -= gx;
				imu_msg.linear_acceleration.y -= gy;
				imu_msg.linear_acceleration.z -= gz;
			}
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
		send(LEFT_IDX, left_data.frame, "mono8");
		send(RIGHT_IDX, right_data.frame, "mono8");

		char key = static_cast<char>(cv::waitKey(1));
		if (key == 27 || key == 'q' || key == 'Q')
		{ // ESC/Q
			break;
		}
		//Send a Transform calculated based on the integration of the Gyro
		//On windows it should be zero since MyntEYE get a fatal error on enabling the IMU in version 1909
		geometry_msgs::msg::TransformStamped static_transformStamped;

		static_transformStamped.header.stamp = n->now();
		static_transformStamped.header.frame_id = parent_frame_id;
		static_transformStamped.child_frame_id = frame_id;
		static_transformStamped.transform.translation.x = 0;
		static_transformStamped.transform.translation.y = 0;
		static_transformStamped.transform.translation.z = 0;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, -theta_x); //roll pitch yaw
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();

		/*auto gy = geometry_msgs::Quaternion();
		auto rz = theta_z;
		auto rx = theta_y;
		auto ry = theta_y;
		gy.x = sin(ry)*sin(rz)*cos(rx) + cos(ry)*cos(rz)*sin(rx);
		gy.y = sin(ry)*cos(rz)*cos(rx) + cos(ry)*sin(rz)*sin(rx);
		gy.z = cos(ry)*sin(rz)*cos(rx) - sin(ry)*cos(rz)*sin(rx);
		gy.w = cos(ry)*cos(rz)*cos(rx) - sin(ry)*sin(rz)*sin(rx);

		static_transformStamped.transform.rotation = gy;*/
		tf2_ros::TransformBroadcaster brod(n);
		brod.sendTransform(static_transformStamped);
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
	topic_depth = n->declare_parameter<std::string>("topic_depth", "/mynteye/image/depth");
	topic_color_map = n->declare_parameter<std::string>("topic_color_map", "/mynteye/image/depth/color_map");
	topic_imu = n->declare_parameter<std::string>("topic_imu", "/mynteye/imu");
	topic_point_cloud = n->declare_parameter<std::string>("topic_point_cloud", "/mynteye/image/point_cloud");
	frame_id = n->declare_parameter<std::string>("frame_id", "mynteye");
	parent_frame_id = n->declare_parameter<std::string>("parent_frame_id", "map");

	p[LEFT_IDX] = n->create_publisher<sensor_msgs::msg::Image>(topic_left, 1);
	p[DEPTH_IDX] = n->create_publisher<sensor_msgs::msg::Image>(topic_depth, 1);
	p[RIGHT_IDX] = n->create_publisher<sensor_msgs::msg::Image>(topic_right, 1);
	p[DEPTH_CM_IDX] = n->create_publisher<sensor_msgs::msg::Image>(topic_color_map, 1);
	pimu = n->create_publisher<sensor_msgs::msg::Imu>(topic_imu, 100);
	ppcl = n->create_publisher<sensor_msgs::msg::PointCloud2>(topic_point_cloud, 1);

	RCLCPP_INFO(n->get_logger(), "Advertised topics");
	RCLCPP_INFO(n->get_logger(), "\t topic_left=%s", topic_left.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_right=%s", topic_right.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_depth=%s", topic_depth.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_color_map=%s", topic_color_map.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_imu=%s", topic_imu.c_str());
	RCLCPP_INFO(n->get_logger(), "\t topic_point_cloud=%s", topic_point_cloud.c_str());
	RCLCPP_INFO(n->get_logger(), "\t frame_id=%s", frame_id.c_str());
	RCLCPP_INFO(n->get_logger(), "\t parent_frame_id=%s", parent_frame_id.c_str());
	RCLCPP_INFO(n->get_logger(), "Obtaining MyntEYE API");
	auto api = API::Create(argc, argv);

	if (api)
		HandleCaptureAndROS(api);
	else
		RCLCPP_ERROR(n->get_logger(), "There is no MyntEye camera to output");
}
int nonZero(const cv::Mat m) //Calculate the ammount of non zero point in image to resize the point cloud
{
	int c = 0;
	ushort *ptr = (ushort *)m.data;
	for (int y = 0; y < m.rows; ++y)
	{
		for (int x = 0; x < m.cols; ++x)
		{
			if (*ptr++ != 0)
				c++;
		}
	}
	return c;
}
#define PI 3.14159265
void publishPoints(std::shared_ptr<mynteye::API> api,
				   const api::StreamData &data, cv::Mat left)
{
	//Enable the following line to enable point cloud only if there is a subscriber, it will reduce CPU load
	if (ppcl->get_subscription_count())
	{
		auto &&in = api->GetIntrinsicsBase(Stream::LEFT);

		int count = 0;
		cv::Mat m = data.frame;
		ushort min, max;
		analyzeRange(m, &min, &max);
		//Count non zero points
		ushort *ptr = (ushort *)m.data;
		for (int y = 0; y < m.rows; ++y)
		{
			for (int x = 0; x < m.cols; ++x)
			{
				auto point = *ptr++;
				if (point != 0)
					count++;
			}
		}
		auto time = n->now();
		sensor_msgs::msg::PointCloud2 msg;
		msg.header.stamp = time;
		msg.header.frame_id = frame_id;

		msg.width = count;
		msg.height = 1;
		msg.is_dense = true;

		sensor_msgs::PointCloud2Modifier modifier(msg);

		modifier.setPointCloud2Fields(
			4,
			"x", 1, sensor_msgs::msg::PointField::FLOAT32,
			"y", 1, sensor_msgs::msg::PointField::FLOAT32,
			"z", 1, sensor_msgs::msg::PointField::FLOAT32,
			"m", 1, sensor_msgs::msg::PointField::UINT8);

		modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

		sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

		//We were not able to find a way to send gray so we send as RGB8
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

		ptr = (ushort *)m.data;

		//Field of view of Mynteye S camera, adapt to your camera
		float FOV_H = 122;
		float FOV_V = 76;

		byte *color = left.data;

		for (int y = 0; y < m.rows; ++y)
		{
			for (int x = 0; x < m.cols; ++x)
			{
				auto point = *ptr++;
				auto clr = *color++;
				if (point != 0) //for each non zero point
				{
#define ALIGN(A)
#define CENTER(FOV) -FOV * 0.5
					float h_theta = (FOV_H * (x / (float)m.cols) ALIGN(CENTER(FOV_H))) * PI / 180.0;
					float v_theta = (FOV_V * ((m.rows - y) / (float)m.rows - 0.5f) ALIGN(CENTER(FOV_V))) * PI / 180.0;
					float yaw = h_theta;
					float pitch = v_theta;

					//Calculate the 3d point from a 2d depth map using camera FOV
					float x = sin(yaw) * cos(pitch) * point / 1000;
					float y = cos(yaw) * cos(pitch) * point / 1000;
					float z = sin(pitch) * point / 1000;

					*iter_x = x;
					*iter_y = y;
					*iter_z = z;

					*iter_r = static_cast<uint8_t>(clr);
					*iter_g = static_cast<uint8_t>(clr);
					*iter_b = static_cast<uint8_t>(clr);

					++iter_x;
					++iter_y;
					++iter_z;
					++iter_r;
					++iter_g;
					++iter_b;
				}
			}
		}
		//Publish the generated point cloud
		ppcl->publish(msg);
	}
}
#ifdef __WIN32
#pragma warning(pop)
#endif