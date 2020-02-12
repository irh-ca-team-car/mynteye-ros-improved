// MyntEyeDemo2017.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include<mynteye/api/api.h>
#ifdef _WIN32
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
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#ifdef ISDEBUG
#define _DEBUG
#endif

#ifdef _WIN32
#include <conio.h>
#endif

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

using namespace mynteye;
using namespace cv;
using byte = unsigned char;
typedef cv::Point3_<uint8_t> Pixel;
void publishPoints(std::shared_ptr<mynteye::API> api, const mynteye::api::StreamData &data, cv::Mat left, std::uint32_t seq);

double average(Mat frame)
{
	double d = 0;

	int max = frame.cols*frame.rows;
	int ct = 0;
	ushort* dataPtr = (ushort*)(frame.data);
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

struct Operator
{
	void operator ()(ushort &pixel, const int * position) const
	{
		if (pixel < 100)
			pixel = UINT16_MAX;
	}
};
struct OperatorRange
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
	void operator ()(ushort &pixel, const int * position) const
	{
		if (pixel > ma)
		{
			pixel = UINT16_MAX;
			return;
		}
		if (pixel < mi)
		{
			pixel = 0;
			return;
		}
		pixel = (ushort)((pixel - mi) / r * UINT16_MAX);
	}
private:
	double mi, ma;
	double r;
};
std::string to_string(double data)
{
	return std::to_string(data / 1000) + "m";
}
ros::NodeHandle* n;
ros::Publisher* p[6];
#define DEPTH_IDX 1
#define DEPTH_CM_IDX 3
#define LEFT_IDX 0
#define RIGHT_IDX 2
#define IMU_IDX 4
#define PCL_IDX 5

void analyzeRange(Mat m, ushort* min, ushort* max)
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

void send(int idx, Mat m, std::string format)
{
	cv_bridge::CvImage img;
	img.image = m;
	img.encoding = format;
	auto msg = img.toImageMsg();
	p[idx]->publish(msg);
}
void HandleCaptureAndROS(std::shared_ptr<mynteye::API> api)
{
	api->EnableMotionDatas();
	api->EnableStreamData(Stream::DEPTH);

	printf("Enabling DEPTH stream\r\n");

	//auto stream_intrinsics = api->GetIntrinsics(Stream::POINTS);

	//PCViewer pcviewer(stream_intrinsics,1000);
	//cv::namedWindow("frame");

	std::atomic_uint depth_count(0);

	cv::Mat left;

	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	geometry_msgs::TransformStamped static_transformStamped;

	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = "map";
	static_transformStamped.child_frame_id = "mynteye";
	static_transformStamped.transform.translation.x = 0;
	static_transformStamped.transform.translation.y = 0;
	static_transformStamped.transform.translation.z = 0;
	tf2::Quaternion quat;
	quat.setRPY(0,0,0);
	static_transformStamped.transform.rotation.x = quat.x();
	static_transformStamped.transform.rotation.y = quat.y();
	static_transformStamped.transform.rotation.z = quat.z();
	static_transformStamped.transform.rotation.w = quat.w();
	static_broadcaster.sendTransform(static_transformStamped);

	std::mutex depth_mtx, left_mtx;
	printf("Enabling DEPTH processing\r\n");
	api->SetStreamCallback(
		Stream::DEPTH,
		[&api,&depth_mtx,&left](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(depth_mtx);

		auto mat = data.frame;
		publishPoints(api, data,left, 0);

		send(DEPTH_IDX, mat, "mono16");
		if (p[DEPTH_CM_IDX]->getNumSubscribers() > 0)
		{
			Mat mat2;
			double avgDepthMM;
			auto str = to_string(avgDepthMM = average(mat));
			//mat.convertTo(mat, CV_8UC1, 1);
			//std::cout << str << std::endl;

			ushort min, max;
			analyzeRange(mat, &min, &max);
			mat.forEach<ushort>(Operator());
			mat.forEach<ushort>(OperatorRange(min, max));
			//mat = UINT16_MAX - mat;//0.8m = 800

			cv::convertScaleAbs(mat, mat2, 1 / 255.0, 0);

			applyColorMap(mat2, mat2, COLORMAP_JET);
			//0m = white
			//16m= black

			send(DEPTH_CM_IDX, mat2, "bgr8");
		}
		//cv::putText(mat2, str, Point(0, 20), FONT_HERSHEY_COMPLEX, 1, Scalar(INT8_MAX, INT8_MAX, INT8_MAX, INT8_MAX), 3, 8, false);
		//		
		//cv::imshow("depth", mat2); // CV_16UC1

	});
	printf("DEPTH processing ENABLED\r\n");
	printf("Enabling MOTION stream\r\n");
	api->SetMotionCallback([](const api::MotionData & data) {
		if (p[IMU_IDX]->getNumSubscribers() > 0)
		{
			auto msg = sensor_msgs::Imu();
			msg.header.frame_id = "map";
			msg.header.seq = 0;

			auto gyro = geometry_msgs::Vector3();
			auto acc = geometry_msgs::Vector3();
			if (data.imu->gyro)
			{
				gyro.x = data.imu->gyro[0];
				gyro.y = data.imu->gyro[1];
				gyro.z = data.imu->gyro[2];
				msg.angular_velocity = gyro;
			}
			if (data.imu->accel)
			{
				acc.x = data.imu->accel[0];
				acc.y = data.imu->accel[1];
				acc.z = data.imu->accel[2];
				msg.linear_acceleration = acc;
			}

			p[IMU_IDX]->publish(msg);
		}
		//double G = 9.81;
		//printf("IMU: Acc : x=%5.2lf m/s2 y=%5.2lf m/s2 z=%5.2lf m/s2 ", data.imu->accel[0] * G, data.imu->accel[1] * G, data.imu->accel[2] * G);
		//printf(" Gyro: x=%8.1lf d/s y=%8.1lf d/s z=%8.1lf d/s ", data.imu->gyro[0], data.imu->gyro[1], data.imu->gyro[2]);
		//printf(" DEPTH: %5.2lf M\r\n", avgDepthMM / 1000);
	});
	printf("MOTION stream ENABLED\r\n");
	printf("Starting camera\r\n");
	api->Start(Source::ALL);
	printf("Camera Started\r\n");
#ifdef _WIN32
	//FreeConsole();
#endif
	//CVPainter painter(0);

	while (ros::ok()) {
		api->WaitForStreams();
		auto &&left_data = api->GetStreamData(Stream::LEFT);
		auto &&right_data = api->GetStreamData(Stream::RIGHT);

		left = left_data.frame;

		send(LEFT_IDX, left_data.frame, "mono8");
		send(RIGHT_IDX, right_data.frame, "mono8");
		
		char key = static_cast<char>(cv::waitKey(1));
		if (key == 27 || key == 'q' || key == 'Q') { // ESC/Q
			break;
		}
	}
	printf("Closing camera\r\n");
	api->Stop(Source::ALL);
	printf("Camera closed\r\n");
}

int main(int argc, char* argv[])
{
	std::map<std::string, std::string> emptyMap;
	printf("Created empty map\r\n");
	ros::init(emptyMap, "mynteye_win", ros::init_options::AnonymousName);
	printf("Ros has been initialized\r\n");
	ros::NodeHandle nl;
	printf("Obtained NodeHandle\r\n");
	ros::Publisher pl= nl.advertise<sensor_msgs::Image>("image/left", 1);
	ros::Publisher pd = nl.advertise<sensor_msgs::Image>("image/depth", 1);
	ros::Publisher pdc = nl.advertise<sensor_msgs::Image>("image/depth/color_map", 1);
	ros::Publisher pr = nl.advertise<sensor_msgs::Image>("image/right", 1);
	ros::Publisher pimu = nl.advertise<sensor_msgs::Imu>("mynteye/imu", 100);
	ros::Publisher ppc = nl.advertise<sensor_msgs::PointCloud2>("image/point_cloud", 1);
	printf("Advertised topics\r\n");

	p[LEFT_IDX] = &pl;
	p[DEPTH_IDX] = &pd;
	p[RIGHT_IDX] = &pr;
	p[DEPTH_CM_IDX] = &pdc;
	p[IMU_IDX] = &pimu;
	p[PCL_IDX] = &ppc;

	printf("Obtaining MyntEYE API\r\n");
	auto api = API::Create(argc, argv);
	
	if (api)
		HandleCaptureAndROS(api);
	else
		std::cerr << "There is no MyntEye camera to output";
}
int nonZero(const cv::Mat m)
{
	int c = 0;
	ushort* ptr = (ushort*)m.data;
	for (std::size_t y = 0; y < m.rows; ++y) {
		for (std::size_t x = 0; x < m.cols; ++x) {
			if (*ptr++ != 0)
				c++;
		}
	}
	return c;
}
#define PI 3.14159265
void publishPoints(std::shared_ptr<mynteye::API> api,
	const api::StreamData &data, cv::Mat left, std::uint32_t seq) {
	// if (points_publisher_.getNumSubscribers() == 0)
	//   return;

	if (p[PCL_IDX]->getNumSubscribers())
	{
		auto &&in = api->GetIntrinsicsBase(Stream::LEFT);



		int count = 0;
		cv::Mat m = data.frame;
		ushort min, max;
		analyzeRange(m, &min, &max);

		ushort* ptr = (ushort*)m.data;
		for (std::size_t y = 0; y < m.rows; ++y) {
			for (std::size_t x = 0; x < m.cols; ++x) {
				auto point = *ptr++;
				if (point != 0)
					count++;
			}
		}
		auto time = ros::Time::now();
		sensor_msgs::PointCloud2 msg;
		msg.header.seq = seq;
		msg.header.stamp = time;
		msg.header.frame_id = "map";
		
		msg.width = count;
		msg.height = 1;
		msg.is_dense = true;

		sensor_msgs::PointCloud2Modifier modifier(msg);

		modifier.setPointCloud2Fields(
			4,
			"x", 1, sensor_msgs::PointField::FLOAT32,
			"y", 1, sensor_msgs::PointField::FLOAT32,
			"z", 1, sensor_msgs::PointField::FLOAT32,
			"m", 1, sensor_msgs::PointField::UINT8);

		modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

		sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
		sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
		sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

		sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
		sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

		ptr = (ushort*)m.data;

		float FOV_H = 122;
		float FOV_V = 76;

		byte* color = left.data;

		for (int y = 0; y < m.rows; ++y) {
			for (int x = 0; x < m.cols; ++x) {
				auto point = *ptr++;
				auto clr = *color++;
				if (point != 0)
				{
#define ALIGN(A) 
#define CENTER(FOV) - FOV * 0.5
#define LEFT (FOV)
#define TOP(FOV)
#define RIGHT(FOV)
#define BOTTOM(FOV)
					float h_theta = (FOV_H * (x / (float)m.cols) ALIGN(CENTER(FOV_H)))* PI / 180.0;
					float v_theta = (FOV_V * ((m.rows - y) / (float)m.rows - 0.5f) ALIGN(CENTER(FOV_V)))* PI / 180.0;
					float yaw = h_theta;
					float pitch = v_theta;

					float x = sin(yaw)*cos(pitch)*point / 1000;
					float y = cos(yaw)*cos(pitch)*point / 1000;
					float z = sin(pitch)*point / 1000;


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

		p[PCL_IDX]->publish(msg);
	}

}