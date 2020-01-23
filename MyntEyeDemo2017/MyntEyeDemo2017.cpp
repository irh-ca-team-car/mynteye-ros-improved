// MyntEyeDemo2017.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <Windows.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<mynteye/api/api.h>
#include <chrono>
#include <thread>
#include <mutex>
#include "MyntEyeDemo2017.h"
#include <atomic>
#include "CVpainter.h"
#include <conio.h>


using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

using namespace mynteye;
using namespace cv;
using byte = unsigned char;
typedef cv::Point3_<uint8_t> Pixel;
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
void clear() {
	COORD topLeft = { 0, 0 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_SCREEN_BUFFER_INFO screen;
	DWORD written;

	GetConsoleScreenBufferInfo(console, &screen);
	FillConsoleOutputCharacterA(
		console, ' ', screen.dwSize.X * screen.dwSize.Y, topLeft, &written
	);
	FillConsoleOutputAttribute(
		console, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE,
		screen.dwSize.X * screen.dwSize.Y, topLeft, &written
	);
	SetConsoleCursorPosition(console, topLeft);
}
struct Operator
{
	void operator ()(ushort &pixel, const int * position) const
	{
		if (pixel < 100)
			pixel = INT16_MAX;
	}
};
std::string to_string(double data)
{
	return std::to_string(data / 1000) + "m";
}
void IMU(std::shared_ptr<mynteye::API> api)
{
	api->EnableMotionDatas();
	api->EnableStreamData(Stream::DEPTH);
	api->EnableStreamData(Stream::POINTS);
	//auto stream_intrinsics = api->GetIntrinsics(Stream::POINTS);

	//PCViewer pcviewer(stream_intrinsics,1000);
	cv::namedWindow("frame");

	std::atomic_uint depth_count(0);
	double avgDepthMM = 0;
	std::mutex depth_mtx, left_mtx;
	api->SetStreamCallback(
		Stream::DEPTH,
		[&depth_mtx,&avgDepthMM](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(depth_mtx);

		auto mat = data.frame;

		auto str = to_string(avgDepthMM=average(mat));
		//mat.convertTo(mat, CV_8UC1, 1);
		//std::cout << str << std::endl;

		mat.forEach<ushort>(Operator());
		mat *= 2;
		mat = INT16_MAX - mat;//0.8m = 800
		//0m = white
		//16m= black
		
		putText(data.frame, str, Point(0, 20), FONT_HERSHEY_COMPLEX, 1, Scalar(INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX), 3, 8, false);

		imshow("depth", mat); // CV_16UC1 

	});
	api->SetStreamCallback(Stream::POINTS, [](const api::StreamData &data) {
		int inttype = data.frame.type();
		//printf("%d", inttype);
	});
	api->SetMotionCallback([&avgDepthMM](const api::MotionData & data) {
		//clear();
		double G = 9.81;
		printf("IMU: Acc : x=%5.2lf m/s2 y=%5.2lf m/s2 z=%5.2lf m/s2 ", data.imu->accel[0]*G, data.imu->accel[1] * G, data.imu->accel[2] * G);
		printf(" Gyro: x=%8.1lf d/s y=%8.1lf d/s z=%8.1lf d/s ", data.imu->gyro[0], data.imu->gyro[1], data.imu->gyro[2]);
		printf(" DEPTH: %5.2lf M\r\n", avgDepthMM/1000);
	});

	api->Start(Source::ALL);
	CVPainter painter;
	
	
	while (true) {
		api->WaitForStreams();
		auto &&left_data = api->GetStreamData(Stream::LEFT);
		auto &&right_data = api->GetStreamData(Stream::RIGHT);
		cv::Mat img;
		cv::hconcat(left_data.frame, right_data.frame, img);
		auto &&motion_datas = api->GetMotionDatas();
		/*
		for (auto &&data : motion_datas) {
		LOG(INFO) << "Imu frame_id: " << data.imu->frame_id
		<< ", timestamp: " << data.imu->timestamp
		<< ", accel_x: " << data.imu->accel[0]
		<< ", accel_y: " << data.imu->accel[1]
		<< ", accel_z: " << data.imu->accel[2]
		<< ", gyro_x: " << data.imu->gyro[0]
		<< ", gyro_y: " << data.imu->gyro[1]
		<< ", gyro_z: " << data.imu->gyro[2]
		<< ", temperature: " << data.imu->temperature;
		}
		*/
		painter.DrawStreamData(img, left_data);
		static std::vector<api::MotionData> motion_datas_s = motion_datas;
		if (!motion_datas.empty() && motion_datas.size() > 0) {
			motion_datas_s = motion_datas;
		}
		if (!motion_datas_s.empty() && motion_datas_s.size() > 0) {
			painter.DrawMotionData(img, motion_datas_s[0]);
		}
		cv::imshow("frame", img);
		char key = static_cast<char>(cv::waitKey(1));
		if (key == 27 || key == 'q' || key == 'Q') { // ESC/Q
			break;
		}
	}

	api->Stop(Source::ALL);

}

int main(int argc, char* argv[])
{
	auto api = API::Create(argc, argv);

	namedWindow("frame");
	namedWindow("depth");
	Mat img = Mat(200, 400, CV_8UC3);
	Mat depth = Mat(200, 400, CV_16UC1);
	srand(time(0));

	createImg(img, depth);

	imwrite("currentFrame.png", img);

	if (api)
	{
		IMU(api);
	}
	else
		while (true) {
			createImg(img, depth);
			imshow("frame", img);
			imshow("depth", depth);
			waitKey(33);
		}
}

void Method1(std::shared_ptr<mynteye::API> api)
{
	api->EnableStreamData(Stream::DEPTH);

	api->Start(Source::VIDEO_STREAMING);

	while (true) {
		api->WaitForStreams();
		auto &&left_data = api->GetStreamData(Stream::LEFT);
		auto &&right_data = api->GetStreamData(Stream::RIGHT);

		//hconcat(left_data.frame, right_data.frame, img);
		//imshow("frame", img);
		auto &&depth_data = api->GetStreamData(Stream::DEPTH);
		if (!depth_data.frame.empty()) {

			imshow("depth", depth_data.frame); // CV_16UC1 
		}
		char key = static_cast<char>(cv::waitKey(1));
		if (key == 27 || key == 'q' || key == 'Q') { // ESC/Q
			break;
		}
	}
	api->Stop(Source::VIDEO_STREAMING);
}

void Method2(std::shared_ptr<mynteye::API> api)
{
	api->EnableStreamData(Stream::DEPTH);
	api->EnableStreamData(Stream::LEFT);

	std::atomic_uint depth_count(0);
	std::mutex depth_mtx, left_mtx;
	api->SetStreamCallback(
		Stream::DEPTH,

		[&depth_mtx](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(depth_mtx);

		auto mat = data.frame;

		auto str = to_string(average(mat));
		//mat.convertTo(mat, CV_8UC1, 1);
		std::cout << str << std::endl;

		mat.forEach<ushort>(Operator());
		mat *= 2;
		mat = INT16_MAX - mat;//0.8m = 800
		//0m = white
		//16m= black
		

		putText(data.frame, str, Point(0, 20), FONT_HERSHEY_COMPLEX, 1, Scalar(INT16_MAX, INT16_MAX, INT16_MAX, INT16_MAX), 3, 8, false);

		imshow("depth", mat); // CV_16UC1 

	});
	api->SetStreamCallback(
		Stream::LEFT,

		[&left_mtx](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(left_mtx);

		imshow("frame", data.frame); // CV_16UC1 

	});
	api->Start(Source::ALL);
	while (true)
	{
		api->WaitForStreams();
		waitKey(1);
	}
	api->Stop(Source::ALL);
}

void Method3(std::shared_ptr<mynteye::API> api)
{
	api->EnableStreamData(Stream::DEPTH);
	api->EnableStreamData(Stream::LEFT);

	std::atomic_uint depth_count(0);
	std::mutex depth_mtx;
	std::mutex left_mtx;
	api->SetStreamCallback(
		Stream::DEPTH,

		[&depth_mtx](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(depth_mtx);

		auto mat = data.frame;
		mat.convertTo(mat, CV_8UC3, 1);
		auto str = to_string(average(mat));
		std::cout << str << std::endl;
		putText(data.frame, str, Point(0, 20), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 255, 255), 3, 8, false);

		imshow("depth", mat); // CV_16UC1 

	});
	api->SetStreamCallback(
		Stream::LEFT,

		[&left_mtx](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(left_mtx);

		imshow("frame", data.frame); // CV_8UC1 

	});
	waitKey();
}

void createImg(cv::Mat &img, cv::Mat &depth)
{
	for (auto i = 0; i < img.rows; i++)
		for (auto j = 0; j < img.cols; j++)
			img.at<Pixel>(i, j) = Pixel(rand() % 256, rand() % 256, rand() % 256);
	putText(img, "DEMO", Point(0, 20), FONT_HERSHEY_COMPLEX, 1, Scalar(rand() % 256, rand() % 256, rand() % 256), 3, 8, false);
	for (auto i = 0; i < img.rows; i++)
		for (auto j = 0; j < img.cols; j++)
			depth.at<uint16_t>(i, j) = rand() % (UINT16_MAX + 1);
}

void MyntEyeBase::run(int argc, char** argv)
{
	auto api = API::Create(argc, argv);
	api->EnableStreamData(Stream::DEPTH);
	api->EnableStreamData(Stream::LEFT);

	std::atomic_uint depth_count(0);
	std::mutex depth_mtx, left_mtx;
	api->SetStreamCallback(
		Stream::DEPTH,

		[&depth_mtx, this](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(depth_mtx);
		auto mat = data.frame;
		if (depthCall)
			depthCall->ReceiveFrame(1,mat.cols, mat.rows, mat.data);
	});
	api->SetStreamCallback(
		Stream::LEFT,

		[&left_mtx, this](const api::StreamData &data) {

		std::lock_guard<std::mutex> _(left_mtx);
		auto mat = data.frame;
		if (grayCall)
			grayCall->ReceiveFrame(2,mat.cols, mat.rows, mat.data);

	});
	api->Start(Source::ALL);
	while (true)
	{
		api->WaitForStreams();
		waitKey(1);
	}
	api->Stop(Source::ALL);
}
