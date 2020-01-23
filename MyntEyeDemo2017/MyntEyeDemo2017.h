#pragma once

void createImg(cv::Mat &img, cv::Mat &depth);

void Method1(std::shared_ptr<mynteye::API> api);
void Method2(std::shared_ptr<mynteye::API> api);
void Method3(std::shared_ptr<mynteye::API> api);

typedef std::function<void(void*)> Callback;

namespace C
{
	class MyntEyeCallback
	{
	public:
		virtual void ReceiveFrame(int stream, int width, int height, void* ptr) = 0;
	};
}
class MyntEyeBase
{
public:
	void SetDepthCallback(C::MyntEyeCallback* f)
	{
		depthCall = f;
	}
	void SetGrayCallback(C::MyntEyeCallback* f)
	{
		grayCall = f;
	}
	void run(int argc, char** argv);
private:
	C::MyntEyeCallback* depthCall,* grayCall;
};