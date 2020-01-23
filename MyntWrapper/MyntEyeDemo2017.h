#pragma once
#include<functional>

namespace C
{
	class MyntEyeCallback
	{
		void ReceiveFrame(int stream, int width, int height, void* ptr) = 0;
	};
}
class MyntEyeBase
{
public:
	void SetDepthCallback(C::MyntEyeCallback f)
	{
		depthCall = f;
	}
	void SetGrayCallback(C::MyntEyeCallback f)
	{
		grayCall = f;
	}
	void run(int argc, char** argv);
private :
	C::MyntEyeCallback depthCall, grayCall;
};