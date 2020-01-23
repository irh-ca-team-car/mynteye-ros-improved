#pragma once


typedef std::function<void(void*)> Callback;

class MyntEyeBase
{
public:
	void SetDepthCallback(Callback f)
	{
		depthCall = f;
	}
	void SetGrayCallback(Callback f)
	{
		grayCall = f;
	}
	void run(int argc, char** argv);
private :
	Callback depthCall, grayCall;
};