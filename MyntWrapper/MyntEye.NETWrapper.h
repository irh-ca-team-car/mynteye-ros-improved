#pragma once
#include "MyntEyeDemo2017.h"
#include <vector>
using namespace System;

namespace MyntEyeNETWrapper {
	public interface class MyntEyeCallback
	{
		void ReceiveFrame(int stream, int width, int height, void* ptr);
	};
	private class CLB : C::MyntEyeCallback
	{
	public:
		class FRAME
		{
		public:
			int s, w, h;
			void* ptr;
			FRAME(int s, int w, int h, void* ptr)
			{
				this->s = s;
				this->w = w;
				this->h = h;
				this->ptr = ptr;
			}
		};
		
		volatile int flag = 0;
		std::vector<FRAME> frames;
		void ReceiveFrame(int stream, int width, int height, void* ptr)
		{
			frames.push_back(FRAME(stream, width, height, ptr));
			flag++;
		}
	};
	public ref class MyntEye
	{
	public:

		void SetDepthCallback(MyntEyeCallback^ callback)
		{
			depthCall = callback;
		}
		void SetGrayCallback(MyntEyeCallback^ callback)
		{
			grayCall = callback;
		}
		void run()
		{
			base->run(0,0);
		}
		MyntEye()
		{
			base = new MyntEyeBase();
		}
		~MyntEye()
		{
			delete base;
		}
	private :
		MyntEyeBase* base;
		MyntEyeCallback^ grayCall, ^depthCall;
	};
}
