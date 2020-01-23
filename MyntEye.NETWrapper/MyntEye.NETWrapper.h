#pragma once
#include "MyntEyeDemo2017.h"
using namespace System;

namespace MyntEyeNETWrapper {
	public ref class MyntEye
	{
	public:

		void SetDepthCallback(System::Action<System::IntPtr>^ callback)
		{

		}
		void SetGrayCallback(System::Action<System::IntPtr>^ callback)
		{

		}
	private :
		MyntEyeBase base;
	};
}
