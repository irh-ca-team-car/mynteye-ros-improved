# mynteye-ros-improved
Improvement to the ros wrapper for mynteyesdk that works on both windows and linux

The project has been created using MyntEye demo and converted to the ros node

**OpenCV 4.1.2 is required to be installed in C:\oc2\ in Windows**

Headers files for opencv should be here : C:\oc2\opencv\build\include

On windows the project requires Ros Melodic : http://wiki.ros.org/Installation/Windows

Project cannot be used in DEBUG mode since of ros 

```
Unhandled exception at 0x00007FFFB748A839 in MyntEye_ROS.exe: Microsoft C++ exception: std::length_error at memory location 0x0000000CC9F0EEF0.
```

Tested and working on NVidia Jetson AGX Xavier
