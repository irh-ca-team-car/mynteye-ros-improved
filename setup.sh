#!/bin/bash

mkdir ~/git
cd ~/git
[ -e MYNT-EYE-S-SDK ] || git clone https://github.com/irh-ca-team-car/MYNT-EYE-S-SDK
[ -e opencv ] || git clone https://github.com/opencv/opencv.git
cd opencv
git checkout tags/3.4.1

mkdir _build
cd _build/

cmake \
-DCMAKE_BUILD_TYPE=RELEASE \
-DCMAKE_INSTALL_PREFIX=/usr \
\
-DWITH_CUDA=OFF \
\
-DBUILD_DOCS=OFF \
-DBUILD_EXAMPLES=OFF \
-DBUILD_TESTS=OFF \
-DBUILD_PERF_TESTS=OFF \
-DBUILD_opencv_python2=OFF \
-DBUILD_opencv_python3=OFF \
..

make -j4
sudo make install

export OpenCV_DIR=~/git/opencv
cd ..

cd MYNT-EYE-S-SDK

make init
make install
