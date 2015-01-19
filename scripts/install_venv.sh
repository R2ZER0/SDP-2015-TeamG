#!/bin/bash

#set up virtual environment
cd ../../
virtualenv venv
source venv/bin/activate

#install dependencies
pip install Polygon2
pip install numpy
pip install pySerial
pip install ArgParse

#download cv2
curl -L -O http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.10/opencv-2.4.10.zip
unzip opencv-2.4.10.zip
cd opencv-2.4.10

#build cv2
mkdir release
cd release
cmake -D MAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$VIRTUAL_ENV/local/ -D PYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python -D PYTHON_PACKAGES_PATH=$VIRTUAL_ENV/lib/python2.7/site-packages -D INSTALL_PYTHON_EXAMPLES=ON ..
sed -i '/string(MD5 hash "${lines}")/d' ../cmake/cl2cpp.cmake

#install to the venv
make -j8
make install

#move cv2 to the right directory
cd ../../
mv venv/lib/python2.7/site-packages/* venv/lib/python2.6/site-packages/

#try running the controller
cd SDP-2015-Group9
python controller.py 0 left yellow
