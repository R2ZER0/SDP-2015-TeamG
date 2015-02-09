#!/bin/bash

########################################################################
#                          INSTALL_VENV.SH                             #
#                 Author: Ayrton Massey, s1208057                      #
#                                                                      #
#                                                                      #
# Installs the dependencies for the project. The whole process will    #
# take around 20 minutes and take up about 3.0GB of your disk quota.   #
#                                                                      #
# Run this from inside the scripts folder. The script will create a    #
# virtual environment called venv in a folder beside your local        #
# repo. It installs Polygon2, numpy, pySerial and ArgParse using pip,  #
# then downloads opencv 2.4.10 and installs it. The script removes the #
# MD5 hashing from cl2pp.cmake to prevent the crash at 71%. If         #
# everything worked correctly, the controller should open if you're on #
# a computer with access to the pitch video feed.                      #
########################################################################


#set up virtual environment
cd ../../
virtualenv venv
source venv/bin/activate

#install dependencies
pip install Polygon2
pip install numpy
pip install pySerial
pip install ArgParse
pip install unittest2

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

#remove cv2 compiled files
cd ../../
rm -rf opencv-2.4.10
rm opencv-2.4.10.zip

#move cv2 to the right directory
mv venv/lib/python2.7/site-packages/* venv/lib/python2.6/site-packages/

#try running the controller
cd SDP-2015-Group9
python controller.py 0 left yellow
