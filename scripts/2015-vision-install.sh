#! /bin/bash
 
STARTDIR=$(pwd)
INSTALL_PREFIX="$STARTDIR"/libs
echo "OpenCV libraries will be placed in $INSTALL_PREFIX"
 
PYTHON_PATH="$INSTALL_PREFIX"/lib/python2.6/site-packages
echo "Python files placed in $PYTHON_PATH"
 
mkdir -p "$INSTALL_PREFIX"
mkdir -p "$PYTHON_PATH"
 
export LOCALBASE="$INSTALL_PREFIX"
export PYTHONPATH="$PYTHONPATH:$PYTHON_PATH" # easy_install needs python path to be set up
 
# Add path to pythonpath
echo "Adding Python Path to ~./bashrc"
STR="export PYTHONPATH=\$PYTHONPATH:$PYTHON_PATH"
grep -Fxq "$STR" ~/.bashrc || echo "$STR" >> ~/.bashrc
 
echo "Downloading OpenCV..."
 
cd /disk/scratch/sdp/
wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip/download -O opencv.zip
unzip opencv.zip

mkdir /disk/scratch/sdp/opencv-2.4.9/release
cd /disk/scratch/sdp/opencv-2.4.9/

echo "Installing OpenCV..." 
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" -DWITH_FFMPEG=OFF -DBUILD_NEW_PYTHON_SUPPORT=ON .
make && make install
 
echo "Installing pySerial"
easy_install --prefix="$INSTALL_PREFIX" pyserial

echo "Installing Polygon2"
easy_install --prefix="$INSTALL_PREFIX" polygon2

echo "Installing ArgParse"
easy_install --prefix="$INSTALL_PREFIX" argparse
