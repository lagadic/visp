FROM ubuntu:22.04

## Variable containing the URL of the GIT repository that needs to be tested
## To build this container use docker build . -t ubuntu-dep-src:v1 --build-arg GIT_URL="${YOUR_URL}" [--build-arg GIT_BRANCH_NAME="{BRANCH_NAME}"]
ARG GIT_URL
ARG GIT_BRANCH_NAME
RUN ["/bin/bash", "-c", ": ${GIT_URL:?Build argument GIT_URL needs to be set and not null.}"]
ENV GIT_URL="$GIT_URL"
ENV GIT_BRANCH_CMD="${GIT_BRANCH_NAME}"
ENV GIT_BRANCH_CMD=${GIT_BRANCH_CMD:+"--branch $GIT_BRANCH_NAME --depth 1"}

# Install common dependencies
RUN apt update &&\
    apt install -y libdc1394-dev \
    libx11-dev libv4l-dev gfortran liblapack-dev libeigen3-dev \
    libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    mesa-common-dev mesa-utils freeglut3-dev libflann-dev libboost-all-dev \
    nlohmann-json3-dev

RUN apt install -y build-essential cmake-curses-gui git

# Install OpenBLAS from source
RUN mkdir -p /3rdparty &&\
    cd /3rdparty &&\
    git clone --depth 1 https://github.com/xianyi/OpenBLAS.git &&\
    cd OpenBLAS &&\
    mkdir install &&\
    make -j10 &&\
    make -j10 install PREFIX=$(pwd)/install

ENV OpenBLAS_HOME=/3rdparty/OpenBLAS/install

# Install VTK from source
ENV GIT_CLONE_PROTECTION_ACTIVE=false
RUN cd /3rdparty &&\
    git clone --recursive --depth 1 --branch v9.3.0 https://github.com/Kitware/VTK.git &&\
    cd VTK &&\
    mkdir build && cd build && mkdir install &&\
    cmake .. -DVTK_ANDROID_BUILD=OFF -DVTK_BUILD_DOCUMENTATION=OFF -DVTK_BUILD_EXAMPLES=OFF -DVTK_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release \
      -DVTK_GROUP_ENABLE_Imaging=DONT_WANT -DVTK_GROUP_ENABLE_MPI=DONT_WANT -DVTK_GROUP_ENABLE_Web=DONT_WANT -DCMAKE_INSTALL_PREFIX=/3rdparty/VTK/build/install &&\
    make -j10 install

ENV VTK_DIR=/3rdparty/VTK/build/install

# Install OpenCV from source
RUN cd /3rdparty &&\
    git clone --depth 1 https://github.com/opencv/opencv.git &&\
    mkdir opencv/build &&\
    cd opencv/build &&\
    mkdir install &&\
    cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/3rdparty/opencv/build/install &&\
    make -j10 install

ENV OpenCV_DIR=/3rdparty/opencv/build/install

# Install Intel® RealSense™ SDK
RUN cd /3rdparty &&\
    git clone --depth 1 https://github.com/IntelRealSense/librealsense.git &&\
    mkdir librealsense/build &&\
    cd librealsense/build &&\
    mkdir install &&\
    cmake .. -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/3rdparty/librealsense/build/install &&\
    make -j10 install

ENV REALSENSE2_DIR=/3rdparty/librealsense/build/install

# Build PCL from source
RUN cd /3rdparty &&\
    git clone --depth 1 https://github.com/PointCloudLibrary/pcl.git &&\
    mkdir pcl/build &&\
    cd pcl/build &&\
    mkdir install &&\
    cmake .. -DBUILD_tools=OFF -DBUILD_global_tests=OFF -DPCL_DISABLE_GPU_TESTS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/3rdparty/pcl/build/install &&\
    make -j10 install

ENV PCL_DIR=/3rdparty/pcl/build/install

# Download ViSP image ViSP
RUN mkdir -p /root/visp_ws/ &&\
    cd /root/visp_ws &&\
    git clone --depth 1 https://github.com/lagadic/visp-images /root/visp_ws/visp-images

ENV VISP_INPUT_IMAGE_PATH=/root/visp_ws/visp-images

# Download ViSP fork
RUN cd /root/visp_ws &&\
    git clone "${GIT_URL}" /root/visp_ws/visp "${GIT_BRANCH_CMD}" &&\
    mkdir -p visp-build &&\
    cd visp-build && mkdir install &&\
    cmake .. -DCMAKE_INSTALL_PREFIX=/root/visp_ws/visp-build/install &&\
    make -j10 developer_scripts &&\
    make -j10 install

ENV VISP_DIR=/root/visp_ws/visp-build/install
