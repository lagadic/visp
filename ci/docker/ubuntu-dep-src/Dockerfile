FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1001
ENV TZ=Europe/Paris

## Variable containing the URL of the GIT repository that needs to be tested.
## By default GIT_URL is set to https://githb.com/lagadic/visp and GIT_BRANCH_NAME is set to master branch
## To build this container use :
##
##   docker build . -t ubuntu-dep-src:v1 --build-arg GIT_URL="${YOUR_URL}" [--build-arg GIT_BRANCH_NAME="{BRANCH_NAME}"]
##
ARG GIT_URL=https://github.com/lagadic/visp
ARG GIT_BRANCH_NAME=master
RUN ["/bin/bash", "-c", ": ${GIT_URL:?Build argument GIT_URL needs to be set and not null.}"]
ENV GIT_URL="$GIT_URL"
ENV GIT_BRANCH_CMD="${GIT_BRANCH_NAME}"
ENV GIT_BRANCH_CMD=${GIT_BRANCH_CMD:+"--branch $GIT_BRANCH_NAME --depth 1"}

# Update aptitude with default packages
RUN apt-get update \
    && apt-get install -y \
      build-essential \
      cmake \
      cmake-curses-gui \
      curl \
      gedit \
      git \
      locales \
      lsb-release \
      iputils-ping \
      nano \
      sudo \
    && apt-get clean

# Install common dependencies
RUN apt update \
    && apt install -y \
      gfortran \
      freeglut3-dev \
      mesa-common-dev \
      mesa-utils \
      nlohmann-json3-dev \
      libboost-all-dev \
      libdc1394-dev \
      libeigen3-dev \
      libflann-dev \
      libgl1-mesa-dev \
      libglfw3-dev \
      libglu1-mesa-dev \
      libgtk-3-dev \
      liblapack-dev \
      libssl-dev \
      libusb-1.0-0-dev \
      libv4l-dev \
      libx11-dev \
      pkg-config \
    && apt-get clean

# Set Locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

ENV USERNAME vispci

RUN useradd -U --uid $USER_UID -ms /bin/bash ${USERNAME} \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && adduser ${USERNAME} sudo \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video

# Commands below are now run as normal user
USER ${USERNAME}

# When running a container start in the home folder
WORKDIR /home/$USERNAME

# Some apps don't show controls without this
ENV QT_X11_NO_MITSHM 1

# Create folder for 3rd parties
RUN mkdir -p ${HOME}/visp-ws/3rdparty

# Install OpenBLAS from source
RUN cd ${HOME}/visp-ws/3rdparty \
    && git clone --depth 1 https://github.com/xianyi/OpenBLAS.git \
    && cd OpenBLAS \
    && mkdir install \
    && make -j$(nproc) \
    && make -j$(nproc) install PREFIX=$(pwd)/install

ENV OpenBLAS_HOME=/3rdparty/OpenBLAS/install

# Install VTK from source
ENV GIT_CLONE_PROTECTION_ACTIVE=false
RUN cd ${HOME}/visp-ws/3rdparty \
    && git clone --recursive --depth 1 --branch v9.3.0 https://github.com/Kitware/VTK.git \
    && cd VTK \
    && mkdir build && cd build && mkdir install \
    && cmake .. -DVTK_ANDROID_BUILD=OFF -DVTK_BUILD_DOCUMENTATION=OFF -DVTK_BUILD_EXAMPLES=OFF -DVTK_BUILD_EXAMPLES=OFF \
      -DCMAKE_BUILD_TYPE=Release -DVTK_GROUP_ENABLE_Imaging=DONT_WANT -DVTK_GROUP_ENABLE_MPI=DONT_WANT \
      -DVTK_GROUP_ENABLE_Web=DONT_WANT -DCMAKE_INSTALL_PREFIX=${HOME}/visp-ws/3rdparty/VTK/build/install \
    && make -j$(nproc) install

ENV VTK_DIR=${HOME}/visp-ws/3rdparty/VTK/build/install

# Install OpenCV from source
RUN cd ${HOME}/visp-ws/3rdparty \
    && git clone --depth 1 https://github.com/opencv/opencv.git \
    && mkdir opencv/build \
    && cd opencv/build \
    && mkdir install \
    && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=${HOME}/visp-ws/3rdparty/opencv/build/install \
    && make -j$(nproc) install

ENV OpenCV_DIR=${HOME}/visp-ws/3rdparty/opencv/build/install

# Install Intel® RealSense™ SDK
RUN cd ${HOME}/visp-ws/3rdparty \
    && git clone --depth 1 https://github.com/IntelRealSense/librealsense.git \
    && mkdir librealsense/build \
    && cd librealsense/build \
    && mkdir install \
    && cmake .. -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=${HOME}/visp-ws/3rdparty/librealsense/build/install \
    && make -j$(nproc) install

ENV REALSENSE2_DIR=${HOME}/visp-ws/3rdparty/librealsense/build/install

# Build PCL from source
RUN cd ${HOME}/visp-ws/3rdparty \
    && git clone --depth 1 https://github.com/PointCloudLibrary/pcl.git \
    && mkdir pcl/build \
    && cd pcl/build \
    && mkdir install \
    && cmake .. -DBUILD_tools=OFF -DBUILD_global_tests=OFF -DPCL_DISABLE_GPU_TESTS=ON -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=${HOME}/visp-ws/3rdparty/pcl/build/install \
    && make -j10 install

ENV PCL_DIR=${HOME}/visp-ws/3rdparty/pcl/build/install

# Install visp-images
RUN mkdir -p ${HOME}/visp-ws \
    && cd ${HOME}/visp-ws \
    && git clone https://github.com/lagadic/visp-images.git \
    && echo "export VISP_WS=${HOME}/visp-ws" >> ${HOME}/.bashrc \
    && echo "export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images" >> ${HOME}/.bashrc

# Download ViSP fork
RUN cd ${HOME}/visp-ws \
    && git clone ${GIT_URL} ${GIT_BRANCH_CMD}

# Build visp
RUN cd ${HOME}/visp-ws \
    && mkdir visp-build \
    && cd visp-build \
    && cmake ../visp -DCMAKE_INSTALL_PREFIX=${HOME}/visp-ws/visp-build/install \
    && make -j$(nproc) developer_scripts \
    && make -j$(nproc) install

CMD ["/bin/bash"]
