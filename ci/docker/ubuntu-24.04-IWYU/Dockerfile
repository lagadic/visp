FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1001
ENV TZ=Europe/Paris

## Update aptitude with default packages
RUN apt-get update \
    && apt-get install -y \
      cmake \
      cmake-curses-gui \
      curl \
      gedit \
      gdb \
      git \
      iputils-ping \
      locales \
      locate \
      lsb-release \
      mercurial \
      nano \
      net-tools \
      python3 \
      python3-dev \
      python3-pip \
      python3-dbg \
      python3-empy \
      python3-numpy \
      python3-pip \
      python3-psutil \
      python3-venv \
      software-properties-common \
      sudo \
      tzdata \
      vim \
      wget \
    && apt-get clean
#    build-essential \

# Update aptitude with recommended ViSP 3rd parties
RUN apt-get update \
    && apt-get install -y \
      libdc1394-dev \
      libeigen3-dev \
      liblapack-dev \
      libopencv-dev \
      libv4l-dev \
      libx11-dev \
      libzbar-dev \
      nlohmann-json3-dev \
    && apt-get clean

# Update aptitude with other optional ViSP 3rd parties
RUN apt-get update \
    && apt-get install -y \
      libcoin-dev \
      libdmtx-dev \
      libgsl-dev \
      libjpeg-turbo8-dev \
      libpng-dev \
      libogre-1.9-dev \
      libois-dev \
      libpcl-dev \
    && apt-get clean

RUN apt-get update \
    && apt-get install -y \
      libedit-dev \
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

# Install visp-images
RUN mkdir -p ${HOME}/visp-ws \
    && cd ${HOME}/visp-ws \
    && git clone https://github.com/lagadic/visp-images.git \
    && echo "export VISP_WS=${HOME}/visp-ws" >> ${HOME}/.bashrc \
    && echo "export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images" >> ${HOME}/.bashrc

# Get IWYU
RUN cd ${HOME}/visp-ws \
    && mkdir -p 3rdparty \
    && cd 3rdparty \
    && git clone https://github.com/include-what-you-use/include-what-you-use.git

# Build llvm + clang + include-what-you-use
RUN cd ${HOME}/visp-ws \
    && mkdir -p 3rdparty \
    && cd 3rdparty \
    && git clone --depth=1 https://github.com/llvm/llvm-project.git \
    && cd llvm-project \
    && mkdir build \
    && cd build \
    && cmake -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release -DLLVM_EXTERNAL_PROJECTS=iwyu -DLLVM_EXTERNAL_IWYU_SOURCE_DIR=/home/vispci/visp-ws/3rdparty/include-what-you-use ../llvm \
    && make -j4 \
    && sudo make install

# Build and install Panda3D from source
RUN cd ${HOME}/visp-ws \
    && mkdir -p 3rdparty \
    && cd 3rdparty \
    && git clone https://github.com/panda3d/panda3d \
    && cd panda3d \
    && python3 makepanda/makepanda.py --everything --installer --no-egl --no-gles --no-gles2 --no-opencv --threads 4 \
    && sudo dpkg -i panda3d*.deb \
    && echo "export Panda3D_DIR=${HOME}/visp-ws/3rdparty/panda3d" >> ~/.bashrc \
    && . ~/.bashrc

# Build and install YARP from source
RUN sudo apt-get install -y \
    libeigen3-dev \
    libace-dev \
    libedit-dev \
    libsqlite3-dev \
    libtinyxml-dev \
  && cd ${HOME}/visp-ws \
  && mkdir -p 3rdparty \
  && cd 3rdparty \
  && git clone https://github.com/robotology/ycm-cmake-modules \
  && cd ycm-cmake-modules \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j4 \
  && sudo make install \
  && cd ${HOME}/visp-ws/3rdparty \
  && git clone https://github.com/robotology/yarp.git \
  && cd yarp \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j4 \
  && sudo make install

# Create visp build folder
RUN cd ${HOME}/visp-ws \
    && mkdir visp-build

CMD ["/bin/bash"]
