FROM ubuntu:14.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1001
ENV TZ=Europe/Paris

# Update aptitude with default packages
RUN apt-get update \
    && apt-get install -y \
      build-essential \
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
      nano \
      net-tools \
      software-properties-common \
      sudo \
      tzdata \
      vim \
      wget \
    && apt-get clean

# Update aptitude with recommended ViSP 3rd parties
RUN apt-get update \
    && apt-get install -y \
      libdc1394-22-dev \
      libeigen3-dev \
      liblapack-dev \
      libopencv-dev \
      libv4l-dev \
      libx11-dev \
      libzbar-dev \
    && apt-get clean

# Update aptitude with other optional ViSP 3rd parties
RUN apt-get update \
    && apt-get install -y \
      libcoin80-dev \
      libjpeg-dev \
      libpng-dev \
      libogre-1.9-dev \
      libois-dev \
      libdmtx-dev \
    && apt-get clean

# Set Locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

ENV USERNAME=vispci

RUN useradd -U --uid $USER_UID -ms /bin/bash ${USERNAME} \
    && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && adduser ${USERNAME} sudo \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video

# Commands below are now run as normal user
USER ${USERNAME}

# When running a container start in the home folder
WORKDIR /home/$USERNAME

# Create visp workspace
ENV VISP_WS=/home/$USERNAME/visp-ws

# Install visp-images
RUN mkdir -p ${VISP_WS} \
    && cd ${VISP_WS} \
    && git clone https://github.com/lagadic/visp-images.git \
    && echo "export VISP_WS=${HOME}/visp-ws" >> ${HOME}/.bashrc \
    && echo "export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images" >> ${HOME}/.bashrc

# Download cmake 3.32.2 linux-x86_64
RUN mkdir -p ${VISP_WS}/3rdparty/cmake \
    && cd ${VISP_WS}/3rdparty/cmake \
    && curl -SLO https://github.com/Kitware/CMake/archive/refs/tags/v3.10.0.tar.gz \
    && tar xvzf v3.10.0.tar.gz \
    && cd CMake-3.10.0 \
    && sh bootstrap --prefix=/usr \
    && make -j$(nproc) \
    && sudo make install

# Get visp
RUN cd ${VISP_WS} \
    && git clone https://github.com/lagadic/visp

# Build visp
RUN cd ${VISP_WS} \
    && mkdir visp-build \
    && cd visp-build \
    && cmake ../visp \
    && make -j$(nproc)

CMD ["/bin/bash"]
