FROM ubuntu:22.04

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
      libpcl-dev \
      libpng-dev \
      libogre-1.9-dev \
      libois-dev \
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

# Some apps don't show controls without this
ENV QT_X11_NO_MITSHM=1

# Create visp workspace
ENV VISP_WS=/home/$USERNAME/visp-ws

# Install visp-images
RUN mkdir -p ${VISP_WS} \
    && cd ${VISP_WS} \
    && git clone https://github.com/lagadic/visp-images.git \
    && echo "export VISP_WS=${HOME}/visp-ws" >> ${HOME}/.bashrc \
    && echo "export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images" >> ${HOME}/.bashrc

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
