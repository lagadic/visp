FROM ubuntu:26.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1001
ENV TZ=Europe/Paris

## Update aptitude with default packages
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
      libpng-dev \
      libogre-1.9-dev \
      libois-dev \
      libpcl-dev \
      libassimp-dev \
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

# Build latest Panda3D *release* (not master) from source and install the generated .deb
ARG PANDA3D_GIT=https://github.com/panda3d/panda3d
RUN mkdir -p ${VISP_WS}/3rdparty/panda3d \
    && cd ${VISP_WS}/3rdparty/panda3d \
    && PANDA3D_LATEST=$(git -c "versionsort.suffix=-" ls-remote --exit-code --refs --sort="version:refname" --tags ${PANDA3D_GIT} "v*.*.*" \
         | cut -d"/" -f3 \
         | grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' \
         | tail -n 1) \
    && echo "Latest Panda3D release is ${PANDA3D_LATEST}" \
    && git clone --depth 1 --branch ${PANDA3D_LATEST} ${PANDA3D_GIT} panda3d \
    && cd panda3d \
    && python3 makepanda/makepanda.py --everything --installer --no-egl --no-gles --no-gles2 --no-opencv --threads 2 \
    && sudo apt-get update \
    && sudo apt-get install -y ./panda3d*.deb \
    && sudo apt-get clean

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
    && make -j$(nproc) visp_core

CMD ["/bin/bash"]
