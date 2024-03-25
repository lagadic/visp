FROM ubuntu:18.04
MAINTAINER Fabien Spindler <Fabien.Spindler@inria.fr>

ARG DEBIAN_FRONTEND=noninteractive
ARG VISPCI_USER_UID=1001
ARG DOCKER_GROUP_GID=130
ENV TZ=Europe/Paris

# Update aptitude with new repo
RUN apt-get update

# Install packages
RUN apt-get install -y \
    sudo \
    build-essential \
    cmake \
    git \
    net-tools \
    iputils-ping \
    # Recommended ViSP 3rd parties
    libopencv-dev \
    libx11-dev \
    liblapack-dev \
    libeigen3-dev \
    libdc1394-22-dev \
    libv4l-dev \
    libzbar-dev \
    libpthread-stubs0-dev \
    # Other optional 3rd parties
    libpcl-dev \
    libcoin80-dev \
    libjpeg-dev \
    libpng-dev \
    libogre-1.9-dev \
    libois-dev \
    libdmtx-dev \
    libgsl-dev

RUN adduser --disabled-password --gecos "" --uid $VISPCI_USER_UID vispci \
    && groupadd docker --gid $DOCKER_GROUP_GID \
    && usermod -aG sudo vispci \
    && usermod -aG docker vispci \
    && echo "%sudo   ALL=(ALL:ALL) NOPASSWD:ALL" > /etc/sudoers \
    && echo "Defaults env_keep += \"DEBIAN_FRONTEND\"" >> /etc/sudoers \
    && adduser vispci video

ENV HOME=/home/vispci

# Install visp-images
RUN mkdir -p ${HOME}/visp-ws \
    && cd ${HOME}/visp-ws \
    && git clone https://github.com/lagadic/visp-images.git \
    && echo "export VISP_WS=${HOME}/visp-ws" >> ${HOME}/.bashrc \
    && echo "export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images" >> ${HOME}/.bashrc

# Get visp
RUN cd ${HOME}/visp-ws \
    && git clone https://github.com/lagadic/visp

# Build visp
RUN cd ${HOME}/visp-ws \
    && mkdir visp-build \
    && cd visp-build \
    && cmake ../visp \
    && make -j4

USER vispci

WORKDIR /home/vispci
