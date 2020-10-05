FROM ubuntu:18.04
MAINTAINER Fabien Spindler <Fabien.Spindler@inria.fr>

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris

# Update aptitude with new repo
RUN apt-get update 
    
# Install packages
RUN apt-get install -y \
    build-essential \
    cmake \
    git \
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
    && make

WORKDIR /
