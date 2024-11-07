# Docker dedicated to IWYU tool

Include-What-You-Use [IWYU](https://include-what-you-use.org/) is a tool for use with clang to analyze
#includes in C and C++ source files. This docker file contains the receipt to build clang with support of IWYU tool
that can be used to analyse ViSP c++ includes.

## Build docker image from Dockerfile

- We suppose here that you already installed docker.
If this is not the case, follow instructions provided for [Ubuntu](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-docker.html#install_docker_engine_ubuntu) or [MacOS](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-docker.html#install_docker_engine_mac).

- Get ViSP source code
  ```
  $ cd $VISP_WS/
  $ git clone https://github.com/lagadic/visp
  ```

- Build docker image from Dockerfile running the following:
  ```
  $ cd $VISP_WS/visp/ci/docker/ubuntu-24.04-IWYU
  $ docker build -t vispci/vispci:ubuntu-24.04-IWYU .
  ```

## Start the container

### On Ubuntu host

- On your computer running Ubuntu, allow access to the X11 server
  ```
  $ xhost +local:docker
  non-network local connections being added to access control list
  ```
- Run your Docker container. The following command connects to the ubuntu-24.04-IWYU Docker container.
  ```
  $ docker run --rm -it --network=host --privileged \
            --env=DISPLAY \
            --env=QT_X11_NO_MITSHM=1 \
            --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
            --volume=/dev:/dev \
            --volume=$VISP_WS/visp:/home/vispci/visp-ws/visp \
            vispci/vispci:ubuntu-24.04-IWYU
  vispci@6c8d67579659:~$ pwd
  /home/vispci
  ```
- Build ViSP with IWYU support
  ```
  vispci@6c8d67579659:~$ cd ~/visp-ws/visp-build
  vispci@6c8d67579659:~$ CC="clang" CXX="clang++" cmake -DCMAKE_CXX_INCLUDE_WHAT_YOU_USE=include-what-you-use ../visp
  vispci@6c8d67579659:~$ make -j4 visp_modules > ../visp/iwyu.log 2>&1
  ```

### On MacOS host

- Get your MacOS computer IP address
  ```
  $ IP=$(/usr/sbin/ipconfig getifaddr en0)
  $ echo $IP
  $ 192.168.1.18
  ```
- Allow connections from MacOS to XQuartz
  ```
  $ xhost + "$IP"
  192.168.1.18 being added to access control list
  ```
- Run your Docker container. The following command connects to the ubuntu-24.04-IWYU Docker container.
  ```
  $ docker run --rm -it --network=host --privileged \
            --env=DISPLAY="${IP}:0" \
            --env=QT_X11_NO_MITSHM=1 \
            --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
            --volume=/dev:/dev \
            --volume=$VISP_WS/visp:/home/vispci/visp-ws/visp \
            vispci/vispci:ubuntu-24.04-IWYU
  vispci@6c8d67579659:~$ pwd
  /home/vispci
  ```
- Build ViSP with IWYU support
  ```
  vispci@6c8d67579659:~$ cd ~/visp-ws/visp-build
  vispci@6c8d67579659:~$ CC="clang" CXX="clang++" cmake -DCMAKE_CXX_INCLUDE_WHAT_YOU_USE=include-what-you-use ../visp
  vispci@6c8d67579659:~$ make -j4 visp_modules > ../visp/iwyu.log 2>&1
  ```

# Analyse IWYU logs

IWYU logs are available either on your host in `$VISP_WS/visp/iwyu.log` or in your docker container in `/home/vispci/visp-ws/visp/iwyu.log`.
