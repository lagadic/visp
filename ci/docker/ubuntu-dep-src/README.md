# Docker dedicated to check compatibility with 3rd party last releases

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
  $ cd $VISP_WS/visp/ci/docker/ubuntu-dep-src
  $ docker build -t vispci/vispci:ubuntu-dep-src .
  ```
  The version of ViSP that is considered is by default the master branch from github official repo in
  https://github.com/lagadic/visp

  There is also the possibility to build visp from a specific repo and branch using `GIT_URL` and `GIT_BRANCH_NAME`
  args, like
  ```
  $ cd $VISP_WS/visp/ci/docker/ubuntu-dep-src
  $ docker build -t vispci/vispci:ubuntu-dep-src --build-arg GIT_URL=https://github.com/lagadic/visp --build-arg GIT_BRANCH_NAME=master .
  ```

## Start the container

### On Ubuntu host

- On your computer running Ubuntu, allow access to the X11 server
  ```
  $ xhost +local:docker
  non-network local connections being added to access control list
  ```
- Run your Docker container. The following command connects to the ubuntu-dep-src Docker container.
  ```
  $ docker run --rm -it --network=host --privileged \
            --env=DISPLAY \
            --env=QT_X11_NO_MITSHM=1 \
            --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
            --volume=/dev:/dev \
            vispci/vispci:ubuntu-dep-src
  vispci@6c8d67579659:~$ pwd
  /home/vispci
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
- Run your Docker container. The following command connects to the ubuntu-dep-src Docker container.
  ```
  $ docker run --rm -it --network=host --privileged \
            --env=DISPLAY="${IP}:0" \
            --env=QT_X11_NO_MITSHM=1 \
            --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
            --volume=/dev:/dev \
            vispci/vispci:ubuntu-dep-src
  vispci@6c8d67579659:~$ pwd
  /home/vispci
  ```
