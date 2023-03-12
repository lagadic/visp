<h1>ViSP: Open source Visual Servoing Platform</h1>

[![Github Releases](https://img.shields.io/github/release/lagadic/visp.svg)](https://github.com/lagadic/visp/releases)
[![License](https://img.shields.io/badge/License-GPLv2-bright)](https://opensource.org/licenses/GPL-2.0)

Platform | Build Status |
-------- | ------------ |
Ubuntu 18.04, 20.04, 22.04 (amd64)| [![ubuntu dep apt workflow](https://github.com/lagadic/visp/actions/workflows/ubuntu-dep-apt.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-dep-apt.yml) [![ubuntu dep src workflow](https://github.com/lagadic/visp/actions/workflows/ubuntu-dep-src.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-dep-src.yml)
macOS 11 and 12 | [![macos workflow](https://github.com/lagadic/visp/actions/workflows/macos.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/macos.yml)
iOS on macOS 11.0| [![ios workflow](https://github.com/lagadic/visp/actions/workflows/ios.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ios.yml)
Windows 10 | [![Build status](https://ci.appveyor.com/api/projects/status/121dscdkryf5dbn0/branch/master?svg=true)](https://ci.appveyor.com/project/fspindle/visp/branch/master)
ARM | [![Build Status](https://cloud.drone.io/api/badges/lagadic/visp/status.svg)](https://cloud.drone.io/lagadic/visp)
Other arch Ubuntu 20.04 (aarch64, s390x)| [![other arch workflow](https://github.com/lagadic/visp/actions/workflows/other-arch.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/other-arch.yml)
ROS Melodic Ubuntu 18.04 Bionic | [![Build Status](https://build.ros.org/buildStatus/icon?job=Mdev__visp__ubuntu_bionic_amd64)](https://build.ros.org/job/Mdev__visp__ubuntu_bionic_amd64)
ROS Noetic Ubuntu 20.04 Focal | [![Build Status](https://build.ros.org/buildStatus/icon?job=Ndev__visp__ubuntu_focal_amd64)](https://build.ros.org/job/Ndev__visp__ubuntu_focal_amd64/)
ROS Noetic Debian 10.13 Buster | [![Build Status](https://build.ros.org/buildStatus/icon?job=Ndev_db__visp__debian_buster_amd64)](https://build.ros.org/job/Ndev_db__visp__debian_buster_amd64)
ROS2 Foxy Ubuntu 20.04 Focal| [![Build Status](https://build.ros2.org/buildStatus/icon?job=Fdev__visp__ubuntu_focal_amd64)](https://build.ros2.org/job/Fdev__visp__ubuntu_focal_amd64/)
ROS2 Galactic Ubuntu 20.04 Focal| [![Build Status](https://build.ros2.org/buildStatus/icon?job=Gdev__visp__ubuntu_focal_amd64)](https://build.ros2.org/job/Gdev__visp__ubuntu_focal_amd64/)
ROS2 Rolling Ubuntu 22.04 Jammy| [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__visp__ubuntu_jammy_amd64)](https://build.ros2.org/job/Rdev__visp__ubuntu_jammy_amd64)
Valgrind | [![valgrind workflow](https://github.com/lagadic/visp/actions/workflows/valgrind.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/valgrind.yml)
Sanitizer | [![sanitizers workflow](https://github.com/lagadic/visp/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/sanitizers.yml)
Code coverage | [![Code coverage](https://codecov.io/gh/lagadic/visp/branch/master/graph/badge.svg?token=GQIiKbA3BC)](https://codecov.io/gh/lagadic/visp)

Other projects | Build Status |
-------------- | ------------ |
[UsTK](https://github.com/lagadic/ustk) | [![macOS](https://github.com/lagadic/visp/actions/workflows/macos-ustk.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/macos-ustk.yml) [![Ubuntu](https://github.com/lagadic/visp/actions/workflows/ubuntu-ustk.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-ustk.yml)
[visp_contrib](https://github.com/lagadic/visp_contrib) | [![Ubuntu](https://github.com/lagadic/visp/actions/workflows/ubuntu-contrib.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-contrib.yml)
[visp_sample](https://github.com/lagadic/visp_sample) | [![macos workflow](https://github.com/lagadic/visp/actions/workflows/macos.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/macos.yml) [![ubuntu dep apt workflow](https://github.com/lagadic/visp/actions/workflows/ubuntu-dep-apt.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-dep-apt.yml)
[camera_localization](https://github.com/lagadic/camera_localization) | [![ubuntu_3rdparty_workflow](https://github.com/lagadic/visp/actions/workflows/ubuntu-3rdparty.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-3rdparty.yml)
[visp_started](https://github.com/lagadic/visp_started) | [![ubuntu_3rdparty_workflow](https://github.com/lagadic/visp/actions/workflows/ubuntu-3rdparty.yml/badge.svg)](https://github.com/lagadic/visp/actions/workflows/ubuntu-3rdparty.yml)


ViSP is a cross-platform library (Linux, Windows, MacOS, iOS, Android) that allows prototyping and developing applications using visual tracking and visual servoing technics at the heart of the researches done now by Inria <a href="http://team.inria.fr/rainbow">Rainbow team</a> and before 2018 by <a href="http://team.inria.fr/lagadic">Lagadic team</a>. ViSP is able to compute control laws that can be applied to robotic systems. It provides a set of visual features that can be tracked using real time image processing or computer vision algorithms. ViSP provides also simulation capabilities. ViSP can be useful in robotics, computer vision, augmented reality and computer animation. Our <a href="https://www.youtube.com/user/VispTeam">YouTube channel</a> gives an overview of the applications that could be tackled.

#### Citing ViSP
Please cite ViSP in your publications if it helps your research:
```
@article{Marchand05b,
   Author = {Marchand, E. and Spindler, F. and Chaumette, F.},
   Title = {ViSP for visual servoing: a generic software platform with a wide class of robot control skills},
   Journal = {IEEE Robotics and Automation Magazine},
   Volume = {12},
   Number = {4},
   Pages = {40--52},
   Publisher = {IEEE},
   Month = {December},
   Year = {2005}
}
```
To cite the generic model-based tracker:
```
@InProceedings{Trinh18a,
   Author = {Trinh, S. and Spindler, F. and Marchand, E. and Chaumette, F.},
   Title = {A modular framework for model-based visual tracking using edge, texture and depth features},
   BookTitle = {{IEEE/RSJ Int. Conf. on Intelligent Robots and Systems, IROS'18}},
   Address = {Madrid, Spain},
   Month = {October},
   Year = {2018}
}
```

#### Resources
- Homepage: http://visp.inria.fr
- Wiki: https://github.com/lagadic/visp/wiki
- Code documentation: http://visp-doc.inria.fr/doxygen/visp-daily
- Q&A forum: http://gforge.inria.fr/forum/?group_id=397
- Issue tracking: https://github.com/lagadic/visp/issues
- YouTube: https://www.youtube.com/user/VispTeam

#### Contributing

Please read before starting work on a pull request: http://visp.inria.fr/contributing-code/
