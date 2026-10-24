# ViSP third-party libraries and licenses

## ViSP license

ViSP is distributed under the **GNU General Public License, version 2 or (at your option) any later version**
(GPL-2.0-or-later). See [LICENSE.txt](LICENSE.txt) for the full text.

The GPL is a *copyleft* license. If you distribute a program that is derived from ViSP, or that links with ViSP
(statically or dynamically), the whole program is considered a derivative work and must itself be distributed under
the terms of the GPL. In practice, this means that you must:

- distribute the complete corresponding source code of your program, or offer to provide it;
- license your program under the GPL, giving recipients the same rights to use, study, modify and redistribute it;
- keep the copyright notices and the license text with the program.

These obligations apply when the software is distributed. Internal use without distribution does not trigger them.

If these constraints do not suit your needs, for example if you want to integrate ViSP into a closed-source product,
you can lift them by purchasing a **professional (commercial) license**. To do so, contact the ViSP authors at
[visp@inria.fr](mailto:visp@inria.fr).

## About this file

This file lists all the third-party libraries that ViSP can interface with, together with their licenses.

**Using these libraries is optional.** ViSP builds without any of them. Each one is enabled at configuration time
through a CMake option (`USE_<lib>` for external libraries, `WITH_<lib>` for libraries embedded in ViSP), usually
switched on automatically when the library is found on the system.

Some of these libraries come with licenses that may conflict with the license you choose for ViSP, see
[Points of attention](#points-of-attention) below.

> [!NOTE]
> The licenses of the external libraries are given for information only and may change between versions.
> Always check the license of the exact version you use.

## 1. Libraries embedded in ViSP

These libraries are shipped with ViSP source code, mostly in the [3rdparty](3rdparty) folder.

| Library | CMake option | Usage in ViSP | License |
|---|---|---|---|
| AprilTag | (built-in) | Tag detection | BSD-2-Clause |
| ATI DAQ F/T | `WITH_ATIDAQ` | ATI force/torque sensor | MIT |
| Catch2 | (tests only) | Unit tests | BSL-1.0 |
| CLAPACK | `WITH_LAPACK` | Built-in linear algebra | Modified BSD-3-Clause (Univ. of Tennessee) |
| Clipper | `WITH_CLIPPER` | Polygon clipping | BSL-1.0 |
| miniz | `WITH_MINIZ` | npz I/O | MIT |
| Pololu Maestro | `WITH_POLOLU` | Pololu servo motors | MIT |
| pugixml | `WITH_PUGIXML` | XML parsing | MIT |
| qbdevice | `WITH_QBDEVICE` | qbrobotics soft hand | BSD-3-Clause |
| Reflex TakkTile2 | `WITH_TAKKTILE2` | RightHand Robotics hand | Apache-2.0 |
| Simd | `WITH_SIMDLIB` | SIMD optimizations | MIT |
| stb_image | `WITH_STBIMAGE` | Image I/O | Public domain or MIT (at your choice) |
| TinyEXR | `WITH_TINYEXR` | EXR image I/O | BSD-3-Clause (includes OpenEXR code, BSD-3-Clause) |

## 2. Optional external libraries

These libraries are not shipped with ViSP and have to be installed separately.

| Library | CMake option | Usage in ViSP | License |
|---|---|---|---|
| **Linear algebra / computation** | | | |
| Eigen3 | `USE_EIGEN3` | Linear algebra | MPL-2.0 |
| Intel MKL | `USE_MKL` | BLAS/LAPACK | Intel Simplified Software License (proprietary, free of charge) |
| OpenBLAS | `USE_OPENBLAS` | BLAS/LAPACK | BSD-3-Clause |
| ATLAS | `USE_ATLAS` | BLAS/LAPACK | BSD-style |
| Netlib LAPACK | `USE_NETLIB` | BLAS/LAPACK | Modified BSD-3-Clause |
| GSL | `USE_GSL` | BLAS/LAPACK | ⚠️ **GPL-3.0** |
| OpenMP | `USE_OPENMP` | Parallelization | Compiler runtime (libgomp: GPL-3.0 with runtime exception; libomp: Apache-2.0 with LLVM exception) |
| **Vision / 3D** | | | |
| OpenCV | `USE_OPENCV` | Computer vision | Apache-2.0 (>= 4.5), BSD-3-Clause before |
| PCL | `USE_PCL` | Point clouds | BSD-3-Clause |
| VTK | `USE_VTK` | PCL dependency | BSD-3-Clause |
| Panda3D | `USE_PANDA3D` | 3D rendering (RBT tracker) | BSD-3-Clause |
| Ogre3D | `USE_OGRE` | Augmented reality | MIT (Ogre >= 1.7) |
| OIS | `USE_OIS` | Keyboard/mouse input for Ogre | zlib |
| Coin3D | `USE_COIN3D` | Simulator / augmented reality | BSD-3-Clause |
| SoQt / SoWin / SoXt | `USE_SOQT` / `USE_SOWIN` / `USE_SOXT` | Coin3D GUI bindings | BSD-3-Clause |
| TensorRT | `USE_TENSORRT` | DNN inference | NVIDIA proprietary |
| **Display / GUI** | | | |
| X11 | `USE_X11` | Display | MIT/X11 |
| GTK2 | `USE_GTK2` | Display | LGPL-2.1 |
| Qt | `USE_QT` | GUI for SoQt | LGPL-3.0 / GPL / commercial |
| GDI | `USE_GDI` | Windows display | Microsoft system library (proprietary) |
| Direct3D | `USE_DIRECT3D` | Windows display | Microsoft system library (proprietary) |
| **Images / formats / I/O** | | | |
| libjpeg / libjpeg-turbo | `USE_JPEG` | JPEG I/O | IJG / BSD-3-Clause / zlib |
| libpng | `USE_PNG` | PNG I/O | libpng License (permissive) |
| zlib | `USE_ZLIB` | Compression | zlib |
| libxml2 | `USE_XML2` | XML parsing | MIT |
| nlohmann/json | `USE_NLOHMANN_JSON` | JSON serialization | MIT |
| FFmpeg | `USE_FFMPEG` | Video I/O | ⚠️ LGPL-2.1-or-later (GPL-2.0-or-later when built with `--enable-gpl`) |
| ZBar | `USE_ZBAR` | QR code detection | LGPL-2.1 |
| libdmtx | `USE_DMTX` | Data Matrix detection | BSD-2-Clause |
| iconv | `USE_ICONV` | Character encoding conversion | LGPL (GNU libiconv / glibc) or system libc |
| **Cameras / sensors** | | | |
| libdc1394 | `USE_DC1394` | FireWire cameras | LGPL-2.1 |
| V4L2 (libv4l2) | `USE_V4L2` | Linux cameras | LGPL-2.1 |
| librealsense2 | `USE_REALSENSE2` | Intel RealSense cameras | Apache-2.0 |
| libfreenect | `USE_LIBFREENECT` | Kinect | Apache-2.0 or GPL-2.0 (at your choice) |
| libusb-1.0 | `USE_LIBUSB_1` | libfreenect dependency | LGPL-2.1 |
| Occipital Structure SDK | `USE_OCCIPITAL_STRUCTURE` | Structure Core sensor | Proprietary |
| Basler Pylon | `USE_PYLON` | Basler cameras | Proprietary |
| FLIR FlyCapture | `USE_FLYCAPTURE` | PointGrey/FLIR cameras | Proprietary |
| IDS uEye | `USE_UEYE` | IDS cameras | Proprietary |
| Comedi | `USE_COMEDI` | Data acquisition boards | LGPL-2.1 |
| **Robots / hardware** | | | |
| libfranka | `USE_FRANKA` | Franka Emika Panda robot | Apache-2.0 |
| ur_rtde | `USE_UR_RTDE` | Universal Robots | MIT |
| MAVSDK | `USE_MAVSDK` | PX4/MAVLink drones | BSD-3-Clause |
| Parrot ARSDK | `USE_ARSDK` | Parrot Bebop 2 drone | BSD-3-Clause |
| ARIA | `USE_ARIA` | Pioneer mobile robots | ⚠️ GPL-2.0 (or commercial) |
| Kinova Jaco SDK | `USE_JACOSDK` | Kinova arms | Proprietary |
| FLIR PTU SDK | `USE_FLIRPTUSDK` | FLIR pan-tilt unit | Proprietary |
| Biclops | `USE_BICLOPS` | Biclops pan-tilt head | Proprietary (TRACLabs) |
| PTU-46 | `USE_PTU46` | Directed Perception pan-tilt unit | Vendor driver, check license |
| Haption Virtuose | `USE_VIRTUOSE` | Haptic device | Proprietary |
| FT IIT SDK | `USE_FTIITSDK` | IIT force/torque sensor | Proprietary (IIT) |
| Afma6 / Viper650 / Viper850 | `USE_AFMA6`, `USE_VIPER650`, `USE_VIPER850` | Inria robots | Inria internal drivers (not distributed) |
| YARP | `USE_YARP` | Robotics middleware | BSD-3-Clause |
| **Motion capture** | | | |
| Qualisys C++ SDK | `USE_QUALISYS` | Qualisys motion capture | MIT |
| Vicon DataStream SDK | `USE_VICON` | Vicon motion capture | Proprietary |
| **System** | | | |
| pthreads / Threads | `USE_THREADS` | Multi-threading | System libc (glibc: LGPL-2.1) |
| librt | `USE_RT` | Real-time extensions | System libc (glibc: LGPL-2.1) |
| libdl | `USE_DL` | Dynamic loading | System libc (glibc: LGPL-2.1) |
| **Bindings / documentation** | | | |
| pybind11 | `USE_PYBIND11` | Python bindings | BSD-3-Clause |
| MathJax | `USE_MATHJAX` | Doxygen documentation | Apache-2.0 |

## Points of attention

- **Strong copyleft (GPL) libraries**: GSL (GPL-3.0), ARIA (GPL-2.0), FFmpeg when built in GPL mode, and libfreenect
  when used under its GPL option. Linking ViSP with them is fine under the GPL license of ViSP (note that GPL-3.0
  libraries such as GSL are only compatible thanks to the "or any later version" clause), but they are **not
  compatible with a professional license** of ViSP: disable them if you want to keep your product closed-source.
- **LGPL libraries** (GTK2, Qt, libdc1394, libv4l2, ZBar, libusb, Comedi, FFmpeg in LGPL mode, glibc): they can be
  used with a professional license of ViSP as long as they are linked dynamically and the LGPL requirements are met
  (e.g. allowing the user to replace the LGPL library).
- **Proprietary SDKs** (Intel MKL, TensorRT, Pylon, FlyCapture, uEye, Occipital, robot and Vicon SDKs): they come
  with their own license terms. Distributing a GPL binary of ViSP linked with such SDKs may raise GPL compatibility
  issues and should be checked case by case.
