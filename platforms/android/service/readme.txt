How to select the proper version of VISP Manager
--------------------------------------------------

The new package selection logic in most cases simplifies VISP installation on end user devices. In most cases VISP Manager may be installed automatically
from Google Play.

If Google Play is not available (i.e. on emulator, developer board, etc), you can install it
manually using adb tool:

    adb install <path-to-VISP-sdk>/apk/VISP_<version>_Manager_<app_version>_<platform>.apk

Example: VISP_3.4.0-dev_Manager_3.40_armeabi-v7a.apk

Use the list of platforms below to determine proper VISP Manager package for your device:

- armeabi-v7a (ARMv7-A + NEON)
- arm64-v8a
- x86
- x86_64
