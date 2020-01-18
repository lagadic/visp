How to select the proper version of ViSP Manager
--------------------------------------------------

Since version 1.7 several packages of ViSP Manager are built. Every package is targeted for some
specific hardware platform and includes corresponding ViSP binaries. So, in all cases ViSP
Manager uses built-in version of ViSP. The new package selection logic in most cases simplifies
ViSP installation on end user devices. In most cases ViSP Manager may be installed automatically
from Google Play.

If Google Play is not available (i.e. on emulator, developer board, etc), you can install it
manually using adb tool:

    adb install <path-to-ViSP-sdk>/apk/ViSP_<version>_Manager_<app_version>_<platform>.apk

Example: ViSP_3.4.0-dev_Manager_3.40_armeabi-v7a.apk

Use the list of platforms below to determine proper ViSP Manager package for your device:

- armeabi (ARMv5, ARMv6)
- armeabi-v7a (ARMv7-A + NEON)
- arm64-v8a
- mips
- mips64
- x86
- x86_64
