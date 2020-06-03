#!/usr/bin/env python
"""
The script builds visp3.framework for iOS.
The built framework is universal, it can be used to build app and run it on either iOS simulator
or real device.

Usage:
    ./build_framework.py <outputdir>

By cmake conventions (and especially if you work with ViSP repository),
the output dir should not be a subdirectory of ViSP source tree.

Script will create <outputdir> if it's missing, and a few subdirectories:

    <outputdir>
        build/
            *-iPhoneOS/
               [cmake-generated build tree for an iOS device target]
            *-iPhoneSimulator/
               [cmake-generated build tree for iOS simulator]
        visp3.framework/
            [the framework content]

The script does not recompile the library from scratc each time.
However, visp3.framework directory is erased and recreated on each run.

Adding --dynamic parameter will build visp3.framework as App Store dynamic framework. Only iOS 8+ versions are supported.
"""

from __future__ import print_function
import glob, re, os, os.path, shutil, string, sys, argparse, traceback, multiprocessing
from subprocess import check_call, check_output, CalledProcessError

IPHONEOS_DEPLOYMENT_TARGET='8.0'  # default, can be changed via command line options or environment variable

def execute(cmd, cwd = None):
    print("Executing: %s in %s" % (cmd, cwd), file=sys.stderr)
    print('Executing: ' + ' '.join(cmd))
    retcode = check_call(cmd, cwd = cwd)
    if retcode != 0:
        raise Exception("Child returned:", retcode)

def getXCodeMajor():
    ret = check_output(["xcodebuild", "-version"])
    m = re.match(r'Xcode\s+(\d+)\..*', ret, flags=re.IGNORECASE)
    if m:
        return int(m.group(1))
    else:
        raise Exception("Failed to parse Xcode version")

class Builder:
    def __init__(self, visp, contrib, dynamic, bitcodedisabled, exclude,  disable, targets, debug, debug_info):
        self.visp = os.path.abspath(visp)
        self.contrib = None
        if contrib:
            modpath = os.path.join(contrib, "modules")
            if os.path.isdir(modpath):
                self.contrib = os.path.abspath(modpath)
            else:
                print("Note: contrib repository is bad - modules subfolder not found", file=sys.stderr)
        self.dynamic = dynamic
        self.bitcodedisabled = bitcodedisabled
        self.exclude = exclude
        self.disable = disable
        self.targets = targets
        self.debug = debug
        self.debug_info = debug_info

    def getBD(self, parent, t):

        if len(t[0]) == 1:
            res = os.path.join(parent, 'build-%s-%s' % (t[0][0].lower(), t[1].lower()))
        else:
            res = os.path.join(parent, 'build-%s' % t[1].lower())

        if not os.path.isdir(res):
            os.makedirs(res)
        return os.path.abspath(res)

    def _build(self, outdir):
        outdir = os.path.abspath(outdir)
        if not os.path.isdir(outdir):
            os.makedirs(outdir)
        mainWD = os.path.join(outdir, "build")
        dirs = []

        xcode_ver = getXCodeMajor()

        if self.dynamic:
            alltargets = self.targets
        else:
            # if we are building a static library, we must build each architecture separately
            alltargets = []

            for t in self.targets:
                for at in t[0]:
                    current = ( [at], t[1] )

                    alltargets.append(current)

        for t in alltargets:
            mainBD = self.getBD(mainWD, t)
            dirs.append(mainBD)

            cmake_flags = []
            if self.contrib:
                cmake_flags.append("-DVISP_CONTRIB_MODULES_PATH=%s" % self.contrib)
            if xcode_ver >= 7 and t[1] == 'iPhoneOS' and self.bitcodedisabled == False:
                cmake_flags.append("-DCMAKE_C_FLAGS=-fembed-bitcode -Wno-implicit-function-declaration -Wno-logical-op-parentheses -Wno-unused-variable -Wno-parentheses -Wno-sometimes-uninitialized -Wno-unused-parameter -Wno-shorten-64-to-32")
                cmake_flags.append("-DCMAKE_CXX_FLAGS=-fembed-bitcode")
            else:
                cmake_flags.append("-DCMAKE_C_FLAGS=-Wno-implicit-function-declaration -Wno-logical-op-parentheses -Wno-unused-variable -Wno-parentheses -Wno-sometimes-uninitialized -Wno-unused-parameter -Wno-shorten-64-to-32")
            self.buildOne(t[0], t[1], mainBD, cmake_flags)

            if self.dynamic == False:
                self.mergeLibs(mainBD)
        self.makeFramework(outdir, dirs)

    def build(self, outdir):
        try:
            self._build(outdir)
        except Exception as e:
            print("="*60, file=sys.stderr)
            print("ERROR: %s" % e, file=sys.stderr)
            print("="*60, file=sys.stderr)
            traceback.print_exc(file=sys.stderr)
            sys.exit(1)

    def getToolchain(self, arch, target):
        return None

    def getConfiguration(self):
        return "Debug" if self.debug else "Release"

    def getCMakeArgs(self, arch, target):
        args = [
            "cmake",
            "-GXcode",
            "-DAPPLE_FRAMEWORK=ON",
            "-DCMAKE_INSTALL_PREFIX=install",
            "-DCMAKE_BUILD_TYPE=%s" % self.getConfiguration(),
            "-DBUILD_DEMOS=OFF",
            "-DBUILD_EXAMPLES=OFF",
            "-DBUILD_TESTS=OFF",
            "-DBUILD_TUTORIALS=OFF",
        ] + ([
            "-DBUILD_SHARED_LIBS=ON",
            "-DCMAKE_MACOSX_BUNDLE=ON",
            "-DCMAKE_XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED=NO",
        ] if self.dynamic else []) + ([
            "-DBUILD_WITH_DEBUG_INFO=ON"
        ] if self.debug_info else [])

        if len(self.disable) > 0:
            args += ["-DUSE_%s=OFF" % f for f in self.disable]

        return args

    def getBuildCommand(self, archs, target):

        buildcmd = [
            "xcodebuild",
        ]

        if self.dynamic:
            buildcmd += [
                "IPHONEOS_DEPLOYMENT_TARGET=" + os.environ['IPHONEOS_DEPLOYMENT_TARGET'],
                "ONLY_ACTIVE_ARCH=NO",
            ]

            if not self.bitcodedisabled:
                buildcmd.append("BITCODE_GENERATION_MODE=bitcode")

            for arch in archs:
                buildcmd.append("-arch")
                buildcmd.append(arch.lower())
        else:
            arch = ";".join(archs)
            buildcmd += [
                "IPHONEOS_DEPLOYMENT_TARGET=" + os.environ['IPHONEOS_DEPLOYMENT_TARGET'],
                "ARCHS=%s" % arch,
            ]

        buildcmd += [
                "-sdk", target.lower(),
                "-configuration", self.getConfiguration(),
                "-parallelizeTargets",
                "-jobs", str(multiprocessing.cpu_count()),
            ] + (["-target","ALL_BUILD"] if self.dynamic else [])

        return buildcmd

    def getInfoPlist(self, builddirs):
        return os.path.join(builddirs[0], "ios", "Info.plist")

    def buildOne(self, arch, target, builddir, cmakeargs = []):
        # Run cmake
        toolchain = self.getToolchain(arch, target)
        cmakecmd = self.getCMakeArgs(arch, target) + \
            (["-DCMAKE_TOOLCHAIN_FILE=%s" % toolchain] if toolchain is not None else [])
        if target.lower().startswith("iphoneos"):
            cmakecmd.append("-DENABLE_NEON=ON")
        cmakecmd.append(self.visp)
        cmakecmd.extend(cmakeargs)
        execute(cmakecmd, cwd = builddir)

        # Clean and build
        clean_dir = os.path.join(builddir, "install")
        if os.path.isdir(clean_dir):
            shutil.rmtree(clean_dir)
        buildcmd = self.getBuildCommand(arch, target)
        execute(buildcmd + ["-target", "ALL_BUILD", "build"], cwd = builddir)
        execute(["cmake", "-DBUILD_TYPE=%s" % self.getConfiguration(), "-P", "cmake_install.cmake"], cwd = builddir)

    def mergeLibs(self, builddir):
        res = os.path.join(builddir, "lib", self.getConfiguration(), "libvisp_merged.a")
        libs = glob.glob(os.path.join(builddir, "install", "lib", "*.a"))
        libs3 = glob.glob(os.path.join(builddir, "install", "share", "ViSP", "3rdparty", "lib", "*.a"))
        print("Merging libraries:\n\t%s" % "\n\t".join(libs + libs3), file=sys.stderr)
        execute(["libtool", "-static", "-o", res] + libs + libs3)

    def makeFramework(self, outdir, builddirs):
        name = "visp3"

        # set the current dir to the dst root
        framework_dir = os.path.join(outdir, "%s.framework" % name)
        if os.path.isdir(framework_dir):
            shutil.rmtree(framework_dir)
        os.makedirs(framework_dir)

        if self.dynamic:
            dstdir = framework_dir
            libname = "visp3.framework/visp3"
        else:
            dstdir = os.path.join(framework_dir, "Versions", "A")
            libname = "libvisp_merged.a"

        # copy headers from one of build folders
        shutil.copytree(os.path.join(builddirs[0], "install", "include", "visp3"), os.path.join(dstdir, "Headers"))

        # make universal static lib
        libs = [os.path.join(d, "lib", self.getConfiguration(), libname) for d in builddirs]
        lipocmd = ["lipo", "-create"]
        lipocmd.extend(libs)
        lipocmd.extend(["-o", os.path.join(dstdir, name)])
        print("Creating universal library from:\n\t%s" % "\n\t".join(libs), file=sys.stderr)
        execute(lipocmd)

        # dynamic framework has different structure, just copy the Plist directly
        if self.dynamic:
            resdir = dstdir
            shutil.copyfile(self.getInfoPlist(builddirs), os.path.join(resdir, "Info.plist"))
        else:
            # copy Info.plist
            resdir = os.path.join(dstdir, "Resources")
            os.makedirs(resdir)
            shutil.copyfile(self.getInfoPlist(builddirs), os.path.join(resdir, "Info.plist"))

            # make symbolic links
            links = [
                (["A"], ["Versions", "Current"]),
                (["Versions", "Current", "Headers"], ["Headers"]),
                (["Versions", "Current", "Resources"], ["Resources"]),
                (["Versions", "Current", name], [name])
            ]
            for l in links:
                s = os.path.join(*l[0])
                d = os.path.join(framework_dir, *l[1])
                os.symlink(s, d)

class iOSBuilder(Builder):

    def getToolchain(self, arch, target):
        toolchain = os.path.join(self.visp, "platforms", "ios", "cmake", "Toolchains", "Toolchain-%s_Xcode.cmake" % target)
        return toolchain

    def getCMakeArgs(self, arch, target):
        arch = ";".join(arch)

        args = Builder.getCMakeArgs(self, arch, target)
        args = args + [
            '-DIOS_ARCH=%s' % arch
        ]
        return args

if __name__ == "__main__":
    folder = os.path.abspath(os.path.join(os.path.dirname(sys.argv[0]), "../.."))
    parser = argparse.ArgumentParser(description='The script builds visp3.framework for iOS.')
    parser.add_argument('out', metavar='OUTDIR', help='folder to put built framework')
    parser.add_argument('--visp', metavar='DIR', default=folder, help='folder with visp repository (default is "../.." relative to script location)')
    parser.add_argument('--contrib', metavar='DIR', default=None, help='folder with visp_contrib repository (default is "None" - build only main framework)')
    parser.add_argument('--without', metavar='MODULE', default=[], action='append', help='ViSP modules to exclude from the framework')
    parser.add_argument('--disable', metavar='FEATURE', default=[], action='append', help='ViSP features to disable (add USE_*=OFF)')
    parser.add_argument('--dynamic', default=False, action='store_true', help='build dynamic framework (default is "False" - builds static framework)')
    parser.add_argument('--disable-bitcode', default=False, dest='bitcodedisabled', action='store_true', help='disable bitcode (enabled by default)')
    args = parser.parse_args()
    parser.add_argument('--iphoneos_deployment_target', default=os.environ.get('IPHONEOS_DEPLOYMENT_TARGET', IPHONEOS_DEPLOYMENT_TARGET), help='specify IPHONEOS_DEPLOYMENT_TARGET')
    parser.add_argument('--iphoneos_archs', default='armv7,armv7s,arm64', help='select iPhoneOS target ARCHS')
    parser.add_argument('--iphonesimulator_archs', default='i386,x86_64', help='select iPhoneSimulator target ARCHS')
    parser.add_argument('--debug', default=False, dest='debug', action='store_true', help='Build "Debug" binaries (disabled by default)')
    parser.add_argument('--debug_info', default=False, dest='debug_info', action='store_true', help='Build with debug information (useful for Release mode: BUILD_WITH_DEBUG_INFO=ON)')
    args = parser.parse_args()

    os.environ['IPHONEOS_DEPLOYMENT_TARGET'] = args.iphoneos_deployment_target
    print('Using IPHONEOS_DEPLOYMENT_TARGET=' + os.environ['IPHONEOS_DEPLOYMENT_TARGET'])
    iphoneos_archs = args.iphoneos_archs.split(',')
    print('Using iPhoneOS ARCHS=' + str(iphoneos_archs))
    iphonesimulator_archs = args.iphonesimulator_archs.split(',')
    print('Using iPhoneSimulator ARCHS=' + str(iphonesimulator_archs))

    b = iOSBuilder(args.visp, args.contrib, args.dynamic, args.bitcodedisabled, args.without, args.disable,
        [
            (iphoneos_archs, "iPhoneOS"),
        ] if os.environ.get('BUILD_PRECOMMIT', None) else
        [
            (iphoneos_archs, "iPhoneOS"),
            (iphonesimulator_archs, "iPhoneSimulator"),
        ], args.debug, args.debug_info)
    b.build(args.out)
