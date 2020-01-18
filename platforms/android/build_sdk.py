#!/usr/bin/env python

import os, sys, subprocess, argparse, shutil, glob, re
import logging as log
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

class Fail(Exception):
    def __init__(self, text=None):
        self.t = text
    def __str__(self):
        return "ERROR" if self.t is None else self.t

def execute(cmd, shell=False):
    try:
        log.debug("Executing: %s" % cmd)
        log.info('Executing: ' + ' '.join(cmd))
        retcode = subprocess.call(cmd, shell=shell)
        if retcode < 0:
            raise Fail("Child was terminated by signal: %s" % -retcode)
        elif retcode > 0:
            raise Fail("Child returned: %s" % retcode)
    except OSError as e:
        raise Fail("Execution failed: %d / %s" % (e.errno, e.strerror))

def rm_one(d):
    d = os.path.abspath(d)
    if os.path.exists(d):
        if os.path.isdir(d):
            log.info("Removing dir: %s", d)
            shutil.rmtree(d)
        elif os.path.isfile(d):
            log.info("Removing file: %s", d)
            os.remove(d)

def check_dir(d, create=False, clean=False):
    d = os.path.abspath(d)
    log.info("Check dir %s (create: %s, clean: %s)", d, create, clean)
    if os.path.exists(d):
        if not os.path.isdir(d):
            raise Fail("Not a directory: %s" % d)
        if clean:
            for x in glob.glob(os.path.join(d, "*")):
                rm_one(x)
    else:
        if create:
            os.makedirs(d)
    return d

def check_executable(cmd):
    try:
        log.debug("Executing: %s" % cmd)
        result = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        log.debug("Result: %s" % (result+"\n".encode('ascii')).split("\n".encode('ascii'))[0])
        return True
    except Exception as e:
        log.debug('Failed: %s' % e)
        return False

def determine_visp_version(version_hpp_path):
    with open(version_hpp_path, "rt") as f:
        data = f.read()
        major = re.search(r'^set\(VISP_VERSION_MAJOR \"(\d+)\"\)$', data, re.MULTILINE).group(1)
        minor = re.search(r'^set\(VISP_VERSION_MINOR \"(\d+)\"\)$', data, re.MULTILINE).group(1)
        patch = re.search(r'^set\(VISP_VERSION_PATCH \"(\d+)\"\)$', data, re.MULTILINE).group(1)
        revision = re.search(r'^set\(VISP_REVISION \"(\d+)\"\)$', data, re.MULTILINE).group(1)
        return "%(major)s.%(minor)s.%(patch)s-%(revision)s" % locals()

# shutil.move fails if dst exists
def move_smart(src, dst):
    def move_recurse(subdir):
        s = os.path.join(src, subdir)
        d = os.path.join(dst, subdir)
        if os.path.exists(d):
            if os.path.isdir(d):
                for item in os.listdir(s):
                    move_recurse(os.path.join(subdir, item))
            elif os.path.isfile(s):
                shutil.move(s, d)
        else:
            shutil.move(s, d)
    move_recurse('')

# shutil.copytree fails if dst exists
def copytree_smart(src, dst):
    def copy_recurse(subdir):
        s = os.path.join(src, subdir)
        d = os.path.join(dst, subdir)
        if os.path.exists(d):
            if os.path.isdir(d):
                for item in os.listdir(s):
                    copy_recurse(os.path.join(subdir, item))
            elif os.path.isfile(s):
                shutil.copy2(s, d)
        else:
            if os.path.isdir(s):
                shutil.copytree(s, d)
            elif os.path.isfile(s):
                shutil.copy2(s, d)
    copy_recurse('')

#===================================================================================================

class ABI:
    def __init__(self, platform_id, name, toolchain, ndk_api_level = None, cmake_vars = dict()):
        self.platform_id = platform_id # platform code to add to apk version (for cmake)
        self.name = name # general name (official Android ABI identifier)
        self.toolchain = toolchain # toolchain identifier (for cmake)
        self.cmake_vars = dict(
            ANDROID_STL="gnustl_static",
            ANDROID_ABI=self.name,
            ANDROID_PLATFORM_ID=platform_id,
        )
        if toolchain is not None:
            self.cmake_vars['ANDROID_TOOLCHAIN_NAME'] = toolchain
        else:
            self.cmake_vars['ANDROID_TOOLCHAIN'] = 'clang'
            self.cmake_vars['ANDROID_STL'] = 'c++_shared'
        if ndk_api_level:
            self.cmake_vars['ANDROID_NATIVE_API_LEVEL'] = ndk_api_level
        self.cmake_vars.update(cmake_vars)
    def __str__(self):
        return "%s (%s)" % (self.name, self.toolchain)

#===================================================================================================

class Builder:
    def __init__(self, workdir, vispdir, config):
        self.workdir = check_dir(workdir, create=True)
        self.vispdir = check_dir(vispdir)
        self.config = config
        self.libdest = check_dir(os.path.join(self.workdir, "o4a"), create=True, clean=True)
        self.resultdest = check_dir(os.path.join(self.workdir, 'ViSP-android-sdk'), create=True, clean=True)
        self.docdest = check_dir(os.path.join(self.workdir, 'ViSP-android-sdk', 'sdk', 'java', 'javadoc'), create=True, clean=True)
        self.extra_packs = []
        self.visp_version = determine_visp_version(os.path.join(self.vispdir, "CMakeLists.txt"))
        self.use_ccache = False if config.no_ccache else True
        self.cmake_path = self.get_cmake()
        self.ninja_path = self.get_ninja()
        self.debug = True if config.debug else False

    def get_cmake(self):
        if not self.config.use_android_buildtools and check_executable(['cmake', '--version']):
            log.info("Using cmake from PATH")
            return 'cmake'
        # look to see if Android SDK's cmake is installed
        android_cmake = os.path.join(os.environ['ANDROID_SDK'], 'cmake')
        if os.path.exists(android_cmake):
            cmake_subdirs = [f for f in os.listdir(android_cmake) if check_executable([os.path.join(android_cmake, f, 'bin', 'cmake'), '--version'])]
            if len(cmake_subdirs) > 0:
                # there could be more than one - just take the first one
                cmake_from_sdk = os.path.join(android_cmake, cmake_subdirs[0], 'bin', 'cmake')
                log.info("Using cmake from Android SDK: %s", cmake_from_sdk)
                return cmake_from_sdk
        raise Fail("Can't find cmake")

    def get_ninja(self):
        if not self.config.use_android_buildtools and check_executable(['ninja', '--version']):
            log.info("Using ninja from PATH")
            return 'ninja'
        # Android SDK's cmake includes a copy of ninja - look to see if its there
        android_cmake = os.path.join(os.environ['ANDROID_SDK'], 'cmake')
        if os.path.exists(android_cmake):
            cmake_subdirs = [f for f in os.listdir(android_cmake) if check_executable([os.path.join(android_cmake, f, 'bin', 'ninja'), '--version'])]
            if len(cmake_subdirs) > 0:
                # there could be more than one - just take the first one
                ninja_from_sdk = os.path.join(android_cmake, cmake_subdirs[0], 'bin', 'ninja')
                log.info("Using ninja from Android SDK: %s", ninja_from_sdk)
                return ninja_from_sdk
        raise Fail("Can't find ninja")

    def get_toolchain_file(self):
        if not self.config.force_visp_toolchain:
            toolchain = os.path.join(os.environ['ANDROID_NDK'], 'build', 'cmake', 'android.toolchain.cmake')
            if os.path.exists(toolchain):
                return toolchain
        toolchain = os.path.join(SCRIPT_DIR, "android.toolchain.cmake")
        if os.path.exists(toolchain):
            return toolchain
        else:
            raise Fail("Can't find toolchain")

    def get_engine_apk_dest(self, engdest):
        return os.path.join(engdest, "platforms", "android", "service", "engine", ".build")

    def add_extra_pack(self, ver, path):
        if path is None:
            return
        self.extra_packs.append((ver, check_dir(path)))

    def clean_library_build_dir(self):
        for d in ["CMakeCache.txt", "CMakeFiles/", "bin/", "libs/", "lib/", "package/", "install/samples/"]:
            rm_one(d)

    def build_library(self, abi, do_install):
        cmd = ["cmake", "-GNinja"]
        cmake_vars = dict(
            CMAKE_TOOLCHAIN_FILE=self.get_toolchain_file(),
            BUILD_EXAMPLES="OFF",
            BUILD_TESTS="OFF",
	    BUILD_TUTORIALS="OFF",
	    BUILD_DEMOS="OFF",
            BUILD_ANDROID_EXAMPLES="ON",
            INSTALL_ANDROID_EXAMPLES="ON",
        )
        if self.ninja_path != 'ninja':
            cmake_vars['CMAKE_MAKE_PROGRAM'] = self.ninja_path

        if self.debug:
            cmake_vars['CMAKE_BUILD_TYPE'] = "Debug"

        if self.config.extra_modules_path is not None:
            cmd.append("-DVISP_CONTRIB_MODULES_PATH='%s'" % self.config.extra_modules_path)

        if self.use_ccache == True:
            cmd.append("-DNDK_CCACHE=ccache")

        cmake_vars.update(abi.cmake_vars)
        cmd += [ "-D%s='%s'" % (k, v) for (k, v) in cmake_vars.items() if v is not None]
        cmd.append(self.vispdir)
        execute(cmd)
        execute([self.ninja_path, "install" if (self.debug) else "install/strip"])

    def build_javadoc(self):
        classpaths = []
        for dir, _, files in os.walk(os.environ["ANDROID_SDK"]):
            for f in files:
                if f == "android.jar" or f == "annotations.jar":
                    classpaths.append(os.path.join(dir, f))
        cmd = [
            "javadoc",
            "-header", "ViSP %s" % self.visp_version,
            "-nodeprecated",
            "-public",
            '-sourcepath', os.path.join(self.resultdest, 'sdk', 'java', 'src'),
            "-d", self.docdest,
            "-classpath", ":".join(classpaths),
            '-subpackages', 'org.visp',
        ]
        execute(cmd)

    def gather_results(self):
        # Copy all files
        root = os.path.join(self.libdest, "install")
        for item in os.listdir(root):
            src = os.path.join(root, item)
            dst = os.path.join(self.resultdest, item)
            if os.path.isdir(src):
                log.info("Copy dir: %s", item)
                if self.config.force_copy:
                    copytree_smart(src, dst)
                else:
                    move_smart(src, dst)
            elif os.path.isfile(src):
                log.info("Copy file: %s", item)
                if self.config.force_copy:
                    shutil.copy2(src, dst)
                else:
                    shutil.move(src, dst)


#===================================================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Build ViSP for Android SDK')
    parser.add_argument("work_dir", nargs='?', default='.', help="Working directory (and output)")
    parser.add_argument("visp_dir", nargs='?', default=os.path.join(SCRIPT_DIR, '../..'), help="Path to ViSP source dir")
    parser.add_argument('--config', default='ndk-18-api-level-21.config.py', type=str, help="Package build configuration", )
    parser.add_argument('--ndk_path', help="Path to Android NDK to use for build")
    parser.add_argument('--sdk_path', help="Path to Android SDK to use for build")
    parser.add_argument('--use_android_buildtools', action="store_true", help='Use cmake/ninja build tools from Android SDK')
    parser.add_argument("--extra_modules_path", help="Path to extra modules to use for build")
    parser.add_argument('--sign_with', help="Certificate to sign the Manager apk")
    parser.add_argument('--build_doc', action="store_true", help="Build javadoc")
    parser.add_argument('--no_ccache', action="store_true", help="Do not use ccache during library build")
    parser.add_argument('--force_copy', action="store_true", help="Do not use file move during library build (useful for debug)")
    parser.add_argument('--force_visp_toolchain', action="store_true", help="Do not use toolchain from Android NDK")
    parser.add_argument('--debug', action="store_true", help="Build 'Debug' binaries (CMAKE_BUILD_TYPE=Debug)")
    args = parser.parse_args()

    log.basicConfig(format='%(message)s', level=log.DEBUG)
    log.debug("Args: %s", args)

    if args.ndk_path is not None:
        os.environ["ANDROID_NDK"] = args.ndk_path
    if args.sdk_path is not None:
        os.environ["ANDROID_SDK"] = args.sdk_path

    if not 'ANDROID_HOME' in os.environ and 'ANDROID_SDK' in os.environ:
        os.environ['ANDROID_HOME'] = os.environ["ANDROID_SDK"]

    if not 'ANDROID_SDK' in os.environ:
        raise Fail("SDK location not set. Either pass --sdk_path or set ANDROID_SDK environment variable")

    # look for an NDK installed with the Android SDK
    if not 'ANDROID_NDK' in os.environ and 'ANDROID_SDK' in os.environ and os.path.exists(os.path.join(os.environ["ANDROID_SDK"], 'ndk-bundle')):
        os.environ['ANDROID_NDK'] = os.path.join(os.environ["ANDROID_SDK"], 'ndk-bundle')

    if not 'ANDROID_NDK' in os.environ:
        raise Fail("NDK location not set. Either pass --ndk_path or set ANDROID_NDK environment variable")

    if not check_executable(['ccache', '--version']):
        log.info("ccache not found - disabling ccache support")
        args.no_ccache = True

    if os.path.realpath(args.work_dir) == os.path.realpath(SCRIPT_DIR):
        raise Fail("Specify workdir (building from script directory is not supported)")
    if os.path.realpath(args.work_dir) == os.path.realpath(args.visp_dir):
        raise Fail("Specify workdir (building from ViSP source directory is not supported)")

    # Relative paths become invalid in sub-directories
    if args.visp_dir is not None and not os.path.isabs(args.visp_dir):
        args.visp_dir = os.path.abspath(args.visp_dir)
    if args.extra_modules_path is not None and not os.path.isabs(args.extra_modules_path):
        args.extra_modules_path = os.path.abspath(args.extra_modules_path)

    cpath = args.config
    if not os.path.exists(cpath):
        cpath = os.path.join(SCRIPT_DIR, cpath)
        if not os.path.exists(cpath):
            raise Fail('Config "%s" is missing' % args.config)
    with open(cpath, 'r') as f:
        cfg = f.read()
    print("Package configuration:")
    print('=' * 80)
    print(cfg.strip())
    print('=' * 80)

    ABIs = None  # make flake8 happy
    exec(compile(cfg, cpath, 'exec'))

    log.info("Android NDK path: %s", os.environ["ANDROID_NDK"])
    log.info("Android SDK path: %s", os.environ["ANDROID_SDK"])

    builder = Builder(args.work_dir, args.visp_dir, args)

    log.info("Detected ViSP version: %s", builder.visp_version)

    for i, abi in enumerate(ABIs):
        do_install = (i == 0)

        log.info("=====")
        log.info("===== Building library for %s", abi)
        log.info("=====")

        os.chdir(builder.libdest)
        builder.clean_library_build_dir()
        builder.build_library(abi, do_install)

    builder.gather_results()

    if args.build_doc:
        builder.build_javadoc()

    log.info("=====")
    log.info("===== Build finished")
    log.info("=====")
    log.info("SDK location: %s", builder.resultdest)
    log.info("Documentation location: %s", builder.docdest)
