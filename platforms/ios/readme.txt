The content of this folder allows to build visp3.framework from source for iOS devices.

$ cd ~/<workspace_dir>
$ git clone https://github.com/lagadic/visp.git
$ python visp/platforms/ios/build_framework.py ios

If everything's fine, a few minutes later you will get ~/<workspace_dir>/ios/visp3.framework. You can add this framework to your Xcode projects.

Note that this framework could be built with OpenCV 3rd party. See the tutorials.
