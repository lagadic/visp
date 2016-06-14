The content of this folder allows to build visp3.framework from source for OSX.

$ cd ~/<workspace_dir>
$ git clone https://github.com/lagadic/visp.git
$ python visp/platforms/osx/build_framework.py osx

If everything's fine, a few minutes later you will get ~/<workspace_dir>/osx/visp3.framework. You can add this framework to your Xcode projects.

Note that this framework is built with the 3rd parties that are detected during the build process. In Xcode you should than add them manually (includes/libraries) since they are not embedded in visp3.framework.
