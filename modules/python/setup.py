# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext
from distutils.cmd import Command
from setuptools import setup
from setuptools.command.install import install
from distutils.command import build as build_module
from pathlib import Path
import sys
import subprocess
from glob import glob
__version__ = "0.0.1"

class build(build_module.build):
  def run(self):
    import os
    print('Generating binding code...')
    output = subprocess.run('python generator/generator.py', env=os.environ, shell=True, capture_output=True, check=True)
    print(output.stdout)
    build_module.build.run(self)
# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)
# class GenerateBindings(Command):
#   description = 'generate Pybind11 binding'
#   user_options = []
#   def initialize_options(self):
#     """Use this to set option defaults before parsing."""
#     pass

#   def finalize_options(self):
#     """Code to validate/modify command-line/config input goes here."""
#     pass
#   def run(self):
#     import os
#     print('Generating binding code...')
#     output = subprocess.run('python generator/generator.py', env=os.environ, shell=True, capture_output=True, check=True)
#     print(output.stdout)

# class InstallCommand(install):
#   user_options = install.user_options
#   def run(self):
#     self.run_command('generate_bindings')
#     install.run(self)
#     #self.run_command('delete_fixed_json')

ext_modules = [
    Pybind11Extension("visp",
        ["src/main.cpp", *sorted(glob("src/generated/*.cpp")), *sorted(glob("src/src/*.cpp"))],
        # Example: passing in the version to the compiled code
        define_macros = [('VERSION_INFO', __version__)],
        ),
]
setup(
    name="visp",
    version=__version__,
    author="Samuel Felton",
    author_email="samuel.felton@irisa.fr",
    url="https://github.com/lagadic/visp",
    description="Python wrapper for the Visual Servoing Platform (ViSP) framework",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    setup_requires=[
      "pcpp",
      "cxxheaderparser@git+https://github.com/robotpy/cxxheaderparser#egg=master"
    ],
    cmdclass={"build_ext": build_ext,
              'build': build},
    zip_safe=False,
    python_requires=">=3.9",
)