from pathlib import Path
from setuptools import setup
from setuptools.command.install import install

import subprocess
import sys


class CustomInstall(install):
  def run(self):
    stubs_path = Path('.')
    stubs_path.mkdir(exist_ok=True)
    subprocess.run(['pybind11-stubgen', 'visp', '-o', '.', '--ignore-all-errors'], check=True)
    install.run(self)

setup(
  name='visp_stubs',
  version='0.0.3',
  author="Samuel Felton",
  author_email="samuel.felton@irisa.fr",
  description="Python stubs for the ViSP wrapper",
  zip_safe=False,
  include_package_data=True,
  # package_data={'visp_stubs': [*find_stubs(path=stubs_path)]},
  setup_requires=[
      "pybind11-stubgen",
  ],
  python_requires=">=3.7",
)