from pathlib import Path
from setuptools import setup
from setuptools.command.install import install

import subprocess
import sys


def find_stubs(path: Path):
  print([str(p.relative_to('.')) for p in path.rglob('*.pyi')])
  return [str(p.relative_to('.')) for p in path.rglob('*.pyi')]

class CustomInstall(install):
  def run(self):
    subprocess.run(['pybind11-stubgen', '-o', '.', '--ignore-all-errors', 'visp'], check=True)
    install.run(self)

setup(
  name='visp-stubs',
  version='0.0.4',
  author="Samuel Felton",
  author_email="samuel.felton@irisa.fr",
  description="Python stubs for the ViSP wrapper",
  zip_safe=False,
  include_package_data=True,
  # package_data={'visp-stubs': find_stubs(path=Path('.'))},
  setup_requires=[
      "pybind11-stubgen>=2.1",
      "visp"
  ],
  cmdclass={'install': CustomInstall},
  python_requires=">=3.7",
)