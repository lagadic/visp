'''
This code is directly adapted from proxsuite_nlp, see:

- https://github.com/Simple-Robotics/proxsuite-nlp/blob/main/bindings/python/proxsuite_nlp/windows_dll_manager.py
- https://github.com/Simple-Robotics/proxsuite-nlp/blob/main/bindings/python/proxsuite_nlp/__init__.py

On windows, since Python 3.8, dll directories must be explicetly specified (cannot go through path), see
- https://docs.python.org/3/library/os.html#os.add_dll_directory

'''


import os
import sys
import contextlib


def get_dll_paths():
    # Assumes that we are in a conda environment, and that ViSP DLLs and the dependencies are installed in this environment
    # For the choice of defaults: see https://peps.python.org/pep-0250/#implementation
    DEFAULT_DLL_PATHS = [
      '..\\..\\..\\..\\bin', # when current folder is lib/python-version/site-packages/package
      '..\\..\\..\\bin', # when current folder is lib/site-packages/package
    ]
    # If we have a different setup, the user should specify their own paths
    visp_user_defined_dll_paths = os.getenv("VISP_WINDOWS_DLL_PATH")
    dll_paths = [
      os.path.join(os.path.dirname(__file__), dll_path) for dll_path in DEFAULT_DLL_PATHS
    ]

    if visp_user_defined_dll_paths is not None:
      dll_paths.extend(visp_user_defined_dll_paths.split(os.pathsep))

    return dll_paths

class PathManager(contextlib.AbstractContextManager):
    """Restore PATH state after importing Python module"""

    def add_dll_directory(self, dll_dir: str):
        os.environ["PATH"] += os.pathsep + dll_dir

    def __enter__(self):
        self.old_path = os.environ["PATH"]
        return self

    def __exit__(self, *exc_details):
        os.environ["PATH"] = self.old_path


class DllDirectoryManager(contextlib.AbstractContextManager):
    """Restore DllDirectory state after importing Python module"""

    def add_dll_directory(self, dll_dir: str):
        # add_dll_directory can fail on relative path and non
        # existing path.
        # Since we don't know all the fail criterion we just ignore
        # thrown exception
        try:
            self.dll_dirs.append(os.add_dll_directory(dll_dir))
        except OSError:
            pass

    def __enter__(self):
        self.dll_dirs = []
        return self

    def __exit__(self, *exc_details):
        for d in self.dll_dirs:
            d.close()


def build_directory_manager():
    if sys.version_info >= (3, 8):
        return DllDirectoryManager()
    else: # Below 3.8, the path variable is used to search for DLLs
        return PathManager()
