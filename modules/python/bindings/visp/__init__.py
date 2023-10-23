
import sys
# import os
# sys.path.append(os.path.dirname(__file__))
# print(sys.path)


from .bindings import *
import _visp

# Fake module names
for k in _visp.__dict__:
  from types import ModuleType
  if isinstance(_visp.__dict__[k], ModuleType):
    sys.modules[f'{__name__}.{k}'] = f'{__name__}._visp.{k}'
