import sys

try:
  import dlltracer
except:
  print('You need to install dlltracer to use this script.')
  print('Run: pip install dlltracer')
  sys.exit(-1)

# Trace dlls that are required by the Python bindings, using the dlltracer tool.
# This script should be run as administrator.
with dlltracer.Trace(out=sys.stdout):
  import visp
