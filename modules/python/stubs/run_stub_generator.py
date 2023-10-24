from shutil import copy
from pathlib import Path
import subprocess
import sys
import argparse

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--output-root', type=str, help='Path where to save the stubs')

  args = parser.parse_args()
  output_root = Path(args.output_root)
  assert output_root.exists()
  bin_folder = Path(sys.executable).parent
  stubgen_entry_point = bin_folder / 'pybind11-stubgen'
  assert stubgen_entry_point.exists()

  subprocess.run([str(stubgen_entry_point), '-o', str(output_root.absolute()), '--ignore-all-errors', '_visp'], check=True)

  # Generate stubs for the bindings (C++ side) and mock it so that they appear in the true 'visp' package
  p = Path('./_visp')
  target_path = Path('./visp-stubs')
  target_path.mkdir(exist_ok=True)
  for pyi_file in p.iterdir():
    if pyi_file.name.endswith('.pyi'):
      copy(pyi_file, target_path / pyi_file.name) # Copy replace old files