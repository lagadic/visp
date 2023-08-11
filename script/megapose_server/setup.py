#!/usr/bin/env python

#from distutils.core import setup
from distutils.cmd import Command
from setuptools import setup
from setuptools.command.install import install

from pathlib import Path
base_json_filename = 'megapose_variables.json'
fixed_json_filename = 'megapose_variables_final.json'

class GenerateFixedJSON(Command):
  description = 'generate JSON file with absolute paths'
  user_options = []
  def initialize_options(self):
    """Use this to set option defaults before parsing."""
    pass

  def finalize_options(self):
    """Code to validate/modify command-line/config input goes here."""
    pass

  def run(self):
    import json
    megapose_variables = None
    with open(base_json_filename, 'r') as variables:
      megapose_variables = json.load(variables)

    megapose_dir = Path(megapose_variables['megapose_dir']).absolute()
    megapose_data_dir = Path(megapose_variables['megapose_data_dir']).absolute()
    megapose_variables['megapose_dir'] = str(megapose_dir)
    megapose_variables['megapose_data_dir'] = str(megapose_data_dir)
    with open(fixed_json_filename, "w") as f:
      f.write(json.dumps(megapose_variables))

class DeleteFixedJSON(Command):
  description = 'Remove JSON file with absolute paths'
  user_options = []
  def initialize_options(self):
    pass

  def finalize_options(self):
    pass

  def run(self):
    path = Path(fixed_json_filename)
    path.unlink()

class InstallCommand(install):
  user_options = install.user_options
  def run(self):
    self.run_command('generate_fixed_json')
    install.run(self)
    #self.run_command('delete_fixed_json')
setup(
    name='megapose_server',
    version='0.2',
    description='Megapose TCP server',
    cmdclass={
        'generate_fixed_json': GenerateFixedJSON,
        'install': InstallCommand,
        'delete_fixed_json': DeleteFixedJSON
    },
    package_dir={'megapose_server': '.'},
    include_package_data=True,
    package_data={'': [fixed_json_filename]},
    author='Samuel Felton',
    author_email='samuel.felton@irisa.fr',
)
