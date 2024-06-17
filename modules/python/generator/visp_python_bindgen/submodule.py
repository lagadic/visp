#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP Python bindings generator
#
#############################################################################

from typing import List, Optional, Dict
from pathlib import Path
import json
import logging

from visp_python_bindgen.header import HeaderFile
from visp_python_bindgen.utils import *
from visp_python_bindgen.gen_report import Report

class Submodule():
  def __init__(self, name: str, include_path: Path, config_file_path: Path, submodule_file_path: Path):
    self.name = name
    self.include_path = include_path
    self.submodule_file_path = submodule_file_path
    self.config_path = config_file_path /  (name + '.json')
    self.config = self._get_config_file_or_create_default(self.config_path)
    self.report = Report(self)
    self.headers = self._get_headers()
    assert self.include_path.exists(), f'Submodule path {self.include_path} not found'

  def set_dependencies_from_dict(self, dict_modules: Dict[str, 'Submodule'], dep_names: List[str]):
    deps = []
    for dep_name in dep_names:
      if dep_name in dict_modules:
        deps.append(dict_modules[dep_name])
    self.dependencies = deps

  def _get_headers(self) -> List[HeaderFile]:
    headers = []
    for include_file in self.include_path.iterdir():
      if not include_file.name.endswith('.h') and not include_file.name.endswith('.hpp'):
        continue
      if self.header_should_be_ignored(include_file.name):
        self.report.add_ignored_header(include_file)
        continue
      headers.append(HeaderFile(include_file, self))
    return headers

  def _get_config_file_or_create_default(self, path: Path) -> Dict:
    if not path.exists():
      default_config = {
        'ignored_headers': [],
        'ignored_classes': [],
        'user_defined_headers': [],
        'classes': {},
        'enums': {},
        'config_includes': []
      }
      with open(path, 'w') as config_file:
        json.dump(default_config, config_file)
      return default_config
    else:
      with open(path, 'r') as config_file:
        config = json.load(config_file)
      for included_config_filename in config.get('config_includes', []):
        included_config_path: Path = path.parent / included_config_filename
        if not included_config_path.exists():
          raise RuntimeError(f'Sub config file {included_config_path} does not exist')
        logging.info(f'Trying to load subconfig file: {included_config_path}')
        with open(included_config_path, 'r') as additional_config_file:
          additional_config = json.load(additional_config_file)
        self.add_subconfig_file(config, additional_config)



      return config

  def add_subconfig_file(self, base_config: Dict[str, any], add_config: Dict[str, any]) -> None:
    ignored_fields = [
      'ignored_headers',
      'ignored_classes',
      'user_defined_headers',
      'functions',
      'enums'
    ]
    for field_name in ignored_fields:
      if field_name in add_config:
        raise RuntimeError(f'All the field in {ignored_fields} cannot be added in the sub configuration file, but found {field_name}')

    if 'classes' not in base_config:
      base_config['classes'] = add_config['classes']
    else:
      base_cls_dict: Dict[str, Dict] = base_config['classes']
      add_cls_dict: Dict[str, Dict] = add_config.get('classes', {})
      for k, v in add_config['classes'].items():
        if k not in base_cls_dict:
          base_cls_dict[k] = v
        else:
          raise RuntimeError(f'The configuration for a single class should be contained in a single dictionary, but found multiple definitions for {k}')



  def set_headers_from_common_list(self, all_headers: List[HeaderFile]) -> None:
    '''
    Set the submodule's headers from a list containing headers from multiple modules
    '''
    new_headers = []
    for header in all_headers:
      if header.submodule.name == self.name:
        new_headers.append(header)
        header.submodule = self
    self.headers = new_headers

  def generate(self) -> None:

    # Sort by dependency level so that generation is in correct order
    module_bindings = BindingsContainer()
    includes = []
    for header in self.headers:
      header.generate_binding_code(module_bindings)
      includes.extend(header.includes)
    submodule_declaration = f'py::module_ submodule = m.def_submodule("{self.name}");\n'
    publicists = module_bindings.get_publicists()
    bindings = module_bindings.get_definitions()
    declarations = module_bindings.get_declarations()
    user_defined_headers = '\n'.join(self.get_user_defined_headers())

    includes_set = set(includes)
    includes_strs = [f'#include {include}' for include in includes_set]
    includes_str = '\n'.join(includes_strs)
    additional_required_headers = '\n'.join(self.get_required_headers())

    format_str = f'''
//#define PYBIND11_DETAILED_ERROR_MESSAGES
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <vector>
#include <map>

/*User-defined headers (e.g. additional bindings)*/
{user_defined_headers}

/*Required headers that are not retrieved in submodule headers (e.g. there are forward definitions but no includes) */
{additional_required_headers}
/*Submodule headers*/
{includes_str}

namespace py = pybind11;

{publicists}

void {self.generation_function_name()}(py::module_ &m) {{
py::options options;
options.disable_enum_members_docstring();

/*
 * Submodule declaration
 */
{submodule_declaration}

/*
 * Class and enums declarations
 */
{declarations}

/*
 * Bindings for methods and enum values
 */
{bindings}
}}
'''
    with open(self.submodule_file_path, 'w') as submodule_file:
      submodule_file.write(format_str)

    logs_path = self.submodule_file_path.parent.parent / 'logs'
    logs_path.mkdir(exist_ok=True)
    self.report.write(logs_path / f'{self.name}_log.json')

  def generation_function_name(self) -> str:
    return f'init_submodule_{self.name}'

  def class_should_be_ignored(self, class_name: str) -> bool:
    if 'ignored_classes' not in self.config:
      return False
    return class_name in self.config['ignored_classes']

  def header_should_be_ignored(self, header_name: str) -> bool:
    if 'ignored_headers' not in self.config:
      return False
    return header_name in self.config['ignored_headers']

  def get_user_defined_headers(self) -> List[str]:
    if 'user_defined_headers' in self.config:
      header_names = self.config['user_defined_headers']
      return [f'#include "{header_name}"' for header_name in header_names]
    return []

  def get_required_headers(self) -> List[str]:
    if 'required_headers' in self.config:
      header_names = self.config['required_headers']
      return [f'#include <{header_name}>' for header_name in header_names]
    return []

  def get_class_config(self, class_name: str) -> Optional[Dict]:
    default_config = {
      'methods': [],
      'operators': [],
      'ignored_attributes': [],
      'use_buffer_protocol': False,
      'additional_bindings': None,
      'ignore_repr': False,
      'is_virtual': False,
      'use_publicist': False,
      'trampoline': None
    }
    if 'classes' not in self.config:
      return default_config
    if class_name not in self.config['classes']:
      return default_config
    default_config.update(self.config['classes'][class_name])
    return default_config
  def get_enum_config(self, enum_name: str) -> Optional[Dict]:
    default_config = {
      'ignore': False,
    }
    if 'enums' not in self.config:
      return default_config
    if enum_name not in self.config['enums']:
      return default_config
    default_config.update(self.config['enums'][enum_name])
    return default_config

  def get_method_config(self, class_name: Optional[str], method, owner_specs, header_mapping) -> Dict:
    res = {
      'ignore': False,
      'use_default_param_policy': False, # Handling
      'param_is_input': None,
      'param_is_output': None,
      'custom_name': None,
      'custom_code': None,
      'keep_alive': None,
      'return_policy': None,
      'returns_ref_ok': False,
    }
    functions_container = None
    keys = ['classes', class_name, 'methods'] if class_name is not None else ['functions']
    tmp = self.config
    for k in keys:
      if k not in tmp:
        return res
      tmp = tmp[k]
    functions_container = tmp
    for function_config in functions_container:
      if method_matches_config(method, function_config, owner_specs, header_mapping):
        res.update(function_config)
        return res
    return res

def get_submodules(config_path: Path, generate_path: Path) -> List[Submodule]:
  modules_input_data = GeneratorConfig.module_data
  result: Dict[str, Submodule] = {}
  for module_data in modules_input_data:
    headers = module_data.headers
    if len(headers) == 0:
      print(f'Module {module_data.name} has no input headers, skipping!')
      continue
    # The headers are already filtered: they should all come from the ViSP source folder (no vpConfig, etc.)
    include_dir = headers[0].parent
    hh = "\n".join(map(lambda s: str(s), headers))
    assert all(map(lambda header_path: header_path.parent == include_dir, headers)), f'Found headers in different directory, this case is not yet handled. Headers = {hh}'
    submodule = Submodule(module_data.name, include_dir, config_path, generate_path / f'{module_data.name}.cpp')
    result[module_data.name] = submodule

  # Second pass to link dependencies
  for module_data in modules_input_data:
    if module_data.name in result:
      result[module_data.name].set_dependencies_from_dict(result, module_data.dependencies)
  return sort_submodules(list(result.values()))

def sort_submodules(submodules: List[Submodule]) -> List[Submodule]:
  res = []
  submodules_tmp = submodules.copy()
  while len(res) < len(submodules):
    can_add = lambda submodule: all(map(lambda dep: dep in res, submodule.dependencies))
    res_round = list(filter(can_add, submodules_tmp))
    submodules_tmp = [submodule for submodule in submodules_tmp if submodule not in res_round]
    res += res_round
  return res
