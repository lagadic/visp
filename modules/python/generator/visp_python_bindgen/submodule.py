from typing import List, Optional, Dict
from pathlib import Path
import json

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
        'enums': {}
      }
      with open(path, 'w') as config_file:
        json.dump(default_config, config_file)
      return default_config
    else:
      with open(path, 'r') as config_file:
        config = json.load(config_file)
      return config

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

    header_code = []
    declarations = []
    includes = []
    for header in self.headers:
      header.generate_binding_code()
      header_code.append(header.binding_code)
      declarations.extend(header.declarations)
      includes.extend(header.includes)
    includes_set = set(includes)
    submodule_declaration = f'py::module_ submodule = m.def_submodule("{self.name}");\n'
    bindings = '\n'.join(header_code)
    declarations = '\n'.join(declarations)
    user_defined_headers = '\n'.join(self.get_user_defined_headers())
    includes_strs = [f'#include {include}' for include in includes_set]
    includes_str = '\n'.join(includes_strs)
    additional_required_headers = '\n'.join(self.get_required_headers())

    format_str = f'''
#define PYBIND11_DETAILED_ERROR_MESSAGES
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <map>

/*User-defined headers (e.g. additional bindings)*/
{user_defined_headers}
/*Required headers that are not retrieved in submodule headers (e.g. there are forward definitions but no includes) */
{additional_required_headers}
/*Submodule headers*/
{includes_str}

namespace py = pybind11;


void {self.generation_function_name()}(py::module_ &m) {{

/*
Submodule declaration
*/
{submodule_declaration}

/*
Class and enums declarations
*/
{declarations}


/*
Bindings for methods and enum values
*/
{bindings}
}}
'''
    with open(self.submodule_file_path, 'w') as submodule_file:
      submodule_file.write(format_str)

    logs_path = self.submodule_file_path.parent / 'logs'
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
      'use_buffer_protocol': False,
      'additional_bindings': None,
      'ignore_repr': False,
      'is_virtual': False
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
      'use_default_param_policy': False,
      'param_is_input': None,
      'param_is_output': None,
      'custom_name': None,
      'custom_code': None
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

    #import sys; sys.exit()
    return res


def get_submodules(config_path: Path, generate_path: Path) -> List[Submodule]:
  modules = ['core', 'imgproc', 'vision', 'visual_features', 'vs', 'sensor', 'io', 'detection', 'robot', 'gui']
  result = []
  for module in modules:
    result.append(Submodule(module, Path(f'/home/sfelton/software/visp-sfelton/modules/{module}/include/visp3/{module}'), config_path, generate_path / f'{module}.cpp'))
  tracking_modules = ['tt', 'tt_mi', 'klt', 'me', 'blob', ('dnn', 'dnn_tracker'), 'mbt']
  for module in tracking_modules:
    if isinstance(module, str):
      result.append(Submodule(module, Path(f'/home/sfelton/software/visp-sfelton/modules/tracker/{module}/include/visp3/{module}'), config_path, generate_path / f'{module}.cpp'))
    else:
      result.append(Submodule(module[1], Path(f'/home/sfelton/software/visp-sfelton/modules/tracker/{module[0]}/include/visp3/{module[1]}'), config_path, generate_path / f'{module[1]}.cpp'))


  return result
