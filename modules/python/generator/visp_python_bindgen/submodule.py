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
    module_bindings = BindingsContainer()
    includes = []
    for header in self.headers:
      header.generate_binding_code(module_bindings)
      includes.extend(header.includes)
    submodule_declaration = f'py::module_ submodule = m.def_submodule("{self.name}");\n'
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
      'ignored_attributes': [],
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
  modules_input_data = GeneratorConfig.module_data
  result: Dict[str, Submodule] = {}
  for module_data in modules_input_data:
    headers = module_data.headers
    if len(headers) == 0:
      print(f'Module {module_data.name} has no input headers, skipping!')

      continue
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

  print(list(map(lambda sub: sub.name, res)))
  return res
