from typing import List, Optional, Set, Tuple
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData
from pathlib import Path
import json

def get_type(param: types.DecoratedType, owner_specs: List[str]) -> str:
  pass

def filter_includes(include_names: Set[str]) -> List[str]:
  result = []
  for include_name in include_names:
    if include_name.startswith('"') and include_name.endswith('"'):
      continue
    forbidden_strs = ['winsock', '<io.h>']
    forbidden_found = False
    for forbidden_str in forbidden_strs:
      if forbidden_str in include_name:
        forbidden_found = True
        break
    if forbidden_found:
      continue

    result.append(include_name)
  return result


class Visitor(CxxVisitor):
  def __init__(self):
    self.result = ''
    self.includes = []
    self.current_class: Optional[ClassBlockState] = None

  def on_class_start(self, state: ClassBlockState) -> None:
    print(state.class_decl.typename)
    if self.current_class is not None:
      pass
    self.current_class = state

    if self.current_class.class_decl.template is None:
      name_cpp = '::'.join([seg.name for seg in self.current_class.class_decl.typename.segments])
      name_python = name_cpp.replace('vp', '')
      # if name_cpp == 'vpColVector':
      self.result += f'py::class_ py{name_python} = py::class_<{name_cpp}>(submodule, "{name_python}");'

  # def on_include(self, state: State, filename: str) -> None:
    #self.includes.append(filename)


  def on_class_end(self, state: ClassBlockState) -> None:
    self.current_class = None



class Submodule():
  def __init__(self, name: str, include_path: Path, submodule_file_path: Path):
    self.name = name
    self.include_path = include_path
    self.submodule_file_path = submodule_file_path
    self.config_path = Path('config') /  (name + '.json')
    with open(self.config_path, 'r') as config_file:
      self.config = json.load(config_file)

    assert self.include_path.exists(), f'Submodule path {self.include_path} not found'

  def generate(self) -> None:
    header_code = []
    includes = []
    for include_file in self.include_path.iterdir():
      if not include_file.name.endswith('.h') and not include_file.name.endswith('.hpp'):
        continue
      if include_file.name in self.config['ignored_headers']:
        continue
      header = HeaderFile(include_file, self)
      header_code.append(header.binding_code)
      includes.extend(header.includes)
    includes_set = filter_includes(set(includes))

    body = f'py::module_ submodule = m.def_submodule("{self.name}");\n' + '\n'.join(header_code)
    includes_strs = [f'#include {include}' for include in includes_set]
    includes_str = '\n'.join(includes_strs)
    format_str = f'''
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <map>
{includes_str}

namespace py = pybind11;


void {self.generation_function_name()}(py::module_ &m) {{
  {body}
}}
'''
    with open(self.submodule_file_path, 'w') as submodule_file:
      submodule_file.write(format_str)

  def generation_function_name(self) -> str:
    return f'init_submodule_{self.name}'



def get_submodules(include_path: Path, generate_path: Path) -> List[Submodule]:
  return [
    Submodule('core', include_path / 'core', generate_path / 'core.cpp'),
    # Submodule('vs', include_path / 'vs', generate_path / 'vs.cpp')

  ]

class HeaderFile():
  def __init__(self, path: Path, submodule: Submodule):
    self.path = path
    self.submodule = submodule
    content = self.run_preprocessor()
    self.includes = [f'<visp3/{self.submodule.name}/{self.path.name}>']
    self.binding_code = None
    self.generate_binding_code(content)

  def run_preprocessor(self):
    tmp_file_path = self.submodule.submodule_file_path.parent / "tmp" / self.path.name
    argv = [
      '',
      # '-N', 'VISP_EXPORT',
      '-D', 'visp_deprecated=""',
      '-I', '/usr/local/include',
      '--passthru-includes', "^((?!vpConfig.h).)*$",
      '--passthru-unfound-includes',
      '--passthru-comments',
      '--line-directive', '',
      '-o', f'{tmp_file_path}',
      str(self.path.absolute())
    ]
    pcpp.CmdPreprocessor(argv)
    preprocessed_header_content = None
    with open(tmp_file_path, 'r') as header_file:
      preprocessed_header_content = '\n'.join(header_file.readlines())
    return preprocessed_header_content
  def generate_binding_code(self, content: str) -> None:
    parsed_data = parse_string(content)
    self.binding_code = self.parse_data(parsed_data)


  def parse_data(self, data: ParsedData):
    result = ''

    print(f'Parsing namespace "{data.namespace.name}"')
    for cls in data.namespace.classes:
      print(f'Parsing class "{cls.class_decl.typename}"')
      if cls.class_decl.template is not None:
        print('Skipping because class is templated')
        continue
      name_cpp = '::'.join([seg.name for seg in cls.class_decl.typename.segments])
      name_python = name_cpp.replace('vp', '')
      python_ident = f'py{name_python}'
      # if name_cpp == 'vpColVector':
      cls_result = f'\tpy::class_ {python_ident} = py::class_<{name_cpp}>(submodule, "{name_python}");'
      methods_str = ''
      skip_class = False
      for method in cls.methods:
        if method.pure_virtual:
          skip_class = True
          break
      if skip_class:
        continue

      for method in cls.methods:
        if method.access is not None and method.access != 'public':
          continue
        if method.constructor:
          params_strs = []
          skip_constructor = False
          for param in method.parameters:

            if isinstance(param.type, types.Type):
              print(param.type.typename)
              params_strs.append('::'.join([seg.name for seg in param.type.typename.segments]))
            elif isinstance(param.type, types.Reference):
              param_str = ''
              print(param)
              print(param.type.ref_to.typename.segments[0].specialization)
              if param.type.ref_to.const:
                param_str += 'const '
              param_str += '::'.join([seg.name for seg in param.type.ref_to.typename.segments])

              param_str += '&'
              params_strs.append(param_str)
            else:
              skip_constructor = True
              break
          if not skip_constructor:
            argument_types = ', '.join(params_strs)
            print(argument_types)
            ctor_str = f'''
              {python_ident}.def(py::init<{argument_types}>());
            '''
            methods_str += ctor_str
        else:
          pass
      result += cls_result
      result += methods_str



    return result






def generate_module(generate_path: Path) -> None:
  main_path = generate_path / 'main.cpp'
  include_path = Path('/usr/local/include/visp3')
  submodules: List[Submodule] = get_submodules(include_path, generate_path / 'generated')
  for submodule in submodules:
    submodule.generate()

  with open(main_path, 'w') as main_file:
    submodule_fn_declarations = []
    submodule_fn_calls = []
    for submodule in submodules:
      name = submodule.generation_function_name()
      submodule_fn_declarations.append(f'void {name}(py::module_&);')
      submodule_fn_calls.append(f'{name}(m);')

    submodule_fn_declarations = '\n'.join(submodule_fn_declarations)
    submodule_fn_calls = '\n'.join(submodule_fn_calls)

    format_str = f'''
#include <pybind11/pybind11.h>
namespace py = pybind11;
{submodule_fn_declarations}

PYBIND11_MODULE(visp, m)
{{
  m.doc() = "ViSP Python binding";

  {submodule_fn_calls}
}}
'''
    main_file.write(format_str)

if __name__ == '__main__':
  import  sys
  generate_module(Path('src'))