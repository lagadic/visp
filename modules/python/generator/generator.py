from typing import List, Tuple
from cxxheaderparser.parserstate import ClassBlockState
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser.parser import CxxParser
from pathlib import Path
import json
class Visitor(CxxVisitor):
  def __init__(self):
    self.result = ''
  def on_class_start(self, state: ClassBlockState) -> None:
    print(state.class_decl.typename)

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

    for include_file in self.include_path.iterdir():
      if not include_file.name.endswith('.h') and not include_file.name.endswith('.hpp'):
        continue
      if include_file.name in self.config['ignored_headers']:
        continue
      header = HeaderFile(include_file, self)
    body = f'py::module_ submodule = m.def_submodule("{self.name}");'
    format_str = f'''
#include <pybind11/pybind11.h>
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
    Submodule('vs', include_path / 'vs', generate_path / 'vs.cpp')

  ]

class HeaderFile():
  def __init__(self, path: Path, submodule: Submodule):
    self.path = path
    self.submodule = submodule
    content = self.run_preprocessor()
    self.generate_binding_code(content)
  def run_preprocessor(self):
    tmp_file_path = self.submodule.submodule_file_path.parent / "tmp" / self.path.name
    argv = [
      '',
      '-D', 'VISP_EXPORT=""',
      '-D', 'visp_deprecated=""',
      '-I', '/usr/local/include',
      '--passthru-includes', "^((?!vpConfig.h).)*$",
      '-o', f'{tmp_file_path}',
      str(self.path.absolute())
    ]
    pcpp.CmdPreprocessor(argv)
    preprocessed_header_content = None
    with open(tmp_file_path, 'r') as header_file:
      preprocessed_header_content = '\n'.join(header_file.readlines())
    return preprocessed_header_content
  def generate_binding_code(self, content: str) -> str:
    visitor = Visitor()
    parser = CxxParser(None, content, visitor)
    parser.parse()




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