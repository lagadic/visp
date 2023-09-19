from typing import List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData, NamespaceScope, ClassScope
from pathlib import Path
import json
from tqdm.contrib.concurrent import process_map
from header import *
from submodule import *

from multiprocessing import Pool
def header_preprocess(header: HeaderFile):
  header.preprocess()
  return header


def generate_module(generate_path: Path) -> None:
  main_path = generate_path / 'main.cpp'
  include_path = Path('/usr/local/include/visp3')
  submodules: List[Submodule] = get_submodules(include_path, generate_path / 'generated')

  all_headers: List[HeaderFile] = []
  for submodule in submodules:
    all_headers.extend(submodule.headers)
  from tqdm import tqdm
  with Pool() as pool:
    new_all_headers = []
    for result in list(tqdm(pool.imap(header_preprocess, all_headers), total=len(all_headers), file=sys.stdout)):
      new_all_headers.append(result)

  new_all_headers = sort_headers(new_all_headers)
  for submodule in submodules:
    submodule.set_headers_from_common_list(new_all_headers)
  # for header in all_headers:
  #   header.preprocess()


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