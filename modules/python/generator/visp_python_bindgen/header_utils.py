from typing import List, Set, Dict, Union
import sys

from cxxheaderparser import types
from cxxheaderparser.simple import ParsedData, NamespaceScope, ClassScope

from visp_python_bindgen.utils import *
from visp_python_bindgen.methods import *
from visp_python_bindgen.doc_parser import *

from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from visp_python_bindgen.header import HeaderFile

def sort_headers(headers: List['HeaderFile']) -> List['HeaderFile']:
  '''
  Sort headers based on their dependencies on other classes.
  If a class does not inherit from any other, then it will be placed at the start of the list.
  If it has a dependency, then it will be placed after this dependency in the list.
  This step is important to ensure that the code generation is performed in the correct order.
  It is not possible to declare an inheriting class to pybind without first exposing the base class.
  '''
  def add_level(result: List['HeaderFile'], remainder: List['HeaderFile'], dependencies: Set[str]):
    if len(remainder) == 0:
      return

    include_in_result_fn = None
    if len(result) == 0: # First iteration, query all headers that have no dependencies
      include_in_result_fn = lambda h: len(h.depends) == 0
    else:
      # Some header define multiple classes, where one may rely on another. So we filter h.depends
      include_in_result_fn = lambda h: all(map(lambda x: x in dependencies, filter(lambda s: s not in h.contains, h.depends)))
    new_remainder = []
    new_dependencies = dependencies.copy()
    for header_file in remainder:
      has_dependency = include_in_result_fn(header_file)
      if has_dependency:
        new_dependencies = new_dependencies | set(header_file.contains)
        result.append(header_file)
      else:
        new_remainder.append(header_file)
    if new_remainder == remainder:
      print(f'Warning: Could not completely solve dependencies, generating but might have some errors\n Faulty headers: {[h.path.name for h in remainder]}', file=sys.stderr)
      result.extend(remainder)
    else:
      add_level(result, new_remainder, set(new_dependencies))
  result = []
  add_level(result, headers, set())
  return result

class HeaderEnvironment():
  def __init__(self, data: ParsedData):
    self.mapping: Dict[str, str] = self.build_naive_mapping(data.namespace, {})

    # Step 2: resolve enumeration names that are possibly hidden behind typedefs
    from visp_python_bindgen.enum_binding import resolve_enums_and_typedefs
    enum_reprs, _ = resolve_enums_and_typedefs(data.namespace, self.mapping)
    for enum_repr in enum_reprs:
      for value in enum_repr.values:
        self.mapping[value.name] = enum_repr.name + '::' + value.name

  def build_naive_mapping(self, data: Union[NamespaceScope, ClassScope], mapping, scope: str = ''):
    if isinstance(data, NamespaceScope):
      for alias in data.using_alias:
        mapping[alias.alias] = get_type(alias.type, {}, mapping)

      for typedef in data.typedefs:
        mapping[typedef.name] = scope + typedef.name

      for enum in data.enums:
        if not name_is_anonymous(enum.typename):
          enum_name = '::'.join([seg.name for seg in enum.typename.segments])
          mapping[enum_name] = scope + enum_name

      for cls in data.classes:
        cls_name = '::'.join([seg.name for seg in cls.class_decl.typename.segments])
        mapping[cls_name] = scope + cls_name
        mapping.update(self.build_naive_mapping(cls, mapping=mapping, scope=f'{scope}{cls_name}::'))

      for namespace in data.namespaces:
        mapping.update(self.build_naive_mapping(data.namespaces[namespace], mapping=mapping, scope=f'{scope}{namespace}::'))

    elif isinstance(data, ClassScope):
      for alias in data.using_alias:
        mapping[alias.alias] = get_type(alias.type, {}, mapping)

      for typedef in data.typedefs:
        mapping[typedef.name] = scope + typedef.name

      for enum in data.enums:
        if not name_is_anonymous(enum.typename):
          enum_name = '::'.join([seg.name for seg in enum.typename.segments])
          mapping[enum_name] = scope + enum_name

      for cls in data.classes:
        cls_name = '::'.join([seg.name for seg in cls.class_decl.typename.segments if not isinstance(seg, types.AnonymousName)])
        mapping[cls_name] = scope + cls_name
        mapping.update(self.build_naive_mapping(cls, mapping=mapping, scope=f'{scope}{cls_name}::'))
    return mapping


  def update_with_dependencies(self, other_envs: List['HeaderEnvironment']) -> None:
    for env in other_envs:
      self.mapping.update(env)
