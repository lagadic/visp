from typing import Callable, List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser import types
from cxxheaderparser.simple import NamespaceScope, ClassScope
from utils import *
from dataclasses import dataclass
from submodule import Submodule
@dataclass
class EnumRepr:
  id: Optional[int]
  name: Optional[str]
  values: Optional[List[types.Enumerator]]
  public_access: bool = True


def is_typedef_to_enum(typedef: types.Typedef):
  '''
  Check whether a typedef refers to an enum
  '''
  if not isinstance(typedef.type, types.Type):
    return False
  if not typedef.type.typename.classkey == 'enum':
    return False
  return True


def is_anonymous_name(typename: types.PQName) -> Tuple[bool, int]:
  '''
  Check whether the name is anonymous. If it is, then the actual name is defined in a typedef.
  '''
  for segment in typename.segments:
    if isinstance(segment, types.AnonymousName):
      return True, segment.id
  return False, 0


def get_owner_py_ident(owner_name: str, root_scope: NamespaceScope) -> Optional[str]:
  '''
  Get the the owner's identifier (variable in generated code).
  If the owner is not an exported symbol (exported symbols are classes, enums, structs) then None is returned
  '''
  scope = get_cpp_identifier_scope(owner_name, root_scope)

  if isinstance(scope, NamespaceScope):
    return None

  return f'py{get_name(scope.class_decl.typename).replace("vp", "")}' #TODO: fix for custom names



def get_cpp_identifier_scope(fully_qualified_name: str, root_scope: Union[NamespaceScope, ClassScope]) -> Union[NamespaceScope, ClassScope]:
  if fully_qualified_name == '':
    return root_scope
  segments = fully_qualified_name.split('::')
  first_segment, remainder = segments[0], segments[1:]
  for cls in root_scope.classes:
    if get_name(cls.class_decl.typename) == first_segment:
      return get_cpp_identifier_scope('::'.join(remainder), cls)
  if isinstance(root_scope, NamespaceScope):
    for ns in root_scope.namespaces:
      if ns == first_segment:
        return get_cpp_identifier_scope('::'.join(remainder), root_scope.namespaces[ns])
  return root_scope


def enum_bindings(root_scope: NamespaceScope, mapping: Dict, submodule: Submodule) -> List[Tuple[str, str]]:
  final_data: List[EnumRepr] = []
  temp_data: List[EnumRepr] = [] # Data that is incomplete for preprocessing
  match_id = lambda repr, enum_id: repr.id is not None and repr.id == enum_id
  match_name = lambda repr, full_name: repr.name is not None and full_name is not None and repr.name == full_name
  enum_repr_is_ready = lambda repr: repr.name is not None and repr.values is not None and repr.public_access

  def accumulate_data(scope: Union[NamespaceScope, ClassScope]):
    if isinstance(scope, ClassScope):
      if scope.class_decl.access is not None and scope.class_decl.access != 'public':
        return
    for cls in scope.classes:
      accumulate_data(cls)
    if isinstance(scope, NamespaceScope):
      for namespace in scope.namespaces:
        accumulate_data(scope.namespaces[namespace])
    for enum in scope.enums:
      public_access = True
      if enum.access is None or enum.access != 'public':
        public_access = False
      anonymous_enum, enum_id = is_anonymous_name(enum.typename)

      full_name = get_typename(enum.typename, {}, mapping) if not anonymous_enum else None

      matches = lambda repr: match_id(repr, enum_id) or match_name(repr, full_name)
      matching = list(filter(matches, temp_data))
      assert len(matching) <= 1, f"There cannot be more than one repr found. Matches = {matching}"
      if len(matching) == 0:
        temp_data.append(EnumRepr(enum_id, full_name, enum.values, public_access))
      else:
        if full_name is not None:
          matching[0].name = full_name
        if enum.values is not None:
          matching[0].values = enum.values
        matching[0].public_access = matching[0].public_access and public_access # If we found a private def somewhere, mark enum as private

    for typedef in scope.typedefs:
      if not is_typedef_to_enum(typedef):
        continue
      public_access = True
      if typedef.access is None or typedef.access != 'public':
        public_access = False

      anonymous_enum, enum_id = is_anonymous_name(typedef.type.typename)
      full_name = mapping[typedef.name]

      matches = lambda repr: match_id(repr, enum_id) or match_name(repr, full_name)
      matching = list(filter(matches, temp_data))
      assert len(matching) <= 1, f"There cannot be more than one repr found. Matches = {matching}"
      if len(matching) == 0:
        temp_data.append(EnumRepr(enum_id, full_name, None, public_access))
      else:
        if full_name is not None:
          matching[0].name = full_name
        matching[0].public_access = matching[0].public_access and public_access

    ready_enums = list(filter(enum_repr_is_ready, temp_data))
    for repr in ready_enums:
      final_data.append(repr)
      temp_data.remove(repr)

  accumulate_data(root_scope)
  result = []

  for repr in temp_data:
    print(f'Enum {repr} was ignored, because it is either marked as private or it is incomplete (missing values or name)')

  for enum_repr in final_data:
    name_segments = enum_repr.name.split('::')
    py_name = name_segments[-1].replace('vp', '')
    # If an owner class is ignored, don't export this enum
    parent_ignored = False
    ignored_parent_name = None
    for segment in name_segments[:-1]:
      full_segment_name = mapping.get(segment)
      if full_segment_name is not None and submodule.class_should_be_ignored(full_segment_name):
        parent_ignored = True
        ignored_parent_name = full_segment_name
        break
    if parent_ignored:
      print(f'Ignoring enum {py_name} because {ignored_parent_name} is ignored')
      continue

    owner_full_name = '::'.join(name_segments[:-1])
    owner_py_ident = get_owner_py_ident(owner_full_name, root_scope) or 'submodule'
    py_ident = f'py{owner_py_ident}{py_name}'

    declaration = f'py::enum_<{enum_repr.name}> {py_ident}({owner_py_ident}, "{py_name}", py::arithmetic());'
    values = []
    for enumerator in enum_repr.values:
      values.append(f'{py_ident}.value("{enumerator.name}", {enum_repr.name}::{enumerator.name});')

    values.append(f'{py_ident}.export_values();')
    definition = '\n'.join(values)
    result.append((declaration, definition))


  return result