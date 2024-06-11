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

from typing import List, Optional, Tuple, Dict, Union
from dataclasses import dataclass
import logging

from cxxheaderparser import types
from cxxheaderparser.simple import NamespaceScope, ClassScope

from visp_python_bindgen.utils import *
from visp_python_bindgen.submodule import Submodule


from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from visp_python_bindgen.header import SingleObjectBindings, HeaderFile

@dataclass
class EnumRepr:
  '''
  Intermediate representation of an enumeration
  '''
  id: Optional[int] # Integer Id for an enumeration. Used when an enumeration is anonymous (hidden behind a typedef). Id is unique per header file
  name: Optional[str] # Name of the enumeration
  values: Optional[List[types.Enumerator]] # The values of the enumeration
  public_access: bool = True # Whether this enum is visible from outside the header file

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
  return False, None

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

def resolve_enums_and_typedefs_to_enums(root_scope: NamespaceScope, mapping: Dict) -> Tuple[List[EnumRepr], List[EnumRepr]]:
  final_data: List[EnumRepr] = []
  temp_data: List[EnumRepr] = [] # Data that is incomplete for preprocessing
  match_id = lambda repr, enum_id: repr.id is not None and enum_id is not None and repr.id == enum_id
  match_name = lambda repr, full_name: repr.name is not None and full_name is not None and repr.name == full_name
  enum_repr_is_ready = lambda repr: repr.name is not None and repr.values is not None

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
      anonymous_enum, enum_id = is_anonymous_name(enum.typename)

      full_name = get_typename(enum.typename, {}, mapping) if not anonymous_enum else None
      # If in a namespace or parent class, this symbol can be referenced from outside and the visibility will be incorrect
      if enum.access is not None and enum.access != 'public':
        public_access = False

      if full_name is not None and '::' in full_name:
        base_ref = fetch_fully_qualified_id(root_scope, full_name.split('::'))
        if base_ref is not None and isinstance(base_ref, types.EnumDecl) and base_ref.access is not None and base_ref.access != 'public':
          public_access = False

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
      if typedef.access is not None and typedef.access != 'public':
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
  return final_data, temp_data

def get_enum_bindings(root_scope: NamespaceScope, mapping: Dict, submodule: Submodule, header: 'HeaderFile') -> List[SingleObjectBindings]:

  final_data, filtered_reprs = resolve_enums_and_typedefs_to_enums(root_scope, mapping)

  for repr in filtered_reprs:
    logging.info(f'Enum {repr} was ignored, because it is incomplete (missing values or name)')

  result: List['SingleObjectBindings'] = []
  final_reprs = []
  for repr in final_data:

    enum_config = submodule.get_enum_config(repr.name)
    if enum_config['ignore']:
      filtered_reprs.append(repr)
      logging.info(f'Enum {repr.name} is ignored by user')
    elif repr.public_access:
      final_reprs.append(repr)
    else:
      filtered_reprs.append(repr)
  doc_holder = header.documentation_holder
  for enum_repr in final_reprs:
    name_segments = enum_repr.name.split('::')
    py_name = name_segments[-1].replace('vp', '')
    # If an owner class is ignored, don't export this enum
    parent_ignored = False
    ignored_parent_name = None
    enum_doc = None
    if doc_holder is not None:
      enum_doc = header.documentation_holder.get_documentation_for_enum(repr.name)

    for segment in name_segments[:-1]:
      full_segment_name = mapping.get(segment)
      if full_segment_name is not None and submodule.class_should_be_ignored(full_segment_name):
        parent_ignored = True
        ignored_parent_name = full_segment_name
        break

    if parent_ignored:
      logging.info(f'Ignoring enum {py_name} because {ignored_parent_name} is ignored')
      continue

    owner_full_name = '::'.join(name_segments[:-1])
    owner_py_ident = get_owner_py_ident(owner_full_name, root_scope) or 'submodule'
    py_ident = f'py{owner_py_ident}{py_name}'
    py_args = ['py::arithmetic()']
    if enum_doc is not None:
      py_args = [enum_doc.get_overall_doc()] + py_args

    py_args_str = ','.join(py_args)
    declaration = f'py::enum_<{enum_repr.name}> {py_ident}({owner_py_ident}, "{py_name}", {py_args_str});'
    values = []
    for enumerator in enum_repr.values:
      maybe_value_doc = None
      # if enum_doc is not None:
      #   maybe_value_doc = enum_doc.get_value_doc(enumerator.name)
      maybe_value_doc_str = f', {maybe_value_doc}' if maybe_value_doc else ''

      values.append(f'{py_ident}.value("{enumerator.name}", {enum_repr.name}::{enumerator.name}{maybe_value_doc_str});')

    values.append(f'{py_ident}.export_values();')
    enum_names = BoundObjectNames(py_ident, py_name, enum_repr.name, enum_repr.name)
    enum_binding = SingleObjectBindings(enum_names, declaration, values, GenerationObjectType.Enum)
    result.append(enum_binding)
  return result
