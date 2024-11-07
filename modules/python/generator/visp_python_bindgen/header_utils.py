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

from typing import List, Set, Dict, Union
import sys

import logging

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
      warning_msg = f'''
      Warning: Could not completely solve dependencies, generating but might have some errors
      Faulty headers: {[h.path.name for h in remainder]}'''
      logging.warning(warning_msg)
      print(warning_msg, file=sys.stderr)
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
    from visp_python_bindgen.enum_binding import resolve_enums_and_typedefs_to_enums
    enum_reprs, _ = resolve_enums_and_typedefs_to_enums(data.namespace, self.mapping)
    for enum_repr in enum_reprs:
      for value in enum_repr.values:
        self.mapping[value.name] = enum_repr.name + '::' + value.name

  def build_naive_mapping(self, data: Union[NamespaceScope, ClassScope], mapping, scope: str = ''):
    current_mapping = mapping.copy()
    previous_mapping = None

    while current_mapping != previous_mapping:
      previous_mapping = current_mapping.copy()

      for alias in data.using_alias:
        current_mapping[alias.alias] = get_type(alias.type, {}, current_mapping)

      for typedef in data.typedefs:
        if not typedef_is_anonymous(typedef.type):
          current_mapping[typedef.name] = get_type(typedef.type, {}, current_mapping)
        else:
          current_mapping[typedef.name] = scope + typedef.name
      for enum in data.enums:
        if not name_is_anonymous(enum.typename):
          enum_name = '::'.join([seg.name for seg in enum.typename.segments])
          current_mapping[enum_name] = scope + enum_name

      for cls in data.classes:
        cls_name = '::'.join([seg.name for seg in cls.class_decl.typename.segments if not isinstance(seg, types.AnonymousName)])
        current_mapping[cls_name] = scope + cls_name
        current_mapping.update(self.build_naive_mapping(cls, mapping=current_mapping, scope=f'{scope}{cls_name}::'))

      if isinstance(data, NamespaceScope):
        for namespace in data.namespaces:
          current_mapping.update(self.build_naive_mapping(data.namespaces[namespace], mapping=current_mapping, scope=f'{scope}{namespace}::'))

    return current_mapping

  def update_with_dependencies(self, other_envs: List['HeaderEnvironment']) -> None:
    for env in other_envs:
      self.mapping.update(env)
