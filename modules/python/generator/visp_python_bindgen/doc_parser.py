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

import_failed = False
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import re
from cxxheaderparser.simple import parse_string
try:
  import doxmlparser
  from doxmlparser.compound import DoxCompoundKind, DoxygenType, compounddefType, descriptionType, docParaType, MixedContainer
except ImportError:
  print('Cannot import xml parser')
  import_failed = True

from visp_python_bindgen.utils import *
from visp_python_bindgen.generator_config import GeneratorConfig

class DocumentationObjectKind(Enum):
  '''
  Kind of the object for which we seek the documentation
  '''
  Class = 'class'
  Struct = 'struct'
  Method = 'method'

class DocumentationData(object):

  @staticmethod
  def get_xml_path_if_exists(name: str, kind: DocumentationObjectKind) -> Optional[Path]:
    if import_failed:
      return None

    xml_root = GeneratorConfig.xml_doc_path
    if xml_root is None or not xml_root.exists():
      return None
    p = None
    if kind == DocumentationObjectKind.Class:
      p = xml_root / f'class{name}.xml'
    else:
      assert False, 'Seeking documentation for type other than class not handled for now'

    if not p.exists():
      return None
    return p


def to_cstring(s: str) -> str:
  s = s.replace('\t', '  ')
  s = re.sub('\n\n\n+', '\n\n', s)
  s = re.sub('\\\\ +', '\\\\', s)

  # On Windows, strings have a maximum length.
  per_string_limit = 8192
  current_char = 0
  result = ''
  while current_char < len(s):
    result += f'''R"doc({s[current_char: min((current_char + per_string_limit), len(s))]})doc"
  '''
    current_char += per_string_limit
  return result


@dataclass
class MethodDocSignature:
  name: str
  ret: str
  params: List[str]
  is_const: bool
  is_static: bool

  def __init__(self, name, ret, params, is_const, is_static):
    self.name = name.replace(' ', '')
    self.ret = ret.replace(' ', '')
    self.params = [p.replace(' ', '') for p in params]
    self.is_const = is_const
    self.is_static = is_static

  def __hash__(self) -> int:
    s = f'{self.is_static} {self.ret} {self.name}({",".join(self.params)}) {self.is_const}'
    return s.__hash__()

@dataclass
class MethodDocumentation(object):
  documentation: str

@dataclass
class ClassDocumentation(object):
  documentation: str
@dataclass
class EnumDocumentation(object):
  general_documentation: str
  value_documentation: Dict[str, str]

  def get_overall_doc(self) -> str:
    full_doc = ''
    full_doc += self.general_documentation.strip('\n')
    full_doc += '\n\nValues: \n\n'
    for k,v in self.value_documentation.items():
      full_doc += '* '
      full_doc += '**' + k + '**'
      if len(v.strip('\n').strip()) > 0:
        full_doc += ': ' + v.strip('\n')
      full_doc += '\n\n'

    return to_cstring(full_doc)

  def get_value_doc(self, k: str) -> Optional[str]:
    return to_cstring(self.value_documentation.get(k) or '')

@dataclass
class DocElements(object):
  compounddefs: Dict[str, compounddefType]
  enums: Dict[str, doxmlparser.memberdefType]
  methods: Dict[Tuple[str, MethodDocSignature], List[doxmlparser.memberdefType]]

IGNORED_MIXED_CONTAINERS = [

  'parameterlist'
]

IGNORED_SIMPLE_SECTS = [
  'author',
  'date',
]

def escape_for_rst(text: str) -> str:
  forbidden_chars = ['_', '*']
  res = text
  for c in forbidden_chars:
    res = res.replace(c, '\\' + c)
  return res



def process_mixed_container(container: MixedContainer, level: int, level_string='', escape_rst=False) -> str:
  '''
  :param level_string: the string being built for a single level (e.g. a line/paragraph of text)
  This is equivalent to a left fold operation,
  so don't forget to aggregate the results in level_string if you add another component
  '''
  if container.name in IGNORED_MIXED_CONTAINERS:
    return level_string
  one_indent = ' ' * 2
  indent_str = one_indent * level
  requires_space = not level_string.endswith(('\n', '\t', ' ')) and len(level_string) > 0

  def process_markup(symbol: str) -> str:
    text = symbol if not requires_space or len(level_string) == 0 else ' ' + symbol
    for c in container.value.content_:
      text = process_mixed_container(c, level, text, escape_rst=escape_rst)
    text += symbol + ' '
    return text

  # Inline blocks
  if isinstance(container.value, str) and container.name != 'verbatim':
    content = container.value.replace('\n', '\n' + indent_str).strip()
    if escape_rst:
      content = escape_for_rst(content)
    return level_string + content
  if container.name == 'text':
    content = container.value.replace('\n', '\n' + indent_str).strip()
    if escape_rst:
      content = escape_for_rst(content)
    return level_string + content
  if container.name == 'bold':
    return level_string + process_markup('**')
  if container.name == 'computeroutput':
    return level_string + process_markup('`')
  if container.name == 'emphasis':
    markup_start = '*' if not requires_space else ' *'
    return level_string + process_markup('*')
  if container.name == 'sp':
    return level_string + ' '
  if container.name == 'linebreak':
    return level_string + '\n'
  if container.name == 'heading':
    text = container.value.valueOf_
    new_line_with_sep = '\n' + indent_str + '=' * len(text)
    return level_string + new_line_with_sep + '\n' + indent_str + text + new_line_with_sep
  if container.name == 'ulink':
    url: doxmlparser.docURLLink = container.value
    text = url.valueOf_
    url_value = url.url
    return level_string + (' ' if requires_space else '') + f'`{text} <{url_value}>`_ '

  if container.name == 'formula':
    v: str = container.value.valueOf_.strip()
    if v.startswith(('\\[', '$$')):
      pure_math = indent_str + one_indent + v[2:-2].replace('\n', '')
      return level_string + ('\n' + indent_str) * 2 + '.. math::' + '\n' + pure_math + '\n' + '\n'
    else:
      pure_math = v[1:-1].replace('\n', indent_str + one_indent)
      return level_string + (' ' if requires_space else '') + ':math:`' + pure_math.strip() + '` '

  if container.name == 'ref': # TODO: replace with Python refs if possible
    return level_string + (' ' if requires_space else '') + container.value.valueOf_ + ' '

  if container.name == 'verbatim':
    text = container.value
    text = escape_for_rst(text)
    text = '\n' * 2 + indent_str + '::\n\n' + indent_str + one_indent + text.strip().replace('\n', '\n' + indent_str + one_indent).strip() + '\n' * 2
    return level_string + text

  # Block types
  if container.name == 'simplesect':
    process_fn = lambda item: process_paragraph(item, level + 1)
    res = '\n'
    kind = container.value.kind
    if kind in IGNORED_SIMPLE_SECTS:
      return level_string
    item_content = '\n'.join(map(process_fn, container.value.para))
    item_content = re.sub('\n\n\n+', '\n\n', item_content)
    if kind == 'note':
      res += '\n' + indent_str + '.. note:: ' + item_content + '\n' + indent_str
    elif kind == 'warning' or kind == 'attention':
      res += '\n' + indent_str + '.. warning:: ' + item_content + '\n' + indent_str
    elif kind == 'see':
      res += '\n' + indent_str + '.. note:: See ' + item_content + '\n' + indent_str
    elif kind == 'return': # Don't do anything, we will parse them separately when we get method documentation
      return ''
    else:
      res += '\n' + f'<unparsed SimpleSect of kind {kind}>'
    return level_string + res + '\n'

  if container.name == 'blockquote':
    blockquote: doxmlparser.docBlockQuoteType = container.value
    process_fn = lambda item: process_paragraph(item, level + 1)
    res = '\n'
    item_content = '\n'.join(map(process_fn, blockquote.para))
    return level_string + item_content + '\n'

  if container.name == 'programlisting':
    program: doxmlparser.listingType = container.value
    codelines: List[doxmlparser.codelineType] = program.codeline
    res = '\n\n' + indent_str + '.. code-block:: cpp' + '\n\n' + indent_str + one_indent
    lines = []
    for line in codelines:
      cs = []
      for h in line.highlight:
        c = ''
        for hh in h.content_:
          c = process_mixed_container(hh, level, c, escape_rst=False)
        cs.append(c)
      s = ''.join(cs)
      lines.append(s)

    code = ('\n' + indent_str + one_indent).join(lines)
    res += code + '\n\n'
    return level_string + res

  if container.name == 'itemizedlist':
    items: List[doxmlparser.docListItemType] = container.value.listitem
    res = '\n'
    process_fn = lambda item: process_paragraph(item, level + 1)
    for item in items:
      item_content = '\n'.join(map(process_fn, item.para))
      item_content = re.sub('\n\n\n+', '\n\n', item_content)
      #item_content = item_content.replace('\n' + indent_str, '\n' + indent_str + '  ')
      res += '\n' + indent_str + '* ' + item_content + '\n' + indent_str
    return level_string  + res + '\n'

  return f'<unparsed {container.name} {container.value}>'


def process_paragraph(para: docParaType, level: int) -> str:
  res = ''
  contents: List[MixedContainer] = para.content_
  for content_item in contents:
    res = process_mixed_container(content_item, level, res, escape_rst=True)
  return res

def process_description(brief: Optional[descriptionType]) -> str:
  if brief is None:
    return ''
  para: List[docParaType] = brief.para
  return '\n\n'.join([process_paragraph(par, 0) for par in para])

class DocumentationHolder(object):
  def __init__(self, path: Optional[Path], env_mapping: Dict[str, str]):
    self.xml_path = path
    self.elements = None
    if not import_failed and GeneratorConfig.xml_doc_path is not None:
      if not self.xml_path.exists():
        print(f'Could not find documentation when looking in {str(path)}')
      else:
        self.xml_doc = doxmlparser.compound.parse(str(path), True, False)
        compounddefs_res = {}
        enums_res = {}
        methods_res = {}
        for compounddef in self.xml_doc.get_compounddef():
          compounddef: compounddefType = compounddef
          if compounddef.kind == DoxCompoundKind.CLASS:
            cls_name = compounddef.get_compoundname()
            compounddefs_res[cls_name] = compounddef
            section_defs: List[doxmlparser.sectiondefType] = compounddef.sectiondef
            for section_def in section_defs:
              member_defs: List[doxmlparser.memberdefType] = section_def.memberdef
              enum_defs = [d for d in member_defs if d.kind == doxmlparser.compound.DoxMemberKind.ENUM and d.prot == 'public']
              method_defs = [d for d in member_defs if d.kind == doxmlparser.compound.DoxMemberKind.FUNCTION and d.prot == 'public']
              for method_def in method_defs:
                is_const = False if method_def.const == 'no' else True
                is_static = False if method_def.static == 'no' else True
                ret_type = ''.join(process_mixed_container(c, 0, escape_rst=False) for c in method_def.type_.content_).replace('VP_DEPRECATED', '').replace('VISP_EXPORT', '')
                param_types = []
                for param in method_def.get_param():
                  t = ''.join(process_mixed_container(c, 0, escape_rst=False) for c in param.type_.content_)
                  param_types.append(t)
                if method_def.name == cls_name or ret_type != '':
                  signature_str = f'{ret_type} {cls_name}::{method_def.name}({",".join(param_types)}) {{}}'
                  method = parse_string(signature_str).namespace.method_impls[0]
                  method.static = is_static
                  method.const = is_const

                  signature = MethodDocSignature(method_def.name,
                                                get_type(method.return_type, {}, env_mapping) or '', # Don't use specializations so that we can match with doc
                                                [get_type(param.type, {}, env_mapping) or '' for param in method.parameters],
                                                method.const, method.static)
                  key = (compounddef.get_compoundname(), signature)
                  if key in methods_res:
                    num_paras = len(method_def.detaileddescription.para) + len(method_def.briefdescription.para)
                    if num_paras > 0: # Doxygen adds some empty memberdefs in addition to the ones where the doc is defined...
                      methods_res[key] = method_def
                  else:
                    methods_res[key] = method_def

              for enum_def in enum_defs:
                enums_res[compounddef.get_compoundname() + '::' +  enum_def.name] = enum_def

        self.elements = DocElements(compounddefs_res, enums_res, methods_res)

  def get_documentation_for_class(self, name: str, cpp_ref_to_python: Dict[str, str], specs: Dict[str, str]) -> Optional[ClassDocumentation]:
    compounddef = self.elements.compounddefs.get(name)
    if compounddef is None:
      return None
    cls_str = to_cstring(self.generate_class_description_string(compounddef))
    return ClassDocumentation(cls_str)

  def get_documentation_for_enum(self, enum_name: str) -> Optional[EnumDocumentation]:
    member_def = self.elements.enums.get(enum_name)
    if member_def is None:
      return None
    general_doc = self.generate_method_description_string(member_def)
    value_doc = {}
    for enum_val in member_def.enumvalue:
      enum_value: doxmlparser.enumvalueType = enum_val
      brief = process_description(enum_value.briefdescription)
      detailed = process_description(enum_value.detaileddescription)
      value_doc[enum_value.name] = brief + '\n\n' + detailed
    return EnumDocumentation(general_doc, value_doc)

  def generate_class_description_string(self, compounddef: compounddefType) -> str:
    brief = process_description(compounddef.get_briefdescription())
    detailed = process_description(compounddef.get_detaileddescription())
    return brief + '\n\n' + detailed

  def get_documentation_for_method(self, cls_name: str, signature: MethodDocSignature, cpp_ref_to_python: Dict[str, str],
                                   specs: Dict[str, str], input_param_names: List[str], output_param_names: List[str]) -> Optional[MethodDocumentation]:
    method_def = self.elements.methods.get((cls_name, signature))
    if method_def is None:
      return None

    descr = self.generate_method_description_string(method_def)

    params_dict = self.get_method_params(method_def)
    cpp_return_str = self.get_method_return_str(method_def)

    param_strs = []
    for param_name in input_param_names:
      if param_name in params_dict:
        param_strs.append(f':param {escape_for_rst(param_name)}: {params_dict[param_name]}')
    param_str = '\n'.join(param_strs)

    if len(output_param_names) > 0:
      return_str = ':return: A tuple containing:\n' # TODO: if we only return a single element, we should modify this
      if signature.ret != 'void' and signature.ret is not None:
        return_str += f'\n\t * {cpp_return_str}'
      for param_name in output_param_names:
        if param_name in params_dict:
          return_str += f'\n\t * {escape_for_rst(param_name)}: {params_dict[param_name]}'
        else:
          return_str += f'\n\t * {escape_for_rst(param_name)}'
    else:
      return_str = f':return: {cpp_return_str}' if len(cpp_return_str) > 0 else ''

    res = to_cstring('\n\n'.join([descr, param_str, return_str]))
    return MethodDocumentation(res)

  def generate_method_description_string(self, method_def: doxmlparser.memberdefType) -> str:
    brief = process_description(method_def.get_briefdescription())
    detailed = process_description(method_def.get_detaileddescription())
    return brief + '\n\n' + detailed

  def get_method_params(self, method_def: doxmlparser.memberdefType) -> Dict[str, str]:
    parameter_list_full: List[doxmlparser.docParamListItem] = []
    paras: List[doxmlparser.docParaType] = method_def.detaileddescription.para + method_def.inbodydescription.para + method_def.briefdescription.para
    for paragraph in paras:
      if paragraph.parameterlist is not None:
        parameter_lists: List[doxmlparser.docParamListType] = paragraph.parameterlist
        assert isinstance(parameter_lists, list)
        for param_list in parameter_lists:
          parameter_list_full.extend(param_list.parameteritem)

    params_dict = {}
    for param_info in parameter_list_full:
      from functools import reduce
      name_list: List[doxmlparser.compound.docParamName] = reduce(lambda all, pnl: all + pnl.parametername,  param_info.parameternamelist, [])
      param_descr: doxmlparser.compound.descriptionType = param_info.parameterdescription
      param_descr_str = ' '.join(map(lambda para: process_paragraph(para, 0), param_descr.para)).lstrip(': ').strip().replace('\n', '\n\t')
      for param_name in name_list:
        params_dict[param_name.valueOf_] = param_descr_str
    return params_dict

  def get_method_return_str(self, method_def: doxmlparser.memberdefType) -> Dict[str, str]:
    paras: List[doxmlparser.docParaType] = method_def.detaileddescription.para + method_def.inbodydescription.para + method_def.briefdescription.para
    return_str = ''
    for paragraph in paras:
      sections: List[doxmlparser.docSimpleSectType] = paragraph.simplesect
      for sect in sections:
        if sect.kind == 'return':
          return_str += ' '.join(map(lambda para: process_paragraph(para, 0), sect.para))
    return return_str
