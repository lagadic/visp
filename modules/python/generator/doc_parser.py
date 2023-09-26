import_failed = False
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import re
from cxxheaderparser.simple import parse_string
try:
  import doxmlparser
  from doxmlparser.compound import DoxCompoundKind, DoxygenType, compounddefType, descriptionType, docParaType, MixedContainer
except ImportError:
  print('Cannot import xml parser')
  import_failed = True

from utils import *

class DocumentationObjectKind(Enum):
  '''
  Kind of the object for which we seek the documentation
  '''
  Class = 'class'
  Struct = 'struct'
  Method = 'method'

class DocumentationData(object):
  documentation_xml_location: Optional[Path] = Path('/home/sfelton/software/visp_build/doc/xml')

  @staticmethod
  def get_xml_path_if_exists(name: str, kind: DocumentationObjectKind) -> Optional[Path]:
    if import_failed:
      return None

    xml_root = DocumentationData.documentation_xml_location
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
  return f'''R"doc(
{s}
)doc"'''
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
class DocElements(object):
  compounddefs: Dict[str, compounddefType]
  methods: Dict[Tuple[str, MethodDocSignature], List[doxmlparser.memberdefType]]


def process_mixed_container(container: MixedContainer, level: int) -> str:
  one_indent = ' ' * 2
  indent_str = one_indent * level
  if isinstance(container.value, str):
    return container.value.replace('\n', '\n' + indent_str).strip()
  # text
  if container.name == 'text':
    return container.value.replace('\n', '\n' + indent_str).strip()
  if container.name == 'bold':
    return ' **' + container.value.valueOf_ + '** '
  if container.name == 'emphasis':
    return ' *' + container.value.valueOf_ + '* '
  if container.name == 'sp':
    return ' '

  if container.name == 'formula':
    v: str = container.value.valueOf_.strip()
    if v.startswith(('\\[', '$$')):
      pure_math = indent_str + one_indent + v[2:-2].replace('\n', '')
      return ('\n' + indent_str) * 2 + '.. math::' + '\n' + pure_math + '\n' + '\n'
    else:
      pure_math = v[1:-1].replace('\n', indent_str + one_indent)
      return ' :math:`' + pure_math.strip() + '` '

  if container.name == 'ref': # TODO: replace with Python refs if possible
    return ' ' + container.value.valueOf_ + ' '

  if container.name == 'verbatim':
    raise NotImplementedError()

  if container.name == 'simplesect':
    process_fn = lambda item: process_paragraph(item, level + 1)
    res = '\n'
    item_content = '\n'.join(map(process_fn, container.value.para))
    item_content = re.sub('\n\n\n+', '\n\n', item_content)
    kind = container.value.kind
    if kind == 'note':
      res += '\n' + indent_str + '.. note:: ' + item_content + '\n' + indent_str
    elif kind == 'warning' or kind == 'attention':
      res += '\n' + indent_str + '.. warning:: ' + item_content + '\n' + indent_str
    elif kind == 'see':
      res += '\n' + indent_str + '.. note:: See' + item_content + '\n' + indent_str
    elif kind == 'return': # Don't do anything, we will parse them separately when we get method documentation
      return ''
    else:
      res += '\n' + f'<unparsed SimpleSect of kind {kind}>'
    return res + '\n'

  if container.name == 'programlisting':
    program: doxmlparser.listingType = container.value
    codelines: List[doxmlparser.codelineType] = program.codeline
    res = '\n\n' + indent_str + '.. code-block:: cpp' + '\n\n' + indent_str + one_indent
    lines = []
    for line in codelines:
      cs = []
      for h in line.highlight:
        c = ''.join([process_mixed_container(h, level) for h in h.content_])
        cs.append(c)
      s = ''.join(cs)
      lines.append(s)

    code = ('\n' + indent_str + one_indent).join(lines)
    res += code + '\n\n'
    return res

  if container.name == 'parameterlist': # Parameter list is ignored since we have custom parsing for it
    return ''

  if container.name == 'itemizedlist':
    items: List[doxmlparser.docListItemType] = container.value.listitem
    res = '\n'
    process_fn = lambda item: process_paragraph(item, level + 1)
    for item in items:
      item_content = '\n'.join(map(process_fn, item.para))
      item_content = re.sub('\n\n\n+', '\n\n', item_content)
      #item_content = item_content.replace('\n' + indent_str, '\n' + indent_str + '  ')
      res += '\n' + indent_str + '* ' + item_content + '\n' + indent_str
    return res + '\n'

  return f'<unparsed {container.value}>'


def process_paragraph(para: docParaType, level: int) -> str:
  res = ''
  contents: List[MixedContainer] = para.content_
  for content_item in contents:
    res += process_mixed_container(content_item, level)
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
    if not import_failed and DocumentationData.documentation_xml_location is not None:
      if not self.xml_path.exists():
        print(f'Could not find documentation when looking in {str(path)}')
      else:
        self.xml_doc = doxmlparser.compound.parse(str(path), True, False)
        compounddefs_res = {}
        methods_res = {}
        for compounddef in self.xml_doc.get_compounddef():
          compounddef: compounddefType = compounddef
          if compounddef.kind == DoxCompoundKind.CLASS:
            cls_name = compounddef.get_compoundname()
            compounddefs_res[cls_name] = compounddef
            section_defs: List[doxmlparser.sectiondefType] = compounddef.sectiondef
            for section_def in section_defs:
              member_defs: List[doxmlparser.memberdefType] = section_def.memberdef
              method_defs = [d for d in member_defs if d.kind == doxmlparser.compound.DoxMemberKind.FUNCTION and d.prot == 'public']
              for method_def in method_defs:
                is_const = False if method_def.const == 'no' else True
                is_static = False if method_def.static == 'no' else True
                ret_type = ''.join(process_mixed_container(c, 0) for c in method_def.type_.content_).replace('vp_deprecated', '').replace('VISP_EXPORT', '')
                param_types = []
                for param in method_def.get_param():
                  t = ''.join(process_mixed_container(c, 0) for c in param.type_.content_)

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
        self.elements = DocElements(compounddefs_res, methods_res)

  def get_documentation_for_class(self, name: str, cpp_ref_to_python: Dict[str, str], specs: Dict[str, str]) -> Optional[ClassDocumentation]:
    compounddef = self.elements.compounddefs.get(name)
    if compounddef is None:
      return None
    cls_str = to_cstring(self.generate_class_description_string(compounddef))
    return ClassDocumentation(cls_str)

  def generate_class_description_string(self, compounddef: compounddefType) -> str:
    brief = process_description(compounddef.get_briefdescription())
    detailed = process_description(compounddef.get_detaileddescription())
    return brief + '\n\n' + detailed

  def get_documentation_for_method(self, cls_name: str, signature: MethodDocSignature, cpp_ref_to_python: Dict[str, str], specs: Dict[str, str]) -> Optional[MethodDocumentation]:
    method_def = self.elements.methods.get((cls_name, signature))
    if method_def is None:
      return None

    descr = self.generate_method_description_string(method_def)

    params_dict = self.get_method_params(method_def)
    param_strs = [f':param {name}: {descr}' for name, descr in params_dict.items()]
    param_str = '\n'.join(param_strs)

    cpp_return_str = self.get_method_return_str(method_def)
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
      param_descr_str = ' '.join(map(lambda para: process_paragraph(para, 0), param_descr.para)).lstrip(': ')
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
