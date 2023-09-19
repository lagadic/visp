import_failed = False
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import re
try:
  import doxmlparser
  from doxmlparser.compound import DoxCompoundKind, DoxygenType, compounddefType, descriptionType, docParaType, MixedContainer
except ImportError:
  print('Cannot import xml parser')
  import_failed = True

class DocumentationObjectKind(Enum):
  '''
  Kind of the object for which we seek the documentation
  '''
  Class = 'class'
  Struct = 'struct'
  Method = 'method'

class DocumentationData(object):
  documentation_xml_location: Optional[Path] = Path('/home/sfelton/visp_build/doc/xml')

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

  def post_process_type(self, t: str) -> str:
    res = ''
    if '::' not in t:
      res = t
    else:
      # TODO: this kind of workaround works when you have no template
      is_const = 'const' in t
      is_rvalue = '&&' in t
      is_ref = not is_rvalue and '&' in t
      is_ptr = '*' in t
      last_name = t.split('::')[-1]
      res = 'const' if is_const else ''
      res += last_name
      res += '&&' if is_rvalue else ''
      res += '&' if is_ref else ''
      res += '*' if is_ptr else ''
    return res.replace(' ', '')


  def __init__(self, name, ret, params, is_const, is_static):
    self.name = name.replace(' ', '')
    self.ret = self.post_process_type(ret)
    clean_params = []
    for param in params:
      if param == 'void':
        continue
      clean_params.append(self.post_process_type(param))
    self.params = clean_params
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
    return '**' + container.value.valueOf_ + '**'
  if container.name == 'emphasis':
    return '*' + container.value.valueOf_ + '*'

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
    elif kind == 'warning':
      res += '\n' + indent_str + '.. warning:: ' + item_content + '\n' + indent_str
    elif kind == 'see':
      res += '\n' + indent_str + '.. note:: See' + item_content + '\n' + indent_str
    else:
      res += '\n' + f'<unparsed SimpleSect of kind {kind}>'
    return res + '\n'

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
        print(f'Could not find documentation file for name {name} when looking in {str(path)}')
      else:
        self.xml_doc = doxmlparser.compound.parse(str(path), True, False)
        compounddefs_res = {}
        methods_res = {}
        for compounddef in self.xml_doc.get_compounddef():
          compounddef: compounddefType = compounddef
          if compounddef.kind == DoxCompoundKind.CLASS:
            compounddefs_res[compounddef.get_compoundname()] = compounddef
            section_defs: List[doxmlparser.sectiondefType] = compounddef.sectiondef
            for section_def in section_defs:
              member_defs: List[doxmlparser.memberdefType] = section_def.memberdef
              method_defs = [d for d in member_defs if d.kind == doxmlparser.compound.DoxMemberKind.FUNCTION and d.prot == 'public']
              for method_def in method_defs:
                is_const = False if method_def.const == 'no' else True
                is_static = False if method_def.static == 'no' else True
                ret_type = ''.join(process_mixed_container(c, 0) for c in method_def.type_.content_)
                if ret_type in env_mapping:
                  ret_type = env_mapping[ret_type]
                param_types = []
                for param in method_def.get_param():
                  t = ''.join(process_mixed_container(c, 0) for c in param.type_.content_)
                  if t in env_mapping:
                    t = env_mapping[t]
                  param_types.append(t)
                signature = MethodDocSignature(method_def.name, ret_type, param_types, is_const, is_static)
                methods_res[(compounddef.get_compoundname(), signature)] = method_def
        self.elements = DocElements(compounddefs_res, methods_res)





  def get_documentation_for_class(self, name: str, cpp_ref_to_python: Dict[str, str], specs: Dict[str, str]) -> Optional[ClassDocumentation]:
    compounddef = self.elements.compounddefs.get(name)

    if compounddef is None:
      return None
    cls_str = self.generate_class_description_string(compounddef)
    return ClassDocumentation(cls_str)


  def get_documentation_for_method(self, cls_name: str, signature: MethodDocSignature, cpp_ref_to_python: Dict[str, str], specs: Dict[str, str]) -> Optional[MethodDocumentation]:
    method_def = self.elements.methods.get((cls_name, signature))
    if method_def is None:
      print(f'method {signature} not found')
      print([k[1] for k in self.elements.methods.keys() if k[1].name == signature.name])
      return None
    descr = self.generate_method_description_string(method_def)
    return MethodDocumentation(descr)



  def generate_class_description_string(self, compounddef: compounddefType) -> str:
    brief = process_description(compounddef.get_briefdescription())
    detailed = process_description(compounddef.get_detaileddescription())
    return to_cstring(brief + '\n\n' + detailed)

  def generate_method_description_string(self, method_def: doxmlparser.memberdefType) -> str:
    brief = process_description(method_def.get_briefdescription())
    detailed = process_description(method_def.get_detaileddescription())
    return to_cstring(brief + '\n\n' + detailed)


if __name__ == '__main__':
  name = 'vpBSpline'
  print(DocumentationHolder(name, DocumentationObjectKind.Class, {}).documentation_dict[name])