import_failed = False
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional
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
    print(container.value.valueOf_)
    import sys; sys.exit()

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
  def __init__(self, path: Optional[Path] = None):
    self.xml_path = path

    if not import_failed and DocumentationData.documentation_xml_location is not None:
      if not self.xml_path.exists():
        print(f'Could not find documentation file for name {name} when looking in {str(path)}')
      else:
        self.xml_doc = doxmlparser.compound.parse(str(path), True, False)

  def get_documentation_for_class(self, name: str) -> Optional[str]:
    for compounddef in self.xml_doc.get_compounddef():
      compounddef: compounddefType  = compounddef
      if compounddef.kind == DoxCompoundKind.CLASS and compounddef.get_compoundname() == name:
        return self.generate_class_description_string(compounddef)
    return None

  def get_documentation_for_method(self):
    pass

  def generate_class_description_string(self, compounddef: compounddefType) -> str:
    brief = process_description(compounddef.get_briefdescription())
    detailed = process_description(compounddef.get_detaileddescription())
    print(brief, detailed)
    return to_cstring(brief + '\n' + detailed)


if __name__ == '__main__':
  name = 'vpBSpline'
  print(DocumentationHolder(name, DocumentationObjectKind.Class, {}).documentation_dict[name])