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


class DocumentationData(object):
  documentation_xml_location: Optional[Path] = Path('/home/sfelton/visp_build/doc/xml')

class DocumentationObjectKind(Enum):
  Class = 'class'
  Struct = 'struct'

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

  if container.name == 'ref':
    return ' ' + container.value.valueOf_ + ' '
  if container.name == 'verbatim':
    print(container.value.valueOf_)
    import sys; sys.exit()
  if container.name == 'simplesect':
    process_fn = lambda item: process_paragraph(item, level + 1)
    res = '\n'
    item_content = '\n'.join(map(process_fn, container.value.para))
    item_content = re.sub('\n\n\n+', '\n\n', item_content)
    if container.value.kind == 'note':
      res += '\n' + indent_str + '.. note:: ' + item_content + '\n' + indent_str
    elif container.value.kind == 'warning':
      res += '\n' + indent_str + '.. warning:: ' + item_content + '\n' + indent_str
    elif container.value.kind == 'see':
      res += '\n' + indent_str + '.. note:: See' + item_content + '\n' + indent_str



    print('res = ', res)
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
    print('res = ', res)
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
  def __init__(self, name: str, kind: DocumentationObjectKind, env_mapping: Dict[str, str]):
    self.object_name = name
    self.kind = kind
    self.env_mapping = env_mapping

    self.documentation_dict = {}
    if not import_failed and DocumentationData.documentation_xml_location is not None:
      assert self.kind == DocumentationObjectKind.Class
      xml_path = DocumentationData.documentation_xml_location / f'class{name}.xml'
      if not xml_path.exists():
        print(f'Could not find documentation file for name {name}, looking in {str(xml_path)}')
      else:
        self.documentation_dict = self.parse_xml(xml_path)

  def parse_xml(self, xml_path: Path) -> Dict:
    result_dict = {}
    xml_doc = doxmlparser.compound.parse(str(xml_path), True, False)
    for compounddef in xml_doc.get_compounddef():
      compounddef: compounddefType  = compounddef
      if compounddef.kind == DoxCompoundKind.CLASS:
        print(f'Found class {compounddef.get_compoundname()}')
        result_dict[compounddef.get_compoundname()] = self.generate_class_description_string(compounddef)
    print(xml_doc)
    return result_dict

  def generate_class_description_string(self, compounddef: compounddefType) -> str:
    brief = process_description(compounddef.get_briefdescription())
    detailed = process_description(compounddef.get_detaileddescription())
    print(brief, detailed)
    return to_cstring(brief + '\n' + detailed)


if __name__ == '__main__':
  name = 'vpBSpline'
  print(DocumentationHolder(name, DocumentationObjectKind.Class, {}).documentation_dict[name])