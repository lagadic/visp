from typing import List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData, NamespaceScope, ClassScope, parse_file
from pathlib import Path
import json
from utils import *

def filter_includes(include_names: Set[str]) -> List[str]:
  result = []
  for include_name in include_names:
    if include_name.startswith('"') and include_name.endswith('"'):
      continue
    forbidden_strs = ['winsock', '<io.h>']
    forbidden_found = False
    for forbidden_str in forbidden_strs:
      if forbidden_str in include_name:
        forbidden_found = True
        break
    if forbidden_found:
      continue

    result.append(include_name)
  return result

def sort_headers(headers: List['HeaderFile']) -> List['HeaderFile']:
  def add_level(result: List['HeaderFile'], remainder: List['HeaderFile'], dependencies: Set[str]):
    if len(remainder) == 0:
      return

    include_in_result_fn = None
    if len(result) == 0:
      include_in_result_fn = lambda h: len(h.depends) == 0
    else:
      include_in_result_fn = lambda h: any(map(lambda x: x in dependencies, h.depends))
    new_remainder = []
    new_dependencies = []
    for header_file in remainder:
      has_dependency = include_in_result_fn(header_file)
      if has_dependency:
        new_dependencies.extend(header_file.contains)
        result.append(header_file)
      else:
        new_remainder.append(header_file)
    add_level(result, new_remainder, set(new_dependencies))
  result = []
  add_level(result, headers, set())
  return result



class HeaderEnvironment():
  def __init__(self, data: ParsedData):
    self.mapping = self.build_mapping(data.namespace)
  def build_mapping(self, data: Union[NamespaceScope, ClassScope], mapping={}, scope: str = ''):
    if isinstance(data, NamespaceScope):
      for alias in data.using_alias:
        mapping[alias.alias] = get_type(alias.type, {}, mapping)
      for enum in data.enums:
        print(enum)
        enum_name = '::'.join([seg.name for seg in enum.base.segments])
        mapping[enum_name] = scope + enum_name
      for cls in data.classes:
        cls_name = '::'.join([seg.name for seg in cls.class_decl.typename.segments])
        mapping[cls_name] = scope + cls_name
        mapping.update(self.build_mapping(cls, mapping=mapping, scope=f'{scope}{cls_name}::'))
      for namespace in data.namespaces:
        mapping.update(self.build_mapping(data.namespaces[namespace], mapping=mapping, scope=f'{scope}{namespace}::'))

    elif isinstance(data, ClassScope):
      for alias in data.using_alias:
        mapping[alias.alias] = get_type(alias.type, {}, mapping)
      for typedef in data.typedefs:
        mapping[typedef.name] = scope + typedef.name
      for enum in data.enums:
        enum_name = '::'.join([seg.name for seg in enum.typename.segments if not isinstance(seg, types.AnonymousName)])
        mapping[enum_name] = scope + enum_name
      for cls in data.classes:
        cls_name = '::'.join([seg.name for seg in cls.class_decl.typename.segments])
        mapping[cls_name] = scope + cls_name
        mapping.update(self.build_mapping(cls, mapping=mapping, scope=f'{scope}{cls_name}::'))

    return mapping

class HeaderFile():
  def __init__(self, path: Path, submodule: 'Submodule'):
    self.path = path
    self.submodule = submodule
    content = self.run_preprocessor()
    self.includes = [f'<visp3/{self.submodule.name}/{self.path.name}>']
    self.binding_code = None
    self.contains = []
    self.depends = []
    self.generate_binding_code(content)

  def run_preprocessor(self): # TODO: run without generating a new file
    tmp_file_path = self.submodule.submodule_file_path.parent / "tmp" / self.path.name
    print('Preprocessing', self.path)
    argv = [
      '',
      '-D', 'vp_deprecated=',
      '-D', 'VISP_EXPORT=',
      '-I', '/home/sfelton/visp_build/include',
      '-I', '/usr/include',
      '-N', 'VISP_BUILD_DEPRECATED_FUNCTIONS',
      '--passthru-includes', "^((?!vpConfig.h|!json.hpp).)*$",
      '--passthru-unfound-includes',
      '--passthru-comments',
      '--line-directive', '',
      '-o', f'{tmp_file_path}',
      str(self.path.absolute())
    ]
    pcpp.CmdPreprocessor(argv)
    preprocessed_header_content = None
    with open(tmp_file_path, 'r') as header_file:
      preprocessed_header_content = '\n'.join(header_file.readlines())
      preprocessed_header_content = preprocessed_header_content.replace('#include<', '#include <')
    return preprocessed_header_content

  def generate_binding_code(self, content: str) -> None:
    print('Before parsing cpp structure for file: ', self.path)
    parsed_data = parse_string(content)
    # tmp_file_path = self.submodule.submodule_file_path.parent / "tmp" / self.path.name
    # parsed_data = parse_file(str(tmp_file_path))
    self.binding_code = self.parse_data(parsed_data)


  def parse_data(self, data: ParsedData):
    result = ''
    header_env = HeaderEnvironment(data)
    for cls in data.namespace.classes:
      result += self.generate_class(cls, header_env)
    return result

  def generate_class(self, cls: ClassScope, header_env: HeaderEnvironment) -> str:
    result = ''

    def generate_class_with_potiental_specialization(name_python: str, owner_specs: Dict[str, str]) -> str:
      spec_result = ''
      python_ident = f'py{name_python}'

      name_cpp = get_typename(cls.class_decl.typename, owner_specs, header_env.mapping)
      if cls.class_decl.template is not None:
        template_strs = []
        for template in cls.class_decl.template.params:
          template_strs.append(owner_specs[template.name])
        template_str = f'<{", ".join(template_strs)}>'
        name_cpp += template_str

      base_classes_str = ''
      for base_class in cls.class_decl.bases:
        if base_class.access == 'public':
          base_classes_str += ', ' + get_typename(base_class.typename, owner_specs, header_env.mapping)

      # if name_cpp == 'vpColVector':
      cls_result = f'\tpy::class_ {python_ident} = py::class_<{name_cpp}{base_classes_str}>(submodule, "{name_python}");'
      methods_str = ''
      skip_class = False
      # Skip classes that have pure virtual methods since they cannot be instantiated
      for method in cls.methods:
        if method.pure_virtual:
          skip_class = True
          break
      # if skip_class:
      #   continue

      # Add to string representation
      to_string_str = ''
      for friend in cls.friends:
        if friend.fn is not None:
          is_ostream_operator = True
          fn = friend.fn
          if fn.return_type is None:
            is_ostream_operator = False
          else:
            return_str = get_type(fn.return_type, {}, {})
            if return_str != 'std::ostream&':
              is_ostream_operator = False
          if not is_ostream_operator:
            continue
          if is_ostream_operator:
            to_string_str = f'''
            {python_ident}.def("__repr__", []({name_cpp} &a) {{
              std::stringstream s;
              s << a;
              return s.str();
          }});'''

      for method in cls.methods:
        if method.access is not None and method.access != 'public':
          continue
        params_strs = []
        skip_method = False
        for param in method.parameters:
          param_str = get_type(param.type, owner_specs, header_env.mapping)

          if param_str in header_env.mapping: # Convert to fully qualified names
            param_str = header_env.mapping[param_str]

          if param_str is None:
            print('Skipping constructor', method)
            skip_method = True
            break
          else:
            params_strs.append(param_str)
        argument_types_str = ', '.join(params_strs)
        if skip_method:
          print(f'Skipping method because of args return type')
          continue
        if method.constructor:
          ctor_str = f'''
            {python_ident}.def(py::init<{argument_types_str}>());'''
          methods_str += ctor_str
        else:
          method_name = '::'.join([segment.name for segment in method.name.segments])
          if method.destructor or method_name.startswith('operator'):
            continue
          def_type = 'def' # Def for object methods
          pointer_to_type = f'({name_cpp}::*)'
          if method.template is not None:
            print(f'Skipping method {name_cpp}::{method_name} because it is templated')
            continue
          if method.static:
            #continue
            def_type = 'def_static'
            pointer_to_type = '(*)'
          if method.inline:
            continue
          return_type = get_type(method.return_type, owner_specs, header_env.mapping)
          if return_type is None:
            print(f'Skipping method {name_cpp}::{method_name} because of unhandled return type')
            continue
          maybe_const = ''
          if method.const:
            maybe_const = 'const'
          cast_str = f'{return_type} {pointer_to_type}({argument_types_str}) {maybe_const}'
          method_str = f'{python_ident}.{def_type}("{method_name}", static_cast<{cast_str}>(&{name_cpp}::{method_name}));\n'
          methods_str += method_str
          pass
      spec_result += cls_result
      if not skip_class:
        spec_result += methods_str
        spec_result += to_string_str
      return spec_result

    print(f'Parsing class "{cls.class_decl.typename}"')
    name_cpp_no_template = '::'.join([seg.name for seg in cls.class_decl.typename.segments])

    if self.submodule.class_should_be_ignored(name_cpp_no_template):
      return ''

    # Add base classes as requirements
    self.contains.append(name_cpp_no_template)
    for base_class in cls.class_decl.bases:
      if base_class.access == 'public':
        base_class_str_no_template = '::'.join([segment.name for segment in base_class.typename.segments])
        if base_class_str_no_template.startswith('vp'):
            self.depends.append(base_class_str_no_template)

    if cls.class_decl.template is None:
      name_python = name_cpp_no_template.replace('vp', '')
      return generate_class_with_potiental_specialization(name_python, {})
    else:
      cls_config = self.submodule.get_class_config(name_cpp_no_template)
      if cls_config is None or 'specializations' not in cls_config:
        print(f'Could not find template specialization for class {name_cpp_no_template}: skipping!')
        return ''
      else:
        specialization_strs = []
        specs = cls_config['specializations']
        template_names = [t.name for t in cls.class_decl.template.params]
        for spec in specs:
          print('AAAAA' * 100)
          python_name = spec['python_name']
          args = spec['arguments']
          assert len(template_names) == len(args), f'Specializing {name_cpp_no_template}: Template arguments are {template_names} but found specialization {args} which has the wrong number of arguments'
          spec_dict = {k[0]: k[1] for k in zip(template_names, args)}
          specialization_strs.append(generate_class_with_potiental_specialization(python_name, spec_dict))
          print(specialization_strs[-1])
        return '\n'.join(specialization_strs)
