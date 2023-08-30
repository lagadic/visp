from typing import List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData, NamespaceScope, ClassScope, parse_file
from pathlib import Path
import json
from dataclasses import dataclass
from utils import *
from methods import *


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
    self.class_decls = []
    self.contains = []
    self.depends = []
    self.generate_binding_code(content)

  def run_preprocessor(self): # TODO: run without generating a new file
    tmp_file_path = self.submodule.submodule_file_path.parent / "tmp" / self.path.name
    print(f'preprocessing {self.path}')
    argv = [
      '',
      '-D', 'vp_deprecated=',
      '-D', 'VISP_EXPORT=',
      '-I', '/home/sfelton/software/visp_build/include',
      '-I', '/usr/local/include',
      #'-I', '/usr/include',
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
    parsed_data = parse_string(content)
    self.binding_code = self.parse_data(parsed_data)


  def parse_data(self, data: ParsedData):
    result = ''
    header_env = HeaderEnvironment(data)
    print(data.namespace.doxygen)
    for cls in data.namespace.classes:
      result += self.generate_class(cls, header_env)
    for enum in data.namespace.enums:
      print('=====')
      print(enum)
      print('=====')

    return result

  def generate_class(self, cls: ClassScope, header_env: HeaderEnvironment) -> str:
    result = ''
    print('Doxygen', cls.class_decl.doxygen)
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
      self.class_decls.append(cls_result)

      contains_pure_virtual_methods = False
      # Skip classes that have pure virtual methods since they cannot be instantiated
      for method in cls.methods:
        if method.pure_virtual:
          contains_pure_virtual_methods = True
          break



      generated_methods = []
      method_strs = []
      bindable_methods_and_config, filtered_strs = get_bindable_methods_with_config(self.submodule, cls.methods,
                                                                                    name_cpp_no_template, owner_specs, header_env.mapping)
      print('\n'.join(filtered_strs))

      constructors, non_constructors = split_methods_with_config(bindable_methods_and_config, lambda m: m.constructor)

      cpp_operator_names = cpp_operator_list()
      operators, basic_methods = split_methods_with_config(non_constructors, lambda m: get_name(m.name) in cpp_operator_names)

      # Constructors definitions
      if not contains_pure_virtual_methods:
        for method, method_config in constructors:
          params_strs = [get_type(param.type, owner_specs, header_env.mapping) for param in method.parameters]
          py_arg_strs = [f'py::arg("{param.name}")' for param in method.parameters]
          ctor_str = f'''{python_ident}.{define_constructor(params_strs, py_arg_strs)};'''
          method_strs.append(ctor_str)

      #Operator definitions
      for method, method_config in operators:
        method_name = get_name(method.name)
        params_strs = [get_type(param.type, owner_specs, header_env.mapping) for param in method.parameters]
        if len(params_strs) > 1:
          print(f'Found operator {name_cpp}{method_name} with more than one parameter, skipping')
          continue
        elif len(params_strs) < 1:
          print(f'Found unary operator {name_cpp}::{method_name}, skipping')
          continue
        binary_operators = [
          ('==', 'eq'),
          ('!=', 'ne'),
          ('<', 'lt'),
          ('>', 'gt'),
          ('<=', 'le'),
          ('>=', 'ge'),
          ('+', 'add'),
          ('-', 'sub'),
          ('*', 'mul'),
          ('/', 'truediv'),
          ('%', 'mod'),
          # ('<<', 'lshift'),
          # ('>>', 'rshift'),
          ('&', 'and'),
          ('|', 'or'),
          ('^', 'xor'),
        ]
        binary_op_tuple = None
        for tuple in binary_operators:
          if method_name == f'operator{tuple[0]}':
            operator_str = f'''
{python_ident}.def("__{tuple[1]}__", [](const {name_cpp}& self, {params_strs[0]} o) {{
  return (self {tuple[0]} o);
}}, py::is_operator());'''
            method_strs.append(operator_str)
            break


      # Define classical methods
      for method, method_config in basic_methods:

        params_strs = [get_type(param.type, owner_specs, header_env.mapping) for param in method.parameters]
        py_arg_strs = [f'py::arg("{param.name}")' for param in method.parameters]

        method_name = get_name(method.name)
        py_method_name = method_config.get('custom_name') or method_name

        return_type = get_type(method.return_type, owner_specs, header_env.mapping)
        method_ref_str = ref_to_class_method(method, name_cpp, method_name, return_type, params_strs)
        method_str = define_method(py_method_name, method_ref_str, py_arg_strs, method.static)
        method_str = f'{python_ident}.{method_str};'
        method_strs.append(method_str)
        generated_methods.append((py_method_name, method))


      # Add to string representation
      to_string_str = find_and_define_repr_str(cls, name_cpp, python_ident)
      if len(to_string_str) > 0:
        method_strs.append(to_string_str)



      for enum in cls.enums:
        print('============')
        print(get_type(enum.typename, owner_specs, header_env.mapping))
        print(enum.values)
        print('=============')
      for typedef in cls.typedefs:
        print('typedef============')
        print(typedef)
        # print(get_type(typedef.type, owner_specs, header_env.mapping))
        print(typedef.name)
        print('=============')



      error_generating_overloads = get_static_and_instance_overloads(generated_methods)
      for error_overload in error_generating_overloads:
        print(f'Overload {error_overload} defined for instance and class, this will generate a pybind error')
      if len(error_generating_overloads) > 0:
        raise RuntimeError

      #spec_result += cls_result
      spec_result += '\n'.join(method_strs)
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
          python_name = spec['python_name']
          args = spec['arguments']
          assert len(template_names) == len(args), f'Specializing {name_cpp_no_template}: Template arguments are {template_names} but found specialization {args} which has the wrong number of arguments'
          spec_dict = {k[0]: k[1] for k in zip(template_names, args)}
          specialization_strs.append(generate_class_with_potiental_specialization(python_name, spec_dict))

        return '\n'.join(specialization_strs)
