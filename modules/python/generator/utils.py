from typing import List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData, NamespaceScope, ClassScope
from pathlib import Path
import json
def get_name(name: types.PQName) -> str:
  '''
  Get the fully qualified name of a type.
  Template specializations will not appear!
  '''
  return '::'.join([segment.name for segment in name.segments])

def name_is_anonymous(name: types.PQName) -> bool:
  return any(isinstance(s, types.AnonymousName) for s in name.segments)

def get_typename(typename: types.PQName, owner_specs, header_env_mapping) -> str:
  '''Resolve the string representation of a raw type, resolving template specializations and using complete typenames
  (aliases, shortened names when in same namescope).
  This does not include constness, whether it is a pointer or a ref etc.
  '''
  def segment_repr(segment: types.PQNameSegment) -> str:
    spec_str = ''
    if isinstance(segment, types.FundamentalSpecifier):
      return segment.name
    segment_name = segment.name
    if segment.name in owner_specs:
      segment_name = owner_specs[segment.name]
    if segment.name in header_env_mapping:
      segment_name = header_env_mapping[segment.name]

    if segment.specialization is not None:
      template_strs = []
      for arg in segment.specialization.args:
        template_strs.append(get_type(arg.arg, owner_specs, header_env_mapping))

      spec_str = f'<{",".join(template_strs)}>'

    return segment_name + spec_str
  return '::'.join(list(map(segment_repr, typename.segments)))

def get_type(param: Union[types.FunctionType, types.DecoratedType, types.Value], owner_specs: Dict[str, str], header_env_mapping: Dict[str, str]) -> Optional[str]:
  '''
  Get the type of a parameter. Compared to get_typename, this function resolves the parameter's constness, whether it is a ref, moveref or pointer.
  '''
  if isinstance(param, types.Value):
    return ''.join([token.value for token in param.tokens])
  if isinstance(param, types.FunctionType):
    return_type = get_type(param.return_type, owner_specs, header_env_mapping)
    param_types = [get_type(p.type, owner_specs, header_env_mapping) for p in param.parameters]
    return f'{return_type}({", ".join(param_types)})'
  if isinstance(param, types.Type):
    repr_str = get_typename(param.typename, owner_specs, header_env_mapping)
    split = repr_str.split('<')
    if split[0] in header_env_mapping:
      split[0] = header_env_mapping[split[0]]
    repr_str = '<'.join(split)
    if param.const:
      repr_str = 'const ' + repr_str
    return repr_str
  elif isinstance(param, types.Reference):
    repr_str = get_type(param.ref_to, owner_specs, header_env_mapping)
    if repr_str is not None:
      return repr_str + '&'
    else:
      return None
  elif isinstance(param, types.MoveReference):
    repr_str = get_type(param.moveref_to, owner_specs, header_env_mapping)
    if repr_str is not None:
      return repr_str + '&&'
    else:
      return None
  elif isinstance(param, types.Pointer):
    repr_str = get_type(param.ptr_to, owner_specs, header_env_mapping)
    if repr_str is not None:
      return repr_str + '*'
    else:
      return None
  else:
    return None

def is_pointer_to_const_cstr(param: types.Pointer) -> bool:
  '''
  Whether the passed in pointer is of type const char*
  '''
  ptr_to = param.ptr_to
  if isinstance(ptr_to, types.Type):
    if get_typename(ptr_to.typename, {}, {}) == 'char' and ptr_to.const:
      return True

  return False

IMMUTABLE_TYPES = [
  'double', 'float',
  'int', 'int32_t', 'int16_t', 'int8_t',
  'unsigned int', 'uint8_t', 'uint16_t', 'uint32_t', 'uint64_t',
  'long', 'long long', 'unsigned long',
  'unsigned char', 'char', 'std::string'
]

IMMUTABLE_CONTAINERS = [
  'std::vector', 'std::list'
]

def is_non_const_ref_to_immutable_type(param: types.DecoratedType) -> bool:
  '''
  Returns true if the parameter is a mutable reference to an immutable type in Python.
  In python immutable types are: ints, double, string (std::string or char*)
  This also takes into account STL containers that are converted from Python containers:
    - a Python list is converted to a std::vector. If it is passed by ref, the changes applied to the vector are not propagated to the Python list
    - Same for maps
    - See https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
  '''
  if not isinstance(param, types.Reference):
    return False
  if param.ref_to.const:
    return False
  param_name = get_typename(param.ref_to.typename, {}, {})
  if param_name in IMMUTABLE_TYPES:
    return True
  is_immut_container = any(map(lambda c: param_name.startswith(c), IMMUTABLE_CONTAINERS))
  return is_immut_container


def is_unsupported_return_type(param: Union[types.FunctionType, types.DecoratedType]) -> bool:
  '''
  Returns whether the passed param is supported as a return type for automatic code generation.
  Pointers, arrays, functions are not supported.
  '''
  if isinstance(param, types.FunctionType):
    return True
  if isinstance(param, types.Array):
    return True
  if isinstance(param, types.Type):
    return False
  if isinstance(param, types.Reference):
    return is_unsupported_return_type(param.ref_to)
  if isinstance(param, types.MoveReference):
    return False
  if isinstance(param, types.Pointer):
    return not is_pointer_to_const_cstr(param)
  return True

def is_unsupported_argument_type(param: Union[types.FunctionType, types.DecoratedType]) -> bool:
  '''
  Return whether the passed param is supported for automatic code generation.
  Pointers, arrays, functions are not supported.
  '''
  if isinstance(param, types.FunctionType):
    return True
  if isinstance(param, types.Array):
    return True
  if isinstance(param, types.Type):
    return False
  if isinstance(param, types.Reference):
    return is_unsupported_argument_type(param.ref_to)
  if isinstance(param, types.MoveReference):
    return True
  if isinstance(param, types.Pointer):
    return not is_pointer_to_const_cstr(param)
  return True

def get_method_signature(name: str, return_type: str, params: List[str]) -> str:
  '''
  Get the method signature. This does not include method constness or staticness
  '''
  return f'{return_type} {name}({", ".join(params)})'

def method_matches_config(method: types.Method, config: Dict, owner_specs, header_mapping) -> bool:
  '''
  Returns whether a method matches a configuration dict.
  Matching is performed on the method signature.
  The config should come from those defined in the submodule and its json file
  '''
  params_strs = []
  if config['static'] != method.static:
    return False
  params_strs = [get_type(param.type, owner_specs, header_mapping) or '<unparsed>' for param in method.parameters]
  signature = get_method_signature(get_name(method.name), get_type(method.return_type, owner_specs, header_mapping) or '', params_strs)
  config_signature = config['signature']
  for template in owner_specs:
    config_signature = config_signature.replace(f'<{template}>', f'<{owner_specs[template]}>')

  if signature.replace(' ', '') != config_signature.replace(' ', ''):
    return False

  return True

def get_static_and_instance_overloads(methods: List[Tuple[str, types.Method]]) -> Set[str]:
  '''
  Return the set of method names that have static and non-static versions.
  This is not allowed in PyBind, so this function can be used to raise an error if the resulting set is not empty
  '''
  instance_methods = set([method[0] for method in methods if not method[1].static])
  static_methods = set([method[0] for method in methods if method[1].static])
  return instance_methods.intersection(static_methods)
