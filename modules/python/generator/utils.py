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
  return '::'.join([segment.name for segment in name.segments])

def get_typename(typename: types.PQName, owner_specs, header_env_mapping) -> str:
  '''Resolve the string representation of a raw type, resolving template specializations and using complete typenames
  (aliases, shortened names when in same namescope).
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

  if isinstance(param, types.Value):
    return ''.join([token.value for token in param.tokens])

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
  else:
    return None

def is_pointer_to_const_cstr(param: types.Pointer) -> bool:
  ptr_to = param.ptr_to
  if isinstance(ptr_to, types.Type):
    if get_typename(ptr_to, {}, {}) == 'char' and ptr_to.const:
      return True

  return False

def is_unsupported_return_type(param: Union[types.FunctionType, types.DecoratedType]) -> bool:
  '''
  Return whether the passed param is supported as a return type for automatic code generation.
  Pointers, arrays, functions are not supported.
  '''
  if isinstance(param, types.FunctionType):
    return True
  if isinstance(param, types.Array):
    return True
  if isinstance(param, types.Type):
    return False
  if isinstance(param, types.Reference):
    return False
  if isinstance(param, types.Pointer):
    return not is_pointer_to_const_cstr(param)

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
    return False
  if isinstance(param, types.Pointer):
    return not is_pointer_to_const_cstr(param)



def get_method_signature(name: str, return_type: str, params: List[str]) -> str:
  return f'{return_type} {name}({", ".join(params)})'

def method_matches_config(method: types.Method, config: Dict, owner_specs, header_mapping) -> bool:
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
  instance_methods = set([method[0] for method in methods if not method[1].static])
  static_methods = set([method[0] for method in methods if method[1].static])
  return instance_methods.intersection(static_methods)
