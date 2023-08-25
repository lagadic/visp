from typing import List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData, NamespaceScope, ClassScope
from pathlib import Path
import json

def get_typename(typename: types.PQName, owner_specs, header_env_mapping) -> str:
  '''Resolve the string representation of a raw type, resolving template specializations and using complete typenames
  (aliases, shortened name when in same namescope).
  '''
  def segment_repr(segment: types.PQNameSegment) -> str:
    spec_str = ''
    if isinstance(segment, types.FundamentalSpecifier):
      return segment.name
    if segment.name in owner_specs:
      segment.name = owner_specs[segment.name]

    if segment.specialization is not None:
      template_strs = []
      for arg in segment.specialization.args:
        template_strs.append(get_type(arg.arg, owner_specs, header_env_mapping))

      spec_str = f'<{",".join(template_strs)}>'

    return segment.name + spec_str
  return '::'.join(list(map(segment_repr, typename.segments)))

def get_type(param: Union[types.FunctionType, types.DecoratedType, types.Value], owner_specs: Dict[str, str], header_env_mapping: Dict[str, str]) -> Optional[str]:

  if isinstance(param, types.Value):
    return ''.join([token.value for token in param.tokens])
  if isinstance(param, types.FunctionType):
    return 'FUNCTION'
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

def get_method_signature(name: str, return_type: str, params: List[str]) -> str:
  print(return_type, name)
  return f'{return_type} {name}({", ".join(params)})'

def method_matches_config(method: types.Method, config: Dict, owner_specs, header_mapping) -> bool:
  params_strs = []
  if config['static'] != method.static:
    return False
  params_strs = [get_type(param.type, owner_specs, header_mapping) or '<unparsed>' for param in method.parameters]
  signature = get_method_signature(method.name, get_type(method.return_type, owner_specs, header_mapping) or '', params_strs)
  if signature.replace(' ', '') != config['signature']:
    return False

  return True