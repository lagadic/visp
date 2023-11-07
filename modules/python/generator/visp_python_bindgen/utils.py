from typing import List, Optional, Set, Tuple, Dict, Union
from enum import Enum
from dataclasses import dataclass
from cxxheaderparser.tokfmt import tokfmt
from cxxheaderparser import types
from cxxheaderparser.simple import NamespaceScope, ClassScope

from visp_python_bindgen.generator_config import GeneratorConfig


@dataclass
class BoundObjectNames:
  '''
  The different names that link to a cpp class
  '''
  python_ident: str # the identifier (variable) that defines the pybind object
  python_name: str # the name exposed in Python
  cpp_no_template_name: str # C++ name without any template => vpArray2D<T> becomes vpArray2D (useful for dependencies)
  cpp_name: str # C++ name with templates

class GenerationObjectType(Enum):
  Enum = 'enum'
  Class = 'class'
  Function = 'function'
  Namespace = 'namespace'

@dataclass
class ClassBindingDefinitions:
  '''
  Class containing the bindings for a single class
  '''
  fields: Dict[str, str]
  methods: Dict[str, List[str]] # Mapping from python method name to the bindings definitions. There can be overloads

@dataclass
class SingleObjectBindings:
  '''
  Result of running the binding generator for a single type (class, enum)
  '''
  object_names: BoundObjectNames
  declaration: Optional[str] # Pybind type instanciation, eg py::class_<> ... (if any)
  definitions: Union[List[str], ClassBindingDefinitions] # List of method bindings, attributes etc for this pybind object
  object_type: GenerationObjectType # Type of the object

class BindingsContainer:
  def __init__(self):
    self.object_bindings: List[SingleObjectBindings] = []

  def add_bindings(self, other: SingleObjectBindings) -> None:
    self.object_bindings.append(other)

  def find_bindings(self, cpp_name: str) -> Optional[SingleObjectBindings]:
    for sob in self.object_bindings:
      if sob.object_names.cpp_name == cpp_name:
        return sob
    return None


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
    if isinstance(segment, types.FundamentalSpecifier):
      return segment.name
    segment_name = segment.name
    if segment.name in owner_specs:
      segment_name = owner_specs[segment.name]
    if segment.name in header_env_mapping:
      segment_name = header_env_mapping[segment.name]

    spec_str = ''
    if segment.specialization is not None:
      template_strs = []

      for arg in segment.specialization.args:
        template_strs.append(get_type(arg.arg, owner_specs, header_env_mapping))
      spec_str = f'<{", ".join(template_strs)}>'

    return segment_name + spec_str
  segment_reprs = list(map(segment_repr, typename.segments))
  # Through environment mapping, it is possible that a segment is resolved into two "segments"
  # E.g. a class "vpA" in a namespace "vp" is resolve to "vp::vpA"
  # If we resolve for the segments ["vp", "vpA"], we will obtain "vp::vp::vpA"
  # We must thus check that this is not the case and filter out redundant segments (in this case, "vp")
  final_segment_reprs = [segment_reprs[-1]] # this is always final

  for i in range(len(segment_reprs) - 1):
    all_segs_prefix = '::'.join(segment_reprs[i:-1])
    # We only compare with the last one (which should be a class name) since this is what is resolved to a complete name
    # TODO: When the problem arises with a templated type, this may fail.
    if not final_segment_reprs[-1].startswith(all_segs_prefix + '::'):
      final_segment_reprs.insert(len(final_segment_reprs) - 1, segment_reprs[i])
    else:
      break

  return '::'.join(final_segment_reprs)

def get_type(param: Union[types.FunctionType, types.DecoratedType, types.Value], owner_specs: Dict[str, str], header_env_mapping: Dict[str, str]) -> Optional[str]:
  '''
  Get the type of a parameter. Compared to get_typename, this function resolves the parameter's constness, whether it is a ref, moveref or pointer.
  '''
  if isinstance(param, types.Value):
    return tokfmt(param.tokens) # This can appear when parsing templated types! Example: std::map<std::string, const vpImage<vpRGBa>*> as in MBT
  if isinstance(param, types.FunctionType):
    return_type = get_type(param.return_type, owner_specs, header_env_mapping)
    param_types = [get_type(p.type, owner_specs, header_env_mapping) for p in param.parameters]
    return f'{return_type}({", ".join(param_types)})'
  if isinstance(param, types.Type):
    repr_str = get_typename(param.typename, owner_specs, header_env_mapping)

    # split = repr_str.split('<')
    # if split[0] in header_env_mapping:
    #   split[0] = header_env_mapping[split[0]]
    # repr_str = '<'.join(split)
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

def get_type_for_declaration(param: Union[types.FunctionType, types.DecoratedType, types.Value], owner_specs: Dict[str, str], header_env_mapping: Dict[str, str]) -> Optional[str]:
  '''
  Type string for when we want to declare a variable of a certain type.
  We cannot declare an lref without initializing it (which we can't really do when generating code)
  Example use case: declaring a variable inside a lambda.
  '''

  if isinstance(param, types.Reference):
    return get_type(param.ref_to, owner_specs, header_env_mapping)
  else:
    return get_type(param, owner_specs, header_env_mapping)

def fetch_fully_qualified_id(scope: Union[NamespaceScope, ClassScope], segments: List[str]) -> Union[None, types.EnumDecl, NamespaceScope, ClassScope]:
  '''
  Retrieve the declaration of an object from its fully qualified name.
  This can be useful when a symbol is reference in two places:
  such as in the following header:

  class vpA {
  private:
    enum vpEnum: unsigned int;
  };
  enum vpA::vpEnum : unsigned int {...};

  In this case, the vpA::vpEnum symbol's visibility (here it is actually private) is undefined in cxxheaderparser (None) when looking at enum declarations outside the class
  Here, calling this method with the name vpA::vpEnum will retrieve the enum declaration in the class vpA, for which the visibility is correctly set to private.
  '''

  if len(segments) == 0:
    return scope

  seg = segments[0]
  if isinstance(scope, NamespaceScope):
    for ns in scope.namespaces:
      if ns == seg:
        return fetch_fully_qualified_id(scope.namespaces[ns], segments[1:])
  for cls in scope.classes:
    if get_name(cls.class_decl.typename) == seg:
      return fetch_fully_qualified_id(cls, segments[1:])
  if len(segments) == 1: # Test objects that cannot have children
    for enum in scope.enums:
      if not name_is_anonymous(enum.typename) and get_name(enum.typename) == seg:
        return enum

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
  param_type = get_typename(param.ref_to.typename, {}, {})
  if GeneratorConfig.is_immutable_type(param_type):
    return True
  return GeneratorConfig.is_immutable_type(param_type) or GeneratorConfig.is_immutable_container(param_type)



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
    return not is_pointer_to_const_cstr(param) # Pointers of type "const char*" are handled by pybind
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
