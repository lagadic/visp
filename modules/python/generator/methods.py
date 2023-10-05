from typing import Any, Callable, List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser.visitor import CxxVisitor
from cxxheaderparser import types
from cxxheaderparser.simple import parse_string, ParsedData, NamespaceScope, ClassScope, parse_file
from pathlib import Path
import json
from utils import *
from dataclasses import dataclass
from enum import Enum

from typing import TYPE_CHECKING
from doc_parser import MethodDocSignature
if TYPE_CHECKING:
  from submodule import Submodule
  from header import HeaderFile, HeaderEnvironment, BoundObjectNames

def cpp_operator_list():
  '''
  List of cpp methods that are considered operators.
  Not all of them have a direct mapping to Python
  '''
  symbols = [
    '-', '+', '*', '/',
    '++', '--', '%',
    '==', '=', '!=',
    '>', '>=', '<', '<=', '<=>',
    '>>', '<<', '^',
    '~', '&', '|',
    '&&', '||', '!',
    '=', '+=', '-=', '*=', '/=',
    '%=', '&=', '|=', '^=',
    '<<=', '>>=',
    '[]', '->', '->*',
    '()', ','
  ]
  return [f'operator{s}' for s in symbols] + ['operator']

def supported_const_return_binary_op_map():
  '''
  Mapping between CPP operator symbols and their python equivalence, for binary operators that return a value and do not modify self.
  a binary operator takes as argument self and an another parameter.
  '''
  return {
    '==': 'eq',
    '!=': 'ne',
    '<': 'lt',
    '>': 'gt',
    '<=': 'le',
    '>=': 'ge',
    '+': 'add',
    '-': 'sub',
    '*': 'mul',
    '/': 'truediv',
    '%': 'mod',
    '&': 'and',
    '|': 'or',
    '^': 'xor',
  }
def supported_in_place_binary_op_map():
  return {
    '+=': 'iadd',
    '*=': 'imul',
    '-=': 'isub',
    '/=': 'itruediv',
  }
def find_and_define_repr_str(cls: ClassScope, cls_name: str, python_ident: str) -> str:
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
        return f'''
{python_ident}.def("__repr__", []({cls_name} &a) {{
  std::stringstream s;
  s << a;
  return s.str();
}});'''
  return ''

def ref_to_class_method(method: types.Method, cls_name: str, method_name: str, return_type: str, params: List[str]) -> str:
  maybe_const = ''
  if method.const:
    maybe_const = 'const'
  pointer_to_type = f'({cls_name}::*)' if not method.static else '(*)'
  cast_str = f'{return_type} {pointer_to_type}({", ".join(params)}) {maybe_const}'
  return f'static_cast<{cast_str}>(&{cls_name}::{method_name})'

def method_def(py_name: str, method: str, additional_args: List[str], static: bool) -> str:
  def_type = 'def' if not static else 'def_static'
  additional_args_str = ', '.join(additional_args)
  if len(additional_args) > 0:
    additional_args_str = ', ' + additional_args_str
  return f'{def_type}("{py_name}", {method}{additional_args_str})'

def tokens_to_str(tokens: List[types.Token]) -> str:
  return ''.join([token.value for token in tokens])

def get_py_args(parameters: List[types.Parameter], specs, env_mapping) -> List[str]:
  '''
  Get the py::arg parameters of a function binding definition.
  They are used to give the argument their names in the doc and the api.
  They can also have default values (optional arguments).
  '''
  py_args = []
  for parameter in parameters:
    if parameter.default is None:
      py_args.append(f'py::arg("{parameter.name}")')
    else:
      t = parameter.type
      gt = lambda typename: get_typename(typename, specs, env_mapping)
      if isinstance(t, types.Type):
        type_name = gt(t.typename)
      elif isinstance(t, types.Reference):
        type_name = gt(t.ref_to.typename)
      else:
        type_name = ''

      if type_name.startswith('std::vector'):
        default_value = type_name + '()'
        default_value_rep = '[]'
      elif type_name.startswith('std::optional'):
        default_value = type_name + '{}'
        default_value_rep = 'None'
      else:
        default_value = tokens_to_str(parameter.default.tokens)
        if default_value in ['nullptr', 'NULL']:
          full_typename = get_type(t, specs, env_mapping)
          default_value = f'static_cast<{full_typename}>(nullptr)'
          default_value_rep = 'None'
        else:
          default_value_rep = default_value.strip('"') # To handle default char* and std::string args
          default_value = env_mapping.get(default_value) or default_value

      py_args.append(f'py::arg_v("{parameter.name}", {default_value}, "{default_value_rep}")')

  return py_args

def define_method(method: types.Method, method_config: Dict, is_class_method, specs: Dict, header: 'HeaderFile', header_env: 'HeaderEnvironment', bound_object: 'BoundObjectNames'):
  params_strs = [get_type(param.type, specs, header_env.mapping) for param in method.parameters]
  py_arg_strs = get_py_args(method.parameters, specs, header_env.mapping)
  method_name = get_name(method.name)
  py_method_name = method_config.get('custom_name') or method_name
  return_type = get_type(method.return_type, specs, header_env.mapping)

  # Detect input and output parameters for a method
  use_default_param_policy = method_config['use_default_param_policy']
  param_is_input, param_is_output = method_config['param_is_input'], method_config['param_is_output']
  if use_default_param_policy or param_is_input is None and param_is_output is None:
    param_is_input = [True for _ in range(len(method.parameters))]
    param_is_output = list(map(lambda param: is_non_const_ref_to_immutable_type(param.type), method.parameters))
    if any(param_is_output): # Emit a warning when using default policy
      method_signature = get_method_signature(method_name,
                                              get_type(method.return_type, {}, header_env.mapping),
                                              [get_type(param.type, {}, header_env.mapping) for param in method.parameters])
      header.submodule.report.add_default_policy_method(bound_object.cpp_no_template_name, method, method_signature, param_is_input, param_is_output)
  py_arg_strs = [py_arg_strs[i] for i in range(len(params_strs)) if param_is_input[i]]
  # Get parameter names
  param_names = [param.name or 'arg' + str(i) for i, param in enumerate(method.parameters)]
  input_param_names = [param_names[i] for i in range(len(param_is_input)) if param_is_input[i]]
  output_param_names = [param_names[i] for i in range(len(param_is_output)) if param_is_output[i]]

  # Fetch documentation if available
  if header.documentation_holder is not None:
    method_doc_signature = MethodDocSignature(method_name,
                                              get_type(method.return_type, {}, header_env.mapping), # Don't use specializations so that we can match with doc
                                              [get_type(param.type, {}, header_env.mapping) for param in method.parameters],
                                              method.const, method.static)
    method_doc = header.documentation_holder.get_documentation_for_method(bound_object.cpp_no_template_name, method_doc_signature, {}, specs, input_param_names, output_param_names)
    if method_doc is None:
      print(f'Could not find documentation for {bound_object.cpp_name}::{method_name}!')
    else:
      py_arg_strs = [method_doc.documentation] + py_arg_strs

  # If a function has refs to immutable params, we need to return them.
  should_wrap_for_tuple_return = param_is_output is not None and any(param_is_output)
  if should_wrap_for_tuple_return:

    # Arguments that are inputs to the lambda function that wraps the ViSP function
    input_param_types = [params_strs[i] for i in range(len(param_is_input)) if param_is_input[i]]
    params_with_names = [t + ' ' + name for t, name in zip(input_param_types, input_param_names)]

    # Params that are only outputs: they should be declared in function. Assume that they are default constructible
    param_is_only_output = [not is_input and is_output for is_input, is_output in zip(param_is_input, param_is_output)]
    param_declarations = [f'{params_strs[i]} {param_names[i]};' for i in range(len(param_is_only_output)) if param_is_only_output[i]]
    param_declarations = '\n'.join(param_declarations)
    if not method.static:
      self_param_with_name = bound_object.cpp_name + '& self'
      method_caller = 'self.'
    else:
      self_param_with_name = None
      method_caller = bound_object.cpp_name + '::'

    if return_type is None or return_type == 'void':
      maybe_get_return = ''
      maybe_return_in_tuple = ''
    else:
      maybe_get_return = 'auto res = '
      maybe_return_in_tuple = 'res, '

    if len(output_param_names) == 1 and (return_type is None or return_type == 'void'):
      return_str = output_param_names[0]
    else:
      return_str = f'std::make_tuple({maybe_return_in_tuple}{", ".join(output_param_names)})'

    lambda_body = f'''
      {param_declarations}
      {maybe_get_return}{method_caller}{method_name}({", ".join(param_names)});
      return {return_str};
    '''
    final_lambda_params = [self_param_with_name] + params_with_names if self_param_with_name is not None else params_with_names
    method_body_str = define_lambda('', final_lambda_params, None, lambda_body)

  else:
    method_body_str = ref_to_class_method(method, bound_object.cpp_name, method_name, return_type, params_strs)

  method_str = method_def(py_method_name, method_body_str, py_arg_strs, method.static)
  method_str = f'{bound_object.python_ident}.{method_str};'
  return method_str, (py_method_name, method)

def define_constructor(params: List[str], additional_args: List[str]) -> str:
  additional_args_str = ', '.join(additional_args)
  if len(additional_args) > 0:
    additional_args_str = ', ' + additional_args_str
  return f'def(py::init<{", ".join(params)}>(){additional_args_str})'

def define_lambda(capture: str, params: List[str], return_type: Optional[str], body: str) -> str:
  return_str = f'-> {return_type}' if return_type else ''
  return f'''
[{capture}]({", ".join(params)}) {return_str} {{
  {body}
}}

'''

class NotGeneratedReason(Enum):
  UserIgnored = 'user_ignored',
  Access = 'access',
  Destructor = 'destructor',
  ReturnType = 'return_type'
  ArgumentType = 'argument_type'
  PureVirtual = 'pure_virtual'
  UnspecifiedTemplateSpecialization = 'missing_template'
  NotHandled = 'not_handled'

  @staticmethod
  def is_non_trivial_reason(reason: 'NotGeneratedReason') -> bool:
    return reason in [NotGeneratedReason.ArgumentType,
                       NotGeneratedReason.ReturnType,
                       NotGeneratedReason.UnspecifiedTemplateSpecialization,
                       NotGeneratedReason.NotHandled]

@dataclass
class RejectedMethod:
  cls_name: Optional[str]
  method: types.Method
  method_config: Dict[str, Any]
  signature: str
  rejection_reason: NotGeneratedReason

def get_bindable_methods_with_config(submodule: 'Submodule', methods: List[types.Method], cls_name: str, specializations, mapping) -> Tuple[List[Tuple[types.Method, Dict]], List[RejectedMethod]]:
  bindable_methods = []
  rejected_methods = []
  # Order of predicates is important: The first predicate that matches will be the one shown in the log, and they do not all have the same importance
  filtering_predicates_and_motives = [
    (lambda _, conf: conf['ignore'], NotGeneratedReason.UserIgnored),
    (lambda m, _: m.pure_virtual, NotGeneratedReason.PureVirtual),
    (lambda m, _: m.access is None or m.access != 'public', NotGeneratedReason.Access),
    (lambda m, _: m.destructor, NotGeneratedReason.Destructor),
    (lambda m, conf: m.template is not None and (conf.get('specializations') is None or len(conf['specializations']) == 0), NotGeneratedReason.UnspecifiedTemplateSpecialization),
    (lambda m, _: any(is_unsupported_argument_type(param.type) for param in m.parameters), NotGeneratedReason.ArgumentType),
    (lambda m, _: not m.constructor and is_unsupported_return_type(m.return_type), NotGeneratedReason.ReturnType)
  ]
  for method in methods:
    method_config = submodule.get_method_config(cls_name, method, specializations, mapping)
    method_can_be_bound = True
    for predicate, motive in filtering_predicates_and_motives:
      if predicate(method, method_config):
        return_str = '' if method.return_type is None else (get_type(method.return_type, specializations, mapping) or '<unparsed>')
        method_name = '::'.join(seg.name for seg in method.name.segments)
        param_strs = [get_type(param.type, specializations, mapping) or '<unparsed>' for param in method.parameters]
        rejected_methods.append(RejectedMethod(cls_name, method, method_config, get_method_signature(method_name, return_str, param_strs), motive))
        method_can_be_bound = False
        break
    if method_can_be_bound:
      bindable_methods.append((method, method_config))

  return bindable_methods, rejected_methods



def split_methods_with_config(methods: List[Tuple[types.Method, Dict]], predicate: Callable[[types.Method], bool]) -> Tuple[List[Tuple[types.Method, Dict]], List[Tuple[types.Method, Dict]]]:
  matching = []
  non_matching = []
  for method, method_config in methods:
    if predicate(method):
      matching.append((method, method_config))
    else:
      non_matching.append((method, method_config))
  return matching, non_matching
