#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP Python bindings generator
#
#############################################################################

import logging
from typing import Any, Callable, List, Optional, Tuple, Dict
from enum import Enum
from dataclasses import dataclass

from cxxheaderparser import types
from cxxheaderparser.simple import ClassScope

from visp_python_bindgen.doc_parser import MethodDocSignature
from visp_python_bindgen.utils import *
from visp_python_bindgen.generator_config import GeneratorConfig

from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from visp_python_bindgen.submodule import Submodule
  from visp_python_bindgen.header import HeaderFile, HeaderEnvironment, BoundObjectNames

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

def lambda_const_return_binary_op(python_ident: str, python_op_name: str, cpp_op: str, method_is_const: bool,
                                  cpp_type: str, param_type: str, return_type: str, py_args: List[str]) -> str:
  return f'''
{python_ident}.def("__{python_op_name}__", []({"const" if method_is_const else ""} {cpp_type}& self, {param_type} o) -> {return_type} {{
  return (self {cpp_op} o);
}}, {", ".join(py_args)});'''

def supported_in_place_binary_op_map():
  return {
    '+=': 'iadd',
    '*=': 'imul',
    '-=': 'isub',
    '/=': 'itruediv',
  }

def lambda_in_place_binary_op(python_ident: str, python_op_name: str, cpp_op: str, method_is_const: bool,
                              cpp_type: str, param_type: str, return_type: str, py_args: List[str]) -> str:
  return f'''
{python_ident}.def("__{python_op_name}__", []({"const" if method_is_const else ""} {cpp_type}& self, {param_type} o) -> {return_type} {{
  self {cpp_op} o;
  return self;
}}, {", ".join(py_args)});'''

def supported_const_return_unary_op_map():
  return {
    '-': 'neg',
    '~': 'invert',
  }

def lambda_const_return_unary_op(python_ident: str, python_op_name: str, cpp_op: str, method_is_const: bool,
                          cpp_type: str, return_type: str, py_args: List[str]) -> str:
  return f'''
{python_ident}.def("__{python_op_name}__", []({"const" if method_is_const else ""} {cpp_type}& self) -> {return_type} {{
  return {cpp_op}self;
}}, {", ".join(py_args)});'''

def supported_nary_op_map():
  return {
    '()': 'call',
  }

def lambda_nary_op(python_ident: str, python_op_name: str, cpp_op: str, method_is_const: bool,
                    cpp_type: str, param_types: List[str], return_type: str, py_args: List[str]) -> str:
  param_names = [f'arg{i}' for i in range(len(param_types))]
  param_types_and_names = [f'{t} {n}' for t,n in zip(param_types, param_names)]
  maybe_return = '' if return_type == 'void' else 'return'

  return f'''
{python_ident}.def("__{python_op_name}__", []({"const" if method_is_const else ""} {cpp_type}& self, {",".join(param_types_and_names)}) -> {return_type} {{
  {maybe_return} self.operator{cpp_op}({",".join(param_names)});
}}, {", ".join(py_args)});'''


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

def ref_to_function(method_name: str, return_type: str, params: List[str]) -> str:
  pointer_to_type = '(*)'
  cast_str = f'{return_type} {pointer_to_type}({", ".join(params)})'
  return f'static_cast<{cast_str}>(&{method_name})'

def method_def(py_name: str, method: str, additional_args: List[str], static: bool) -> str:
  def_type = 'def' if not static else 'def_static'
  additional_args_str = ', '.join(additional_args)
  if len(additional_args) > 0:
    additional_args_str = ', ' + additional_args_str
  return f'{def_type}("{py_name}", {method}{additional_args_str})'

def tokens_to_str(tokens: List[types.Token]) -> str:
  return ''.join([token.value for token in tokens])

def parameter_can_have_default_value(parameter: types.Parameter, specs, env_mapping) -> bool:
  '''
  Return whether an argument can have a default value.
  This is important! In python, default argument are instanciated only once (when the package is imported), while this is not the case in C++
  We need to be careful in what we allow
  '''
  t = parameter.type
  gt = lambda typename: get_typename(typename, specs, env_mapping)
  is_const = False
  if isinstance(t, types.Type):
    type_name = gt(t.typename)
    is_const =  t.const
  elif isinstance(t, types.Reference):
    type_name = gt(t.ref_to.typename)
    is_const = t.ref_to.const
  elif isinstance(t, types.Pointer):
    type_name = gt(t.ptr_to.typename)
    is_const = t.ptr_to.const
  else:
    type_name = ''
  if GeneratorConfig.is_forbidden_default_argument_type(type_name):
    return False

  if is_const: # Parameter is const, so we can safely give a default value knowing it won't be modified
    return True
  if GeneratorConfig.is_immutable_type(type_name): # Immutable type on python side
    return True

  return False

def get_py_args(parameters: List[types.Parameter], specs, env_mapping) -> List[str]:
  '''
  Get the py::arg parameters of a function binding definition.
  They are used to give the argument their names in the doc and the api.
  They can also have default values (optional arguments).
  '''
  def make_arg(name: str) -> str:
    return f'py::arg("{name}")'
  py_args = []
  arg_index = 0
  for parameter in parameters:
    parameter_name = parameter.name
    if parameter_name is None:
      parameter_name = f'arg{arg_index}'

    if parameter.default is None or not parameter_can_have_default_value(parameter, specs, env_mapping):
      py_args.append(make_arg(parameter_name))
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
          default_value_rep = default_value.strip('"') # To handle default char* and raw std::string args
          default_value_rep = default_value_rep.replace('"', '\"') # Escape inner quotes in std::string args like std::string("hello"). This would break parsing at compile time
          default_value = env_mapping.get(default_value) or default_value

      py_args.append(f'py::arg_v("{parameter_name}", {default_value}, "{default_value_rep}")')
    arg_index += 1

  return py_args

def define_method(method: types.Method, method_config: Dict, is_class_method, specs: Dict, header: 'HeaderFile', header_env: 'HeaderEnvironment', bound_object: 'BoundObjectNames'):
  params_strs = [get_type(param.type, specs, header_env.mapping) for param in method.parameters]
  py_arg_strs = get_py_args(method.parameters, specs, header_env.mapping)
  method_name = get_name(method.name)
  py_method_name = method_config.get('custom_name') or method_name
  return_type = get_type(method.return_type, specs, header_env.mapping)
  param_is_function_ptr = [is_function_pointer(param.type) for param in method.parameters]

  method_signature = get_method_signature(method_name,
                                          get_type(method.return_type, {}, header_env.mapping),
                                          [get_type(param.type, {}, header_env.mapping) for param in method.parameters])

  # Detect input and output parameters for a method
  use_default_param_policy = method_config['use_default_param_policy']
  param_is_input, param_is_output = method_config['param_is_input'], method_config['param_is_output']
  if use_default_param_policy or param_is_input is None and param_is_output is None:
    param_is_input = [True for _ in range(len(method.parameters))]
    param_is_output = list(map(lambda param: is_non_const_ref_to_immutable_type(param.type), method.parameters))
    if any(param_is_output): # Emit a warning when using default policy
      header.submodule.report.add_default_policy_method(bound_object.cpp_no_template_name, method, method_signature, param_is_input, param_is_output)

  # Pybind arguments
  # Only use py::arg for input values (error otherwise)
  py_arg_strs = [py_arg_strs[i] for i in range(len(params_strs)) if param_is_input[i]]

  pybind_options = py_arg_strs
  # Custom return policy
  return_policy = method_config.get('return_policy')
  if return_policy is not None:
    pybind_options.append(f'py::return_value_policy::{return_policy}')

  # Keep alive values: avoid memory leaks
  keep_alives = method_config.get('keep_alive')
  keep_alive_strs = []
  if keep_alives is not None:
    def make_keep_alive_str(values) -> str:
      assert len(values) == 2 and isinstance(values[0], int) and isinstance(values[1], int), 'Tried to make keep alive with incorrect values'
      return f'py::keep_alive<{values[0]}, {values[1]}>()'
    if not isinstance(keep_alives, list) or len(keep_alives) == 0:
      raise RuntimeError(f'Keep alive value should be a list of int or a list of list of ints (multiple args kept alive), but got {keep_alives} for method {method_signature}')
    if isinstance(keep_alives[0], int):
      keep_alive_strs.append(make_keep_alive_str(keep_alives))
    else:
      for keep_alive in keep_alives:
        keep_alive_strs.append(make_keep_alive_str(keep_alive))
  pybind_options.extend(keep_alive_strs)

  # Get parameter names
  param_names = [param.name or 'arg' + str(i) for i, param in enumerate(method.parameters)]
  input_param_names = [param_names[i] for i in range(len(param_is_input)) if param_is_input[i]]
  output_param_names = [param_names[i] for i in range(len(param_is_output)) if param_is_output[i]]
  output_param_is_ref = [isinstance(method.parameters[i], types.Reference) for i in range(len(params_strs)) if param_is_output[i]]

  # Fetch documentation if available
  if header.documentation_holder is not None:
    if is_class_method:
      method_doc_signature = MethodDocSignature(method_name,
                                                get_type(method.return_type, {}, header_env.mapping), # Don't use specializations so that we can match with doc
                                                [get_type(param.type, {}, header_env.mapping) for param in method.parameters],
                                                method.const, method.static)
    else:
      method_doc_signature = MethodDocSignature(method_name,
                                                get_type(method.return_type, {}, header_env.mapping), # Don't use specializations so that we can match with doc
                                                [get_type(param.type, {}, header_env.mapping) for param in method.parameters],
                                                True, True)
    method_doc = header.documentation_holder.get_documentation_for_method(bound_object.cpp_no_template_name, method_doc_signature, {}, specs, input_param_names, output_param_names)
    if method_doc is None:
      logging.warning(f'Could not find documentation for {bound_object.cpp_name}::{method_name}!')
    else:
      pybind_options = [method_doc.documentation] + pybind_options



  # If a function has refs to immutable params, we need to return them.
  # Also true if user has specified input cpp params as output python params
  should_wrap_for_tuple_return = param_is_output is not None and any(param_is_output)

  # Emit a warning when returnin a ref to an object:
  # this can probably lead to memory leaks or unwanted object copies (and thus undesired behaviour down the line)
  if not should_wrap_for_tuple_return and '&' in return_type and not (method_config.get('returns_ref_ok') or False):
    header.submodule.report.add_method_returning_ref(bound_object.cpp_no_template_name, method, method_signature)

  # Arguments that are inputs to the lambda function that wraps the ViSP function
  input_param_types = [params_strs[i] for i in range(len(param_is_input)) if param_is_input[i]]

  def to_argument_name(type: str, name: str) -> str:
    if '(*)' in type:
      return type.replace('(*)', f'(*{name})', 1)
    else:
      return type + ' ' + name


  params_with_names = [to_argument_name(t, name) for t, name in zip(input_param_types, input_param_names)]

  # Params that are only outputs: they should be declared in function. Assume that they are default constructible
  param_is_only_output = [not is_input and is_output for is_input, is_output in zip(param_is_input, param_is_output)]
  param_type_decl = [get_type_for_declaration(method.parameters[i].type, specs, header_env.mapping) for i in range(len(param_is_only_output))]
  param_decl_data = [(param_type_decl[i], param_names[i], get_default_assignment_str(param_type_decl[i])) for i in range(len(param_is_only_output)) if param_is_only_output[i]]
  param_declarations = [f'{decl_type} {name}{assignment};' for (decl_type, name, assignment) in param_decl_data]
  param_declarations = '\n'.join(param_declarations)

  if is_class_method and not method.static:
    self_param_with_name = bound_object.cpp_name + '& self'
    method_caller = 'self.'
  else:
    self_param_with_name = None
    method_caller = bound_object.cpp_name + '::' if is_class_method else bound_object.cpp_name
  output_param_symbols = []
  if return_type is None or return_type == 'void':
    maybe_get_return = ''
  else:
    maybe_get_return = f'{return_type} res = '
    if '&' in return_type:
      output_param_symbols.append('&res')
    else:
      output_param_symbols.append('res')

  if len(output_param_names) == 0 and (return_type is None or return_type == 'void'):
    return_str = ''
  elif len(output_param_names) == 0:
    return_str = 'res'
  elif len(output_param_names) == 1 and (return_type is None or return_type == 'void'):
    return_str = output_param_names[0]
  else:
    # When returning a tuple we need to explicitly convert references to pointer.
    # This is required since std::tuple will upcast the ref to its base class and try to store a copy of the object
    # If a class is pure virtual, this is not possible and will raise a compilation error!
    output_param_symbols.extend(['&' + name if is_ref else name for is_ref, name in zip(output_param_is_ref, output_param_names)])
    return_str = f'std::make_tuple({", ".join(output_param_symbols)})'

  lambda_body = f'''
    {param_declarations}
    {maybe_get_return}{method_caller}{method_name}({", ".join(param_names)});
    return {return_str};
  '''
  final_lambda_params = [self_param_with_name] + params_with_names if self_param_with_name is not None else params_with_names
  lambda_variant = define_lambda('', final_lambda_params, None, lambda_body)

  if should_wrap_for_tuple_return:
    method_body_str = lambda_variant
  elif is_class_method:
    method_body_str = ref_to_class_method(method, bound_object.cpp_name, method_name, return_type, params_strs)
  else:
    method_body_str = ref_to_function(bound_object.cpp_name + method_name, return_type, params_strs)

  method_str = method_def(py_method_name, method_body_str, pybind_options, method.static if is_class_method else False)
  method_str = f'{bound_object.python_ident}.{method_str};'
  return method_str, MethodData(py_method_name, method, lambda_variant, pybind_options)

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
  Deleted = 'deleted',
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
    (lambda m, _: m.deleted, NotGeneratedReason.Deleted),
    (lambda m, _: m.pure_virtual, NotGeneratedReason.PureVirtual),
    (lambda m, _: m.access is None or m.access != 'public', NotGeneratedReason.Access),
    (lambda m, _: m.destructor, NotGeneratedReason.Destructor),
    (lambda m, conf: m.template is not None and (conf.get('specializations') is None or len(conf['specializations']) == 0), NotGeneratedReason.UnspecifiedTemplateSpecialization),
    (lambda m, _: any(is_unsupported_argument_type(param.type) for param in m.parameters), NotGeneratedReason.ArgumentType),
    (lambda m, _: not m.constructor and is_unsupported_return_type(m.return_type), NotGeneratedReason.ReturnType)
  ]
  for method in methods:
    method_config = submodule.get_method_config(cls_name, method, {}, mapping)
    method_can_be_bound = True
    for predicate, motive in filtering_predicates_and_motives:
      if predicate(method, method_config):
        return_str = '' if method.return_type is None else (get_type(method.return_type, {}, mapping) or '<unparsed>')
        method_name = '::'.join(seg.name for seg in method.name.segments)
        param_strs = [get_type(param.type, {}, mapping) or '<unparsed>' for param in method.parameters]
        rejected_methods.append(RejectedMethod(cls_name, method, method_config, get_method_signature(method_name, return_str, param_strs), motive))
        method_can_be_bound = False
        break
    if method_can_be_bound:
      bindable_methods.append((method, method_config))

  return bindable_methods, rejected_methods

def get_bindable_functions_with_config(submodule: 'Submodule', functions: List[types.Function], mapping) -> Tuple[List[Tuple[types.Function, Dict]], List[RejectedMethod]]:
  bindable_functions = []
  rejected_functions = []
  # Order of predicates is important: The first predicate that matches will be the one shown in the log, and they do not all have the same importance
  filtering_predicates_and_motives = [
    (lambda _, conf: conf['ignore'], NotGeneratedReason.UserIgnored),
    (lambda m, _: GeneratorConfig.is_forbidden_function_name(get_name(m.name)), NotGeneratedReason.UserIgnored),
    (lambda m, conf: m.template is not None and (conf.get('specializations') is None or len(conf['specializations']) == 0), NotGeneratedReason.UnspecifiedTemplateSpecialization),
    (lambda m, _: any(is_unsupported_argument_type(param.type) for param in m.parameters), NotGeneratedReason.ArgumentType),
    (lambda m, _: is_unsupported_return_type(m.return_type), NotGeneratedReason.ReturnType)
  ]
  for function in functions:
    function_config = submodule.get_method_config(None, function, {}, mapping)
    method_can_be_bound = True
    for predicate, motive in filtering_predicates_and_motives:
      if predicate(function, function_config): # Function should be rejected
        return_str = '' if function.return_type is None else (get_type(function.return_type, {}, mapping) or '<unparsed>')
        method_name = get_name(function.name)
        param_strs = [get_type(param.type, {}, mapping) or '<unparsed>' for param in function.parameters]
        rejected_functions.append(RejectedMethod('', function, function_config, get_method_signature(method_name, return_str, param_strs), motive))
        method_can_be_bound = False
        break
    if method_can_be_bound:
      bindable_functions.append((function, function_config))

  return bindable_functions, rejected_functions

def split_methods_with_config(methods: List[Tuple[types.Method, Dict]], predicate: Callable[[types.Method], bool]) -> Tuple[List[Tuple[types.Method, Dict]], List[Tuple[types.Method, Dict]]]:
  matching = []
  non_matching = []
  for method, method_config in methods:
    if predicate(method):
      matching.append((method, method_config))
    else:
      non_matching.append((method, method_config))
  return matching, non_matching
