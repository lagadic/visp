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
if TYPE_CHECKING:
  from submodule import Submodule

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

def define_method(py_name: str, method: str, additional_args: List[str], static: bool) -> str:
  def_type = 'def' if not static else 'def_static'
  additional_args_str = ', '.join(additional_args)
  if len(additional_args) > 0:
    additional_args_str = ', ' + additional_args_str
  return f'{def_type}("{py_name}", {method}{additional_args_str})'

def define_constructor(params: List[str], additional_args: List[str]) -> str:
  additional_args_str = ', '.join(additional_args)
  if len(additional_args) > 0:
    additional_args_str = ', ' + additional_args_str
  return f'def(py::init<{", ".join(params)}>(){additional_args_str})'


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
