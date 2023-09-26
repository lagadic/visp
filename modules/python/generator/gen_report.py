from typing import Callable, List, Optional, Set, Tuple, Dict, Union
from cxxheaderparser.parserstate import ClassBlockState, State
import pcpp
import cxxheaderparser
from cxxheaderparser import types
from cxxheaderparser.simple import NamespaceScope, ClassScope
from utils import *
from methods import NotGeneratedReason, RejectedMethod
class Report(object):
  def __init__(self, submodule: 'Submodule'):
    self.submodule_name = submodule.name
    self.result = {
      'ignored_headers': [],
      'classes': {},
      'methods': {}
    }

  def add_ignored_header(self, path: Path) -> None:
    self.result['ignored_headers'].append(str(path))

  def add_non_generated_class(self, cls_name: str, config: Dict, reason: str) -> None:
    self.result['classes'][cls_name] = {
      'reason': reason
    }

  def add_non_generated_method(self, method: RejectedMethod) -> None:
    if NotGeneratedReason.is_non_trivial_reason(method.rejection_reason):
      proposed_help = {
        'static': method.method.static,
        'signature': method.signature,
        'ignored': True
      }
      report_dict = {
        'reason': method.rejection_reason.value,
        'fix': proposed_help
      }
      if method.cls_name is not None:
        report_dict['class'] = method.cls_name
      self.result['methods'][method.signature] = report_dict
  def write(self, path: Path) -> None:
    import pprint; pprint.pprint(self.result)
    with open(path, 'w') as report_file:
      json.dump(self.result, report_file, indent=2)
