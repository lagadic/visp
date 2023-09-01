from typing import Callable, List, Optional, Set, Tuple, Dict, Union
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

def enum_bindings(root_scope: NamespaceScope, mapping: Dict) -> List[Tuple[str, str]]:
  def accumulate_data(scope: Union[NamespaceScope, ClassScope], data: Dict):
    for cls in scope.classes:
      accumulate_data(cls, data)
    if isinstance(scope, NamespaceScope):
      for namespace in scope.namespaces:
        accumulate_data(namespace, data)
  enum_data = {} # Need to go through an intermediate rep, as some enum are typedefed and others are not
  for typedef in root_scope.typedefs:
