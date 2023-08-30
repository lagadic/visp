from typing import List, Optional, Set, Tuple, Dict, Union
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

def define_method(py_name: str, method: str, additional_args: List[str]) -> str:
  pass

def method_reference() -> str:
  pass

def get_operators(methods: List[types.Method]) -> Tuple[List[types.Method], Tuple[List[types.Method]]]:
  pass
