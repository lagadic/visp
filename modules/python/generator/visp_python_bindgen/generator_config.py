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
from typing import Dict, Final, List, Optional
import re
from pathlib import Path
from dataclasses import dataclass
import json

@dataclass
class ModuleInputData(object):
  name: str
  headers: List[Path]
  dependencies: List[str]

@dataclass
class PreprocessorConfig(object):
  '''
  Preprocessor config. Contains the arguments that are passed to pcpp to preprocess a header.
  This does not include the header search path (-I) or the input and output paths
  '''

  defines: Dict[str, str] # Mapping from a #define to its value (#define A 1 is equal to a pair "A": "1")
  never_defined: List[str] # List of macros that should never be defined, even if they are defined in other included headers
  include_directories: List[str]
  passthrough_includes_regex: str # Regex to see which header should be included (expanded and put in the resulting pcpp output) or ignored
  line_directive: Optional[str] # prefix for Warning/logging emitted by pcpp. If none, no warning
  other_args: List[str]

  def to_pcpp_args_list(self) -> List[str]:
    args = []
    for k,v in self.defines.items():
      args += ['-D', f'{k}={v}'] if v is not None else ['-D', k]
    for v in self.never_defined:
      args += ['-N', v]
    for v in self.include_directories:
      args += ['-I', v]
    args += self.other_args
    args.extend(['--passthru-includes', self.passthrough_includes_regex])
    if self.line_directive is not None:
      args.extend(['--line-directive', self.line_directive])
    else:
      args.extend(['--line-directive', ''])
    return args

'''
Regular expressions that should match with types that are considered as immutable on the Python side
This only encompasses raw types
'''
IMMUTABLE_TYPES_REGEXS = [
  '^(float|double|u?int\d+_t|int|unsigned|unsigned int|size_t|ssize_t|char|long|long\wlong|bool)$',
  '^std::string$'
]

'''
Regular expressions that should match with types that are considered as immutable on the Python side
This only encompasses raw types
'''
IMMUTABLE_CONTAINERS_REGEXS = [
  '^std::vector', '^std::list'
]


'''
Specific argument regexs for which having default arguments is specifically forbidden
'''
FORBIDDEN_DEFAULT_ARGUMENT_TYPES_REGEXS = [
  '^std::function',
  '^std::ostream',
  '^std::initializer_list',
  '^rs2::',
  '^cv::'
]

'''
Regexes for names of functions that should be ignored
'''
FORBIDDEN_FUNCTION_NAMES_REGEXS = [
  '^(from|to)_.*json',
  '^operator.*',
  '^ignored$'
]
class GeneratorConfig(object):
  pcpp_config: Final[PreprocessorConfig] = PreprocessorConfig(
    defines={
      'VISP_EXPORT': '', # remove symbol as it messes up the cxxheaderparsing
      'VP_DEPRECATED': '', # remove symbol as it messes up the cxxheaderparsing
      'DOXYGEN_SHOULD_SKIP_THIS': None, # Do not generate methods that do not appear in public api doc
      'NLOHMANN_JSON_SERIALIZE_ENUM(a,...)': 'void ignored() {}', # Remove json enum serialization as it cnanot correctly be parsed
      'CV_OUT': '', # In vpKeyPoint, this appears and breaks parsing
    },
    never_defined=[
      'VISP_BUILD_DEPRECATED_FUNCTIONS', # Do not bind deprecated functions
      'VISP_RUBIK_REGULAR_FONT_RESOURCES'
    ],
    include_directories=[], # Populated through the main configuration file
    passthrough_includes_regex="^.*$", # Never output the result of other includes.
    line_directive=None,
    other_args=['--passthru-unfound-includes'] #"--passthru-comments"
  )

  xml_doc_path: Optional[Path] = None

  module_data: List[ModuleInputData] = []

  @staticmethod
  def _matches_regex_in_list(s: str, regexes: List[str]) -> bool:
    return any(map(lambda regex: re.match(regex, s) is not None, regexes))

  @staticmethod
  def is_immutable_type(type: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(type, IMMUTABLE_TYPES_REGEXS)

  @staticmethod
  def is_immutable_container(type: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(type, IMMUTABLE_CONTAINERS_REGEXS)

  @staticmethod
  def is_forbidden_default_argument_type(type: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(type, FORBIDDEN_DEFAULT_ARGUMENT_TYPES_REGEXS)

  @staticmethod
  def is_forbidden_function_name(name: str) -> bool:
    return GeneratorConfig._matches_regex_in_list(name, FORBIDDEN_FUNCTION_NAMES_REGEXS)

  @staticmethod
  def update_from_main_config_file(path: Path) -> None:
    assert path.exists(), f'Main config file {path} was not found'
    with open(path, 'r') as main_config_file:
      main_config = json.load(main_config_file)
      print(json.dumps(main_config, indent=2))
      logging.info('Updating the generator config from dict: ', main_config)
      GeneratorConfig.pcpp_config.include_directories = main_config['include_dirs']

      defines = main_config.get('defines')
      if defines is not None:
        for define_key in defines:
          GeneratorConfig.pcpp_config.defines[define_key] = defines[define_key]

      xml_doc_path = main_config.get('xml_doc_path')
      if xml_doc_path is not None:
        GeneratorConfig.xml_doc_path = Path(xml_doc_path)

      modules_dict = main_config.get('modules')
      source_dir = Path(main_config.get('source_dir'))
      for module_name in modules_dict:
        headers = map(lambda s: Path(s), modules_dict[module_name].get('headers'))
        deps = modules_dict[module_name].get('dependencies')

        # Include only headers that are in the VISP source directory
        # Fix: Check specifically in the modules directory,
        # since the build directory (containing vpConfig.h, that we want to ignore)
        # Can be in the src folder
        headers = list(filter(lambda h: (source_dir / 'modules') in h.parents, headers))

        headers_log_str = '\n\t'.join([str(header) for header in headers])
        logging.info(f'Module {module_name} headers: \n\t{headers_log_str}')
        GeneratorConfig.module_data.append(ModuleInputData(module_name, headers, deps))
