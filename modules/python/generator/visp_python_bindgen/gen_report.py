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

from typing import List, Dict
from pathlib import Path
import json

from cxxheaderparser import types

from visp_python_bindgen.utils import *
from visp_python_bindgen.methods import NotGeneratedReason, RejectedMethod

from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from visp_python_bindgen.submodule import Submodule

class Report(object):
  def __init__(self, submodule: 'Submodule'):
    self.submodule_name = submodule.name
    self.result = {
      'ignored_headers': [],
      'classes': {},
      'methods': {},
      'default_param_policy_methods': [],
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
        'ignore': True
      }
      report_dict = {
        'reason': method.rejection_reason.value,
        'fix': proposed_help
      }
      if method.cls_name is not None:
        report_dict['class'] = method.cls_name
      self.result['methods'][method.signature] = report_dict

  def add_default_policy_method(self, cls_name: str, method: types.Method, signature: str, input_params: List[bool], output_params: List[bool]) -> None:
    proposed_help = [
      {
        'static': method.static,
        'signature': signature,
        'use_default_param_policy': True
      },
      {
        'static': method.static,
        'signature': signature,
        'use_default_param_policy': False,
        'param_is_input': input_params,
        'param_is_output': output_params
      }
    ]
    report_dict = {
      'reason': 'Method uses default parameter policy, make sure that this is correct! If it is use "use_default_param_policy": true. Otherwise, set the custom inputness/outputness of params',
      'signature': signature,
      'static': method.static,
      'class': cls_name,
      'possible_fixes': proposed_help
    }
    self.result['default_param_policy_methods'].append(report_dict)

  def write(self, path: Path) -> None:
    print(f'Statistics for module {self.submodule_name} report:')
    stats = [
      f'Ignored headers: {len(self.result["ignored_headers"])}',
      f'Ignored classes: {len(self.result["classes"].keys())}',
      f'Not generated methods: {len(self.result["methods"].keys())}',
      f'Method with default parameter policy: {len(self.result["default_param_policy_methods"])}',
    ]
    print('\n\t'.join(stats))
    with open(path, 'w') as report_file:
      json.dump(self.result, report_file, indent=2)
