# BSD 3-Clause License
#
# Copyright (c) 2018, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

from typing import Dict, List, Tuple, Any, Mapping

import pkg_resources
import warnings
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__bootstrap(Module):
    """Module for library bag_vco_adc cell bootstrap.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'bootstrap.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            lch='channel length of transistors',
            intent='device intent',
            dev_info='devices information including nf, w and stack',
            dum_info='dummy information including nf, w and stack',
            cap_params='capacitor parameters',
            cap_aux_params='capacitor parameters',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            dum_info=[],
            cap_aux_params=None,
            cap_params=None,
        )

    def design(self, lch: int, intent: str, dev_info: Mapping[str, Any], dum_info: List[Tuple[Any]],
               cap_params: Mapping[str, Any], cap_aux_params: Mapping[str, Any]) -> None:
        if 'XSAMPLE_DUM' not in dev_info.keys():
            warnings.warn("Doesn't implement dummhy samlping sw")
            self.remove_pin('in_c')
            self.delete_instance('XSAMPLE_DUM')

        if 'XCAP_P_AUX' not in dev_info.keys():
            self.remove_pin('cap_top_aux')
            self.delete_instance('XCAP_P_AUX')
            self.reconnect_instance_terminal('XCAP_P', 'B', 'cap_top')
            self.reconnect_instance_terminal('XON_P', 'B', 'cap_top')
            self.reconnect_instance_terminal('XINV_P', 'B', 'cap_top')
        for key, _info in dev_info.items():
            _w = _info.get('w', 4)
            _stack = _info.get('stack', 1)
            self.instances[key].design(l=lch, intent=intent, w=_w, nf=_info['nf'], stack=_stack)
        if dum_info:
            self.design_dummy_transistors(dum_info, 'X_DUMMY', 'VDD', 'VSS')
        else:
            self.delete_instance('X_DUMMY')

        if cap_params is None:
            self.delete_instance('X_CBOOT')
        else:
            self.instances['X_CBOOT'].design(**cap_params)
        if cap_aux_params is None:
            self.delete_instance('X_CAUX')
        else:
            self.instances['X_CAUX'].design(**cap_params)
