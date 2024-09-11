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

from pathlib import Path
from typing import Dict, Any, List

import pkg_resources

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__cap_drv(Module):
    """Module for library bag_vco_adc cell cap_drv.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'cap_drv.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            w='width of switch',
            lch='channel length of switch',
            seg='segment of switch',
            intent='intent of switch',
            cm_sw='True to add only one common mode switch for common mode cap',
            gated_list='Gated index',
            sw_type_dict='Dict of switch type, keys are XN XM XP',
            ref_dict='Dict of reference and control signals'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            sw_type_dict=dict(
                XN='n',
                XM='n',
                XP='n',
            ),
            cm_sw=False,
            # gated_list=[],
            ref_dict=dict(
                XM={'ref': 'vref<1>', 'ctrl': 'ctrl<1>'},
                XN={'ref': 'vref<0>', 'ctrl': 'ctrl<0>'},
                XP={'ref': 'vref<2>', 'ctrl': 'ctrl<2>'},
            )
        )

    def design(self, sw_type_dict: Dict[str, str], seg: int, lch: int, w: int, intent: str, cm_sw: bool,
               gated_list: List[str], ref_dict: Dict[str, Dict]) -> None:
        en_sig_list = []
        sw_keys = [key for key, type in sw_type_dict.items()]
        for k in ['XN', 'XM', 'XP']:
            if k not in sw_keys:
                self.remove_instance(k)

        if not cm_sw:
            for k in sw_keys:
                if sw_type_dict[k] != 'n':
                    self.replace_instance_master(k, 'xbase', 'pmos4_stack', keep_connections=True)
                    self.reconnect_instance_terminal(k, 'b', 'VDD')
                self.instances[k].design(lch=lch, w=w, intent=intent, seg=seg, stack=1 + int(k in gated_list))
                if k in gated_list:
                    en_name = 'en' if sw_type_dict == 'p' else 'enb'
                    en_sig_list.append(en_name)
                    self.reconnect_instance(k, [('g<1:0>', f'{en_name},' + ref_dict[k]['ctrl']),
                                                ('s', ref_dict[k]['ref'])])
                else:
                    self.reconnect_instance(k, [('g', ref_dict[k]['ctrl']), ('s', ref_dict[k]['ref'])])
            num_ref = len(sw_keys)
            if num_ref<3:
                self.rename_pin('vref<2:0>', f'vref<{num_ref-1}:0>')
                self.rename_pin('ctrl<2:0>', f'ctrl<{num_ref-1}:0>')
        else:
            self.instances['XM'].design(lch=lch, w=w, intent=intent, seg=seg, stack=1)
            self.reconnect_instance_terminal('XM', 'g', 'ctrl')
            self.reconnect_instance_terminal('XM', 's', 'vref_cm')
            self.rename_pin('vref<2:0>', 'vref_cm')
            self.rename_pin('ctrl<2:0>', 'ctrl')
            self.remove_instance('XN')
            self.remove_instance('XP')

        if 'en' not in en_sig_list:
            self.remove_pin('en')
        if 'enb' not in en_sig_list:
            self.remove_pin('enb')
