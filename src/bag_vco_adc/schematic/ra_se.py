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
from typing import Mapping, Any, Dict

import pkg_resources

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__ra_se(Module):
    """Module for library bag_vco_adc cell ra_se.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'ra_se.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            lch='channel length',
            seg_dict='transistor segments dictionary.',
            w_dict='transistor width dictionary.',
            th_dict='transistor threshold dictionary.',
            stack_dict='transistor stack dictionary',
            dac_params='DAC parameters'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def design(self, lch: int, seg_dict: Mapping[str, int], w_dict: Mapping[str, int], th_dict: Mapping[str, str],
               stack_dict: Mapping[str, int], dac_params) -> None:
        for name in ['in', 'mid', 'out', 'dzd', 'be', 'bias', 'tail']:
            uname = name.upper()
            pname = f'p{name}'
            nname = f'n{name}'
            if nname in seg_dict and pname in seg_dict:
                if 'out' in name:
                    self.instances[f'X{uname}_P'].design(lch=lch, w=w_dict[pname], seg=seg_dict[pname],
                                                         intent=th_dict[pname], stack=stack_dict[pname])
                    self.instances[f'X{uname}_N'].design(lch=lch, w=w_dict[nname], seg=seg_dict[nname],
                                                         intent=th_dict[nname], stack=stack_dict[nname])
                else:
                    self.instances[f'X{uname}_P'].design(l=lch, w=w_dict[pname],
                                                         nf=seg_dict[pname], intent=th_dict[pname])
                    self.instances[f'X{uname}_N'].design(l=lch, w=w_dict[nname],
                                                         nf=seg_dict[nname], intent=th_dict[nname])
            else:
                self.remove_instance(f'X{uname}_P')
                self.remove_instance(f'X{uname}_N')

        if 'nbe' not in seg_dict and 'pbe' not in seg_dict:
            self.reconnect_instance_terminal('XIN_N', 'D', 'mid')
            self.reconnect_instance_terminal('XIN_P', 'D', 'mid')
            self.reconnect_instance_terminal('XMID_N', 'G', 'mid')
            self.reconnect_instance_terminal('XMID_P', 'G', 'mid')

        if 'nfb' in seg_dict:
            self.instances['XFB_N'].design(l=lch, w=w_dict['nfb'], nf=seg_dict['nfb'], intent=th_dict['nfb'])
        if 'pfb' in seg_dict:
            self.instances['XFB_P'].design(l=lch, w=w_dict['pfb'], nf=seg_dict['pfb'], intent=th_dict['pfb'])

        if 'pen' in seg_dict and 'nen' in seg_dict:
            for idx in range(2):
                self.instances[f'XEN_P{idx}'].design(l=lch, w=w_dict['pen'], nf=seg_dict['pen'], intent=th_dict['pen'])
                self.instances[f'XEN_N{idx}'].design(l=lch, w=w_dict['nen'], nf=seg_dict['nen'], intent=th_dict['nen'])

        if seg_dict['pcharge']:
            self.instances[f'XCHARGE_P'].design(l=lch, w=w_dict['pcharge'], nf=seg_dict['pcharge'],
                                                intent=th_dict['pcharge'])
        else:
            self.remove_instance('XCHARGE_P')
        if seg_dict['ncharge']:
            self.instances[f'XCHARGE_N'].design(l=lch, w=w_dict['ncharge'], nf=seg_dict['ncharge'],
                                                intent=th_dict['ncharge'])
        else:
            self.remove_instance('XCHARGE_N')

        num_dacn = len(dac_params['seg_dacn_list'])
        num_dacp = len(dac_params['seg_dacp_list'])
        inst_term_dacp_list, inst_term_dacp_en_list = [], []
        for idx in range(num_dacp):
            seg = dac_params['seg_dacp_list'][idx]
            mid_prefix = f'<{seg - 1}:0>' if seg > 1 else '<0>'
            inst_name_en = f'XFB_P_DAC_EN{idx}' + mid_prefix
            inst_name_dac = f'XFB_P_DAC{idx}' + mid_prefix
            en_term = [('S', 'VDD'), ('D', f'vtop_dac_mid{idx}' + mid_prefix), ('G', f'ctrl_dac_p<{idx}>'),
                       ('B', 'VDD')]
            dac_term = [('D', 'vtop'), ('S', f'vtop_dac_mid{idx}' + mid_prefix), ('G', 'midp'), ('B', 'VDD')]
            inst_term_dacp_list.append((inst_name_dac, dac_term))
            inst_term_dacp_en_list.append((inst_name_en, en_term))
        self.array_instance('XFB_P_DAC_EN', inst_term_list=inst_term_dacp_en_list)
        self.array_instance('XFB_P_DAC', inst_term_list=inst_term_dacp_list)
        for idx in range(num_dacp):
            seg = dac_params['seg_dacp_list'][idx]
            mid_prefix = f'<{seg - 1}:0>' if seg > 1 else '<0>'
            self.instances[f'XFB_P_DAC_EN{idx}' + mid_prefix].design(l=lch, w=dac_params['w_dacp_list'][idx],
                                                                     nf=1, intent=dac_params['th_dacp'])
            self.instances[f'XFB_P_DAC{idx}' + mid_prefix].design(l=lch, w=dac_params['w_dacp_list'][idx],
                                                                  nf=1, intent=dac_params['th_dacp'])

        inst_term_dacn_list, inst_term_dacn_en_list = [], []
        for idx in range(num_dacn):
            seg = dac_params['seg_dacn_list'][idx]
            mid_prefix = f'<{seg - 1}:0>' if seg > 1 else '<0>'
            inst_name_en = f'XFB_N_DAC_EN{idx}'+ mid_prefix
            inst_name_dac = f'XFB_N_DAC{idx}'+ mid_prefix
            en_term = [('S', 'VSS'), ('D', f'vbot_dac_mid{idx}' + mid_prefix), ('G', f'ctrl_dac_n<{idx}>'),
                       ('B', 'VSS')]
            dac_term = [('D', 'vbot'), ('S', f'vbot_dac_mid{idx}' + mid_prefix), ('G', 'midn'), ('B', 'VSS')]
            inst_term_dacn_list.append((inst_name_dac, dac_term))
            inst_term_dacn_en_list.append((inst_name_en, en_term))
        self.array_instance('XFB_N_DAC_EN', inst_term_list=inst_term_dacn_en_list)
        self.array_instance('XFB_N_DAC', inst_term_list=inst_term_dacn_list)
        for idx in range(num_dacp):
            seg = dac_params['seg_dacp_list'][idx]
            mid_prefix = f'<{seg - 1}:0>' if seg > 1 else '<0>'
            self.instances[f'XFB_N_DAC_EN{idx}' + mid_prefix].design(l=lch, w=dac_params['w_dacn_list'][idx],
                                                                     nf=1, intent=dac_params['th_dacn'])
            self.instances[f'XFB_N_DAC{idx}' + mid_prefix].design(l=lch, w=dac_params['w_dacn_list'][idx],
                                                                  nf=1, intent=dac_params['th_dacn'])
        self.rename_pin('ctrl_dac_p', f'ctrl_dac_p<{num_dacp - 1}:0>')
        self.rename_pin('ctrl_dac_n', f'ctrl_dac_n<{num_dacn - 1}:0>')
