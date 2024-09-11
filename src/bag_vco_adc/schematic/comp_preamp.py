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

from typing import Dict, Any, Mapping

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__comp_preamp(Module):
    """Module for library bag_vco_adc cell comp_preamp.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__, str(Path('netlist_info', 'comp_preamp.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            lch='channel length',
            seg_dict='transistor segments dictionary.',
            w_dict='transistor width dictionary.',
            th_dict='transistor threshold dictionary.',
            has_ofst='True to add bridge switch.',
            has_cas='True to add cascode tx to isolate input',
            flip_np='Flip NMOS and PMOS',
            equalizer='True to add equalizer'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(has_ofst=False, has_cas=True, flip_np=False, equalizer=False)

    def design(self, lch: int, seg_dict: Mapping[str, int], w_dict: Mapping[str, int], th_dict: Mapping[str, str],
               has_ofst: bool, has_cas: bool, flip_np: bool, equalizer: bool) -> None:

        if flip_np:
            self.replace_instance_master(f'XTAIL', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XINP', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XINN', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XOSP', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XOSN', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XCASP', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XCASN', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XLOADP', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XLOADN', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            for inst_name in ['XINP', 'XINN', 'XOSP', 'XOSN', 'XCASP', 'XCASN', 'XTAIL']:
                self.reconnect_instance(inst_name, [('B', 'VDD')])
            for inst_name in ['XLOADP', 'XLOADN']:
                self.reconnect_instance(inst_name, [('B', 'VSS')])
            self.reconnect_instance('XTAIL', [('S', 'VDD')])
            self.reconnect_instance('XLOADP', [('S', 'VSS')])
            self.reconnect_instance('XLOADN', [('S', 'VSS')])

        if has_ofst:
            w = w_dict['os']
            nf = seg_dict['os']
            intent = th_dict['os']
            self.instances['XOSP'].design(l=lch, w=w, nf=nf, intent=intent)
            self.instances['XOSN'].design(l=lch, w=w, nf=nf, intent=intent)
        else:
            self.delete_instance('XOSP')
            self.delete_instance('XOSN')
            self.remove_pin('osp')
            self.remove_pin('osn')
        self.remove_pin('midp')
        self.remove_pin('midn')

        dev_list = ['in', 'tail', 'cas', 'load'] if has_cas else ['in', 'tail', 'load']

        if equalizer:
            dev_list.append('eq')

        for name in dev_list:
            uname = name.upper()
            w = w_dict[name]
            nf = seg_dict[name]
            intent = th_dict[name]
            if 'tail' in name or 'eq' in name:
                self.instances[f'X{uname}'].design(l=lch, w=w, nf=nf, intent=intent)
            else:
                self.instances[f'X{uname}P'].design(l=lch, w=w, nf=nf, intent=intent)
                self.instances[f'X{uname}N'].design(l=lch, w=w, nf=nf, intent=intent)

        if not has_cas:
            self.remove_instance('XCASN')
            self.remove_instance('XCASP')
            self.reconnect_instance_terminal('XINN', 'D', 'outp')
            self.reconnect_instance_terminal('XINP', 'D', 'outn')
            if has_ofst:
                self.reconnect_instance('XOSP', [('D', 'outn')])
                self.reconnect_instance('XOSN', [('D', 'outp')])

        if 'eq' not in dev_list:
            self.remove_instance('XEQ')
