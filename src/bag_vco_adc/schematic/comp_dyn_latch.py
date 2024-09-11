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
from bag3_liberty.enum import TermType


class bag_vco_adc__comp_dyn_latch(Module):
    """Module for library bag_vco_adc cell comp_dyn_latch.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'comp_dyn_latch.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            lch='channel length',
            seg_dict='transistor segments dictionary.',
            w_dict='transistor width dictionary.',
            th_dict='transistor threshold dictionary.',
            has_rst='True to add reset dev',
            flip_np='Flip NMOS and PMOS',
            split_ck='Separate p/n clk and tail',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            has_rst=False,
            flip_np=False,
            split_ck=False,
        )

    def design(self, lch: int, seg_dict: Mapping[str, int], w_dict: Mapping[str, int], th_dict: Mapping[str, str],
               has_rst: bool, flip_np: bool, split_ck: bool) -> None:
        if flip_np:
            self.replace_instance_master(f'XINP', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XINN', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XINP_M', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XINN_M', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XPFBP', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XPFBN', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XNFBP', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XNFBN', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XTAILP', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master(f'XTAILN', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.rename_instance('XNFBN', 'XPFBNt')
            self.rename_instance('XNFBP', 'XPFBPt')
            self.rename_instance('XPFBN', 'XNFBN')
            self.rename_instance('XPFBP', 'XNFBP')
            self.rename_instance('XPFBNt', 'XPFBN')
            self.rename_instance('XPFBPt', 'XPFBP')

            for inst_name in ['XINP', 'XINN', 'XINP_M', 'XINN_M', 'XNFBN', 'XNFBP']:
                self.reconnect_instance(inst_name, [('B', 'VSS'), ('S', 'VSS')])
            for inst_name in ['XPFBP', 'XPFBN', 'XTAILP', 'XTAILN']:
                self.reconnect_instance(inst_name, [('B', 'VDD')])
            self.reconnect_instance('XTAILN', [('S', 'VDD')])
            self.reconnect_instance('XTAILP', [('S', 'VDD')])

        for name in ['in', 'tail', 'nfb', 'pfb']:
            uname = name.upper()
            w = w_dict[name]
            nf = seg_dict[name]
            intent = th_dict[name]
            # if 'tail' in name:
            #     nf = nf//2
            self.instances[f'X{uname}P'].design(l=lch, w=w, nf=nf, intent=intent)
            self.instances[f'X{uname}N'].design(l=lch, w=w, nf=nf, intent=intent)
        if has_rst:
            self.instances['XINP_M'].design(l=lch, w=w_dict['rst'], nf=seg_dict['rst'], intent=th_dict['rst'])
            self.instances['XINN_M'].design(l=lch, w=w_dict['rst'], nf=seg_dict['rst'], intent=th_dict['rst'])
            self.reconnect_instance('XINP_M', [('G', 'inn_m'), ('D', 'tailp')])
            self.reconnect_instance('XINN_M', [('G', 'inp_m'), ('D', 'tailn')])
        else:
            self.delete_instance('XINP_M')
            self.delete_instance('XINN_M')
            self.remove_pin('inn_m')
            self.remove_pin('inp_m')
            self.reconnect_instance('XTAILN', [('D', 'tail')])
            self.reconnect_instance('XTAILP', [('D', 'tail')])
            self.reconnect_instance('XPFBN' if flip_np else 'XNFBN', [('S', 'tail')])
            self.reconnect_instance('XPFBP' if flip_np else 'XNFBP', [('S', 'tail')])

        if split_ck:
            self.reconnect_instance('XTAILN', [('G', 'clkp'), ('D', 'tailn')])
            self.reconnect_instance('XTAILP', [('G', 'clkn'), ('D', 'tailp')])
            self.reconnect_instance('XPFBP' if flip_np else 'XNFBP', [('S', 'tailp')])
            self.reconnect_instance('XPFBN' if flip_np else 'XNFBN', [('S', 'tailn')])
            self.rename_pin('clk', 'clkn')
            self.add_pin('clkp', TermType.input)
