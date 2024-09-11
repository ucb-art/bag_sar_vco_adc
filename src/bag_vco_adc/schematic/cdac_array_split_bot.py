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
from pybag.enum import TermType


# noinspection PyPep8Naming
class bag_vco_adc__cdac_array_split_bot(Module):
    """Module for library bag_vco_adc cell cdac_array_split_bot.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'cdac_array_split_bot.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)
        self._has_bot = False

    def export_bot(self):
        return self._has_bot

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            sw_params_list='multiplier list of sw unit',
            unit_params_list='Parameters of unit capacitor + drv',
            dum_params='Dummy parameters',
            sampler_params='sampler params',
            bot_probe='True to export cap unit bottom plate',
            cm='Number of common-mode cap',
            sw_m_list='Number of switches',
            cap_m_list='Number of capacitor',
            remove_cap='True to remove capacitor, use it when doesnt have rmetal',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            bot_probe=True,
            remove_cap=False,
        )

    def design(self, cm: int, sw_params_list: List[Param], dum_params: Param,
               sampler_params: Param, unit_params_list: List[Param], sw_m_list: List[int], cap_m_list: List[int],
               bot_probe: bool, remove_cap: bool) -> None:
        remove_cap = self.params['remove_cap']
        nbits = len(unit_params_list)
        # check length of switch params and cap params list:
        if nbits != len(sw_params_list):
            raise ValueError("[CDAC Array Schematic]: switch and cap params length don't match")
        capp_term_list = []
        capn_term_list = []
        swn_term_list = []
        swp_term_list = []

        # Remove pins first to avoid bus and scalar name conflict
        self.rename_pin('vref', 'vref<1:0>')
        for pname in ['ctrl_n', 'ctrl_p']:
            self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")
        # self.add_pin('bot_cm', TermType.inout)

        # List inst name and term connection
        sampler_out = []
        bot_idx = 0
        for idx, (sw_m, cap_m) in enumerate(zip(sw_m_list, cap_m_list)):
            _termn = [('VDD', 'VDD'), ('VSS', 'VSS'), ('vref<1:0>', 'vref<1:0>'),
                      ('out', f'botn<{bot_idx + cap_m - 1}:{bot_idx}>' if cap_m > 1 else f'botn<{bot_idx}>'),
                      ('ctrl', f"ctrl_n<{idx}>")]
            _termp = [('VDD', 'VDD'), ('VSS', 'VSS'), ('vref<1:0>', 'vref<1:0>'),
                      ('out', f'botp<{bot_idx + cap_m - 1}:{bot_idx}>' if cap_m > 1 else f'botp<{bot_idx}>'),
                      ('ctrl', f"ctrl_p<{idx}>")]
            _capp_name = f'XCAPP{idx}<{cap_m - 1}:0>' if cap_m > 1 else f'XCAPP{idx}'
            _capn_name = f'XCAPN{idx}<{cap_m - 1}:0>' if cap_m > 1 else f'XCAPN{idx}'
            _swp_name = f'XDRVP{idx}<{sw_m - 1}:0>' if sw_m > 1 else f'XDRVP{idx}'
            _swn_name = f'XDRVN{idx}<{sw_m - 1}:0>' if sw_m > 1 else f'XDRVN{idx}'
            capp_term_list.append((_capp_name, [('top', 'top'), (
                'bot', f'botp<{bot_idx + cap_m - 1}:{bot_idx}>' if cap_m > 1 else f'botp<{bot_idx}>')]))
            capn_term_list.append((_capn_name, [('top', 'top'), (
                'bot', f'botn<{bot_idx + cap_m - 1}:{bot_idx}>' if cap_m > 1 else f'botn<{bot_idx}>')]))
            swp_term_list.append((_swp_name, _termp))
            swn_term_list.append((_swn_name, _termn))
            sampler_out.append(f'botn<{bot_idx + cap_m - 1}:{bot_idx}>' if cap_m > 1 else f'botn<{bot_idx}>')
            sampler_out.append(f'botp<{bot_idx + cap_m - 1}:{bot_idx}>' if cap_m > 1 else f'botp<{bot_idx}>')
            bot_idx += cap_m

        # Design sar_sch array
        dx_max = 2 * max(self.instances['XCAPN'].width, self.instances['XCAPN'].width)
        self.array_instance('XCAPN', inst_term_list=capn_term_list, dx=dx_max)
        self.array_instance('XCAPP', inst_term_list=capp_term_list, dx=dx_max)
        self.array_instance('XDRVN', inst_term_list=swn_term_list, dx=dx_max)
        self.array_instance('XDRVP', inst_term_list=swp_term_list, dx=dx_max)
        for idx, (name, _) in enumerate(capn_term_list):
            self.instances[name].design(**unit_params_list[idx])
            if remove_cap:
                self.remove_instance(name)
        for idx, (name, _) in enumerate(capp_term_list):
            self.instances[name].design(**unit_params_list[idx])
            if remove_cap:
                self.remove_instance(name)
        for idx, (name, _) in enumerate(swn_term_list):
            sw_params = sw_params_list[idx].copy(append=dict(ndrv=True))
            self.instances[name].design(**sw_params)
            if 'enb' in self.instances[name].master.pins.keys():
                self.reconnect_instance(name, [('enb', 'samb')])
            if 'en' in self.instances[name].master.pins.keys():
                self.reconnect_instance(name, [('en', 'sam')])
        for idx, (name, _) in enumerate(swp_term_list):
            self.instances[name].design(**sw_params_list[idx])
            if 'enb' in self.instances[name].master.pins.keys():
                self.reconnect_instance(name, [('enb', 'samb')])
            if 'en' in self.instances[name].master.pins.keys():
                self.reconnect_instance(name, [('en', 'sam')])

        self.instances['XDUM'].design(**dum_params)
        self.rename_instance('XDUM', 'XDUM<1:0>', [('top', 'top'), ('bot', 'VSS')])
        self.instances['XSAM'].design(**sampler_params)
        sampler_conns = []
        for sampler_pin in self.instances['XSAM'].master.pins.keys():
            if 'out' in sampler_pin:
                sampler_conns.append([sampler_pin, ','.join(sampler_out)])
        sampler_conns.extend([('sam', 'vg'), ('off', 'voff')])

        self.reconnect_instance('XSAM', sampler_conns)
        [self.remove_pin(pinname) for pinname in ['bot', 'ctrl_s', 'ctrl_m']]
        for pin in [f'botn<{bot_idx - 1}:0>', f'botp<{bot_idx - 1}:0>']:
            self.add_pin(pin, TermType.inout)
