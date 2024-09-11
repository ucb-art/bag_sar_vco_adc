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

from typing import Dict, Any, List

import pkg_resources
from pathlib import Path

from pybag.enum import TermType

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__cdac_array(Module):
    """Module for library bag_vco_adc cell cdac_array.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'cdac_array.yaml')))

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
            cm='Number of unit common-mode capacitor',
            m_list='multiplier list of cap unit',
            sw_list='multiplier list of sw unit',
            unit_params='Parameters of unit capacitor + drv',
            bot_probe='True to export cap unit bottom plate',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            cm=1,
            bot_probe=False,
            sw_list=[]
        )

    def design(self, cm: int, m_list: List[int], sw_list: List[int], unit_params: Param, bot_probe: bool) -> None:
        nbits = len(m_list)
        inst_term_list = []
        unit_params_list = []

        # Remove pins first to avoid bus and scalar name conflict
        self.rename_pin('vref', 'vref<2:0>')
        for pname in ['ctrl_n', 'ctrl_p', 'ctrl_m']:
            self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")

        if bot_probe:
            self.add_pin(f"bot<{nbits-1}:0>", TermType.inout)
            self._has_bot = True

        # List inst name and term connection
        for idx, m in enumerate(m_list):
            _name = f"XB{idx}"
            _term = [('VDD', 'VDD'), ('VSS', 'VSS'), ('vref<2:0>', 'vref<2:0>'),
                     ('bot', f"bot<{idx}>"), ('top', 'top'),
                     ('ctrl<2:0>', f"ctrl_n<{idx}>,ctrl_m<{idx}>,ctrl_p<{idx}>")]
            inst_term_list.append((_name, _term))
            if sw_list:
                unit_params_list.append(unit_params.copy(append=dict(m=m, sw=sw_list[idx])))
            else:
                unit_params_list.append(unit_params.copy(append=dict(m=m)))

        # Design sar_sch array
        self.array_instance('XUNIT', inst_term_list=inst_term_list, dx=2*self.instances['XUNIT'].width)
        for idx, (name, _) in enumerate(inst_term_list):
            self.instances[name].design(**unit_params_list[idx])

        # Design cm cap
        cm_cap_params = unit_params['cap']
        cm_name = f"<XCM{cm - 1}:0>" if cm > 1 else f"XCM"
        self.instances['XCM'].design(**cm_cap_params)
        if cm > 1:
            self.rename_instance('XCM', f'XCM{cm-1:0}')
        self.reconnect_instance_terminal(cm_name, 'minus', 'vref<1>')

