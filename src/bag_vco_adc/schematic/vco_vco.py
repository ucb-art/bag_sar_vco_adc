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

from typing import Dict, Any

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__vco_vco(Module):
    """Module for library bag_vco_adc cell vco_vco.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'vco_vco.yaml')))

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
            ro_params='Ring osc parameters',
            ctrl_params='Ctrl parameters',
            is_pctrl='True to use pmos control',
        )

    def design(self, ro_params, ctrl_params, is_pctrl) -> None:
        nstage = ro_params['num_stage']
        # if is_pctrl:
        #     self.remove_instance('XCTRL_N')
        #     self.remove_pin('vbot')
        #     self.remove_pin('vctrl_n')
        #     self.instances['XCTRL_P'].design(**ctrl_params)
        # else:
        self.remove_pin('vtop')
        self.instances['XCTRL'].design(**ctrl_params)
        self.instances['XRO'].design(**ro_params)
        self.reconnect_instance('XRO', [('VDD', 'VDD'), ('VSS', 'VSS'),
                                        (f'phi<0:{2*nstage-1}>', f'phi<0:{2*nstage-1}>'),
                                        (f'phi_buf<0:{2*nstage-1}>', f'phi_buf<0:{2*nstage-1}>'),
                                        ('VBOT', 'VSS' if is_pctrl else 'vbot'),
                                        ('VTOP', 'vtop' if is_pctrl else 'VDD')])
        self.rename_pin('phi', f'phi<{2*nstage-1}:0>')
        if sum(['phi_buf' in k for k in self.instances['XRO'].master.pins.keys()]):
            self.rename_pin('phi_buf', f'phi_buf<{2*nstage-1}:0>')
        else:
            self.remove_pin('phi_buf')

