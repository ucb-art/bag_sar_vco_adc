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
class bag_vco_adc__vco_cnter_div(Module):
    """Module for library bag_vco_adc cell vco_cnter_div.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'vco_cnter_div.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls):
        # type: () -> Dict[str, str]
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            latch_params_list='unit inverter parameters',
            num_stages='number of stage in RO',
            clkbuf='Has clock bufer',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            clkbuf=None
        )

    def design(self, latch_params_list, num_stages, clkbuf):
        if clkbuf:
            self.instances['XCLKBUF'].design(**clkbuf)
            self.rename_instance('XCLKBUF', 'XCLKBUF<1:0>', [('VDD', 'VDD'), ('VSS', 'VSS'),
                                                             ('in', 'clkn,clkp'), ('out', 'clkp_int,clkn_int')])
        else:
            self.remove_instance('XCLKBUF')
        name_list = [f'XL<{idx}>' for idx in range(num_stages)] if num_stages > 1 else 'XL'
        clkp_name = ['clkp_int', 'clkn_int'] * (num_stages // 2) if clkbuf else ['clkp', 'clkn'] * (num_stages // 2)
        clkn_name = ['clkn_int', 'clkp_int'] * (num_stages // 2) if clkbuf else ['clkn', 'clkp'] * (num_stages // 2)

        out_name = [f'outp<{idx}>' for idx in range(num_stages)]
        out_b_name = [f'outn<{idx}>' for idx in range(num_stages)]

        inp_name_shift = [f'outn<{num_stages-1}>']+[f'outp<{idx}>' for idx in range(num_stages-1)]
        inn_name_shift = [f'outp<{num_stages-1}>']+[f'outn<{idx}>' for idx in range(num_stages-1)]
        term_list = [{'outp': out_name[idx], 'outn': out_b_name[idx],
                      'dn': inn_name_shift[idx], 'd': inp_name_shift[idx],
                      'clkn': clkn_name[idx], 'clkp': clkp_name[idx]} for idx in range(num_stages)]

        self.array_instance('XL', name_list, term_list)

        for idx in range(num_stages):
            self.instances[f'XL<{idx}>'].design(**latch_params_list[idx])
        self.rename_pin('out', f'outp<0:{num_stages-1}>')
        self.rename_pin('outn', f'outn<0:{num_stages-1}>')
