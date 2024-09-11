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
from typing import Dict, Any

import pkg_resources

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__vco_slice(Module):
    """Module for library bag_vco_adc cell vco_slice.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'vco_slice.yaml')))

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
            slice_params='',
        )

    def design(self, slice_params: Param) -> None:
        n_term_list, p_term_list = [], []
        self.instances['XN'].design(**slice_params)
        self.instances['XP'].design(**slice_params)
        lsb = slice_params['nlsb']
        msb = slice_params['nmsb']
        for pinname in ['cnter_outp_n', 'cnter_outn_n', 'cnter_outp_p', 'cnter_outn_p']:
            self.rename_pin(pinname, pinname+f'<{msb * 2 - 1}:0>')

        num_ph = slice_params['ro_params']['vco_params']['ro_params']['num_stage']*2

        self.rename_pin('phi_n', f'phi_n<{num_ph - 1}:0>')
        self.rename_pin('phi_p', f'phi_p<{num_ph - 1}:0>')

        n_term_list.extend([('clk', 'clk'), ('clkb', 'clkb'), ('vctrl_n', 'vctrl_n'), ('vbot', 'vbot_n'),
                            (f'lsb<{lsb - 1}:0>', f'bit_n<{lsb - 1}:0>'),
                            (f'msb<{msb - 1}:0>', f'bit_n<{msb + lsb - 1}:{lsb}>'),
                            (f'cnter_outp<{msb * 2 - 1}:0>', f'cnter_outp_n<{msb * 2 - 1}:0>'),
                            (f'cnter_outn<{msb * 2 - 1}:0>', f'cnter_outn_n<{msb * 2 - 1}:0>'),
                            (f'phi<{num_ph - 1}:0>', f'phi_n<{num_ph - 1}:0>'),
                            ('cnter_clkn', 'cnter_clkn_n'), ('cnter_clkp', 'cnter_clkp_n')])

        p_term_list.extend([('clk', 'clk'), ('clkb', 'clkb'), ('vctrl_n', 'vctrl_p'), ('vbot', 'vbot_p'),
                            (f'lsb<{lsb - 1}:0>', f'bit_p<{lsb - 1}:0>'),
                            (f'msb<{msb - 1}:0>', f'bit_p<{msb + lsb - 1}:{lsb}>'),
                            (f'cnter_outp<{msb * 2 - 1}:0>', f'cnter_outp_p<{msb * 2 - 1}:0>'),
                            (f'cnter_outn<{msb * 2 - 1}:0>', f'cnter_outn_p<{msb * 2 - 1}:0>'),
                            (f'phi<{num_ph - 1}:0>', f'phi_p<{num_ph - 1}:0>'),
                            ('cnter_clkn', 'cnter_clkn_p'), ('cnter_clkp', 'cnter_clkp_p')])

        if sum(['phi_buf' in pinname for pinname in self.instances['XN'].master.pins.keys()]):
            n_term_list.append((f'phi_buf<{num_ph - 1}:0>', f'phi_buf_n<{num_ph - 1}:0>'))
            p_term_list.append((f'phi_buf<{num_ph - 1}:0>', f'phi_buf_p<{num_ph - 1}:0>'))
            self.rename_pin('phi_buf_n', f'phi_buf_n<{num_ph - 1}:0>')
            self.rename_pin('phi_buf_p', f'phi_buf_p<{num_ph - 1}:0>')
        else:
            self.remove_pin('phi_buf_n')
            self.remove_pin('phi_buf_p')

        # for pin in self.instances['XN'].master.pins.keys():
        #     if 'cnter' in pin:
        #         n_term_list.append((pin, pin.replace('cnter', 'cntern')))
        #         p_term_list.append((pin, pin.replace('cnter', 'cnterp')))

        self.reconnect_instance('XN', n_term_list)
        self.reconnect_instance('XP', p_term_list)

        self.rename_pin('bit_n', f'bit_n<{msb + lsb - 1}:0>')
        self.rename_pin('bit_p', f'bit_p<{msb + lsb - 1}:0>')
