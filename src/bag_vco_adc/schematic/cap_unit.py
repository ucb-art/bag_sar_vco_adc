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

from typing import Dict, Any, List, Optional

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__cap_unit(Module):
    """Module for library bag_vco_adc cell cap_unit.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'cap_unit.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            res_minus='Parameters for metal resistor on minus terminal',
            res_plus='Parameters for metal resistor on plus terminal',
            cap='Parameters for momcap schematic value',
            minus_term='Plus term name',
            plus_term='Plus term name',
            m='Number of parallel cap',
            same_net_bot='True to add same net at res two terms',
            same_net_top = 'True to add same net at res two terms'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            plus_term='plus',
            minus_term='minus',
            cap=None,
            same_net_top=False,
            same_net_bot=False,
            m=1,
        )

    def design(self, res_minus: Dict[str, int], res_plus: Dict[str, int], m: int, plus_term: str, minus_term: str,
               cap: Optional[int], same_net_top: bool, same_net_bot: bool) -> None:
        if res_plus is not None and res_minus is not None:
            self.instances['XRES_MINUS'].design(**res_minus)
            self.instances['XRES_PLUS'].design(**res_plus)

            if m > 1:
                self.rename_instance('XRES_MINUS', f'XRES_MINUS<{m-1}:0>')
                self.rename_instance('XRES_PLUS', f'XRES_PLUS<{m-1}:0>')
                self.reconnect_instance_terminal(f'XRES_MINUS<{m-1}:0>', 'PLUS', minus_term)
                if same_net_bot:
                    self.reconnect_instance_terminal(f'XRES_MINUS<{m-1}:0>', 'MINUS', minus_term)
                else:
                    self.reconnect_instance_terminal(f'XRES_MINUS<{m-1}:0>', 'MINUS', f'{minus_term}2<{m-1}:0>')
                self.reconnect_instance_terminal(f'XRES_PLUS<{m-1}:0>', 'PLUS', plus_term)
                if same_net_top:
                    self.reconnect_instance_terminal(f'XRES_PLUS<{m-1}:0>', 'MINUS', plus_term)
                else:
                    self.reconnect_instance_terminal(f'XRES_PLUS<{m-1}:0>', 'MINUS', f'{plus_term}2<{m-1}:0>')
                # If it's schematic-only, add cap for simulation
            else:
                self.reconnect_instance_terminal('XRES_MINUS', 'PLUS', minus_term)
                if same_net_bot:
                    self.reconnect_instance_terminal('XRES_MINUS', 'MINUS', minus_term)
                else:
                    self.reconnect_instance_terminal('XRES_MINUS', 'MINUS', minus_term+'2')

                self.reconnect_instance_terminal('XRES_PLUS', 'PLUS', plus_term)
                if same_net_top:
                    self.reconnect_instance_terminal('XRES_PLUS', 'MINUS', plus_term)
                else:
                    self.reconnect_instance_terminal('XRES_PLUS', 'MINUS', plus_term+'2')
        else:
            self.delete_instance('XRES_MINUS')
            self.delete_instance('XRES_PLUS')

        if cap:
            src_load_list = [dict(type='cap', lib='analogLib', value=cap,
                                  conns=dict(PLUS=plus_term, MINUS=minus_term)) for idx in range(m)]
            self.design_sources_and_loads(src_load_list, default_name='XCAP')
            self.instances['C0'].set_param('lvsignore', 'True')
        else:
            self.delete_instance('XCAP')

        if plus_term != 'plus':
            self.rename_pin('plus', plus_term)
        if minus_term != 'minus':
            self.rename_pin('minus', minus_term)
