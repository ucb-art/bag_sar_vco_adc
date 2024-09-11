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
class bag_vco_adc__vco_ro_diff_coupled(Module):
    """Module for library bag_vco_adc cell vco_ro_diff_coupled.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'vco_ro_diff_coupled.yaml')))

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
            delay_params='unit inverter parameters',
            buf_params='unit inverter parameters',
            num_stage='number of stage in RO',
            delta='coupled inverters from delta stage before',
            dum_info='dummy transistor information',
            vtop_core='name of RO core top supply',
            vbot_core='name of RO core top supply',
            vtop_buf='name of buffer core top supply',
            vbot_buf='name of buffer core top supply',
            ndum='Number of dummy stages',
        )

    @classmethod
    def get_default_param_values(cls):  # type: () -> Dict[str, Any]
        return dict(
            delta=2,
            dum_info=None,
            vtop_buf='VDD',
            vbot_buf='VSS',
            vtop_core='VDD',
            vbot_core='VSS',
            ndum=0,
        )

    def design(self, delay_params, buf_params, num_stage, delta, dum_info,
               vtop_core, vbot_core, vtop_buf, vbot_buf, ndum):
        p_out = f"phi<0:{num_stage-1}>"
        n_out = f"phi<{num_stage}:{2*num_stage-1}>"
        p_in = f"phi<{num_stage-1}:{2*num_stage-2}>"
        n_in = f"phi<{2*num_stage-1}>,"+f"phi<0:{num_stage-2}>"
        p_coupled_in = f"phi<{2*num_stage-delta}:{2*num_stage-1}>,"+f"phi<0:{num_stage-delta-1}>"
        n_coupled_in = f"phi<{num_stage-delta}:{2*num_stage-delta-1 }>"
        suffix = f"<{num_stage - 1}:0>" if num_stage > 1 else ''

        name_list = ['XUNIT' + suffix]
        term_list = [{'out_p': p_out, 'out_n': n_out, 'in_p': p_in, 'in_n': n_in,
                      'in_p_coupled': p_coupled_in, 'in_n_coupled': n_coupled_in,
                      'VDD': 'VDD', 'VSS': 'VSS', 'VTOP': vtop_core, 'VBOT': vbot_core}]

        self.design_dummy_transistors(dum_info, 'XDUMMY', 'VDD', 'VSS')

        self.instances['XUNIT'].design(**delay_params)
        self.array_instance('XUNIT', name_list, term_list)
        self.rename_pin('phi', f"phi<0:{2*num_stage-1}>")

        if delay_params['out_buf']:
            out_buf = f"phi_buf<0:{2*num_stage - 1}>"
            self.instances['XBUF'].design(**buf_params)
            name_list = ['XBUF' + suffix]
            term_list = [{'out_p': p_out.replace('phi', 'phi_buf'), 'out_n': n_out.replace('phi', 'phi_buf'),
                          'in_p': p_out, 'in_n': n_out, 'VDD': 'VDD', 'VSS': 'VSS',
                          'VTOP': vtop_buf, 'VBOT': vbot_buf}]
            self.reconnect_instance_terminal('XBUF', f"in<{2*num_stage-1}:0>", f"phi<{2*num_stage-1}:0>")
            self.reconnect_instance_terminal('XBUF', f"out<{2*num_stage-1}:0>", f"phi_buf<{2*num_stage-1}:0>")
            self.array_instance('XBUF', name_list, term_list)
            self.rename_pin('phi_buf', out_buf)
        else:
            self.remove_instance('XBUF')
            self.remove_pin('phi_buf')

        if vtop_buf == 'VDD' and vtop_core == 'VDD':
            self.remove_pin('VTOP')
        if vbot_buf == 'VSS' and vbot_core == 'VSS':
            self.remove_pin('VBOT')

        if ndum:
            self.instances['XUNIT_DUM'].design(**delay_params)
            self.rename_instance('XUNIT_DUM', f'XUNIT_DUM<{ndum-1}:0>')
            if delay_params['out_buf']:
                self.rename_instance('XBUF_DUM', f'XBUF_DUM<{ndum-1}:0>')
                self.instances['XBUF_DUM'].design(**buf_params)
            else:
                self.remove_instance('XBUF_DUM')
        else:
            self.remove_instance('XBUF_DUM')
            self.remove_instance('XUNIT_DUM')
