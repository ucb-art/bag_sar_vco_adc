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

from pybag.enum import TermType

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


from ..schematic import cdac_array


# noinspection PyPep8Naming
class bag_vco_adc__sar_slice(Module):
    """Module for library bag_vco_adc cell sar_slice.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_slice.yaml')))

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
            nbits='Number of bits',
            comp='Parameters of comparator',
            logic='Parameters of sar logic block',
            cdac='Parameters of cdac',
            ideal_switch='True to put ideal switch in front of SAR for sch simulation'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ideal_switch=True
        )

    def design(self, nbits: int, comp: Param, logic: Param, cdac: Param, ideal_switch: bool) -> None:
        # clk_gen_bit = logic['clkgen']['num_delay']

        for pname in ['dm', 'dn', 'dp', 'data_out']:
            self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")
        self.rename_pin('vref', 'vref<2:0>')
        # self.rename_pin('clk_sel', f"clk_sel<{clk_gen_bit-1}:0>")
        self.remove_pin('clk_sel')

        self.instances['XCOMP'].design(**comp),
        self.instances['XLOGIC'].design(**logic)
        [self.instances[inst].design(**cdac) for inst in ['XDACN', 'XDACP']]

        logic_conn = [(f"state<{nbits - 1}:0>", f"state<{nbits - 1}:0>"),
                      (f"data_out<{nbits - 1}:0>", f"data_out<{nbits - 1}:0>"),
                      (f"dm<{nbits - 1}:0>", f"dm<{nbits - 1}:0>"),
                      (f"dn<{nbits - 1}:0>", f"dn<{nbits - 1}:0>"),
                      (f"dp<{nbits - 1}:0>", f"dp<{nbits - 1}:0>"),]
                      # (f"clk_sel<{clk_gen_bit-1}:0>", f"clk_sel<{clk_gen_bit-1}:0>")]
        self.instances['XLOGIC'].design(**logic)
        for con_pair in logic_conn:
            self.reconnect_instance_terminal('XLOGIC', con_pair[0], con_pair[1])

        dac_conn_p = [(f"vref<2:0>", f"vref<2:0>"),
                      (f"ctrl_m<{nbits - 2}:0>", f"dm<{nbits - 1}:1>"),
                      (f"ctrl_p<{nbits - 2}:0>", f"dp<{nbits - 1}:1>"),
                      (f"ctrl_n<{nbits - 2}:0>", f"dn<{nbits - 1}:1>"),]

        dac_conn_n = [(f"vref<2:0>", f"vref<2:0>"),
                      (f"ctrl_m<{nbits - 2}:0>", f"dm<{nbits - 1}:1>"),
                      (f"ctrl_p<{nbits - 2}:0>", f"dn<{nbits - 1}:1>"),
                      (f"ctrl_n<{nbits - 2}:0>", f"dp<{nbits - 1}:1>"),]

        for con_pair in dac_conn_n:
            self.reconnect_instance_terminal('XDACN', con_pair[0], con_pair[1])
        for con_pair in dac_conn_p:
            self.reconnect_instance_terminal('XDACP', con_pair[0], con_pair[1])

        if cdac['bot_probe']:
            self.reconnect_instance_terminal('XDACN', f'bot<{nbits-2}:0>', f'bot_n<{nbits-2}:0>')
            self.reconnect_instance_terminal('XDACP', f'bot<{nbits-2}:0>', f'bot_p<{nbits-2}:0>')
            self.add_pin(f'bot_n<{nbits-2}:0>', TermType.inout)
            self.add_pin(f'bot_p<{nbits-2}:0>', TermType.inout)

        self.add_pin(f"state<{nbits - 1}:0>", TermType.output)
        # if ideal_switch:
        #     for inst_name in ['XSW_N', 'XSW_P']:
        #         for key, val in [('vt1', 'vopen'), ('vt2', 'vclosed'), ('ro', 50), ('rc', 1.0e12)]:
        #             self.instances[inst_name].set_param(key, val)
        #         self.instances[inst_name].set_param('lvsignore', 'True')
        #
        # else:
        #     for inst_name in ['XSW_N', 'XSW_P']:
        #         self.remove_instance(inst_name)

