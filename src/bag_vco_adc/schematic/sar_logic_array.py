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
import copy
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__sar_logic_array(Module):
    """Module for library bag_vco_adc cell sar_logic_array.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_logic_array.yaml')))

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
            nbits='Number of bits in SAR',
            buf_list='List of buffer segments',
            buf_clk='Parameters for clk buffer (for retimer)',
            buf_out='Parameters for clk buffer (for output)',
            logic='Parameters for sar logic unit',
            logic_list='Parameters list for sar logic unit',
            ret='Parameters for retimer unit',
            has_pmos_sw='True if CDAC has pmos switch, need differential logic',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            logic=None,
            logic_list=[],
            buf_list=[],
            has_pmos_sw=False
        )

    def design(self, nbits: int, buf_list: List[int], logic_list: List[Param], buf_clk: Param, buf_out: Param,
               logic: Param, ret: Param, has_pmos_sw: bool) -> None:
        # Rename pins
        for pname in ['dm', 'dn', 'dp', 'state', 'data_out']:
            self.rename_pin(pname, f"{pname}<{nbits-1}:0>")

        if has_pmos_sw:
            for pname in ['dn_b', 'dp_b']:
                self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")
        else:
            self.remove_pin('dn_b')
            self.remove_pin('dp_b')

        # Design instances
        self.instances['XBUF_CLK'].design(**buf_clk)
        self.instances['XBUF_OUT'].design(**buf_out)
        self.instances["XRET"].design(**ret)

        # Array logic units
        logic_term_list = []
        for idx in range(nbits):
            _name = f'XLOGIC{idx}'
            bit_conn = f"state<{nbits-idx}>" if idx else 'trig'
            if has_pmos_sw:
                _term = [('bit_prev', bit_conn), ('bit', f"state<{nbits - idx - 1}>"),
                         ('dm', f'dm<{nbits - idx - 1}>'), ('dp', f'dp<{nbits - idx - 1}>'),
                         ('dn', f'dn<{nbits - idx - 1}>'), ('dp_b', f'dp_b<{nbits - idx - 1}>'),
                         ('dn_b', f'dn_b<{nbits - idx - 1}>'), ('out_ret', f"out_ret<{nbits - idx - 1}>"),
                         ('clk', 'sar_clk')]
            else:
                _term = [('bit_prev', bit_conn), ('bit', f"state<{nbits-idx-1}>"),
                         ('dm', f'dm<{nbits-idx-1}>'), ('dp', f'dp<{nbits-idx-1}>'), ('dn', f'dn<{nbits-idx-1}>'),
                         ('out_ret', f"out_ret<{nbits-idx-1}>"), ('clk', 'sar_clk')]
            logic_term_list.append((_name, _term))

        self.array_instance('XLOGIC', inst_term_list=logic_term_list, dx=2*self.instances['XLOGIC'].width)

        if logic_list:
            for idx, _params in enumerate(logic_list):
                self.instances[f'XLOGIC{idx}'].design(**_params)
        else:
            logic_unit_params = logic.to_dict()
            for idx in range(nbits):
                _params = copy.deepcopy(logic_unit_params)
                _params.update(buf_seg=buf_list[idx])
                self.instances[f'XLOGIC{idx}'].design(**_params)

        # Array retimer units
        retimer_conn = [('in', f"out_ret<{nbits-1}:0>"), ('out', f"data_out<{nbits-1}:0>")]
        self.rename_instance('XRET', f"XRET<{nbits-1}:0>", retimer_conn)
