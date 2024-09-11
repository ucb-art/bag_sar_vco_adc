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
class bag_vco_adc__sar_logic_bot(Module):
    """Module for library bag_vco_adc cell sar_logic_bot.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_logic_bot.yaml')))

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
            clkgen='Parameters for clkgen',
            sar_logic='Parameters for sar logic and retimer',
        )

    def design(self, nbits: int, sar_logic: Param, clkgen: Param) -> None:
        for pname in ['dm', 'dn', 'dp', 'data_out', 'state']:
            self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")

        if sar_logic['has_pmos_sw']:
            logic_conn = [(f"state<{nbits - 1}:0>", f"state<{nbits - 1}:0>"),
                          (f"data_out<{nbits - 1}:0>", f"data_out<{nbits - 1}:0>"),
                          (f"dm<{nbits - 1}:0>", f"dm<{nbits - 1}:0>"),
                          (f"dn<{nbits - 1}:0>", f"dn<{nbits - 1}:0>"),
                          (f"dn_b<{nbits - 1}:0>", f"dn_b<{nbits - 1}:0>"),
                          (f"dp_b<{nbits - 1}:0>", f"dp_b<{nbits - 1}:0>"),
                          (f"dp<{nbits - 1}:0>", f"dp<{nbits - 1}:0>"),
                          ('sar_clk', 'logic_clk')]
            for pname in ['dn_b', 'dp_b']:
                self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")
        else:
            logic_conn = [(f"state<{nbits - 1}:0>", f"state<{nbits - 1}:0>"),
                          (f"data_out<{nbits - 1}:0>", f"data_out<{nbits - 1}:0>"),
                          (f"dm<{nbits - 1}:0>", f"dm<{nbits - 1}:0>"),
                          (f"dn<{nbits - 1}:0>", f"dn<{nbits - 1}:0>"),
                          (f"dp<{nbits - 1}:0>", f"dp<{nbits - 1}:0>"),
                          ('sar_clk', 'logic_clk')]
        self.instances['XLOGIC'].design(**sar_logic)
        self.rename_instance('XLOGIC', 'XLOGIC0', logic_conn)

        self.instances['XCLKGEN'].design(**clkgen)
        self.reconnect_instance_terminal('XCLKGEN', 'stop', 'state<0>')
