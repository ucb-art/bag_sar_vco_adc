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
class bag_vco_adc__sar_fsm(Module):
    """Module for library bag_vco_adc cell sar_fsm.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_fsm.yaml')))

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
            rst_flop='Parameters for reset flop',
            state_flop='Parameters for state flop',
            inv='Parameters for trigger buffer inverter',
        )

    def design(self, nbits: int, inv: Param, rst_flop: Param, state_flop: Param) -> None:

        self.rename_pin('state', f"state<{nbits-1}:0>")
        self.instances['XF_RST'].design(**rst_flop)
        self.instances['XINV'].design(**inv)

        # Make state flops array
        inst_term_list = []
        for idx in range(nbits):
            _in = f"state_b<{nbits-idx}>" if idx else 'trig_b'
            _name = f"XF_STATE<{idx}>"
            _term = [('VDD', 'VDD'), ('VSS', 'VSS'), ('clk', 'clk'),
                     ('in', _in), ('outb', f"state<{nbits-idx-1}>"), ('out', f"state_b<{nbits-idx-1}>")]
            inst_term_list.append((_name, _term))

        state_flop_params = state_flop.to_dict()
        state_flop_params.update(dual_output=True)
        self.array_instance('XF_STATE', inst_term_list=inst_term_list)
        for name, _ in inst_term_list:
            self.instances[name].design(**state_flop_params)
