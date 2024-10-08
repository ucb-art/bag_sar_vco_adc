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

from typing import Mapping, Any

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__clk_delay_tune(Module):
    """Module for library bag_vco_adc cell clk_delay_tune.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'clk_delay_tune.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Mapping[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            binary_l='',
            binary_u='',
            unary='',
        )

    def design(self, binary_l, binary_u, unary) -> None:
        self.instances['XBINARY_U'].design(**binary_u)
        self.instances['XBINARY_L'].design(**binary_l)
        self.instances['XUNARY'].design(**unary)

        lsb = unary['nbits']
        msb_l = binary_l['nbits']
        msb_u = binary_u['nbits']

        for pin in self.instances['XUNARY'].master.pins.keys():
            self.reconnect_instance_terminal('XUNARY', pin, pin)

        for pin in self.instances['XBINARY_L'].master.pins.keys():
            new_pin = pin if 'in' not in pin else f'in<{lsb+msb_l-1}:{lsb}>'
            self.reconnect_instance_terminal('XBINARY_L', pin, new_pin)

        for pin in self.instances['XBINARY_U'].master.pins.keys():
            new_pin = pin if 'in' not in pin else f'in<{lsb+msb_l+msb_u-1}:{lsb+msb_l}>'
            self.reconnect_instance_terminal('XBINARY_U', pin, new_pin)
        self.rename_pin('in', f'in<{lsb+msb_l+msb_u-1}:0>')

        self.rename_instance('XBINARY_L', 'XBINARY_L<1:0>')
        self.rename_instance('XBINARY_U', 'XBINARY_U<1:0>')
