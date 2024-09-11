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
class bag_vco_adc__ra_bias_unit(Module):
    """Module for library bag_vco_adc cell ra_bias_unit.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'ra_bias_unit.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            nand='Parameters for nand gate',
            inv_in='Parameters for input inv gate',
            inv_out='Parameters for output inv gate',
            pbias='True to generate bias for pmos',
            msb='True to generate msb logic',
            inv_buf='Parameters of buffers'
        )

    def design(self, nand: Param, inv_in: Param, inv_out: Param, pbias: bool, msb: bool,
               inv_buf) -> None:
        if inv_buf:
            in_term = 'inbuf'
            en_term = 'enbuf'
            self.instances['XBUF_IN'].design(**inv_buf)
            self.instances['XBUF_EN'].design(**inv_buf)
        else:
            in_term = 'in'
            en_term = 'en'
            self.remove_instance('XBUF_IN')
            self.remove_instance('XBUF_EN')

        self.instances['XNAND'].design(**nand)
        if msb:
            self.instances['XINV_IN'].design(**inv_in)
            self.reconnect_instance_terminal('XNAND', 'in<1:0>', en_term+',inb')
        else:
            self.remove_instance('XINV_IN')
            self.reconnect_instance_terminal('XNAND', 'in<1:0>', en_term+','+in_term)

        has_inv_out = (pbias and not msb) or (not pbias and msb)
        if has_inv_out:
            self.instances['XINV_OUT'].design(**inv_out)
        else:
            self.remove_instance('XINV_OUT')
            self.reconnect_instance_terminal('XNAND', 'out', 'out')

