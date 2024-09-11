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

from typing import Any, Dict, List

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__sar_comp(Module):
    """Module for library bag_vco_adc cell sar_comp.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_comp.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            buf_outb='True to use outb of invchain',
            sch_cls='sch template name',
            sa_params='Parameters of strongarm core',
            buf_params='Parameters of inv buffers',
            dum_info='Dummy info',
            se_clk='Single ended clock'
        )

    def design(self, sch_cls: str, buf_outb: bool, sa_params: Param, buf_params: Param, dum_info: List,
               se_clk=True) -> None:

        if se_clk:
            self.remove_pin('clkb')
        self.replace_instance_master('XSA', 'bag_vco_adc', sch_cls, keep_connections=True)
        self.instances['XSA'].design(**sa_params)
        self.instances['XBUF<1:0>'].design(**buf_params)
        len_buf = len(buf_params['inv_params'])

        # swap output depending on number of stages in buffers
        if bool(len_buf & 1):
            self.reconnect_instance_terminal('XBUF<1:0>', 'out' if buf_outb else 'outb', 'outn,outp')
            mid_name = 'outb' if buf_outb else 'out'
        else:
            self.reconnect_instance_terminal('XBUF<1:0>', 'outb' if buf_outb else 'out', 'outp,outn')
            mid_name = 'out' if buf_outb else 'outb'
        if len_buf > 1:
            self.reconnect_instance_terminal('XBUF<1:0>', mid_name, mid_name + '_mid_p,' + mid_name + '_mid_n')

        self.reconnect_instance_terminal('XBUF<1:0>', 'mid<1:0>', 'midn,outn_m,midp,outp_m')
        self.reconnect_instance_terminal('XBUF<1:0>', 'in', 'outp_m,outn_m')
        self.reconnect_instance_terminal('XSA', 'outp', 'outp_m')
        self.reconnect_instance_terminal('XSA', 'outn', 'outn_m')

        if dum_info:
            self.design_dummy_transistors(dum_info, 'XDUM', 'VDD', 'VSS')
        else:
            self.remove_instance('XDUM')
