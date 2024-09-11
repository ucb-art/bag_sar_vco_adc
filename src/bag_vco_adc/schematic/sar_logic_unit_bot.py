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

from typing import Dict, Any, Optional

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__sar_logic_unit_bot(Module):
    """Module for library bag_vco_adc cell sar_logic_unit_bot.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_logic_unit_bot.yaml')))

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
            oai='Parameters for oai gate',
            oai_fb='Parameters for oai output middle inv',
            buf='Parameters for output buffers template',
            buf_np='Parameters for output buffers template',
            buf_seg='Segment for buffer',
            buf_ratio='BUffer chain ratio',
            nor='Parameters for nor gate',
            latch='Parameters for retimer latch',
            buf_state='Inverter for state signal',
            flop='Flop parameters',
            pg='Passgate parameters, used when has_pmos_sw = True',
            has_pmos_sw='True if CDAC has pmos switch, need differential logic',
            msb='True to enable msb flops',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            buf_ratio=2,
            buf_seg=-1,
            has_pmos_sw=False,
            pg=None,
            msb=False
        )

    def design(self, flop: Param, buf_state: Param, oai: Param,
               oai_fb: Param, buf: Param, buf_np: Param, nor: Param, latch: Param, pg: Optional[Param],
               has_pmos_sw: bool, buf_seg: int, buf_ratio: int, msb: bool) -> None:

        for gate_type in ['N', 'P']:
            self.instances[f"XOAI_{gate_type}"].design(**oai)
            self.instances[f"XINV_{gate_type}_MID"].design(**oai_fb)

        self.instances['XINV_STATE'].design(**buf_state)
        self.instances['XFF_STATE'].design(**flop)
        if msb:
            self.instances['XFF_STATE_INIT'].design(**flop)
            self.instances['XINV_STATE_INIT'].design(**buf_state)
        else:
            self.delete_instance('XFF_STATE_INIT')
            self.delete_instance('XINV_STATE_INIT')

        # self.instances['XINV_STATE1'].design(**buf_state[1])
        self.instances['XL_RET'].design(**latch)
        self.instances['XBUF_M'].design(**buf)
        self.instances['XBUF_P'].design(**buf_np)
        self.instances['XBUF_N'].design(**buf_np)

        self.reconnect_instance_terminal('XNOR', 'in<2:0>', 'rst,dn_mid,dp_mid')
        self.instances['XNOR'].design(**nor)
        self.remove_pin('rstb')
        if has_pmos_sw:
            self.reconnect_instance_terminal('XBUF_N', 'out', 'dn_m')
            self.reconnect_instance_terminal('XBUF_N', 'outb', 'dn_b')
            self.reconnect_instance_terminal('XBUF_P', 'out', 'dp_m')
            self.reconnect_instance_terminal('XBUF_P', 'outb', 'dp_b')
            self.reconnect_instance_terminal('XL_RET', 'in', 'dp_m')
            self.instances['XPG_P'].design(**pg)
            self.instances['XPG_N'].design(**pg)
        else:
            self.delete_instance('XPG_N')
            self.delete_instance('XPG_P')
            self.remove_pin('dn_b')
            self.remove_pin('dp_b')

