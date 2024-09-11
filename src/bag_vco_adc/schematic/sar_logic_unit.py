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
import copy
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__sar_logic_unit(Module):
    """Module for library bag_vco_adc cell sar_logic_unit.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_logic_unit.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            oai='Parameters for oai gate',
            oai_fb='Parameters for oai output middle inv',
            buf='Parameters for output buffers template',
            buf_seg='Segment for buffer',
            buf_ratio='BUffer chain ratio',
            nand='Parameters for nand gate',
            latch='Parameters for retimer latch',
            ret_inv='Parameters for retiemr inv',
        )
    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            buf_ratio=2
        )

    def design(self, oai: Param, oai_fb: Param, buf: Param, nand: Param,
               latch: Param, ret_inv: Param, buf_seg: int, buf_ratio: int) -> None:

        for gate_type in ['N', 'P']:
            self.instances[f"XOAI_{gate_type}"].design(**oai)
            self.instances[f"XINV_{gate_type}_MID"].design(**oai_fb)

        buf_params = buf.to_dict()
        buf_m = copy.deepcopy(buf_params)
        buf_m.update(seg=buf_seg)
        buf_out = copy.deepcopy(buf_params)
        buf_out.update(seg=buf_ratio*buf_seg)
        buf_chain = dict(
            dual_output=False,
            inv_params=[
                buf_m,
                # buf_out
            ]
        )

        self.instances['XBUF_M'].design(**buf_out)
        self.instances['XBUF_P'].design(**buf_chain)
        self.instances['XBUF_N'].design(**buf_chain)
        self.instances['XNAND'].design(**nand)
        self.instances['XINV_STATE'].design(**ret_inv)
        self.instances['XL_RET'].design(**latch)
