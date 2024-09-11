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
class bag_vco_adc__sar_async_clk_core_v2(Module):
    """Module for library bag_vco_adc cell sar_async_clk_core_v2.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_async_clk_core_v2.yaml')))

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
            inv_fb='Parameters for clk_out inverter',
            nor_en='Nor gate for start-up',
            nand_done='Nand gate for stop',
            nand_en='Nand gate for stop',
            buf='Clock output buffer',
            clkbuf='Clock output buffer',
            lch='channel length.',
            w_p='PMOS width.',
            w_n='NMOS width.',
            th_p='PMOS threshold.',
            th_n='NMOS threshold.',
            seg_dict='Dictionary of other transistors segment',
        )

    def design(self, buf: Param, clkbuf: Param, inv_fb: Param, nor_en: Param, nand_done: Param, nand_en: Param,
               lch: int, w_p: int, w_n: int, th_p: str, th_n: str, seg_dict: Dict[str, int]) -> None:
        self.instances['XINV_FB'].design(**inv_fb)
        self.instances['XBUF_OUT'].design(**buf)
        self.instances['XBUF_LOGIC_CLK'].design(**clkbuf)
        self.instances['XNOR_EN'].design(**nor_en)
        self.instances['XNAND_DONE'].design(**nand_done)
        self.instances['XNAND_EN'].design(**nand_en)

        self.instances['XN_FB'].design(w=w_n, l=lch, nf=seg_dict['fb_n'], intent=th_n)
        self.instances['XN_TAIL'].design(w=w_n, l=lch, nf=seg_dict['tail_n'], intent=th_n)
        self.instances['XN_CLK'].design(w=w_n, l=lch, nf=seg_dict['clk_n'], intent=th_n)

        self.instances['XP_FB'].design(w=w_p, l=lch, nf=seg_dict['fb_p'], intent=th_p)
        self.instances['XP_INN'].design(w=w_p, l=lch, nf=seg_dict['in_p'], intent=th_p)
        self.instances['XP_INP'].design(w=w_p, l=lch, nf=seg_dict['in_p'], intent=th_p)
        self.remove_pin('sel')

