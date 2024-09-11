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
class bag_vco_adc__sar_async_pulse(Module):
    """Module for library bag_vco_adc cell sar_async_pulse.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_async_pulse.yaml')))

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
            buf='Parameters for output buffer',
            seg_rstp='segment for reset p',
            seg_rstn='segment for reset n',
            seg_nandp='segment for nand stack p',
            seg_nandn='segment for nand stack n',
            lch='channel length',
            wn='n-fet width',
            wp='p-fet width',
            intent='dev-type for tx other than buf',
        )

    def design(self, seg_nandp: int, seg_nandn: int, seg_rstp: int, seg_rstn: int, lch: int,
               wn: int, wp: int, intent: str, buf: Param) -> None:
        self.instances['XBUF'].design(**buf)
        self.instances['XP_RST'].design(l=lch, nf=seg_rstp, intent=intent, w=wp)
        self.instances['XN_RST'].design(l=lch, nf=seg_rstn, intent=intent, w=wn)
        self.instances['XP_PULSE'].design(lch=lch, seg=seg_nandp, intent=intent, w=wp, stack=1)
        self.rename_instance('XP_PULSE', 'XP_PULSE<1:0>', [('g', 'done,done_d')])
        self.instances['XN_PULSE'].design(lch=lch, seg=seg_nandn, intent=intent, w=wn, stack=2)


