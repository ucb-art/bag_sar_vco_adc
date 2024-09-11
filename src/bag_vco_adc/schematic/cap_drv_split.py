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

from pathlib import Path
from typing import Any, Dict

import pkg_resources

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__cap_drv_split(Module):
    """Module for library bag_vco_adc cell cap_drv_split.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'cap_drv_split.yaml')))

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
            lch='channel length of switch',
            seg_dict='segment of switch',
            w_dict='width dict',
            intent='intent of switch',
            ndrv='True to rst low',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(ndrv=False)

    def design(self, seg_dict: Dict[str, int], w_dict: Dict[str, int],
               lch: int, intent: str, ndrv: bool) -> None:
        self.instances['XP'].design(l=lch, w=w_dict.get('p', 4), intent=intent, nf=seg_dict['seg'], stack=1)
        self.instances['XN'].design(l=lch, w=w_dict.get('n', 4), intent=intent, nf=seg_dict['seg'], stack=1)
        self.instances['XEN_P'].design(l=lch, w=w_dict.get('muxp', 4), intent=intent, nf=seg_dict['seg_mux'], stack=1)
        self.instances['XEN_N'].design(l=lch, w=w_dict.get('muxn', 4), intent=intent, nf=seg_dict['seg_mux'], stack=1)
        if ndrv:
            self.instances['XRST_N'].design(l=lch, w=w_dict.get('rst', 4),
                                            intent=intent, nf=seg_dict['seg_rst'], stack=1)
            self.remove_instance('XRST_P')
            self.reconnect_instance('XN', [('G', 'ctrl_buf')])
            self.reconnect_instance('XP', [('G', 'ctrl')])
        else:
            self.instances['XRST_P'].design(l=lch, w=w_dict.get('rst', 4),
                                            intent=intent, nf=seg_dict['seg_rst'], stack=1)
            self.remove_instance('XRST_N')
