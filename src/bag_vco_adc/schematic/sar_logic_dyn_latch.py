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
from typing import Mapping, Any

import pkg_resources

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__sar_logic_dyn_latch(Module):
    """Module for library bag_vco_adc cell sar_logic_dyn_latch.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_logic_dyn_latch.yaml')))

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
            seg_d='data tx size',
            seg_en='enable tx size',
            seg_pull='pull pu tx size',
            seg_inv='inv seg',
            seg_inv_fb='inv seg_fb',
            w_dict='width dictionary',
            lch='channel length',
            th_n='nmos threshold',
            th_p='pmos threshold',
            nand='True to use resetb',
            np_coupled='True to export ob fb port',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(nand=False, np_coupled=False)

    def design(self, seg_d, seg_en, seg_pull, seg_inv, seg_inv_fb, w_dict, lch, th_n, th_p, nand, np_coupled) -> None:
        if nand:
            self.rename_pin('rstb', 'rst')
            self.replace_instance_master('XPU', 'BAG_prim', 'nmos4_standard', keep_connections=True)
            self.replace_instance_master('XD', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.replace_instance_master('XEN', 'BAG_prim', 'pmos4_standard', keep_connections=True)
            self.reconnect_instance('XPU', [('S', 'VSS'), ('B', 'VSS'), ('G', 'rst')])
            self.reconnect_instance('XD', [('B', 'VDD')])
            self.reconnect_instance('XEN', [('S', 'VDD'), ('B', 'VDD')])

        w_inv_fb_n, w_inv_fb_p = w_dict['inv_fb']['wn'], w_dict['inv_fb']['wp']
        if np_coupled:
            if nand:
                self.remove_instance('XP1')
                self.instances['XN1'].design(l=lch, nf=seg_inv_fb, stack=1, w=w_inv_fb_n, intent=th_n)
            else:
                self.remove_instance('XN1')
                self.instances['XP1'].design(l=lch, nf=seg_inv_fb, stack=1, w=w_inv_fb_p, intent=th_p)
        else:
            self.reconnect_instance('XP1', [('G', 'o')])
            self.reconnect_instance('XN1', [('G', 'o')])
            self.remove_pin('ob_fb')
            self.instances['XP1'].design(l=lch, nf=seg_inv_fb, stack=1, w=w_inv_fb_p, intent=th_p)
            self.instances['XN1'].design(l=lch, nf=seg_inv_fb, stack=1, w=w_inv_fb_n, intent=th_n)
        w_d, w_pull, w_en = w_dict['d'], w_dict['pull'], w_dict['en']
        w_inv_n, w_inv_p = w_dict['inv']['wn'], w_dict['inv']['wp']
        self.instances['XPU'].design(l=lch, nf=seg_pull, stack=1, w=w_pull, intent=th_p)
        self.instances['XD'].design(l=lch, nf=1, stack=1, w=w_d, intent=th_n)
        self.instances['XEN'].design(l=lch, nf=1, stack=1, w=w_en, intent=th_n)
        self.rename_instance('XD', f'XD<{seg_d - 1}:0>', [('S', 'mid')])
        self.rename_instance('XEN', f'XEN<{seg_en - 1}:0>', [('D', 'mid')])

        self.instances['XP0'].design(l=lch, nf=seg_inv, stack=1, w=w_inv_p, intent=th_p)
        self.instances['XN0'].design(l=lch, nf=seg_inv, stack=1, w=w_inv_n, intent=th_n)

