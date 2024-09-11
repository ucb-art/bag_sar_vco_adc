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
class bag_vco_adc__delay_unit_diff_coupled(Module):
    """Module for library bag_vco_adc cell delay_unit_diff_coupled.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__, str(Path('netlist_info', 'delay_unit_diff_coupled.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls):  # type: () -> Dict[str, str]
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            lch='channel length',
            nth='nMOS threshold',
            pth='pMOS threshold',
            wn='nMOS width',
            wp='pMOS width',
            wp_coupled='coupled pMOS width',
            wn_coupled='coupled nMOS width',
            seg_n='nMOS number of finger',
            seg_p='pMOS number of finger',
            seg_n_coupled='coupled nMOS number of finger',
            seg_p_coupled='coupled pMOS number of finger',
            self_coupled='true to differential inverters output couple to each other',
            out_buf='True to enable output buffer',
        )

    @classmethod
    def get_default_param_values(cls):  # type: () -> Dict[str, Any]
        return dict(
            self=False,
            out_buf=False,
        )

    def design(self, lch, nth, pth, wn, wp, wp_coupled, wn_coupled, seg_n, seg_p, seg_n_coupled,
               seg_p_coupled, self_coupled, out_buf):
        self.instances['XNN'].design(l=lch, w=wn, intent=nth, nf=seg_n)
        self.instances['XPN'].design(l=lch, w=wn, intent=nth, nf=seg_n)
        self.instances['XNP'].design(l=lch, w=wp, intent=pth, nf=seg_p)
        self.instances['XPP'].design(l=lch, w=wp, intent=pth, nf=seg_p)

        wn = wn_coupled if wn_coupled is not None else wn
        wp = wp_coupled if wp_coupled is not None else wp
        self.instances['XNN_coupled'].design(l=lch, w=wn_coupled, intent=nth, nf=seg_n_coupled)
        self.instances['XPN_coupled'].design(l=lch, w=wn_coupled, intent=nth, nf=seg_n_coupled)
        self.instances['XNP_coupled'].design(l=lch, w=wp_coupled, intent=pth, nf=seg_p_coupled)
        self.instances['XPP_coupled'].design(l=lch, w=wp_coupled, intent=pth, nf=seg_p_coupled)

        if self_coupled:
            self.remove_pin('in_n_coupled')
            self.remove_pin('in_p_coupled')
            self.reconnect_instance_terminal('XNP_coupled', 'G', 'out_n')
            self.reconnect_instance_terminal('XNN_coupled', 'G', 'out_n')
            self.reconnect_instance_terminal('XPP_coupled', 'G', 'out_p')
            self.reconnect_instance_terminal('XPN_coupled', 'G', 'out_p')

