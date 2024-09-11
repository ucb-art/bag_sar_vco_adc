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
class bag_vco_adc__ra_core(Module):
    """Module for library bag_vco_adc cell ra_core.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'ra_core.yaml')))

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
            ra_params='',
            cap_fb_params='',
            cap_sam_params='',
            sw_out_params='',
            sw_mid_params='',
            flip_cap='',
        )

    def design(self, ra_params, cap_fb_params, cap_sam_params, sw_out_params, sw_mid_params,
               flip_cap) -> None:
        self.instances['XCORE'].design(**ra_params)
        self.instances['XCAP_FB_N'].design(**cap_fb_params)
        self.instances['XCAP_FB_P'].design(**cap_fb_params)
        self.instances['XCAP_SAM_N'].design(**cap_sam_params)
        self.instances['XCAP_SAM_P'].design(**cap_sam_params)
        self.instances['XSW_OUT'].design(**sw_out_params)
        self.instances['XSW_MID'].design(**sw_mid_params)

        fb_cap_term_list = []
        for pin in self.instances['XCAP_FB_N'].master.pins.keys():
            if 'bit' in pin:
                fb_cap_term_list.append((pin, pin.replace('bit', 'ctrl_fb_cap')))
                self.rename_pin('ctrl_fb_cap', pin.replace('bit', 'ctrl_fb_cap'))

        for pin in self.instances['XCORE'].master.pins.keys():
            if 'ctrl_dac_n' in pin:
                self.reconnect_instance_terminal('XCORE', pin, pin)
                self.rename_pin('ctrl_dac_n', pin)
            if 'ctrl_dac_p' in pin:
                self.reconnect_instance_terminal('XCORE', pin, pin)
                self.rename_pin('ctrl_dac_p', pin)

        self.reconnect_instance('XCAP_FB_N', fb_cap_term_list)
        self.reconnect_instance('XCAP_FB_P', fb_cap_term_list)

        if flip_cap:
            self.reconnect_instance('XCAP_SAM_N', [('top', 'ra_out_n'), ('bot', 'sam_n')])
            self.reconnect_instance('XCAP_SAM_P', [('top', 'ra_out_p'), ('bot', 'sam_p')])
        else:
            self.reconnect_instance('XCAP_SAM_N', [('top', 'ra_out_n'), ('bot', 'VSS')])
            self.reconnect_instance('XCAP_SAM_P', [('top', 'ra_out_p'), ('bot', 'VSS')])
            self.reconnect_instance('XSW_OUT', [('in_p', 'ra_out_n'), ('in_n', 'ra_out_p')])


