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
class bag_vco_adc__ra(Module):
    """Module for library bag_vco_adc cell ra.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'ra.yaml')))

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
            ra_params='',
            cap_gain_params='',
            cmfb_params='',
            bias_params='',
        )

    def design(self, ra_params, cmfb_params, bias_params, cap_gain_params) -> None:
        if cap_gain_params:
            self.instances['XCAP_GAIN_N'].design(**cap_gain_params)
            self.instances['XCAP_GAIN_P'].design(**cap_gain_params)
            gain_cap_term_list = []
            for pin in self.instances['XCAP_GAIN_N'].master.pins.keys():
                if 'bit' in pin:
                    gain_cap_term_list.append((pin, pin.replace('bit', 'ctrl_cdac_gain')))
                    self.rename_pin('ctrl_cdac_gain', pin.replace('bit', 'ctrl_cdac_gain'))
            self.reconnect_instance('XCAP_GAIN_N', gain_cap_term_list)
            self.reconnect_instance('XCAP_GAIN_P', gain_cap_term_list)
        else:
            self.remove_instance('XCAP_GAIN_N')
            self.remove_instance('XCAP_GAIN_P')
            self.remove_pin('ctrl_cdac_gain')

        # cmfb
        self.instances['XCORE'].design(**ra_params)
        # Bising
        nbits = bias_params['nbits']
        self.instances['XBIAS'].design(**bias_params)

        self.reconnect_instance_terminal('XBIAS', f'ctrl_biasn<{nbits-1}:0>', f'ctrl_biasn<{nbits-1}:0>')
        self.reconnect_instance_terminal('XBIAS', f'ctrl_biasp<{nbits-1}:0>', f'ctrl_biasp<{nbits-1}:0>')

        self.rename_pin('ctrl_biasp', f'ctrl_biasp<{nbits-1}:0>')
        self.rename_pin('ctrl_biasn', f'ctrl_biasn<{nbits-1}:0>')

        # Core
        self.instances['XCMFB'].design(**cmfb_params)
        core_term_list = []
        for pin in self.instances['XCORE'].master.pins.keys():
            core_term_list.append((pin, pin))

        self.reconnect_instance('XCORE', core_term_list)
        ctrl_fb_pinname = [pin for pin in self.instances['XCORE'].master.pins.keys() if 'ctrl_fb_cap' in pin][0]
        self.rename_pin('ctrl_fb_cap', ctrl_fb_pinname)

        for pin in self.instances['XCORE'].master.pins.keys():
            if 'ctrl_dac_n' in pin:
                self.reconnect_instance_terminal('XCORE', pin, pin)
                self.rename_pin('ctrl_dac_n', pin)
            if 'ctrl_dac_p' in pin:
                self.reconnect_instance_terminal('XCORE', pin, pin)
                self.rename_pin('ctrl_dac_p', pin)



