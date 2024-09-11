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

import pkg_resources
from pathlib import Path
from typing import Dict, Any

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.util.immutable import Param


# noinspection PyPep8Naming
from pybag.enum import TermType


class bag_vco_adc__sar_slice_wsamp(Module):
    """Module for library bag_vco_adc cell sar_slice_wsamp.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'sar_slice_wsamp.yaml')))

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
            slice_params='',
            sampler_params='',
            rev='True to use the rev version'
        )

    def design(self, sampler_params, slice_params, rev=False) -> None:
        if rev:
            self.replace_instance_master('XSAR', 'bag_vco_adc', 'sar_slice_bot_rev', keep_connections=True)
        self.instances['XSAM'].design(**sampler_params)
        self.instances['XSAR'].design(**slice_params)
        sar_pins = list(self.instances['XSAR'].master.pins.keys())
        sam_pins = list(self.instances['XSAM'].master.pins.keys())
        sar_conn_list = [(p, p) for p in sar_pins]
        sam_conn_list = [(p, p) for p in sam_pins]
        sam_conn_list_new, sar_conn_list_new = [], []
        for pin, ppin in sam_conn_list:
            if 'out_n' in pin:
                ppin = 'top_n'
            if 'out_p' in pin:
                ppin = 'top_p'
            if 'sig' in pin:
                ppin = ppin.replace('sig', 'in')
            if 'vcm' in pin:
                ppin = 'vcm' if slice_params['split_dac'] else 'vref<1>'
            if 'sam' in pin:
                ppin = ppin.replace('sam', 'clk')
            sam_conn_list_new.append((pin, ppin))
        for pin, ppin in sar_conn_list:
            if 'sig' in pin:
                ppin = ppin.replace('sig', 'in')
            if 'sam' in pin:
                ppin = ppin.replace('sam', 'clk')
            sar_conn_list_new.append((pin, ppin))
        nbits = slice_params['nbits']
        sam_conn_list_new.append((f'out_n<{nbits}:0>', f'bot_n<{nbits-1}:0>,bot_n<{nbits}>'))
        sam_conn_list_new.append((f'out_p<{nbits}:0>', f'bot_p<{nbits-1}:0>,bot_p<{nbits}>'))

        self.reconnect_instance('XSAR', sar_conn_list_new)
        self.reconnect_instance('XSAM', sam_conn_list_new)
        if slice_params['split_dac']:
            self.remove_pin('dm')
            for pname in ['dn', 'dp', 'data_out']:
                self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")
            self.rename_pin('vref', 'vref<1:0>')
        else:
            for pname in ['dm', 'dn', 'dp', 'data_out']:
                self.rename_pin(pname, f"{pname}<{nbits - 1}:0>")
            self.rename_pin('vref', 'vref<2:0>')
            self.remove_pin('vcm')

        for pin in ['bot_n', 'bot_p']:
            self.remove_pin(pin)
