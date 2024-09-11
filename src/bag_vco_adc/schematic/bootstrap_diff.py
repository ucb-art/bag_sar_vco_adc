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

from typing import Dict, Any, Mapping

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__bootstrap_diff(Module):
    """Module for library bag_vco_adc cell bootstrap_diff.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'bootstrap_diff.yaml')))

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
            sampler_params='Sampler parameters',
            cdum='True to have xcp mos-cap between output and vg'
        )
    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(cdum=False)

    def design(self, sampler_params: Mapping[str, Any], cdum: bool):
        nouts = len(sampler_params['dev_info']['XSAM']['m_list'])
        if nouts > 1:
            self.reconnect_instance('XN', [(f'out_n<{nouts - 1}:0>', f'out_n<{nouts - 1}:0>')])
            self.reconnect_instance('XP', [(f'out_p<{nouts - 1}:0>', f'out_p<{nouts - 1}:0>')])
            self.rename_pin('out_p', f'out_p<{nouts-1}:0>')
            self.rename_pin('out_n', f'out_n<{nouts-1}:0>')
        self.reconnect_instance_terminal('XN', f'out<{nouts-1}:0>', f'out_n<{nouts-1}:0>')
        self.reconnect_instance_terminal('XP', f'out<{nouts-1}:0>', f'out_p<{nouts-1}:0>')

        self.instances['XN'].design(**sampler_params)
        self.instances['XP'].design(**sampler_params)
        if cdum:
            self.instances['XCDUM_N'].design(l=sampler_params['lch'], intent=sampler_params['intent'],
                                             nf=sampler_params['dev_info']['XSAMPLE']['nf']//4, w=4) #FIXME: get width

            self.instances['XCDUM_P'].design(l=sampler_params['lch'], intent=sampler_params['intent'],
                                             nf=sampler_params['dev_info']['XSAMPLE']['nf']//4, w=4)
        else:
            [self.delete_instance(inst) for inst in ['XCDUM_N', 'XCDUM_P']]

