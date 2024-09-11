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
class bag_vco_adc__ra_diff_core(Module):
    """Module for library bag_vco_adc cell ra_diff_core.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'ra_diff_core.yaml')))

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
        )

    def design(self, ra_params) -> None:
        self.instances['XRA_N'].design(**ra_params)
        self.instances['XRA_P'].design(**ra_params)
        dac_params = ra_params['dac_params']
        num_dacn = len(dac_params['seg_dacn_list'])
        num_dacp = len(dac_params['seg_dacp_list'])
        self.rename_pin('ctrl_dac_p', f'ctrl_dac_p<{num_dacp - 1}:0>')
        self.rename_pin('ctrl_dac_n', f'ctrl_dac_n<{num_dacn - 1}:0>')
        self.reconnect_instance_terminal('XRA_N', f'ctrl_dac_p<{num_dacp - 1}:0>', f'ctrl_dac_p<{num_dacp - 1}:0>')
        self.reconnect_instance_terminal('XRA_N', f'ctrl_dac_n<{num_dacn - 1}:0>', f'ctrl_dac_n<{num_dacn - 1}:0>')
        self.reconnect_instance_terminal('XRA_P', f'ctrl_dac_p<{num_dacp - 1}:0>', f'ctrl_dac_p<{num_dacp - 1}:0>')
        self.reconnect_instance_terminal('XRA_P', f'ctrl_dac_n<{num_dacn - 1}:0>', f'ctrl_dac_n<{num_dacn - 1}:0>')
