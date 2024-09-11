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
class bag_vco_adc__vco_cnter_async(Module):
    """Module for library bag_vco_adc cell vco_cnter_async.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'vco_cnter_async.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls):
        # type: () -> Dict[str, str]
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            div_params_list='list of divider parameters',
            ndivs='number of stage',
            nbits='number of bits in div'
        )

    def design(self, div_params_list, nbits, ndivs):

        # name_list = [f"XDIV<{num_stage-1}:0>"]
        name_list = [f"XDIV<{idx}>" for idx in range(ndivs)]

        div_stages = 2**nbits
        tot_out = div_stages * ndivs

        out_name = [f"outp<{div_stages*(idx-1)}:{div_stages*idx-1}>" for idx in range(1, ndivs+1)]
        out_b_name = [f"outn<{div_stages*(idx-1)}:{div_stages*idx-1}>" for idx in range(1, ndivs+1)]
        out_term_name = f"outp<0:{div_stages-1}>"
        out_b_term_name = f"outn<0:{div_stages-1}>"
        clk_name = ['clkp']+['outp<%d>' % (idx*div_stages-1) for idx in range(1, ndivs)]
        clk_b_name = ['clkn']+['outn<%d>' % (idx*div_stages-1) for idx in range(1, ndivs)]

        term_list = []
        for idx in range(ndivs):
            term_list.append({out_term_name: out_name[idx], out_b_term_name: out_b_name[idx],
                              'clkp': clk_name[idx], 'clkn': clk_b_name[idx]})

        self.array_instance('XDIV', name_list, term_list)
        for idx in range(ndivs):
            self.instances[f'XDIV<{idx}>'].design(**div_params_list[idx])
        self.rename_pin('out', f"outp<{tot_out-1}:0>")
        self.rename_pin('outn', f"outn<{tot_out-1}:0>")
