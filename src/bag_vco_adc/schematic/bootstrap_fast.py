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

from typing import Dict, List, Tuple, Any, Mapping

import warnings
import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag_vco_adc__bootstrap_fast(Module):
    """Module for library bag_vco_adc cell bootstrap_fast.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__, str(Path('netlist_info', 'bootstrap_fast.yaml')))

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
            lch='channel length of transistors',
            intent='device intent',
            dev_info='devices information including nf, w and stack',
            dum_info='dummy information including nf, w and stack',
            cap_params='capacitor parameters',
            cap_aux_params='capacitor parameters',
            fast_on='True to turn-on XON_N fast',
            break_outputs='True to break output signals',
            mid_to_vg2='Connect mid to vg2 node',
            no_sampler='True to remove sampler',
            swap_inout='True to swap inout, used in cm switches',
            dummy_off='True to include dummy off dev',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            dum_info=[],
            cap_aux_params=None,
            cap_params=None,
            break_outputs=True,
            fast_on=True,
            mid_to_vg2=False,
            no_sampler=False,
            swap_inout=False,
            dummy_off=False,
        )

    def design(self, lch: int, intent: str, dev_info: Dict[str, Any], dum_info: List[Tuple[Any]],
               cap_params: Mapping[str, Any], cap_aux_params: Mapping[str, Any], fast_on: bool,
               break_outputs: bool, mid_to_vg2: bool, no_sampler: bool, swap_inout: bool, dummy_off: bool) -> None:

        #         warnings.warn("Doesn't implement dummhy samlping sw")
        if no_sampler:
            self.delete_instance('XSAM')
            self.remove_pin('out')
        else:
            if not dev_info['XSAM'].get('seg_n_dum'):
                self.remove_pin('in_c')
            nouts = len(dev_info['XSAM']['m_list'])
            self.instances['XSAM'].design(**dev_info['XSAM'])
            if nouts > 1:
                self.reconnect_instance('XSAM', [(f'out<{nouts-1}:0>', f'out<{nouts-1}:0>')])
                self.rename_pin('out', f'out<{nouts-1}:0>')

        if not fast_on:
            self.delete_instance('XINV_P_VG2')
            self.delete_instance('XINV_N0_VG2')
            self.delete_instance('XINV_N1_VG2')
            self.reconnect_instance_terminal('XON_N', 'G', 'vg')
            self.remove_pin('vg2')

        if 'XSAMPLE_INVP<1>' not in dev_info.keys():
            [self.remove_instance(instname) for instname in ['XSAMPLE_INVP<1>', 'XSAMPLE_INVN<1>',
                                                             'XSAMPLE_INVP<0>', 'XSAMPLE_INVN<0>', 'XINV_N_BUF']]
            # self.reconnect_instance_terminal('XINV_N', 'G', 'sample')
            # self.reconnect_instance_terminal('XINV_N', 'S', 'cap_bot')
        if 'XPRE' not in dev_info.keys():
            self.remove_instance('XPRE')

        if 'XCAP_P_AUX' not in dev_info.keys():
            self.remove_pin('cap_top_aux')
            self.delete_instance('XCAP_P_AUX')
            self.reconnect_instance_terminal('XCAP_P', 'B', 'cap_top')
            self.reconnect_instance_terminal('XON_P', 'B', 'cap_top')
            if fast_on:
                self.reconnect_instance_terminal('XINV_P_VG2', 'B', 'cap_top')
                self.reconnect_instance_terminal('XINV_P_VG2', 'S', 'cap_top')

        dev_tx_info = dev_info.copy(remove=['XSAM'])
        for key, _info in dev_tx_info.items():
            _w = _info.get('w', 4)
            _stack = _info.get('stack', 1)
            _intent = _info.get('intent', intent)
            self.instances[key].design(l=lch, intent=_intent, w=_w, nf=_info['nf'], stack=_stack)
        if dum_info:
            self.design_dummy_transistors(dum_info, 'X_DUMMY', 'VDD', 'VSS')
        else:
            self.delete_instance('X_DUMMY')

        if cap_params is None:
            self.delete_instance('X_CBOOT')
        else:
            self.instances['X_CBOOT'].design(**cap_params)
        if cap_aux_params is None:
            self.delete_instance('X_CAUX')
        else:
            self.instances['X_CAUX'].design(**cap_params)

        if mid_to_vg2:
            self.reconnect_instance_terminal('XMID', 'G', 'vg2')

        if swap_inout:
            self.reconnect_instance('XSAM', [('in', 'out'), ('out', 'in')])
            self.reconnect_instance_terminal('XON_N', 'D', 'out')

        if not dummy_off:
            self.remove_instance('XOFF_DUM0')
            self.remove_instance('XOFF_DUM1')
            self.remove_pin('dummy_off')

