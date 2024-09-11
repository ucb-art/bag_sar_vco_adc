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

import copy
import random
import string

import datetime
import matplotlib.pyplot as plt
import operator
import os
from functools import reduce
from typing import Mapping, Dict, Any, Tuple, Optional, Type, List

# from bag.env import get_tech_global_info
from bag.concurrent.util import GatherHelper
from bag.design.database import Module
from bag.io.file import read_yaml, yaml
from bag.layout.template import TemplateBase
from bag.simulation.design import DesignerBase, DesignInstance
from ..layout.bootstrap import BootstrapDiff
from ..measurement.sah import SampleHoldMM
from ..schematic.bootstrap_diff import bag_vco_adc__bootstrap_diff


class SampleHoldDesigner(DesignerBase):
    """
    Currently this design script only instantiate SampleHoldMM,
    and sweep variables/design other than signal frequency
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        DesignerBase.__init__(self, *args, **kwargs)

    @classmethod
    def get_dut_sch_class(cls) -> Optional[Type[Module]]:
        return bag_vco_adc__bootstrap_diff

    @classmethod
    def get_dut_lay_class(cls) -> Optional[Type[TemplateBase]]:
        return BootstrapDiff

    async def async_design(self, sim_params: Mapping[str, Any], sim_env_info: Mapping[str, Any],
                           pinfo: Optional[Dict[str, Any]] = None,
                           **kwargs: Mapping[str, Any]) -> Mapping[str, Any]:

        gen_specs: Optional[Mapping[str, Any]] = kwargs.get('gen_cell_specs', None)
        # gen_cell_args: Optional[Mapping[str, Any]] = kwargs.get('gen_cell_args', None)
        dut_swp_params: Optional[List[Dict[str, Any]]] = kwargs.get('dut_swp_params', None)
        swp_input_freq: bool = kwargs.get('swp_input_freq', False)
        gen_specs_file: str = kwargs.get('gen_specs_file', '')
        if gen_specs_file:
            gen_specs: Mapping[str, Any] = read_yaml(gen_specs_file)
            dut_params: Mapping[str, Any] = gen_specs['params']
        else:

            dut_params: Optional[Dict[str, Any]] = kwargs.get('dut_params', None)

        helper = GatherHelper()
        sim_params_dict = dict()
        swp_params_list = []
        for k, v in sim_params.items():
            if isinstance(v, List):
                swp_params_list.append({k: v})
            else:
                sim_params_dict.update({k: v})

        cls_name = self.get_dut_lay_class() if self.get_dut_lay_class() else self.get_dut_sch_class()
        dut = await self.async_new_dut("bootstrap", cls_name, dut_params)
        if dut_swp_params and swp_params_list:
            raise NotImplementedError("Doesn't support both dut sweep and design parameter sweep")
        elif swp_params_list:
            self.swp_var_comb(swp_params_list, sim_params_dict, helper, dut,
                              sim_env_info, swp_input_freq)
        elif dut_swp_params:
            dut_params_list = self.swp_dut_comb(dut_swp_params, dut_params)
            dut_specs_list = [dict(impl_cell='bootstrap' + _id, dut_cls=cls_name, dut_params=_params)
                              for (_id, _params) in dut_params_list]
            dut_list = await self.async_batch_dut(dut_specs_list)
            for _dut in dut_list:
                helper.append(self._sim_and_check_specs(_dut, sim_params_dict, sim_env_info,
                                                        swp_input_freq, _dut.cell_name))
        else:
            helper.append(self._sim_and_check_specs(dut, sim_params_dict,
                                                    sim_env_info, swp_input_freq,
                                                    sim_id=sim_params.get('meas_name', '')))

        results = await helper.gather_err()
        # results.sort(reverse=True, key=lambda x: x[1]['sfdr'][-1])
        for idx in range(10):
            print(max(results, key=lambda x: x[1]['sfdr'][idx])[0])
        plot_len = min(5, len(results))
        f, ax = plt.subplots(1)
        # for idx, (legend, data) in enumerate(results[:plot_len]):
        #     SampleHoldMM.plot_result(data['sfdr'], data['sndr'], data['freq'], legend, idx, ax)
        results.sort(reverse=True, key=lambda x: x[1]['sfdr'][5])
        result_dict = dict()
        for name, res in results:
            sfdr_list = [float(d) for d in res['sfdr']]
            sndr_list = [float(d) for d in res['sndr']]
            freq_list = [float(d) for d in res['freq']]
            result_dict[name] = dict(
                sfdr=sfdr_list,
                freq=freq_list,
                sndr=sndr_list,
            )
        if kwargs.get('save_result', False):
            suffix = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
            cur_path = os.getcwd()
            dsn_param_file = 'btstrp_dsn/dsn_dut_params'
            result_file = 'btstrp_dsn/dsn_result'
            letters = string.ascii_lowercase
            dsn_dut_dict = dut_params.copy()
            if swp_params_list:
                dsn_dut_dict.update(dict=swp_params_list)
            if dut_swp_params:
                dsn_dut_dict.update(dict=dut_swp_params)
            rndm_name = ''.join(random.choice(letters) for i in range(10))
            with open(f'{cur_path}/{dsn_param_file}_{suffix}{rndm_name}', 'w+') as file:
                yaml.dump(dsn_dut_dict, file)
            with open(f'{cur_path}/{result_file}_{suffix}{rndm_name}', 'w+') as file:
                yaml.dump(result_dict, file)

        for idx, (legend, data) in enumerate(results[:plot_len]):
            SampleHoldMM.plot_result(data['sfdr'], data['sndr'], data['freq'], legend, idx, ax)
            # print(legend)
            # print(data['sfdr'])
            # print(data['sndr'])
            print(data['freq'])
        ax.grid()
        plt.show()
        gen_params = dict(
            # lay_class=cls_name,
            **gen_specs,
            # params=dut_params,
        )
        return gen_params

    async def _sim_and_check_specs(self, dut, sim_params, sim_env_info, swp_input_freq, sim_id=''):
        mm_params = self._build_meas_params(sim_params, sim_env_info, dut, swp_input_freq)
        mm = self.make_mm(SampleHoldMM, mm_params)
        sim_id += '_sweep_freq' if swp_input_freq else '_single_freq'
        sim_id.replace('.', 'p')
        sim_result = await self.async_simulate_mm_obj(sim_id, dut, mm)
        return sim_id, sim_result.data

    def swp_var_comb(self, swp_list: List[Dict], params_dict: Dict, _gather,
                     _dut, _sim_env_info, _swp_input_freq, sim_id=''):

        swp_var_name = list(swp_list[0].keys())[0]
        swp_val = swp_list[0][swp_var_name]
        down = len(swp_list) > 1
        for val in swp_val:
            _params = copy.deepcopy(params_dict)
            _params.update({swp_var_name: val})
            if isinstance(val, List):
                _sim_id = sim_id + swp_var_name + '_'.join(val)
            else:
                _sim_id = sim_id + swp_var_name + str(val)
            if down:
                self.swp_var_comb(swp_list[1:], _params, _gather, _dut,
                                  _sim_env_info, _swp_input_freq, _sim_id)
            else:
                _gather.append(self._sim_and_check_specs(_dut, _params, _sim_env_info,
                                                         _swp_input_freq, _sim_id))

    def swp_dut_comb(self, swp_list: List[Dict[str, Any]], dut_params: Dict[str, Any], sim_id='') -> List[Tuple]:
        res_list = []
        down = len(swp_list) > 1
        for val in swp_list[0]['val']:
            _params = copy.deepcopy(dut_params)
            _sim_id = ''
            if not isinstance(swp_list[0]['key'], List):
                swp_list[0]['key'] = [swp_list[0]['key']]
            for swp_var_name in swp_list[0]['key']:
                swp_var_name = swp_var_name.split('.')
                reduce(operator.getitem, swp_var_name[:-1], _params)[swp_var_name[-1]] = val
                if isinstance(val, List):
                    _sim_id = sim_id + f"{swp_var_name[-1]}_" + '_'.join([str(v) for v in val])
                else:
                    _sim_id = sim_id + f"{swp_var_name[-1]}_{str(val)}"
            if down:
                res_list.extend(self.swp_dut_comb(swp_list[1:], _params, _sim_id))
            else:
                res_list.append((_sim_id, _params))
        return res_list

    def _build_meas_params(self, sim_params: Mapping[str, Any], sim_env_info: Mapping[str, Any],
                           dut: DesignInstance, swp_input_freq: bool) -> Dict[str, Any]:
        # default sim parameters and variables
        num_sample = 512
        default_num_sig_list = [idx + 0.5 for idx in range(10)]
        sim_params = dict(
            t_rf=20.0e-12,
            t_settle=100.0e-9,
            num_sample=512,
            num_sig=2.5 * num_sample - 1,
            freq_sig='num_sig/num_sample*freq_sample',
            t_sim='t_settle+(num_sample+1)/freq_sample',
            **sim_params
        )

        dut_pins = dut.sch_master.pins.keys()
        save_pins = [k for k in dut_pins if 'in' in k or 'out' in k]
        load_list = []
        for k in dut_pins:
            if 'out' in k:
                load_list.append(dict(pin=k, type='cap', value='c_load'))
            elif k.startswith('cap_top'):
                _cap_value = 'c_aux' if 'aux' in k else 'c_boot'
                _replace_key = 'top_aux' if 'aux' in k else 'top'
                _cap_bot = k.replace(_replace_key, 'bot')
                if _cap_bot in dut_pins:
                    load_list.append(dict(pin=k, type='cap', value=_cap_value, gnd=_cap_bot))
                else:
                    raise ValueError("Is this bootstrap sampler? "
                                     "Check both cap_top and cap_bot pins in dut")

        stimuli_list = [
            dict(src_type='vpulse', pin='sample',
                 table=dict(
                     td='t_settle', per='1/freq_sample',
                     pw='1/(num_channel*freq_sample)-t_rf', v1='v_VSS', v2='v_VDD',
                     tr='t_rf', tf='t_rf',
                 )),
            dict(src_type='vpulse', pin='sample_b',
                 table=dict(
                     td='t_settle', per='1/freq_sample',
                     pw='1/(num_channel*freq_sample)-t_rf', v1='v_VDD', v2='v_VSS',
                     tr='t_rf', tf='t_rf'
                 )),
        ]
        if self.get_dut_lay_class is None:
            stimuli_list.append(
                dict(src_type='vpulse', pin='sample_b_delay',
                     table=dict(
                         td='t_settle+t_d_sample', per='1/freq_sample',
                         pw='1/(num_channel*freq_sample)-t_rf', v1='v_VDD', v2='v_VSS',
                         tr='t_rf', tf='t_rf',
                     ))
            )

        for k in ['sig', 'sig_p', 'sig_n']:
            sp = 180 if k == 'sig_n' and 'sig_p' in dut_pins else 0
            if k in dut_pins:
                stimuli_list.append(
                    dict(src_type='vsin', pin=k,
                         table=dict(vdc='v_vcm', va='v_amp', freq='freq_sig', sinephase=sp)
                         ))

        save_pins.extend(['sample', 'sample_b']),
        for cap_var in ['c_boot', 'c_aux']:
            if cap_var not in sim_params:
                sim_params.update({cap_var: 0})

        mm_specs = dict(
            tbm_specs=dict(
                sim_params=sim_params,
                sim_envs=sim_env_info['sim_envs'],
                save_outputs=save_pins,
                tran_options=dict(
                    maxstep=1.0e-12,
                    errpreset='conservative',
                ),
                sup_values=dict(
                    VDD=sim_env_info['VDD'],
                    VSS=0.0,
                    vcm=sim_env_info['VDD'] / 2,
                ),
                stimuli_list=stimuli_list,
                load_list=load_list,
            )
        )

        if swp_input_freq:
            mm_specs['tbm_specs'].update(
                swp_info=dict(
                    num_sig=[idx * num_sample - 1 for idx in default_num_sig_list]
                )
            )
        return mm_specs


# def sort_result(res_name):
#     dsn_name, result = [], []
#     res_yaml = read_yaml(res_name)
#     for k, v in res_yaml.items():
#         dsn_name.append(k)
#         result.append(v)
#     max_idx =
