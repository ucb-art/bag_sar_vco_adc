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

from typing import Any, Dict, Type, Optional, Mapping, Union, Tuple

from bag.design.database import ModuleDB, Module
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager, WireArray
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from bag.util.importlib import import_class
from bag.util.math import HalfInt
from bag_vco_adc.layout.sah.sar_samp import Sampler
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orientation, Direction, Orient2D
from xbase.layout.mos.base import MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo
from .sar_cdac import CapDacColCore
from .sar_logic import SARLogicArray
from ..util.template import TrackIDZL as TrackID, TemplateBaseZL
from ..util.wrapper import GenericWrapper, IntegrationWrapper

"""
This class implement a version with comparator at bottom
"""

class SARSliceRev(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None

    @property
    def tr_manager(self):
        return self._tr_manager

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_slice_wsamp' if self.params['sampler_params'] else
        'sar_slice_bot_rev')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            comp_params='Tri-tail comparator parameters.',
            logic_params='SAR logic parameters',
            cdac_params='Comparator DAC parameters',
            sampler_params='Sampler parameters',
            logic_buf_params='Logic buffer parameters',
            tr_widths='track width dictionary',
            tr_spaces='track space dictionary',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(sampler_params=[])

    def draw_layout(self) -> None:
        cdac_params: Param = self.params['cdac_params']
        comp_params: Param = self.params['comp_params']
        logic_params: Param = self.params['logic_params']
        sampler_params: Param = self.params['sampler_params']
        logic_buf_params: Param = self.params['logic_buf_params']

        if isinstance(cdac_params, str):
            cdac_yaml = read_yaml(cdac_params)
            cdac_params = cdac_yaml['params']
        if isinstance(comp_params, str):
            comp_yaml = read_yaml(comp_params)
            comp_params = comp_yaml['params']
            comp_params = ImmutableSortedDict(comp_params)
        if isinstance(logic_params, str):
            logic_yaml = read_yaml(logic_params)
            logic_params = logic_yaml['params']
            logic_params = ImmutableSortedDict(logic_params)
        if isinstance(logic_buf_params, str):
            logic_buf_yaml = read_yaml(logic_buf_params)
            logic_buf_class = logic_buf_yaml['lay_class']
            logic_buf_params = logic_buf_yaml['params']
            logic_buf_params = ImmutableSortedDict(logic_buf_params)

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        self._tr_manager = tr_manager

        nbits = cdac_params['nbits']
        pipeline_sar = True
        if not pipeline_sar:
            nbits += 1

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      comp_params['params']['params']['pinfo']['tile_specs'][
                                                          'arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        ################################
        # Preparing templates
        ################################
        cdac_master: CapDacColCore = self.new_template(CapDacColCore, params=cdac_params)
        comp_master_dummy: MOSBase = self.new_template(IntegrationWrapper, params=comp_params)
        logic_master_dummy: MOSBase = self.new_template(IntegrationWrapper, params=logic_params)
        comp_ncols_tot = 2 * (cdac_master.actual_width // comp_master_dummy.core.core.sd_pitch)

        logic_master: TemplateBase = self.new_template(IntegrationWrapper, params=logic_params)
        logic_buf_master: IntegrationWrapper = self.new_template(IntegrationWrapper, params=logic_buf_params)

        cdac_actual_width = cdac_master.actual_width
        logic_buf_width = logic_buf_master.bound_box.w
        comp_bnd_x = (comp_master_dummy.core.bound_box.w - comp_master_dummy.core.core.bound_box.w) // 2

        top_layer = max(logic_master_dummy.top_layer, comp_master_dummy.top_layer, cdac_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer, half_blk_y=True, half_blk_x=True)
        w_comp_blk, h_comp_blk = self.grid.get_block_size(max(comp_master_dummy.top_layer, logic_master.top_layer),
                                                          half_blk_y=True, half_blk_x=True)
        # Calculate comparator width, filled with tap and supply routing
        w_blk_in_sd_picth = w_blk // comp_master_dummy.core.core.sd_pitch
        comp_w = max(2 * (cdac_actual_width - logic_buf_width - comp_bnd_x), comp_master_dummy.bound_box.w)
        ncols_tot_logic = comp_w // comp_master_dummy.core.core.sd_pitch
        ncols_tot = ncols_tot_logic // 2
        ncols_tot += ncols_tot & 1
        ncols_tot = ncols_tot * 2

        logic_new_params = logic_params['params'].to_dict()
        logic_new_params['ncols_tot'] = ncols_tot_logic
        logic_gen_params = dict(
            cls_name=logic_params['cls_name'],
            top_sup_layer=comp_params['top_sup_layer'],
            params=logic_new_params
        )

        comp_core_params = comp_params['params']['params'].to_dict()
        comp_core_params['ncols_tot'] = comp_ncols_tot
        comp_gen_params = dict(
            cls_name=comp_params['cls_name'],
            top_sup_layer=comp_params['top_sup_layer'],
            params=dict(
                cls_name=comp_params['params']['cls_name'],
                export_private=comp_params['params']['export_private'],
                params=comp_core_params
            )
        )
        comp_master: IntegrationWrapper = self.new_template(IntegrationWrapper, params=comp_gen_params)
        logic_master: IntegrationWrapper = self.new_template(IntegrationWrapper, params=logic_gen_params)

        # Connect digital signals
        # If CDAC has pmos switch, connect logic signal dn/dp together
        sw_type = cdac_params.get('sw_type', ['n', 'n', 'n'])

        ################################
        # floorplanning
        ################################
        w_logic, h_logic = logic_master.bound_box.w, logic_master.bound_box.h
        w_comp, h_comp = comp_master.bound_box.w, comp_master.bound_box.h
        w_dac, h_dac = cdac_master.bound_box.w, cdac_master.bound_box.h

        # Calculate logic signal routing
        comp_out_type_list = ['dum'] + ['clk']
        type_list = ['dig'] * nbits * 3
        type_list += comp_out_type_list

        num_dig_tr, _ = tr_manager.place_wires(xm_layer, type_list, align_idx=0)
        coord_dig_tr = self.grid.track_to_coord(xm_layer, num_dig_tr)

        w_tot = max(w_logic, w_comp, 2 * w_dac)
        w_tot = -(-w_tot // w_blk // 2) * w_blk * 2
        h_tot = -(-h_comp // h_blk) * h_blk

        comp_y = h_tot
        h_tot += coord_dig_tr
        logic_x = -(-(w_tot - w_logic) // 2 // w_comp_blk) * w_comp_blk
        logic_y = -(-h_tot // h_blk) * h_blk
        comp_x = -(-(w_tot - w_comp) // 2 // w_comp_blk) * w_comp_blk
        dac_x = -(-(w_tot - 2 * w_dac) // 2 // w_blk) * w_blk

        num_cap_top_tr, _ = tr_manager.place_wires(xm_layer, ['cap'] * 3)
        coord_cap_top_tr = self.grid.track_to_coord(xm_layer, num_cap_top_tr)
        cap_y = -(-(logic_y + h_logic + coord_cap_top_tr) // h_blk) * h_blk
        h_tot = -(-cap_y // h_blk) * h_blk + h_dac

        comp = self.add_instance(comp_master, inst_name='XCOMP', xform=Transform(comp_x, comp_y, mode=Orientation.MX))
        logic = self.add_instance(logic_master, inst_name='XLOGIC', xform=Transform(logic_x, logic_y))
        cdac_n = self.add_instance(cdac_master, inst_name='XDAC_N', xform=Transform(dac_x, cap_y, mode=Orientation.R0))
        cdac_p = self.add_instance(cdac_master, inst_name='XDAC_P',
                                   xform=Transform(w_tot - dac_x, cap_y, mode=Orientation.MY))

        comp_buf_top_layer = max(comp_master.top_layer, logic_buf_master.top_layer)
        comp_buf_top_layer_dir = self.grid.get_direction(comp_buf_top_layer)
        comp_buf_top_layer = comp_buf_top_layer if comp_buf_top_layer_dir == Orient2D.x else comp_buf_top_layer - 1
        tr_w_sp_comp_buf_top = tr_manager.get_width(comp_buf_top_layer, 'sup')
        if tr_w_sp_comp_buf_top > 0:
            tr_w_sp_comp_buf_top = tr_w_sp_comp_buf_top
        else:
            tr_w_sp_comp_buf_top = TrackID(comp_buf_top_layer, 0, tr_w_sp_comp_buf_top, grid=self.grid).width
        comp_buf_sp = self.grid.get_line_end_space(comp_buf_top_layer, tr_w_sp_comp_buf_top)
        buf_y = logic.bound_box.yh
        # buf_y = logic_y
        buf_y = - (-buf_y // h_blk) * h_blk
        buf_n = self.add_instance(logic_buf_master, inst_name='XBUF_N',
                                  xform=Transform(logic.bound_box.xl-comp_buf_sp, buf_y, mode=Orientation.R180))
        buf_x = -(-(logic.bound_box.xh) // w_comp_blk) * w_comp_blk
        buf_p = self.add_instance(logic_buf_master, inst_name='XBUF_P',
                                  xform=Transform(buf_x+comp_buf_sp, buf_y, mode=Orientation.MX))
        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')
        sampler_n_xm_list, sampler_p_xm_list = [], []
        ################################
        # Add sampler if have params
        ################################
        if sampler_params:
            if isinstance(sampler_params, str):
                sampler_yaml = read_yaml(sampler_params)
                sampler_params = sampler_yaml['params']
                cls_name = import_class(sampler_yaml['lay_class'])
                sampler_master: TemplateBase = self.new_template(cls_name, params=sampler_params)

            else:
                cls_name = Sampler.get_qualified_name()
                sar_gen_params = dict(
                    cls_name=cls_name,
                    params=sampler_params
                )
                sampler_master: TemplateBase = self.new_template(GenericWrapper, params=sar_gen_params)
            w_sam, h_sam = sampler_master.bound_box.w, sampler_master.bound_box.h
            # calculate space for cap bot routing
            w_blk, h_blk = self.grid.get_block_size(sampler_master.top_layer, half_blk_x=True, half_blk_y=True)
            sam_y = -(-h_tot // h_blk) * h_blk
            sam_bot_xm_tidx = self.grid.coord_to_track(xm1_layer, sam_y, mode=RoundMode.NEAREST)
            tr_w_samp_sig_xm1 = tr_manager.get_width(xm1_layer, 'samp_sig')
            tr_samp_sig_sep = self.get_track_sep(xm1_layer, 1, tr_w_samp_sig_xm1)
            sam_bot_locs = sam_bot_xm_tidx + tr_samp_sig_sep
            coord_sam_tr = self.grid.track_to_coord(xm1_layer, 2 * tr_samp_sig_sep)

            sam_y = -(-(h_tot + coord_sam_tr) // h_blk) * h_blk
            sam_x = -(-(w_tot // 2 - w_sam // 2) // w_blk) * w_blk
            sampler = self.add_instance(sampler_master, inst_name='XSAM',
                                        xform=Transform(sam_x, sam_y, mode=Orientation.R0))
            vg_n_xm1 = self.connect_to_tracks(sampler.get_all_port_pins('vg_n_ym'),
                                              TrackID(xm1_layer, sam_bot_locs, tr_w_samp_sig_xm1, grid=self.grid))
            vg_p_xm1 = self.connect_to_tracks(sampler.get_all_port_pins('vg_p_ym'),
                                              TrackID(xm1_layer, sam_bot_locs, tr_w_samp_sig_xm1, grid=self.grid))
            _, sam_bot_locs = tr_manager.place_wires(xm1_layer, ['dum'] + ['sig'] * 2, align_idx=0,
                                                     align_track=sam_bot_xm_tidx)
            self.connect_differential_tracks([cdac_n.get_pin('in_c'), cdac_p.get_pin('in')],
                                             [cdac_n.get_pin('in'), cdac_p.get_pin('in_c')],
                                             xm1_layer, sam_bot_locs[-1], sam_bot_locs[-2],
                                             width=tr_manager.get_width(xm1_layer, 'sig'))
            w_tot = max(w_tot, sampler.bound_box.xl)
            h_tot = -(-sampler.bound_box.yh // h_blk) * h_blk
        else:
            sampler_master = None
            sig_n_xm1, sig_p_xm1, sig_p_c_xm1, sig_n_c_xm1, vg_n_xm1, vg_p_xm1 = None, None, None, None, None, None
            h_tot = -(-cdac_n.bound_box.yh // h_blk) * h_blk
            sampler = None

        w_blk, h_blk = self.grid.get_block_size(ym1_layer + 1)
        self.set_size_from_bound_box(ym1_layer + 1, BBox(0, 0, w_tot, h_tot), half_blk_x=False)

        sar_params = dict(
            nbits=nbits,
            comp=comp_master.sch_params,
            logic=logic_master.sch_params,
            cdac=cdac_master.sch_params,
            buf=logic_buf_master.sch_params,
            tri_sa=True,
        )
        #
        if sampler_params:
            self._sch_params = dict(
                slice_params=sar_params,
                sampler_params=sampler_master.sch_params,
                rev=True,
            )
        else:
            self._sch_params = sar_params

