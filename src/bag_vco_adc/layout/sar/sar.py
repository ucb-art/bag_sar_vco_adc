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

from typing import Any, Dict, Type, Optional, Mapping, Union, Tuple, cast

from bag.design.database import ModuleDB, Module
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager, WireArray
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from bag.util.importlib import import_class
from bag.util.math import HalfInt
from bag_vco_adc.layout.sah.sar_samp import Sampler
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orientation, Direction, Orient2D, PinMode
from xbase.layout.mos.base import MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo
from .sar_cdac import CapDacColCore, CapDacSplitCore
from .sar_logic import SARLogicArray
from ..util.template import TrackIDZL as TrackID, TemplateBaseZL
from ..util.util import MOSBaseFiller, basefill_bbox
from ..util.wrapper import GenericWrapper, IntegrationWrapper


class SARSlice(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None

    @property
    def tr_manager(self):
        return self._tr_manager

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_slice_wsamp' if self.params['sampler_params'] else
        'sar_slice_bot')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            comp_params='Tri-tail comparator parameters.',
            logic_params='SAR logic parameters',
            cdac_params='Comparator DAC parameters',
            sampler_params='Sampler parameters',
            tr_widths='track width dictionary',
            tr_spaces='track space dictionary',
            split_dac='True to use split dac'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(sampler_params=[], split_dac=False)

    def draw_layout(self) -> None:
        comp_params: Param = self.params['comp_params']
        logic_params: Param = self.params['logic_params']
        cdac_params: Param = self.params['cdac_params']
        sampler_params: Param = self.params['sampler_params']

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

        cdac_master: CapDacColCore = self.new_template(CapDacColCore, params=cdac_params)
        comp_master_dummy: MOSBase = self.new_template(IntegrationWrapper, params=comp_params)
        logic_ncols_tot = 2 * (cdac_master.actual_width // comp_master_dummy.core.core.sd_pitch)

        logic_new_params = logic_params['params'].to_dict()
        logic_new_params['ncols_tot'] = logic_ncols_tot
        logic_gen_params = dict(
            cls_name=logic_params['cls_name'],
            top_sup_layer=comp_params['top_sup_layer'],
            params=logic_new_params
        )

        logic_master: TemplateBase = self.new_template(IntegrationWrapper, params=logic_gen_params)
        cdac_actual_width = cdac_master.actual_width

        top_layer = max(logic_master.top_layer, comp_master_dummy.top_layer, cdac_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer, half_blk_y=True, half_blk_x=True)
        w_comp_blk, h_comp_blk = self.grid.get_block_size(max(comp_master_dummy.top_layer, logic_master.top_layer),
                                                          half_blk_y=True, half_blk_x=True)
        w_blk_in_sd_picth = w_blk // comp_master_dummy.core.core.sd_pitch
        ncols_tot = 2 * cdac_actual_width // comp_master_dummy.core.core.sd_pitch
        ncols_tot = ncols_tot // (2 * w_blk_in_sd_picth) * (2 * w_blk_in_sd_picth)

        comp_core_params = comp_params['params']['params'].to_dict()
        comp_core_params['ncols_tot'] = ncols_tot
        comp_gen_params = dict(
            cls_name=comp_params['cls_name'],
            top_sup_layer=comp_params['top_sup_layer'],
            params=dict(
                cls_name=comp_params['params']['cls_name'],
                export_private=comp_params['params']['export_private'],
                params=comp_core_params
            )
        )
        comp_master: TemplateBase = self.new_template(IntegrationWrapper, params=comp_gen_params)

        # Connect digital signals
        # If CDAC has pmos switch, connect logic signal dn/dp together
        sw_type = cdac_params.get('sw_type', ['n', 'n', 'n'])
        has_pmos_sw = 'p' in sw_type

        # floorplanning
        w_logic, h_logic = logic_master.bound_box.w, logic_master.bound_box.h
        w_comp, h_comp = comp_master.bound_box.w, comp_master.bound_box.h
        w_dac, h_dac = cdac_master.bound_box.w, cdac_master.bound_box.h

        # Calculate logic signal routing
        lower_layer_routing = logic_master.sch_params['lower_layer_routing']
        comp_out_type_list = ['dum'] + ['sig'] * 2 + ['clk'] * 2 + ['sig'] * 2
        type_list = ['dig'] * nbits * 5 if has_pmos_sw else ['dig'] * nbits * 3
        type_list += comp_out_type_list

        if lower_layer_routing:
            type_list += type_list
        num_dig_tr, _ = tr_manager.place_wires(xm_layer, type_list, align_idx=0)
        coord_dig_tr = self.grid.track_to_coord(xm_layer, num_dig_tr)

        w_tot = max(w_logic, w_comp, 2 * w_dac)
        w_tot = -(-w_tot // w_blk // 2) * w_blk * 2
        h_tot = -(-h_logic // h_blk) * h_blk + h_comp + coord_dig_tr

        comp_y = -(-h_tot // h_blk) * h_blk

        logic_x = -(-(w_tot - w_logic) // 2 // w_comp_blk) * w_comp_blk
        comp_x = -(-(w_tot - w_comp) // 2 // w_comp_blk) * w_comp_blk
        dac_x = -(-(w_tot - 2 * w_dac) // 2 // w_blk) * w_blk

        num_cap_top_tr, _ = tr_manager.place_wires(xm_layer, ['cap'] * 3)
        coord_cap_top_tr = self.grid.track_to_coord(xm_layer, num_cap_top_tr)
        cap_y = -(-(comp_y + coord_cap_top_tr) // h_blk) * h_blk
        h_tot = -(-cap_y // h_blk) * h_blk + h_dac

        logic = self.add_instance(logic_master, inst_name='XLOGIC', xform=Transform(logic_x, 0))
        comp = self.add_instance(comp_master, inst_name='XCOMP', xform=Transform(comp_x, comp_y, mode=Orientation.MX))
        cdac_n = self.add_instance(cdac_master, inst_name='XDAC_N', xform=Transform(dac_x, cap_y, mode=Orientation.R0))
        cdac_p = self.add_instance(cdac_master, inst_name='XDAC_P',
                                   xform=Transform(w_tot - dac_x, cap_y, mode=Orientation.MY))

        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')
        sampler_n_xm_list, sampler_p_xm_list = [], []
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

            vcm_xm1_locs_middle = \
                self.get_available_tracks(xm1_layer,
                                          self.grid.coord_to_track(xm_layer, h_tot, RoundMode.NEAREST),
                                          self.grid.coord_to_track(xm_layer, sam_y, RoundMode.NEAREST),
                                          sampler.bound_box.xl, sampler.bound_box.xh,
                                          width=tr_manager.get_width(xm1_layer, 'sup'),
                                          sep=tr_manager.get_sep(xm1_layer, ('sup', 'sup')))[1:]

            tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
            tr_w_sup_top = tr_manager.get_width(ym1_layer + 1, 'sup')
            tr_sp_sup_top = tr_manager.get_sep(ym1_layer + 1, ('sup', 'sup'))
            wl, wh = self.grid.get_wire_bounds(ym1_layer + 1, 0, width=tr_w_sup_top)
            w_top = int((wh - wl) / 2)
            via_ext_ym, via_ext_top = self.grid.get_via_extensions(Direction.LOWER, ym1_layer, tr_w_sup_ym1,
                                                                   tr_w_sup_top)
            ym_dy = w_top + via_ext_top

            top_locs = self.get_available_tracks(ym1_layer + 1,
                                                 self.grid.coord_to_track(ym1_layer + 1, sampler.bound_box.yl + ym_dy
                                                                          , RoundMode.GREATER),
                                                 self.grid.coord_to_track(ym1_layer + 1, sampler.bound_box.yh - ym_dy,
                                                                          RoundMode.LESS_EQ),
                                                 lower=sampler.bound_box.xl, upper=sampler.bound_box.xh,
                                                 width=tr_w_sup_top, sep=tr_sp_sup_top, include_last=True)
            vcm_sam_ym1 = sampler.get_all_port_pins('vcm')
            vdd_sam_ym1 = sampler.get_all_port_pins('VDD')
            vss_sam_ym1 = sampler.get_all_port_pins('VSS')

        else:
            sampler_master = None
            sig_n_xm1, sig_p_xm1, sig_p_c_xm1, sig_n_c_xm1, vg_n_xm1, vg_p_xm1 = None, None, None, None, None, None
            h_tot = -(-cdac_n.bound_box.yh // h_blk) * h_blk
            sampler = None

        w_blk, h_blk = self.grid.get_block_size(ym1_layer + 1)
        w_tot = max(w_tot, sampler.bound_box.xl)
        h_tot = -(-sampler.bound_box.yh // h_blk) * h_blk
        ################################
        # Connect differential input to comp
        ################################

        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')
        dac_top_n, dac_top_p = cdac_n.get_pin('top'), cdac_p.get_pin('top')
        dac_top_tidx = self.grid.coord_to_track(xm_layer, (comp.bound_box.yh + cdac_n.bound_box.yl) // 2,
                                                mode=RoundMode.NEAREST)
        dac_n_xm = self.connect_to_tracks(dac_top_n, TrackID(xm_layer, dac_top_tidx, tr_w_cap_xm))
        dac_p_xm = self.connect_to_tracks(dac_top_p, TrackID(xm_layer, dac_top_tidx, tr_w_cap_xm))

        self.connect_to_track_wires(comp.get_all_port_pins('inp'), dac_p_xm)
        self.connect_to_track_wires(comp.get_all_port_pins('inn'), dac_n_xm)

        # Connect comp_out_m
        tr_w_dig_vm = tr_manager.get_width(vm_layer, 'dig')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        logic_top_hm_tidx = max(logic.get_all_port_pins('VDD_hm', layer=hm_layer),
                                key=lambda x: x.track_id.base_index).track_id.base_index
        logic_top_coord = self.grid.track_to_coord(hm_layer, logic_top_hm_tidx)

        logic_top_xm_tid = self.grid.coord_to_track(xm_layer, logic_top_coord, RoundMode.NEAREST)
        _, comp_out_locs = tr_manager.place_wires(xm_layer, comp_out_type_list, align_track=logic_top_xm_tid)
        comp_out_locs.pop(0)

        # Connect comp_out

        comp_p_m_xm, comp_n_m_xm = self.connect_differential_tracks(logic.get_pin('comp_p_m'),
                                                                    logic.get_pin('comp_n_m'), xm_layer,
                                                                    comp_out_locs[-2], comp_out_locs[-1],
                                                                    width=tr_w_sig_xm)

        comp_dir = Direction.LOWER if lower_layer_routing else Direction.UPPER
        logic_dir = Direction.LOWER if lower_layer_routing else Direction.UPPER

        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        logic_top_xm1_tid = self.grid.coord_to_track(xm1_layer, logic.bound_box.yh, RoundMode.NEAREST)
        _, comp_out_xm1_locs = tr_manager.place_wires(xm1_layer, ['dum', 'sig', 'sig'],
                                                      align_track=logic_top_xm1_tid)
        comp_out_xm1_locs.pop(0)
        logic_comp_n, logic_comp_p = self.connect_differential_tracks(logic.get_all_port_pins('comp_n'),
                                                                      logic.get_all_port_pins('comp_p'),
                                                                      xm1_layer, comp_out_locs[0], comp_out_locs[1],
                                                                      width=tr_w_sig_xm1)
        self.connect_differential_wires(comp.get_all_port_pins('outn'), comp.get_all_port_pins('outp'),
                                        logic_comp_n, logic_comp_p)
        self.connect_differential_wires(comp.get_all_port_pins('outn_m'), comp.get_all_port_pins('outp_m'),
                                        comp_n_m_xm, comp_p_m_xm)

        # List used to extend wire length
        comp_xm_list = []
        comp_m_xm_list = []

        self.connect_to_track_wires(logic.get_pin('comp_clk', layer=xm1_layer), comp.get_all_port_pins('clk'))
        # self.connect_to_track_wires(comp_clk_xm, comp.get_all_port_pins('clk_ym'))

        self.set_size_from_bound_box(ym1_layer + 1, BBox(0, 0, w_tot, h_tot), half_blk_x=False)

        sig_type_list = ['clk'] + ['dig'] * nbits * 5 if has_pmos_sw else ['clk'] + ['dig'] * nbits * 3

        sig_type_list = sig_type_list * 2 if lower_layer_routing else sig_type_list

        _, dig_tr_locs = tr_manager.place_wires(xm_layer, sig_type_list, align_idx=0, align_track=comp_out_locs[-1])
        tr_w_dig_xm = tr_manager.get_width(xm_layer, 'dig')
        dig_tr_locs.pop(0)
        dig_tr_locs_r = dig_tr_locs[::-1]
        if lower_layer_routing:
            rt_tidx_start = self.grid.coord_to_track(vm_layer, cdac_n.bound_box.xl)
            rt_tidx_stop = self.grid.coord_to_track(vm_layer, cdac_p.bound_box.xh)
            tr_w_vm_sig = tr_manager.get_width(vm_layer, 'dig')
            tr_sp_vm_sig = tr_manager.get_sep(vm_layer, ('dig', 'dig'))
            for idx in range(nbits - 1):
                dacp_n, dacp_p, dacp_m = \
                    cdac_p.get_pin(f'ctrl_n<{idx}>'), cdac_p.get_pin(f'ctrl_p<{idx}>'), cdac_p.get_pin(f'ctrl_m<{idx}>')
                dacn_n, dacn_p, dacn_m = \
                    cdac_n.get_pin(f'ctrl_n<{idx}>'), cdac_n.get_pin(f'ctrl_p<{idx}>'), cdac_n.get_pin(f'ctrl_m<{idx}>')
                if has_pmos_sw:
                    dn, dnb, dp, dpb, dm = logic.get_pin(f'dn<{idx + 1}>'), logic.get_pin(f'dn_b<{idx + 1}>'), \
                                           logic.get_pin(f'dp<{idx + 1}>'), logic.get_pin(f'dp_b<{idx + 1}>'), \
                                           logic.get_pin(f'dm<{idx + 1}>')
                    dac_p = self.connect_to_tracks([dacp_p], TrackID(xm_layer, dig_tr_locs_r[5 * idx], tr_w_dig_xm))
                    dac_pb = self.connect_to_tracks([dacp_n],
                                                    TrackID(xm_layer, dig_tr_locs_r[5 * idx + 1], tr_w_dig_xm))
                    dac_n = self.connect_to_tracks([dacn_p], TrackID(xm_layer, dig_tr_locs_r[5 * idx + 2], tr_w_dig_xm))
                    dac_nb = self.connect_to_tracks([dacn_n],
                                                    TrackID(xm_layer, dig_tr_locs_r[5 * idx + 3], tr_w_dig_xm))
                    dac_m = self.connect_to_tracks([dacp_m, dacn_m],
                                                   TrackID(xm_layer, dig_tr_locs_r[5 * idx + 4], tr_w_dig_xm))
                    dp = self.connect_bbox_to_tracks(logic_dir,
                                                     (logic.get_port(f'dp<{idx + 1}>').get_single_layer(), 'drawing'),
                                                     logic.get_pin(f'dp<{idx + 1}>'),
                                                     TrackID(xm_layer, dig_tr_locs[5 * idx], tr_w_dig_xm))
                    dnb = self.connect_bbox_to_tracks(logic_dir,
                                                      (
                                                          logic.get_port(f'dn_b<{idx + 1}>').get_single_layer(),
                                                          'drawing'),
                                                      logic.get_pin(f'dn_b<{idx + 1}>'),
                                                      TrackID(xm_layer, dig_tr_locs[5 * idx + 1], tr_w_dig_xm))
                    dn = self.connect_bbox_to_tracks(logic_dir,
                                                     (logic.get_port(f'dn<{idx + 1}>').get_single_layer(), 'drawing'),
                                                     logic.get_pin(f'dn<{idx + 1}>'),
                                                     TrackID(xm_layer, dig_tr_locs[5 * idx + 2], tr_w_dig_xm))
                    dpb = self.connect_bbox_to_tracks(logic_dir,
                                                      (
                                                          logic.get_port(f'dp_b<{idx + 1}>').get_single_layer(),
                                                          'drawing'),
                                                      logic.get_pin(f'dp_b<{idx + 1}>'),
                                                      TrackID(xm_layer, dig_tr_locs[5 * idx + 3], tr_w_dig_xm))
                    dm = self.connect_bbox_to_tracks(logic_dir,
                                                     (logic.get_port(f'dm<{idx + 1}>').get_single_layer(), 'drawing'),
                                                     logic.get_pin(f'dm<{idx + 1}>'),
                                                     TrackID(xm_layer, dig_tr_locs[5 * idx + 4], tr_w_dig_xm))
                    rt_tidx_list = self.get_available_tracks(vm_layer, rt_tidx_start, rt_tidx_stop,
                                                             logic.bound_box.yh,
                                                             comp.bound_box.yl, width=tr_w_vm_sig, sep=tr_sp_vm_sig)
                    rt_tidx_coord_list = [self.grid.track_to_coord(vm_layer, x) for x in rt_tidx_list]
                    mid_coord = (self.bound_box.xl + self.bound_box.xh) // 2
                    for _d, _c in zip([dn, dp, dm, dpb, dnb], [dac_n, dac_p, dac_m, dac_nb, dac_pb]):
                        _y_wire = self.connect_to_tracks([_d, _c], TrackID(vm_layer,
                                                                           SARLogicArray.get_nearest_tidx(_d,
                                                                                                          rt_tidx_list,
                                                                                                          rt_tidx_coord_list,
                                                                                                          mid_coord),
                                                                           tr_w_vm_sig))
                else:
                    dn, dp, dm = logic.get_pin(f'dn<{idx + 1}>'), logic.get_pin(f'dp<{idx + 1}>'), logic.get_pin(
                        f'dm<{idx + 1}>')
                    dac_p = self.connect_to_tracks([dacp_p, dacn_n],
                                                   TrackID(xm_layer, dig_tr_locs[3 * idx], tr_w_dig_xm))
                    dac_n = self.connect_to_tracks([dacp_n, dacn_p],
                                                   TrackID(xm_layer, dig_tr_locs[3 * idx + 1], tr_w_dig_xm))
                    dac_m = self.connect_to_tracks([dacp_m, dacn_m],
                                                   TrackID(xm_layer, dig_tr_locs[3 * idx + 2], tr_w_dig_xm))
                    self.connect_to_track_wires(logic.get_pin(f'dp<{idx + 1}>'), dac_p)
                    self.connect_to_track_wires(logic.get_pin(f'dn<{idx + 1}>'), dac_n)
                    self.connect_to_track_wires(logic.get_pin(f'dm<{idx + 1}>'), dac_m)
        else:
            for idx in range(nbits):
                dacp_n, dacp_p, dacp_m = \
                    cdac_p.get_pin(f'ctrl_n<{idx}>'), cdac_p.get_pin(f'ctrl_p<{idx}>'), cdac_p.get_pin(
                        f'ctrl_m<{idx}>')
                dacn_n, dacn_p, dacn_m = \
                    cdac_n.get_pin(f'ctrl_n<{idx}>'), cdac_n.get_pin(f'ctrl_p<{idx}>'), cdac_n.get_pin(
                        f'ctrl_m<{idx}>')
                if has_pmos_sw:
                    dac_p = self.connect_to_tracks([dacp_p], TrackID(xm_layer, dig_tr_locs[5 * idx], tr_w_dig_xm))
                    dac_pb = self.connect_to_tracks([dacp_n], TrackID(xm_layer, dig_tr_locs[5 * idx + 1], tr_w_dig_xm))
                    dac_n = self.connect_to_tracks([dacn_p], TrackID(xm_layer, dig_tr_locs[5 * idx + 2], tr_w_dig_xm))
                    dac_nb = self.connect_to_tracks([dacn_n], TrackID(xm_layer, dig_tr_locs[5 * idx + 3], tr_w_dig_xm))
                    dac_m = self.connect_to_tracks([dacp_m, dacn_m],
                                                   TrackID(xm_layer, dig_tr_locs[5 * idx + 4], tr_w_dig_xm))
                    self.connect_to_track_wires(logic.get_pin(f'dp<{idx}>'), dac_p)
                    self.connect_to_track_wires(logic.get_pin(f'dp_b<{idx}>'), dac_nb)
                    self.connect_to_track_wires(logic.get_pin(f'dn<{idx}>'), dac_n)
                    self.connect_to_track_wires(logic.get_pin(f'dn_b<{idx}>'), dac_pb)
                    self.connect_to_track_wires(logic.get_pin(f'dm<{idx}>'), dac_m)
                else:
                    dn, dp, dm = logic.get_pin(f'dn<{idx + 1}>'), logic.get_pin(f'dp<{idx + 1}>'), logic.get_pin(
                        f'dm<{idx + 1}>')
                    dac_p = self.connect_to_tracks([dacp_p, dacn_n],
                                                   TrackID(xm_layer, dig_tr_locs[3 * idx], tr_w_dig_xm))
                    dac_n = self.connect_to_tracks([dacp_n, dacn_p],
                                                   TrackID(xm_layer, dig_tr_locs[3 * idx + 1], tr_w_dig_xm))
                    dac_m = self.connect_to_tracks([dacp_m, dacn_m],
                                                   TrackID(xm_layer, dig_tr_locs[3 * idx + 2], tr_w_dig_xm))
                    self.connect_to_track_wires(logic.get_pin(f'dp<{idx}>'), dac_p)
                    self.connect_to_track_wires(logic.get_pin(f'dn<{idx}>'), dac_n)
                    self.connect_to_track_wires(logic.get_pin(f'dm<{idx}>'), dac_m)

        # --- export pins:
        for idx in range(nbits):
            self.reexport(logic.get_port(f'data_out<{idx}>'))
            self.reexport(logic.get_port(f'dn<{idx}>'))
            self.reexport(logic.get_port(f'dp<{idx}>'))
            self.reexport(logic.get_port(f'dm<{idx}>'))

        if sampler_params:
            cap_in_layer = cdac_p.get_port('in').get_single_layer()
            self.connect_to_track_wires(cdac_p.get_all_port_pins('in'), sampler.get_pin('sig_p',
                                                                                        layer=cap_in_layer + 1))
            self.connect_to_track_wires(cdac_n.get_all_port_pins('in'), sampler.get_pin('sig_n',
                                                                                        layer=cap_in_layer + 1))
            self.connect_to_track_wires(cdac_n.get_all_port_pins('sam'), vg_n_xm1)
            self.connect_to_track_wires(cdac_p.get_all_port_pins('sam'), vg_p_xm1)

            for p, n in zip(sampler.get_all_port_pins('out_p_bot'), sampler.get_all_port_pins('out_n_bot')):
                self.connect_differential_wires(cdac_n.get_all_port_pins('top'),
                                                cdac_p.get_all_port_pins('top'), n, p)

        # Get vdd_xm, vss_xm from logic and comp
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # Connect CDAC top with shield
        dac_top_xm_shield_top_tidx = tr_manager.get_next_track(xm_layer, dac_top_tidx, 'cap', 'sup', up=True)
        dac_top_xm_shield_bot_tidx = tr_manager.get_next_track(xm_layer, dac_top_tidx, 'cap', 'sup', up=False)
        ym_tidx_rbnd = self.grid.coord_to_track(ym_layer,
                                                max(logic.bound_box.xh, comp.bound_box.xh, cdac_p.bound_box.xh),
                                                RoundMode.GREATER_EQ)
        ym_tidx_lbnd = self.grid.coord_to_track(ym_layer,
                                                min(logic.bound_box.xl, comp.bound_box.xl, cdac_n.bound_box.xl),
                                                RoundMode.LESS_EQ)
        _, dac_top_ym_locs_l = tr_manager.place_wires(ym_layer, ['sup', 'cap', 'cap', 'sup', 'dum'],
                                                      align_track=ym_tidx_lbnd,
                                                      align_idx=-1)
        _, dac_top_ym_locs_r = tr_manager.place_wires(ym_layer, ['dum', 'sup', 'cap', 'cap', 'sup'],
                                                      align_track=ym_tidx_rbnd)

        dac_top_xm_shield_top = self.add_wires(xm_layer, dac_top_xm_shield_top_tidx, self.bound_box.xl,
                                               self.bound_box.xh, width=tr_manager.get_width(xm_layer, 'sup'))
        dac_top_xm_shield_bot = self.add_wires(xm_layer, dac_top_xm_shield_bot_tidx, self.bound_box.xl,
                                               self.bound_box.xh, width=tr_manager.get_width(xm_layer, 'sup'))

        dac_top_ym_shield_ll = self.add_wires(ym_layer, dac_top_ym_locs_l[0], self.bound_box.yl,
                                              comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))
        dac_top_ym_shield_lr = self.add_wires(ym_layer, dac_top_ym_locs_l[-2], self.bound_box.yl,
                                              comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))
        dac_top_ym_shield_rl = self.add_wires(ym_layer, dac_top_ym_locs_r[-1], self.bound_box.yl,
                                              comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))
        dac_top_ym_shield_rr = self.add_wires(ym_layer, dac_top_ym_locs_r[1], self.bound_box.yl,
                                              comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))

        dac_top_p_ym = [self.connect_to_tracks(dac_p_xm, TrackID(ym_layer, dac_top_ym_locs_r[-2],
                                                                 tr_manager.get_width(ym_layer, 'cap')),
                                               track_lower=self.bound_box.yl),
                        self.connect_to_tracks(dac_p_xm, TrackID(ym_layer, dac_top_ym_locs_r[-3],
                                                                 tr_manager.get_width(ym_layer, 'cap')),
                                               track_lower=self.bound_box.yl)]
        dac_top_n_ym = [self.connect_to_tracks(dac_n_xm, TrackID(ym_layer, dac_top_ym_locs_l[1],
                                                                 tr_manager.get_width(ym_layer, 'cap')),
                                               track_lower=self.bound_box.yl),
                        self.connect_to_tracks(dac_n_xm, TrackID(ym_layer, dac_top_ym_locs_l[2],
                                                                 tr_manager.get_width(ym_layer, 'cap')),
                                               track_lower=self.bound_box.yl)]
        self.connect_to_track_wires(dac_top_xm_shield_bot, [dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                                            dac_top_ym_shield_rr, dac_top_ym_shield_rl])
        self.connect_to_track_wires(dac_top_xm_shield_top, [dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                                            dac_top_ym_shield_rr, dac_top_ym_shield_rl])
        shield_sup_xm = self.connect_to_track_wires([dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                                     dac_top_ym_shield_rr, dac_top_ym_shield_rl],
                                                    comp.get_all_port_pins('VDD', xm1_layer))[0]
        self.connect_to_track_wires([dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                     dac_top_ym_shield_rr, dac_top_ym_shield_rl],
                                    logic.get_all_port_pins('VDD', xm1_layer))[0]
        self.connect_to_track_wires([dac_top_xm_shield_top, dac_top_xm_shield_bot],
                                    comp.get_all_port_pins('VDD', ym_layer))

        self.extend_wires(logic.get_all_port_pins('VSS', xm1_layer),
                          lower=shield_sup_xm.lower, upper=shield_sup_xm.upper)
        self.extend_wires(comp.get_all_port_pins('VSS', xm1_layer),
                          lower=shield_sup_xm.lower, upper=shield_sup_xm.upper)

        # Get all ym1 tidx from cdac
        cdac_pname_list = ['VDD', 'VSS', 'vref<0>', 'vref<1>', 'vref<2>']
        ym1_tidx_list = []
        for pname in cdac_pname_list:
            _ym1_list = cdac_n.get_all_port_pins(pname)
            ym1_tidx_list.extend([ym1.track_id.base_index for ym1 in _ym1_list])
            _ym1_list = cdac_p.get_all_port_pins(pname)
            ym1_tidx_list.extend([ym1.track_id.base_index for ym1 in _ym1_list])
        ym1_tidx_list.sort()
        num_ym1 = len(ym1_tidx_list)
        ym1_tidx_list = ym1_tidx_list[:num_ym1 // 2] + ym1_tidx_list[num_ym1 // 2:]
        vdd_ym1_list, vss_ym1_list = [], []

        vdd_xm1_comp = comp.get_all_port_pins('VDD', layer=xm1_layer)
        vss_xm1_comp = comp.get_all_port_pins('VSS', layer=xm1_layer)
        vdd_xm1_logic = logic.get_all_port_pins('VDD', layer=xm1_layer)
        vss_xm1_logic = logic.get_all_port_pins('VSS', layer=xm1_layer)
        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        #


        # Connect  btw capdac hold and logic rst
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        cdac_hold_xm_tidx = self.grid.coord_to_track(xm_layer, cdac_n.bound_box.yl, RoundMode.LESS_EQ)
        cdac_hold_xm_tidx = tr_manager.get_next_track(xm_layer, cdac_hold_xm_tidx, 'dum', 'clk', up=False)
        cdac_hold_xm = self.connect_to_tracks([cdac_n.get_pin('sam_b'), cdac_p.get_pin('sam_b')],
                                              TrackID(xm_layer, cdac_hold_xm_tidx, tr_w_clk_xm))
        rst_xm_comp_top_tidx = self.grid.coord_to_track(xm_layer, comp.bound_box.yh, RoundMode.GREATER_EQ)
        rst_xm_comp_top_tidx = tr_manager.get_next_track(xm_layer, rst_xm_comp_top_tidx, 'clk', 'clk', 2)
        if sampler_params:

            sam_clk_ym_tidx = self.grid.coord_to_track(ym_layer, sampler.get_pin('sam_e_lower', layer=xm_layer).middle,
                                                       RoundMode.NEAREST)
            sam_clk_ym = self.connect_to_tracks(sampler.get_pin('sam_e_lower', layer=xm_layer),
                                                TrackID(ym_layer, sam_clk_ym_tidx, tr_w_clk_ym, grid=self.grid))
            rst_xm_comp_top = self.connect_to_tracks(sam_clk_ym,
                                                     TrackID(xm_layer, rst_xm_comp_top_tidx, tr_w_clk_xm))
        else:
            cdac_hold_ym_tidx = self.grid.coord_to_track(ym_layer, cdac_n.bound_box.xh, RoundMode.NEAREST)
            cdac_hold_ym = self.connect_to_tracks(cdac_hold_xm, TrackID(ym_layer, cdac_hold_ym_tidx, tr_w_clk_ym))
            rst_xm_comp_top = self.add_wires(xm_layer, rst_xm_comp_top_tidx, width=tr_w_clk_xm,
                                             lower=cdac_n.bound_box.xl, upper=cdac_n.bound_box.xh)
            self.add_pin('sam_e', rst_xm_comp_top)
        rst_ym_comp_left_tidx = self.grid.coord_to_track(ym_layer, comp.bound_box.xl, RoundMode.LESS_EQ)
        rst_ym_comp_left = self.connect_to_tracks(rst_xm_comp_top, TrackID(ym_layer, rst_ym_comp_left_tidx,
                                                                           tr_w_clk_ym, grid=self.grid))
        rst_xm_logic_top_tidx = self.grid.coord_to_track(xm_layer, logic.bound_box.yh, RoundMode.GREATER_EQ)
        rst_xm_logic_top = self.connect_to_tracks(rst_ym_comp_left, TrackID(xm_layer, rst_xm_logic_top_tidx,
                                                                            tr_w_clk_xm))
        self.connect_to_track_wires(rst_xm_logic_top, logic.get_all_port_pins('rst'))
        # export clock signal
        # clock for logic
        _, logic_rst_xm1_locs = tr_manager.place_wires(xm1_layer, ['dum', 'clk', 'clk'],
                                                       align_track=self.grid.coord_to_track(xm1_layer,
                                                                                            logic.bound_box.yh,
                                                                                            RoundMode.NEAREST))

        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        hold_b_xm1 = self.connect_to_tracks(logic.get_all_port_pins('rst_comp_clk'),
                                            TrackID(xm1_layer, logic_rst_xm1_locs[1], tr_w_clk_xm1))
        rst_xm1 = self.connect_to_tracks(logic.get_all_port_pins('rst'),
                                         TrackID(xm1_layer, logic_rst_xm1_locs[2], tr_w_clk_xm1))

        hold_b_xm1 = self.connect_wires(hold_b_xm1, lower=self.bound_box.xl)
        rst_xm1 = self.connect_wires(rst_xm1, lower=self.bound_box.xl)
        self.add_pin('clk_e', rst_xm1)
        self.add_pin('hold_b', hold_b_xm1)

        if sampler_params:
            self.reexport(sampler.get_port('sig_n'), net_name='in_n')
            self.reexport(sampler.get_port('sig_p'), net_name='in_p')
            _, ym_sam_clk_locs = tr_manager.place_wires(ym_layer, ['dum'] + ['clk'] * 4,
                                                        align_track=self.grid.coord_to_track(ym_layer,
                                                                                             self.bound_box.xl))
            sam_ym, sam_b_ym, sam_e_ym, sam_e_b_ym = \
                self.connect_matching_tracks([sampler.get_pin('sam', layer=xm_layer),
                                              sampler.get_pin('sam_b', layer=xm_layer),
                                              sampler.get_pin('sam_e', layer=xm_layer),
                                              sampler.get_pin('sam_e_b', layer=xm_layer)],
                                             ym_layer, ym_sam_clk_locs[1:],
                                             width=tr_manager.get_width(ym_layer, 'clk'))

            _, xm1_sam_clk_locs = tr_manager.place_wires(xm1_layer, ['clk'] * 4, center_coord=sam_ym.middle)
            sam_xm1, sam_b_xm1, sam_e_xm1, sam_e_b_xm1 = \
                self.connect_matching_tracks([sam_ym, sam_b_ym, sam_e_ym, sam_e_b_ym],
                                             xm1_layer, xm1_sam_clk_locs,
                                             width=tr_manager.get_width(xm1_layer, 'clk'))
            self.add_pin('clk', sam_xm1)
            self.add_pin('clk_e', sam_e_xm1)
            self.add_pin('clk_b', sam_b_xm1)
            self.add_pin('clk_e_b', sam_e_b_xm1)
        else:
            self.reexport(cdac_p.get_port('in'), net_name='sig_p', connect=True)
            self.reexport(cdac_n.get_port('in'), net_name='sig_n', connect=True)
            self.reexport(cdac_p.get_port('in_c'), net_name='sig_n', connect=True)
            self.reexport(cdac_n.get_port('in_c'), net_name='sig_p', connect=True)
            self.reexport(cdac_p.get_port('sam'), net_name='vg_p')
            self.reexport(cdac_n.get_port('sam'), net_name='vg_n')

        self.reexport(cdac_p.get_port('sam_b'), net_name='hold', connect=True)
        self.reexport(cdac_n.get_port('sam_b'), net_name='hold', connect=True)
        self.reexport(cdac_p.get_port('top'), net_name='top_p')
        self.reexport(cdac_n.get_port('top'), net_name='top_n')

        self.reexport(logic.get_port('clk_out'))
        self.reexport(logic.get_port('comp_clk'))
        self.reexport(logic.get_port('rst_comp_clk'), net_name='hold_b')
        self.reexport(comp.get_port('outp'), net_name='comp_p')
        self.reexport(comp.get_port('outn'), net_name='comp_n')
        self.reexport(comp.get_port('osp'))
        self.reexport(comp.get_port('osn'))

        # Connect supplies together
        cdac_top_layer = cdac_n.master.top_layer
        cdac_vdd_top = cdac_n.get_all_port_pins('VDD', cdac_top_layer) + cdac_p.get_all_port_pins('VDD', cdac_top_layer)
        cdac_vss_top = cdac_n.get_all_port_pins('VSS', cdac_top_layer) + cdac_p.get_all_port_pins('VSS', cdac_top_layer)
        cdac_vss_top = self.extend_wires(cdac_vss_top, lower=self.bound_box.yl, upper=self.bound_box.yh)
        cdac_vdd_top = self.extend_wires(cdac_vdd_top, lower=self.bound_box.yl, upper=self.bound_box.yh)

        vdd_topm = comp.get_all_port_pins('VDD', cdac_top_layer - 1) + logic.get_all_port_pins('VDD',
                                                                                               cdac_top_layer - 1) + \
                   sampler.get_all_port_pins('VDD', cdac_top_layer - 1)
        vss_topm = comp.get_all_port_pins('VSS', cdac_top_layer - 1) + logic.get_all_port_pins('VSS',
                                                                                               cdac_top_layer - 1) + \
                   sampler.get_all_port_pins('VSS', cdac_top_layer - 1)
        self.connect_to_track_wires(cdac_vdd_top, vdd_topm)
        self.connect_to_track_wires(cdac_vss_top, vss_topm)

        self.add_pin('VDD', cdac_vdd_top)
        self.add_pin('VSS', cdac_vss_top)
        self.add_pin('vref<1>', sampler.get_all_port_pins('vcm'), connect=True)
        self.add_pin('vref<1>', cdac_n.get_all_port_pins('vref<1>'), connect=True)
        self.add_pin('vref<2>', cdac_n.get_all_port_pins('vref<2>'), connect=True)
        self.add_pin('vref<0>', cdac_n.get_all_port_pins('vref<0>'), connect=True)
        self.add_pin('vref<1>', cdac_p.get_all_port_pins('vref<1>'), connect=True)
        self.add_pin('vref<2>', cdac_p.get_all_port_pins('vref<2>'), connect=True)
        self.add_pin('vref<0>', cdac_p.get_all_port_pins('vref<0>'), connect=True)

        sar_params = dict(
            nbits=nbits,
            comp=comp_master.sch_params,
            logic=logic_master.sch_params,
            cdac=cdac_master.sch_params,
            tri_sa=True,
            has_pmos_sw=has_pmos_sw,
        )

        if sampler_params:
            self._sch_params = dict(
                slice_params=sar_params,
                sampler_params=sampler_master.sch_params
            )
        else:
            self._sch_params = sar_params


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
            split_dac='True to use split dac'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(sampler_params=[], split_dac=False)

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

        split_dac = self.params['split_dac']
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
        cdac_lay_class = CapDacSplitCore if split_dac else CapDacColCore
        cdac_master: cdac_lay_class = self.new_template(cdac_lay_class, params=cdac_params)
        comp_master_dummy: MOSBase = self.new_template(IntegrationWrapper, params=comp_params)
        logic_ncols_tot = 2 * (cdac_master.bound_box.w // comp_master_dummy.core.core.sd_pitch)

        logic_new_params = logic_params['params'].to_dict()
        logic_new_params['params'] = logic_new_params['params'].to_dict()
        logic_new_params['ncols_tot'] = logic_ncols_tot
        logic_new_params['params']['split_dac'] = split_dac
        logic_gen_params = dict(
            cls_name=logic_params['cls_name'],
            top_sup_layer=comp_params['top_sup_layer'],
            params=logic_new_params,
            extra_mid_space=6,
        )

        logic_master: TemplateBase = self.new_template(IntegrationWrapper, params=logic_gen_params)
        logic_buf_master: IntegrationWrapper = self.new_template(IntegrationWrapper, params=logic_buf_params)

        cdac_actual_width = cdac_master.actual_width_sampler
        logic_buf_width = logic_buf_master.bound_box.w
        comp_bnd_x = (comp_master_dummy.core.bound_box.w - comp_master_dummy.core.core.bound_box.w) // 2

        top_layer = max(logic_master.top_layer, comp_master_dummy.top_layer, cdac_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer, half_blk_y=True, half_blk_x=True)
        w_comp_blk, h_comp_blk = self.grid.get_block_size(max(comp_master_dummy.top_layer, logic_master.top_layer),
                                                          half_blk_y=True, half_blk_x=True)
        # Calculate comparator width, filled with tap and supply routing
        w_blk_in_sd_picth = w_blk // comp_master_dummy.core.core.sd_pitch
        comp_w = max(2 * (cdac_actual_width - logic_buf_width - comp_bnd_x), comp_master_dummy.bound_box.w)
        ncols_tot_comp = comp_w // comp_master_dummy.core.core.sd_pitch
        ncols_tot = ncols_tot_comp // 2
        ncols_tot += ncols_tot & 1
        ncols_tot = ncols_tot * 2

        comp_core_params = comp_params['params']['params'].to_dict()
        comp_core_params['ncols_tot'] = ncols_tot
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

        # Connect digital signals
        # If CDAC has pmos switch, connect logic signal dn/dp together
        sw_type = cdac_params.get('sw_type', ['n', 'n', 'n'])

        ###############################
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
        h_tot = -(-h_logic // h_blk) * h_blk + h_comp + coord_dig_tr

        comp_y = -(-h_tot // h_blk) * h_blk

        logic_x = -(-(w_tot - w_logic) // 2 // w_comp_blk) * w_comp_blk
        comp_x = -(-(w_tot - w_comp) // 2 // w_comp_blk) * w_comp_blk
        dac_x = -(-(w_tot - 2 * w_dac) // 2 // w_blk) * w_blk

        num_cap_top_tr, _ = tr_manager.place_wires(xm_layer, ['cap'] * 3)
        coord_cap_top_tr = self.grid.track_to_coord(xm_layer, num_cap_top_tr)
        cap_y = -(-(comp_y + coord_cap_top_tr) // h_blk) * h_blk
        h_tot = -(-cap_y // h_blk) * h_blk + h_dac

        logic = self.add_instance(logic_master, inst_name='XLOGIC', xform=Transform(logic_x, 0))
        comp = self.add_instance(comp_master, inst_name='XCOMP', xform=Transform(comp_x, comp_y, mode=Orientation.MX))
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
        buf_y = comp.bound_box.yl + logic_buf_master.bound_box.h
        buf_y = - (-buf_y // h_blk) * h_blk
        buf_n = self.add_instance(logic_buf_master, inst_name='XBUF_N',
                                  xform=Transform(comp_x - comp_buf_sp, buf_y, mode=Orientation.R180))
        buf_x = -(-(comp.bound_box.w + comp_x) // w_comp_blk) * w_comp_blk
        buf_p = self.add_instance(logic_buf_master, inst_name='XBUF_P',
                                  xform=Transform(buf_x + comp_buf_sp, buf_y, mode=Orientation.MX))
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
                sampler_params['cdac_width'] = cdac_params['width']
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
            tr_samp_sig_sep = self.get_track_sep(xm1_layer, tr_w_samp_sig_xm1, tr_w_samp_sig_xm1)
            sam_bot_locs = sam_bot_xm_tidx + tr_samp_sig_sep.div2()
            coord_sam_tr = self.grid.track_to_coord(xm1_layer, tr_samp_sig_sep)

            sam_y = -(-(h_tot + coord_sam_tr) // h_blk) * h_blk
            sam_x = -(-(w_tot // 2 - w_sam // 2) // w_blk) * w_blk
            sampler = self.add_instance(sampler_master, inst_name='XSAM',
                                        xform=Transform(sam_x, sam_y, mode=Orientation.R0))
            vg_n_xm1 = self.connect_to_tracks(sampler.get_all_port_pins('vg_n_ym'),
                                              TrackID(xm1_layer, sam_bot_locs, tr_w_samp_sig_xm1, grid=self.grid))
            vg_p_xm1 = self.connect_to_tracks(sampler.get_all_port_pins('vg_p_ym'),
                                              TrackID(xm1_layer, sam_bot_locs, tr_w_samp_sig_xm1, grid=self.grid))
            _, sam_bot_locs = tr_manager.place_wires(xm1_layer, ['dum'] + ['sig'] * 4, align_idx=0,
                                                     align_track=sam_bot_xm_tidx)
            sam_bot_locs = sam_bot_locs[1:]
            self.connect_to_track_wires(cdac_n.get_pin('voff'), sampler.get_pin('voff_n'))
            self.connect_to_track_wires(cdac_p.get_pin('voff'), sampler.get_pin('voff_p'))
            self.connect_differential_tracks([cdac_n.get_pin('in_c'), cdac_p.get_pin('in')],
                                             [cdac_n.get_pin('in'), cdac_p.get_pin('in_c')],
                                             xm1_layer, sam_bot_locs[-1], sam_bot_locs[1],
                                             width=tr_manager.get_width(xm1_layer, 'sig'))
            self.connect_differential_tracks([cdac_n.get_pin('in_c'), cdac_p.get_pin('in')],
                                             [cdac_n.get_pin('in'), cdac_p.get_pin('in_c')],
                                             xm1_layer, sam_bot_locs[-2], sam_bot_locs[0],
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
        ################################
        # Connect differential input to comp
        ################################

        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')
        dac_top_n, dac_top_p = cdac_n.get_pin('top', layer=ym_layer), cdac_p.get_pin('top', layer=ym_layer)
        dac_top_tidx = self.grid.coord_to_track(xm_layer, (comp.bound_box.yh + cdac_n.bound_box.yl) // 2,
                                                mode=RoundMode.NEAREST)
        dac_n_xm = self.connect_to_tracks(dac_top_n, TrackID(xm_layer, dac_top_tidx, tr_w_cap_xm))
        dac_p_xm = self.connect_to_tracks(dac_top_p, TrackID(xm_layer, dac_top_tidx, tr_w_cap_xm))

        self.connect_to_track_wires(comp.get_all_port_pins('inp'), dac_p_xm)
        self.connect_to_track_wires(comp.get_all_port_pins('inn'), dac_n_xm)
        #
        # Connect comp_out_m
        tr_w_dig_vm = tr_manager.get_width(vm_layer, 'dig')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')

        logic_top_xm_tid = self.grid.coord_to_track(xm_layer, logic.bound_box.yh, RoundMode.NEAREST)
        _, comp_out_locs = tr_manager.place_wires(xm_layer, comp_out_type_list, align_track=logic_top_xm_tid)
        sig_type_list = ['clk'] + ['dig'] * nbits * 3

        _, dig_tr_locs = tr_manager.place_wires(xm_layer, sig_type_list, align_idx=0, align_track=comp_out_locs[-1])
        tr_w_dig_xm = tr_manager.get_width(xm_layer, 'dig')
        dig_tr_locs.pop(0)

        # Connect vm ctrl signal if it's not in cdac

        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        ctrl_vm_rt = cdac_params['ctrl_vm_rt']
        ctrl_sig_type_list = ['ctrl'] * (2 * nbits + 3) if split_dac else ['ctrl'] * (3 * nbits + 3)
        ctrl_tidx_start_vm_l = \
            self.grid.coord_to_track(vm_layer, min(cdac_n.bound_box.xh - cdac_master.actual_width, buf_n.bound_box.xl),
                                     RoundMode.LESS)
        _, ctrl_tidx_locs_l = tr_manager.place_wires(vm_layer, ctrl_sig_type_list, align_idx=-1,
                                                     align_track=ctrl_tidx_start_vm_l)
        ctrl_tidx_start_vm_r = \
            self.grid.coord_to_track(vm_layer, max(cdac_p.bound_box.xl + cdac_master.actual_width, buf_p.bound_box.xh),
                                     RoundMode.LESS)
        _, ctrl_tidx_locs_r = tr_manager.place_wires(vm_layer, ctrl_sig_type_list, align_idx=0,
                                                     align_track=ctrl_tidx_start_vm_r)
        ctrl_tidx_locs_r = ctrl_tidx_locs_r[::-1]

        for idx in range(nbits):
            bufp_n, bufp_p = buf_p.get_pin(f'din0<{idx}>'), \
                             buf_p.get_pin(f'din1<{idx}>' if split_dac else f'din2<{idx}>')
            bufn_n, bufn_p = buf_n.get_pin(f'din0<{idx}>'), \
                             buf_n.get_pin(f'din1<{idx}>' if split_dac else f'din2<{idx}>')
            _buf_p = self.connect_to_tracks([bufn_p, bufp_n], TrackID(xm_layer, dig_tr_locs[3 * idx], tr_w_dig_xm))
            _buf_n = self.connect_to_tracks([bufn_n, bufp_p], TrackID(xm_layer, dig_tr_locs[3 * idx + 1], tr_w_dig_xm))
            if not split_dac:
                bufp_m, bufn_m = buf_p.get_pin(f'din1<{idx}>'), buf_n.get_pin(f'din1<{idx}>')
                _buf_m = self.connect_to_tracks([bufp_m, bufn_m],
                                                TrackID(xm_layer, dig_tr_locs[3 * idx + 2], tr_w_dig_xm))
                self.connect_to_track_wires(logic.get_pin(f'dm<{idx}>'), _buf_m)
            else:
                _buf_p, _buf_n = _buf_n, _buf_p
            self.connect_to_track_wires(logic.get_pin(f'dp<{idx}>'), _buf_p)
            self.connect_to_track_wires(logic.get_pin(f'dn<{idx}>'), _buf_n)
            bufp_n, bufp_p = buf_p.get_pin(f'dout0<{idx}>'), \
                             buf_p.get_pin(f'dout1<{idx}>' if split_dac else f'dout2<{idx}>')
            bufn_n, bufn_p = buf_n.get_pin(f'dout0<{idx}>'), \
                             buf_n.get_pin(f'dout1<{idx}>' if split_dac else f'dout2<{idx}>')
            dacp_n, dacp_p = cdac_p.get_all_port_pins(f'ctrl_n<{idx}>'), cdac_p.get_all_port_pins(f'ctrl_p<{idx}>')
            dacn_n, dacn_p = cdac_n.get_all_port_pins(f'ctrl_n<{idx}>'), cdac_n.get_all_port_pins(f'ctrl_p<{idx}>')
            if not ctrl_vm_rt:
                tid_idx = 2 * idx if split_dac else 3 * idx
                dacp_n = self.connect_to_tracks(dacp_n, TrackID(vm_layer, ctrl_tidx_locs_r[tid_idx], tr_w_ctrl_vm))
                dacp_p = self.connect_to_tracks(dacp_p, TrackID(vm_layer, ctrl_tidx_locs_r[tid_idx + 1], tr_w_ctrl_vm))
                dacn_n = self.connect_to_tracks(dacn_n, TrackID(vm_layer, ctrl_tidx_locs_l[tid_idx], tr_w_ctrl_vm))
                dacn_p = self.connect_to_tracks(dacn_p, TrackID(vm_layer, ctrl_tidx_locs_l[tid_idx + 1], tr_w_ctrl_vm))

            self.connect_to_track_wires(bufp_n, dacp_n)
            self.connect_to_track_wires(bufp_p, dacp_p)
            self.connect_to_track_wires(bufn_n, dacn_n)
            self.connect_to_track_wires(bufn_p, dacn_p)
            if not split_dac:
                dacp_m, dacn_m = cdac_p.get_all_port_pins(f'ctrl_m<{idx}>'), cdac_n.get_all_port_pins(f'ctrl_m<{idx}>')
                bufp_m, bufn_m = buf_p.get_all_port_pins(f'dout1<{idx}>'), buf_n.get_all_port_pins(f'dout1<{idx}>')
                if not ctrl_vm_rt:
                    tid_idx = 2 * idx if split_dac else 3 * idx
                    dacp_m = self.connect_to_tracks(dacp_m,
                                                    TrackID(vm_layer, ctrl_tidx_locs_r[tid_idx + 2], tr_w_ctrl_vm))
                    dacn_m = self.connect_to_tracks(dacn_m,
                                                    TrackID(vm_layer, ctrl_tidx_locs_l[tid_idx + 2], tr_w_ctrl_vm))

                self.connect_to_track_wires(bufp_m, dacp_m)
                self.connect_to_track_wires(bufn_m, dacn_m)
        #
        # --- export pins:
        for idx in range(nbits):
            self.reexport(logic.get_port(f'data_out<{idx}>'))
            self.reexport(logic.get_port(f'dn<{idx}>'))
            self.reexport(logic.get_port(f'dp<{idx}>'))
            if not split_dac:
                self.reexport(logic.get_port(f'dm<{idx}>'))
        if sampler_params:
            cap_in_layer = cdac_p.get_port('in').get_single_layer()
            self.connect_to_track_wires(cdac_p.get_all_port_pins('in'),
                                        sampler.get_pin('sig_p', layer=cap_in_layer + 1))
            self.connect_to_track_wires(cdac_n.get_all_port_pins('in'),
                                        sampler.get_pin('sig_n', layer=cap_in_layer + 1))

            # check if has more than 1 signal routing for cross couple
            sig_p_top = sampler.get_pin('sig_p', layer=cap_in_layer + 1).to_warr_list()[0]
            sig_n_bot = sampler.get_pin('sig_n', layer=cap_in_layer + 1).to_warr_list()[-1]
            sig_p_ret = self.connect_to_track_wires(sig_p_top, cdac_n.get_all_port_pins('in_c'))
            sig_n_ret = self.connect_to_track_wires(sig_n_bot, cdac_p.get_all_port_pins('in_c'))
            self.match_warr_length([sig_p_ret, sig_n_ret])
            vgn = self.connect_to_track_wires(vg_n_xm1, cdac_n.get_all_port_pins('vg'))
            vgp = self.connect_to_track_wires(vg_p_xm1, cdac_p.get_all_port_pins('vg'))
            voffn = self.connect_to_track_wires(sampler.get_pin('voff_n'), cdac_n.get_pin('voff'))
            voffp = self.connect_to_track_wires(sampler.get_pin('voff_p'), cdac_p.get_pin('voff'))
            self.match_warr_length(vgn + vgp + [voffp, voffn])

            for p, n in zip(sampler.get_all_port_pins('out_p_bot'), sampler.get_all_port_pins('out_n_bot')):
                self.connect_to_track_wires(cdac_n.get_all_port_pins('top', layer=ym1_layer), n)
                self.connect_to_track_wires(cdac_p.get_all_port_pins('top', layer=ym1_layer), p)

        # Get vdd_xm, vss_xm from logic and comp
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # Connect CDAC top with shield
        cdac_top_export = False
        if cdac_top_export:
            dac_top_xm_shield_top_tidx = tr_manager.get_next_track(xm_layer, dac_top_tidx, 'cap', 'sup', up=True)
            dac_top_xm_shield_bot_tidx = tr_manager.get_next_track(xm_layer, dac_top_tidx, 'cap', 'sup', up=False)
            ym_tidx_rbnd = self.grid.coord_to_track(ym_layer,
                                                    max(logic.bound_box.xh, comp.bound_box.xh, cdac_p.bound_box.xh),
                                                    RoundMode.GREATER_EQ)
            ym_tidx_lbnd = self.grid.coord_to_track(ym_layer,
                                                    min(logic.bound_box.xl, comp.bound_box.xl, cdac_n.bound_box.xl),
                                                    RoundMode.LESS_EQ)
            _, dac_top_ym_locs_l = tr_manager.place_wires(ym_layer, ['sup', 'cap', 'cap', 'sup', 'dum'],
                                                          align_track=ym_tidx_lbnd,
                                                          align_idx=-1)
            _, dac_top_ym_locs_r = tr_manager.place_wires(ym_layer, ['dum', 'sup', 'cap', 'cap', 'sup'],
                                                          align_track=ym_tidx_rbnd)

            dac_top_xm_shield_top = self.add_wires(xm_layer, dac_top_xm_shield_top_tidx, self.bound_box.xl,
                                                   self.bound_box.xh, width=tr_manager.get_width(xm_layer, 'sup'))
            dac_top_xm_shield_bot = self.add_wires(xm_layer, dac_top_xm_shield_bot_tidx, self.bound_box.xl,
                                                   self.bound_box.xh, width=tr_manager.get_width(xm_layer, 'sup'))

            dac_top_ym_shield_ll = self.add_wires(ym_layer, dac_top_ym_locs_l[0], self.bound_box.yl,
                                                  comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))
            dac_top_ym_shield_lr = self.add_wires(ym_layer, dac_top_ym_locs_l[-2], self.bound_box.yl,
                                                  comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))
            dac_top_ym_shield_rl = self.add_wires(ym_layer, dac_top_ym_locs_r[-1], self.bound_box.yl,
                                                  comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))
            dac_top_ym_shield_rr = self.add_wires(ym_layer, dac_top_ym_locs_r[1], self.bound_box.yl,
                                                  comp.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'))

            dac_top_p_ym = [self.connect_to_tracks(dac_p_xm, TrackID(ym_layer, dac_top_ym_locs_r[-2],
                                                                     tr_manager.get_width(ym_layer, 'cap')),
                                                   track_lower=self.bound_box.yl),
                            self.connect_to_tracks(dac_p_xm, TrackID(ym_layer, dac_top_ym_locs_r[-3],
                                                                     tr_manager.get_width(ym_layer, 'cap')),
                                                   track_lower=self.bound_box.yl)]
            dac_top_n_ym = [self.connect_to_tracks(dac_n_xm, TrackID(ym_layer, dac_top_ym_locs_l[1],
                                                                     tr_manager.get_width(ym_layer, 'cap')),
                                                   track_lower=self.bound_box.yl),
                            self.connect_to_tracks(dac_n_xm, TrackID(ym_layer, dac_top_ym_locs_l[2],
                                                                     tr_manager.get_width(ym_layer, 'cap')),
                                                   track_lower=self.bound_box.yl)]
            self.connect_to_track_wires(dac_top_xm_shield_bot, [dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                                                dac_top_ym_shield_rr, dac_top_ym_shield_rl])
            self.connect_to_track_wires(dac_top_xm_shield_top, [dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                                                dac_top_ym_shield_rr, dac_top_ym_shield_rl])
            shield_sup_xm = self.connect_to_track_wires([dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                                         dac_top_ym_shield_rr, dac_top_ym_shield_rl],
                                                        comp.get_all_port_pins('VDD', xm1_layer))[0]
            self.connect_to_track_wires([dac_top_ym_shield_ll, dac_top_ym_shield_lr,
                                         dac_top_ym_shield_rr, dac_top_ym_shield_rl],
                                        logic.get_all_port_pins('VDD', xm1_layer))[0]
            self.connect_to_track_wires([dac_top_xm_shield_top, dac_top_xm_shield_bot],
                                        comp.get_all_port_pins('VDD', ym_layer))

            self.extend_wires(logic.get_all_port_pins('VSS', xm1_layer),
                              lower=shield_sup_xm.lower, upper=shield_sup_xm.upper)
            self.extend_wires(comp.get_all_port_pins('VSS', xm1_layer),
                              lower=shield_sup_xm.lower, upper=shield_sup_xm.upper)

        # Get all ym1 tidx from cdac
        cdac_pname_list = ['VDD', 'VSS', 'vref<0>', 'vref<1>', 'vref<2>']
        ym1_tidx_list = []
        for pname in cdac_pname_list:
            _ym1_list = cdac_n.get_all_port_pins(pname)
            ym1_tidx_list.extend([ym1.track_id.base_index for ym1 in _ym1_list])
            _ym1_list = cdac_p.get_all_port_pins(pname)
            ym1_tidx_list.extend([ym1.track_id.base_index for ym1 in _ym1_list])
        ym1_tidx_list.sort()
        num_ym1 = len(ym1_tidx_list)
        ym1_tidx_list = ym1_tidx_list[:num_ym1 // 2] + ym1_tidx_list[num_ym1 // 2:]
        vdd_ym1_list, vss_ym1_list = [], []

        vdd_xm1_comp = comp.get_all_port_pins('VDD', layer=xm1_layer)
        vss_xm1_comp = comp.get_all_port_pins('VSS', layer=xm1_layer)
        vdd_xm1_logic = logic.get_all_port_pins('VDD', layer=xm1_layer)
        vss_xm1_logic = logic.get_all_port_pins('VSS', layer=xm1_layer)
        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')

        # Connect  btw capdac hold and logic rst
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        cdac_hold_xm1_tidx = self.grid.coord_to_track(xm1_layer, cdac_n.bound_box.yl, RoundMode.LESS_EQ)
        cdac_hold_xm1_tidx = tr_manager.get_next_track(xm1_layer, cdac_hold_xm1_tidx, 'dum', 'clk', up=-2)
        cdac_hold_xm1 = self.connect_to_tracks([cdac_n.get_pin('samb'), cdac_p.get_pin('samb')],
                                               TrackID(xm1_layer, cdac_hold_xm1_tidx, tr_w_clk_xm1),
                                               track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
        self.add_pin('hold', cdac_hold_xm1, mode=PinMode.LOWER)
        if split_dac:
            cdac_holdb_xm1_tidx = tr_manager.get_next_track(xm1_layer, cdac_hold_xm1_tidx, 'clk', 'clk', up=False)
            cdac_holdb_xm1 = self.connect_to_tracks([cdac_n.get_pin('sam'), cdac_p.get_pin('sam')],
                                                    TrackID(xm1_layer, cdac_holdb_xm1_tidx, tr_w_clk_xm1),
                                                    track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            self.add_pin('hold_b', cdac_holdb_xm1, mode=PinMode.LOWER)
        # self.reexport(cdac_p.get_port('samb'), net_name='hold', connect=True)
        # self.reexport(cdac_n.get_port('samb'), net_name='hold', connect=True)
        rst_xm_comp_top_tidx = self.grid.coord_to_track(xm_layer, comp.bound_box.yh, RoundMode.GREATER_EQ)
        rst_xm_comp_top_tidx = tr_manager.get_next_track(xm_layer, rst_xm_comp_top_tidx, 'clk', 'clk', 2)
        _, logic_rst_xm1_locs = tr_manager.place_wires(xm1_layer, ['dum', 'clk', 'clk'],
                                                       align_track=self.grid.coord_to_track(xm1_layer,
                                                                                            logic.bound_box.yh,
                                                                                            RoundMode.NEAREST))

        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        hold_b_ym_tidx = self.grid.coord_to_track(ym_layer, comp.bound_box.xl, RoundMode.NEAREST)
        avail_ym_locs = self.get_available_tracks(ym_layer,
                                                  self.grid.coord_to_track(ym_layer, self.bound_box.xl,
                                                                           RoundMode.NEAREST),
                                                  self.grid.coord_to_track(ym_layer, comp.bound_box.xl,
                                                                           RoundMode.NEAREST),
                                                  logic.bound_box.yh, buf_n.bound_box.yl,
                                                  tr_w_clk_ym, tr_manager.get_sep(ym_layer, ('clk', 'clk')))
        hold_ym = self.connect_to_tracks(comp.get_all_port_pins('start'),
                                         TrackID(ym_layer, avail_ym_locs[-1], tr_w_clk_ym))
        self.connect_to_track_wires(hold_ym, cdac_hold_xm1)
        hold_xm1 = self.connect_to_tracks(hold_ym, TrackID(xm1_layer, logic_rst_xm1_locs[1], tr_w_clk_xm1),
                                          track_lower=self.bound_box.xl)
        rst_xm1 = self.connect_to_tracks(logic.get_all_port_pins('rst'),
                                         TrackID(xm1_layer, logic_rst_xm1_locs[2], tr_w_clk_xm1))

        rst_xm1 = self.connect_wires(rst_xm1, lower=self.bound_box.xl)
        if sampler_params:
            sam_clk_ym_tidx = self.grid.coord_to_track(ym_layer, sampler.get_pin('sam_e_lower', layer=xm_layer).middle,
                                                       RoundMode.NEAREST)
            sam_clk_ym = self.connect_to_tracks(sampler.get_pin('sam_e_lower', layer=xm_layer),
                                                TrackID(ym_layer, sam_clk_ym_tidx, tr_w_clk_ym, grid=self.grid))
            rst_xm_comp_top = self.connect_to_tracks(sam_clk_ym,
                                                     TrackID(xm_layer, rst_xm_comp_top_tidx, tr_w_clk_xm))
            rst_ym = self.connect_to_tracks(rst_xm_comp_top,
                                            TrackID(ym_layer, avail_ym_locs[-2], tr_w_clk_ym))
            self.connect_to_track_wires(rst_xm1, rst_ym)

        # export clock signal
        # clock for logic

        # #TODO: debug
        # self.add_pin('in_n', [cdac_n.get_pin('in'), cdac_p.get_pin('in_c')])
        # self.add_pin('in_p', [cdac_p.get_pin('in'), cdac_n.get_pin('in_c')])

        # Logic clock
        tr_w_xm1_ana_sig = tr_manager.get_width(xm1_layer, 'ana_sig')
        comp_out_mid_coord = (comp.bound_box.yl + logic.bound_box.yh) // 2
        comp_out_xm1_tidx = self.grid.coord_to_track(xm1_layer, comp_out_mid_coord, RoundMode.NEAREST)
        comp_sep_xm1 = self.get_track_sep(xm1_layer, tr_w_xm1_ana_sig, 1)
        self.connect_differential_wires(logic.get_all_port_pins('comp_p'), logic.get_all_port_pins('comp_n'),
                                        WireArray.list_to_warr(comp.get_all_port_pins('outp')),
                                        WireArray.list_to_warr(comp.get_all_port_pins('outn')))

        logic_xm1_clk_tidx = self.grid.coord_to_track(xm1_layer, comp.bound_box.yl, RoundMode.NEAREST)
        self.connect_to_tracks([comp.get_pin('logic_clk'), logic.get_pin('sar_clk')],
                               TrackID(xm1_layer, logic_xm1_clk_tidx, tr_w_clk_xm1))

        # Connect comp stop signal down
        comp_stop_xm = comp.get_pin('stop')
        logic_stop_ym = logic.get_pin('state<0>')
        stop_ym_tidx_l = self.grid.coord_to_track(ym_layer, comp.bound_box.xl, RoundMode.GREATER_EQ)
        stop_ym_tidx_h = self.grid.coord_to_track(ym_layer, comp.bound_box.xh, RoundMode.LESS_EQ)
        stop_avail_ym_locs = self.get_available_tracks(ym_layer, stop_ym_tidx_l, stop_ym_tidx_h,
                                                       comp.bound_box.yl, comp_stop_xm.bound_box.yl,
                                                       width=tr_w_clk_ym,
                                                       sep=tr_manager.get_sep(ym_layer, ('clk', 'dum')))
        _len_ym = len(stop_avail_ym_locs)
        comp_stop_ym_tidx = min(stop_avail_ym_locs,
                                key=lambda tid: abs(self.grid.track_to_coord(ym_layer, tid) - comp_stop_xm.middle))
        comp_stop_ym = self.connect_to_tracks(comp_stop_xm, TrackID(ym_layer, comp_stop_ym_tidx, tr_w_clk_ym))
        stop_xm1_tidx = tr_manager.get_next_track(xm1_layer, logic_xm1_clk_tidx, 'clk', 'clk')
        tr_w_dig_xm1 = tr_manager.get_width(xm1_layer, 'dig')
        self.connect_to_tracks([logic_stop_ym, comp_stop_ym], TrackID(xm1_layer, stop_xm1_tidx, tr_w_dig_xm1))
        # self.connect_to_track_wires(comp.get_pin('stop'), logic.get_pin('state<0>'))
        # ==== connect cdac top to bottom ====
        export_cdac_top_to_bot = True
        if export_cdac_top_to_bot:
            cdac_top_n = cdac_n.get_pin('top', layer=ym1_layer)
            cdac_top_p = cdac_p.get_pin('top', layer=ym1_layer)
            export_cdac_hor_tidx = self.grid.coord_to_track(cdac_top_n.layer_id + 1,
                                                            (cdac_n.bound_box.yl + comp.bound_box.yh) // 2,
                                                            RoundMode.NEAREST)
            export_cdac_vert_mid_tidx = self.grid.coord_to_track(cdac_top_n.layer_id + 2,
                                                                 cdac_n.bound_box.xh, RoundMode.NEAREST)
            tr_w_cap_exp_hor = tr_manager.get_width(cdac_top_n.layer_id + 1, 'cap')
            tr_w_cap_exp_ver = tr_manager.get_width(cdac_top_n.layer_id + 2, 'cap')
            export_cdac_vert_sp = self.get_track_sep(cdac_top_n.layer_id + 2, tr_w_cap_exp_ver, tr_w_cap_exp_ver).div2()
            vert_tidx_n = export_cdac_vert_mid_tidx - export_cdac_vert_sp
            vert_tidx_p = export_cdac_vert_mid_tidx + export_cdac_vert_sp

            cdac_top_n = self.connect_to_tracks(cdac_top_n, TrackID(cdac_top_n.layer_id + 1,
                                                                    export_cdac_hor_tidx, tr_w_cap_exp_hor))
            cdac_top_p = self.connect_to_tracks(cdac_top_p, TrackID(cdac_top_p.layer_id + 1,
                                                                    export_cdac_hor_tidx, tr_w_cap_exp_hor))
            self.connect_to_tracks(cdac_top_n, TrackID(cdac_top_n.layer_id + 1, vert_tidx_n, tr_w_cap_exp_ver),
                                   track_lower=self.bound_box.yl)
            self.connect_to_tracks(cdac_top_p, TrackID(cdac_top_n.layer_id + 1, vert_tidx_p, tr_w_cap_exp_ver),
                                   track_lower=self.bound_box.yl)
            self.add_pin('cdac_top_n', cdac_top_n, hide=True)
            self.add_pin('cdac_top_p', cdac_top_p, hide=True)
        basefill_buffer_gap = True
        # Basefill
        if basefill_buffer_gap:
            side_margin = logic_x
            buf_master: IntegrationWrapper = cast(IntegrationWrapper, buf_n.master)
            mosbase_margin = buf_n.bound_box.w - buf_master.core.core_bound_box.w
            num_cols_basefill = (buf_n.bound_box.xl - mosbase_margin - side_margin) // buf_master.core.core.sd_pitch
            basefill_pinfo = buf_master.core.core.draw_base_info
            cap_basefill_master = \
                self.new_template(GenericWrapper,
                                  params=dict(cls_name=MOSBaseFiller.get_qualified_name(),
                                              params=dict(pinfo=basefill_pinfo, num_cols=num_cols_basefill,
                                                          num_tiles=buf_master.core.core.num_tile_rows)))

            self.add_instance(cap_basefill_master, xform=Transform(buf_n.bound_box.xl, buf_n.bound_box.yl,
                                                                   Orientation.MY))
            self.add_instance(cap_basefill_master, xform=Transform(buf_p.bound_box.xh, buf_n.bound_box.yl,
                                                                   Orientation.R0))
            comp_logic_bbox = BBox(side_margin, logic.bound_box.yh, self.bound_box.xh - side_margin, comp.bound_box.yl)
            basefill_bbox(self, comp_logic_bbox)
            comp_logic_bbox = BBox(side_margin, max(comp.bound_box.yh, buf_n.bound_box.yh),
                                   self.bound_box.xh - side_margin, cap_y)
            basefill_bbox(self, comp_logic_bbox)

        if sampler_params:
            self.add_pin('in_n', sampler.get_pin('sig_n', layer=cap_in_layer + 2), mode=PinMode.UPPER)
            self.add_pin('in_p', sampler.get_pin('sig_p', layer=cap_in_layer + 2), mode=PinMode.UPPER)

            ym_clk_list = [sampler.get_pin('sam', layer=ym_layer), sampler.get_pin('sam_e', layer=ym_layer),
                           sampler.get_pin('sam_b', layer=ym_layer), sampler.get_pin('sam_e_b', layer=ym_layer)]
            xm1_clk_list = []
            for ym_clk in ym_clk_list:
                tidx = self.grid.coord_to_track(xm1_layer, ym_clk.middle, RoundMode.NEAREST)
                xm1_clk_list.append(self.connect_to_tracks(ym_clk, TrackID(xm1_layer, tidx, tr_w_clk_xm1),
                                                           track_lower=self.bound_box.xl,
                                                           track_upper=self.bound_box.xh))
            self.add_pin('clk', xm1_clk_list[0], mode=PinMode.LOWER)
            self.add_pin('clk_e', xm1_clk_list[1], mode=PinMode.LOWER)
            self.add_pin('clk_b', xm1_clk_list[2], mode=PinMode.LOWER)
            self.add_pin('clk_e_b', xm1_clk_list[3], mode=PinMode.LOWER)
        else:
            self.reexport(cdac_p.get_port('in'), net_name='sig_p', connect=True)
            self.reexport(cdac_n.get_port('in'), net_name='sig_n', connect=True)
            self.reexport(cdac_p.get_port('in_c'), net_name='sig_n', connect=True)
            self.reexport(cdac_n.get_port('in_c'), net_name='sig_p', connect=True)
            self.reexport(cdac_p.get_port('sam'), net_name='vg_p')
            self.reexport(cdac_n.get_port('sam'), net_name='vg_n')

        self.add_pin('done', logic_stop_ym)
        self.add_pin('clk_sel', comp.get_pin('ctrl_ext_clk'))
        self.add_pin('comp_clk', comp.get_pin('comp_clk'))
        self.add_pin('comp_m_n', comp.get_pin('outn_m'))
        self.add_pin('comp_m_p', comp.get_pin('outp_m'))
        self.add_pin('clk_e' if sampler_params else 'sam_e', rst_xm1, mode=PinMode.LOWER)
        self.add_pin('hold', hold_xm1, mode=PinMode.LOWER)
        self.reexport(cdac_p.get_port('top'), net_name='top_p')
        self.reexport(cdac_n.get_port('top'), net_name='top_n')
        self.reexport(logic.get_port('clk_out'))
        self.reexport(comp.get_port('outp'), net_name='comp_p')
        self.reexport(comp.get_port('outn'), net_name='comp_n')
        self.reexport(comp.get_port('osp'))
        self.reexport(comp.get_port('osn'))

        ####################
        # Connet supply all together
        ####################
        # Connect supplies together
        cdac_top_layer = cdac_n.master.top_layer
        cdac_vdd_top = cdac_n.get_all_port_pins('VDD', cdac_top_layer) + cdac_p.get_all_port_pins('VDD', cdac_top_layer)
        cdac_vss_top = cdac_n.get_all_port_pins('VSS', cdac_top_layer) + cdac_p.get_all_port_pins('VSS', cdac_top_layer)
        cdac_vss_top = self.extend_wires(cdac_vss_top, lower=self.bound_box.yl, upper=self.bound_box.yh)
        cdac_vdd_top = self.extend_wires(cdac_vdd_top, lower=self.bound_box.yl, upper=self.bound_box.yh)
        # Always connect from cdac as it's very regular
        buf_comp_sup_top = max(comp_master.core_supply_layer, logic_buf_master.core_supply_layer)
        # -- connect buffers to comparator
        buf_top_sup_layer = logic_buf_master.top_layer
        bufn_dict, bufp_dict = {}, {}
        bufn_dict[buf_top_sup_layer] = {'VDD': buf_n.get_all_port_pins('VDD', layer=buf_top_sup_layer),
                                        'VSS': buf_n.get_all_port_pins('VSS', layer=buf_top_sup_layer)}
        bufp_dict[buf_top_sup_layer] = {'VDD': buf_p.get_all_port_pins('VDD', layer=buf_top_sup_layer),
                                        'VSS': buf_p.get_all_port_pins('VSS', layer=buf_top_sup_layer)}
        bufn_bbox, bufp_bbox = buf_n.bound_box, buf_p.bound_box
        for lay_id in range(buf_top_sup_layer + 1, cdac_top_layer):
            if self.grid.get_direction(lay_id) == Orient2D.x and lay_id <= comp_master.top_layer:
                _vdd = self.connect_to_track_wires(bufn_dict[lay_id - 1]['VDD'] + bufp_dict[lay_id - 1]['VDD'],
                                                   comp.get_all_port_pins('VDD', layer=lay_id))
                _vss = self.connect_to_track_wires(bufn_dict[lay_id - 1]['VSS'] + bufp_dict[lay_id - 1]['VSS'],
                                                   comp.get_all_port_pins('VSS', layer=lay_id))
                bufn_dict[lay_id] = {'VDD': _vdd, 'VSS': _vss}
                bufp_dict[lay_id] = {'VDD': _vdd, 'VSS': _vss}
            else:
                _vdd, _vss = self.connect_supply_warr(tr_manager,
                                                      [bufn_dict[lay_id - 1]['VDD'], bufn_dict[lay_id - 1]['VSS']],
                                                      lay_id - 1, bufn_bbox)
                bufn_dict[lay_id] = {'VDD': _vdd, 'VSS': _vss}
                _vdd, _vss = self.connect_supply_warr(tr_manager,
                                                      [bufp_dict[lay_id - 1]['VDD'], bufp_dict[lay_id - 1]['VSS']],
                                                      lay_id - 1, bufp_bbox, align_upper=True)
                bufp_dict[lay_id] = {'VDD': _vdd, 'VSS': _vss}

        last_vdd, last_vss = bufn_dict[cdac_top_layer - 1]['VDD'] + bufp_dict[cdac_top_layer - 1]['VDD'], \
                             bufn_dict[cdac_top_layer - 1]['VSS'] + bufp_dict[cdac_top_layer - 1]['VSS']

        self.connect_to_track_wires(last_vdd, cdac_vdd_top)
        self.connect_to_track_wires(last_vss, cdac_vss_top)

        if sampler_params:
            self.connect_to_track_wires(sampler.get_all_port_pins('VDD', sampler_master.top_layer), cdac_vdd_top)
            self.connect_to_track_wires(sampler.get_all_port_pins('VSS', sampler_master.top_layer), cdac_vss_top)

        self.connect_to_track_wires(logic.get_all_port_pins('VDD', logic_master.top_layer), cdac_vdd_top)
        self.connect_to_track_wires(logic.get_all_port_pins('VSS', logic_master.top_layer), cdac_vss_top)

        topm_layer_vdd = logic.get_all_port_pins('VDD', logic_master.top_layer) + last_vdd
        topm_layer_vss = logic.get_all_port_pins('VSS', logic_master.top_layer) + last_vss
        if sampler_params:
            topm_layer_vdd += sampler.get_all_port_pins('VDD', sampler_master.top_layer)
            topm_layer_vss += sampler.get_all_port_pins('VSS', sampler_master.top_layer)

        self.extend_wires(topm_layer_vdd, lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.extend_wires(topm_layer_vss, lower=self.bound_box.xl, upper=self.bound_box.xh)

        self.add_pin('VDD', cdac_vdd_top)
        self.add_pin('VSS', cdac_vss_top)
        if sampler_params:
            self.add_pin('vcm' if split_dac else 'vref<1>',
                         sampler.get_all_port_pins('vcm'), connect=True)
        self.add_pin('vref<1>', cdac_n.get_all_port_pins('vref<1>'), connect=True)
        self.add_pin('vref<2>', cdac_n.get_all_port_pins('vref<2>'), connect=True)
        self.add_pin('vref<0>', cdac_n.get_all_port_pins('vref<0>'), connect=True)
        self.add_pin('vref<1>', cdac_p.get_all_port_pins('vref<1>'), connect=True)
        self.add_pin('vref<2>', cdac_p.get_all_port_pins('vref<2>'), connect=True)
        self.add_pin('vref<0>', cdac_p.get_all_port_pins('vref<0>'), connect=True)

        sar_params = dict(
            nbits=nbits,
            comp=comp_master.sch_params,
            logic=logic_master.sch_params,
            cdac=cdac_master.sch_params,
            buf=logic_buf_master.sch_params,
            tri_sa=True,
            split_dac=split_dac,
        )

        if sampler_params:
            self._sch_params = dict(
                slice_params=sar_params,
                sampler_params=sampler_master.sch_params,
                rev=True,
            )
        else:
            self._sch_params = sar_params


class SARWrap(SARSlice):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = SARSlice.get_params_info()
        ans['num_vert_sup_side'] = 'Number of side supplies around core layout'
        return ans

    def draw_layout(self) -> None:
        sar_templates: SARSlice = self.new_template(SARSlice, params=self.params)
        tr_manager = sar_templates.tr_manager

        num_vert_sup_side = self.params['num_vert_sup_side']
        top_layer = sar_templates.top_layer
        vert_sup_l_ntr, vert_sup_l_locs = \
            tr_manager.place_wires(top_layer - 1, ['sup'] + ['sup'] * num_vert_sup_side + ['dum'])  # last one for vcm
        vcm_vert_locs = [vert_sup_l_locs[0]]
        vert_sup_l_locs = vert_sup_l_locs[1:-1]
        x_sar = self.grid.track_to_coord(top_layer - 1, vert_sup_l_ntr)
        sar = self.add_instance(sar_templates, xform=Transform(x_sar, 0, Orientation.R0))
        for port in sar.port_names_iter():
            self.reexport(sar.get_port(port))

        bbox = sar.bound_box
        bbox.extend(x=0)
        bbox.extend(x=bbox.xh + x_sar)

        self.set_size_from_bound_box(top_layer, bbox)

        vert_sup_r_start_tidx = self.grid.coord_to_track(top_layer - 1, bbox.xh, RoundMode.NEAREST)
        # hack: halfint 1 to compensate the offset
        _, vert_sup_r_locs = \
            tr_manager.place_wires(top_layer - 1, ['dum'] + ['sup'] * num_vert_sup_side + ['sup'],
                                   align_idx=-1, align_track=vert_sup_r_start_tidx - HalfInt(1))  # last one for vcm
        vcm_vert_locs.append(vert_sup_r_locs[-1])
        vert_sup_r_locs = vert_sup_r_locs[1:-1]
        vdd_vert_locs = vert_sup_l_locs[1::2] + vert_sup_r_locs[::2]
        vss_vert_locs = vert_sup_l_locs[::2] + vert_sup_r_locs[1::2]

        tr_w_sup_lower = tr_manager.get_width(top_layer - 1, 'sup')
        vdd_vert = [self.add_wires(top_layer - 1, tid, lower=self.bound_box.yl, upper=self.bound_box.yh,
                                   width=tr_w_sup_lower) for tid in vdd_vert_locs]
        vss_vert = [self.add_wires(top_layer - 1, tid, lower=self.bound_box.yl, upper=self.bound_box.yh,
                                   width=tr_w_sup_lower) for tid in vss_vert_locs]
        vdd_top = sar.get_all_port_pins('VDD', top_layer)
        vss_top = sar.get_all_port_pins('VSS', top_layer)

        vcm_vert = [self.add_wires(top_layer - 1, tid, lower=self.bound_box.yl, upper=self.bound_box.yh,
                                   width=tr_w_sup_lower) for tid in vcm_vert_locs]

        self.connect_to_track_wires(sar.get_all_port_pins('vref<1>'), vcm_vert)

        for vdd, vss in zip(vdd_vert, vss_vert):
            self.connect_differential_wires(vdd_top, vss_top, vdd, vss)

        self._sch_params = sar_templates.sch_params
