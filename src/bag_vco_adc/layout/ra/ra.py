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


from typing import Any, Dict, Union, Mapping, Tuple, Optional, Type

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.layout.util import IPMarginTemplate
from bag.util.immutable import Param
from bag.util.math import HalfInt
from pybag.core import Transform, BBox
from pybag.enum import Orientation, Orient2D, RoundMode, PinMode
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo
from .ra_bias import RABiasTop
from .ra_cap import MOMCapOnMOS
from .ra_ringamp import RingAmpDCCMFB
from ..sah.sar_samp import CMSwitch
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID
from ..util.wrapper import GenericWrapper, IntegrationWrapper


class RACMFB(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_cmfb_top')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cm_ss_cap_params='',
            cm_fb_cap_params='',
            cm_sw_params='',
            dc_cmfb_params='',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
        )

    @staticmethod
    def place_on_track(x: int, y: int, w_blk: int, h_blk: int):
        return -(-x // w_blk) * w_blk, -(-y // h_blk) * h_blk

    def draw_layout(self) -> None:
        cm_ss_cap_params: dict = self.params['cm_ss_cap_params']
        cm_fb_cap_params: dict = self.params['cm_fb_cap_params']
        dc_cmfb_params: dict = self.params['dc_cmfb_params']
        cm_sw_params: dict = self.params['cm_sw_params']

        if isinstance(cm_sw_params, str):
            spec_yaml = read_yaml(cm_sw_params)
            cm_sw_params = spec_yaml['params']['params']

        cap_fb_gen_params = dict(
            cls_name=MOMCapOnMOS.get_qualified_name(),
            export_private=False,
            params=cm_fb_cap_params
        )

        cap_ss_gen_params = dict(
            cls_name=MOMCapOnMOS.get_qualified_name(),
            export_private=False,
            params=cm_ss_cap_params
        )
        dc_cmfb_gen_params = dict(
            cls_name=RingAmpDCCMFB.get_qualified_name(),
            export_private=False,
            export_hidden=True,
            params=dc_cmfb_params
        )
        cap_ss_master: GenericWrapper = self.new_template(GenericWrapper, params=cap_ss_gen_params)
        cap_fb_master: GenericWrapper = self.new_template(GenericWrapper, params=cap_fb_gen_params)
        dc_cmfb_master: GenericWrapper = self.new_template(GenericWrapper, params=dc_cmfb_gen_params)
        dc_ncols = dc_cmfb_master.core.num_cols
        # cm_sw_params_new = cm_sw_params.copy(append=dict(ncols_tot=dc_ncols - 2 * dc_cmfb_master.core.min_sep_col))
        cm_sw_params['ncols_tot'] = dc_ncols
        cm_sw_gen_params = dict(
            cls_name=CMSwitch.get_qualified_name(),
            params=cm_sw_params
        )
        cm_sw_master: GenericWrapper = self.new_template(GenericWrapper, params=cm_sw_gen_params)

        conn_layer = cm_sw_master.core.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # ----- floorplan -----
        top_layer = max(cap_ss_master.top_layer, cap_fb_master.top_layer, dc_cmfb_master.top_layer,
                        cm_sw_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        w_cap_ss, h_cap_ss = cap_ss_master.bound_box.w, cap_ss_master.bound_box.h
        w_cap_fb, h_cap_fb = cap_fb_master.bound_box.w, cap_fb_master.bound_box.h
        w_dc_fb, h_dc_fb = dc_cmfb_master.bound_box.w, dc_cmfb_master.bound_box.h
        w_cm_sw, h_cm_sw = cm_sw_master.bound_box.w, cm_sw_master.bound_box.h

        # ----- add blocks -----
        cap_fb_n_x = 0
        cap_fb_n_y = 0
        cap_fb_n_x, cap_fb_n_y = self.place_on_track(cap_fb_n_x, cap_fb_n_y, w_blk, h_blk)
        cap_fb_n = self.add_instance(cap_fb_master, xform=Transform(cap_fb_n_x, cap_fb_n_y, Orientation.R0))
        cap_fb_p_x, cap_fb_p_y = self.place_on_track(
            2 * cap_fb_master.bound_box.w - 2 * cap_fb_master.core_bound_box.xl, cap_fb_n_y, w_blk, h_blk)
        cap_fb_p = self.add_instance(cap_fb_master, xform=Transform(cap_fb_p_x, cap_fb_p_y, Orientation.MY))

        cap_ss_n_x, cap_ss_n_y = self.place_on_track(0, cap_fb_master.bound_box.h + h_cap_ss, w_blk, h_blk)
        cap_ss_n = self.add_instance(cap_ss_master, xform=Transform(cap_ss_n_x, cap_ss_n_y, Orientation.MX))
        cap_ss_p_x, cap_ss_p_y = self.place_on_track(
            2 * cap_ss_master.bound_box.w - 2 * cap_fb_master.core_bound_box.xl,
            cap_fb_master.bound_box.h + h_cap_ss, w_blk, h_blk)
        cap_ss_p = self.add_instance(cap_ss_master, xform=Transform(cap_ss_p_x, cap_ss_p_y, Orientation.R180))

        cm_sw_x = cap_fb_p.bound_box.xh
        cm_sw_y = 0
        cm_sw_x, cm_sw_y = self.place_on_track(cm_sw_x, cm_sw_y, w_blk, h_blk)
        cm_sw = self.add_instance(cm_sw_master, xform=Transform(cm_sw_x, cm_sw_y, Orientation.R0))

        cm_dc_x, cm_dc_y = self.place_on_track(cap_ss_p.bound_box.xh, cm_sw.bound_box.yh, w_blk, h_blk)
        cm_dc = self.add_instance(dc_cmfb_master, xform=Transform(cm_dc_x, cm_dc_y, Orientation.R0))

        cm_dc_fill_params = dc_cmfb_params['cap_params'].to_dict().copy()
        cm_dc_fill_params['ncol_tot'] = cap_fb_master.core.num_cols * 2
        cm_dc_fill_params['num_tiles'] = cm_dc_fill_params['ntiles_fill']
        cm_dc_fill_params['pinfo'] = dc_cmfb_params['pinfo']
        cm_dc_fill_gen_params = dict(
            cls_name=MOMCapOnMOS.get_qualified_name(),
            params=cm_dc_fill_params,
            export_private=False,
        )
        cap_dc_fill_master: GenericWrapper = self.new_template(GenericWrapper, params=cm_dc_fill_gen_params)
        cap_dc_fill = self.add_instance(cap_dc_fill_master, xform=Transform(0, cap_ss_p.bound_box.yh, Orientation.R0))

        bbox_w = max(cm_sw.bound_box.xh, cm_dc.bound_box.xh)
        bbox_h = max(cap_ss_n.bound_box.yh, cm_dc.bound_box.yh)
        bbox_w = -(-bbox_w // w_blk) * w_blk
        bbox_w = -(-bbox_w // w_blk) * w_blk
        bbox = BBox(0, 0, bbox_w, bbox_h)
        self.set_size_from_bound_box(top_layer, bbox)
        # for inst in [cap_fb_n, cap_fb_p, cap_ss_n, cap_ss_p, cm_sw, cm_dc]:
        #     for pin in inst.port_names_iter():
        #         self.reexport(inst.get_port(pin), connect=True)

        tr_manager = TrackManager(self.grid, tr_spaces=self.params['tr_spaces'], tr_widths=self.params['tr_widths'])
        ra_in_mid_locs = self.grid.coord_to_track(xm1_layer, cap_fb_n.bound_box.yh, RoundMode.NEAREST)
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        ra_in_xm1_tid_sep = self.get_track_sep(xm1_layer, tr_w_sig_xm1, tr_w_sig_xm1).div2(True)
        ra_in_xm1_locs = [ra_in_mid_locs + ra_in_xm1_tid_sep, ra_in_mid_locs - ra_in_xm1_tid_sep]

        cap_n_bbox_list = [cap_ss_n.get_pin('plus', layer=ym_layer), cap_fb_n.get_pin('plus', layer=ym_layer)]
        cap_p_bbox_list = [cap_ss_p.get_pin('plus', layer=ym_layer), cap_fb_p.get_pin('plus', layer=ym_layer)]
        ra_inn_xm1 = self.connect_to_tracks(cap_n_bbox_list, TrackID(xm1_layer, ra_in_xm1_locs[0], tr_w_sig_xm1,
                                                                     grid=self.grid))
        ra_inp_xm1 = self.connect_to_tracks(cap_p_bbox_list, TrackID(xm1_layer, ra_in_xm1_locs[1], tr_w_sig_xm1,
                                                                     grid=self.grid))
        ra_inn_xm1 = self.extend_wires(ra_inn_xm1, lower=cap_fb_n.bound_box.xl, upper=cap_fb_p.bound_box.xh)
        ra_inp_xm1 = self.extend_wires(ra_inp_xm1, lower=cap_fb_n.bound_box.xl, upper=cap_fb_p.bound_box.xh)

        tr_w_cap_ym = tr_manager.get_width(ym_layer, 'cap')
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        dc_cap_in_ym_tidx = self.grid.coord_to_track(ym_layer, cap_fb_n.bound_box.xh - cap_fb_master.core_bound_box.xl,
                                                     RoundMode.NEAREST)
        dc_cap_in_ym = self.connect_to_tracks([cap_ss_n.get_pin('minus', layer=xm_layer),
                                               cap_ss_p.get_pin('minus', layer=xm_layer)],
                                              TrackID(ym_layer, dc_cap_in_ym_tidx, tr_w_cap_ym, grid=self.grid))
        self.connect_to_track_wires(dc_cap_in_ym, cm_dc.get_all_port_pins('in'))

        # Connect output
        dc_cmfb_out = cm_dc.get_pin('out')
        self.connect_to_track_wires(dc_cmfb_out, cm_sw.get_pin('sig'))
        cmfb_xm1_tidx = self.grid.coord_to_track(xm1_layer, self.bound_box.yl, RoundMode.NEAREST)
        cmfb_xm1_tidx = cmfb_xm1_tidx - self.get_track_sep(xm1_layer, tr_w_sig_xm1, 1)
        cmfb_xm1 = self.connect_to_tracks(dc_cmfb_out, TrackID(xm1_layer, cmfb_xm1_tidx, tr_w_sig_xm1, grid=self.grid),
                                          track_lower=self.bound_box.xl)

        self.connect_to_track_wires(cap_fb_n.get_pin('minus', layer=ym_layer), cmfb_xm1)
        self.connect_to_track_wires(cap_fb_p.get_pin('minus', layer=ym_layer), cmfb_xm1)

        _, ctrl_ym_locs = tr_manager.place_wires(ym_layer, ['ctrl'] * 2, center_coord=cm_dc.bound_box.xh)
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        phi_a, phi_ab = self.connect_matching_tracks([cm_dc.get_pin('phi_a'), cm_dc.get_pin('phi_ab')],
                                                     ym_layer, ctrl_ym_locs, width=tr_w_ctrl_ym,
                                                     track_upper=self.bound_box.yh)
        self.connect_to_track_wires(phi_a, cm_sw.get_pin('sam_b'))
        self.connect_to_track_wires(phi_ab, cm_sw.get_pin('sam'))

        cm_sw_vss_ym_tidx = self.grid.coord_to_track(ym_layer, cm_sw.get_pin('VSS').upper, RoundMode.NEAREST)
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')

        # Connect to filling dc cap
        fill_cap_ym_tidx = self.grid.coord_to_track(ym_layer, cm_dc.bound_box.xl, RoundMode.NEAREST)
        tr_w_cap_ym = tr_manager.get_width(ym1_layer, 'cap')
        self.connect_to_tracks([cm_dc.get_pin('plus', layer=xm_layer), cap_dc_fill.get_pin('plus', layer=xm_layer)],
                               TrackID(ym_layer, fill_cap_ym_tidx, tr_w_cap_ym))
        self.connect_differential_wires(cm_dc.get_all_port_pins('VDD', layer=ym_layer),
                                        cm_dc.get_all_port_pins('VSS', layer=ym_layer),
                                        cm_sw.get_pin('VDD'), cm_sw.get_pin('VSS'))
        cm_sw_vss_xm1_coord = self.grid.track_to_coord(xm1_layer, cm_sw.get_pin('VSS').track_id.base_index)
        cm_sw_vss_xm1_tidx = self.grid.coord_to_track(xm1_layer, cm_sw_vss_xm1_coord, RoundMode.NEAREST)
        cm_sw_vdd_xm1_coord = self.grid.track_to_coord(xm1_layer, cm_sw.get_pin('VDD').track_id.base_index)
        cm_sw_vdd_xm1_tidx = self.grid.coord_to_track(xm1_layer, cm_sw_vdd_xm1_coord, RoundMode.NEAREST)
        cm_sw_vss_xm1 = self.connect_to_tracks(cm_dc.get_all_port_pins('VSS', layer=ym_layer),
                                               TrackID(xm1_layer, cm_sw_vss_xm1_tidx, tr_w_sup_xm1))
        cm_sw_vdd_xm1 = self.connect_to_tracks(cm_dc.get_all_port_pins('VDD', layer=ym_layer),
                                               TrackID(xm1_layer, cm_sw_vdd_xm1_tidx, tr_w_sup_xm1))

        cmdc_vdd_xm1_tidx = cm_dc.get_pin('VDD', layer=xm1_layer).track_id.base_index
        top_xm1_tidx = self.grid.coord_to_track(xm1_layer, self.bound_box.yh, RoundMode.LESS_EQ)
        tr_sp_sup_xm1 = tr_manager.get_sep(xm1_layer, ('sup', 'sup'))
        sup_xm1_l, sup_xm1_h = cm_sw_vdd_xm1[0].lower, cm_sw_vdd_xm1[0].upper
        xm1_list_r = self.get_available_tracks(xm1_layer, cmdc_vdd_xm1_tidx, top_xm1_tidx,
                                               cm_dc.bound_box.xl, cm_dc.bound_box.xh, tr_w_sup_xm1, tr_sp_sup_xm1,
                                               align_to_higer=True)
        xm1_list_l = self.get_available_tracks(xm1_layer, 0, top_xm1_tidx,
                                               cap_ss_p.bound_box.xl, cap_ss_p.bound_box.xh, tr_w_sup_xm1,
                                               tr_sp_sup_xm1,
                                               align_to_higer=True)
        vdd_xm1_list_r = [self.add_wires(xm1_layer, tid, width=tr_w_sup_xm1, lower=sup_xm1_l,
                                         upper=self.bound_box.xh) for tid in xm1_list_r[::2]]
        vss_xm1_list_r = [self.add_wires(xm1_layer, tid, width=tr_w_sup_xm1, lower=sup_xm1_l,
                                         upper=self.bound_box.xh) for tid in xm1_list_r[1::2]]

        vdd_xm1_list_l = [self.add_wires(xm1_layer, tid, width=tr_w_sup_xm1, lower=cap_ss_n.bound_box.xl,
                                         upper=cap_ss_p.bound_box.xh) for tid in xm1_list_l[::2]]
        vss_xm1_list_l = [self.add_wires(xm1_layer, tid, width=tr_w_sup_xm1, lower=cap_ss_n.bound_box.xl,
                                         upper=cap_ss_p.bound_box.xh) for tid in xm1_list_l[1::2]]
        _vdd_xm1 = self.connect_wires(vdd_xm1_list_r + vdd_xm1_list_l) + [cm_sw_vdd_xm1,
                                                                          cm_dc.get_pin('VDD', layer=xm1_layer)]
        _vss_xm1 = self.connect_wires(vss_xm1_list_r + vss_xm1_list_l) + [cm_sw_vss_xm1,
                                                                          cm_dc.get_pin('VSS', layer=xm1_layer)]
        self.connect_to_track_wires(vss_xm1_list_l[0], cap_dc_fill.get_pin('minus', layer=ym_layer))

        self.match_warr_length([cm_sw_vdd_xm1, cm_dc.get_pin('VDD', layer=xm1_layer),
                                cm_sw_vss_xm1, cm_dc.get_pin('VSS', layer=xm1_layer)])

        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        ym1_l_l = self.grid.coord_to_track(ym1_layer, cm_sw_vss_xm1[0].lower, RoundMode.GREATER_EQ)
        ym1_l_r = self.grid.coord_to_track(ym1_layer, cm_sw_vss_xm1[0].upper, RoundMode.LESS_EQ)
        sup_ym1_locs = self.get_tids_between(ym1_layer, ym1_l_l, ym1_l_r, tr_w_sup_ym1,
                                             sep=tr_manager.get_sep(ym1_layer, ('sup', 'sup')), sep_margin=0,
                                             include_last=True)
        vdd_ym1_r = [self.connect_to_tracks(vdd_xm1_list_r + [cm_sw_vdd_xm1, cm_dc.get_pin('VDD', layer=xm1_layer)],
                                            tid) for tid in sup_ym1_locs[::2]]
        vss_ym1_r = [self.connect_to_tracks(vss_xm1_list_r + [cm_sw_vss_xm1, cm_dc.get_pin('VSS', layer=xm1_layer)],
                                            tid) for tid in sup_ym1_locs[1::2]]

        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        ym1_l_l = self.grid.coord_to_track(ym1_layer, vdd_xm1_list_l[0].lower, RoundMode.GREATER_EQ)
        ym1_l_r = self.grid.coord_to_track(ym1_layer, vdd_xm1_list_l[0].upper, RoundMode.LESS_EQ)
        sup_ym1_locs = self.get_tids_between(ym1_layer, ym1_l_l, ym1_l_r, tr_w_sup_ym1,
                                             sep=tr_manager.get_sep(ym1_layer, ('sup', 'sup')), sep_margin=0,
                                             include_last=True)
        vdd_ym1_l = [self.connect_to_tracks(vdd_xm1_list_l, tid) for tid in sup_ym1_locs[::2]]
        vss_ym1_l = [self.connect_to_tracks(vss_xm1_list_l, tid) for tid in sup_ym1_locs[1::2]]
        vdd_ym1 = self.extend_wires(vdd_ym1_l + vdd_ym1_r, upper=self.bound_box.yh, lower=self.bound_box.yl)
        vss_ym1 = self.extend_wires(vss_ym1_l + vss_ym1_r, upper=self.bound_box.yh, lower=self.bound_box.yl)

        self.add_pin('VDD', vdd_ym1)
        self.add_pin('VSS', vss_ym1)
        self.add_pin('amp', phi_a, mode=PinMode.UPPER)
        self.add_pin('amp_b', phi_ab, mode=PinMode.UPPER)
        self.add_pin('v_cmfb', cmfb_xm1)
        self.add_pin('ra_out_n', ra_inn_xm1)
        self.add_pin('ra_out_p', ra_inp_xm1)
        self.reexport(cm_sw.get_port('ref'), net_name='vcm_ext')
        self.add_pin('v_cmss', dc_cap_in_ym)

        self._sch_params = dict(
            dc_params=dc_cmfb_master.sch_params.copy(append=dict(cap_fill_params=cap_dc_fill_master.sch_params)),
            cm_sw_params=cm_sw_master.sch_params,
            cap_fb_params=cap_fb_master.sch_params,
            cap_ss_params=cap_ss_master.sch_params,
        )


class RASWTop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._bias_ycoord = 0

    @property
    def bias_ycoord(self):
        return self._bias_ycoord

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_core')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            ra_core_params='',
            sw_out_params='',
            sw_mid_params='',
            sam_cap_params='',
            fb_cap_params='',
            # gain_cap_params='',
            tr_widths='',
            tr_spaces='',
            flip_cap='',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(flip_cap=True)

    @staticmethod
    def place_on_track(x: int, y: int, w_blk: int, h_blk: int):
        return -(-x // w_blk) * w_blk, -(-y // h_blk) * h_blk

    def draw_layout(self) -> None:
        flip_cap: bool = self.params['flip_cap']
        ra_core_params: dict = self.params['ra_core_params']
        sw_out_params: dict = self.params['sw_out_params']
        sw_mid_params: dict = self.params['sw_mid_params']
        sam_cap_params: dict = self.params['sam_cap_params']
        fb_cap_params: dict = self.params['fb_cap_params']
        # gain_cap_params: Param = self.params['gain_cap_params']

        if isinstance(ra_core_params, str):
            spec_yaml = read_yaml(ra_core_params)
            ra_core_params = spec_yaml['params']
            ra_core_params.pop('top_sup_layer')
        if isinstance(sw_mid_params, str):
            spec_yaml = read_yaml(sw_mid_params)
            sw_mid_params = spec_yaml['params']
        if isinstance(sw_out_params, str):
            spec_yaml = read_yaml(sw_out_params)
            sw_out_params = spec_yaml['params']
        if isinstance(sam_cap_params, str):
            spec_yaml = read_yaml(sam_cap_params)
            sam_cap_params = spec_yaml['params']
        if isinstance(fb_cap_params, str):
            spec_yaml = read_yaml(fb_cap_params)
            fb_cap_params = spec_yaml['params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        self._tr_manager = tr_manager

        for params in [sw_out_params, sw_mid_params, sam_cap_params]:
            params['export_private'] = False
            params['export_hidden'] = True
        ra_core_master = self.new_template(IntegrationWrapper, params=ra_core_params)
        sw_out_master = self.new_template(GenericWrapper, params=sw_out_params)
        sw_mid_master = self.new_template(GenericWrapper, params=sw_mid_params)
        sam_cap_master = self.new_template(GenericWrapper, params=sam_cap_params)
        fb_cap_master = self.new_template(IPMarginTemplate, params=fb_cap_params)

        # --- Placement --- #

        conn_layer = ra_core_master.core.core.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1
        tr_w_xm_sig = tr_manager.get_width(xm_layer, 'sig')
        tr_w_xm1_sig = tr_manager.get_width(xm1_layer, 'sig')
        y_rt_margin_ntr_xm = 2 * self.get_track_sep(xm_layer, 1, tr_w_xm_sig)
        y_rt_margin_ntr2_xm1 = 4 * self.get_track_sep(xm1_layer, 1, tr_w_xm1_sig)
        y_margin_xm = self.grid.track_to_coord(xm_layer, y_rt_margin_ntr_xm)
        y_margin2_xm1 = self.grid.track_to_coord(xm1_layer, y_rt_margin_ntr2_xm1)

        top_layer = ym1_layer
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        w_ra, h_ra = ra_core_master.bound_box.w, ra_core_master.bound_box.h
        w_sam_cap, h_sam_cap = sam_cap_master.bound_box.w, sam_cap_master.bound_box.h
        w_fb_cap, h_fb_cap = fb_cap_master.bound_box.w, fb_cap_master.bound_box.h
        w_sw_out, h_sw_out = sw_out_master.bound_box.w, sw_out_master.bound_box.h
        w_sw_mid, h_sw_mid = sw_mid_master.bound_box.w, sw_mid_master.bound_box.h

        mid_max_width = max(w_ra, 2 * w_fb_cap, 2 * w_sam_cap, w_sw_out, w_sw_mid)

        bnd_width = ra_core_master.core.bound_box.w - ra_core_master.core.core.bound_box.w
        ncols_mid_tot = (mid_max_width - bnd_width) // ra_core_master.core.core.sd_pitch

        ra_core_params['params']['params']['ncols_tot'] = ncols_mid_tot
        sw_out_params['params']['ncols_tot'] = ncols_mid_tot
        sw_mid_params['params']['ncols_tot'] = ncols_mid_tot
        ra_core_master = self.new_template(IntegrationWrapper, params=ra_core_params)
        sw_mid_master = self.new_template(GenericWrapper, params=sw_mid_params)
        w_ra, h_ra = ra_core_master.bound_box.w, ra_core_master.bound_box.h
        w_sw_mid, h_sw_mid = sw_mid_master.bound_box.w, sw_mid_master.bound_box.h
        mid_max_width = max(w_ra, 2 * w_fb_cap, 2 * w_sam_cap, w_sw_out)

        # w_gain_cap, h_gain_cap = gain_cap_master.bound_box.w, gain_cap_master.bound_box.h
        # w_bias = max(w_bias, w_gain_cap) + x_margin
        fb_cap_n_x, fb_cap_n_y = self.place_on_track((mid_max_width - 2 * w_fb_cap) // 2, 0, blk_w, blk_h)
        fb_cap_n = self.add_instance(fb_cap_master, xform=Transform((mid_max_width - 2 * w_fb_cap) // 2,
                                                                    fb_cap_n_y, mode=Orientation.R0))
        fb_cap_p_x, fb_cap_p_y = self.place_on_track((mid_max_width + 2 * w_fb_cap) // 2, 0, blk_w, blk_h)
        fb_cap_p = self.add_instance(fb_cap_master, xform=Transform((mid_max_width + 2 * w_fb_cap) // 2,
                                                                    fb_cap_p_y, mode=Orientation.MY))

        core_x, core_y = self.place_on_track((mid_max_width + w_ra) // 2, fb_cap_n.bound_box.yh + y_margin2_xm1,
                                             blk_w, blk_h)
        core = self.add_instance(ra_core_master, xform=Transform(core_x, core_y, mode=Orientation.MY))

        sw_mid_x, sw_mid_y = self.place_on_track((mid_max_width - w_sw_mid) // 2, core.bound_box.yh + y_margin_xm,
                                                 blk_w, blk_h)
        sw_mid = self.add_instance(sw_mid_master, xform=Transform(sw_mid_x, sw_mid_y, mode=Orientation.R0))
        sam_cap_x, sam_cap_y = self.place_on_track(mid_max_width // 2,
                                                   sw_mid.bound_box.yh + y_margin_xm, blk_w, blk_h)
        sam_cap_n = self.add_instance(sam_cap_master,
                                      xform=Transform(mid_max_width // 2, sam_cap_y, mode=Orientation.MY))
        sam_cap_p = self.add_instance(sam_cap_master,
                                      xform=Transform(mid_max_width // 2, sam_cap_y, mode=Orientation.R0))

        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        sam_cap_n_top, sam_cap_p_top = sam_cap_n.get_pin('top'), sam_cap_p.get_pin('top')
        bnd_width_ym_tidx = self.grid.coord_to_track(ym_layer, bnd_width // 2, mode=RoundMode.NEAREST) - 0.5
        sw_out_n_tidx = sam_cap_n_top.track_id.base_index - self.get_track_sep(ym_layer, sam_cap_n_top.track_id.width,
                                                                               tr_w_sig_ym) - bnd_width_ym_tidx
        sw_out_p_tidx = sam_cap_p_top.track_id.base_index + self.get_track_sep(ym_layer, sam_cap_n_top.track_id.width,
                                                                               tr_w_sig_ym) - bnd_width_ym_tidx
        sw_out_sig_locs = {'sw_in_n': sw_out_n_tidx, 'sw_in_p': sw_out_p_tidx, }
        sw_out_params['params']['sig_locs'] = sw_out_sig_locs
        sw_out_master = self.new_template(GenericWrapper, params=sw_out_params)
        w_sw_out, h_sw_out = sw_out_master.bound_box.w, sw_out_master.bound_box.h
        sw_out_x, sw_out_y = self.place_on_track((mid_max_width - w_sw_out) // 2,
                                                 sam_cap_p.bound_box.yh + y_margin_xm, blk_w, blk_h)
        sw_out = self.add_instance(sw_out_master, xform=Transform(sw_out_x, sw_out_y, mode=Orientation.R0))

        blk_w, blk_h = self.grid.get_block_size(ym1_layer, half_blk_x=False, half_blk_y=False)
        tot_w = -(-mid_max_width // blk_w) * blk_w
        tot_h = -(-sw_out.bound_box.yh // blk_h) * blk_h

        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))

        self.connect_to_track_wires(core.get_all_port_pins('in_p'), fb_cap_p.get_all_port_pins('in'))
        self.connect_to_track_wires(core.get_all_port_pins('in_n'), fb_cap_n.get_all_port_pins('in'))
        self.connect_to_track_wires(sw_mid.get_all_port_pins('fb_in_n'), fb_cap_n.get_all_port_pins('out'))
        self.connect_to_track_wires(sw_mid.get_all_port_pins('fb_in_p'), fb_cap_p.get_all_port_pins('out'))

        def get_tids_above(coord, layer, type_list):
            if self.grid.get_direction(layer) == Orient2D.y:
                print("This function take horizontal wires only")
                return

            align_tidx = self.grid.coord_to_track(layer, coord)
            tids = []
            for idx, wtype in enumerate(type_list):
                tr_w = tr_manager.get_width(layer, wtype)
                tr_w_pre = tr_manager.get_width(layer, type_list[idx - 1]) if idx else 1
                align_tidx += self.get_track_sep(layer, tr_w, tr_w_pre)
                tids.append(TrackID(layer, align_tidx, tr_w, grid=self.grid))

            return tids

        core_out_xm = get_tids_above(core.bound_box.yh, xm_layer, ['dum', 'sig'])[1]
        core_out_n = self.connect_to_tracks(core.get_all_port_pins('out_n'), core_out_xm)
        core_out_p = self.connect_to_tracks(core.get_all_port_pins('out_p'), core_out_xm)
        self.connect_to_track_wires(core_out_n, sw_mid.get_all_port_pins('amp_out_p'))
        self.connect_to_track_wires(core_out_p, sw_mid.get_all_port_pins('amp_out_n'))

        if flip_cap:
            sw_mid_out_xm = get_tids_above(sw_mid.bound_box.yh, xm_layer, ['dum', 'sig'])[1]
            self.connect_to_tracks(sw_mid.get_all_port_pins('out_n') + sam_cap_n.get_all_port_pins('bot'),
                                   sw_mid_out_xm)
            self.connect_to_tracks(sw_mid.get_all_port_pins('out_p') + sam_cap_p.get_all_port_pins('bot'),
                                   sw_mid_out_xm)
            sw_out_in_xm = get_tids_above(sam_cap_n.bound_box.yh, xm_layer, ['dum', 'sig'])[1]
            self.connect_to_tracks(sw_out.get_all_port_pins('in_n') + sam_cap_n.get_all_port_pins('top'), sw_out_in_xm)
            self.connect_to_tracks(sw_out.get_all_port_pins('in_p') + sam_cap_p.get_all_port_pins('top'), sw_out_in_xm)
            cap_vss = []
        else:
            sw_mid_out_xm = get_tids_above(sw_mid.bound_box.yh, xm_layer, ['dum', 'sig'])[1]
            self.connect_to_tracks(sw_mid.get_all_port_pins('out_n') + sam_cap_n.get_all_port_pins('top'),
                                   sw_mid_out_xm)
            self.connect_to_tracks(sw_mid.get_all_port_pins('out_p') + sam_cap_p.get_all_port_pins('top'),
                                   sw_mid_out_xm)
            sw_out_in_xm = get_tids_above(sam_cap_n.bound_box.yh, xm_layer, ['dum', 'sig'])[1]
            self.connect_to_tracks(sw_out.get_all_port_pins('in_n') + sam_cap_n.get_all_port_pins('top'), sw_out_in_xm)
            self.connect_to_tracks(sw_out.get_all_port_pins('in_p') + sam_cap_p.get_all_port_pins('top'), sw_out_in_xm)

            tr_w_xm1_sup = tr_manager.get_width(xm1_layer, 'sup')
            cap_vss_tidx = self.get_available_tracks(xm1_layer,
                                                     self.grid.coord_to_track(xm1_layer, sam_cap_n.bound_box.yl,
                                                                              RoundMode.NEAREST),
                                                     self.grid.coord_to_track(xm1_layer, sam_cap_n.bound_box.yh,
                                                                              RoundMode.NEAREST),
                                                     self.bound_box.xl, self.bound_box.xh, width=tr_w_xm1_sup,
                                                     sep=tr_manager.get_sep(xm1_layer, ('sup', 'sup')))
            cap_vss = [self.connect_to_tracks(sam_cap_n.get_all_port_pins('bot') + sam_cap_p.get_all_port_pins('bot'),
                                              TrackID(xm1_layer, tidx, tr_w_xm1_sup)) for tidx in cap_vss_tidx]

        # Connect amp and ampb signals
        self.connect_differential_wires(sw_mid.get_all_port_pins('amp'), sw_mid.get_all_port_pins('amp_b'),
                                        core.get_pin('en'), core.get_pin('enb'))
        self.connect_wires(sw_mid.get_all_port_pins('amp') + sw_mid.get_all_port_pins('amp_b') +
                           sw_out.get_all_port_pins('phi1_b') + sw_out.get_all_port_pins('phi1'))

        # Fill supply cor ringamp core
        bbox_l = BBox(core.bound_box.xl, core.bound_box.yl, core.bound_box.w // 2, core.bound_box.yh)
        bbox_r = BBox(core.bound_box.w // 2, core.bound_box.yl, core.bound_box.xh, core.bound_box.yh)

        vss_xm_core, vdd_xm_core = core.get_all_port_pins('VSS'), core.get_all_port_pins('VDD')
        sup_stack_dict_l = self.connect_supply_stack_warr(tr_manager, [vdd_xm_core, vss_xm_core], xm_layer,
                                                          ym_layer, bbox_l, side_sup=False)
        sup_stack_dict_r = self.connect_supply_stack_warr(tr_manager, [vdd_xm_core, vss_xm_core], xm_layer,
                                                          ym_layer, bbox_r, side_sup=False, align_upper=True)
        core_sup_stack_dict = \
            self.connect_supply_stack_warr(tr_manager, [sup_stack_dict_l[0][ym_layer] + sup_stack_dict_r[0][ym_layer],
                                                        sup_stack_dict_l[1][ym_layer] + sup_stack_dict_r[1][ym_layer]],

                                           ym_layer, xm1_layer, core.bound_box, side_sup=False)

        # core dac control
        for pin in core.master.port_names_iter():
            if 'ctrl_dac' in pin:
                self.reexport(core.get_port(pin))

        self.add_pin('amp', sw_mid.get_all_port_pins('amp'))
        self.add_pin('amp_b', sw_mid.get_all_port_pins('amp_b'))

        self.add_pin('amp_out_p', sw_mid.get_all_port_pins('amp_out_n'))
        self.add_pin('amp_out_n', sw_mid.get_all_port_pins('amp_out_p'))
        self.add_pin('amp_out_p_xm1', sw_mid.get_all_port_pins('amp_out_p_xm1'), hide=True)
        self.add_pin('amp_out_n_xm1', sw_mid.get_all_port_pins('amp_out_n_xm1'), hide=True)
        self.add_pin('fb_in_n', sw_mid.get_all_port_pins('fb_in_n'))
        self.add_pin('fb_in_p', sw_mid.get_all_port_pins('fb_in_p'))
        self.add_pin('in_n', fb_cap_n.get_pin('in'))
        self.add_pin('in_p', fb_cap_p.get_pin('in'))
        self.add_pin('out_n', sw_out.get_pin('out_n'))
        self.add_pin('out_p', sw_out.get_pin('out_p'))

        self._bias_ycoord = max(core.get_pin('v_biasn').bound_box.yh, core.get_pin('v_biasp').bound_box.yh)
        self.add_pin('v_cmfb', self.extend_wires(core.get_pin('v_cmfb'), upper=self.bound_box.xh), mode=PinMode.UPPER)
        self.add_pin('v_biasn', self.extend_wires(core.get_pin('v_biasn'), lower=self.bound_box.xl), mode=PinMode.LOWER)
        self.add_pin('v_biasp', self.extend_wires(core.get_pin('v_biasp'), lower=self.bound_box.xl), mode=PinMode.LOWER)
        # fb cap control bits
        for port in fb_cap_p.port_names_iter():
            if 'bit' in port:
                self.add_pin(port.replace('bit', 'ctrl_fb_cap'),
                             self.connect_wires([fb_cap_n.get_pin(port), fb_cap_p.get_pin(port)]))

        # export amp for bias
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        phi_a_xm1_tidx = self.grid.coord_to_track(xm1_layer, sw_out.bound_box.yl, RoundMode.GREATER_EQ)
        phi_a_xm1_tidx = - self.get_track_sep(xm1_layer, tr_w_clk_xm1, 1) + phi_a_xm1_tidx
        phi_ab_xm1_tidx = phi_a_xm1_tidx - self.get_track_sep(xm1_layer, tr_w_clk_xm1, tr_w_clk_xm1)

        phi_a_xm1, phi_ab_xm1 = self.connect_matching_tracks([sw_out.get_all_port_pins('phi1_b'),
                                                              sw_out.get_all_port_pins('phi1')],
                                                             xm1_layer, [phi_a_xm1_tidx, phi_ab_xm1_tidx],
                                                             width=tr_w_clk_xm1)
        self.add_pin('amp', phi_a_xm1)
        self.add_pin('amp_b', phi_ab_xm1)

        # ---- Connect supplies ----
        vcm_ym1_tidx = self.grid.coord_to_track(ym1_layer, (self.bound_box.xl + self.bound_box.xh) // 2,
                                                RoundMode.NEAREST)
        vcm_ym1 = self.connect_to_tracks(sw_out.get_all_port_pins('vcm') + sw_mid.get_all_port_pins('vcm'),
                                         TrackID(ym1_layer, vcm_ym1_tidx, tr_manager.get_width(ym1_layer, 'vcm'),
                                                 grid=self.grid))
        vdd_xm1_list = sw_mid.get_all_port_pins('VDD') + sw_out.get_all_port_pins('VDD') + \
                       core_sup_stack_dict[0][xm1_layer] + fb_cap_n.get_all_port_pins('VDD', layer=xm1_layer) + \
                       fb_cap_p.get_all_port_pins('VDD', layer=xm1_layer)
        vss_xm1_list = sw_mid.get_all_port_pins('VSS') + sw_out.get_all_port_pins('VSS') + \
                       core_sup_stack_dict[1][xm1_layer] + fb_cap_n.get_all_port_pins('VSS', layer=xm1_layer) + \
                       fb_cap_p.get_all_port_pins('VSS', layer=xm1_layer)
        bbox_l = BBox(self.bound_box.xl, self.bound_box.yl, self.bound_box.w // 2, self.bound_box.yh)
        bbox_r = BBox(self.bound_box.w // 2, self.bound_box.yl, self.bound_box.xh, self.bound_box.yh)

        sup_stack_dict_l = self.connect_supply_warr(tr_manager, [vdd_xm1_list, vss_xm1_list], xm1_layer, bbox_l,
                                                    side_sup=False)
        sup_stack_dict_r = self.connect_supply_warr(tr_manager, [vdd_xm1_list, vss_xm1_list], xm1_layer, bbox_r,
                                                    side_sup=False, align_upper=True)

        if cap_vss:
            self.connect_to_track_wires(sup_stack_dict_l[1] + sup_stack_dict_r[1], cap_vss)

        self.add_pin('vcm', vcm_ym1)
        self.add_pin('VDD', sup_stack_dict_l[0] + sup_stack_dict_r[0])
        self.add_pin('VSS', sup_stack_dict_l[1] + sup_stack_dict_r[1])

        self._sch_params = dict(
            ra_params=ra_core_master.sch_params,
            cap_fb_params=fb_cap_master.sch_params,
            # cap_gain_params=gain_cap_master.sch_params,
            cap_sam_params=sam_cap_master.sch_params,
            sw_out_params=sw_out_master.sch_params,
            sw_mid_params=sw_mid_master.sch_params,
            flip_cap=flip_cap,
        )


class RATop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None

    @property
    def tr_manager(self):
        return self._tr_manager
    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            ra_params='',
            bias_params='',
            cmfb_params='',
            tr_widths='',
            tr_spaces='',
            top_sup_layer='',
            top_vcm_layer='',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(top_sup_layer=0, top_vcm_layer=0)

    @staticmethod
    def place_on_track(x: int, y: int, w_blk: int, h_blk: int):
        return -(-x // w_blk) * w_blk, -(-y // h_blk) * h_blk

    def draw_layout(self) -> None:
        ra_params: Param = self.params['ra_params']
        bias_params: Param = self.params['bias_params']
        cmfb_params: Param = self.params['cmfb_params']

        if isinstance(ra_params, str):
            spec_yaml = read_yaml(ra_params)
            ra_params = spec_yaml['params']
        if isinstance(bias_params, str):
            spec_yaml = read_yaml(bias_params)
            bias_params = spec_yaml['params']
        if isinstance(cmfb_params, str):
            spec_yaml = read_yaml(cmfb_params)
            cmfb_params = spec_yaml['params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        self._tr_manager = tr_manager

        ra_master = self.new_template(RASWTop, params=ra_params)
        bias_master = self.new_template(RABiasTop, params=bias_params)
        cmfb_master = self.new_template(RACMFB, params=cmfb_params)

        # --- Placement --- #
        conn_layer = \
            MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                             bias_params['bias_gen_params']['pinfo']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1
        x_rt_margin_ntr, _ = tr_manager.place_wires(ym_layer, ['dum', 'bias', 'bias', 'dum'])
        x_margin = self.grid.track_to_coord(ym_layer, x_rt_margin_ntr)

        top_sup_layer = self.params['top_sup_layer']
        top_vcm_layer = self.params['top_vcm_layer']
        top_layer = max(ra_master.top_layer, bias_master.top_layer, cmfb_master.top_layer, top_sup_layer,
                        top_vcm_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        w_bias, h_bias = bias_master.bound_box.w, bias_master.bound_box.h
        w_ra, h_ra = ra_master.bound_box.w, ra_master.bound_box.h
        w_cmfb, h_cmfb = cmfb_master.bound_box.w, cmfb_master.bound_box.h

        # Align ra bias loc to the middle of two bias in biias top
        ra_bias_ycoord = ra_master.bias_ycoord
        bias_ycoord = bias_master.bias_ycoord
        bias_y = ra_bias_ycoord - (bias_ycoord[0] + bias_ycoord[1]) // 2
        bias_x, bias_y = self.place_on_track(0, bias_y, blk_w, blk_h)
        bias = self.add_instance(bias_master, xform=Transform(bias_x, bias_y, mode=Orientation.R0))
        ra_x, ra_y = self.place_on_track(bias.bound_box.xh + x_margin, 0, blk_w, blk_h)
        ra = self.add_instance(ra_master, xform=Transform(ra_x, ra_y, mode=Orientation.R0))

        cmfb_y = self.grid.track_to_coord(xm1_layer, ra.get_pin('v_cmfb').track_id.base_index)
        tr_w_cmfb_xm1 = cmfb_master.get_port('v_cmfb').get_pins()[0].track_id.width
        coord_sp_cmfb_xm1 = self.get_track_sep(xm1_layer, tr_w_cmfb_xm1, tr_w_cmfb_xm1) * \
                            self.grid.get_track_pitch(xm1_layer)
        coord_sp_cmfb_xm1 = coord_sp_cmfb_xm1.dbl_value
        cmfb_ra_sp_x = self.get_track_sep(xm1_layer, tr_manager.get_width(ym1_layer, 'bias'),
                                          tr_manager.get_width(ym1_layer, 'bias')) * \
                       self.grid.get_track_pitch(xm1_layer)
        tr_manager.get_width(ym1_layer, 'bias')
        cmfb_x, cmfb_y = self.place_on_track(ra.bound_box.xh + x_margin + cmfb_ra_sp_x, cmfb_y + 2 * coord_sp_cmfb_xm1,
                                             blk_w, blk_h)

        cmfb = self.add_instance(cmfb_master, xform=Transform(cmfb_x, cmfb_y, mode=Orientation.R0))

        top_layer = max(ra_master.top_layer, bias_master.top_layer, cmfb_master.top_layer, top_sup_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        tot_w = -(-(2*ra_x+ra.bound_box.w) // blk_w) * blk_w
        tot_h = -(-max(ra.bound_box.yh, bias.bound_box.yh, cmfb.bound_box.yh) // blk_h) * blk_h
        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))

        _, bias_ym_tidx = tr_manager.place_wires(ym_layer, ['bias'] * 2,
                                                 center_coord=(bias.bound_box.xh + ra.bound_box.xl) // 2)
        _, out_ym_tidx = tr_manager.place_wires(ym_layer, ['bias'] * 2,
                                                center_coord=(ra.bound_box.xh + cmfb.bound_box.xl) // 2)

        tr_w_bias_ym = tr_manager.get_width(ym_layer, 'bias')
        pbias_ym, nbias_ym = \
            self.connect_differential_tracks(bias.get_pin('v_biasp', layer=xm1_layer),
                                             bias.get_pin('v_biasn', layer=xm1_layer),
                                             ym_layer, bias_ym_tidx[0], bias_ym_tidx[1], width=tr_w_bias_ym)
        self.connect_differential_wires(ra.get_pin('v_biasp'), ra.get_pin('v_biasn'), pbias_ym, nbias_ym)
        amp_n_ym, amp_p_ym = \
            self.connect_differential_tracks(cmfb.get_pin('ra_out_n'), cmfb.get_pin('ra_out_p'),
                                             ym_layer, out_ym_tidx[0], out_ym_tidx[1], width=tr_w_bias_ym)
        self.connect_differential_wires(ra.get_pin('amp_out_n_xm1'), ra.get_pin('amp_out_p_xm1'), amp_n_ym, amp_p_ym)

        vcmfb_ym1_tidx = self.grid.coord_to_track(ym1_layer, (ra.bound_box.xh + cmfb.bound_box.xl) // 2,
                                                  RoundMode.NEAREST)
        vcmfb_ym1 = self.connect_to_tracks([cmfb.get_pin('v_cmfb'), ra.get_pin('v_cmfb')],
                                           TrackID(ym1_layer, vcmfb_ym1_tidx, tr_manager.get_width(ym1_layer, 'bias')))

        self.add_pin('v_cmss', cmfb.get_pin('v_cmss'))
        self.add_pin('vcm_ext', cmfb.get_pin('vcm_ext'))

        # Connect amp to bias
        bias_ym_tidx = self.grid.coord_to_track(ym_layer, bias.bound_box.xh, RoundMode.LESS)
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        amp_bias_ym = self.connect_to_tracks(bias.get_all_port_pins('amp'),
                                             TrackID(ym_layer, bias_ym_tidx, tr_w_clk_ym))
        self.connect_to_track_wires(ra.get_pin('amp', layer=xm1_layer), amp_bias_ym)
        self.connect_differential_wires(ra.get_pin('amp_b', layer=xm1_layer),
                                        ra.get_pin('amp', layer=xm1_layer),
                                        cmfb.get_pin('amp_b'), cmfb.get_pin('amp'))

        xm2_layer = ym1_layer + 1
        ym2_layer = xm2_layer + 1
        vcm_ym1 = ra.get_pin('vcm')
        tr_w_sup_xm2 = tr_manager.get_width(xm2_layer, 'vcm')
        vcm_xm2_tidx = [self.grid.coord_to_track(xm2_layer, vcm_ym1.upper, RoundMode.NEAREST),
                        self.grid.coord_to_track(xm2_layer, vcm_ym1.lower, RoundMode.NEAREST)]
        vcm_xm2_sep = self.get_track_sep(xm2_layer, tr_w_sup_xm2, 1)
        vcm_xm2_tidx = [vcm_xm2_tidx[0] - vcm_xm2_sep, vcm_xm2_tidx[1] + vcm_xm2_sep]
        vcm_xm2 = [self.connect_to_tracks(vcm_ym1, TrackID(xm2_layer, vcm_xm2_tidx[0], tr_w_sup_xm2, grid=self.grid)),
                   self.connect_to_tracks(vcm_ym1, TrackID(xm2_layer, vcm_xm2_tidx[1], tr_w_sup_xm2, grid=self.grid))]
        # vcm_xm2 = self.extend_wires(vcm_xm2, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_ym1 = ra.get_all_port_pins('VDD') + bias.get_all_port_pins('VDD') + cmfb.get_all_port_pins('VDD')
        vss_ym1 = ra.get_all_port_pins('VSS') + bias.get_all_port_pins('VSS') + cmfb.get_all_port_pins('VSS')
        xm2_power_dict = self.connect_supply_warr(tr_manager, [vdd_ym1, vss_ym1], ym1_layer, self.bound_box)

        if top_vcm_layer > xm2_layer:
            vcm_stack_up_0 = self.via_stack_up(tr_manager, vcm_xm2[0], xm2_layer, top_vcm_layer, 'sup')
            vcm_stack_up_1 = self.via_stack_up(tr_manager, vcm_xm2[1], xm2_layer, top_vcm_layer, 'sup')
            self.connect_wires([vcm_stack_up_0[top_vcm_layer-1],vcm_stack_up_1[top_vcm_layer-1]])
            self.add_pin('vcm', [vcm_stack_up_0[top_vcm_layer],vcm_stack_up_1[top_vcm_layer]])
        if top_sup_layer > xm2_layer:
            power_dict = self.connect_supply_stack_warr(tr_manager, [xm2_power_dict[0], xm2_power_dict[1]],
                                                        xm2_layer, top_sup_layer, self.bound_box)
            self.add_pin('VDD', power_dict[0][top_sup_layer])
            self.add_pin('VSS', power_dict[1][top_sup_layer])
        else:
            self.add_pin('vcm', vcm_xm2)
            self.add_pin('VDD', xm2_power_dict[0])
            self.add_pin('VSS', xm2_power_dict[1])

        bias_top_sch_params = bias_master.sch_params.copy()

        for pinname in ra.port_names_iter():
            if sum([sup_pin in pinname for sup_pin in ['vcm', 'VDD', 'VSS', 'amp', 'amp_b']]):
                continue
            else:
                self.reexport(ra.get_port(pinname))
        self.add_pin('amp', self.extend_wires(ra.get_pin('amp', layer=xm1_layer), lower=self.bound_box.xl),
                     mode=PinMode.LOWER)
        self.add_pin('amp_b', self.extend_wires(ra.get_pin('amp_b', layer=xm1_layer), lower=self.bound_box.xl),
                     mode=PinMode.LOWER)

        self.reexport(ra.get_port('amp_out_n'))
        self.reexport(ra.get_port('amp_out_p'))
        for pinname in bias.port_names_iter():
            if 'ctrl' in pinname:
                self.reexport(bias.get_port(pinname))

        self._sch_params = dict(
            bias_params=bias_top_sch_params,
            ra_params=ra_master.sch_params,
            cmfb_params=cmfb_master.sch_params,
            cap_gain_params=None,
        )
