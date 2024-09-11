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


from typing import Any, Dict, Type, Optional, Mapping, Tuple, Union

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict
from bag.util.math import HalfInt
from bag_vco_adc.layout.util.util import get_available_tracks_reverse, fill_conn_layer_intv
from pybag.enum import RoundMode, MinLenMode
from xbase.layout.enum import MOSWireType, MOSType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import TilePatternElement, TilePattern, MOSArrayPlaceInfo
from ..util.template import TrackIDZL as TrackID, TemplateBaseZL


class FBCapSW(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._has_ofst = False

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_fb_sw')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            ridx_cm='pmos row index.',
            has_ofst='True to add bridge switch.',
            vertical_out='True to connect outputs to vm_layer.',
            vertical_sup='True to connect outputs to vm_layer.',
            sig_locs='Optional dictionary of user defined signal locations',
            tot_seg='Total segments',
            swp_phi12='Change phi1 and phi2 location',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            has_ofst=False,
            vertical_out=True,
            vertical_sup=True,
            swp_phi12=False,
            ridx_cm=2,
            ridx_p=1,
            ridx_n=0,
            tot_seg=0,
        )

    @property
    def has_ofst(self) -> bool:
        return self._has_ofst

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']

        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        ridx_cm: int = self.params['ridx_cm']

        seg_n = seg_dict['sw_n']
        seg_p = seg_dict['sw_p']
        seg_cm = seg_dict['sw_cm']

        w_n = w_dict['sw_n']
        w_p = w_dict['sw_p']
        w_cm = w_dict['sw_cm']

        if seg_n & 1 or seg_p & 1 or seg_cm & 1:
            raise ValueError('in, tail, nfb, or pfb must have even number of segments')
        mid_sep = self.min_sep_col

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        vdd_xm_list, vss_xm_list = [], []

        vm_ntr, vm_locs = tr_manager.place_wires(vm_layer, ['sup'] + ['ctrl'] * 2 + ['sup'] + ['sig', 'dum', 'sig'] + \
                                                 ['sup'], align_track=self.arr_info.col_to_track(vm_layer, 0))
        ctrl_locs = vm_locs[1:3]
        inout_locs = [vm_locs[4], vm_locs[6]]

        # Add tap at side
        # vdd_list, vss_list = [], []
        # tap_ncol = self.get_tap_ncol(tile_idx=0)
        # tap_sep_col = self.sub_sep_col

        tot_cols = max(self.num_cols, self.arr_info.track_to_col(vm_layer, vm_ntr))

        # placement
        m_n = self.add_mos(ridx_n, (tot_cols - max(seg_n, seg_p)) // 2, seg_n, w=w_n)
        m_p = self.add_mos(ridx_p, (tot_cols - max(seg_n, seg_p)) // 2, seg_p, w=w_p)
        m_cm_n = self.add_mos(ridx_cm + 1, (tot_cols - seg_cm) // 2, seg_cm, w=w_cm)
        m_cm_p = self.add_mos(ridx_cm, (tot_cols - seg_cm) // 2, seg_cm, w=w_cm)
        #
        # sw_bnd_col = self.num_cols
        # sup_hm_bnd = self.arr_info.col_to_coord(tot_cols)
        # self.add_tap(tot_cols + tap_sep_col, vdd_list, vss_list)

        self.set_mos_size(max(self.num_cols, self.params['tot_seg'], tot_cols))

        # bot_ng_tid = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        # top_ng_tid = self.get_track_id(ridx_cm + 1, MOSWireType.G, wire_name='sig', wire_idx=0)
        # bot_pg_tid = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1)
        # top_pg_tid = self.get_track_id(ridx_cm, MOSWireType.G, wire_name='sig', wire_idx=-1)

        tr_w_hm_ctrl_ow = tr_manager.get_width(hm_layer, 'ctrl_ow')
        tile_flip = self.get_tile_info(0)[2]
        bot_ng_tid = self.get_track_index(ridx_n, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        # bot_ng_tid1 = tr_manager.get_next_track(hm_layer, bot_ng_tid0, 'ctrl_ow', 'ctrl_ow', up=not bool(tile_flip))
        bot_pg_tid = self.get_track_index(ridx_p, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        # pg_tid1 = tr_manager.get_next_track(hm_layer, pg_tid0, 'ctrl_ow', 'ctrl_ow', up=bool(tile_flip))
        top_ng_tid = self.get_track_index(ridx_cm + 1, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        top_pg_tid = self.get_track_index(ridx_cm, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        # pg_tid1 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='ctrl', wire_idx=-2)
        # bot_ng_tid1 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='ctrl', wire_idx=1)
        [bot_ng_tid, bot_pg_tid, top_ng_tid, top_pg_tid] = \
            [TrackID(hm_layer, tidx, tr_w_hm_ctrl_ow) for tidx in
             [bot_ng_tid, bot_pg_tid, top_ng_tid, top_pg_tid]]

        bot_nd_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        bot_pd_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)
        top_nd_tid0 = self.get_track_id(ridx_cm + 1, MOSWireType.DS, wire_name='sig', wire_idx=0)
        top_pd_tid0 = self.get_track_id(ridx_cm, MOSWireType.DS, wire_name='sig', wire_idx=1)
        bot_nd_tid1 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        bot_pd_tid1 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        top_nd_tid1 = self.get_track_id(ridx_cm + 1, MOSWireType.DS, wire_name='sig', wire_idx=1)
        top_pd_tid1 = self.get_track_id(ridx_cm, MOSWireType.DS, wire_name='sig', wire_idx=0)

        # Conn layer connection
        in_hm_warr = [m_n.s, m_p.s, m_cm_n.s, m_cm_p.s]
        self.connect_wires(in_hm_warr)

        phi1 = [self.connect_to_tracks(m_n.g, bot_ng_tid), self.connect_to_tracks(m_cm_p.g, top_pg_tid)]
        phi1b = [self.connect_to_tracks(m_p.g, bot_pg_tid), self.connect_to_tracks(m_cm_n.g, top_ng_tid)]
        #
        cm_hm = [self.connect_to_tracks(m_cm_n.d, top_nd_tid0), self.connect_to_tracks(m_cm_p.d, top_pd_tid0)]
        out_hm = [self.connect_to_tracks(m_n.d, bot_nd_tid0), self.connect_to_tracks(m_p.d, bot_pd_tid0)]
        in_hm = [self.connect_to_tracks(m_n.s, bot_nd_tid1), self.connect_to_tracks(m_p.s, bot_pd_tid1),
                 self.connect_to_tracks(m_cm_n.s, top_nd_tid1), self.connect_to_tracks(m_cm_p.s, top_pd_tid1)]
        # #
        # vss_bot_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sup')
        # vss_hm = self.connect_to_tracks(vss_list, vss_bot_tid, track_lower=sup_hm_bnd, track_upper=self.bound_box.xh)
        # vdd_top_tid = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sup')
        # vdd_hm = self.connect_to_tracks(vdd_list, vdd_top_tid, track_lower=sup_hm_bnd, track_upper=self.bound_box.xh)
        # #
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        out_vm = self.connect_to_tracks(out_hm, TrackID(vm_layer, inout_locs[0], tr_w_sig_vm))
        in_vm = self.connect_to_tracks(in_hm, TrackID(vm_layer, inout_locs[1], tr_w_sig_vm))
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        phi1_vm = self.connect_to_tracks(phi1, TrackID(vm_layer, ctrl_locs[0], tr_w_ctrl_vm))
        phi1b_vm = self.connect_to_tracks(phi1b, TrackID(vm_layer, ctrl_locs[1], tr_w_ctrl_vm))
        self.match_warr_length([phi1_vm, phi1b_vm])
        self.add_pin('phi1', phi1_vm)
        self.add_pin('phi1b', phi1b_vm)

        self.add_pin('phi1_hm', phi1)
        self.add_pin('phi1b_hm', phi1b)

        vcm_vm_list = []
        for vcm in cm_hm:
            _tidx = vm_locs[3]
            vcm_vm = [self.connect_to_tracks(vcm, TrackID(vm_layer, _tidx, tr_w_sup_vm))]
            _tidx = vm_locs[-1]
            vcm_vm.append(self.connect_to_tracks(vcm, TrackID(vm_layer, _tidx, tr_w_sup_vm)))
            vcm_vm_list.extend(vcm_vm)

        vcm_sw_vm_list = []
        for vcm in cm_hm:
            vcm_sw_vm_list.append(self.connect_to_tracks(vcm, TrackID(vm_layer, inout_locs[0], tr_w_sig_vm)))

        vcm_vm_list = self.connect_wires(vcm_vm_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
        self.add_pin('vcm', vcm_vm_list, connect=True)
        self.add_pin('vcm_sw', vcm_sw_vm_list)
        # self.add_pin('VSS', vss_xm_list)
        # self.add_pin('VDD', vdd_xm_list)
        self.add_pin('in', in_vm)
        self.add_pin('out', out_vm)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=dict(
                sw_n=self.get_row_info(ridx_n, 0).threshold,
                sw_p=self.get_row_info(ridx_p, 0).threshold,
                sw_cm=self.get_row_info(ridx_cm, 0).threshold,
            ),
        )


class IsolationSWHalf(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._has_ofst = False

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            ridx_cm='pmos row index.',
            has_ofst='True to add bridge switch.',
            vertical_out='True to connect outputs to vm_layer.',
            vertical_sup='True to connect outputs to vm_layer.',
            sig_locs='Optional dictionary of user defined signal locations',
            out_cm='True to add output cm switch',
            out_cmos_cm='True to add cmos switch at output',
            in_cmos_cm='True to add cmos switch at input',
            tot_seg='Total segments',
            swp_phi12='Change phi1 and phi2 location',
            in_sw_term='',
            out_sw_term='',
            one_phase='',
            flip_inout='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            has_ofst=False,
            vertical_out=True,
            vertical_sup=True,
            swp_phi12=False,
            ridx_cm=2,
            ridx_p=1,
            ridx_n=0,
            out_cm=True,
            in_cmos_cm=False,
            out_cmos_cm=False,
            tot_seg=0,
            one_phase=False,
            in_sw_term='vcm',
            out_sw_term='vcm',
            flip_inout=False,
        )

    @property
    def has_ofst(self) -> bool:
        return self._has_ofst

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']
        out_cm: bool = self.params['out_cm']
        in_cmos_cm: bool = self.params['in_cmos_cm']
        out_cmos_cm: bool = self.params['out_cmos_cm']
        flip_inout: bool = self.params['flip_inout']

        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        ridx_cm: int = self.params['ridx_cm']

        in_sw_term: str = self.params['in_sw_term']
        out_sw_term: str = self.params['out_sw_term']

        seg_n = seg_dict['sw_n']
        seg_p = seg_dict['sw_p']
        seg_cm = seg_dict['sw_cm']

        w_n = w_dict['sw_n']
        w_p = w_dict['sw_p']
        w_cm = w_dict['sw_cm']

        if seg_n & 1 or seg_p & 1 or seg_cm & 1:
            raise ValueError('in, tail, nfb, or pfb must have even number of segments')
        mid_sep = self.min_sep_col

        seg_cm_p = seg_dict.get('sw_cm_p', seg_cm)
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        vdd_xm_list, vss_xm_list = [], []

        # vm_ntr, vm_locs = tr_manager.place_wires(vm_layer, ['sup'] + ['ctrl'] * 4 + ['sup'] + ['sig'] * 2 + ['sup'],
        #                                          align_track=self.arr_info.col_to_track(vm_layer, 0))

        vm_ntr, vm_locs = tr_manager.place_wires(vm_layer, ['dum'] + ['ctrl'] * 4 + ['sup'] + ['sig'] * 2 + ['sup'] * 3,
                                                 align_track=self.arr_info.col_to_track(vm_layer, 0))
        ctrl_locs = vm_locs[1:5]
        inout_locs = vm_locs[6:8]
        place_start_col = self.arr_info.track_to_col(vm_layer, vm_locs[5])

        # placement
        m_n = self.add_mos(ridx_n, place_start_col, seg_n, w=w_n)
        m_p = self.add_mos(ridx_p, place_start_col, seg_p, w=w_p)
        m_cm_in = self.add_mos(ridx_cm + 1, place_start_col, seg_cm, w=w_cm)
        m_cm_in_p = self.add_mos(ridx_cm, place_start_col, seg_cm, w=w_cm) if in_cmos_cm else None
        max_col = max(seg_n, seg_p)
        m_cm_out = self.add_mos(ridx_n, place_start_col + max_col + mid_sep, seg_cm, w=w_cm) if out_cm else None
        m_cm_out_p = self.add_mos(ridx_p, place_start_col + max_col + mid_sep, seg_cm_p,
                                  w=w_cm) if out_cm and out_cmos_cm else None
        for idx in range(self.num_tile_rows):
            for jdx in range(self.get_tile_info(0)[0].num_rows):
                fill_conn_layer_intv(self, idx, jdx, stop_col=place_start_col)

        # Add tap at side
        vdd_list, vss_list = [], []
        tap_ncol = self.get_tap_ncol(tile_idx=0)
        tap_sep_col = self.sub_sep_col

        tot_cols = max(self.num_cols, self.arr_info.track_to_col(vm_layer, vm_ntr))
        fill_conn_layer_intv(self, 0, ridx_cm, start_col=place_start_col + max_col + mid_sep - 1, stop_col=tot_cols)
        fill_conn_layer_intv(self, 0, ridx_cm + 1, start_col=place_start_col + max_col + mid_sep - 1, stop_col=tot_cols)
        if not out_cm:
            fill_conn_layer_intv(self, 0, ridx_n, start_col=place_start_col + max_col + mid_sep - 1, stop_col=tot_cols)
        if not (out_cm and out_cmos_cm):
            fill_conn_layer_intv(self, 0, ridx_p, start_col=place_start_col + max_col + mid_sep - 1, stop_col=tot_cols)
        sup_hm_bnd = self.arr_info.col_to_coord(tot_cols)
        sup_col = self.arr_info.track_to_col(vm_layer, vm_locs[-3])
        sup_col = max(sup_col, place_start_col + max_col + mid_sep)
        sup_start_vm_tidx = self.arr_info.col_to_track(vm_layer, sup_col)
        _, vm_locs[-3:] = tr_manager.place_wires(vm_layer, ['sup'] * 3, align_track=sup_start_vm_tidx)
        self.add_tap(tot_cols + tap_sep_col, vdd_list, vss_list)

        self.set_mos_size(max(self.num_cols, self.params['tot_seg']))

        tr_w_hm_ctrl_ow = tr_manager.get_width(hm_layer, 'ctrl_ow')
        tile_flip = self.get_tile_info(0)[2]
        bot_ng_tid0 = self.get_track_index(ridx_n, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        bot_ng_tid1 = tr_manager.get_next_track(hm_layer, bot_ng_tid0, 'ctrl_ow', 'ctrl_ow', up=not bool(tile_flip))
        pg_tid0 = self.get_track_index(ridx_p, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        pg_tid1 = tr_manager.get_next_track(hm_layer, pg_tid0, 'ctrl_ow', 'ctrl_ow', up=bool(tile_flip))
        top_ng_tid = self.get_track_index(ridx_cm + 1, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        # pg_tid1 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='ctrl', wire_idx=-2)
        # bot_ng_tid1 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='ctrl', wire_idx=1)
        [bot_ng_tid0, bot_ng_tid1, pg_tid0, pg_tid1, top_ng_tid] = \
            [TrackID(hm_layer, tidx, tr_w_hm_ctrl_ow) for tidx in
             [bot_ng_tid0, bot_ng_tid1, pg_tid0, pg_tid1, top_ng_tid]]

        bot_nd_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        bot_nd_tid1 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        top_nd_tid0 = self.get_track_id(ridx_cm + 1, MOSWireType.DS, wire_name='sig', wire_idx=0)
        top_nd_tid1 = self.get_track_id(ridx_cm + 1, MOSWireType.DS, wire_name='sig', wire_idx=1)
        # top_nd_tid2 = self.get_track_id(ridx_cm, MOSWireType.DS, wire_name='sig', wire_idx=2)
        pd_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        pd_tid1 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2)

        # Conn layer connection
        if out_cm:
            self.connect_wires([m_cm_in.g, m_cm_out.g])
            self.connect_wires([m_cm_in.s, m_cm_out.s])
        in_hm_warr = [m_n.s, m_p.s, m_cm_in.s]
        if in_cmos_cm:
            in_hm_warr.append(m_cm_in_p.s)
        self.connect_wires(in_hm_warr)

        phi1 = self.connect_to_tracks(m_n.g, bot_ng_tid0)
        phi1b = self.connect_to_tracks(m_p.g, pg_tid0)
        phi2_top = self.connect_to_tracks(m_cm_in.g, top_ng_tid)
        phi2_bot = self.connect_to_tracks(m_cm_out.g, bot_ng_tid1) if out_cm else None
        phi2b = self.connect_to_tracks(m_cm_out_p.g, pg_tid1) if out_cm and out_cmos_cm else None

        cm_in = self.connect_to_tracks(m_cm_in.d, top_nd_tid0)
        cm_out = self.connect_to_tracks(m_cm_out.s, bot_nd_tid0) if out_cm else None
        cm_out_p = self.connect_to_tracks(m_cm_out_p.s, pd_tid1) if out_cm and out_cmos_cm else None
        in_top = self.connect_to_tracks([m_cm_in.s], top_nd_tid1)
        out_top = self.connect_to_tracks([m_p.d, m_cm_out_p.d] if out_cm and out_cmos_cm else m_p.d, pd_tid0)
        out_bot = self.connect_to_tracks([m_n.d, m_cm_out.d] if out_cm else [m_n.d], bot_nd_tid1)

        if in_cmos_cm:
            top_pd_tid0 = self.get_track_id(ridx_cm, MOSWireType.DS, wire_name='sig', wire_idx=-2)
            top_pd_tid1 = self.get_track_id(ridx_cm, MOSWireType.DS, wire_name='sig', wire_idx=-1)
            top_pg_tid = self.get_track_index(ridx_cm, MOSWireType.G, wire_name='ctrl', wire_idx=0)
            top_pg_tid = TrackID(hm_layer, top_pg_tid, tr_w_hm_ctrl_ow)

            phi2b_top = self.connect_to_tracks(m_cm_in_p.g, top_pg_tid)
            cm_in_p = self.connect_to_tracks(m_cm_in_p.d, top_pd_tid1)
            in_top_p = self.connect_to_tracks([m_cm_in_p.s], top_pd_tid0)
        else:
            phi2b_top, cm_in_p, in_top_p = None, None, None

        vss_bot_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sup')
        vss_top_tid = self.get_track_id(ridx_cm + 1, MOSWireType.DS, wire_name='sup')
        vss_hm = [self.connect_to_tracks(vss_list, vss_bot_tid, track_lower=sup_hm_bnd, track_upper=self.bound_box.xh),
                  self.connect_to_tracks(vss_list, vss_top_tid, track_lower=sup_hm_bnd, track_upper=self.bound_box.xh)]
        # if in_cmos_cm:
        vdd_top_tid = self.get_track_id(ridx_cm, MOSWireType.DS, wire_name='sup')
        # else:
        #     vdd_top_tid = self.get_track_id(ridx_cm + 1, MOSWireType.G, wire_name='sup')
        vdd_hm = [self.connect_to_tracks(vdd_list, vdd_top_tid, track_lower=sup_hm_bnd, track_upper=self.bound_box.xh)]

        if in_sw_term == 'VDD':
            vdd_hm.extend([cm_in, cm_in_p] if cm_in_p else [cm_in])
        elif in_sw_term == 'VSS':
            vss_hm.extend([cm_in, cm_in_p] if cm_in_p else [cm_in])

        if out_sw_term == 'VDD':
            vdd_hm.extend([cm_out, cm_out_p])
        elif out_sw_term == 'VSS':
            vss_hm.extend([cm_out, cm_out_p])

        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')

        vss_vm = self.connect_to_tracks(vss_hm, TrackID(vm_layer, vm_locs[-1], tr_w_sup_vm),
                                        track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
        for vss in vss_hm[:2]:
            xm_tidx_coord = self.grid.track_to_coord(hm_layer, vss.track_id.base_index)
            xm_tidx = self.grid.coord_to_track(xm_layer, xm_tidx_coord, RoundMode.NEAREST)
            vss_xm_list.append(self.connect_to_tracks(vss_vm, TrackID(xm_layer, xm_tidx, tr_w_sup_xm),
                                                      track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))

        vdd_vm = self.connect_to_tracks(vdd_hm, TrackID(vm_layer, vm_locs[-3], tr_w_sup_vm),
                                        track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
        xm_tidx_coord = self.grid.track_to_coord(hm_layer, vdd_hm[0].track_id.base_index)
        xm_tidx = self.grid.coord_to_track(xm_layer, xm_tidx_coord, RoundMode.NEAREST)
        vdd_xm_list.append(self.connect_to_tracks(vdd_vm, TrackID(xm_layer, xm_tidx, tr_w_sup_xm),
                                                  track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))
        self.connect_to_tracks(vdd_xm_list, TrackID(vm_layer, vm_locs[5], tr_w_sup_vm),
                               track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        out_vm = self.connect_to_tracks([out_top, out_bot], TrackID(vm_layer, inout_locs[1], tr_w_sig_vm))
        in_mid = self.connect_to_tracks([m_n.s, m_p.s], self.get_track_id(ridx_p, MOSWireType.DS, 'sup'))
        in_bot = self.connect_to_tracks([m_n.s, m_p.s], self.get_track_id(ridx_n, MOSWireType.DS, 'sup'))
        in_vm = self.connect_to_tracks([in_top, in_top_p, in_mid, in_bot] if in_cmos_cm else [in_top, in_mid, in_bot],
                                       TrackID(vm_layer, inout_locs[0], tr_w_sig_vm))

        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')

        if self.params['swp_phi12']:
            ctrl_locs = ctrl_locs[2:] + ctrl_locs[0:2]
        if self.params['one_phase']:
            ctrl_locs = [ctrl_locs[2], ctrl_locs[3], ctrl_locs[3], ctrl_locs[2]]

        phi1_vm = self.connect_to_tracks(phi1, TrackID(vm_layer, ctrl_locs[0], tr_w_ctrl_vm),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
        phi1b_vm = self.connect_to_tracks(phi1b, TrackID(vm_layer, ctrl_locs[1], tr_w_ctrl_vm),
                                          track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
        phi2_vm = self.connect_to_tracks([phi2_bot, phi2_top] if out_cm else phi2_top,
                                         TrackID(vm_layer, ctrl_locs[2], tr_w_ctrl_vm),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)

        if self.params['one_phase']:
            self.add_pin('phi1', phi1_vm)
            self.add_pin('phi1b', phi1b_vm)
            self.add_pin('phi1b', phi2_vm)

            self.add_pin('phi1_hm', phi1)
            self.add_pin('phi1b_hm', phi1b)
            self.add_pin('phi1b_hm', [phi2_bot, phi2_top] if out_cm else phi2_top)
        else:
            self.add_pin('phi1', phi1_vm)
            self.add_pin('phi1b', phi1b_vm)
            self.add_pin('phi2', phi2_vm)

            self.add_pin('phi1_hm', phi1)
            self.add_pin('phi1b_hm', phi1b)
            self.add_pin('phi2_hm', [phi2_bot, phi2_top] if out_cm else phi2_top)

        phi2b_hm = []
        if out_cm and out_cmos_cm:
            phi2b_hm.append(phi2b)
        if in_cmos_cm:
            phi2b_hm.append(phi2b_top)
        if phi2b_hm:
            phi2b_vm = self.connect_to_tracks(phi2b_hm, TrackID(vm_layer, ctrl_locs[3], tr_w_ctrl_vm),
                                              track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
            self.add_pin('phi1' if self.params['one_phase'] else 'phi2b', phi2b_vm)
            self.add_pin('phi1_hm' if self.params['one_phase'] else 'phi2b_hm', phi2b_hm)
        if flip_inout:
            out_vm, in_vm = in_vm, out_vm
        self.add_pin('out', out_vm, connect=True)
        self.add_pin('in', in_vm, connect=True)

        cm_out_hm_arr = [cm_out, cm_out_p] if out_cmos_cm else [cm_out]
        cm_in_hm_arr = [cm_in, cm_in_p] if in_cmos_cm else [cm_in]

        cm_hm_warr = []
        cm_hm_warr.extend(cm_out_hm_arr if out_sw_term == 'vcm' and (out_cmos_cm or out_cm) else [])
        cm_hm_warr.extend(cm_in_hm_arr if in_sw_term == 'vcm' else [])

        # Connect vcm
        vcm_xm_list = []
        vcm_vm_list = []
        for vcm in cm_hm_warr:
            _tidx = vm_locs[-2]
            vcm_vm = [self.connect_to_tracks(vcm, TrackID(vm_layer, _tidx, tr_w_sup_vm))]
            _tidx = self.arr_info.col_to_track(vm_layer, -self.min_sep_col // 2, mode=RoundMode.NEAREST)
            vcm_vm.append(self.connect_to_tracks(vcm, TrackID(vm_layer, _tidx, tr_w_sup_vm)))
            xm_tidx_coord = self.grid.track_to_coord(hm_layer, vcm.track_id.base_index)
            xm_tdix = self.grid.coord_to_track(xm_layer, xm_tidx_coord, RoundMode.NEAREST)
            vcm_xm_list.append(self.connect_to_tracks(vcm_vm, TrackID(xm_layer, xm_tdix, tr_w_sup_xm),
                                                      track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))
            vcm_vm_list.extend(vcm_vm)
        self.connect_wires(vcm_vm_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
        self.add_pin('vcm', vcm_xm_list, connect=True)
        self.add_pin('VSS', vss_xm_list)
        self.add_pin('VDD', vdd_xm_list)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=dict(
                sw_n=self.get_row_info(ridx_n, 0).threshold,
                sw_p=self.get_row_info(ridx_p, 0).threshold,
                sw_cm=self.get_row_info(ridx_cm, 0).threshold,
                sw_mid=self.get_row_info(ridx_cm, 0).threshold,
            ),
            out_cm=out_cm,
            in_cmos_cm=in_cmos_cm,
            out_cmos_cm=out_cmos_cm,
            one_phase=self.params['one_phase'],
            in_sw_term=in_sw_term,
            out_sw_term=out_sw_term,
            flip_inout=flip_inout,
        )


class IsolationSW(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_iso_sw')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = IsolationSWHalf.get_params_info()
        ans['even_center'] = 'True to force center column to be even.'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = IsolationSWHalf.get_default_param_values()
        ans['even_center'] = False
        return ans

    def draw_layout(self):
        master: IsolationSWHalf = self.new_template(IsolationSWHalf, params=self.params)
        self.draw_base(master.draw_base_info)
        even_center: bool = self.params['even_center']

        # placement
        nsep = self.min_sep_col
        nsep += (nsep & 1)
        if even_center and nsep % 4 == 2:
            nsep += 2

        nhalf = master.num_cols
        corel = self.add_tile(master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(master, 0, nhalf + nsep)
        self.set_mos_size(num_cols=nsep + 2 * nhalf)

        # self.connect_wires([corel.get_pin('mid'), corer.get_pin('mid')])
        vdd_xm = self.connect_wires(corel.get_all_port_pins('VDD') + corer.get_all_port_pins('VDD'))
        vss_xm = self.connect_wires(corel.get_all_port_pins('VSS') + corer.get_all_port_pins('VSS'))

        self.add_pin('phi1', corel.get_all_port_pins('phi1') + corer.get_all_port_pins('phi1'), connect=True)
        self.add_pin('phi1_b', corel.get_all_port_pins('phi1b') + corer.get_all_port_pins('phi1b'), connect=True)
        self.add_pin('phi2', corel.get_all_port_pins('phi2') + corer.get_all_port_pins('phi2'), connect=True)
        self.add_pin('phi1_hm', corel.get_all_port_pins('phi1_hm') + corer.get_all_port_pins('phi1_hm'), connect=True)
        self.add_pin('phi1b_hm', corel.get_all_port_pins('phi1b_hm') + corer.get_all_port_pins('phi1b_hm'),
                     connect=True)
        self.add_pin('phi2_hm', corel.get_all_port_pins('phi2_hm') + corer.get_all_port_pins('phi2_hm'), connect=True)
        if corel.has_port('phi2b'):
            self.add_pin('phi2_b', corel.get_all_port_pins('phi2b') + corer.get_all_port_pins('phi2b'),
                         connect=True)
            self.add_pin('phi2b_hm', corel.get_all_port_pins('phi2b_hm') + corer.get_all_port_pins('phi2b_hm'),
                         connect=True)
        self.add_pin('vcm', self.connect_wires(corel.get_all_port_pins('vcm') +
                                               corer.get_all_port_pins('vcm')), connect=True)
        self.reexport(corer.get_port('out'), net_name='out_p')
        self.reexport(corel.get_port('out'), net_name='out_n')
        self.reexport(corer.get_port('in'), net_name='in_p')
        self.reexport(corel.get_port('in'), net_name='in_n')

        self.sch_params = master.sch_params
        self.add_pin('VSS', vss_xm, connect=True)
        self.add_pin('VDD', vdd_xm, connect=True)


class CMSwitch(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg='segments dictionary.',
            w='widths.',
            ncols_tot='Total number of fingersa',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w=4,
            ncols_tot=0,
        )

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        tr_manager = self.tr_manager

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg: int = self.params['seg']
        w: int = self.params['w']
        tap_ncol = self.get_tap_ncol(tile_idx=0)
        tap_sep_col = self.sub_sep_col
        tap_ncol += tap_sep_col

        vdd_list, vss_list = [], []
        tot_cols = max(self.params['ncols_tot'], seg + 2 * tap_ncol) + 2 * self.min_sep_col
        self.add_tap(0, vdd_list, vss_list, tile_idx=0)
        sw = self.add_mos(0, (tot_cols - seg) // 2, seg, w=w)
        self.add_tap(tot_cols, vdd_list, vss_list, tile_idx=0, flip_lr=True)
        self.set_mos_size()

        tid_g = self.get_track_id(0, MOSWireType.G, wire_name='ctrl', wire_idx=0)
        tid_sig = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=0)
        tid_ref = self.get_track_id(0, MOSWireType.DS, wire_name='sup', wire_idx=1)

        sam_hm = self.connect_to_tracks(sw.g, tid_g)
        ref_hm = self.connect_to_tracks(sw.d, tid_ref)
        sig_hm = self.connect_to_tracks(sw.s, tid_sig)

        # get middle track for sample signal
        mid_vm_tidx = self.arr_info.col_to_track(vm_layer, tot_cols // 2, RoundMode.NEAREST)
        sam_vm = self.connect_to_tracks(sam_hm, TrackID(vm_layer, mid_vm_tidx, tr_manager.get_width(vm_layer, 'ctrl')))
        tid_l = self.arr_info.col_to_track(vm_layer, tap_ncol, mode=RoundMode.NEAREST)
        tid_r = self.arr_info.col_to_track(vm_layer, self.num_cols - tap_ncol, mode=RoundMode.NEAREST)

        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        vm_l_locs = self.get_available_tracks(vm_layer, tid_l, mid_vm_tidx, self.bound_box.yl, self.bound_box.yh,
                                              tr_manager.get_width(vm_layer, 'sup'),
                                              tr_manager.get_sep(vm_layer, ('sup', 'sup')))
        vm_r_locs = get_available_tracks_reverse(self, vm_layer, mid_vm_tidx, tid_r, self.bound_box.yl,
                                                 self.bound_box.yh, tr_manager.get_width(vm_layer, 'sup'),
                                                 tr_manager.get_sep(vm_layer, ('sup', 'sup')))
        if len(vm_l_locs) & 1:
            vm_l_locs.pop(-1)
        if len(vm_r_locs) & 1:
            vm_r_locs.pop(-1)
        vref_vm_locs = vm_l_locs[::2] + vm_r_locs[::2]
        sig_vm_locs = vm_l_locs[1::2] + vm_r_locs[1::2]
        vref_vm = [self.connect_to_tracks(ref_hm, TrackID(vm_layer, _tid, tr_w_sup_vm)) for _tid in vref_vm_locs]
        sig_vm = [self.connect_to_tracks(sig_hm, TrackID(vm_layer, _tid, tr_w_sup_vm)) for _tid in sig_vm_locs]
        vm_warrs = vref_vm + sig_vm
        vm_warrs_max_coord, vm_warrs_min_coord = max([v.upper for v in vm_warrs]), min([v.lower for v in vm_warrs])
        vref_vm = self.extend_wires(vref_vm, upper=vm_warrs_max_coord, lower=vm_warrs_min_coord)
        sig_vm = self.extend_wires(sig_vm, upper=vm_warrs_max_coord, lower=vm_warrs_min_coord)

        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        # Connect supplies
        tr_sup_hm_w = tr_manager.get_width(hm_layer, 'sup')
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
        sup_hm_tids = [self.get_track_id(0, MOSWireType.DS, wire_name='sup')]

        sup_hm_list = []
        for tid in sup_hm_tids:
            sup_hm_list.append(self.connect_to_tracks(vss_list, tid))

        sup_vm_locs = self.get_available_tracks(vm_layer,
                                                self.arr_info.col_to_track(vm_layer, 0),
                                                self.arr_info.col_to_track(vm_layer, tap_ncol),
                                                self.bound_box.yl, self.bound_box.yh,
                                                tr_manager.get_width(vm_layer, 'sup'),
                                                tr_manager.get_sep(vm_layer, ('sup', 'sup')),
                                                include_last=True)[::2]
        sup_vm_locs += get_available_tracks_reverse(self, vm_layer,
                                                    self.arr_info.col_to_track(vm_layer, self.num_cols - tap_ncol,
                                                                               RoundMode.NEAREST),
                                                    self.arr_info.col_to_track(vm_layer, self.num_cols,
                                                                               RoundMode.NEAREST),
                                                    self.bound_box.yl, self.bound_box.yh,
                                                    tr_manager.get_width(vm_layer, 'sup'),
                                                    tr_manager.get_sep(vm_layer, ('sup', 'sup')),
                                                    include_last=True)[::2]

        sup_vm_list = []
        for tid in sup_vm_locs:
            sup_vm_list.append(self.connect_to_tracks(sup_hm_list, TrackID(vm_layer, tid, tr_sup_vm_w)))

        tile_info, yb, _ = self.get_tile_info(0)
        tile_height = tile_info.height
        xm_locs = self.get_available_tracks(xm_layer, self.grid.coord_to_track(xm_layer, yb, RoundMode.NEAREST),
                                            self.grid.coord_to_track(xm_layer, yb + tile_height, RoundMode.NEAREST),
                                            self.bound_box.xl, self.bound_box.xh, tr_sup_xm_w,
                                            tr_manager.get_sep(xm_layer, ('sup', 'sup')), False)
        if not len(xm_locs):
            xm_locs = xm_locs[:-1]
        # y_mid_coord = (self.bound_box.yl + self.bound_box.yh) // 2
        # xm_mid_tidx = self.grid.coord_to_track(xm_layer, y_mid_coord, mode=RoundMode.NEAREST)
        vref_xm = self.connect_to_tracks(vref_vm, TrackID(xm_layer, xm_locs[len(xm_locs) // 2], tr_w_sup_xm))
        xm_locs.pop(len(xm_locs) // 2)
        sig_xm = self.connect_to_tracks(sig_vm, TrackID(xm_layer, xm_locs[len(xm_locs) // 2], tr_w_sup_xm))
        xm_locs.pop(len(xm_locs) // 2)
        sup_xm_list = []
        for tid in xm_locs[1:]:
            sup_xm_list.append(self.connect_to_tracks(sup_vm_list, TrackID(xm_layer, tid, tr_sup_xm_w)))
        self.add_pin('sam', sam_vm)
        self.add_pin('ref', vref_xm)
        self.add_pin('sig', sig_xm)
        self.add_pin('VSS', sup_xm_list)

        self.sch_params = dict(
            l=self.arr_info.lch,
            nf=seg,
            w=w,
            intent=self.place_info.get_row_place_info(0).row_info.threshold,
        )


class ResidueAmpSwMid(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_mid_fb_sw')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            fb_sw_params='',
            isolate_sw_mid_params='',
            tr_nums='',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            ncols_tot=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(ncols_tot=0, tr_nums={})

    def fill_tap(self, tile_idx, start_col, stop_col, row_list=None):
        nrow = self.get_tile_info(tile_idx)[0].num_rows
        if row_list:
            for idx in row_list:
                cur_row_type = self.get_tile_info(tile_idx)[0].get_row_place_info(idx).row_info.row_type
                if cur_row_type is MOSType.ptap or cur_row_type is MOSType.nch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
                elif cur_row_type is MOSType.ntap or cur_row_type is MOSType.pch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
        else:
            for idx in range(nrow):
                cur_row_type = self.get_tile_info(tile_idx)[0].get_row_place_info(idx).row_info.row_type
                if cur_row_type is MOSType.ptap or cur_row_type is MOSType.nch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
                elif cur_row_type is MOSType.ntap or cur_row_type is MOSType.pch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)

    def draw_layout(self) -> None:
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['isolate_sw_mid_params']['pinfo'])
        self.draw_base(place_info)
        tr_manager = self.tr_manager

        fb_sw_params: Param = self.params['fb_sw_params']
        isolate_sw_mid_params: Param = self.params['isolate_sw_mid_params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

        # Create templates
        fb_sw_master: MOSBase = self.new_template(FBCapSW, params=fb_sw_params)
        iso_sw_mid_master: MOSBase = self.new_template(IsolationSW, params=isolate_sw_mid_params)

        # Setup layers
        conn_layer = iso_sw_mid_master.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        # left ym margin for cmfb cap routing
        ym_cap_ntr, _ = tr_manager.place_wires(ym_layer, ['cap'] * 3)
        ym_cap_ntr_col = self.arr_info.track_to_col(ym_layer, ym_cap_ntr, RoundMode.NEAREST)

        # Floorplaning and add instances
        cur_col = 0
        tot_ncols = max(iso_sw_mid_master.num_cols + 2 * fb_sw_master.num_cols + 4 * self.min_sep_col,
                        self.params['ncols_tot'])
        tot_ncols += tot_ncols & 1
        sw_fb_n = self.add_tile(fb_sw_master, 0, cur_col + (tot_ncols - iso_sw_mid_master.num_cols) // 2, flip_lr=True)
        sw_mid = self.add_tile(iso_sw_mid_master, 0, cur_col + (tot_ncols - iso_sw_mid_master.num_cols) // 2)
        sw_fb_p = self.add_tile(fb_sw_master, 0, cur_col + (tot_ncols + iso_sw_mid_master.num_cols) // 2, flip_lr=False)

        self.set_mos_size(num_cols=tot_ncols)

        # === Connections
        # Connect to amp in and out
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        #
        # Connect clock signals
        tr_w_ctrl_xm = tr_manager.get_width(xm_layer, 'ctrl')
        # sw mid out
        # sw_midout_samb = self.connect_wires(sw_out.get_all_port_pins('phi1_b'))
        sw_midout_amp = self.connect_wires(sw_mid.get_all_port_pins('phi1'))
        sw_midout_ampb = self.connect_wires(sw_mid.get_all_port_pins('phi1_b'))
        self.add_pin('phi1_vm', sw_midout_amp, hide=True)
        self.add_pin('phi1_b_vm', sw_midout_ampb, hide=True)
        # sw fb amp/ampb
        sw_fb_amp_xm_tidx = self.grid.coord_to_track(xm_layer, min(sw_fb_n.get_pin('phi1').upper,
                                                                   sw_fb_p.get_pin('phi1b').upper), RoundMode.NEAREST)
        sw_fb_ampb_xm_tidx = tr_manager.get_next_track(xm_layer, sw_fb_amp_xm_tidx, 'ctrl', 'ctrl', up=False)
        sw_mid_out_tidx = tr_manager.get_next_track(xm_layer, sw_fb_amp_xm_tidx, 'ctrl', 'sig', up=True)
        sw_fb_amp_xm_tidx_top = self.grid.coord_to_track(xm_layer, sw_mid.bound_box.yh, RoundMode.LESS_EQ)
        sw_fb_amp_xm_tidx_bot = self.grid.coord_to_track(xm_layer, sw_mid.bound_box.yl, RoundMode.GREATER_EQ)

        sw_in_xm_locs = self.get_available_tracks(xm_layer, sw_fb_amp_xm_tidx_bot, sw_fb_amp_xm_tidx_top,
                                                  self.bound_box.xl, self.bound_box.xh, tr_w_sig_xm)

        sw_amp_xm_locs = self.get_available_tracks(xm_layer, sw_fb_amp_xm_tidx_bot, sw_fb_amp_xm_tidx_top,
                                                   self.bound_box.xl, self.bound_box.xh, tr_w_ctrl_xm,
                                                   sep=tr_manager.get_sep(xm_layer, ('ctrl', 'ctrl')))
        sw_mid_in_n = self.connect_to_tracks(sw_mid.get_all_port_pins('in_n') + sw_fb_n.get_all_port_pins('out'),
                                             TrackID(xm_layer, sw_in_xm_locs[1], tr_w_sig_xm))
        sw_mid_in_p = self.connect_to_tracks(sw_mid.get_all_port_pins('in_p') + sw_fb_p.get_all_port_pins('out'),
                                             TrackID(xm_layer, sw_in_xm_locs[1], tr_w_sig_xm))

        amp_xm = self.connect_to_tracks(sw_midout_amp, TrackID(xm_layer, sw_amp_xm_locs[0], tr_w_ctrl_xm))
        amp_b_xm = self.connect_to_tracks(sw_midout_ampb, TrackID(xm_layer, sw_amp_xm_locs[1], tr_w_ctrl_xm))
        self.connect_wires(sw_mid.get_all_port_pins('phi1_hm') + sw_mid.get_all_port_pins('phi1_hm')
                           + sw_fb_n.get_all_port_pins('phi1_hm') + sw_fb_p.get_all_port_pins('phi1_hm'))
        self.connect_wires(sw_mid.get_all_port_pins('phi1b_hm') + sw_mid.get_all_port_pins('phi1b_hm')
                           + sw_fb_n.get_all_port_pins('phi1b_hm') + sw_fb_p.get_all_port_pins('phi1b_hm'))
        # -- clk signals ym
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        _, clk_ym_locs = tr_manager.place_wires(ym_layer, ['ctrl', 'ctrl', 'dum', 'ctrl', 'ctrl'],
                                                center_coord=(self.bound_box.xl + self.bound_box.xh) // 2)
        clk_ym_locs.pop(2)

        # ra enable
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        _, clk_ym_locs = tr_manager.place_wires(ym_layer, ['ctrl', 'ctrl', 'dum', 'ctrl', 'ctrl'],
                                                center_coord=(self.bound_box.xl + self.bound_box.xh) // 2)
        clk_ym_locs.pop(2)

        amp_ym = [self.connect_to_tracks(amp_xm, TrackID(ym_layer, clk_ym_locs[0], tr_w_ctrl_ym),
                                         track_upper=self.bound_box.yh),
                  self.connect_to_tracks(amp_xm, TrackID(ym_layer, clk_ym_locs[-1], tr_w_ctrl_ym),
                                         track_upper=self.bound_box.yh)]
        amp_b_ym = [self.connect_to_tracks(amp_b_xm, TrackID(ym_layer, clk_ym_locs[1], tr_w_ctrl_ym),
                                           track_upper=self.bound_box.yh),
                    self.connect_to_tracks(amp_b_xm, TrackID(ym_layer, clk_ym_locs[-2], tr_w_ctrl_ym),
                                           track_upper=self.bound_box.yh)]

        out_xm_locs = self.get_available_tracks(xm_layer, sw_fb_amp_xm_tidx_bot, sw_fb_amp_xm_tidx_top,
                                                self.bound_box.xl, self.bound_box.xh, tr_w_sig_xm)

        sw_mid_out_n = self.connect_to_tracks(sw_mid.get_all_port_pins('out_n'),
                                              TrackID(xm_layer, out_xm_locs[0], tr_w_sig_xm))
        sw_mid_out_p = self.connect_to_tracks(sw_mid.get_all_port_pins('out_p'),
                                              TrackID(xm_layer, out_xm_locs[0], tr_w_sig_xm))

        vcm_sw_p_xm = self.connect_to_tracks(sw_fb_p.get_all_port_pins('vcm_sw'),
                                             TrackID(xm_layer, out_xm_locs[0], tr_w_sig_xm))
        vcm_sw_n_xm = self.connect_to_tracks(sw_fb_n.get_all_port_pins('vcm_sw'),
                                             TrackID(xm_layer, out_xm_locs[0], tr_w_sig_xm))
        # VCM
        vcm_xm = self.connect_to_track_wires(sw_fb_n.get_all_port_pins('vcm') + sw_fb_p.get_all_port_pins('vcm'),
                                             sw_mid.get_all_port_pins('VSS'))
        #
        tr_nums = self.params['tr_nums']

        def coord_to_track_list(layer, coord, num_tr, tr_type, align='left'):
            middle_tidx = self.grid.coord_to_track(layer, coord, RoundMode.NEAREST)
            tr_w = tr_manager.get_width(layer, tr_type)
            if tr_w > 0:
                ntr, locs = tr_manager.place_wires(layer, [tr_type] * num_tr,
                                                   align_idx=num_tr // 2 if align == 'left' else -num_tr // 2 - 1,
                                                   align_track=middle_tidx)
            else:
                locs = TrackID(layer, middle_tidx, tr_w, grid=self.grid)

            return locs

        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        sw_out_n_ym_tidx = coord_to_track_list(ym_layer, sw_mid_out_n.middle, tr_nums['sig'][ym_layer], 'sig')
        sw_out_p_ym_tidx = coord_to_track_list(ym_layer, sw_mid_out_p.middle, tr_nums['sig'][ym_layer], 'sig',
                                               align='right')
        if tr_w_sig_ym > 0:
            sw_out_n = [self.connect_to_tracks(sw_mid_out_n, TrackID(ym_layer, idx, tr_w_sig_ym),
                                               min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
                        for idx in sw_out_n_ym_tidx]
            sw_out_p = [self.connect_to_tracks(sw_mid_out_p, TrackID(ym_layer, idx, tr_w_sig_ym),
                                               min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
                        for idx in sw_out_p_ym_tidx]

            sw_in_n = [self.connect_to_tracks(sw_mid_in_n, TrackID(ym_layer, idx, tr_w_sig_ym),
                                              min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yl)
                       for idx in sw_out_n_ym_tidx]
            sw_in_p = [self.connect_to_tracks(sw_mid_in_p, TrackID(ym_layer, idx, tr_w_sig_ym),
                                              min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yl)
                       for idx in sw_out_p_ym_tidx]
        else:
            sw_out_n = self.connect_to_tracks(sw_mid_out_n, sw_out_n_ym_tidx,
                                              min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
            sw_out_p = self.connect_to_tracks(sw_mid_out_p, sw_out_p_ym_tidx,
                                              min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
            sw_in_n = self.connect_to_tracks(sw_mid_in_n, sw_out_n_ym_tidx,
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
            sw_in_p = self.connect_to_tracks(sw_mid_in_p, sw_out_p_ym_tidx,
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)

        fb_in_n_xm = self.connect_to_tracks(sw_fb_n.get_pin('in'), TrackID(xm_layer, sw_in_xm_locs[0], tr_w_sig_xm))
        fb_in_p_xm = self.connect_to_tracks(sw_fb_p.get_pin('in'), TrackID(xm_layer, sw_in_xm_locs[0], tr_w_sig_xm))

        fb_in_n_ym_tidx = coord_to_track_list(ym_layer, fb_in_n_xm.lower, tr_nums['sig'][ym_layer], 'sig')
        fb_in_p_ym_tidx = coord_to_track_list(ym_layer, fb_in_p_xm.upper, tr_nums['sig'][ym_layer], 'sig',
                                              align='right')

        if tr_w_sig_ym > 0:
            fb_in_n_ym = [self.connect_to_tracks(fb_in_n_xm, TrackID(ym_layer, tidx, tr_w_sig_ym),
                                                 min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
                          for tidx in fb_in_n_ym_tidx]
            fb_in_p_ym = [self.connect_to_tracks(fb_in_p_xm, TrackID(ym_layer, tidx, tr_w_sig_ym),
                                                 min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
                          for tidx in fb_in_p_ym_tidx]
        else:
            fb_in_n_ym = self.connect_to_tracks(fb_in_n_xm, fb_in_n_ym_tidx)
            fb_in_p_ym = self.connect_to_tracks(fb_in_p_xm, fb_in_p_ym_tidx)

        vcm_n_ym_tidx = TrackID(ym_layer, self.grid.coord_to_track(ym_layer, fb_in_n_xm.middle, RoundMode.LESS),
                                tr_manager.get_width(ym_layer, 'sup_w'))
        vcm_p_ym_tidx = TrackID(ym_layer, self.grid.coord_to_track(ym_layer, fb_in_p_xm.middle, RoundMode.GREATER),
                                tr_manager.get_width(ym_layer, 'sup_w'))
        vcm_sw_p_ym = self.connect_to_tracks(vcm_sw_p_xm, vcm_p_ym_tidx)
        vcm_sw_n_ym = self.connect_to_tracks(vcm_sw_n_xm, vcm_n_ym_tidx)

        xm1_layer = ym_layer + 1
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        fb_in_xm1_tidx = self.grid.coord_to_track(xm_layer, fb_in_n_ym.upper, RoundMode.LESS)
        fb_in_n_xm1 = self.connect_to_tracks(fb_in_n_ym, TrackID(xm1_layer, fb_in_xm1_tidx, tr_w_sig_xm1,
                                                                 grid=self.grid))
        fb_in_p_xm1 = self.connect_to_tracks(fb_in_p_ym, TrackID(xm1_layer, fb_in_xm1_tidx, tr_w_sig_xm1,
                                                                 grid=self.grid))

        # Connect amp out to xm1 laeyr
        amp_out_xm1_tidx_mid = self.grid.coord_to_track(xm1_layer, sw_in_p.lower, RoundMode.NEAREST)
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        amp_out_xm1_sep = self.get_track_sep(xm1_layer, tr_w_sig_xm1, tr_w_sig_xm1).div2()
        amp_out_xm1_tidx = [amp_out_xm1_tidx_mid - amp_out_xm1_sep, amp_out_xm1_tidx_mid + amp_out_xm1_sep]

        amp_outp_xm1 = self.connect_to_tracks(sw_in_p,
                                              TrackID(xm1_layer, amp_out_xm1_tidx[0], tr_w_sig_xm1, grid=self.grid))
        amp_outn_xm1 = self.connect_to_tracks(sw_in_n,
                                              TrackID(xm1_layer, amp_out_xm1_tidx[1], tr_w_sig_xm1, grid=self.grid))
        [amp_outp_xm1, amp_outn_xm1] = self.match_warr_length([amp_outp_xm1, amp_outn_xm1])

        ym1_layer = xm1_layer + 1
        fb_in_n_ym1_tidx = self.grid.coord_to_track(ym1_layer, fb_in_n_xm1.middle, RoundMode.GREATER_EQ)
        fb_in_p_ym1_tidx = self.grid.coord_to_track(ym1_layer, fb_in_p_xm1.middle, RoundMode.LESS_EQ)
        tr_w_sig_ym1 = tr_manager.get_width(ym1_layer, 'sig')
        fb_in_n_ym1 = self.connect_to_tracks(fb_in_n_xm1, TrackID(ym1_layer, fb_in_n_ym1_tidx, tr_w_sig_ym1))
        fb_in_p_ym1 = self.connect_to_tracks(fb_in_p_xm1, TrackID(ym1_layer, fb_in_p_ym1_tidx, tr_w_sig_ym1))

        self.add_pin('amp_out_p_xm1', amp_outp_xm1, hide=True)
        self.add_pin('amp_out_n_xm1', amp_outn_xm1, hide=True)
        self.add_pin('amp_out_n', sw_in_n)
        self.add_pin('amp_out_p', sw_in_p)
        self.add_pin('fb_in_n', fb_in_n_ym1)
        self.add_pin('fb_in_p', fb_in_p_ym1)
        self.add_pin('out_n', sw_out_n)
        self.add_pin('out_p', sw_out_p)
        self.add_pin('amp', amp_ym)
        self.add_pin('amp_b', amp_b_ym)

        # Connect to ym supplies
        tot_cols = self.num_cols
        mid_ym_tidx = self.arr_info.col_to_track(ym_layer, tot_cols // 2, RoundMode.NEAREST)
        tid_l = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.NEAREST)
        tid_r = self.arr_info.col_to_track(ym_layer, tot_cols, mode=RoundMode.NEAREST)

        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        ym_l_locs = self.get_available_tracks(ym_layer, tid_l, mid_ym_tidx, self.bound_box.yl, self.bound_box.yh,
                                              tr_manager.get_width(vm_layer, 'sup'),
                                              tr_manager.get_sep(ym_layer, ('sup', 'sup')))
        ym_r_locs = get_available_tracks_reverse(self, ym_layer, mid_ym_tidx, tid_r, self.bound_box.yl,
                                                 self.bound_box.yh, tr_manager.get_width(vm_layer, 'sup'),
                                                 tr_manager.get_sep(ym_layer, ('sup', 'sup')))
        vdd_xm_list, vss_xm_list, vcm_xm_list = [], [], []
        inst_list = [sw_mid]
        for inst in inst_list:
            for p, l in zip(['VDD', 'VSS', 'vcm'], [vdd_xm_list, vss_xm_list, vcm_xm_list]):
                if inst.has_port(p):
                    l.extend(inst.get_all_port_pins(p))

        ym_r_locs = ym_r_locs[1:-1]
        ym_l_locs = ym_l_locs[1:-1]

        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        vdd_ym_locs = ym_r_locs[::3] + ym_l_locs[::3]
        vss_ym_locs = ym_r_locs[1::3] + ym_l_locs[1::3]
        vcm_ym_locs = ym_r_locs[2::3] + ym_l_locs[2::3]

        vdd_ym_list = [self.connect_to_tracks(vdd_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym),
                                              track_upper=self.bound_box.yh, track_lower=self.bound_box.yl,
                                              wire_lower=self.bound_box.xl, wire_upper=self.bound_box.xh)
                       for tid in vdd_ym_locs]
        vss_ym_list = [self.connect_to_tracks(vss_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym),
                                              track_upper=self.bound_box.yh, track_lower=self.bound_box.yl,
                                              wire_lower=self.bound_box.xl, wire_upper=self.bound_box.xh)
                       for tid in vss_ym_locs]
        vcm_ym_list = [self.connect_to_tracks(vcm_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym),
                                              track_upper=self.bound_box.yh, track_lower=self.bound_box.yl,
                                              wire_lower=self.bound_box.xl, wire_upper=self.bound_box.xh)
                       for tid in vcm_ym_locs]

        self.add_pin('VDD_ym', vdd_ym_list, label='VDD')
        self.add_pin('VSS_ym', vss_ym_list, label='VSS')
        self.add_pin('vcm_ym', vcm_ym_list, label='vcm')

        vss_xm1_list, vdd_xm1_list = [], []
        vcm_xm1_list = []
        xm1_layer = vdd_ym_list[0].layer_id + 1
        sup_xm1_locs = \
            self.get_available_tracks(xm1_layer,
                                      self.grid.coord_to_track(xm1_layer, self.bound_box.yl, RoundMode.NEAREST),
                                      self.grid.coord_to_track(xm1_layer, self.bound_box.yh, RoundMode.NEAREST),
                                      self.bound_box.xl, self.bound_box.xh, tr_manager.get_width(xm1_layer, 'sup'),
                                      tr_manager.get_sep(xm1_layer, ('sup', 'sup')),
                                      include_last=True)
        vdd_xm1_tidx_list, vss_xm1_tidx_list, vcm_xm1_tidx_list = \
            sup_xm1_locs[1::3], sup_xm1_locs[::3], sup_xm1_locs[2::3]

        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        for tidx in vdd_xm1_tidx_list:
            vdd_xm1_list.append(self.connect_to_tracks(vdd_ym_list, TrackID(xm1_layer, tidx, tr_w_sup_xm1),
                                                       track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))
        for tidx in vss_xm1_tidx_list:
            vss_xm1_list.append(self.connect_to_tracks(vss_ym_list, TrackID(xm1_layer, tidx, tr_w_sup_xm1),
                                                       track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))
        for tidx in vcm_xm1_tidx_list:
            vcm_xm1_list.append(self.connect_to_tracks(vcm_ym_list, TrackID(xm1_layer, tidx, tr_w_sup_xm1),
                                                       track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))

        self.connect_to_track_wires([vcm_sw_p_ym, vcm_sw_n_ym], vss_xm1_list)
        self.add_pin('VDD', vdd_xm1_list)
        self.add_pin('VSS', vss_xm1_list)
        self.add_pin('vcm', vcm_xm1_list)

        for idx in range(self.num_tile_rows):
            _nrow = self.get_tile_info(idx)[0].num_rows
            for jdx in range(_nrow):
                fill_conn_layer_intv(self, idx, jdx, True)

        self._sch_params = dict(
            sw_mid_params=iso_sw_mid_master.sch_params,
            sw_fb_params=fb_sw_master.sch_params,
        )


class ResidueAmpSwOut(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_iso_sw')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            isolate_sw_out_params='',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            tr_nums='',
            ncols_tot='',
            sig_locs='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(ncols_tot=0, tr_nums={}, sig_locs={})

    def fill_tap(self, tile_idx, start_col, stop_col, row_list=None):
        nrow = self.get_tile_info(tile_idx)[0].num_rows
        if row_list:
            for idx in row_list:
                cur_row_type = self.get_tile_info(tile_idx)[0].get_row_place_info(idx).row_info.row_type
                if cur_row_type is MOSType.ptap or cur_row_type is MOSType.nch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
                elif cur_row_type is MOSType.ntap or cur_row_type is MOSType.pch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
        else:
            for idx in range(nrow):
                cur_row_type = self.get_tile_info(tile_idx)[0].get_row_place_info(idx).row_info.row_type
                if cur_row_type is MOSType.ptap or cur_row_type is MOSType.nch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
                elif cur_row_type is MOSType.ntap or cur_row_type is MOSType.pch:
                    self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)

    def draw_layout(self) -> None:
        isolate_sw_out_params: Param = self.params['isolate_sw_out_params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

        iso_sw_out_master: MOSBase = self.new_template(IsolationSW, params=isolate_sw_out_params)

        # use Ptap tile as dummy tile, leave space for routing
        tinfo_table = iso_sw_out_master.tile_table
        dummy_pinfo = tinfo_table['dummy_tile']
        dummy_pinfo = TilePatternElement(dummy_pinfo)

        tile_ele = [dummy_pinfo, dummy_pinfo, iso_sw_out_master.get_tile_pattern_element(), dummy_pinfo]

        self.draw_base((TilePattern(tile_ele), iso_sw_out_master.draw_base_info[1]))

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      isolate_sw_out_params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        # left ym margin for cmfb cap routing
        ym_cap_ntr, _ = tr_manager.place_wires(ym_layer, ['cap'] * 3)
        ym_cap_ntr_col = self.arr_info.track_to_col(ym_layer, ym_cap_ntr, RoundMode.NEAREST)

        ra_tile_idx = 0
        cur_col = 0
        tot_ncols = max(iso_sw_out_master.num_cols, self.params['ncols_tot'])
        sw_out = self.add_tile(iso_sw_out_master, 2, cur_col + (tot_ncols - iso_sw_out_master.num_cols) // 2)

        self.set_mos_size(num_cols=tot_ncols, num_tiles=3)

        # === Connections
        def get_tile_ymid(tile_idx):
            tile_pinfo = self.get_tile_info(tile_idx)
            return tile_pinfo[1] + tile_pinfo[0].height // 2

        # inst_list = [sw_out]
        # for inst in inst_list:
        #     for pin in inst.port_names_iter():
        #         self.reexport(inst.get_port(pin), connect=True)

        # Connect to amp in and out
        amp_in_xm_tidx = min([x.track_id.base_index for x in sw_out.get_all_port_pins('VSS')[0].to_warr_list()])
        amp_in_xm_tidx = tr_manager.get_next_track(xm_layer, amp_in_xm_tidx, 'sup', 'sig', up=False)
        amp_in_n, amp_in_p = sw_out.get_all_port_pins('in_n'), sw_out.get_all_port_pins('in_p')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        amp_in_n_xm = self.connect_to_tracks(amp_in_n, TrackID(xm_layer, amp_in_xm_tidx, tr_w_sig_xm))
        amp_in_p_xm = self.connect_to_tracks(amp_in_p, TrackID(xm_layer, amp_in_xm_tidx, tr_w_sig_xm))

        # Connect clock signals
        tr_w_ctrl_xm = tr_manager.get_width(xm_layer, 'ctrl')
        sw_midout_sam = self.connect_wires(sw_out.get_all_port_pins('phi1'))
        sw_midout_samb = self.connect_wires(sw_out.get_all_port_pins('phi1_b'))
        sw_midout_amp = self.connect_wires(sw_out.get_all_port_pins('phi1_b'))
        sw_midout_ampb = self.connect_wires(sw_out.get_all_port_pins('phi1'))

        #
        sam_xm_y = get_tile_ymid(0)
        sw_midout_xm_start = self.grid.coord_to_track(xm_layer, sam_xm_y, RoundMode.NEAREST)
        _, sw_midout_xm_locs = tr_manager.place_wires(xm_layer, ['ctrl'] * 5, align_track=sw_midout_xm_start)
        sw_midout_amp_xm = self.connect_to_tracks(sw_midout_amp,
                                                  TrackID(xm_layer, sw_midout_xm_locs[1], tr_w_ctrl_xm))
        sw_midout_ampb_xm = self.connect_to_tracks(sw_midout_ampb + sw_midout_sam,
                                                   TrackID(xm_layer, sw_midout_xm_locs[2], tr_w_ctrl_xm))
        self.add_pin('phi1_b_vm', sw_midout_amp, hide=True)
        self.add_pin('phi1_vm', sw_midout_ampb + sw_midout_sam, hide=True)
        #
        # -- clk signals ym
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        _, clk_ym_locs = tr_manager.place_wires(ym_layer, ['ctrl', 'ctrl', 'dum', 'ctrl', 'ctrl'],
                                                center_coord=(self.bound_box.xl + self.bound_box.xh) // 2)
        clk_ym_locs.pop(2)

        # # ra enable
        # _, ra_enable_xm_locs = tr_manager.place_wires(xm_layer, ['ctrl'] * 2, center_coord=ra.get_pin('en').upper)
        # ra_amp_xm = self.connect_to_tracks(ra.get_all_port_pins('en'), TrackID(xm_layer, ra_enable_xm_locs[0],
        #                                                                        tr_w_ctrl_xm))
        # ra_ampb_xm = self.connect_to_tracks(ra.get_all_port_pins('enb'), TrackID(xm_layer, ra_enable_xm_locs[1],
        #                                                                          tr_w_ctrl_xm))
        # amp_en_vm_coord = self.arr_info.col_to_coord(self.num_cols +
        #                                              (-(tot_ncols - ra_master.num_cols) // 2) // 2)
        # _, amp_en_vm_locs = tr_manager.place_wires(vm_layer, ['ctrl'] * 2, center_coord=amp_en_vm_coord)
        #
        amp_ym = [self.connect_to_tracks([sw_midout_amp_xm], TrackID(ym_layer, clk_ym_locs[0], tr_w_ctrl_ym),
                                         track_upper=self.bound_box.yh),
                  self.connect_to_tracks([sw_midout_amp_xm], TrackID(ym_layer, clk_ym_locs[-1], tr_w_ctrl_ym),
                                         track_upper=self.bound_box.yh)]
        ampb_ym = [self.connect_to_tracks([sw_midout_ampb_xm], TrackID(ym_layer, clk_ym_locs[1], tr_w_ctrl_ym),
                                          track_upper=self.bound_box.yh),
                   self.connect_to_tracks([sw_midout_ampb_xm], TrackID(ym_layer, clk_ym_locs[-2], tr_w_ctrl_ym),
                                          track_upper=self.bound_box.yh)]

        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')

        def coord_to_track_list(layer, coord, num_tr, tr_type, align='left', alignment=RoundMode.NEAREST):
            middle_tidx = self.grid.coord_to_track(layer, coord, alignment)
            tr_w = tr_manager.get_width(layer, tr_type)
            if tr_w > 0:
                ntr, locs = tr_manager.place_wires(layer, [tr_type] * num_tr,
                                                   align_idx=num_tr // 2 if align == 'left' else -num_tr // 2 - 1,
                                                   align_track=middle_tidx)
            else:
                locs = middle_tidx

            return locs

        tr_nums = self.params['tr_nums']

        out_xm_tidx = max([x.track_id.base_index for x in sw_out.get_all_port_pins('VSS')[0].to_warr_list()])
        out_xm_tidx = tr_manager.get_next_track(xm_layer, out_xm_tidx, 'sup', 'sig')
        sw_out_n = self.connect_to_tracks(sw_out.get_pin('out_n'), TrackID(xm_layer, out_xm_tidx, tr_w_sig_xm))
        sw_out_p = self.connect_to_tracks(sw_out.get_pin('out_p'), TrackID(xm_layer, out_xm_tidx, tr_w_sig_xm))

        sw_out_n_ym_tidx = coord_to_track_list(ym_layer, amp_in_n_xm.middle, tr_nums['sig'][ym_layer], 'sig')
        sw_out_p_ym_tidx = coord_to_track_list(ym_layer, amp_in_p_xm.middle, tr_nums['sig'][ym_layer], 'sig',
                                               align='right')
        sig_locs = self.params['sig_locs']
        sw_out_ym_dtidx = abs(sig_locs.get('sw_in_n', 0) - sig_locs.get('sw_in_p', 0))

        if tr_w_sig_ym > 0:
            sw_out_n = [self.connect_to_tracks(sw_out_n, TrackID(ym_layer, idx, tr_w_sig_ym),
                                               min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
                        for idx in sw_out_n_ym_tidx]
            sw_out_p = [self.connect_to_tracks(sw_out_p, TrackID(ym_layer, idx, tr_w_sig_ym),
                                               min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
                        for idx in sw_out_p_ym_tidx]
        else:
            if sw_out_ym_dtidx > 0:
                sw_out_ym_dtidx = abs(sw_out_ym_dtidx - abs(sw_out_n_ym_tidx - sw_out_p_ym_tidx)) // 2
            else:
                sw_out_ym_dtidx = 0
            sw_out_n_ym_tidx = TrackID(ym_layer, sw_out_n_ym_tidx - sw_out_ym_dtidx, tr_w_sig_ym, grid=self.grid)
            sw_out_p_ym_tidx = TrackID(ym_layer, sw_out_p_ym_tidx + sw_out_ym_dtidx, tr_w_sig_ym, grid=self.grid)
            sw_out_n = self.connect_to_tracks(sw_out_n, sw_out_n_ym_tidx,
                                              min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)
            sw_out_p = self.connect_to_tracks(sw_out_p, sw_out_p_ym_tidx,
                                              min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)

        xm1_layer = ym_layer + 1
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        sw_out_xm1_tidx = self.grid.coord_to_track(xm1_layer, sw_out_n.middle, RoundMode.NEAREST)
        sw_out_n_xm1 = self.connect_to_tracks(sw_out_n, TrackID(xm1_layer, sw_out_xm1_tidx, tr_w_sig_xm1,
                                                                grid=self.grid))
        sw_out_p_xm1 = self.connect_to_tracks(sw_out_p, TrackID(xm1_layer, sw_out_xm1_tidx, tr_w_sig_xm1,
                                                                grid=self.grid))

        ym1_layer = xm1_layer + 1
        tr_w_sig_ym1 = tr_manager.get_width(ym1_layer, 'sig')
        sw_out_n_ym1_tidx = self.grid.coord_to_track(ym1_layer, sw_out_n_xm1.lower, RoundMode.LESS)
        sw_out_p_ym1_tidx = self.grid.coord_to_track(ym1_layer, sw_out_p_xm1.upper, RoundMode.GREATER)

        sw_out_n_ym1 = self.connect_to_tracks(sw_out_n_xm1, TrackID(ym1_layer, sw_out_n_ym1_tidx, tr_w_sig_ym1))
        sw_out_p_ym1 = self.connect_to_tracks(sw_out_p_xm1, TrackID(ym1_layer, sw_out_p_ym1_tidx, tr_w_sig_ym1))

        in_n_ym_tidx = coord_to_track_list(ym_layer, amp_in_n_xm.middle, tr_nums['sig'][ym_layer], 'sig',
                                           alignment=RoundMode.LESS)
        in_p_ym_tidx = coord_to_track_list(ym_layer, amp_in_p_xm.middle, tr_nums['sig'][ym_layer], 'sig',
                                           align='right', alignment=RoundMode.GREATER)
        sw_in_ym_dtidx = abs(sig_locs.get('sw_in_n', 0) - sig_locs.get('sw_in_p', 0))
        if tr_w_sig_ym > 0:
            in_n_ym = [self.connect_to_tracks(amp_in_n_xm, TrackID(ym_layer, idx, tr_w_sig_ym),
                                              min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
                       for idx in in_n_ym_tidx]
            in_p_ym = [self.connect_to_tracks(amp_in_p_xm, TrackID(ym_layer, idx, tr_w_sig_ym),
                                              min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
                       for idx in in_p_ym_tidx]
        else:
            if sw_in_ym_dtidx > 0:
                sw_in_ym_dtidx = abs(sw_in_ym_dtidx - abs(in_n_ym_tidx - in_p_ym_tidx)) // 2
            else:
                sw_in_ym_dtidx = 0
            sw_in_n_ym_tidx = TrackID(ym_layer, in_n_ym_tidx - sw_in_ym_dtidx, tr_w_sig_ym,
                                      grid=self.grid)
            sw_in_p_ym_tidx = TrackID(ym_layer, in_p_ym_tidx + sw_in_ym_dtidx, tr_w_sig_ym,
                                      grid=self.grid)

            in_n_ym = self.connect_to_tracks(amp_in_n_xm, sw_in_n_ym_tidx, min_len_mode=MinLenMode.MIDDLE,
                                             track_lower=self.bound_box.yl)
            in_p_ym = self.connect_to_tracks(amp_in_p_xm, sw_in_p_ym_tidx, min_len_mode=MinLenMode.MIDDLE,
                                             track_lower=self.bound_box.yl)
        #
        # xm1_layer = ym_layer + 1
        # tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        # vcmfb_xm1_tidx = self.grid.coord_to_track(xm1_layer, vcmfb_ym.middle, RoundMode.NEAREST)
        # vcmfb_xm1 = self.connect_to_tracks(vcmfb_ym, TrackID(xm1_layer, vcmfb_xm1_tidx, tr_w_sig_xm1))
        #
        # _, out_xm1_locs = tr_manager.place_wires(xm1_layer, ['sig'] * 2, center_coord=amp_outn_ym.middle)
        # outn_xm, outp_xm = self.connect_matching_tracks([amp_outn_ym, amp_outp_ym], xm1_layer, out_xm1_locs,
        #                                                 width=tr_w_sig_xm1)
        #
        # Connect to ym supplies
        tot_cols = self.num_cols
        mid_ym_tidx = self.arr_info.col_to_track(ym_layer, tot_cols // 2, RoundMode.NEAREST)
        tid_l = self.grid.coord_to_track(ym_layer, self.bound_box.xl, mode=RoundMode.NEAREST)
        tid_r = self.grid.coord_to_track(ym_layer, self.bound_box.xh, mode=RoundMode.NEAREST)

        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        ym_l_locs = self.get_available_tracks(ym_layer, tid_l, mid_ym_tidx, self.bound_box.yl, self.bound_box.yh,
                                              tr_manager.get_width(vm_layer, 'sup'),
                                              tr_manager.get_sep(ym_layer, ('sup', 'sup')))
        ym_r_locs = get_available_tracks_reverse(self, ym_layer, mid_ym_tidx, tid_r, self.bound_box.yl,
                                                 self.bound_box.yh, tr_manager.get_width(vm_layer, 'sup'),
                                                 tr_manager.get_sep(ym_layer, ('sup', 'sup')))
        vdd_xm_list, vss_xm_list, vcm_xm_list = [], [], []
        inst_list = [sw_out]
        for inst in inst_list:
            for p, l in zip(['VDD', 'VSS', 'vcm'], [vdd_xm_list, vss_xm_list, vcm_xm_list]):
                if inst.has_port(p):
                    l.extend(inst.get_all_port_pins(p))

        ym_r_locs = ym_r_locs[1:-1]
        ym_l_locs = ym_l_locs[1:-1]

        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        vdd_xm_locs = ym_r_locs[::3] + ym_l_locs[::3]
        vss_xm_locs = ym_r_locs[1::3] + ym_l_locs[1::3]
        vcm_xm_locs = ym_r_locs[2::3] + ym_l_locs[2::3]

        vdd_ym_list = [self.connect_to_tracks(vdd_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym),
                                              track_upper=self.bound_box.yh, track_lower=self.bound_box.yl)
                       for tid in vdd_xm_locs]
        vss_ym_list = [self.connect_to_tracks(vss_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym),
                                              track_upper=self.bound_box.yh, track_lower=self.bound_box.yl)
                       for tid in vss_xm_locs]
        vcm_ym_list = [self.connect_to_tracks(vcm_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym),
                                              track_upper=self.bound_box.yh, track_lower=self.bound_box.yl)
                       for tid in vcm_xm_locs]
        #
        # self.add_pin('VDD_ym', vdd_ym_list, label='VDD')
        # self.add_pin('VSS_ym', vss_ym_list, label='VSS')
        # self.add_pin('vcm_ym', vcm_ym_list, label='vcm')
        #
        vss_xm1_list, vdd_xm1_list = [], []
        vcm_xm1_list = []
        xm1_layer = vdd_ym_list[0].layer_id + 1
        sup_xm1_locs = \
            self.get_available_tracks(xm1_layer,
                                      self.grid.coord_to_track(xm1_layer, self.bound_box.yl, RoundMode.NEAREST),
                                      self.grid.coord_to_track(xm1_layer, self.bound_box.yh, RoundMode.NEAREST),
                                      self.bound_box.xl, self.bound_box.xh, tr_manager.get_width(xm1_layer, 'sup'),
                                      tr_manager.get_sep(xm1_layer, ('sup', 'sup')),
                                      include_last=True)
        vdd_xm1_tidx_list, vss_xm1_tidx_list, vcm_xm1_tidx_list = \
            sup_xm1_locs[1::3], sup_xm1_locs[::3], sup_xm1_locs[2::3]

        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        for tidx in vdd_xm1_tidx_list:
            vdd_xm1_list.append(self.connect_to_tracks(vdd_ym_list, TrackID(xm1_layer, tidx, tr_w_sup_xm1),
                                                       track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))
        for tidx in vss_xm1_tidx_list:
            vss_xm1_list.append(self.connect_to_tracks(vss_ym_list, TrackID(xm1_layer, tidx, tr_w_sup_xm1),
                                                       track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))
        for tidx in vcm_xm1_tidx_list:
            vcm_xm1_list.append(self.connect_to_tracks(vcm_ym_list, TrackID(xm1_layer, tidx, tr_w_sup_xm1),
                                                       track_lower=self.bound_box.xl, track_upper=self.bound_box.xh))

        self.add_pin('phi1_b', amp_ym)
        self.add_pin('phi1', ampb_ym)
        self.add_pin('in_n', in_n_ym)
        self.add_pin('in_p', in_p_ym)
        self.add_pin('out_n', sw_out_n_ym1)
        self.add_pin('out_p', sw_out_p_ym1)
        self.add_pin('VDD', vdd_xm1_list, connect=True)
        self.add_pin('VSS', vss_xm1_list, connect=True)
        self.add_pin('vcm', vcm_xm1_list, connect=True)

        for idx in range(self.num_tile_rows):
            _nrow = self.get_tile_info(idx)[0].num_rows
            for jdx in range(_nrow):
                fill_conn_layer_intv(self, idx, jdx, True)

        self._sch_params = iso_sw_out_master.sch_params
