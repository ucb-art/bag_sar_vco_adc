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

from typing import Any, Dict, Optional, Type

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.routing import WireArray
from bag.layout.template import TemplateDB
from bag.util.immutable import Param
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from pybag.enum import RoundMode, MinLenMode, PinMode, Orient2D
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import TilePatternElement, TilePattern


class SamplerUnit(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg='segments dictionary.',
            seg_p='segments dictionary.',
            seg_n='segments dictionary.',
            ridx_n='',
            ridx_p='',
            sampler_type='',
            xcp_dummy='',
            xcp_metal='',
            g_on_s='',
            w_n='',
            w_p='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            seg_n=0,
            seg_p=0,
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            sampler_type='nmos',
            xcp_dummy=False,
            xcp_metal=True,
            g_on_s=False,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_n = self.params['seg_n']
        seg_p = self.params['seg_p']
        seg = self.params['seg']
        ridx_n = self.params['ridx_n']
        ridx_p = self.params['ridx_p']
        w_n = self.params['w_n']
        w_p = self.params['w_p']
        sampler_type: str = self.params['sampler_type']

        # check xcp dummy setting
        xcp_metal = self.params['xcp_metal']
        xcp_dummy = self.params['xcp_dummy']
        if xcp_metal and xcp_dummy:
            raise ValueError('sar_samp/SamplerUnit: choose only metal or device dummy, cant enable both')

        if seg_p <= 0:
            seg_p = seg
        if seg_n <= 0:
            seg_n = seg
        if seg_p <= 0 or seg_n <= 0:
            raise ValueError('Invalid segments.')

        has_nmos = sampler_type == 'cmos' or sampler_type == 'nmos'
        has_pmos = sampler_type == 'cmos' or sampler_type == 'pmos'
        if has_pmos:
            raise NotImplementedError

        g_on_s = self.params['g_on_s']
        nsam = self.add_mos(ridx_n, 0, seg_n, w=w_n, g_on_s=g_on_s,
                            tile_idx=1 if xcp_dummy else 0) if has_nmos else None
        psam = self.add_mos(ridx_p, 0, seg_p, w=w_p, g_on_s=g_on_s,
                            tile_idx=1 if xcp_dummy else 0) if has_pmos else None
        max_sam_col = max(seg_n, seg_p)

        nd_conn, ns_conn = ([nsam.d], [nsam.s]) if has_nmos else ([], [])
        pd_conn, ps_conn = ([psam.d], [psam.s]) if has_pmos else ([], [])

        if xcp_dummy:
            ndum = self.add_mos(ridx_n - 1, 0, seg_n, w=w_n) if has_nmos else None
            pdum = self.add_mos(ridx_p + 1, 0, seg_p, w=w_p) if has_pmos else None
            if ndum:
                nd_conn.append(ndum.d)
                # ns_conn.append(ndum.s)
                d_num_tid_n = self.get_track_id(0, MOSWireType.DS, 'ana_sig', 1, tile_idx=0)
                dum_s_n = self.connect_to_tracks(ndum.s, d_num_tid_n)
                self.add_pin('in_c', dum_s_n, connect=True)
                en_tid_n = self.get_track_id(0, MOSWireType.G_MATCH, 'clk', 0, tile_idx=0)
                self.add_pin('VSS', self.connect_to_tracks(ndum.g, en_tid_n))
            if pdum:
                pd_conn.append(pdum.d)
                d_num_tid_p = self.get_track_id(1, MOSWireType.DS, 'ana_sig', 1, tile_idx=0)
                dum_s_p = self.connect_to_tracks(pdum.s, d_num_tid_p)
                self.add_pin('in_c', dum_s_p, connect=True)
                en_tid_n = self.get_track_id(1, MOSWireType.G_MATCH, 'clk', 0, tile_idx=0)
                self.add_pin('VDD', self.connect_to_tracks(pdum.g, en_tid_n))
                # ps_conn.append(pdum.s)

        if xcp_metal:
            pinfo, tile_yb, flip_tile = self.used_array.get_tile_info(0)
            if has_nmos:
                row_info, y0, orient = MOSBase.get_mos_row_info(pinfo, tile_yb, flip_tile, ridx_n - 1)
                if row_info.flip:
                    xcp_metal_upper, xcp_metal_lower = (y0 - row_info.ds_conn_y[0],
                                                        y0 - row_info.ds_conn_y[1])
                else:
                    xcp_metal_lower, xcp_metal_upper = (y0 + row_info.ds_conn_y[0], y0 + row_info.ds_conn_y[1])
                d_start_tidx = self.arr_info.col_to_track(self.conn_layer, 1)
                s_start_tidx = self.arr_info.col_to_track(self.conn_layer, 0)
                xcp_metal_d = self.add_wires(self.conn_layer, d_start_tidx, lower=xcp_metal_lower,
                                             upper=xcp_metal_upper, num=seg_n // 2, pitch=2)
                xcp_metal_s = self.add_wires(self.conn_layer, s_start_tidx, lower=xcp_metal_lower,
                                             upper=xcp_metal_upper, num=seg_n // 2 + 1, pitch=2)
                nd_conn.append(xcp_metal_d)
                d_num_tid_n = self.get_track_id(ridx_n - 1, MOSWireType.DS, 'ana_sig', 1, tile_idx=0)
                dum_s_n = self.connect_to_tracks(xcp_metal_s, d_num_tid_n)
                self.add_pin('in_c', dum_s_n, connect=True)
            if has_pmos:
                row_info, y0, orient = MOSBase.get_mos_row_info(pinfo, tile_yb, flip_tile, ridx_p + 1)
                if row_info.flip:
                    xcp_metal_upper, xcp_metal_lower = (y0 - row_info.ds_conn_y[0],
                                                        y0 - row_info.ds_conn_y[1])
                else:
                    xcp_metal_lower, xcp_metal_upper = (y0 + row_info.ds_conn_y[0], y0 + row_info.ds_conn_y[1])
                xcp_metal_lower, xcp_metal_upper = psam.s.lower, psam.d.upper
                d_start_tidx = self.arr_info.col_to_track(self.conn_layer, max_sam_col + 1)
                s_start_tidx = self.arr_info.col_to_track(self.conn_layer, max_sam_col + 2)
                xcp_metal_d = self.add_wires(self.conn_layer, d_start_tidx, lower=xcp_metal_lower,
                                             upper=xcp_metal_upper, num=seg_p // 2 + 1)
                xcp_metal_s = self.add_wires(self.conn_layer, s_start_tidx, lower=xcp_metal_lower,
                                             upper=xcp_metal_upper, num=seg_p // 2)
                pd_conn.append(xcp_metal_d)
                d_num_tid_p = self.get_track_id(ridx_p + 1, MOSWireType.DS, 'sig', 1, tile_idx=0)
                dum_s_p = self.connect_to_tracks(xcp_metal_s, d_num_tid_p)
                self.add_pin('in_c', dum_s_p, connect=True)

        if has_nmos:
            en_tid_n = self.get_track_id(ridx_n, MOSWireType.G_MATCH, 'clk', 0, tile_idx=1 if xcp_dummy else 0)
            d_tid_n = self.get_track_id(ridx_n, MOSWireType.DS, 'ana_sig', 0, tile_idx=1 if xcp_dummy else 0)
            s_tid_n = self.get_track_id(ridx_n, MOSWireType.DS, 'ana_sig', 1, tile_idx=1 if xcp_dummy else 0)
            clk_n = self.connect_to_tracks(nsam.g, en_tid_n)
            d_n = self.connect_to_tracks(nd_conn, d_tid_n)
            s_n = self.connect_to_tracks(ns_conn, s_tid_n)
            self.add_pin('sam', clk_n)
            self.add_pin('in', s_n)
            self.add_pin('out', d_n)
            self.add_pin('sam_conn', nsam.g, hide=True)
            self.add_pin('in_conn', ns_conn, hide=True)
            self.add_pin('out_conn', nd_conn, hide=True)
        if has_pmos:
            en_tid_p = self.get_track_id(ridx_p, MOSWireType.G_MATCH, 'clk', 0, tile_idx=1 if xcp_dummy else 0)
            d_tid_p = self.get_track_id(ridx_p, MOSWireType.DS, 'ana_sig', 0, tile_idx=1 if xcp_dummy else 0)
            s_tid_p = self.get_track_id(ridx_p, MOSWireType.DS, 'ana_sig', 1, tile_idx=1 if xcp_dummy else 0)
            clk_p = self.connect_to_tracks(psam.g, en_tid_p)
            d_p = self.connect_to_tracks(pd_conn, d_tid_p)
            s_p = self.connect_to_tracks(ps_conn, s_tid_p)
            self.add_pin('sam_b', clk_p)
            self.add_pin('in', s_p)
            self.add_pin('out', d_p)
            self.add_pin('sam_conn', psam.g, hide=True)
            self.add_pin('in_conn', ps_conn, hide=True)
            self.add_pin('out_conn', pd_conn, hide=True)

        self.set_mos_size()
        sampler_unit_params = dict(
            lch=self.arr_info.lch,
        )
        if has_pmos:
            sampler_unit_params['seg_p'] = seg_p
            sampler_unit_params['w_p'] = w_p
            sampler_unit_params['th_p'] = self.place_info.get_row_place_info(ridx_p).row_info.threshold
            if xcp_dummy:
                sampler_unit_params['seg_p_dum'] = seg_p
        if has_nmos:
            sampler_unit_params['seg_n'] = seg_n
            sampler_unit_params['w_n'] = w_n
            sampler_unit_params['th_n'] = self.place_info.get_row_place_info(ridx_n).row_info.threshold
            if xcp_dummy:
                sampler_unit_params['seg_n_dum'] = seg_n

        self._sch_params = sampler_unit_params


class Sampler(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            m_list='',
            nbits='',
            sampler_unit_params='',
            connect_xm_clk='Connect clock signal to xmlayer',
            nside='',
            tap_conn=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            m_list=[],
            nbits=1,
            connect_xm_clk=False,
            nside=True,
            tap_conn=SubPortMode.EVEN
        )

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'nmos_sampler')

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['sampler_unit_params']['pinfo'])
        self.draw_base(place_info)

        sampler_master = self.new_template(SamplerUnit, params=self.params['sampler_unit_params'])

        # by default use binary segments
        sampler_params_list = []
        nbits = self.params['nbits']
        if self.params['m_list']:
            m_list = self.params['m_list']
        else:
            m_list = [2 ** idx for idx in range(nbits)]

        tap_n_cols = self.get_tap_ncol()
        tap_sep_col = self.sub_sep_col
        tap_vdd_list, tap_vss_list = [], []
        tap_conn_mode = self.params['tap_conn']
        self.add_tap(tap_n_cols if tap_conn_mode == SubPortMode.EVEN else 0, tap_vdd_list, tap_vss_list,
                     tile_idx=0, flip_lr=True if tap_conn_mode == SubPortMode.EVEN else False)
        if self.params['sampler_unit_params']['xcp_dummy'] or self.params['sampler_unit_params']['xcp_dummy']:
            self.add_tap(tap_n_cols if tap_conn_mode == SubPortMode.EVEN else 0, tap_vdd_list, tap_vss_list,
                         tile_idx=1, flip_lr=True if tap_conn_mode == SubPortMode.EVEN else False)
        cur_col = tap_n_cols + tap_sep_col
        cur_col += cur_col & 1
        vm_col_l = cur_col
        sampler_list, sampler_list_list = [], []
        for s in m_list:
            sampler_sub_list = []
            for idx in range(s):
                sampler_sub_list.append(self.add_tile(sampler_master, 0, cur_col))
                cur_col += sampler_master.num_cols
            cur_col += self.min_sep_col
            sampler_list.extend(sampler_sub_list)
            sampler_list_list.append(sampler_sub_list)

        vm_col_r = self.num_cols
        tap_r = tap_n_cols + tap_sep_col + self.num_cols
        tap_r += tap_r & 1
        self.add_tap(tap_r - tap_n_cols if tap_conn_mode == SubPortMode.EVEN else tap_r, tap_vdd_list, tap_vss_list,
                     tile_idx=0, flip_lr=False if tap_conn_mode == SubPortMode.EVEN else True)
        if self.params['sampler_unit_params']['xcp_dummy'] or self.params['sampler_unit_params']['xcp_dummy']:
            self.add_tap(tap_r - tap_n_cols if tap_conn_mode == SubPortMode.EVEN else tap_r, tap_vdd_list, tap_vss_list,
                         tile_idx=1, flip_lr=False if tap_conn_mode == SubPortMode.EVEN else True)

        self.set_mos_size()

        in_warr = self.connect_wires([s.get_pin('in') for s in sampler_list])
        # self.add_pin('in', in_warr)

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        # export output

        for idx, s_list in enumerate(sampler_list_list):
            for s in s_list:
                out_vm_tid = self.grid.coord_to_track(vm_layer, s.get_pin('out').middle, mode=RoundMode.NEAREST)
                self.connect_to_tracks(s.get_pin('out'), TrackID(vm_layer, out_vm_tid, tr_w_sig_vm))

        # Collect vm tid list to export input
        tr_sp_sig_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        vm_tid_list = []
        cur_col = vm_col_l
        for m in m_list:
            _col_l = cur_col
            _col_r = cur_col + sampler_master.num_cols * m

            vm_tid_list.extend(self.get_available_tracks(vm_layer, self.arr_info.col_to_track(vm_layer, _col_l),
                                                         self.arr_info.col_to_track(vm_layer, _col_r),
                                                         self.bound_box.yl,
                                                         self.bound_box.yh, sep=tr_sp_sig_vm, sep_margin=tr_sp_sig_vm,
                                                         include_last=True))
            cur_col = _col_r + self.min_sep_col

        in_vm_list = [self.connect_to_tracks(in_warr, TrackID(vm_layer, _tid, tr_w_sig_vm))
                      for _tid in vm_tid_list]

        in_xm_tid = self.grid.coord_to_track(xm_layer, in_vm_list[0].middle, mode=RoundMode.NEAREST)
        in_vm_list_ret = []
        in_xm = self.connect_to_tracks(in_vm_list, TrackID(xm_layer, in_xm_tid, tr_w_sig_xm, grid=self.grid),
                                       ret_wire_list=in_vm_list_ret)
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        in_ym_list = []
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        for in_vm in in_vm_list:
            in_vm_coord = self.grid.track_to_coord(vm_layer, in_vm.track_id.base_index)
            in_ym_tidx = self.grid.coord_to_track(ym_layer, in_vm_coord, mode=RoundMode.NEAREST)
            in_ym_list.append(self.connect_to_tracks(in_xm, TrackID(ym_layer, in_ym_tidx, tr_w_sig_ym,
                                                                    grid=self.grid)))
        in_xm1_tid = self.grid.coord_to_track(xm1_layer, in_ym_list[0].upper, mode=RoundMode.NEAREST)
        in_xm1 = self.connect_to_tracks(in_ym_list, TrackID(xm1_layer, in_xm1_tid,
                                                            tr_manager.get_width(xm1_layer, 'sig'),
                                                            grid=self.grid))

        nside = self.params['nside']
        if sampler_list[0].has_port('in_c'):
            inc_warr = self.connect_wires([s.get_pin('in_c') for s in sampler_list])
            # self.add_pin('in_c', inc_warr)
            inc_vm_list = [self.connect_to_tracks(inc_warr, TrackID(vm_layer, _tid, tr_w_sig_vm))
                           for _tid in vm_tid_list]
            inc_xm_tid = self.grid.coord_to_track(xm_layer, inc_vm_list[0].middle, mode=RoundMode.NEAREST)
            inc_vm_list_ret = []
            inc_xm = self.connect_to_tracks(inc_vm_list, TrackID(xm_layer, inc_xm_tid, tr_w_sig_xm),
                                            ret_wire_list=inc_vm_list_ret)
            inc_ym_list = []
            for inc_vm in inc_vm_list:
                inc_vm_coord = self.grid.track_to_coord(vm_layer, inc_vm.track_id.base_index)
                inc_ym_tidx = self.grid.coord_to_track(ym_layer, inc_vm_coord, mode=RoundMode.NEAREST)
                inc_ym_list.append(self.connect_to_tracks(inc_xm, TrackID(ym_layer, inc_ym_tidx, tr_w_sig_vm)))
            inc_xm1_tid = self.grid.coord_to_track(xm1_layer, inc_ym_list[0].middle, mode=RoundMode.NEAREST)
            tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
            in_xm1_tid_coupled = tr_manager.get_next_track(xm1_layer, inc_xm1_tid, 'sig', 'sig', up=False)
            if nside:
                in_xm1_tid_coupled, inc_xm1_tid = inc_xm1_tid, in_xm1_tid_coupled

            inc_xm1 = self.connect_to_tracks(inc_ym_list, TrackID(xm1_layer, inc_xm1_tid, tr_w_sig_xm1))
            in_xm1_coupled = self.add_wires(xm1_layer, in_xm1_tid_coupled, width=tr_w_sig_xm1,
                                            lower=inc_xm1.lower, upper=inc_xm1.upper)

            out_vm_lower = inc_vm_list_ret[0].lower
            self.add_pin('in_coupled', in_xm1_coupled, hide=True)
            self.add_pin('in_c', inc_xm1)
        else:
            out_vm_lower = in_vm_list_ret[0].lower

        out_vm_upper = in_vm_list_ret[0].upper
        for idx, s_list in enumerate(sampler_list_list):
            _out_vm_list = []
            for s in s_list:
                out_vm_tid = self.grid.coord_to_track(vm_layer, s.get_pin('out').middle, mode=RoundMode.NEAREST)
                _out_vm_list.append(self.connect_to_tracks(s.get_pin('out'),
                                                           TrackID(vm_layer, out_vm_tid, tr_w_sig_vm),
                                                           track_upper=out_vm_upper, track_lower=out_vm_lower))
            self.connect_wires([s.get_pin('out') for s in s_list])
            self.add_pin(f'out<{idx}>' if len(m_list) > 1 else 'out', _out_vm_list)
        self.add_pin('in', in_xm1)

        connect_xm_clk = self.params['connect_xm_clk']
        if sampler_list[0].has_port('sam'):
            clk = self.connect_wires([s.get_pin('sam') for s in sampler_list])
            self.add_pin('sam', clk)
            if connect_xm_clk:
                clk_vm_list = [self.connect_to_tracks(clk, TrackID(vm_layer, _tid, tr_w_sig_vm))
                               for _tid in vm_tid_list]
                clk_xm_tid = self.grid.coord_to_track(xm_layer, clk_vm_list[0].middle, mode=RoundMode.NEAREST)
                clk_xm = self.connect_to_tracks(clk_vm_list, TrackID(xm_layer, clk_xm_tid, tr_w_sig_xm))
                self.add_pin('sam', clk_xm)
        if sampler_list[0].has_port('sam_b'):
            clkb = self.connect_wires([s.get_pin('sam_b') for s in sampler_list])
            self.add_pin('sam_b', clkb)
            if connect_xm_clk:
                clkb_vm_list = [self.connect_to_tracks(clkb, TrackID(vm_layer, _tid, tr_w_sig_vm))
                                for _tid in vm_tid_list]
                clkb_xm_tid = self.grid.coord_to_track(xm_layer, clkb_vm_list[0].middle, mode=RoundMode.NEAREST)
                clkb_xm = self.connect_to_tracks(clkb_vm_list, TrackID(xm_layer, clkb_xm_tid, tr_w_sig_xm))
                self.add_pin('sam_b', clkb_xm)

        if sampler_list[0].has_port('VDD'):
            self.connect_to_track_wires(tap_vdd_list, [s.get_pin('VDD') for s in sampler_list])
            self.add_pin('VDD', [s.get_pin('VDD') for s in sampler_list])
        else:
            self.add_pin('VDD', tap_vdd_list)
        if sampler_list[0].has_port('VSS'):
            self.connect_to_track_wires(tap_vss_list, [s.get_pin('VSS') for s in sampler_list])
            self.add_pin('VSS', [s.get_pin('VSS') for s in sampler_list])
        else:
            self.add_pin('VSS', tap_vss_list)

        for s in sampler_list:
            self.reexport(s.get_port('sam_conn'))
            self.reexport(s.get_port('in_conn'))
            self.reexport(s.get_port('out_conn'))

        self._sch_params = dict(
            m_list=m_list,
            **sampler_master.sch_params
        )


class SamplerVertCore(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'nmos_sampler')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            min_height='Height to match capdac',
            pinfo='placement information object.',
            seg='segments dictionary.',
            sp_tap='tap and transistor separation',
            sp='dummy seperation',
            w='widths.',
            sw_type='Type of switch',
            top_layer='Top supply and vin layer',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w=4,
            sp=2,
            sp_tap=1,
            min_height=0,
            sw_type='nch',
        )

    def draw_layout(self):
        min_height: int = self.params['min_height']
        sw_type: str = self.params['sw_type']
        w: int = self.params['w']
        ny = 1
        pinfo_dict = self.params['pinfo'].to_yaml()

        if min_height > 0:
            pinfo_dict['tile_specs']['place_info']['drv_tile']['min_height'] = min_height
        pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['mos_type'] = sw_type
        # pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['width'] = w
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        pinfo0 = [TilePatternElement(pinfo[1]['drv_tile'])] * ny
        self.draw_base((TilePattern(pinfo0), pinfo[1]))

        seg: int = self.params['seg']
        sp_tap: int = self.params['sp_tap']
        sp: int = self.params['sp']
        w: int = self.params['w']

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        tr_vg_hm_w = tr_manager.get_width(hm_layer, 'vg')
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_vin_vm_w = tr_manager.get_width(vm_layer, 'ana_sig')
        tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
        tr_vin_xm_w = tr_manager.get_width(xm_layer, 'ana_sig')
        tr_sup_ym_w = tr_manager.get_width(ym_layer, 'sup')
        tr_vg_ym_w = tr_manager.get_width(ym_layer, 'vg')
        tr_vin_ym_w = tr_manager.get_width(ym_layer, 'ana_sig')

        pin_lower = self.arr_info.col_to_coord(0)
        sup_bot_tid_list = []
        xm_tid_list = []
        tap_ncol = self.get_tap_ncol(tile_idx=0, seg=2)
        tap_sep_col = self.sub_sep_col
        tap_ncol += tap_sep_col
        tile_height = self.get_tile_info(0)[0].height

        side_ncol = tap_ncol + sp_tap
        sw_col = side_ncol
        tr_w_sig_ow = tr_manager.get_width(hm_layer, 'sig_ow')
        tid_bot = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=0)
        tid_ref = TrackID(hm_layer, tr_manager.get_next_track(hm_layer, tid_bot.base_index, 'sig', 'sig_ow', up=-1),
                          tr_w_sig_ow)
        # tid_ref = self.get_track_id(0, MOSWireType.DS, wire_name='ref', wire_idx=0, tile_idx=0)

        sw_list, ctrl_list, vin_list = [], [], []
        tile_info, yb, _ = self.get_tile_info(0)
        for jdx in range(2):
            sw_list.append(self.add_mos(0, sw_col, seg, w=w, tile_idx=0))
            sw_col += seg + sp
            vin_list.append(self.connect_to_tracks(sw_list[-1].d, tid_ref, min_len_mode=MinLenMode.MIDDLE))

        self.set_mos_size(num_cols=2 * side_ncol + 2 * seg + sp)

        vg_tid = self.get_track_id(0, MOSWireType.G, wire_name='sup', wire_idx=1, tile_idx=0)

        vg_locs = self.get_tids_between(hm_layer, vg_tid.base_index,
                                        self.grid.coord_to_track(hm_layer, yb + tile_info.height,
                                                                 RoundMode.NEAREST),
                                        tr_vg_hm_w, self.get_track_sep(hm_layer, tr_vg_hm_w, tr_vg_hm_w), 1, True)

        vg = WireArray.list_to_warr([self.connect_to_tracks(sw_list[0].g, vg_tid) for vg_tid in vg_locs])
        vg_dummy = WireArray.list_to_warr([self.connect_to_tracks(sw_list[-1].g, vg_tid) for vg_tid in vg_locs])
        cap_bot = self.connect_to_tracks([sw.s for sw in sw_list], tid_bot,
                                         min_len_mode=MinLenMode.MIDDLE)

        sup_hm_list = []
        vdd_list, vss_list = [], []
        self.add_tap(sw_col - sp + side_ncol, vdd_list, vss_list, seg=2, tile_idx=0, flip_lr=True)

        for tid in [tid_ref]:
            sup_conn_list = vdd_list if sw_type == 'pch' else vss_list
            sup_hm_list.append(self.connect_to_tracks(sup_conn_list, tid))
        self.add_pin('VSS_hm', sup_hm_list, label='VSS')
        # self.connect_to_track_wires(sw_list[-1].g, sup_hm_list[2])

        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        sup_vm_locs = self.get_tids_between(vm_layer,
                                            self.arr_info.col_to_track(vm_layer, self.num_cols - tap_ncol + 1),
                                            self.arr_info.col_to_track(vm_layer, self.num_cols + 1),
                                            tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_sup_vm_w),
                                            0, include_last=False, align_to_higher=True)

        vg_dummy = [self.connect_to_tracks(vg_dummy, tid, track_upper=self.bound_box.yh, track_lower=self.bound_box.yl)
                    for tid in sup_vm_locs]
        # sup_vm_list = []
        # for tid in sup_vm_locs:
        #     sup_vm_list.append(self.connect_to_tracks(sup_hm_list, tid))

        vin_vm_list = []
        for idx in range(2):
            tx = sw_list[idx]
            vin_vm_tid = TrackID(vm_layer, tx.d.track_id.base_index, pitch=tx.s.track_id.pitch,
                                 num=tx.d.track_id.num, width=tr_manager.get_width(vm_layer, 'vg'), grid=self.grid)
            vin_vm_list.append(self.connect_to_tracks(vin_list[idx], vin_vm_tid, min_len_mode=MinLenMode.UPPER,
                                                      track_lower=yb))

        sup_vm_locs = self.get_tids_between(vm_layer,
                                            self.arr_info.col_to_track(vm_layer, -1),
                                            self.arr_info.col_to_track(vm_layer, tap_ncol - 1),
                                            tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_sup_vm_w),
                                            0, include_last=False, align_to_higher=False)
        vg_vm = [self.connect_to_tracks(vg, tid, track_upper=self.bound_box.yh, track_lower=self.bound_box.yl) for
                 tid in sup_vm_locs]

        cap_bot_vm_tid = []
        for idx in range(2):
            tx = sw_list[idx]
            cap_bot_vm_tid.append(TrackID(vm_layer, tx.s.track_id.base_index, pitch=tx.s.track_id.pitch,
                                          num=tx.s.track_id.num, width=tr_manager.get_width(vm_layer, 'ana_sig'),
                                          grid=self.grid))
        cap_bot_vm = [self.connect_to_tracks(cap_bot, tid, min_len_mode=MinLenMode.LOWER) for tid in cap_bot_vm_tid]

        fill_conn_layer_intv(self, 0, 0)
        tile_info, yb, _ = self.get_tile_info(0)
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_sup_sig_xm = self.get_track_sep(xm_layer, tr_w_sup_xm, tr_w_sig_xm)
        tr_sp_dum_sig_xm = self.get_track_sep(xm_layer, 1, tr_w_sig_xm)
        xm_sup_locs = self.grid.coord_to_track(xm_layer, self.grid.track_to_coord(hm_layer, tid_ref.base_index),
                                               RoundMode.LESS)
        xm_bot_locs = self.get_tids_between(xm_layer, xm_sup_locs + tr_sp_sup_sig_xm,
                                            self.grid.coord_to_track(xm_layer, yb + tile_info.height,
                                                                     RoundMode.NEAREST) - 1,
                                            tr_w_sig_xm, 0, 0, False, center=False)
        xm_bot_locs = [xm_bot_locs[0]]
        # xm_bot_locs = [xm_bot_locs[len(xm_bot_locs)//2]]
        # cap_bot_xm = WireArray.list_to_warr([self.connect_to_tracks(cap_bot_vm, tid) for tid in xm_bot_locs])
        #
        vin_xm_list = [[] for _ in range(len(vin_vm_list))]
        for idx, vref in enumerate(vin_vm_list):
            _vin_xm = self.connect_to_tracks(vref, TrackID(xm_layer, xm_sup_locs, tr_vin_xm_w, grid=self.grid),
                                             min_len_mode=MinLenMode.MIDDLE)
            vin_xm_list[idx].append(_vin_xm)
            self.add_pin(f'vref{idx}_xm', vref)

        vg_dummy_xm = self.connect_to_tracks(vg_dummy, TrackID(xm_layer, xm_sup_locs, tr_sup_xm_w, grid=self.grid))
        vg_xm = self.connect_to_tracks(vg_vm, TrackID(xm_layer, xm_sup_locs, tr_sup_xm_w, grid=self.grid))

        ###########
        # ym connection
        ###########

        vin_top_list_list = []
        top_layer = self.params['top_layer']
        for vin_xm in vin_xm_list:
            vin_tid_list = []
            vin_bnd_list = []
            for idx in range(xm_layer + 1, top_layer + 1):
                lay_dir = self.grid.get_direction(idx)
                _tr_w = tr_manager.get_width(idx, 'ana_sig')
                if lay_dir == Orient2D.y:
                    vin_locs = \
                        self.get_tids_between(idx, self.grid.coord_to_track(idx, vin_xm[0].lower, RoundMode.LESS),
                                              self.grid.coord_to_track(idx, vin_xm[0].upper, RoundMode.GREATER),
                                              _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                              0, include_last=True)

                    vin_tid_list.append(vin_locs)
                    vin_bnd_list.append(None)
                    # vin_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
                else:
                    ref_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                    vin_tid_list.append([TrackID(idx, ref_loc, _tr_w, grid=self.grid)])
                    vin_bnd_list.append(None)

            vin_top_list_list.append(self.connect_warr_to_tids_stack(vin_xm, vin_tid_list, vin_bnd_list,
                                                                     MinLenMode.MIDDLE)[-1])

        self.add_pin('vin', vin_top_list_list[0])
        self.add_pin('vin_c', vin_top_list_list[1])

        # Supply
        sup_tid_list = []
        sup_bnd_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            _tr_w = tr_manager.get_width(idx, 'sup')
            if lay_dir == Orient2D.y:
                sup_locs = self.get_tids_between(idx,
                                                 self.arr_info.col_to_track(idx, self.num_cols - tap_ncol + 1),
                                                 self.arr_info.col_to_track(idx, self.num_cols + 1),
                                                 _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                                 0, include_last=True, align_to_higher=True)
                sup_tid_list.append(sup_locs)
                # sup_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
                sup_bnd_list.append(None)

            else:
                sup_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                sup_tid_list.append([TrackID(idx, sup_loc, _tr_w, grid=self.grid)])
                sup_bnd_list.append(None)

        sup_warr_list = self.connect_warr_to_tids_stack(vg_dummy_xm, sup_tid_list, sup_bnd_list, MinLenMode.MIDDLE)[-1]

        # VG
        vg_tid_list = []
        vg_bnd_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            _tr_w = tr_manager.get_width(idx, 'vg')
            if lay_dir == Orient2D.y:
                vg_locs = self.get_tids_between(idx,
                                                self.arr_info.col_to_track(idx, -1),
                                                self.arr_info.col_to_track(idx, tap_ncol - 1),
                                                _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                                0, include_last=True, align_to_higher=True)
                vg_tid_list.append(vg_locs)
                # vg_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
                vg_bnd_list.append(None)

            else:
                vg_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                vg_tid_list.append([TrackID(idx, vg_loc, _tr_w, grid=self.grid)])
                vg_bnd_list.append(None)

        vg_warr_list = self.connect_warr_to_tids_stack(vg_xm, vg_tid_list, vg_bnd_list, MinLenMode.MIDDLE)[-1]

        self.add_pin('vg', vg_warr_list)

        if vdd_list:
            self.add_pin('VDD', sup_warr_list)
        if vss_list:
            self.add_pin('VSS', sup_warr_list)
        self.add_pin('out', cap_bot_vm)
        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_n=seg,
            seg_n_dum=seg,
            w_n=w,
            m_list=[1] * ny,
            th_n=self.get_row_info(0, 0).threshold
        )


class SamplerVertCol(SamplerVertCore):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'nmos_sampler')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = SamplerVertCore.get_params_info()
        ans['ny'] = 'number of rows'
        ans['compact'] = 'True to put sampler in a stdcell way'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = SamplerVertCore.get_default_param_values()
        ans['ny'] = 1
        ans['compact'] = False
        return ans

    def draw_layout(self):
        sw_type: str = self.params['sw_type']
        min_height: int = self.params['min_height']
        seg: int = self.params['seg']
        ny: int = self.params['ny']
        w: int = self.params['w']
        pinfo_dict = self.params['pinfo'].to_yaml()

        compact = self.params['compact']

        if min_height > 0:
            pinfo_dict['tile_specs']['place_info']['drv_tile']['min_height'] = min_height
        pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['mos_type'] = sw_type
        # pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['width'] = w
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        pinfo0 = [TilePatternElement(pinfo[1]['drv_tile'], flip=bool(idx & 1) if compact else False) for idx in
                  range(ny)]
        self.draw_base((TilePattern(pinfo0), pinfo[1]))

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        sw_unit = self.new_template(SamplerVertCore, params=self.params)
        sw_list = []
        for idx in range(ny):
            sw_list.append(self.add_tile(sw_unit, tile_idx=idx, col_idx=0))

        self.set_mos_size()

        top_layer = self.params['top_layer']

        vg_list, vin_list, vin_c_list = [], [], []
        [vg_list.extend(inst.get_all_port_pins('vg')) for inst in sw_list]
        [vin_list.extend(inst.get_all_port_pins('vin')) for inst in sw_list]
        [vin_c_list.extend(inst.get_all_port_pins('vin_c')) for inst in sw_list]
        self.add_pin('vg', self.connect_wires(vg_list))
        self.add_pin('vin', self.connect_wires(vin_list))
        self.add_pin('vin_c', self.connect_wires(vin_c_list))

        vss_list, vss_hm_list = [], []
        [vss_list.extend(inst.get_all_port_pins('VSS')) for inst in sw_list]
        [vss_hm_list.extend(inst.get_all_port_pins('VSS_hm')) for inst in sw_list]

        self.add_pin('VSS', self.connect_wires(vss_list))
        self.add_pin('VSS_hm', self.connect_wires(vss_hm_list))

        [self.add_pin(f'out<{idx}>', inst.get_all_port_pins('out'), mode=PinMode.UPPER) for idx, inst in
         enumerate(sw_list)]
        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_n=seg,
            seg_n_dum=seg,
            w_n=w,
            m_list=[1] * ny,
            th_n=self.get_row_info(0, 0).threshold
        )


class SamplerVertGR(SamplerVertCol):
    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        master: MOSBase = self.new_template(SamplerVertCol, params=self.params)
        tinfo_table = master.tile_table

        # construct TilePattern object, and call draw_base()
        bot_pinfo = tinfo_table['ngr']
        tile_list = [TilePatternElement(bot_pinfo), master.get_tile_pattern_element(),
                     TilePatternElement(bot_pinfo, flip=True)]
        self.draw_base((TilePattern(tile_list), tinfo_table))

        tech_cls = self.tech_cls
        edge_ncol = tech_cls.gr_edge_col
        sep_l = 2 * tech_cls.sub_sep_col
        sep_r = 2 * tech_cls.sub_sep_col
        ncol = master.num_cols + 2 * edge_ncol + sep_l + sep_r
        ntile = master.num_tile_rows + 2
        inst = self.add_tile(master, 1, edge_ncol + sep_l)
        self.set_mos_size(num_cols=ncol, num_tiles=ntile)

        sup_list = []
        vdd_vm_list = []
        vss_vm_list = []
        vdd_hm_keys = []
        vss_hm_keys = []
        vdd_hm_dict = {}
        vss_hm_dict = {}
        grid = self.grid
        hm_layer = self.conn_layer + 1
        pmos_sub_type = tinfo_table['pgr'].get_row_place_info(0).row_info.row_type
        nmos_sub_type = tinfo_table['ngr'].get_row_place_info(0).row_info.row_type
        for tile_idx in range(ntile):
            cur_pinfo = self.get_tile_pinfo(tile_idx)
            vdd_hm_list = []
            vss_hm_list = []
            for ridx in range(cur_pinfo.num_rows):
                row_info = cur_pinfo.get_row_place_info(ridx).row_info
                row_type = row_info.row_type
                if row_type.is_substrate and row_info.guard_ring:
                    tid = self.get_track_id(ridx, MOSWireType.DS, 'guard', tile_idx=tile_idx)
                    sub = self.add_substrate_contact(ridx, 0, tile_idx=tile_idx, seg=ncol)
                    warr = self.connect_to_tracks(sub, tid)
                    coord = grid.track_to_coord(hm_layer, tid.base_index)
                    if row_type.is_pwell:
                        vss_hm_list.append(warr)
                        vss_hm_keys.append(coord)
                        vss_hm_dict[coord] = warr
                    else:
                        vdd_hm_list.append(warr)
                        vdd_hm_keys.append(coord)
                        vdd_hm_dict[coord] = warr
                else:
                    if row_type.is_substrate:
                        sub_type = pmos_sub_type if row_type.is_n_plus else nmos_sub_type
                    else:
                        sub_type = nmos_sub_type if row_type.is_n_plus else pmos_sub_type
                    sub0 = self.add_substrate_contact(ridx, 0, tile_idx=tile_idx, seg=edge_ncol,
                                                      guard_ring=True, sub_type=sub_type)
                    sub1 = self.add_substrate_contact(ridx, ncol, tile_idx=tile_idx, seg=edge_ncol,
                                                      guard_ring=True, flip_lr=True,
                                                      sub_type=sub_type)
                    if sub_type.is_pwell:
                        vss_vm_list.append(sub0)
                        vss_vm_list.append(sub1)
                    else:
                        vdd_vm_list.append(sub0)
                        vdd_vm_list.append(sub1)

            sup_list.append((vss_hm_list, vdd_hm_list))

        gr_hm = [it[1] for it in vdd_hm_dict.items()]
        conn_layer = master.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        # Connect VDD at left side
        tr_w_sup_vm = master.tr_manager.get_width(vm_layer, 'sup')
        sup_locs = self.get_tids_between(vm_layer,
                                         self.arr_info.col_to_track(vm_layer, -1),
                                         self.arr_info.col_to_track(vm_layer, edge_ncol + 1),
                                         tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_w_sup_vm),
                                         0, include_last=True, align_to_higher=True)
        gr_vm = [self.connect_to_tracks(gr_hm, tid) for tid in sup_locs]
        gr_xm_list = []
        for hmwarr in gr_hm:
            xm_tid = self.grid.coord_to_track(xm_layer, (hmwarr.bound_box.yh + hmwarr.bound_box.yl) // 2,
                                              RoundMode.NEAREST)
            gr_xm_list.append(
                self.connect_to_tracks(gr_vm, TrackID(xm_layer, xm_tid, master.tr_manager.get_width(xm_layer, 'sup'),
                                                      grid=self.grid)))

        self.connect_to_track_wires(vdd_vm_list, [it[1] for it in vdd_hm_dict.items()])

        sup_tid_list = []
        sup_bnd_list = []
        top_layer = self.params['top_layer']
        tr_manager = master.tr_manager
        for idx in range(xm_layer + 1, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            _tr_w = tr_manager.get_width(idx, 'sup')
            if lay_dir == Orient2D.y:
                sup_locs = self.get_tids_between(idx,
                                                 self.arr_info.col_to_track(idx, -1),
                                                 self.arr_info.col_to_track(idx, edge_ncol + 1),
                                                 _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                                 0, include_last=True, align_to_higher=True)
                sup_tid_list.append(sup_locs)
                sup_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
            else:
                sup_loc = [self.grid.coord_to_track(idx, (gr_hm[0].bound_box.yh + gr_hm[0].bound_box.yl) // 2,
                                                    RoundMode.NEAREST),
                           self.grid.coord_to_track(idx, (gr_hm[1].bound_box.yh + gr_hm[1].bound_box.yl) // 2,
                                                    RoundMode.NEAREST)]
                sup_tid_list.append([TrackID(idx, loc, _tr_w, grid=self.grid) for loc in sup_loc])
                sup_bnd_list.append(None)

        sup_warr_list = self.connect_warr_to_tids_stack(gr_xm_list, sup_tid_list, sup_bnd_list, MinLenMode.MIDDLE)[-1]
        self.add_pin('VDD', sup_warr_list)

        # Connect VSS at left side
        tr_w_sup_vm = master.tr_manager.get_width(vm_layer, 'sup')
        sup_locs = self.get_tids_between(vm_layer,
                                         self.arr_info.col_to_track(vm_layer, self.num_cols - edge_ncol - 1),
                                         self.arr_info.col_to_track(vm_layer, self.num_cols + 1),
                                         tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_w_sup_vm),
                                         0, include_last=True, align_to_higher=True)
        vss_vm = [self.connect_to_tracks(inst.get_all_port_pins('VSS_hm'), tid) for tid in sup_locs]
        vss_xm_list = []
        # for hmwarr in inst.get_all_port_pins('VSS_hm')[0].to_warr_list():
        for hmwarr in gr_xm_list:
            xm_tid = self.grid.coord_to_track(xm_layer, (hmwarr.bound_box.yh + hmwarr.bound_box.yl) // 2,
                                              RoundMode.NEAREST)
            vss_xm_list.append(
                self.connect_to_tracks(vss_vm, TrackID(xm_layer, xm_tid, master.tr_manager.get_width(xm_layer, 'sup'),
                                                       grid=self.grid)))

        self.connect_to_track_wires(vss_vm_list, [it[1] for it in vdd_hm_dict.items()])

        sup_tid_list = []
        sup_bnd_list = []
        top_layer = self.params['top_layer']
        tr_manager = master.tr_manager
        for idx in range(xm_layer + 1, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            _tr_w = tr_manager.get_width(idx, 'sup')
            if lay_dir == Orient2D.y:
                sup_locs = self.get_tids_between(idx,
                                                 self.arr_info.col_to_track(idx, self.num_cols - edge_ncol - 1),
                                                 self.arr_info.col_to_track(idx, self.num_cols + 1),
                                                 _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                                 0, include_last=True, align_to_higher=True)
                sup_tid_list.append(sup_locs)
                sup_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
            else:
                sup_loc = [self.grid.coord_to_track(idx, (gr_hm[0].bound_box.yh + gr_hm[0].bound_box.yl) // 2,
                                                    RoundMode.NEAREST),
                           self.grid.coord_to_track(idx, (gr_hm[1].bound_box.yh + gr_hm[1].bound_box.yl) // 2,
                                                    RoundMode.NEAREST)]
                sup_tid_list.append([TrackID(idx, loc, _tr_w, grid=self.grid) for loc in sup_loc])
                sup_bnd_list.append(None)

        sup_warr_list = self.connect_warr_to_tids_stack(vss_xm_list, sup_tid_list, sup_bnd_list, MinLenMode.MIDDLE)[-1]
        self.add_pin('VSS', sup_warr_list)
        self.add_pin('VSS_vm', vss_vm)

        for name in inst.port_names_iter():
            if 'VSS' not in name:
                self.reexport(inst.get_port(name))
        self.reexport(inst.get_port('VSS'), net_name='off')
        self.sch_params = master.sch_params


class CMSwitch(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            segp='segments.',
            segn='segments.',
            wp='widths.',
            wn='widths.',
            ncols_tot='Total number of fingersa',
            swap_inout=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            wp=4,
            wn=4,
            ncols_tot=0,
            swap_inout=False,
        )

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        tr_manager = self.tr_manager

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        segn: int = self.params['segn']
        segp: int = self.params['segp']
        wn: int = self.params['wn']
        wp: int = self.params['wp']

        tap_ncol = self.get_tap_ncol(tile_idx=0)
        tap_sep_col = self.sub_sep_col
        tap_sep_col += tap_sep_col & 1
        tap_ncol += tap_sep_col

        tot_cols = max(self.params['ncols_tot'], max(segn, segp) + 2 * tap_ncol + 2) 
        seg_sub_conn = (tot_cols - max(segn, segp) - 2) // 2
        # Placement
        ntap_l = self.add_substrate_contact(0, 0, seg=seg_sub_conn - tap_sep_col, w=wn)
        ptap_l = self.add_substrate_contact(1, 0, seg=seg_sub_conn - tap_sep_col, w=wp)

        sw_n = self.add_mos(0, (tot_cols - segn) // 2, segn, w=wn)
        sw_p = self.add_mos(1, (tot_cols - segp) // 2, segp, w=wp)

        ntap_r = self.add_substrate_contact(0, tot_cols, seg=seg_sub_conn - tap_sep_col, w=wn, flip_lr=True)
        ptap_r = self.add_substrate_contact(1, tot_cols, seg=seg_sub_conn - tap_sep_col, w=wp, flip_lr=True)
        vdd_list, vss_list = [ptap_l, ptap_r], [ntap_l, ntap_r]
        self.set_mos_size()

        ntid_g = self.get_track_id(0, MOSWireType.G, wire_name='clk', wire_idx=0)
        ntid_sig = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=0)

        ptid_g = self.get_track_id(1, MOSWireType.G, wire_name='clk', wire_idx=0)
        ptid_sig = self.get_track_id(1, MOSWireType.DS, wire_name='sig', wire_idx=0)
        ntid_ref = ptid_ref = self.grid.get_middle_track(ntid_sig.base_index, ptid_sig.base_index)

        sam_hm = self.connect_to_tracks(sw_n.g, ntid_g)
        sam_b_hm = self.connect_to_tracks(sw_p.g, ptid_g)
        ref_hm = self.connect_to_tracks([sw_n.d, sw_p.d],
                                        TrackID(hm_layer, ntid_ref, tr_manager.get_width(hm_layer, 'sig')))
        n_sig_hm = self.connect_to_tracks(sw_n.s, ntid_sig)
        p_sig_hm = self.connect_to_tracks(sw_p.s, ptid_sig)

        # get middle track for sample signal
        mid_vm_tidx = self.arr_info.col_to_track(vm_layer, tot_cols // 2, RoundMode.NEAREST)
        sam_vm = self.connect_to_tracks(sam_hm, TrackID(vm_layer, mid_vm_tidx, tr_manager.get_width(vm_layer, 'clk')))
        sam_b_vm = self.connect_to_tracks(sam_b_hm,
                                          TrackID(vm_layer, mid_vm_tidx, tr_manager.get_width(vm_layer, 'clk')))
        tid_l = self.arr_info.col_to_track(vm_layer, tap_ncol, mode=RoundMode.NEAREST)
        tid_r = self.arr_info.col_to_track(vm_layer, self.num_cols - tap_ncol, mode=RoundMode.NEAREST)

        tr_w_vref_vm = tr_manager.get_width(vm_layer, 'vref')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        vref_vm_locs = self.get_tids_between(vm_layer, tid_l,
                                             mid_vm_tidx - self.get_track_sep(vm_layer, tr_w_vref_vm, tr_w_clk_vm),
                                             tr_w_vref_vm, 0, 0, True)
        sig_vm_locs = self.get_tids_between(vm_layer,
                                            mid_vm_tidx + self.get_track_sep(vm_layer, tr_w_clk_vm, tr_w_sig_vm),
                                            tid_r, tr_manager.get_width(vm_layer, 'sig'), 0, 0, True)
        vref_vm = [self.connect_to_tracks(ref_hm, _tid) for _tid in vref_vm_locs]
        nsig_vm = [self.connect_to_tracks(n_sig_hm, _tid) for _tid in sig_vm_locs]
        psig_vm = [self.connect_to_tracks(p_sig_hm, _tid)
                   for _tid in sig_vm_locs]
        vm_warrs = vref_vm + nsig_vm + psig_vm
        vm_warrs_max_coord, vm_warrs_min_coord = max([v.upper for v in vm_warrs]), min([v.lower for v in vm_warrs])
        vref_vm = self.extend_wires(vref_vm, upper=vm_warrs_max_coord, lower=vm_warrs_min_coord)
        sig_vm = self.extend_wires(nsig_vm + psig_vm, upper=vm_warrs_max_coord, lower=vm_warrs_min_coord)

        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')

        # Connect supplies
        # tr_sup_hm_w = tr_manager.get_width(hm_layer, 'sup')
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
        vss_hm = self.connect_to_tracks(vss_list, self.get_track_id(0, MOSWireType.G, wire_name='sup'))
        vdd_hm = self.connect_to_tracks(vdd_list, self.get_track_id(1, MOSWireType.G, wire_name='sup'))

        vdd_vm_list, vss_vm_list = [], []
        sup_vm_locs = self.get_tids_between(vm_layer, self.arr_info.col_to_track(vm_layer, 0),
                                            self.arr_info.col_to_track(vm_layer, tap_ncol),
                                            tr_sup_vm_w, 0, 0, True)
        sup_vm_locs += self.get_tids_between(vm_layer, self.arr_info.col_to_track(vm_layer, tot_cols - seg_sub_conn),
                                             self.arr_info.col_to_track(vm_layer, tot_cols),
                                             tr_sup_vm_w, 0, 0, True)
        for tid in sup_vm_locs:
            vss_vm_list.append(self.connect_to_tracks(vss_hm, tid))
            vdd_vm_list.append(self.connect_to_tracks(vdd_hm, tid))

        swap_inout = self.params['swap_inout']
        _, sig_xm_locs = tr_manager.place_wires(xm_layer, ['sig', 'sig'],
                                                center_coord=self.grid.track_to_coord(hm_layer, ptid_ref))
        vin_xm = self.connect_to_tracks(vref_vm, TrackID(xm_layer, sig_xm_locs[0] if swap_inout else sig_xm_locs[1],
                                                         tr_w_sig_xm))
        sig_xm = self.connect_to_tracks(sig_vm, TrackID(xm_layer, sig_xm_locs[1] if swap_inout else sig_xm_locs[0],
                                                        tr_w_sig_xm))

        vdd_xm_tidx = self.grid.coord_to_track(xm_layer, self.grid.track_to_coord(hm_layer, vdd_hm.track_id.base_index),
                                               RoundMode.NEAREST)
        vss_xm_tidx = self.grid.coord_to_track(xm_layer, self.grid.track_to_coord(hm_layer, vss_hm.track_id.base_index),
                                               RoundMode.NEAREST)
        vdd_xm = self.connect_to_tracks(vdd_vm_list, TrackID(xm_layer, vdd_xm_tidx, tr_w_sup_xm))
        vss_xm = self.connect_to_tracks(vss_vm_list, TrackID(xm_layer, vss_xm_tidx, tr_w_sup_xm))

        xm_avail_locs = self.get_available_tracks(xm_layer, vss_xm_tidx, vdd_xm_tidx,
                                                  self.bound_box.xl, self.bound_box.xh,
                                                  tr_manager.get_width(xm_layer, 'clk'),
                                                  tr_manager.get_sep(xm_layer, ('clk', 'clk')),
                                                  sep_margin=tr_manager.get_sep(xm_layer, ('clk', 'sup')),
                                                  include_last=True)

        sam_xm = self.connect_to_tracks(sam_vm, TrackID(xm_layer, xm_avail_locs[0], tr_w_clk_xm))
        sam_b_xm = self.connect_to_tracks(sam_b_vm, TrackID(xm_layer, xm_avail_locs[-1], tr_w_clk_xm))

        self.add_pin('sam', sam_xm)
        self.add_pin('sam_b', sam_b_xm)
        self.add_pin('ref', vin_xm)
        self.add_pin('sig', sig_xm)
        self.add_pin('VSS', vss_xm)
        self.add_pin('VDD', vdd_xm)

        self.sch_params = dict(
            n=dict(
                lch=self.arr_info.lch,
                seg=segn,
                w=wn,
                intent=self.place_info.get_row_place_info(0).row_info.threshold
            ),
            p=dict(
                lch=self.arr_info.lch,
                seg=segn,
                w=wn,
                intent=self.place_info.get_row_place_info(0).row_info.threshold
            )
        )
