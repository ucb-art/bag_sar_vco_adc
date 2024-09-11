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

import copy
from bisect import bisect_left
from typing import Any, Dict, Type, Optional, List, Mapping, Union

from bag.design.database import ModuleDB, Module
from bag.layout.routing.base import TrackManager, WireArray
from bag.layout.routing.base import WDictType, SpDictType
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from bag_vco_adc.layout.ra.ra_cap import MOMCapOnMOS
from bag_vco_adc.layout.sah.sar_samp import Sampler, SamplerVertCol
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from bag_vco_adc.layout.util.wrapper import GenericWrapper
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orientation, MinLenMode, Orient2D
from xbase.layout.cap.core import MOMCapCore
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase, MOSArrayPlaceInfo
from xbase.layout.mos.placement.data import TilePatternElement, TilePattern


class BootstrapNMOS(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._dum_sampler = False

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            unit_sampler_params='Unit sampler parameters',
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            ridx_samp='index for nmos row with sampler',
            ridx_off0='index for nmos row with turn-off transistor 0',
            ridx_off1='index for nmos row with turn-off transistor 1',
            dum_sampler='True to enable dummy sampler',
            fast_on='True to fast turn-on XON_N',
            nside='Negative side of sampler, to xcp input',
            swap_inout='Change input and output for samplers',
            no_sampler='True to exclude sampler, export vg and vin signals',
            dummy_off='For signal sampler, need to match gate resistance of dummy',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            ridx_samp=0,
            ridx_off0=0,
            ridx_off1=1,
            dum_sampler=True,
            fast_on=True,
            nside=True,
            swap_inout=False,
            no_sampler=False,
            dummy_off=False,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        dum_sampler: bool = self.params['dum_sampler']
        fast_on: bool = self.params['fast_on']
        swap_inout: bool = self.params['swap_inout']
        no_sampler: bool = self.params['no_sampler']
        dummy_off: bool = self.params['dummy_off']

        if not dum_sampler or (no_sampler and not dummy_off):
            self.draw_base(pinfo)
            tidx_samp, tidx_off = 1, 2
            tidx_dummy_off = None
        elif no_sampler and dummy_off:
            pinfo0 = [TilePatternElement(pinfo[1]['ptap_tile'])] + \
                     [TilePatternElement(pinfo[1]['off_tile'], flip=True)] + \
                     [TilePatternElement(pinfo[1][tile['name']]) for tile in self.params['pinfo']['tiles'][2:]]
            self.draw_base((TilePattern(pinfo0), pinfo[1]))
            tidx_samp, tidx_dummy_off, tidx_off = 1, 1, 2
        else:
            pinfo0 = [TilePatternElement(pinfo[1]['ptap_tile'])] + \
                     [TilePatternElement(pinfo[1]['sampler_tile'], flip=True)] + \
                     [TilePatternElement(pinfo[1][tile['name']]) for tile in self.params['pinfo']['tiles'][1:]]
            self.draw_base((TilePattern(pinfo0), pinfo[1]))
            tidx_samp, tidx_off = 1, 3
            tidx_dummy_off = None
        ptap_tile = 0

        seg_dict: Dict[str, Any] = self.params['seg_dict']
        w_n: int = self.params['w_n']
        ridx_samp: int = self.params['ridx_samp']
        ridx_off0: int = self.params['ridx_off0']
        ridx_off1: int = self.params['ridx_off1']

        seg_off0 = seg_dict['off0']
        seg_off1 = seg_dict['off1']
        seg_inv_n0_vg2 = seg_dict['inv_n0_vg2']
        seg_inv_n1_vg2 = seg_dict['inv_n1_vg2']
        seg_on_n = seg_dict['on_n']
        seg_capn = seg_dict['cap_n']

        seg_invp = seg_dict['inv_p']
        seg_invn = seg_dict['inv_n']
        seg_mid = seg_dict['mid']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # Calculate segments

        # side tap ncols
        tap_ncol = self.get_tap_ncol() + self.sub_sep_col
        tap_ncol += tap_ncol & 1
        # Calc max segments
        seg_max = max(seg_off0 + seg_inv_n0_vg2 + 2 * min_sep,
                      seg_off1 + seg_inv_n1_vg2 + 2 * min_sep) + max(seg_capn, seg_on_n) + 2 * tap_ncol
        if no_sampler or dummy_off:
            sampler_master = None
        else:
            sampler_params = \
                dict(m_list=seg_dict['sampler'], nside=self.params['nside'],
                     tap_conn=SubPortMode.ODD if swap_inout else SubPortMode.EVEN,
                     sampler_unit_params=self.params['unit_sampler_params'].copy(
                         append=dict(g_on_s=swap_inout,
                                     pinfo=self.get_draw_base_sub_pattern(1, 3 if dum_sampler else 2))))
            sampler_master = self.new_template(Sampler, params=sampler_params)
            sampler_ncols = sampler_master.num_cols
            seg_max = max(seg_max, sampler_ncols + 2 * int(swap_inout))
            if ((seg_max - sampler_ncols) // 2) & 1:
                seg_max += 2
        seg_tot = seg_max
        seg_tot2 = seg_tot // 2

        # Place other transistors
        # Align cap_n and on_n
        cur_col_off0 = seg_tot2 - (seg_off0 + seg_on_n + seg_inv_n0_vg2 + 2 * min_sep) // 2
        cur_col_off1 = seg_tot2 - (seg_off1 + seg_capn + seg_inv_n1_vg2 + 2 * min_sep) // 2
        cur_col0 = cap_bot_vm_start_col = min(cur_col_off0, cur_col_off1)
        cur_col0 += (cur_col0 & 1)

        # left side tap
        tap_vdd_list, tap_vss_list = [], []
        self.add_tap(self.get_tap_ncol(), tap_vdd_list, tap_vss_list, tile_idx=tidx_off, flip_lr=True)
        self.add_tap(self.get_tap_ncol(), tap_vdd_list, tap_vss_list, tile_idx=tidx_samp, flip_lr=True)

        cur_col = max(cur_col0 + seg_capn + min_sep, cur_col0 + seg_on_n + min_sep)
        cur_col += (cur_col & 1)

        # Middle row: on_n and off0
        on_n = self.add_mos(ridx_off0, cur_col0, seg_on_n, w=w_n, g_on_s=False, tile_idx=tidx_off)
        off0_vg2 = self.add_mos(ridx_off0, cur_col, seg_inv_n0_vg2, w=w_n, g_on_s=True, tile_idx=tidx_off)
        off0_vg = self.add_mos(ridx_off0, cur_col + min_sep + max(seg_inv_n0_vg2, seg_inv_n1_vg2),
                               seg_off0, w=w_n, g_on_s=True, tile_idx=tidx_off)

        if no_sampler and dummy_off:
            off0_dummy = self.add_mos(ridx_off0, cur_col + min_sep + max(seg_inv_n0_vg2, seg_inv_n1_vg2),
                                      seg_off0, g_on_s=True, tile_idx=tidx_dummy_off)
        else:
            off0_dummy = None

        vg2_vm_stop_col = cur_col + seg_inv_n0_vg2
        vg2_vm_start_col = cur_col
        vg_vm_start_col = cur_col + min_sep + seg_inv_n0_vg2
        vg_vm_stop_col = vg_vm_start_col + seg_off0 // 3 * 2

        # Top row: cap_n and off1
        capn = self.add_mos(ridx_off1, cur_col0, seg_capn, w=w_n, g_on_s=True, tile_idx=tidx_off)
        off1_vg2 = self.add_mos(ridx_off1, cur_col, seg_inv_n1_vg2, g_on_s=False, tile_idx=tidx_off)
        off1_vg = self.add_mos(ridx_off1, cur_col + min_sep + max(seg_inv_n0_vg2, seg_inv_n1_vg2),
                               seg_off1, g_on_s=False, tile_idx=tidx_off)
        if no_sampler and dummy_off:
            off1_dummy = self.add_mos(ridx_off1, cur_col + min_sep + max(seg_inv_n0_vg2, seg_inv_n1_vg2),
                                      seg_off1, g_on_s=False, tile_idx=tidx_dummy_off)
        else:
            off1_dummy = None

        seg_max = max(seg_max, cur_col + max(seg_off0 + seg_inv_n0_vg2, seg_off1 + seg_inv_n1_vg2) + min_sep + tap_ncol)

        self.set_mos_size(num_cols=seg_max, num_tiles=tidx_off + 2)

        if not no_sampler:
            # Place sampler first, If has dummy sampler, put output btw dum_samp and samp
            sampler_col = (seg_max // 2 - sampler_ncols // 2)
            sampler_col += sampler_col & 1
            sampler_col -= int(swap_inout)
            sampler = self.add_tile(sampler_master, 1, sampler_col)
        else:
            fill_conn_layer_intv(self, 1, 0, extend_to_gate=False, fill_empty=True)
            if dummy_off:
                fill_conn_layer_intv(self, 1, 1, extend_to_gate=False, fill_empty=True)
            sampler = None

        nouts = len(seg_dict['sampler'])
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        tr_manager = self.tr_manager
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        out_xm_tidx = self.grid.coord_to_track(xm_layer, (sampler if not no_sampler else self).bound_box.yl,
                                               RoundMode.NEAREST)

        if sampler:
            if nouts > 1:
                for idx in range(nouts):
                    _out = sampler.get_all_port_pins(f'out<{idx}>')
                    _xm_out = self.connect_to_tracks(_out, TrackID(xm_layer, out_xm_tidx, tr_w_sig_xm))
                    # _ym_tidx = self.grid.coord_to_track(ym_layer, _xm_out.middle, RoundMode.NEAREST)
                    # _, ym_locs = tr_manager.place_wires(ym_layer, ['sig'] * len(_out), center_coord=_xm_out.middle)
                    ym_avail_locs = \
                        self.get_available_tracks(ym_layer,
                                                  self.grid.coord_to_track(ym_layer, _xm_out.lower,
                                                                           RoundMode.GREATER_EQ),
                                                  self.grid.coord_to_track(ym_layer, _xm_out.upper,
                                                                           RoundMode.LESS_EQ),
                                                  _xm_out.bound_box.yl, _xm_out.bound_box.yh,
                                                  width=tr_w_sig_ym, sep=tr_manager.get_sep(ym_layer, ('sig', 'sig')))
                    _ym_out = [self.connect_to_tracks(_xm_out, TrackID(ym_layer, _ym_tidx, tr_w_sig_ym)) for _ym_tidx in
                               ym_avail_locs]
                    self.add_pin(f'out<{idx}>', _ym_out)
            else:
                _out = sampler.get_all_port_pins('out')
                _xm_out = self.connect_to_tracks(_out, TrackID(xm_layer, out_xm_tidx, tr_w_sig_xm, grid=self.grid))
                # In this case, use half of the width for inout connection
                _xm_out_len = _xm_out.upper - _xm_out.lower
                _ym_out_lower = _xm_out.lower + _xm_out_len // 4
                _ym_out_upper = _xm_out.upper

                ym_avail_tids = \
                    self.get_tids_between(ym_layer,
                                          self.grid.coord_to_track(ym_layer, _ym_out_lower, RoundMode.GREATER_EQ),
                                          self.grid.coord_to_track(ym_layer, _ym_out_upper, RoundMode.LESS_EQ),
                                          tr_manager.get_width(ym_layer, 'ana_sig'), 0, 0, True)

                _ym_out = [self.connect_to_tracks(_xm_out, tid, min_len_mode=MinLenMode.LOWER) for tid in ym_avail_tids]
                xm1_layer = ym_layer + 1
                xm1_out_tidx = self.grid.coord_to_track(xm1_layer, _ym_out[0].lower, RoundMode.NEAREST)
                xm1_out = self.connect_to_tracks(_ym_out, TrackID(xm1_layer, xm1_out_tidx,
                                                                  tr_manager.get_width(xm1_layer, 'sig'),
                                                                  grid=self.grid))
                self.add_pin('out', xm1_out)
        self.add_tap(self.num_cols - self.get_tap_ncol(), tap_vdd_list, tap_vss_list, tile_idx=tidx_off, flip_lr=False)
        self.add_tap(self.num_cols - self.get_tap_ncol(), tap_vdd_list, tap_vss_list, tile_idx=tidx_samp, flip_lr=False)

        # Add sub-contact
        vss_conn_bot = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=ptap_tile,
                                                  port_mode=SubPortMode.ODD)
        vss_conn_top = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=tidx_off + 1,
                                                  port_mode=SubPortMode.ODD)
        # self.add_tap(self.num_cols - self.get_tap_ncol(), tap_vdd_list, tap_vss_list, tile_idx=4, flip_lr=False)

        # _nrow = self.get_tile_info(tidx_inv)[0].num_rows
        # for jdx in range(_nrow):
        #     fill_conn_layer_intv(self, tidx_inv, jdx, True)

        # --- Connection
        # # - sampler -> on_n
        if sampler:
            self.connect_wires([on_n.s] + sampler.get_all_port_pins('out_conn' if swap_inout else 'in_conn'))
        cap_bot_on = self.connect_to_tracks(on_n.d,
                                            self.get_track_id(ridx_off0, MOSWireType.DS, 'sig_w', 0, tile_idx=tidx_off))
        in_hm = self.connect_to_tracks(on_n.s,
                                       self.get_track_id(ridx_off0, MOSWireType.DS, 'sig', 0, tile_idx=tidx_off))
        vg2 = self.connect_to_tracks(on_n.g,
                                     self.get_track_id(ridx_off0, MOSWireType.G, 'clk', 0, tile_idx=tidx_off))
        # - off0
        vdd_off0 = self.connect_to_tracks([off0_vg.g, off0_vg2.g], self.get_track_id(ridx_off0, MOSWireType.G, 'sup', 0,
                                                                                     tile_idx=tidx_off))
        _vg_off = self.connect_to_tracks(off0_vg.s, self.get_track_id(ridx_off0, MOSWireType.DS, 'sig', 0,
                                                                      tile_idx=tidx_off))
        vg0 = self.connect_to_tracks(off0_vg.d, self.get_track_id(ridx_off0, MOSWireType.DS, 'sig_w', 0,
                                                                  tile_idx=tidx_off))
        _vg2_mid = self.connect_to_tracks(off0_vg2.s, self.get_track_id(ridx_off0, MOSWireType.DS, 'sig', 0,
                                                                        tile_idx=tidx_off))
        _vg2_off = self.connect_to_tracks(off0_vg2.d, self.get_track_id(ridx_off0, MOSWireType.DS, 'sig_w', 0,
                                                                        tile_idx=tidx_off))
        self.extend_wires(off0_vg2.d, lower=vdd_off0.bound_box.yl)
        # - off0 -> off1
        self.connect_wires([off0_vg.s, off1_vg.s])
        self.connect_wires([off0_vg2.s, off1_vg2.s])
        # - off1
        samp_b = self.connect_to_tracks([off1_vg.g, off1_vg2.g, capn.g],
                                        self.get_track_id(ridx_off1, MOSWireType.G, 'clk', 0, tile_idx=tidx_off))
        self.connect_to_tracks(off1_vg.s,
                               self.get_track_id(ridx_off1, MOSWireType.DS, 'sig_w', 0, tile_idx=tidx_off))
        self.connect_to_tracks(off1_vg2.s,
                               self.get_track_id(ridx_off1, MOSWireType.DS, 'sig_w', 0, tile_idx=tidx_off))

        self.connect_to_track_wires(vg2, off0_vg2.d)

        # Dummy off connections:
        if no_sampler and dummy_off:
            vdd_off_dummy = [self.connect_to_tracks(off0_dummy.g, self.get_track_id(ridx_off0, MOSWireType.G,
                                                                                    'sup', 0, tile_idx=tidx_dummy_off)),
                             self.connect_to_tracks(off1_dummy.g, self.get_track_id(ridx_off1, MOSWireType.G,
                                                                                    'clk', 0, tile_idx=tidx_dummy_off))]
            off_hm = self.connect_to_tracks(off0_dummy.d, self.get_track_id(ridx_off0, MOSWireType.DS, 'sig_w', 0,
                                                                            tile_idx=tidx_dummy_off))
            self.connect_to_tracks(off1_dummy.s,
                                   self.get_track_id(ridx_off1, MOSWireType.DS, 'sig_w', 0, tile_idx=tidx_dummy_off))
            self.connect_wires([off0_dummy.s, off1_dummy.s])
        else:
            vdd_off_dummy = None
            off_hm = None
        # self.add_pin('vg2', vg2, show=self.show_pins)
        # - on_n -> cap_n
        # cap_n
        cap_bot_n = self.connect_to_tracks(capn.s,
                                           self.get_track_id(ridx_off1, MOSWireType.DS, 'sig_w', 0, tile_idx=tidx_off))

        # - sampler -> off0
        if sampler:
            self.connect_wires(sampler.get_all_port_pins('sam_conn') + [off0_vg.d])
        #####################
        # vm connections
        #####################
        cap_bot_vm_stop_col = min(seg_invp, seg_invn, seg_on_n, seg_capn) + cap_bot_vm_start_col
        cap_bot_vm_tidx_list = \
            self.get_tids_between(vm_layer, self.arr_info.col_to_track(vm_layer, cap_bot_vm_start_col),
                                  self.arr_info.col_to_track(vm_layer, cap_bot_vm_stop_col),
                                  tr_manager.get_width(vm_layer, 'cap'), 0, 0, True)

        tr_w_cap_vm = tr_manager.get_width(vm_layer, 'cap')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        cap_bot_vm = [self.connect_to_tracks([cap_bot_on, cap_bot_n], tid) for tid in cap_bot_vm_tidx_list]

        vg2_vm_tidx_list = self.get_tids_between(vm_layer, self.arr_info.col_to_track(vm_layer, vg2_vm_start_col),
                                                 self.arr_info.col_to_track(vm_layer, vg2_vm_stop_col),
                                                 tr_manager.get_width(vm_layer, 'cap'), 0, 0, True)

        vg2_vm = [self.connect_to_tracks(vg2, tid, min_len_mode=MinLenMode.UPPER) for tid in vg2_vm_tidx_list]

        # if has pre charge device:
        seg_pre = seg_dict.get('pre', 0)
        has_pre_dev = bool(seg_pre)
        if has_pre_dev:
            pre = self.add_mos(ridx_off0, cur_col + 2*min_sep + max(seg_inv_n0_vg2, seg_inv_n1_vg2) + seg_off0,
                               seg_pre, w=w_n, g_on_s=True, tile_idx=tidx_off)
            self.connect_to_track_wires(vdd_off0, pre.d)
            self.connect_to_track_wires(vg0, pre.s)
            sam_hm = self.connect_to_tracks(pre.g, vg2.track_id)
            pre_sam_vm_tidx = self.grid.coord_to_track(vm_layer, sam_hm.middle, RoundMode.NEAREST)
            sam_vm = self.connect_to_tracks(sam_hm, TrackID(vm_layer, pre_sam_vm_tidx, tr_w_sig_vm),
                                            min_len_mode=MinLenMode.MIDDLE)
            sam_xm_tidx = self.grid.coord_to_track(xm_layer, sam_vm.middle, RoundMode.NEAREST)
            tr_w_clk_xm = tr_manager.get_width(xm_layer, 'ana_sig')
            sam_xm_tidx += self.get_track_sep(xm_layer, 1, tr_w_clk_xm)
            sam_xm = self.connect_to_tracks(sam_vm, TrackID(xm_layer, sam_xm_tidx, tr_w_clk_xm, grid=self.grid),
                                            min_len_mode=MinLenMode.MIDDLE)
            self.add_pin('sam', sam_xm)

        # Fill empty m1
        _nrow = self.get_tile_info(tidx_off)[0].num_rows
        for jdx in range(_nrow):
            fill_conn_layer_intv(self, tidx_off, jdx, True)

        # - tap
        vss_top = self.connect_to_tracks([vss_conn_top, off1_vg.d, off1_vg2.d, capn.d] + tap_vss_list,
                                         self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=tidx_off + 1))
        vss_bot = \
            self.connect_to_tracks([vss_conn_bot] + tap_vss_list + sampler.get_all_port_pins('VSS', self.conn_layer)
                                   if sampler else [vss_conn_bot] + tap_vss_list,
                                   self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=ptap_tile))
        if no_sampler and dummy_off:
            self.connect_to_track_wires(vss_bot, off1_dummy.d)

        # Connect wide supply, it will be used for entire btstrp
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        vdd_vm_l_tidx = self.arr_info.col_to_track(vm_layer, 0)
        vdd_vm_r_tidx = self.arr_info.col_to_track(vm_layer, self.num_cols)
        sup_sep_ntr = self.get_track_sep(vm_layer, tr_w_sup_vm, tr_w_sup_vm)
        vss_vm_l_tidx = vdd_vm_l_tidx - sup_sep_ntr
        vss_vm_r_tidx = vdd_vm_r_tidx + sup_sep_ntr
        vdd_vm = [self.connect_to_tracks([vdd_off0], TrackID(vm_layer, tidx, tr_w_sup_vm, grid=self.grid)) for tidx in
                  [vss_vm_l_tidx, vss_vm_r_tidx]]

        vss_vm = [self.connect_to_tracks([vss_top, vss_bot], TrackID(vm_layer, tidx, tr_w_sup_vm, grid=self.grid)) for
                  tidx in
                  [vdd_vm_l_tidx, vdd_vm_r_tidx]]

        #####################
        # xm connections
        #####################
        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')
        cap_bot_xm_tidx = self.grid.coord_to_track(xm_layer, cap_bot_vm[0].upper, RoundMode.NEAREST)
        cap_bot_xm_tidx = \
            self.get_tids_between(xm_layer,
                                  self.grid.coord_to_track(xm_layer, cap_bot_vm[0].lower, RoundMode.NEAREST),
                                  self.grid.coord_to_track(xm_layer, cap_bot_vm[0].upper, RoundMode.NEAREST),
                                  tr_manager.get_width(xm_layer, 'cap'), 0, 0, True)
        cap_bot_xm = [self.connect_to_tracks(cap_bot_vm, tid) for tid in cap_bot_xm_tidx]
        # If not sampler, export vg and input signal to xm layer
        vg_vm_tidx_list = self.get_tids_between(vm_layer, self.arr_info.col_to_track(vm_layer, vg_vm_start_col),
                                                vdd_vm_r_tidx - sup_sep_ntr // 2,
                                                tr_manager.get_width(vm_layer, 'cap'), 0, 1, True)
        num_vg_vm = len(vg_vm_tidx_list)
        vg_vm = [self.connect_to_tracks([sampler.get_pin('sam'), vg0] if sampler else vg0, tid) for tid in
                 vg_vm_tidx_list[:num_vg_vm // 2 + 1]]

        if no_sampler:
            tile_info, yb, _ = self.get_tile_info(tidx_samp)
            tile_height = tile_info.height
            y_sampler_top = yb + tile_height
            # _, xm_locs = tr_manager.place_wires(xm_layer, ['ana_sig'] * 2, center_coord=y_sampler_mid)
            tr_w_ana_sig_xm = tr_manager.get_width(xm_layer, 'ana_sig')
            xm_tids = self.get_tids_between(xm_layer,
                                            self.grid.coord_to_track(xm_layer, yb, RoundMode.NEAREST),
                                            self.grid.coord_to_track(xm_layer, y_sampler_top, RoundMode.NEAREST),
                                            tr_w_ana_sig_xm, 0, 0, True)
            num_xm_tids = len(xm_tids)

            vg_xm_l = vg_vm[0].bound_box.xl
            vg_xm_r = vg_vm[-1].bound_box.xl
            vin_xm_l = cap_bot_vm[-1].bound_box.xl
            vin_xm_r = vg2_vm[0].bound_box.xl
            vin_xm = [self.add_wires(xm_layer, tid.base_index, lower=vin_xm_l, upper=vin_xm_r,
                                     width=tid.width) for tid in xm_tids]
            xm_tids = xm_tids[num_xm_tids // 2:] if dummy_off else xm_tids
            vg_xm = [self.add_wires(xm_layer, tid.base_index, lower=vg_xm_l, upper=vg_xm_r,
                                    width=tid.width) for tid in xm_tids]
            vin_xm = WireArray.list_to_warr(vin_xm)
            vg_xm = WireArray.list_to_warr(vg_xm)
            self.connect_to_track_wires(vg_xm, vg_vm)

            if dummy_off:
                self.connect_to_track_wires(vdd_vm, vdd_off_dummy)
                off_vm = [self.connect_to_tracks(off_hm, tid) for tid in vg_vm_tidx_list[num_vg_vm // 2 + 1:]]
                off_xm = [self.connect_to_tracks(off_vm, tid) for tid in xm_tids]
                self.add_pin('off_xm', off_xm)

            self.add_pin('in_hm', in_hm)
            self.add_pin('in_xm', vin_xm)
            self.add_pin('vg_vm', vg_vm)
            self.add_pin('vg_xm', vg_xm)

        # self.add_pin('sam', samp_inv)
        self.add_pin('sam_b', samp_b)
        self.add_pin('vg', vg_vm)
        # self.add_pin('mid', vmid_vm)
        self.add_pin('vg2', vg2_vm)
        self.add_pin('cap_bot', cap_bot_xm)
        self.add_pin('cap_bot_hm', [cap_bot_on, cap_bot_n], hide=True)

        if sampler:
            if sampler.has_port('in_c'):
                self.reexport(sampler.get_port('in_c'))
            if sampler.has_port('VSS'):
                self.reexport(sampler.get_port('VSS'), net_name='VSS_hm')
            if sampler.has_port('VDD'):
                self.reexport(sampler.get_port('VDD'), net_name='VDD_hm')

            # self.connect_to_track_wires(vss_top, tap_vss_list)
            self.add_pin('in', sampler.get_all_port_pins('in'))
            self.add_pin('in_coupled', sampler.get_all_port_pins('in_coupled'))
        self.add_pin('VSS', vss_vm, show=self.show_pins)
        self.add_pin('VDD', vdd_vm, show=self.show_pins)
        #
        dev_info = dict(
            XSAM=sampler_master.sch_params if sampler else None,
            XON_N={'nf': seg_dict['on_n'],
                   'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(ridx_off0).row_info.threshold},
            XOFF_N0={'nf': seg_dict['off0'],
                     'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(ridx_off0).row_info.threshold},
            XOFF_N1={'nf': seg_dict['off1'],
                     'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(ridx_off1).row_info.threshold},
            XCAP_N={'nf': seg_dict['cap_n'],
                    'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(ridx_off1).row_info.threshold},
        )
        off_info = self.get_tile_pinfo(tidx_off).get_row_place_info(ridx_off0)
        if dummy_off:
            dev_info['XOFF_DUM0'] = {'nf': seg_dict['off0'],
                                     'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(
                                         ridx_off0).row_info.threshold}
            dev_info['XOFF_DUM1'] = {'nf': seg_dict['off1'],
                                     'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(
                                         ridx_off0).row_info.threshold}
        if fast_on:
            dev_info.update(
                dict(
                    XINV_N0_VG2={'nf': seg_dict['inv_n0_vg2'], 'intent': off_info.row_info.threshold},
                    XINV_N1_VG2={'nf': seg_dict['inv_n1_vg2'], 'intent': off_info.row_info.threshold},

                )
            )

        if seg_dict.get('pre', 0):
            dev_info.update(
                XPRE={'nf': seg_dict['pre'],
                      'intent': self.get_tile_pinfo(tidx_off).get_row_place_info(ridx_off1).row_info.threshold},
            )
        self.sch_params = dict(
            dev_info=dev_info,
            has_dum_sampler=dum_sampler,
            dummy_off=dummy_off
        )


class BootstrapNMOSWrap(BootstrapNMOS):
    """
    This class wrap BootstrapNMOS with a guardring
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._dum_sampler = False

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        master: MOSBase = self.new_template(BootstrapNMOS, params=self.params)
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
        for name in inst.port_names_iter():
            self.reexport(inst.get_port(name))
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
        self.connect_to_track_wires([it[1] for it in vdd_hm_dict.items()], inst.get_all_port_pins('VDD'))
        self.connect_to_track_wires(vdd_vm_list, [it[1] for it in vdd_hm_dict.items()])
        self.sch_params = master.sch_params

    def _connect_vm(self, warr_list: List[WireArray], keys: List[int],
                    table: Dict[int, WireArray]) -> None:
        keys.sort()
        for warr in warr_list:
            idx = bisect_left(keys, warr.middle)
            if idx == 0:
                raise ValueError('Cannot find a lower horizontal wire to connect guard ring edge')
            self.connect_to_track_wires(warr, [table[keys[idx - 1]], table[keys[idx]]])


class BootstrapNWL(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row with turn-off transistor 0',
            ridx_p='index for pmos row with turn-off transistor 1',
            fast_on='True to enable fast turn on ON_N',
            has_buf='True to add buffer before sample',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=1,
            fast_on=True,
            has_buf=False,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        seg_dict: Dict[str, Union[int, List]] = self.params['seg_dict']
        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        fast_on: int = self.params['fast_on']
        has_buf: int = self.params['has_buf']

        min_sep = self.min_sep_col

        seg_cap_p = seg_dict['cap_p']
        seg_on_p = seg_dict['on_p']
        seg_cap_p_aux = seg_dict.get('cap_p_aux', 0)

        seg_mid = seg_dict['mid']
        seg_invn = seg_dict['inv_n']
        seg_invp = seg_dict['inv_p']

        # buffers layout in a reversed order

        # Assign tiles for each part
        tidx_inv, tidx_nwl = 1, 3
        tidx_ntapb, tidx_ptap, tidx_ntapt = 0, 2, 4

        # Setup track manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        tr_manager = self.tr_manager

        # --- Floorplan ---
        # = calc. total seg.
        seg_tot_nwl = seg_cap_p_aux + min_sep if seg_cap_p_aux else 0
        seg_tot_nwl += seg_cap_p + seg_on_p + min_sep + min_sep

        if fast_on:
            seg_inv_p = seg_dict['inv_p_vg2']
            seg_inv_n0 = seg_dict['inv_n0_vg2']
            seg_inv_n1 = seg_dict['inv_n1_vg2']
            seg_tot_nwl += min_sep + seg_inv_p
            seg_tot_nwl = max(seg_tot_nwl, seg_inv_n0 + seg_inv_n1 + min_sep) + min_sep
        else:
            seg_inv_p, seg_inv_n0, seg_inv_n1 = 0, 0, 0

        if has_buf:
            seg_invn_buf = seg_dict.get('inv_n_buf', 2)
            seg_tot0_inv = max(seg_invp, seg_invn + min_sep + seg_mid + min_sep + seg_invn_buf + min_sep) + min_sep
            seg_buf_list_n = seg_dict['buf_n'][::-1]
            seg_buf_list_p = seg_dict['buf_p'][::-1]
            if len(seg_buf_list_n) != len(seg_buf_list_p):
                raise ValueError("Buffer length n/p doesn't match")
            seg_tot_inv = seg_tot0_inv + sum([max(a, b) for a, b in zip(seg_buf_list_n, seg_buf_list_p)])
        else:
            seg_tot0_inv = max(seg_invp, seg_invn + min_sep + seg_mid + min_sep) + min_sep
            seg_buf_list_n, seg_buf_list_p = [], []
            seg_tot_inv = seg_tot0_inv
            seg_invn_buf = 0
        seg_tot = max(seg_tot_nwl, seg_tot_inv)

        # Add substrate connection
        ntap_b = self.add_substrate_contact(0, 0, seg=seg_tot, tile_idx=tidx_ntapb)
        ntap_t = self.add_substrate_contact(0, 0, seg=seg_tot, tile_idx=tidx_ntapt)
        ptap = self.add_substrate_contact(0, 0, seg=seg_tot, tile_idx=tidx_ptap)

        # Add inv for vg
        invp = self.add_mos(ridx_p, seg_tot0_inv - seg_invp, seg=seg_invp, tile_idx=tidx_inv, w=w_p)
        if has_buf:
            cur_col = seg_tot0_inv - seg_invn_buf - min_sep
            invn_buf = self.add_mos(ridx_n, cur_col, seg=seg_invn_buf, tile_idx=tidx_inv, w=w_n)
            cur_col -= min_sep + seg_invn
            invn = self.add_mos(ridx_n, cur_col, seg=seg_invn, tile_idx=tidx_inv, w=w_n)
            cur_col -= min_sep + seg_mid
            mid = self.add_mos(ridx_n, cur_col, seg=seg_mid, tile_idx=tidx_inv, w=w_n)
        else:
            cur_col = seg_tot0_inv - seg_invn - min_sep
            invn = self.add_mos(ridx_n, cur_col, seg=seg_invn, tile_idx=tidx_inv, w=w_n)
            cur_col -= min_sep + seg_mid
            mid = self.add_mos(ridx_n, cur_col, seg=seg_mid, tile_idx=tidx_inv, w=w_n)
            invn_buf = None

        # Add buffers for inv_vg
        buf_n_list, buf_p_list = [], []
        cur_loc = seg_tot0_inv
        if has_buf:
            for idx, (segn, segp) in enumerate(zip(seg_buf_list_n, seg_buf_list_p)):
                buf_n_list.append(self.add_mos(ridx_n, cur_loc, seg=segn, tile_idx=tidx_inv, w=w_n))
                buf_p_list.append(self.add_mos(ridx_p, cur_loc, seg=segp, tile_idx=tidx_inv, w=w_p))
                cur_loc += max(segn, segp)

        # Add XCAP_P and inv for vg2
        cur_col = min_sep
        if seg_cap_p_aux:
            cap_p_aux = self.add_mos(0, cur_col, seg_cap_p_aux, w=w_p, tile_idx=tidx_nwl)
            cur_col += seg_cap_p_aux + min_sep
        else:
            cap_p_aux = None

        cap_p = self.add_mos(0, cur_col, seg_cap_p, w=w_p, tile_idx=tidx_nwl)
        cur_col += seg_cap_p + min_sep
        inv_p_2 = self.add_mos(0, cur_col, seg_inv_p, w=w_n, tile_idx=tidx_nwl, g_on_s=True)
        cur_col += seg_inv_p + min_sep
        cur_col += (cur_col & 1)
        on_p = self.add_mos(0, cur_col, seg_on_p, w=w_p, tile_idx=tidx_nwl, g_on_s=True)

        # --- Connections ---

        cap_vg_conn = [cap_p.g, cap_p_aux.g, on_p.d] if seg_cap_p_aux else [cap_p.g, on_p.d]
        vg_cap = self.connect_to_tracks(cap_vg_conn,
                                        self.get_track_id(0, MOSWireType.G, 'sig', 0, tile_idx=tidx_nwl))
        vg_on = self.connect_to_tracks(on_p.d, self.get_track_id(0, MOSWireType.DS, 'cap', 1, tile_idx=tidx_nwl))

        cap_vdd_conn = [cap_p.d, cap_p_aux.d] if seg_cap_p_aux else [cap_p.d]
        cap_vdd = self.connect_to_tracks(cap_vdd_conn,
                                         self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=tidx_nwl))

        vmid = self.connect_to_tracks([inv_p_2.g, on_p.g],
                                      self.get_track_id(0, MOSWireType.G, 'clk', 0, tile_idx=tidx_nwl))
        # _inv1_d_p = self.connect_to_tracks([inv_p_2.d],
        #                                    self.get_track_id(0, MOSWireType.DS, 'cap', 1, tile_idx=tidx_nwl),
        #                                    min_len_mode=MinLenMode.MIDDLE)
        # -- vg2
        vg2_hm = self.connect_to_tracks([inv_p_2.d],
                                        self.get_track_id(0, MOSWireType.DS, 'cap', 0, tile_idx=tidx_nwl),
                                        min_len_mode=MinLenMode.MIDDLE)

        cap_top_conn = [ntap_t, cap_p.s, inv_p_2.s, on_p.s] if fast_on else [ntap_t, cap_p.s, on_p.s]
        cap_top = self.connect_to_tracks(cap_top_conn,
                                         self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=tidx_ntapt))

        # --- Inv's connection:
        vss_inv = self.connect_to_tracks([ptap] + [tx.s for tx in buf_n_list],
                                         self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=tidx_ptap))
        vdd_inv = self.connect_to_tracks([ntap_b, invp.s] + [tx.s for tx in buf_p_list],
                                         self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=tidx_ntapb))

        samp_inv = self.connect_to_tracks([invp.g, invn.g],
                                          self.get_track_id(ridx_p, MOSWireType.G, 'sig', 0, tile_idx=1))
        cap_bot = self.connect_to_tracks([mid.s, invn.s],
                                         self.get_track_id(ridx_n, MOSWireType.DS, 'sig', 0, tile_idx=1))
        vmid_tidx = self.arr_info.col_to_track(vm_layer, seg_tot0_inv, mode=RoundMode.NEAREST)
        samp_b_buf, samp_b_inv = None, None
        buf_mid_conn_vm_list = []

        vg_inv = self.connect_to_tracks(mid.g, self.get_track_id(ridx_n, MOSWireType.G, 'sig', 0, tile_idx=1))
        vmid_inv_p = self.connect_to_tracks(invp.d, self.get_track_id(ridx_p, MOSWireType.DS, 'sig', 0, tile_idx=1))
        vmid_conn_n = [invn.d, mid.d, invn_buf.d] if has_buf else [invn.d, mid.d]
        vmid_inv_n = self.connect_to_tracks(vmid_conn_n,
                                            self.get_track_id(ridx_n, MOSWireType.DS, 'sig', 1, tile_idx=1))

        self.set_mos_size()
        for idx in range(self.num_tile_rows):
            _nrow = self.get_tile_info(idx)[0].num_rows
            for jdx in range(_nrow):
                fill_conn_layer_intv(self, idx, jdx, True)

        self.add_pin('VSS', vss_inv, show=self.show_pins, connect=False)
        self.add_pin('VDD', [cap_vdd, vdd_inv], show=self.show_pins, connect=False)
        self.add_pin('cap_top', cap_top, show=self.show_pins)
        self.add_pin('cap_bot', cap_bot, show=self.show_pins)
        self.add_pin('vg', [vg_cap, vg_on], show=self.show_pins)
        self.add_pin('vg2', [vg2_hm, vg_inv], show=self.show_pins)
        self.add_pin('vmid', [vmid_inv_p, vmid_inv_n, vmid, ], show=self.show_pins)
        self.add_pin('sample', samp_inv, show=self.show_pins)

        prow_info = self.get_tile_pinfo(tidx_nwl).get_row_place_info(0)
        ninv_info, pinv_info = self.get_tile_pinfo(tidx_inv).get_row_place_info(0), self.get_tile_pinfo(
            tidx_inv).get_row_place_info(1)

        dev_info = dict(
            XON_P={'nf': seg_dict['on_p'], 'intent': prow_info.row_info.threshold},
            XCAP_P={'nf': seg_dict['cap_p'], 'intent': prow_info.row_info.threshold},
            XINV_P={'nf': seg_dict['inv_p'], 'intent': pinv_info.row_info.threshold},
            XINV_N={'nf': seg_dict['inv_n'], 'intent': ninv_info.row_info.threshold},
            XMID={'nf': seg_dict['mid'], 'intent': ninv_info.row_info.threshold},
        )
        for idx, (segn, segp) in enumerate(zip(seg_buf_list_n[::-1], seg_buf_list_p[::-1])):
            dev_info.update({f"XSAMPLE_INVN<{idx}>": {'nf': segn, 'intent': ninv_info.row_info.threshold}})
            dev_info.update({f"XSAMPLE_INVP<{idx}>": {'nf': segp, 'intent': pinv_info.row_info.threshold}})
        if seg_invn_buf:
            dev_info.update({'XINV_N_BUF': {'nf': seg_invn_buf, 'intent': pinv_info.row_info.threshold}})

        if seg_cap_p_aux:
            dev_info.update({'XCAP_P_AUX': {'nf': seg_dict['cap_p_aux'], 'intent': prow_info.row_info.threshold}})
        if fast_on:
            dev_info.update(
                dict(
                    XINV_P_VG2={'nf': seg_dict['inv_p_vg2'], 'intent': prow_info.row_info.threshold},
                )
            )
        self.sch_params = dict(
            lch=self.arr_info.lch,
            intent='lvt',
            dev_info=dev_info
        )


class Bootstrap(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._top_hor_vdd_coord = 0

    @property
    def top_hor_vdd_coord(self):
        return self._top_hor_vdd_coord

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'bootstrap_fast')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            nmos_params='NMOS part parameters.',
            nwl_params='NWL part parameters.',
            mom_params='capacitor parameters',
            seg_dict='Segment dict for all transistors',
            top_layer='Top routing layer',
            tr_widths='track widths',
            tr_spaces='track widths',
            fast_on='True to fast turn on XON_N',
            vertical_layout='true to put cap on the top of transistors',
            nside='True to swap dummy sampler signal wires',
            vert_sampler='True to exclude sampler, export vg and vin signals',
            no_sampler='True to exclude sampler',
            sampler_params='Parameter for vertical sampler',
            swap_inout='True to swap input/output in schematic',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(fast_on=True, nside=True, vertical_layout=False, vert_sampler=False, sampler_params={},
                    no_sampler=False, swap_inout=False)

    def draw_layout(self) -> None:
        top_layer = self.params['top_layer']
        nmos_params: Param = self.params['nmos_params']
        nwl_params: Param = self.params['nwl_params']
        mom_params: ImmutableSortedDict[str, Any] = self.params['mom_params']
        seg_dict: Mapping[str, int] = self.params['seg_dict']
        tr_widths: WDictType = self.params['tr_widths']
        tr_spaces: SpDictType = self.params['tr_spaces']
        fast_on: bool = self.params['fast_on']
        vert_sampler: bool = self.params['vert_sampler']
        no_sampler: bool = self.params['no_sampler']
        vertical_layout: bool = self.params['vertical_layout']
        swap_inout: bool = self.params['swap_inout']

        has_cap_aux = bool(seg_dict.get('cap_p_aux', 0))

        # get track manager
        tr_manager = TrackManager(grid=self.grid, tr_widths=tr_widths, tr_spaces=tr_spaces)
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      nmos_params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        nmos_new_params = nmos_params.to_dict()
        nwl_new_params = nwl_params.to_dict()
        nmos_new_params.update(dict(fast_on=fast_on, nside=self.params['nside'], no_sampler=vert_sampler))
        nwl_new_params.update(dict(fast_on=fast_on))
        btstrp_nmos_params = dict(
            cls_name=BootstrapNMOSWrap.get_qualified_name() if no_sampler else BootstrapNMOS.get_qualified_name(),
            params=nmos_new_params,
            export_private=False
        )
        btstrp_nwl_params = dict(
            cls_name=BootstrapNWL.get_qualified_name(),
            params=nwl_new_params,
            export_private=False
        )
        nmos_master: TemplateBase = self.new_template(GenericWrapper, params=btstrp_nmos_params)
        nwl_master: TemplateBase = self.new_template(GenericWrapper, params=btstrp_nwl_params)
        cboot_params = copy.deepcopy(mom_params.to_dict())

        w_blk, h_blk = self.grid.get_block_size(max(nmos_master.top_layer, nwl_master.top_layer))
        nmos_box, nwl_box = nmos_master.bound_box, nwl_master.bound_box

        w_nmos, h_nmos = nmos_box.w, nmos_box.h
        w_nwl, h_nwl = nwl_box.w, nwl_box.h

        w_cap = int(mom_params['width'] / self.grid.resolution)
        if vertical_layout:
            w_cap = int(max(nmos_master.bound_box.w, nwl_master.bound_box.w))
        w_cap = (w_cap // w_blk) * w_blk - 2 * w_blk
        h_tot = h_nmos + h_nwl
        if vertical_layout and not bool(mom_params['height']):
            raise ValueError('cap height needs to be set in vertical layout mode')

        h_cap = max(int(mom_params['height'] / self.grid.resolution), h_tot)
        h_tot = -(-h_tot // h_blk) * h_blk

        h_tot += 2 * h_blk if no_sampler else 0

        if has_cap_aux:
            h_cap_tot = h_cap - 4 * mom_params['margin'] - 4 * h_blk
            cboot_height = (int(0.85 * h_cap_tot) // h_blk) * h_blk
        else:
            h_cap_tot = h_cap - 2 * mom_params['margin'] - 2 * h_blk
            cboot_height = -(-(h_cap_tot // h_blk)) * h_blk

        cboot_params['height'] = cboot_height
        cboot_params['width'] = w_cap
        cboot_master: GenericWrapper = self.new_template(GenericWrapper,
                                                         params=dict(cls_name=MOMCapOnMOS.get_qualified_name(),
                                                                     export_private=False,
                                                                     params=cboot_params))
        tr_w_cap_ym = tr_manager.get_width(ym_layer, 'cap')
        cap_ym_start_idx = self.grid.coord_to_track(vm_layer, 0 if vertical_layout else cboot_master.bound_box.w,
                                                    mode=RoundMode.GREATER_EQ)
        ym_cap_ntr, locs = tr_manager.place_wires(ym_layer, ['cap'] * 3, align_track=cap_ym_start_idx)
        ym_tids = [TrackID(ym_layer, tidx, tr_w_cap_ym) for tidx in locs[1:]]

        w_cap = cboot_master.bound_box.w
        w_tot = max(w_nmos, w_nwl) if vertical_layout else max(w_nmos, w_nwl) + w_cap
        clk_ym_ntr, _ = tr_manager.place_wires(ym_layer, ['dum'] + ['clk'] * 2)
        w_tot += self.grid.track_to_coord(ym_layer, ym_cap_ntr)
        w_tot = -(-w_tot // w_blk) * w_blk

        nmos = self.add_instance(nmos_master, inst_name='XNMOS', xform=Transform(w_tot - max(w_nmos, w_nwl), 0))
        y_nwl = -(-nmos.bound_box.yh // h_blk) * h_blk
        x_nwl = -(-(w_tot - (w_nmos + w_nwl) // 2) // w_blk) * w_blk
        nwl = self.add_instance(nwl_master, inst_name='XNMOS', xform=Transform(x_nwl, h_tot, Orientation.MX))

        if vertical_layout:
            y_cap = -(-nwl.bound_box.yh // h_blk) * h_blk
            x_cap = -(-(w_tot - cboot_master.core_bound_box.w) // w_blk) * w_blk - 2 * w_blk
            cboot = self.add_instance(cboot_master, inst_name='CBOOT', xform=Transform(x_cap, y_cap))
        else:
            cboot = self.add_instance(cboot_master, inst_name='CBOOT', xform=Transform(0, 0))

        if has_cap_aux:
            caux_params = copy.deepcopy(mom_params.to_dict())
            caux_height = h_cap_tot - cboot_height
            if 'aux_width' in caux_params.keys():
                w_aux_cap = int(mom_params['aux_width'] / self.grid.resolution)
                w_aux_cap = -(-w_aux_cap // w_blk) * w_blk
            else:
                w_aux_cap = w_cap
            caux_params['height'] = caux_height
            caux_params['width'] = w_aux_cap
            caux_master = self.new_template(MOMCapCore, params=caux_params)
            y_caux = -(-(int(h_cap - caux_master.bound_box.h)) // h_blk) * h_blk
            caux = self.add_instance(caux_master, inst_name='CAUX', xform=Transform(0, y_caux))
        else:
            caux = None
            caux_master = None

        w_blk, h_blk = self.grid.get_block_size(top_layer, half_blk_x=True)
        w_tot += self.grid.track_to_coord(ym_layer, clk_ym_ntr)
        w_tot = -(-w_tot // w_blk) * w_blk
        h_tot = -(-(cboot.bound_box.yh if vertical_layout else max(h_tot, h_cap)) // h_blk) * h_blk
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot))

        if vert_sampler and not no_sampler:
            sampler_params = self.params['sampler_params']
            sampler_params = dict(
                cls_name=SamplerVertCol.get_qualified_name(),
                params=sampler_params
            )
            sampler_vert_temp = self.new_template(GenericWrapper, params=sampler_params)
            y_sam = -(sampler_vert_temp.bound_box.h // h_blk) * h_blk
            x_sam = - (-(self.bound_box.xh - sampler_vert_temp.bound_box.w) // w_blk) * w_blk
            sampler_vert = self.add_instance(sampler_vert_temp, inst_name='XSAM', xform=Transform(x_sam, y_sam))

        # --- Connections:
        cap_top_hm = nwl.get_pin('cap_top')
        cap_bot_xm = nmos.get_pin('cap_bot')
        mom_cap_bot = cboot.get_pin('minus', layer=xm1_layer)
        if has_cap_aux:
            mom_cap_bot.append(caux.get_pin('minus', layer=xm_layer))
            mom_cap_aux = caux.get_pin('plus', layer=xm_layer)
        else:
            mom_cap_aux = None
        mom_cap_top = cboot.get_pin('plus', layer=xm1_layer)

        tr_w_cap_vm = tr_manager.get_width(vm_layer, 'cap')

        cap_top_vm_tidx_list = \
            self.get_tids_between(vm_layer,
                                  self.grid.coord_to_track(vm_layer, cap_bot_xm.lower, RoundMode.GREATER_EQ),
                                  self.grid.coord_to_track(vm_layer, cap_bot_xm.upper, RoundMode.GREATER_EQ),
                                  tr_manager.get_width(vm_layer, 'cap'), 0, 0.5, True)
        cap_top_vm = [self.connect_to_tracks(cap_top_hm, tid, min_len_mode=MinLenMode.MIDDLE) for tid in
                      cap_top_vm_tidx_list]
        cap_top_xm_tidx = self.grid.coord_to_track(xm_layer, cap_top_vm[0].middle, RoundMode.NEAREST)
        cap_top_xm = self.connect_to_tracks(cap_top_vm, TrackID(xm_layer, cap_top_xm_tidx,
                                                                tr_manager.get_width(xm_layer, 'cap'), grid=self.grid))

        cap_top, cap_bot = self.connect_matching_tracks([cap_top_xm, cap_bot_xm], ym_layer,
                                                        [ym.base_index for ym in ym_tids[:2]],
                                                        width=tr_manager.get_width(ym_layer, 'cap'),
                                                        track_upper=self.bound_box.yh,
                                                        track_lower=self.bound_box.yl)
        self.connect_differential_wires(cap_top, cap_bot, mom_cap_top, mom_cap_bot)
        # - vg2
        if fast_on:
            vg2_nmos = nmos.get_port('vg2')
            vg2_nwl = nwl.get_port('vg2')
            self.connect_to_track_wires(vg2_nmos.get_pins(), vg2_nwl.get_pins())
            self.add_pin('vg2', vg2_nmos.get_pins())
        # vg
        vg_nmos = nmos.get_port('vg')
        vg_nwl = nwl.get_port('vg')
        self.connect_to_track_wires(vg_nmos.get_pins(), vg_nwl.get_pins())
        self.add_pin('vg', vg_nmos.get_pins())

        # mid and sampleb connection
        vm_avail_bnd_u = self.grid.coord_to_track(vm_layer, min([vg.bound_box.xl for vg in vg_nmos.get_pins()]),
                                                  RoundMode.LESS_EQ)
        vm_avail_bnd_l = self.grid.coord_to_track(vm_layer, nwl.bound_box.xl, RoundMode.GREATER_EQ)
        vm_avail_tidx = self.get_available_tracks(vm_layer, vm_avail_bnd_l, vm_avail_bnd_u,
                                                  lower=nmos.get_all_port_pins('cap_bot')[0].bound_box.yh,
                                                  upper=nwl.bound_box.yh, width=tr_manager.get_width(vm_layer, 'sig'),
                                                  sep=tr_manager.get_sep(vm_layer, ('sig', 'sig')))
        _mid = self.connect_to_tracks(nwl.get_all_port_pins('vmid'), TrackID(vm_layer, vm_avail_tidx[-1],
                                                                             tr_manager.get_width(vm_layer, 'sig')))
        sample_vm = self.connect_to_tracks(nwl.get_all_port_pins('sample'),
                                           TrackID(vm_layer, vm_avail_tidx[-2], tr_manager.get_width(vm_layer, 'sig')),
                                           min_len_mode=MinLenMode.MIDDLE)
        sampleb_vm = self.connect_to_tracks(nmos.get_pin('sam_b'),
                                            TrackID(vm_layer, vm_avail_tidx[-1], tr_manager.get_width(vm_layer, 'sig')),
                                            min_len_mode=MinLenMode.MIDDLE)
        nmos_cap_bot_topwire = max(nmos.get_all_port_pins('cap_bot'), key=lambda x: x.track_id.base_index)
        _cap_bot = [self.connect_to_tracks([nmos_cap_bot_topwire] + nwl.get_all_port_pins('cap_bot'),
                                           TrackID(vm_layer, tidx, tr_manager.get_width(vm_layer, 'sig')))
                    for tidx in vm_avail_tidx[1:-2]]
        if not vert_sampler:
            in_xm_list = [nmos.get_pin('in'), nmos.get_pin('in_coupled')] if nmos_params['dum_sampler'] else [
                nmos.get_pin('in')]

            in_top_layer = top_layer - 1 if self.grid.get_direction(top_layer) == Orient2D.x else top_layer
            in_sup_top_sp_htr = self.get_track_sep(in_top_layer, tr_manager.get_width(top_layer, 'ana_sig'), 1)
            in_sup_top_sp = (tr_manager.get_width(top_layer, 'ana_sig') * self.grid.get_track_info(top_layer).pitch)

            in_xm_conn_list = []
            warr_mid = self.quentize_to_track_pitch(in_xm_list[0].middle, in_top_layer)
            for warr in in_xm_list:
                in_xm_conn_list.append(self.add_wires(warr.layer_id, warr.track_id.base_index,
                                                      width=warr.track_id.width, num=warr.track_id.num,
                                                      pitch=warr.track_id.pitch,
                                                      lower=warr_mid - in_sup_top_sp, upper=warr_mid + in_sup_top_sp))

            vin_stackup_dict = self.via_stack_up(tr_manager, in_xm_conn_list, in_xm_conn_list[0].layer_id,
                                                 in_top_layer - 1, 'ana_sig')
            vin_top_tid = self.grid.coord_to_track(in_top_layer, in_xm_conn_list[0].middle, RoundMode.LESS_EQ)
            vin_top = self.connect_to_tracks(vin_stackup_dict[in_top_layer - 1],
                                             TrackID(in_top_layer, vin_top_tid,
                                                     tr_manager.get_width(in_top_layer, 'ana_sig')))
            vin_top = self.extend_wires(vin_top, lower=self.bound_box.yl, upper=self.bound_box.yh)
            self.add_pin('in', vin_top)
            if nmos_params['dum_sampler']:
                self.add_pin('in_xm1', nmos.get_pin('in_coupled'), hide=True)
                self.add_pin('in_c', nmos.get_pin('in_c'))
        else:
            tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
            vin_vm = [self.connect_to_tracks(nmos.get_pin('in_hm'), TrackID(vm_layer, tidx, tr_w_sig_vm),
                                             min_len_mode=MinLenMode.MIDDLE) for tidx in vm_avail_tidx[1:-2]]
            self.connect_to_track_wires(vin_vm, nmos.get_pin('in_xm'))

            vin_xm: List[WireArray] = nmos.get_all_port_pins('in_xm')
            in_top_layer = top_layer - 1 if self.grid.get_direction(top_layer) == Orient2D.x else top_layer
            vin_stackup_dict = self.via_stack_up(tr_manager, vin_xm, xm_layer, in_top_layer - 1, 'ana_sig')
            vin_top_hor = vin_stackup_dict[in_top_layer - 1]
            # vinc_top_hor_tidx = tr_manager.get_next_track(in_top_layer - 1, vin_top_hor.track_id.base_index,
            #                                               'ana_sig', 'ana_sig', up=-1)
            # vinc = self.add_wires(in_top_layer - 1, vinc_top_hor_tidx, vin_top_hor.lower, vin_top_hor.upper,
            #                       width=tr_manager.get_width(in_top_layer - 1, 'ana_sig'))

            in_top_tidx = self.grid.coord_to_track(in_top_layer, vin_top_hor.middle, RoundMode.LESS)
            in_top_tid = TrackID(in_top_layer, in_top_tidx, tr_manager.get_width(in_top_layer, 'ana_sig'))
            in_top = self.connect_to_tracks(vin_top_hor, in_top_tid,
                                            track_upper=self.bound_box.yh, track_lower=self.bound_box.yl)

            self.add_pin('in', in_top)
            self.add_pin('in', vin_top_hor)
            # self.add_pin('in_c', vinc)
            if not no_sampler:
                self.connect_to_track_wires(nmos.get_pin('vg_xm'), sampler_vert.get_pin('vg'))
                self.connect_to_track_wires(nmos.get_pin('in_xm'), sampler_vert.get_pin('in'))
                self.connect_to_track_wires(vin_top_hor, sampler_vert.get_pin('in_c'))
                self.connect_to_track_wires(vinc, sampler_vert.get_pin('in'))
                for pname in sampler_vert.port_names_iter():
                    if 'out' in pname:
                        self.reexport(sampler_vert.get_port(pname))
            else:
                vg_ym = self.via_up(tr_manager, nmos.get_all_port_pins('vg_xm'), xm_layer, 'ana_sig')
                self.add_pin('vg_ym', self.extend_wires(vg_ym, lower=self.bound_box.yl), label='vg')
                self.add_pin('vg_xm', nmos.get_all_port_pins('vg_xm'), label='vg')
                if nmos.has_port('off_xm'):
                    off_ym = self.via_up(tr_manager, nmos.get_all_port_pins('off_xm'), xm_layer, 'ana_sig', )
                    off_ym = self.extend_wires(off_ym, lower=vin_stackup_dict[ym_layer][0].lower)
                    self.add_pin('off_ym', off_ym, label='dummy_off')
                    off_xm1 = self.via_up(tr_manager, off_ym, ym_layer, 'ana_sig')
                    # self.add_pin('vg_ym', self.extend_wires(vg_ym, lower=self.bound_box.yl), label='vg')
                    self.add_pin('off_xm1', off_xm1, label='dummy_off')
                in_xm_conn_list = []
                # self.add_pin('vin_ym', vin_ym, label='vin')
                # self.add_pin('vinc_ym', vinc_ym, label='vinc')

        sample_xm_tidx = self.grid.coord_to_track(xm_layer, sample_vm.middle, RoundMode.NEAREST)
        sampleb_xm_tidx = self.grid.coord_to_track(xm_layer, sampleb_vm.middle, RoundMode.NEAREST)
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        sample_xm = self.connect_to_tracks(sample_vm, TrackID(xm_layer, sample_xm_tidx, tr_w_clk_xm),
                                           min_len_mode=MinLenMode.UPPER)
        sampleb_xm = self.connect_to_tracks(sampleb_vm, TrackID(xm_layer, sampleb_xm_tidx, tr_w_clk_xm),
                                            min_len_mode=MinLenMode.UPPER)
        if nmos.has_port('sam'):
            sample_xm = [sample_xm, nmos.get_pin('sam')]
        self.add_pin('sample', sample_xm, connect=True)
        self.add_pin('sample_b', sampleb_xm)

        for pname in nmos.port_names_iter():
            if 'out' in pname:
                self.reexport(nmos.get_port(pname))

        # Add pins:
        self.add_pin('cap_bot', cap_bot, show=self.show_pins)
        self.add_pin('cap_top', cap_top, show=self.show_pins)
        vm_lay_purp = self.grid.tech_info.get_lay_purp_list(vm_layer)[0]

        ####################
        # Connect supply
        ####################
        # supply vm
        vdd_vm_list = nmos.get_all_port_pins('VDD')
        vss_vm_list = nmos.get_all_port_pins('VSS')
        self.connect_to_track_wires(vdd_vm_list, nwl.get_all_port_pins('VDD'))
        self.connect_to_track_wires(vss_vm_list, nwl.get_all_port_pins('VSS'))

        # M4
        power_bbox_xh = max(nwl.bound_box.xh, nmos.bound_box.xh)
        power_bbox_xl = min(nwl.bound_box.xl, nmos.bound_box.xl)
        power_bbox_yl = nmos.bound_box.yl if vert_sampler else vin_stackup_dict[ym1_layer].bound_box.yh
        power_bbox_yh = nwl.bound_box.yh

        # Get line-end space between transistor and cap
        power_bbox_list = []
        le_sp_x, le_sp_y = 0, 0
        for idx in range(top_layer, vm_layer, -1):
            _width = tr_manager.get_width(idx, 'sup')
            if _width < 0:
                _tid = TrackID(idx, 0, _width, grid=self.grid)
                _width = _tid.width
            if self.grid.get_direction(idx) == Orient2D.y:
                le_sp_y = self.grid.get_line_end_space(idx, _width)
            else:
                le_sp_x = self.grid.get_line_end_space(idx, _width)

            if idx > ym1_layer + 1:
                power_bbox_list.append(BBox(self.bound_box.xl + le_sp_x, self.bound_box.yl,
                                            self.bound_box.xh - le_sp_x, self.bound_box.yh))
            elif cboot_master.top_layer < idx:
                power_bbox_list.append(BBox(self.bound_box.xl + le_sp_x,
                                            power_bbox_yl,
                                            self.bound_box.xh - le_sp_x, self.bound_box.yh))
            else:
                power_bbox_list.append(BBox(power_bbox_xl + le_sp_x, power_bbox_yl,
                                            power_bbox_xh - le_sp_x, power_bbox_yh - le_sp_y))

        power_bbox_list = power_bbox_list[::-1]

        vdd_dict, vss_dict = self.connect_supply_stack_warr(tr_manager, [vdd_vm_list, vss_vm_list],
                                                            vm_layer, top_layer, power_bbox_list,
                                                            extend_lower_layer=True)

        #
        for idx in vdd_dict.keys():
            self.add_pin('VDD', vdd_dict[idx], show=False)
            self.add_pin('VSS', vss_dict[idx], show=False)

        self.add_pin('VDD_top', vdd_dict[top_layer], label='VDD')
        self.add_pin('VSS_top', vss_dict[top_layer], label='VSS')

        top_hor_lay = top_layer if self.grid.get_direction(top_layer) == Orient2D.x else top_layer - 1
        self._top_hor_vdd_coord = self.grid.track_to_coord(top_hor_lay, vdd_dict[top_hor_lay][0].track_id.base_index)
        #
        dev_info = {
            **nmos_master.sch_params['dev_info'],
            **nwl_master.sch_params['dev_info']
        }
        if vert_sampler and not no_sampler:
            dev_info['XSAM'] = sampler_vert_temp.sch_params

        self._sch_params = dict(
            lch=nwl_master.sch_params['lch'],
            intent='lvt',
            dev_info=dev_info,
            cap_params=cboot_master.sch_params.copy(append=dict(cap=mom_params.get('cap', 0))),
            fast_on=fast_on,
            no_sampler=no_sampler,
            dummy_off=nmos_master.sch_params['dummy_off'],
            mid_to_vg2=True,
            # swap_inout=swap_inout,
        )
        if has_cap_aux:
            self._sch_params.update(
                dict(cap_aux_params=caux_master.sch_params.copy(append=dict(cap=mom_params.get('cap_aux', 0)))))


class BootstrapDiff(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._top_hor_vdd_coord = 0

    @property
    def top_hor_vdd_coord(self):
        return self._top_hor_vdd_coord

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'bootstrap_diff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            sampler_params='single end sampler parameters.',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def draw_layout(self) -> None:
        sampler_params: Param = self.params['sampler_params']

        sampler_p_master: Bootstrap = self.new_template(Bootstrap, params=sampler_params)
        sampler_n_master: Bootstrap = self.new_template(Bootstrap,
                                                        params=sampler_params.copy(append=dict(nside=False)))

        top_layer = sampler_p_master.top_layer
        w_blk, h_blk = self.grid.get_block_size(top_layer)
        w_samp, h_samp = sampler_p_master.bound_box.w, sampler_p_master.bound_box.h

        w_tot = -(-w_samp // w_blk) * w_blk
        h_tot = h_samp
        h_tot = -(-h_tot // h_blk) * h_blk
        w_tot = 2 * w_tot

        samp_n = self.add_instance(sampler_n_master, inst_name='XN', xform=Transform(0, 0))
        samp_p = self.add_instance(sampler_p_master, inst_name='XP',
                                   xform=Transform(w_tot, 0, mode=Orientation.MY))

        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot))
        if samp_n.has_port('in_c'):
            self.connect_wires([samp_n.get_pin('in_c'), samp_p.get_pin('in_c'),
                                samp_n.get_pin('in_xm1'), samp_p.get_pin('in_xm1')])

        sample_xm = self.connect_wires(samp_n.get_all_port_pins('sample') +
                                       samp_p.get_all_port_pins('sample'))
        sampleb_xm = self.connect_wires([samp_n.get_pin('sample_b'), samp_p.get_pin('sample_b')])
        self._top_hor_vdd_coord = sampler_p_master.top_hor_vdd_coord

        # --- export pins:
        self.reexport(samp_n.get_port('VDD'), connect=True)
        self.reexport(samp_p.get_port('VDD'), connect=True)
        self.reexport(samp_n.get_port('VSS'), connect=True)
        self.reexport(samp_p.get_port('VSS'), connect=True)
        self.add_pin('sample', sample_xm)
        self.add_pin('sample_b', sampleb_xm)

        pin_list = ['cap_top', 'cap_bot', 'vg']
        if samp_n.has_port('cap_top_aux'):
            pin_list.append('cap_top_aux')

        for pinname in samp_p.port_names_iter():
            if 'out' in pinname:
                ppin = pinname.replace('out', 'out_p')
                npin = pinname.replace('out', 'out_n')
                self.reexport(samp_n.get_port(pinname), net_name=npin)
                self.reexport(samp_p.get_port(pinname), net_name=ppin)

            for pinparttern in pin_list:
                if pinparttern == pinname:
                    self.reexport(samp_n.get_port(pinname), net_name=pinname + '_n')
                    self.reexport(samp_p.get_port(pinname), net_name=pinname + '_p')

        self.reexport(samp_n.get_port('in'), net_name='sig_n', connect=True)
        self.reexport(samp_p.get_port('in'), net_name='sig_p', connect=True)

        self._sch_params = dict(sampler_params=sampler_n_master.sch_params.copy())
