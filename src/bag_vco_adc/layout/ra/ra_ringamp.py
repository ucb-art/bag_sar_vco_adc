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


from typing import Any, Dict, Type, Optional, Tuple, Mapping, Union

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.routing import WireArray
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict
from bag_vco_adc.layout.ra.ra_cap import MOMCapOnMOS
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from pybag.enum import RoundMode, MinLenMode, PinMode
from xbase.layout.enum import MOSWireType, MOSType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import TilePatternElement, TilePattern
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID


class RingAmpDiffHalf(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ringamp_se')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_dict='Dict of width',
            stack_dict='Dict of stack number',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']
        stack_dict: Mapping[str, int] = self.params['stack_dict']
        tx_names = list(w_dict.keys())
        tx_name_row_pair = [(tx, ridx_n if tx.startswith('n') else ridx_p) for tx in tx_names]

        w_ans = {}
        th_ans = {}
        stack_ans = {}
        for name, row_idx in tx_name_row_pair:
            rinfo = self.get_row_info(row_idx, 3)
            w = w_dict.get(name, 0)
            stack = stack_dict.get(name, 3)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold
            stack_ans[name] = stack

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans), ImmutableSortedDict(stack_ans)

    def fill_tap(self, tile_idx, start_col, stop_col):
        nrow = self.get_tile_info(tile_idx)[0].num_rows
        for idx in range(nrow):
            cur_row_type = self.get_tile_info(tile_idx)[0].get_row_place_info(idx).row_info.row_type
            if cur_row_type is MOSType.ptap or cur_row_type is MOSType.nch:
                self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)
            elif cur_row_type is MOSType.ntap or cur_row_type is MOSType.pch:
                self.add_substrate_contact(idx, start_col, tile_idx=tile_idx, seg=stop_col - start_col)

    def draw_fb_dac(self, tile_idx, start_col, seg_dac_list, w_dac_list):
        if len(seg_dac_list) != len(w_dac_list):
            raise ValueError("seg list length doesn't match w list")
        num_dac = len(seg_dac_list)
        cur_col = start_col
        tx_list = []
        ctrl_hm_list = []
        ctrl_tid = self.get_track_id(0, MOSWireType.G, 'ctrl', tile_idx=tile_idx)
        sig_tid = self.get_track_id(0, MOSWireType.G, 'sig', tile_idx=tile_idx)
        for idx in range(num_dac):
            tx = self.add_nand2(0, cur_col, seg=seg_dac_list[idx], stack=1, w=w_dac_list[idx], tile_idx=tile_idx,
                                export_mid=False)
            tx_list.append(tx)
            cur_col += 2 * seg_dac_list[idx]
            ctrl_hm = self.connect_to_tracks(tx.g1, ctrl_tid, min_len_mode=MinLenMode.MIDDLE)
            ctrl_hm_list.append(ctrl_hm)
        sig_hm = self.connect_to_tracks([w for tx in tx_list for w in tx.g0], sig_tid)
        sig_tid = self.get_track_id(0, MOSWireType.DS, 'sig', tile_idx=tile_idx)
        out_hm = self.connect_to_tracks([tx.s for tx in tx_list], sig_tid)
        return sig_hm, out_hm, ctrl_hm_list, [tx.d for tx in tx_list]

    def get_nearest_avail_track(self, tr_manager, warr, layer, type):
        width = tr_manager.get_width(layer, type)
        avail_locs = self.get_available_tracks(layer, self.grid.coord_to_track(layer, warr.lower, RoundMode.LESS),
                                               self.grid.coord_to_track(layer, warr.upper, RoundMode.GREATER),
                                               warr.bound_box.yl, warr.bound_box.yh, width=width, include_last=True)
        key = lambda x: abs(self.grid.track_to_coord(layer, x) - (warr.upper + warr.lower) // 2)
        nearest = min(avail_locs, key=key)
        return TrackID(layer, nearest, width)

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)
        tr_manager = self.tr_manager
        ridx_p, ridx_n = -1, 0
        seg_dict: ImmutableSortedDict[str, Union[int, dict]] = self.params['seg_dict']
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        w_dict, th_dict, stack_dict = self._get_w_th_dict(ridx_n, ridx_p)
        seg_nen = seg_dict['nen']
        seg_pen = seg_dict['pen']
        seg_nin = seg_dict['nin']
        seg_pin = seg_dict['pin']
        seg_nbe = seg_dict['nbe']
        seg_pbe = seg_dict['pbe']
        seg_nmid = seg_dict['nmid']
        seg_pmid = seg_dict['pmid']
        seg_nout = seg_dict['nout']
        seg_pout = seg_dict['pout']
        seg_nbias = seg_dict['nbias']
        seg_pbias = seg_dict['pbias']
        seg_ntail = seg_dict['ntail']
        seg_ptail = seg_dict['ptail']
        seg_nfb = seg_dict['nfb']
        seg_pfb = seg_dict['pfb']
        seg_nchagre = seg_dict.get('ncharge', 0)
        seg_pchagre = seg_dict.get('pcharge', 0)

        w_nen = w_dict['nen']
        w_pen = w_dict['pen']
        w_nin = w_dict['nin']
        w_pin = w_dict['pin']
        w_nbe = w_dict['nbe']
        w_pbe = w_dict['pbe']
        w_nmid = w_dict['nmid']
        w_pmid = w_dict['pmid']
        w_nout = w_dict['nout']
        w_pout = w_dict['pout']
        w_nbias = w_dict['nbias']
        w_pbias = w_dict['pbias']
        w_ntail = w_dict['ntail']
        w_ptail = w_dict['ptail']
        w_ncharge = w_dict['ncharge']
        w_pcharge = w_dict['pcharge']
        w_nfb = w_dict['nfb']
        w_pfb = w_dict['pfb']

        # placement
        min_sep = self.min_sep_col
        vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig', 'sig', 'sig', 'sig', 'sig'])
        vm_rt_ncol = self.arr_info.track_to_col(vm_layer, vm_ntr, RoundMode.NEAREST)

        main_ncol = max(vm_rt_ncol, seg_nin, seg_pin, seg_nmid, seg_pmid, seg_nout, seg_pout)

        front_tile_idx, mid_tile_idx, out_tile_idx = 3, 7, 9
        # --- first stage ---
        cur_col = (main_ncol - max(seg_nin, seg_pin, seg_ntail, seg_ptail)) // 2
        cur_col += cur_col & 1
        dacn_sig_hm, dacn_out_hm, dacn_ctrl_list_hm, dacn_sup_conn = \
            self.draw_fb_dac(1, cur_col, seg_dict['nfb_dac'], self.params['w_dict']['nfb_dac'])
        dacp_sig_hm, dacp_out_hm, dacp_ctrl_list_hm, dacp_sup_conn = \
            self.draw_fb_dac(5, cur_col, seg_dict['pfb_dac'], self.params['w_dict']['pfb_dac'])
        m_nin = self.add_mos(ridx_n + 1, cur_col, seg_nin, w=w_nin, tile_idx=front_tile_idx)
        m_pin = self.add_mos(ridx_p - 1, cur_col, seg_pin, w=w_pin, tile_idx=front_tile_idx)
        m_nbe = self.add_mos(ridx_n + 1, cur_col + max(seg_nin, seg_pin) + min_sep, seg_nbe, w=w_nbe,
                             tile_idx=front_tile_idx)
        m_pbe = self.add_mos(ridx_p - 1, cur_col + max(seg_nin, seg_pin) + min_sep, seg_pbe, w=w_pbe,
                             tile_idx=front_tile_idx)

        m_ntail = self.add_mos(ridx_n, cur_col, seg_ntail, w=w_ntail, tile_idx=front_tile_idx, g_on_s=True)
        m_nfb = self.add_mos(ridx_n, cur_col + seg_ntail + min_sep, seg_nfb, w=w_nfb,
                             tile_idx=front_tile_idx, g_on_s=True)
        m_ptail = self.add_mos(ridx_p, cur_col, seg_ptail, w=w_ptail, tile_idx=front_tile_idx, g_on_s=True)
        m_pfb = self.add_mos(ridx_p, cur_col + seg_ptail + min_sep, seg_pfb, w=w_pfb,
                             tile_idx=front_tile_idx, g_on_s=True)

        first_stage_ncol = cur_col + max(seg_ntail + min_sep + seg_nfb, seg_ptail, seg_nin + min_sep + seg_nbe)

        en_col = cur_col + seg_ntail + min_sep + seg_nfb
        in_mid_col = cur_col + max(seg_nin, seg_pin) // 2
        cur_col = main_ncol + min_sep
        cur_col += cur_col & 1

        # --- second stage ---
        # cur_col = abs((max(seg_nmid, seg_pmid) - main_ncol)) // 2
        cur_col = (main_ncol - max(seg_nin, seg_pin, seg_ntail, seg_ptail)) // 2
        cur_col += cur_col & 1
        m_nmid = self.add_mos(ridx_n, cur_col, seg_nmid, w=w_nmid, tile_idx=mid_tile_idx)
        m_pmid = self.add_mos(ridx_p, cur_col, seg_pmid, w=w_pmid, tile_idx=mid_tile_idx)
        cur_col += max(seg_nmid, seg_pmid) + min_sep
        cur_col += cur_col & 1

        m_ndz = self.add_mos(ridx_n, cur_col, seg_nbias, w=w_nbias, tile_idx=mid_tile_idx)
        m_pdz = self.add_mos(ridx_p, cur_col, seg_pbias, w=w_pbias, tile_idx=mid_tile_idx)
        ctrl_mid_col = cur_col + max(seg_nbias, seg_pbias) // 2
        cur_col += max(seg_nbias, seg_pbias) + min_sep

        cur_col = max(cur_col, en_col)
        m_nen0 = self.add_mos(ridx_n, cur_col, seg_nen, w=w_nen, tile_idx=mid_tile_idx, g_on_s=True)
        m_pen0 = self.add_mos(ridx_p, cur_col, seg_pen, w=w_pen, tile_idx=mid_tile_idx, g_on_s=True)
        cur_col += max(seg_nen, seg_pen) + min_sep

        m_nen1 = self.add_mos(ridx_n, cur_col, seg_nen, w=w_nen, tile_idx=mid_tile_idx, g_on_s=True)
        m_pen1 = self.add_mos(ridx_p, cur_col, seg_pen, w=w_pen, tile_idx=mid_tile_idx, g_on_s=True)
        cur_col += max(seg_nen, seg_pen) + min_sep

        if seg_pchagre:
            m_pcharge = self.add_mos(ridx_p, cur_col, seg_pchagre, w=w_pcharge, tile_idx=mid_tile_idx, g_on_s=True)
            # cur_col += seg_pchagre + min_sep
        else:
            m_pcharge = None

        if seg_nchagre:
            m_ncharge = self.add_mos(ridx_n, cur_col, seg_nchagre, w=w_ncharge, tile_idx=mid_tile_idx, g_on_s=True)
            # cur_col += seg_nchagre + min_sep
        else:
            m_ncharge = None

        # --- third stage ---
        # cur_col = abs((max(seg_nout, seg_pout) - main_ncol)) // 2
        cur_col = (main_ncol - max(seg_nin, seg_pin, seg_ntail, seg_ptail)) // 2
        cur_col += cur_col & 1
        m_nout = self.add_mos(ridx_n, cur_col, seg_nout, w=w_nout, tile_idx=out_tile_idx, stack=stack_dict['nout'])
        m_pout = self.add_mos(ridx_p, cur_col, seg_pout, w=w_pout, tile_idx=out_tile_idx, stack=stack_dict['pout'])
        out_mid_col = cur_col + max(seg_pout, seg_nout) // 2
        cur_col += max(seg_nen, seg_pen) + min_sep
        ncol_tot = self.num_cols
        ptap0 = self.add_substrate_contact(0, 0, seg=ncol_tot + min_sep, tile_idx=0)
        ptap1 = self.add_substrate_contact(0, 0, seg=ncol_tot + min_sep, tile_idx=2)
        ntap0 = self.add_substrate_contact(0, 0, seg=ncol_tot + min_sep, tile_idx=4)
        ntap1 = self.add_substrate_contact(0, 0, seg=ncol_tot + min_sep, tile_idx=6)
        ptap2 = self.add_substrate_contact(0, 0, seg=ncol_tot + min_sep, tile_idx=8)
        ntap2 = self.add_substrate_contact(0, 0, seg=ncol_tot + min_sep, tile_idx=10)
        self.set_mos_size()

        # === Connections
        vm_start_col = (main_ncol - max(seg_nin, seg_pin, seg_ntail, seg_ptail)) // 2
        vm_start_col += vm_start_col & 1
        vm_start_tidx = self.arr_info.col_to_track(vm_layer, vm_start_col, RoundMode.NEAREST)
        tr_w_vm_in = tr_manager.get_width(vm_layer, 'in')
        tr_w_vm_sig = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_sig_dum_sep = self.get_track_sep(vm_layer, tr_w_vm_in, 1)
        tr_sp_sig_sep = self.get_track_sep(vm_layer, tr_w_vm_in, tr_w_vm_in)

        in_vm_tidx = vm_start_tidx + tr_sp_sig_dum_sep
        dac_vm_tidx = vm_start_tidx + self.get_track_sep(vm_layer, tr_w_vm_sig, 1)
        midn_vm_tidx = in_vm_tidx + self.get_track_sep(vm_layer, tr_w_vm_in, tr_w_vm_sig)
        midp_vm_tidx = midn_vm_tidx + self.get_track_sep(vm_layer, tr_w_vm_sig, tr_w_vm_sig)
        ctrln_vm_tidx = midn_vm_tidx - self.get_track_sep(vm_layer, tr_w_vm_sig, tr_w_vm_sig)
        ctrlp_vm_tidx = ctrln_vm_tidx - self.get_track_sep(vm_layer, tr_w_vm_sig, tr_w_vm_sig)
        out_vm_tidx = ctrln_vm_tidx + self.get_track_sep(vm_layer, tr_w_vm_in, tr_w_vm_sig)

        front_mid_vm_col_start = vm_start_col + max(seg_ntail, seg_ptail) + min_sep
        front_mid_vm_tidx_start = self.arr_info.col_to_track(vm_layer, front_mid_vm_col_start, RoundMode.GREATER_EQ)
        front_mid_vm_tidx_start = max(front_mid_vm_tidx_start,
                                      midp_vm_tidx + self.get_track_sep(vm_layer, tr_w_vm_sig, tr_w_vm_sig))
        midn_front_vm_tidx = front_mid_vm_tidx_start
        midp_front_vm_tidx = midn_front_vm_tidx + self.get_track_sep(vm_layer, tr_w_vm_sig, tr_w_vm_sig)

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')

        # --- First stage
        nds_tid0 = self.get_track_id(ridx_n + 1, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=front_tile_idx)
        nds_tid1 = self.get_track_id(ridx_n + 1, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=front_tile_idx)
        pds_tid0 = self.get_track_id(ridx_p - 1, MOSWireType.DS, wire_name='sig', wire_idx=-1, tile_idx=front_tile_idx)
        pds_tid1 = self.get_track_id(ridx_p - 1, MOSWireType.DS, wire_name='sig', wire_idx=-2, tile_idx=front_tile_idx)

        ng_tid1 = self.get_track_id(ridx_n + 1, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=front_tile_idx)
        pg_tid1 = self.get_track_id(ridx_p - 1, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=front_tile_idx)

        midn = self.connect_to_tracks([m_pin.s], pds_tid0)
        midp = self.connect_to_tracks([m_nin.s], nds_tid0)
        midn_d = self.connect_to_tracks([m_nbe.d], nds_tid0)
        midp_d = self.connect_to_tracks([m_pbe.d], pds_tid0)
        midn_s = self.connect_to_tracks([m_nbe.s], nds_tid1)
        midp_s = self.connect_to_tracks([m_pbe.s], pds_tid1)
        en0 = self.connect_to_tracks(m_nbe.g, ng_tid1)
        enb0 = self.connect_to_tracks(m_pbe.g, pg_tid1)

        # input
        in_hm = [self.connect_to_tracks([m_nin.g, m_pin.g], ng_tid1),
                 self.connect_to_tracks([m_nin.g, m_pin.g], pg_tid1)]
        in_vm = self.connect_to_tracks(in_hm, TrackID(vm_layer, in_vm_tidx, tr_w_vm_in, grid=self.grid))

        nds_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=front_tile_idx)
        pds_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1, tile_idx=front_tile_idx)

        ng_tid0 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=front_tile_idx)
        pg_tid0 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=front_tile_idx)

        vmcfb_bot_hm = self.connect_to_tracks(m_ntail.g, ng_tid0)
        nfb_hm = self.connect_to_tracks(m_nfb.g, ng_tid0)
        pfb_hm = self.connect_to_tracks(m_pfb.g, pg_tid0)
        vcmfb_top_hm = self.connect_to_tracks(m_ptail.g, pg_tid0)
        vbot_hm = self.connect_to_tracks([m_ntail.d, m_nfb.d, m_nin.d], nds_tid0)
        vtop_hm = self.connect_to_tracks([m_ptail.d, m_pfb.d, m_pin.d], pds_tid0)

        # --- Second stage
        nds_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=mid_tile_idx)
        nds_tid1 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=mid_tile_idx)
        pds_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1, tile_idx=mid_tile_idx)
        pds_tid1 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2, tile_idx=mid_tile_idx)

        ng_tid1 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=mid_tile_idx)
        pg_tid1 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=mid_tile_idx)
        ng_tid0 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='bias', wire_idx=0, tile_idx=mid_tile_idx)
        pg_tid0 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='bias', wire_idx=0, tile_idx=mid_tile_idx)

        ctrlp = self.connect_to_tracks([m_pmid.d, m_pdz.d], pds_tid0)
        ctrln = self.connect_to_tracks([m_nmid.d, m_ndz.d], nds_tid0)
        ctrlp_d = self.connect_to_tracks([m_ndz.s], nds_tid1)
        ctrln_d = self.connect_to_tracks([m_pdz.s], pds_tid1)

        midn_g = self.connect_to_tracks(m_nmid.g, ng_tid1)
        midp_g = self.connect_to_tracks(m_pmid.g, pg_tid1)

        en1 = self.connect_to_tracks([m_nen1.g, m_pcharge.g], pg_tid1)
        enb1 = self.connect_to_tracks([m_pen0.g, m_ncharge.g], ng_tid1)

        biasn_hm = self.connect_to_tracks([m_pen0.s, m_pcharge.d], pds_tid1)
        biasp_hm = self.connect_to_tracks([m_nen1.s, m_ncharge.d], nds_tid1)
        en_vm_start_tdix = self.arr_info.col_to_track(vm_layer, max(seg_nin, seg_pin) + min_sep, RoundMode.GREATER_EQ)
        en1_vm_tidx = tr_manager.get_next_track(vm_layer, en_vm_start_tdix, 'sig', 'sig', up=0)
        enb1_vm_tidx = tr_manager.get_next_track(vm_layer, en1_vm_tidx, 'sig', 'sig', up=1)
        biasn_vm_tidx = tr_manager.get_next_track(vm_layer, en_vm_start_tdix, 'sig', 'sig', up=2)
        biasp_vm_tidx = tr_manager.get_next_track(vm_layer, en_vm_start_tdix, 'sig', 'sig', up=3)

        biasn_vm, biasp_vm = self.connect_differential_tracks(biasn_hm, biasp_hm, vm_layer, biasn_vm_tidx,
                                                              biasp_vm_tidx, width=tr_w_sig_vm)
        _, bias_xm_locs = tr_manager.place_wires(xm_layer, ['bias'] * 2, center_coord=biasn_vm.middle)
        tr_w_bias_xm = tr_manager.get_width(xm_layer, 'bias')
        biasn_xm = self.connect_to_tracks(biasn_vm, TrackID(xm_layer, bias_xm_locs[0], tr_w_bias_xm),
                                          min_len_mode=MinLenMode.MIDDLE)
        biasp_xm = self.connect_to_tracks(biasp_vm, TrackID(xm_layer, bias_xm_locs[1], tr_w_bias_xm),
                                          min_len_mode=MinLenMode.MIDDLE)

        _ndz_bias = self.connect_to_tracks([m_ndz.g, m_nen0.d, m_pen0.d], ng_tid0)
        _pdz_bias = self.connect_to_tracks([m_pdz.g, m_nen1.d, m_pen1.d], pg_tid0)

        # --- Third stage
        nds_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='out', wire_idx=0, tile_idx=out_tile_idx)
        pds_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='out', wire_idx=-1, tile_idx=out_tile_idx)

        ng_tid1 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='out', wire_idx=0, tile_idx=out_tile_idx)
        pg_tid1 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='out', wire_idx=0, tile_idx=out_tile_idx)
        ctrln_g = self.connect_to_tracks(m_nout.g, ng_tid1)
        ctrlp_g = self.connect_to_tracks(m_pout.g, pg_tid1)

        nout = self.connect_to_tracks(m_nout.d, nds_tid0)
        pout = self.connect_to_tracks(m_pout.d, pds_tid0)

        # output
        tr_w_vm_out = tr_manager.get_width(vm_layer, 'out')
        out_vm = self.connect_to_tracks([nout, pout], TrackID(vm_layer, out_vm_tidx, tr_w_vm_out, grid=self.grid),
                                        track_upper=self.bound_box.yh)

        # mid
        _, ctrl_vm_tidx = tr_manager.place_wires(vm_layer, ['sig', 'sig'],
                                                 center_coord=self.arr_info.col_to_coord(ctrl_mid_col))
        # --- control
        out_tile_pinfo = self.get_tile_info(-2)
        en_vm_upper = out_tile_pinfo[1] + out_tile_pinfo[0].height // 2
        en_vm, enb_vm = self.connect_matching_tracks([en1, enb1], vm_layer, [en1_vm_tidx, enb1_vm_tidx],
                                                     width=tr_manager.get_width(vm_layer, 'ctrl'),
                                                     track_upper=en_vm_upper)
        self.connect_differential_wires(en_vm, enb_vm, en0, enb0)

        midn_vm, midp_vm = self.connect_differential_tracks([midn], [midp], vm_layer,
                                                            midn_vm_tidx, midp_vm_tidx, width=tr_w_sig_vm)
        self.connect_to_tracks([midn, midn_s, midp_d, nfb_hm], TrackID(vm_layer, midn_front_vm_tidx, tr_w_sig_vm))
        self.connect_to_tracks([midp, midp_s, midn_d, pfb_hm], TrackID(vm_layer, midp_front_vm_tidx, tr_w_sig_vm))
        self.connect_differential_wires(midn_vm, midp_vm, midn_g, midp_g)
        ctrln_vm, ctrlp_vm = self.connect_differential_tracks([ctrln, ctrln_d], [ctrlp, ctrlp_d], vm_layer,
                                                              ctrln_vm_tidx, ctrlp_vm_tidx, width=tr_w_sig_vm)
        self.connect_differential_wires(ctrln_vm, ctrlp_vm, ctrln_g, ctrlp_g)

        vcmfb_tidx = self.arr_info.col_to_track(vm_layer, -1)
        vcmfb = self.connect_to_tracks([vmcfb_bot_hm, vcmfb_top_hm], TrackID(vm_layer, vcmfb_tidx, tr_w_sig_vm))

        # DAC connection
        self.connect_to_tracks([vbot_hm, dacn_out_hm], TrackID(vm_layer, dac_vm_tidx, tr_w_sig_vm))
        self.connect_to_tracks([vtop_hm, dacp_out_hm], TrackID(vm_layer, dac_vm_tidx, tr_w_sig_vm))
        self.connect_to_track_wires(midn_vm, dacn_sig_hm)
        self.connect_to_track_wires(midp_vm, dacp_sig_hm)
        dacn_ctrl_list_vm, dacp_ctrl_list_vm = [], []
        for ctrl_hm in dacn_ctrl_list_hm:
            tid = self.get_nearest_avail_track(tr_manager, ctrl_hm, vm_layer, 'ctrl_dac')
            ctrl_vm = self.connect_to_tracks(ctrl_hm, tid, min_len_mode=MinLenMode.MIDDLE)
            dacn_ctrl_list_vm.append(ctrl_vm)
        for ctrl_hm in dacp_ctrl_list_hm:
            tid = self.get_nearest_avail_track(tr_manager, ctrl_hm, vm_layer, 'ctrl_dac')
            ctrl_vm = self.connect_to_tracks(ctrl_hm, tid, min_len_mode=MinLenMode.MIDDLE)
            dacp_ctrl_list_vm.append(ctrl_vm)
        _, dacn_xm_tidx_locs = tr_manager.place_wires(xm_layer, ['ctrl_dac'] * len(dacn_ctrl_list_vm),
                                                      center_coord=dacn_ctrl_list_vm[0].middle)
        _, dacp_xm_tidx_locs = tr_manager.place_wires(xm_layer, ['ctrl_dac'] * len(dacp_ctrl_list_vm),
                                                      center_coord=dacp_ctrl_list_vm[0].middle)

        dacn_xm_list = self.connect_matching_tracks(dacn_ctrl_list_vm, xm_layer, dacn_xm_tidx_locs,
                                                    width=tr_manager.get_width(xm_layer, 'ctrl_dac'))
        dacp_xm_list = self.connect_matching_tracks(dacp_ctrl_list_vm, xm_layer, dacp_xm_tidx_locs,
                                                    width=tr_manager.get_width(xm_layer, 'ctrl_dac'))
        _, dac_ym_tidx_locs = \
            tr_manager.place_wires(ym_layer, ['ctrl_dac'] * (len(dacn_xm_list) + len(dacp_xm_list)),
                                   align_idx=-1, align_track=self.grid.coord_to_track(ym_layer, self.bound_box.xh))

        dac_ym_list = self.connect_matching_tracks(dacn_xm_list + dacp_xm_list, ym_layer,
                                                   dac_ym_tidx_locs, width=tr_manager.get_width(ym_layer, 'ctrl_dac'))

        # Input
        self.connect_wires([m_nin.g, m_pin.g])
        self.connect_wires([m_nen0.g, m_pen0.g])
        self.connect_wires([m_nen1.g, m_pen1.g])

        # Connect supply
        vss_hm_list, vdd_hm_list = [], []

        vss_conn0 = [ptap0] + dacn_sup_conn
        vss_conn1 = [ptap1, m_ntail.s, m_nfb.s]
        vdd_conn0 = [ntap0, m_ptail.s, m_pfb.s]
        vdd_conn1 = [ntap1, m_pmid.s, m_pen1.s] + dacp_sup_conn
        if seg_pchagre:
            vdd_conn1.append(m_pcharge.s)
        vdd_tid0 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=4)
        vdd_tid1 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=6)
        vss_tid0 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=0)
        vss_tid1 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=2)
        vdd_hm_list.append(self.connect_to_tracks(vdd_conn0, vdd_tid0))
        vdd_hm_list.append(self.connect_to_tracks(vdd_conn1, vdd_tid1))
        vss_hm_list.append(self.connect_to_tracks(vss_conn0, vss_tid0))
        vss_hm_list.append(self.connect_to_tracks(vss_conn1, vss_tid1))
        vss_conn1 = [ptap2, m_nmid.s, m_nen0.s, m_nout.s]
        if seg_nchagre:
            vss_conn1.append(m_ncharge.s)
        vss_tid1 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=8)
        vss_hm_list.append(self.connect_to_tracks(vss_conn1, vss_tid1))
        vdd_conn2 = [ntap2, m_pout.s]
        vdd_tid2 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=10)
        vdd_hm_list.append(self.connect_to_tracks(vdd_conn2, vdd_tid2))
        # for idx in range(self.num_tile_rows):
        #     for jdx in range(self.get_tile_pinfo(idx).num_rows):
        #         fill_conn_layer_intv(self, idx, jdx, extend_to_gate=False)

        # --- export suplies ---
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_sup_vm_sp = tr_manager.get_sep(vm_layer, ('sup', 'sup'))

        def export_hm_sup(hm_sup: WireArray):
            yloc = (hm_sup.bound_box.yl + hm_sup.bound_box.yh) // 2
            vm_locs = \
                self.get_available_tracks(vm_layer,
                                          self.grid.coord_to_track(vm_layer, hm_sup.bound_box.xl, RoundMode.NEAREST),
                                          self.grid.coord_to_track(vm_layer, hm_sup.bound_box.xh, RoundMode.NEAREST),
                                          hm_sup.bound_box.yl, hm_sup.bound_box.yh, tr_sup_vm_w,
                                          tr_sup_vm_sp)[1:]
            vm_sup_list = [self.connect_to_tracks(hm_sup, TrackID(vm_layer, tidx, tr_sup_vm_w)) for tidx in vm_locs]
            xm_sup_tidx = self.grid.coord_to_track(xm_layer, yloc, RoundMode.NEAREST)
            tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
            xm_sup = self.connect_to_tracks(vm_sup_list, TrackID(xm_layer, xm_sup_tidx, tr_sup_xm_w),
                                            track_lower=hm_sup.lower, track_upper=hm_sup.upper)
            return xm_sup

        vss_xm_list, vdd_xm_list = [], []
        for vss_warr in vss_hm_list:
            vss_xm_list.append(export_hm_sup(vss_warr))
        for vdd_warr in vdd_hm_list:
            vdd_xm_list.append(export_hm_sup(vdd_warr))

        for idx, dac_ym in enumerate(dac_ym_list):
            num_dacn = len(dacn_xm_list)
            num_dacp = len(dacp_xm_list)
            if idx < num_dacn:
                self.add_pin(f'ctrl_dac_n<{idx}>', dac_ym, mode=PinMode.LOWER)
            else:
                self.add_pin(f'ctrl_dac_p<{idx - num_dacp}>', dac_ym, mode=PinMode.LOWER)

        self.add_pin('ctrl_dac_xm', dacn_xm_list + dacp_xm_list, hide=True)

        self.add_pin('VSS_hm', vss_hm_list, hide=True)
        self.add_pin('VDD_hm', vdd_hm_list, hide=True)
        self.add_pin('VSS', vss_xm_list, connect=True)
        self.add_pin('VDD', vdd_xm_list, connect=True)
        self.add_pin('in', in_vm)
        self.add_pin('out', out_vm)
        self.add_pin('biasn', biasn_xm)
        self.add_pin('biasp', biasp_xm)
        self.add_pin('en', en_vm, connect=True)
        self.add_pin('enb', enb_vm, connect=True)
        self.add_pin('ctrln', ctrln)
        self.add_pin('ctrlp', ctrlp)
        self.add_pin('vbot', vbot_hm)
        self.add_pin('vtop', vtop_hm)
        self.add_pin('vcmfb', vcmfb)

        # routing
        self._sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            stack_dict=stack_dict,
            dac_params=dict(seg_dacn_list=seg_dict['nfb_dac'],
                            seg_dacp_list=seg_dict['pfb_dac'],
                            w_dacn_list=self.params['w_dict']['nfb_dac'],
                            w_dacp_list=self.params['w_dict']['pfb_dac'],
                            th_dacn=self.get_row_info(0, 3).threshold,
                            th_dacp=self.get_row_info(-1, 3).threshold)
        )


class RingAmpVertDiff(MOSBase, TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_diff_core')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = RingAmpDiffHalf.get_params_info()
        ans['ncols_tot'] = 'Total number of cols'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = RingAmpDiffHalf.get_default_param_values()
        ans['ncols_tot'] = 0
        return ans

    def draw_layout(self):
        master: RingAmpDiffHalf = self.new_template(RingAmpDiffHalf, params=self.params)
        self.draw_base(master.draw_base_info)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        tr_manager = self.tr_manager

        # placement
        ncols_tot = self.params['ncols_tot']
        nsep = self.min_sep_col
        nsep += (nsep & 1)

        nhalf = max(master.num_cols, (ncols_tot - nsep) // 2)

        corel = self.add_tile(master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(master, 0, nhalf + nsep)
        self.set_mos_size(num_cols=nsep + 2 * nhalf)

        self.connect_wires([corel.get_pin('vbot'), corer.get_pin('vbot')])
        self.connect_wires([corel.get_pin('vtop'), corer.get_pin('vtop')])
        biasn = self.connect_wires([corel.get_pin('biasn'), corer.get_pin('biasn')])
        biasp = self.connect_wires([corel.get_pin('biasp'), corer.get_pin('biasp')])

        self.connect_wires(corel.get_all_port_pins('VDD_hm') + corer.get_all_port_pins('VDD_hm'))

        self.add_pin('VSS', self.connect_wires(corel.get_all_port_pins('VSS') +
                                               corer.get_all_port_pins('VSS')))
        self.add_pin('VDD', self.connect_wires(corel.get_all_port_pins('VDD') +
                                               corer.get_all_port_pins('VDD')))

        self.reexport(corel.get_port('out'), net_name='out_n')
        self.reexport(corer.get_port('out'), net_name='out_p')

        bias_mid_ym_tidx = self.grid.coord_to_track(ym_layer, biasn[0].middle, RoundMode.NEAREST)
        bias_ym_tidx = [bias_mid_ym_tidx - self.get_track_sep(ym_layer, tr_manager.get_width(ym_layer, 'bias'), 1),
                        bias_mid_ym_tidx + self.get_track_sep(ym_layer, tr_manager.get_width(ym_layer, 'bias'), 1)]
        biasn, biasp = self.connect_matching_tracks([biasn, biasp], ym_layer, bias_ym_tidx,
                                                    width=tr_manager.get_width(ym_layer, 'bias'),
                                                    min_len_mode=MinLenMode.MIDDLE)
        bias_xm1_tidx = tr_manager.place_wires(xm1_layer, ['bias'] * 2, center_coord=biasn.middle)[1]
        biasn, biasp = self.connect_matching_tracks([biasn, biasp], xm1_layer, bias_xm1_tidx,
                                                    width=tr_manager.get_width(xm1_layer, 'bias'),
                                                    min_len_mode=MinLenMode.MIDDLE)

        # Connect clk, input, bias to higher layer
        hm_layer = master.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        tr_manager = self.tr_manager

        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        tr_w_ctrl_xm = tr_manager.get_width(xm_layer, 'ctrl')

        in_xm_tidx = self.grid.coord_to_track(xm_layer, corel.get_pin('in').upper, RoundMode.GREATER_EQ)
        in_xm_tidx = tr_manager.get_next_track(xm_layer, in_xm_tidx, 'dum', 'sig', up=False)
        in_n_xm = self.connect_to_tracks(corer.get_all_port_pins('in'), TrackID(xm_layer, in_xm_tidx, tr_w_sig_xm))
        in_p_xm = self.connect_to_tracks(corel.get_all_port_pins('in'), TrackID(xm_layer, in_xm_tidx, tr_w_sig_xm))

        in_n_ym_tidx = self.grid.coord_to_track(ym_layer, in_n_xm.middle, RoundMode.GREATER_EQ)
        in_p_ym_tidx = self.grid.coord_to_track(ym_layer, in_p_xm.middle, RoundMode.LESS_EQ)
        in_n_ym = [self.connect_to_tracks(in_n_xm, TrackID(ym_layer, in_n_ym_tidx, tr_w_sig_ym, grid=self.grid),
                                          min_len_mode=MinLenMode.MIDDLE)]
        in_p_ym = [self.connect_to_tracks(in_p_xm, TrackID(ym_layer, in_p_ym_tidx, tr_w_sig_ym, grid=self.grid),
                                          min_len_mode=MinLenMode.MIDDLE)]

        vcmfb_xm_tidx = tr_manager.get_next_track(xm_layer, in_xm_tidx, 'sig', 'sig')
        vcmfb_xm = self.connect_to_tracks([corel.get_pin('vcmfb'), corer.get_pin('vcmfb')],
                                          TrackID(xm_layer, vcmfb_xm_tidx, tr_w_sig_xm))
        vcmfb_ym_tidx = self.grid.coord_to_track(ym_layer, vcmfb_xm.middle, RoundMode.NEAREST)
        vcmfb_ym = self.connect_to_tracks(vcmfb_xm, TrackID(ym_layer, vcmfb_ym_tidx, tr_w_sig_ym, grid=self.grid))

        xm1_layer = ym_layer + 1
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        vcmfb_xm1_tidx = self.grid.coord_to_track(xm1_layer, vcmfb_ym.middle, RoundMode.NEAREST)
        vcmfb_xm1 = self.connect_to_tracks(vcmfb_ym, TrackID(xm1_layer, vcmfb_xm1_tidx, tr_w_sig_xm1, grid=self.grid))

        # amp and ampb signal
        _, ra_enable_xm_locs = tr_manager.place_wires(xm_layer, ['ctrl'] * 2, center_coord=corer.get_pin('en').upper)
        ra_amp_xm = self.connect_to_tracks([corel.get_pin('en'), corer.get_pin('en')],
                                           TrackID(xm_layer, ra_enable_xm_locs[0], tr_w_ctrl_xm))
        ra_ampb_xm = self.connect_to_tracks([corel.get_pin('enb'), corer.get_pin('enb')],
                                            TrackID(xm_layer, ra_enable_xm_locs[1], tr_w_ctrl_xm))

        for idx in range(self.num_tile_rows):
            for jdx in range(self.get_tile_pinfo(idx).num_rows):
                fill_conn_layer_intv(self, idx, jdx, extend_to_gate=False)

        for pin in corel.port_names_iter():
            if 'ctrl_dac' in pin:
                self.reexport(corel.get_port(pin))
                self.reexport(corer.get_port(pin))

        self.connect_wires(corel.get_all_port_pins('ctrl_dac_xm') + corer.get_all_port_pins('ctrl_dac_xm'))

        xm1_layer = ym_layer + 1
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        in_xm1_tidx = self.grid.coord_to_track(xm_layer, in_p_ym[0].middle, RoundMode.LESS)
        in_n_xm1 = self.connect_to_tracks(in_n_ym, TrackID(xm1_layer, in_xm1_tidx, tr_w_sig_xm1,
                                                           grid=self.grid))
        in_p_xm1 = self.connect_to_tracks(in_p_ym, TrackID(xm1_layer, in_xm1_tidx, tr_w_sig_xm1,
                                                           grid=self.grid))

        ym1_layer = xm1_layer + 1
        tr_w_sig_ym1 = tr_manager.get_width(ym1_layer, 'sig')
        fb_in_tidx_sp = self.get_track_sep(ym1_layer, 1, tr_w_sig_ym1).div2()
        fb_in_n_ym1_tidx = self.grid.coord_to_track(ym1_layer, in_n_xm1.middle, RoundMode.NEAREST)
        fb_in_p_ym1_tidx = self.grid.coord_to_track(ym1_layer, in_p_xm1.middle, RoundMode.NEAREST)
        in_n_ym1 = self.connect_to_tracks(in_n_xm1, TrackID(ym1_layer, fb_in_n_ym1_tidx+fb_in_tidx_sp, tr_w_sig_ym1))
        in_p_ym1 = self.connect_to_tracks(in_p_xm1, TrackID(ym1_layer, fb_in_p_ym1_tidx-fb_in_tidx_sp, tr_w_sig_ym1))

        self.add_pin('in_p', in_p_ym1)
        self.add_pin('in_n', in_n_ym1)
        self.add_pin('en', ra_amp_xm)
        self.add_pin('enb', ra_ampb_xm)
        self.add_pin('v_cmfb', vcmfb_xm1)

        self.add_pin('v_biasn', biasn)
        self.add_pin('v_biasp', biasp)
        self._sch_params = dict(ra_params=master.sch_params)


class RingAmpDCCMFB(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_cmfb_dc')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_dict='Dict of width',
            cap_params=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']
        tx_names = list(w_dict.keys())
        tx_name_row_pair = [(tx, ridx_n if tx.startswith('n') else ridx_p) for tx in tx_names]

        w_ans = {}
        th_ans = {}
        for name, row_idx in tx_name_row_pair:
            rinfo = self.get_row_info(row_idx, 1)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        cap_params: ImmutableSortedDict[str, int] = self.params['cap_params']
        # Setup tile info
        num_tiles = cap_params['ntiles']
        tile_ele = [TilePatternElement(pinfo[1]['ptap_tile']),
                    TilePatternElement(pinfo[1]['amp_tile']),
                    TilePatternElement(pinfo[1]['ntap_tile'])]

        for idx in range(num_tiles):
            tile_ele.append(TilePatternElement(pinfo[1]['cap_tile']))

        tile_ele = TilePatternElement(TilePattern(tile_ele))
        self.draw_base((tile_ele, pinfo[1]))
        tr_manager = self.tr_manager
        ridx_p, ridx_n = -1, 0
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        sig_vm_w = self.tr_manager.get_width(vm_layer, 'sig')

        w_dict, th_dict = self._get_w_th_dict(ridx_n, ridx_p)
        seg_azn = seg_dict['az_n']
        seg_azp = seg_dict['az_p']
        seg_ampn = seg_dict['amp_n']
        seg_ampp = seg_dict['amp_p']
        seg_sharen = seg_dict['share_n']
        seg_sharep = seg_dict['share_p']
        seg_chargen = seg_dict['charge_n']
        seg_chargep = seg_dict['charge_p']
        seg_invn0 = seg_dict['inv_n0']
        seg_invn1 = seg_dict['inv_n1']
        seg_invp0 = seg_dict['inv_p0']
        seg_invp1 = seg_dict['inv_p1']

        w_azn = w_dict['az_n']
        w_azp = w_dict['az_p']
        w_ampn = w_dict['amp_n']
        w_ampp = w_dict['amp_p']
        w_sharen = w_dict['share_n']
        w_sharep = w_dict['share_p']
        w_chargen = w_dict['charge_n']
        w_chargep = w_dict['charge_p']
        w_invn0 = w_dict['inv_n0']
        w_invn1 = w_dict['inv_n1']
        w_invp0 = w_dict['inv_p0']
        w_invp1 = w_dict['inv_p1']

        # placement
        min_sep = self.min_sep_col
        pinfo = self.draw_base_info[0]
        num_tiles = pinfo.num_tiles

        vm_ctrl_ntr, vm_ctrl_locs = tr_manager.place_wires(vm_layer, ['ctrl'] * 3)

        # --- first stage ---
        vm_col_list = []
        cur_col = self.arr_info.track_to_col(vm_layer, vm_ctrl_ntr)
        m_naz = self.add_mos(ridx_n, cur_col, seg_azn, w=w_azn, tile_idx=1)
        m_paz = self.add_mos(ridx_p, cur_col, seg_azp, w=w_azp, tile_idx=1)
        vm_col_list.append(cur_col + seg_azn // 2)

        cur_col += max(seg_azn, seg_azp) + min_sep
        cur_col += cur_col & 1

        m_nin0 = self.add_mos(ridx_n, cur_col, seg_invn0, w=w_invn0, tile_idx=1)
        m_pin0 = self.add_mos(ridx_p, cur_col, seg_invp0, w=w_invp0, tile_idx=1)
        vm_col_list.append(cur_col + seg_invn0 // 2)

        cur_col += max(seg_invn0, seg_invp0)
        cur_col += cur_col & 1

        m_nin1 = self.add_mos(ridx_n, cur_col, seg_invn1, w=w_invn1, tile_idx=1)
        m_pin1 = self.add_mos(ridx_p, cur_col, seg_invp1, w=w_invp1, tile_idx=1)
        vm_col_list.append(cur_col + seg_invn1 // 2)

        cur_col += max(seg_invn1, seg_invp1) + min_sep
        cur_col += cur_col & 1

        m_ampn = self.add_mos(ridx_n, cur_col, seg_ampn, w=w_ampn, tile_idx=1)
        m_ampp = self.add_mos(ridx_p, cur_col, seg_ampp, w=w_ampp, tile_idx=1)
        vm_col_list.append(cur_col + seg_ampn // 2)

        cur_col += max(seg_ampn, seg_ampp) + min_sep
        cur_col += cur_col & 1

        m_sharen = self.add_mos(ridx_n, cur_col, seg_sharen, w=w_sharen, tile_idx=1)
        m_sharep = self.add_mos(ridx_p, cur_col, seg_sharep, w=w_sharep, tile_idx=1)
        vm_col_list.append(cur_col + seg_sharen // 2)

        cur_col += max(seg_sharen, seg_sharep) + min_sep
        cur_col += cur_col & 1

        m_chargen = self.add_mos(ridx_n, cur_col, seg_chargen, w=w_chargen, tile_idx=1)
        m_chargep = self.add_mos(ridx_p, cur_col, seg_chargep, w=w_chargep, tile_idx=1)
        vm_col_list.append(cur_col + seg_chargen // 2)

        cur_col += max(seg_chargen, seg_chargep) + min_sep
        cur_col += cur_col & 1

        cap_pinfo = self.get_draw_base_sub_pattern(3, num_tiles)
        cap_ncol = 2*self.arr_info.track_to_col(vm_layer, vm_ctrl_ntr) + self.num_cols
        cap_gen_params = cap_params.copy(append=dict(pinfo=cap_pinfo, ncol_tot=cap_ncol))

        cap_master: MOSBase = self.new_template(MOMCapOnMOS, params=cap_gen_params)
        cap_col = cap_master.num_cols
        cap = self.add_tile(cap_master, col_idx=cap_col, tile_idx=3, flip_lr=True)
        self.set_mos_size(num_tiles=num_tiles,
                          num_cols=max(self.num_cols, cap_col))

        ptap0 = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0)
        ntap0 = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2)

        # === Connections
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')

        nds_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=1)
        nds_tid1 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)
        pds_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1, tile_idx=1)
        pds_tid1 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2, tile_idx=1)

        ng_tid1 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        pg_tid1 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        ng_tid0 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=1)
        pg_tid0 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=1)

        pg_hi_tid1 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        ng_lo_tid0 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        pg_hi_tid0 = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=1)
        ng_lo_tid1 = self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=1)
        pds_hi_tid0 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)
        nds_lo_tid0 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=1)
        pds_hi_tid1 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=1)
        nds_lo_tid1 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)

        # Input
        in_conn = self.connect_wires([m_nin0.g, m_pin0.g])
        mid_conn = self.connect_wires([m_nin1.g, m_pin1.g])

        in_hm = [self.connect_to_tracks(in_conn + [m_paz.s, m_naz.s], pg_hi_tid1),
                 self.connect_to_tracks(in_conn + [m_paz.s, m_naz.s], ng_lo_tid1)]
        mid_hm = [self.connect_to_tracks(mid_conn, ng_lo_tid0), self.connect_to_tracks(mid_conn, pg_hi_tid0)]

        nmid = self.connect_to_tracks([m_nin0.d, m_naz.d], nds_lo_tid0)
        pmid = self.connect_to_tracks([m_pin0.d, m_paz.d], pds_hi_tid0)

        namp = self.connect_to_tracks([m_nin1.d, m_ampn.d], nds_lo_tid1)
        pamp = self.connect_to_tracks([m_pin1.d, m_ampp.d], pds_hi_tid1)

        nshare = self.connect_to_tracks([m_sharen.d, m_ampn.s], nds_tid0)
        pshare = self.connect_to_tracks([m_sharep.d, m_ampp.s], pds_tid0)

        ncmfb = self.connect_to_tracks([m_chargen.d, m_sharen.s], nds_tid1)
        pcmfb = self.connect_to_tracks([m_chargep.d, m_sharep.s], pds_tid1)

        ncap = self.connect_to_tracks(m_chargen.s, nds_tid0)
        pcap = self.connect_to_tracks(m_chargep.s, pds_tid0)

        in_vm_tidx = self.arr_info.col_to_track(vm_layer, vm_col_list[0], RoundMode.NEAREST)
        in_vm = self.connect_to_tracks(in_hm, TrackID(vm_layer, in_vm_tidx, tr_w_sig_vm))

        in_xm_tidx = self.grid.coord_to_track(xm_layer, in_vm.middle, RoundMode.NEAREST)
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        in_xm = self.connect_to_tracks(in_vm, TrackID(xm_layer, in_xm_tidx,
                                                      tr_w_sig_xm), min_len_mode=MinLenMode.MIDDLE)

        mid_vm_tidx = self.arr_info.col_to_track(vm_layer, vm_col_list[1], RoundMode.NEAREST)
        mid_vm = self.connect_to_tracks([nmid, pmid] + mid_hm, TrackID(vm_layer, mid_vm_tidx, tr_w_sig_vm))
        amp_vm_tidx = self.arr_info.col_to_track(vm_layer, vm_col_list[2], RoundMode.NEAREST)
        amp_vm = self.connect_to_tracks([namp, pamp], TrackID(vm_layer, amp_vm_tidx, tr_w_sig_vm))
        share_vm_tidx = self.arr_info.col_to_track(vm_layer, vm_col_list[3], RoundMode.NEAREST)
        share_vm = self.connect_to_tracks([nshare, pshare], TrackID(vm_layer, share_vm_tidx, tr_w_sig_vm))
        cmfb_vm_tidx = self.arr_info.col_to_track(vm_layer, vm_col_list[4], RoundMode.NEAREST)
        cmfb_vm = self.connect_to_tracks([ncmfb, pcmfb], TrackID(vm_layer, cmfb_vm_tidx, tr_w_sig_vm))
        cmfb_xm_tidx = self.grid.coord_to_track(xm_layer, cmfb_vm.middle, RoundMode.NEAREST)
        cmfb_xm = self.connect_to_tracks(cmfb_vm, TrackID(xm_layer, cmfb_xm_tidx, tr_w_sig_xm))
        cmfb_ym_tid = self.grid.coord_to_track(ym_layer, cmfb_xm.middle, RoundMode.NEAREST)
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        cmfb_ym = self.connect_to_tracks(cmfb_xm, TrackID(ym_layer, cmfb_ym_tid, tr_w_sig_ym, grid=self.grid))

        cap_vm_tidx = self.arr_info.col_to_track(vm_layer, vm_col_list[5], RoundMode.NEAREST)
        cap_vm = self.connect_to_tracks([ncap, pcap], TrackID(vm_layer, cap_vm_tidx, tr_w_sig_vm))

        cap_xm_tidx = self.grid.coord_to_track(xm_layer, cap_vm.middle, RoundMode.NEAREST)
        cap_xm = self.connect_to_tracks(cap_vm, TrackID(xm_layer, cap_xm_tidx,
                                                        tr_manager.get_width(xm_layer, 'sig')))

        self.connect_to_track_wires(cap_xm, cap.get_pin('plus', layer=ym_layer))

        pphi_a = self.connect_to_tracks([m_sharep.g, m_chargep.g], pg_tid0)
        pphi_ab = self.connect_to_tracks([m_ampp.g], pg_tid1)
        nphi_ab = self.connect_to_tracks([m_sharen.g, m_chargen.g], ng_tid0)
        nphi_a = self.connect_to_tracks([m_ampn.g], ng_tid1)

        pphi_a_az = self.connect_to_tracks([m_paz.g], pg_tid0)
        nphi_ab_az = self.connect_to_tracks([m_naz.g], ng_tid1)

        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        main_tile_top = self.get_tile_info(3)[1]
        phi_a_az_vm = self.connect_to_tracks([pphi_a_az], TrackID(vm_layer, vm_ctrl_locs[1], tr_w_ctrl_vm),
                                             track_upper=main_tile_top)
        phi_ab_az_vm = self.connect_to_tracks([nphi_ab_az], TrackID(vm_layer, vm_ctrl_locs[2], tr_w_ctrl_vm),
                                              track_upper=main_tile_top)
        _, vm_ctrl_locs = tr_manager.place_wires(vm_layer, ['sig', 'ctrl', 'ctrl'],
                                                 align_track=cmfb_vm.track_id.base_index, align_idx=0)
        phi_a_vm = self.connect_to_tracks([pphi_a, nphi_a], TrackID(vm_layer, vm_ctrl_locs[1], tr_w_ctrl_vm),
                                             track_upper=main_tile_top)
        phi_ab_vm = self.connect_to_tracks([pphi_ab, nphi_ab], TrackID(vm_layer, vm_ctrl_locs[2], tr_w_ctrl_vm),
                                              track_upper=main_tile_top)

        # Connect supply
        vss_hm_list, vdd_hm_list = [], []
        vss_conn0 = [ptap0, m_nin0.s, m_nin1.s]
        vdd_conn0 = [ntap0, m_pin0.s, m_pin1.s]
        vdd_tid0 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=2)
        vss_tid0 = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=0)
        vdd_hm_list.append(self.connect_to_tracks(vdd_conn0, vdd_tid0))
        vss_hm_list.append(self.connect_to_tracks(vss_conn0, vss_tid0))

        # --- export suplies ---
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_sup_vm_sp = tr_manager.get_sep(vm_layer, ('sup', 'sup'))

        def export_hm_sup(hm_sup: WireArray):
            yloc = (hm_sup.bound_box.yl + hm_sup.bound_box.yh) // 2
            vm_locs = \
                self.get_available_tracks(vm_layer,
                                          self.grid.coord_to_track(vm_layer, hm_sup.bound_box.xl, RoundMode.NEAREST),
                                          self.grid.coord_to_track(vm_layer, hm_sup.bound_box.xh, RoundMode.NEAREST),
                                          hm_sup.bound_box.yl, hm_sup.bound_box.yh, tr_sup_vm_w,
                                          tr_sup_vm_sp)[1:]
            vm_sup_list = [self.connect_to_tracks(hm_sup, TrackID(vm_layer, tidx, tr_sup_vm_w)) for tidx in vm_locs]
            xm_sup_tidx = self.grid.coord_to_track(xm_layer, yloc, RoundMode.NEAREST)
            tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
            xm_sup = self.connect_to_tracks(vm_sup_list, TrackID(xm_layer, xm_sup_tidx, tr_sup_xm_w),
                                            track_lower=hm_sup.lower, track_upper=hm_sup.upper)
            return xm_sup

        vss_xm_list, vdd_xm_list = [], []
        for vss_warr in vss_hm_list:
            vss_xm_list.append(export_hm_sup(vss_warr))
        for vdd_warr in vdd_hm_list:
            vdd_xm_list.append(export_hm_sup(vdd_warr))

        self.connect_to_track_wires(vss_xm_list, cap.get_pin('minus', layer=ym_layer))
        _, phi_xm_locs = tr_manager.place_wires(xm_layer, ['sup', 'ctrl', 'ctrl'],
                                                align_track=vdd_xm_list[0].track_id.base_index)
        phi_a_xm, phi_ab_xm = self.connect_matching_tracks([[phi_a_vm, phi_a_az_vm],
                                                            [phi_ab_vm, phi_ab_az_vm]], xm_layer, phi_xm_locs[1:],
                                                           width=tr_manager.get_width(xm_layer, 'ctrl'))

        ym_l_locs = self.arr_info.col_to_track(ym_layer, 0)
        ym_r_locs = self.arr_info.col_to_track(ym_layer, self.num_cols)
        ym_sup_locs = self.get_available_tracks(ym_layer, ym_l_locs, ym_r_locs, self.bound_box.yl,
                                                cap.bound_box.yl, width=tr_manager.get_width(ym_layer, 'sup'),
                                                sep=tr_manager.get_sep(ym_layer, ('sup', 'sup')))
        ym_sup_locs = ym_sup_locs[:len(ym_sup_locs)//2*2]
        vss_ym_list, vdd_ym_list = [], []
        for idx in range(len(ym_sup_locs)//2):
            vss, vdd = self.connect_matching_tracks([vss_xm_list, vdd_xm_list], ym_layer,
                                                    ym_sup_locs[2*idx:2*idx+2],
                                                    width=tr_manager.get_width(ym_layer, 'sup'))
            vss_ym_list.append(vss)
            vdd_ym_list.append(vdd)

        xm1_layer = ym_layer + 1
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        vss_xm1_tidx = self.grid.coord_to_track(xm1_layer,
                                                self.grid.track_to_coord(xm_layer, vss_xm_list[0].track_id.base_index),
                                                RoundMode.NEAREST)
        vdd_xm1_tidx = self.grid.coord_to_track(xm1_layer,
                                                self.grid.track_to_coord(xm_layer, vdd_xm_list[0].track_id.base_index),
                                                RoundMode.NEAREST)
        vdd_xm1 = self.connect_to_tracks(vdd_ym_list, TrackID(xm1_layer, vdd_xm1_tidx, tr_w_sup_xm1))
        vss_xm1 = self.connect_to_tracks(vss_ym_list, TrackID(xm1_layer, vss_xm1_tidx, tr_w_sup_xm1))
        self.reexport(cap.get_port('minus'), hide=True)
        self.reexport(cap.get_port('plus'), hide=True)
        self.add_pin('VSS', vss_ym_list)
        self.add_pin('VDD', vdd_ym_list)
        self.add_pin('VSS', vss_xm1)
        self.add_pin('VDD', vdd_xm1)
        self.add_pin('in', in_xm)
        self.add_pin('phi_a', phi_a_xm, connect=True)
        self.add_pin('phi_ab', phi_ab_xm, connect=True)
        self.add_pin('out', cmfb_ym)
        fill_conn_layer_intv(self, 1, 0, extend_to_gate=False)
        fill_conn_layer_intv(self, 1, 1, extend_to_gate=False)

        # routing
        self._sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            cap_params=cap_master.sch_params,
        )
