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

from typing import Any, Dict, Tuple, Optional, Type

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from pybag.enum import RoundMode, MinLenMode
from xbase.layout.enum import MOSWireType, MOSType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase


class SourceFollower(MOSBase, TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'source_follower')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            w_dict='widths dictionary.',
            seg_dict='segments dictionary.',
            ridx_dict='bottom nmos row index.',
            fill_dummy='True to fill dummy',
            ndum='Number of dummy at sides',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_dict={},
            ofst_dict={},
            fill_dummy=False,
            ndum=0,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        ndum: int = self.params['ndum']

        seg_in, seg_bias = seg_dict['in'], seg_dict['bias']
        max_seg = max(seg_in, seg_bias)
        max_seg += max_seg & 1
        seg_tot = max_seg + 2 * ndum
        col_in = (seg_tot - seg_in) // 2
        col_bias = (seg_tot - seg_bias) // 2
        col_in += col_in & 1
        col_bias += col_bias & 1
        m_in = self.add_mos(ridx_dict['in'], seg=seg_in, col_idx=col_in, tile_idx=1)
        m_bias = self.add_mos(ridx_dict['bias'], seg=seg_bias, col_idx=col_bias, tile_idx=1)

        m_decap_l = self.add_mos(ridx_dict['bias'], seg=col_bias, col_idx=0, tile_idx=1)
        m_decap_r = self.add_mos(ridx_dict['bias'], seg=seg_tot - seg_bias - col_bias,
                                 col_idx=col_bias + seg_bias, tile_idx=1)

        self.set_mos_size()

        vss_tap = self.add_substrate_contact(0, 0, tile_idx=0, seg=self.num_cols)
        vdd_tap = self.add_substrate_contact(0, 0, tile_idx=2, seg=self.num_cols)
        vss_tidx = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0)
        vdd_tidx = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2)

        tx_vss_conn = [m_in.s] if ridx_dict['in'] < ridx_dict['bias'] else [m_bias.s, m_decap_l.s, m_decap_r.s]
        tx_vdd_conn = [m_bias.s, m_decap_l.s, m_decap_r.s] if ridx_dict['in'] < ridx_dict['bias'] else [m_in.s]
        vss_hm = self.connect_to_tracks([vss_tap] + tx_vss_conn, vss_tidx)
        vdd_hm = self.connect_to_tracks([vdd_tap] + tx_vdd_conn, vdd_tidx)
        decap_sup_tidx = self.get_track_id(ridx_dict['bias'], MOSWireType.DS, 'sup', tile_idx=1)
        decap_sup = self.connect_to_tracks([m_decap_l.d, m_decap_r.d], decap_sup_tidx)

        bias_hm_tidx = self.get_track_id(ridx_dict['bias'], MOSWireType.G, 'bias', tile_idx=1)
        bias_hm = self.connect_to_tracks([m_bias.g, m_decap_r.g, m_decap_l.g], bias_hm_tidx)

        in_hm_tidx = self.get_track_id(ridx_dict['in'], MOSWireType.G, 'sig', tile_idx=1)
        in_hm = self.connect_to_tracks(m_in.g, in_hm_tidx)

        out_hm_tidx = self.get_track_id(ridx_dict['bias'], MOSWireType.DS, 'sig', tile_idx=1)
        out_hm = self.connect_to_tracks([m_in.d, m_bias.d], out_hm_tidx)

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        bbox_m = (self.bound_box.xl + self.bound_box.xh) // 2
        vm_vss_l = self.export_tap_hm(tr_manager, vss_hm, hm_layer, vm_layer, bbox=[0, bbox_m])
        vm_vss_r = self.export_tap_hm(tr_manager, vss_hm, hm_layer, vm_layer,
                                      bbox=[bbox_m, self.bound_box.xh], align_upper=True)
        s_dummy, d_dummy, _ = fill_conn_layer_intv(self, 1, ridx_dict['in'])

        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        vss_xm_tidx = self.grid.coord_to_track(xm_layer, vm_vss_l[0].middle)
        vss_xm = self.connect_to_tracks(vm_vss_l + vm_vss_r, TrackID(xm_layer, vss_xm_tidx, tr_w_sup_xm))
        self.connect_to_track_wires(vm_vss_l + vm_vss_r, decap_sup)
        vdd_xm_l = self.export_tap_hm(tr_manager, vdd_hm, hm_layer, xm_layer, bbox=[0, bbox_m])
        vdd_xm_r = self.export_tap_hm(tr_manager, vdd_hm, hm_layer, xm_layer, bbox=[bbox_m, self.bound_box.xh])
        vdd_xm = self.connect_wires(vdd_xm_l + vdd_xm_r)

        if ridx_dict['in']:
            self.extend_wires(s_dummy + d_dummy, upper=vdd_hm.bound_box.yh)
        else:
            self.extend_wires(s_dummy + d_dummy, lower=vss_hm.bound_box.yl)

        self.add_pin('vb', bias_hm)
        self.add_pin('vin', in_hm)
        self.add_pin('vout', out_hm)
        self.add_pin('VSS', vss_hm)
        self.add_pin('VDD', vdd_hm)

        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())
        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            n_type=self.get_tile_pinfo(1).get_row_place_info(ridx_dict['in']).row_info.row_type == MOSType.nch,
            ndecap=2 * (m_decap_l.d.track_id.num + m_decap_r.d.track_id.num)
        )

    def _get_w_th_dict(self, ridx_dict: dict, w_dict: dict) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_ans = {}
        th_ans = {}
        for name, row_idx in ridx_dict.items():
            rinfo = self.get_row_info(row_idx, 1)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)

    @classmethod
    def get_group_node(cls, group, tx, node):
        nodes = [w for g in group for w in g.get_all_port_pins(tx + node)]
        return nodes


class SuperSourceFollower(MOSBase, TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._ntype = False

    def get_schematic_class(self) -> Optional[Type[Module]]:
        if self._ntype:
            # noinspection PyTypeChecker
            return ModuleDB.get_schematic_class('bag_vco_adc', 'super_source_follower_n')
        else:
            # noinspection PyTypeChecker
            return ModuleDB.get_schematic_class('bag_vco_adc', 'super_source_follower_p')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            w_dict='widths dictionary.',
            seg_dict='segments dictionary.',
            ridx_dict='bottom nmos row index.',
            fill_dummy='True to fill dummy',
            ndum='Number of dummy at sides',
            replica='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_dict={},
            ofst_dict={},
            fill_dummy=False,
            replica=False,
            ndum=0,
        )

    def draw_dac(self, seg_dict, w_dict, ridx_dict, start_coln, start_colp, ntype):
        ridx_biasn = ridx_dict['biasn']
        ridx_biasp = ridx_dict['biasp']
        seg_biasn_dac = seg_dict['biasn_dac']
        seg_biasp_dac = seg_dict['biasp_dac']

        ntap_sup_tid = self.get_track_id(0, MOSWireType.DS, 'sup_w', tile_idx=2)
        ptap_sup_tid = self.get_track_id(0, MOSWireType.DS, 'sup_w', tile_idx=0)

        ndac_list, pdac_list = [], []
        cur_col = start_coln + self.min_sep_col
        if seg_biasn_dac:
            for w, seg in zip(w_dict['biasn_dac'], seg_biasn_dac):
                cur_col += cur_col & 1
                dac = self.add_mos(ridx_biasn, seg=seg, w=w, col_idx=cur_col, tile_idx=1)
                cur_col += seg + self.min_sep_col
                ndac_list.append(dac)
        tot_col_n = cur_col

        cur_col = start_colp + self.min_sep_col
        if seg_biasp_dac:
            for w, seg in zip(w_dict['biasp_dac'], seg_biasp_dac):
                cur_col += cur_col & 1
                dac = self.add_mos(ridx_biasp, seg=seg, w=w, col_idx=cur_col, tile_idx=1)
                cur_col += seg + self.min_sep_col
                pdac_list.append(dac)
        tot_col_p = cur_col
        self.connect_to_tracks([dac.s if ntype else dac.d for dac in ndac_list], ptap_sup_tid)
        self.connect_to_tracks([dac.d if ntype else dac.s for dac in pdac_list], ntap_sup_tid)
        ndac_d = [dac.d if ntype else dac.s for dac in ndac_list]
        pdac_d = [dac.s if ntype else dac.d for dac in pdac_list]
        biasn_ctrl, biasp_ctrl = [], []
        n_ctrl_tid = self.get_track_id(ridx_dict['biasn'], MOSWireType.G, 'bias', wire_idx=1, tile_idx=1)
        p_ctrl_tid = self.get_track_id(ridx_dict['biasp'], MOSWireType.G, 'bias', wire_idx=1, tile_idx=1)

        vm_layer = n_ctrl_tid.layer_id + 1
        tr_w_sig_vm = self.tr_manager.get_width(vm_layer, 'sig')
        for dac in ndac_list:
            dac_hm = self.connect_to_tracks(dac.g, n_ctrl_tid, min_len_mode=MinLenMode.MIDDLE)
            dac_vm_tidx = self.grid.coord_to_track(vm_layer, dac_hm.middle, RoundMode.NEAREST)
            biasn_ctrl.append(self.connect_to_tracks(dac_hm, TrackID(vm_layer, dac_vm_tidx, tr_w_sig_vm)))
        for dac in pdac_list:
            dac_hm = self.connect_to_tracks(dac.g, p_ctrl_tid, min_len_mode=MinLenMode.MIDDLE)
            dac_vm_tidx = self.grid.coord_to_track(vm_layer, dac_hm.middle, RoundMode.NEAREST)
            biasp_ctrl.append(self.connect_to_tracks(dac_hm, TrackID(vm_layer, dac_vm_tidx, tr_w_sig_vm)))

        return biasn_ctrl, biasp_ctrl, ndac_d, pdac_d, max(tot_col_n, tot_col_p)

    def draw_biasing(self, seg_dict, w_dict, ridx_dict, ndum, ntype):
        start_col = ndum
        start_col += start_col & 1
        m_b = self.add_mos(ridx_dict['b'], seg=seg_dict['b'], col_idx=start_col, tile_idx=1)
        m_mi0 = self.add_mos(ridx_dict['mi0'], seg=seg_dict['mi0'], w=w_dict['mi0'],
                             col_idx=start_col, tile_idx=1)

        bias_col = start_col + max(seg_dict['b'], seg_dict['mi0'])
        bias_col += bias_col & 1
        m_sr0 = self.add_mos(ridx_dict['sr0'], seg=seg_dict['sr0'], w=w_dict['sr0'],
                             col_idx=bias_col, tile_idx=1)
        m_mi1 = self.add_mos(ridx_dict['mi1'], seg=seg_dict['mi1'], w=w_dict['mi1'],
                             col_idx=bias_col, tile_idx=1)
        m_mid = self.add_mos(ridx_dict['in'], seg=seg_dict['mid'], w=w_dict['mid'],
                             col_idx=bias_col, tile_idx=1)
        vb_hm_tidx = self.get_track_id(ridx_dict['b'], MOSWireType.G, 'bias', tile_idx=1, wire_idx=1)
        vb_hm = self.connect_to_tracks(m_b.g, vb_hm_tidx)
        b_hm_tidx = self.get_track_id(ridx_dict['b'], MOSWireType.DS, 'sig', tile_idx=1)
        b_hm = self.connect_to_tracks(m_b.d, b_hm_tidx)
        vb0_hm_tidx = self.get_track_id(ridx_dict['sr0'], MOSWireType.G, 'bias', tile_idx=1)
        vb0_hm = self.connect_to_tracks([m_sr0.g, m_mi0.g, m_mi0.d], vb0_hm_tidx)

        mid_s_hm_tidx = self.get_track_id(ridx_dict['in'], MOSWireType.DS, 'sig', wire_idx=1, tile_idx=1)
        mid_s_hm = self.connect_to_tracks(m_mid.s, mid_s_hm_tidx)
        mid_g_hm_tidx = self.get_track_id(ridx_dict['in'], MOSWireType.G, 'sig', tile_idx=1)
        mid_g_hm = self.connect_to_tracks(m_mid.g, mid_g_hm_tidx)
        vb1_hm_tidx = self.get_track_id(ridx_dict['mi1'], MOSWireType.G, 'bias', tile_idx=1)
        vb1_hm = self.connect_to_tracks([m_mi1.d, m_mi1.g], vb1_hm_tidx)

        sr0_hm_tidx = self.get_track_id(ridx_dict['sr0'], MOSWireType.DS, 'sig', tile_idx=1)
        sr0_hm = self.connect_to_tracks(m_sr0.d, sr0_hm_tidx)
        vss_conn = [m_b.s, m_mi1.s] if ntype else [m_sr0.s, m_mi0.s]
        vdd_conn = [m_sr0.s, m_mi0.s] if ntype else [m_b.s, m_mi1.s]
        self.connect_wires([m_mi1.d, m_mid.d])

        vm_layer = b_hm.layer_id + 1
        tr_w_bias_vm = self.tr_manager.get_width(vm_layer, 'bias')
        vm0_tidx = self.grid.coord_to_track(vm_layer, b_hm.middle, RoundMode.NEAREST)
        vm1_tidx = self.grid.coord_to_track(vm_layer, mid_s_hm.middle, RoundMode.NEAREST)
        mid_vm_tidx = self.tr_manager.get_next_track(vm_layer, vm1_tidx, 'bias', 'bias')
        mid_bias = self.connect_to_tracks(mid_g_hm, TrackID(vm_layer, mid_vm_tidx, tr_w_bias_vm))
        vm0 = self.connect_to_tracks([b_hm, vb0_hm], TrackID(vm_layer, vm0_tidx, tr_w_bias_vm))
        vm1 = self.connect_to_tracks([mid_s_hm, sr0_hm], TrackID(vm_layer, vm1_tidx, tr_w_bias_vm))

        vb_vm = self.connect_to_tracks(vb_hm, TrackID(vm_layer, vm0_tidx, tr_w_bias_vm), min_len_mode=MinLenMode.MIDDLE)

        return bias_col + max(seg_dict['mi1'], seg_dict['mid'],
                              seg_dict['sr0']), vss_conn, vdd_conn, vb0_hm, vb1_hm, mid_bias, vb_vm

    def draw_ssf(self, seg_dict, w_dict, ridx_dict, col_in, col_biasn, col_biasp, ntype, replica=False):
        name_list = ['in_r', 'biasn_r', 'biasp_r', 'fb_r'] if replica else ['in', 'biasn', 'biasp', 'fb']
        seg_in, seg_biasn, seg_biasp, seg_fb = [seg_dict[name] for name in name_list]
        w_in, w_biasn, w_biasp, w_fb = [w_dict.get(name + '_r', w_dict[name]) for name in
                                        ['in', 'biasn', 'biasp', 'fb']]
        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        m_in = self.add_mos(ridx_dict['in'], w=w_in, seg=seg_in, col_idx=col_in, tile_idx=1)
        m_biasn = self.add_mos(ridx_dict['biasn'], w=w_biasn, seg=seg_biasn, col_idx=col_biasn, tile_idx=1,
                               g_on_s=False)
        m_biasp = self.add_mos(ridx_dict['biasp'], w=w_biasp, seg=seg_biasp, col_idx=col_biasp, tile_idx=1,
                               g_on_s=False)
        col_fb = col_biasp + seg_biasp + min_sep if ntype else col_biasn + seg_biasn + min_sep
        m_fb = self.add_mos(ridx_dict['biasp'] if ntype else ridx_dict['biasn'], w=w_fb,
                            seg=seg_fb, col_idx=col_fb, tile_idx=1, g_on_s=True)
        return m_in, m_biasn, m_biasp, m_fb

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        replica: bool = self.params['replica']
        ndum: int = self.params['ndum']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        ntype = self.get_tile_pinfo(1).get_row_place_info(ridx_dict['in']).row_info.row_type == MOSType.nch
        self._ntype = ntype
        seg_in, seg_biasn, seg_biasp, seg_fb = seg_dict['in'], seg_dict['biasn'], seg_dict['biasp'], seg_dict['fb']
        max_seg = max(seg_in, seg_biasp + seg_fb + min_sep, seg_biasn) if ntype \
            else max(seg_in, seg_biasp, seg_biasn + seg_fb + min_sep)
        max_seg += max_seg & 1
        start_col, vss_conn, vdd_conn, vb0, vb1, mid_bias, vb_vm = \
            self.draw_biasing(seg_dict, w_dict, ridx_dict, ndum, ntype)

        if replica:
            seg_in_r, seg_biasn_r, seg_biasp_r, seg_fb_r = \
                seg_dict['in_r'], seg_dict['biasn_r'], seg_dict['biasp_r'], seg_dict['fb_r']
            max_seg_r = max(seg_in_r, seg_biasp_r + seg_fb_r + min_sep, seg_biasn_r) if ntype \
                else max(seg_in_r, seg_biasp_r, seg_biasn_r + seg_fb_r + min_sep)
            max_seg_r += min_sep
        else:
            max_seg_r = 0

        seg_tot = max_seg_r + max_seg + min_sep + ndum + start_col

        if replica:
            col_in = min_sep + start_col
            col_biasn, col_biasp = min_sep + start_col, min_sep + start_col
            col_in += col_in & 1
            col_biasn += col_biasn & 1
            col_biasp += col_biasp & 1
            m_in_r, m_biasn_r, m_biasp_r, m_fb_r = \
                self.draw_ssf(seg_dict, w_dict, ridx_dict, col_in, col_biasn, col_biasp, ntype, replica=True)
        else:
            m_in_r, m_biasn_r, m_biasp_r, m_fb_r = None, None, None, None

        start_col += max_seg_r
        col_in = min_sep + start_col
        col_biasn, col_biasp = min_sep + start_col, min_sep + start_col
        col_in += col_in & 1
        col_biasn += col_biasn & 1
        col_biasp += col_biasp & 1
        col_fb = col_biasp + seg_biasp + min_sep if ntype else col_biasn + seg_biasn + min_sep
        m_in, m_biasn, m_biasp, m_fb = self.draw_ssf(seg_dict, w_dict, ridx_dict, col_in, col_biasn, col_biasp, ntype,
                                                     replica=False)
        dac_start_coln = col_biasp + seg_biasn if ntype else col_fb + seg_fb
        dac_start_colp = col_fb + seg_fb if ntype else col_biasn + seg_biasp
        dacn_ctrl, dacp_ctrl, dacn_d, dacp_d, end_col = \
            self.draw_dac(seg_dict, w_dict, ridx_dict, dac_start_coln, dac_start_colp, ntype)

        # m_decap_p = self.add_mos(ridx_dict['biasp'], seg=col_biasp, col_idx=0, tile_idx=1)
        # m_decap_n = self.add_mos(ridx_dict['biasn'], seg=col_biasn, col_idx=0, tile_idx=1)

        self.set_mos_size(num_cols=max(seg_tot, end_col + ndum), num_tiles=3)

        tap_start = start_col + min_sep
        vss_tap = [
            self.add_substrate_contact(0, tap_start - max_seg_r, tile_idx=0, seg=self.num_cols - tap_start + max_seg_r,
                                       port_mode=SubPortMode.EVEN if ntype else SubPortMode.ODD)]
        vss_tap += [self.add_substrate_contact(0, 0, tile_idx=0, seg=tap_start - min_sep - max_seg_r)]
        vdd_tap = [
            self.add_substrate_contact(0, tap_start - max_seg_r, tile_idx=2, seg=self.num_cols - tap_start + max_seg_r,
                                       port_mode=SubPortMode.ODD if ntype else SubPortMode.EVEN)]
        vdd_tap += [self.add_substrate_contact(0, 0, tile_idx=2, seg=tap_start - min_sep - max_seg_r)]
        vss_tidx = self.get_track_id(0, MOSWireType.DS, 'sup_w', tile_idx=0)
        vdd_tidx = self.get_track_id(0, MOSWireType.DS, 'sup_w', tile_idx=2)

        tx_vss_conn = [m_biasn.s] if ntype else [m_biasn.d, m_fb.d]
        tx_vdd_conn = [m_biasp.d, m_fb.d] if ntype else [m_biasp.s]
        #
        fb_g_hm_tidx = self.get_track_id(ridx_dict['biasp'] if ntype else ridx_dict['biasn'],
                                         MOSWireType.G, 'bias', tile_idx=1, wire_idx=1)
        biasn_hm_tidx = self.get_track_id(ridx_dict['biasn'], MOSWireType.G, 'bias', tile_idx=1)
        biasn_hm = self.connect_to_tracks([m_biasn.g], biasn_hm_tidx)
        biasp_hm_tidx = self.get_track_id(ridx_dict['biasp'], MOSWireType.G, 'bias', tile_idx=1)
        biasp_hm = self.connect_to_tracks([m_biasp.g], biasp_hm_tidx)

        in_hm_tidx = self.get_track_id(ridx_dict['in'], MOSWireType.G, 'sig', tile_idx=1)

        out_hm_tidx = self.get_track_id(ridx_dict['biasn'] if ntype else ridx_dict['biasp'],
                                        MOSWireType.DS, 'sig', tile_idx=1)
        out_fb_hm_tidx = self.get_track_id(ridx_dict['biasp'] if ntype else ridx_dict['biasn'],
                                           MOSWireType.DS, 'sig', tile_idx=1)
        out_fb_hm = self.connect_to_tracks(m_fb.s, out_fb_hm_tidx)
        out_hm = self.connect_to_tracks([m_in.d, m_biasn.d] if ntype else [m_in.d, m_biasp.d], out_hm_tidx)
        in_hm = self.connect_to_tracks(m_in.g, in_hm_tidx)
        fb_g_hm = self.connect_to_tracks(m_fb.g, fb_g_hm_tidx)

        vm_hm_tidx = self.get_track_id(ridx_dict['in'], MOSWireType.DS, 'sig', tile_idx=1, wire_idx=1)
        vm_fb = self.connect_to_tracks([m_in.s, m_biasp.s, m_fb.g] if ntype else [m_in.s, m_biasn.s, m_fb.g],
                                       vm_hm_tidx)

        dacn_hm_tid = self.get_track_id(ridx_dict['biasn'], MOSWireType.DS, 'sup', tile_idx=1)
        dacp_hm_tid = self.get_track_id(ridx_dict['biasp'], MOSWireType.DS, 'sup', tile_idx=1)
        self.connect_to_tracks(dacp_d + [m_biasp.s if ntype else m_biasp.d], dacp_hm_tid)
        self.connect_to_tracks(dacn_d + [m_biasn.d if ntype else m_biasn.s], dacn_hm_tid)

        tr_w_ana_sig_vm = tr_manager.get_width(vm_layer, 'ana_sig')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        if replica:
            out_fb_hm_r = self.connect_to_tracks(m_fb_r.s, out_fb_hm_tidx)
            out_hm_r = self.connect_to_tracks([m_in_r.d, m_biasn_r.d] if ntype else [m_in_r.d, m_biasp_r.d],
                                              out_hm_tidx)
            in_hm_r = self.connect_to_tracks(m_in_r.g, in_hm_tidx)
            fb_g_hm_r = self.connect_to_tracks(m_fb_r.g, fb_g_hm_tidx)
            self.connect_to_tracks([m_biasn_r.g], biasn_hm_tidx)
            self.connect_to_tracks([m_biasp_r.g], biasp_hm_tidx)
            self.connect_to_track_wires(biasn_hm, m_biasn_r.g)
            self.connect_to_track_wires(biasp_hm, m_biasp_r.g)
            self.connect_to_tracks([m_biasp_r.s if ntype else m_biasp_r.d], dacp_hm_tid)
            self.connect_to_tracks([m_biasn_r.d if ntype else m_biasn_r.s], dacn_hm_tid)
            self.connect_to_tracks([m_in_r.s, m_biasp_r.s, m_fb_r.g] if ntype else [m_in_r.s, m_biasn_r.s, m_fb_r.g],
                                   vm_hm_tidx)
            tx_vss_conn.extend([m_biasn_r.s] if ntype else [m_biasn_r.d, m_fb_r.d])
            tx_vdd_conn.extend([m_biasp_r.d, m_fb_r.d] if ntype else [m_biasp_r.s])
            vout_r_vm_tidx = self.grid.coord_to_track(vm_layer, out_hm_r.middle, RoundMode.NEAREST)
            vout_r_vm = self.connect_to_tracks([out_hm_r, out_fb_hm_r],
                                               TrackID(vm_layer, vout_r_vm_tidx, tr_w_sig_vm, grid=self.grid))
            vout_r_xm_tidx = self.grid.coord_to_track(xm_layer, vout_r_vm.lower, RoundMode.NEAREST)
            vout_fb_xm = self.connect_to_tracks(vout_r_vm, TrackID(xm_layer, vout_r_xm_tidx, tr_w_sig_xm))
            self.connect_wires([in_hm, in_hm_r])
            self.add_pin('vout_fb', vout_fb_xm)

        vss_hm = self.connect_to_tracks(vss_tap + tx_vss_conn + vss_conn, vss_tidx)
        vdd_hm = self.connect_to_tracks(vdd_tap + tx_vdd_conn + vdd_conn, vdd_tidx)
        self.connect_to_track_wires(mid_bias, vdd_hm if ntype else vss_hm)

        vout_vm_tidx = self.grid.coord_to_track(vm_layer, out_hm.middle, RoundMode.NEAREST)
        vout_vm = self.connect_to_tracks([out_hm, out_fb_hm], TrackID(vm_layer, vout_vm_tidx, tr_w_ana_sig_vm,
                                                                      grid=self.grid))
        tr_sp_ana_sig_vm = self.get_track_sep(vm_layer, tr_w_ana_sig_vm, tr_w_sig_vm)
        in_vm_coord = min(self.grid.track_to_coord(vm_layer, vout_vm_tidx - tr_sp_ana_sig_vm),
                          in_hm.middle)
        in_vm_tidx = self.grid.coord_to_track(vm_layer, in_vm_coord, RoundMode.LESS)
        in_vm = self.connect_to_tracks(in_hm, TrackID(vm_layer, in_vm_tidx, tr_w_sig_vm, grid=self.grid),
                                       min_len_mode=MinLenMode.MIDDLE)
        bbox_m = (self.bound_box.xl + self.bound_box.xh) // 2
        vss_xm_l = self.export_tap_hm(tr_manager, vss_hm, hm_layer, xm_layer, bbox=[0, bbox_m])
        vss_xm_r = self.export_tap_hm(tr_manager, vss_hm, hm_layer, xm_layer,
                                      bbox=[bbox_m, self.bound_box.xh], align_upper=True)

        for idx in range(3):
            s_dummy, d_dummy, _ = fill_conn_layer_intv(self, 1, idx)
        #
        vdd_xm_l = self.export_tap_hm(tr_manager, vdd_hm, hm_layer, xm_layer, bbox=[0, bbox_m])
        vdd_xm_r = self.export_tap_hm(tr_manager, vdd_hm, hm_layer, xm_layer, bbox=[bbox_m, self.bound_box.xh])
        vdd_xm = self.connect_wires(vdd_xm_l + vdd_xm_r)
        vss_xm = self.connect_wires(vss_xm_l + vss_xm_r)
        #
        # if ridx_dict['in']:
        #     self.extend_wires(s_dummy + d_dummy, upper=vdd_hm.bound_box.yh)
        # else:
        #     self.extend_wires(s_dummy + d_dummy, lower=vss_hm.bound_box.yl)
        self.connect_wires([biasp_hm, biasn_hm, vb0, vb1])

        tr_w_bias_xm = tr_manager.get_width(xm_layer, 'bias')
        vb_xm_tidx = self.grid.coord_to_track(xm_layer, vb_vm.middle, RoundMode.NEAREST)
        vb_xm = self.connect_to_tracks(vb_vm, TrackID(xm_layer, vb_xm_tidx, tr_w_bias_xm),
                                       track_lower=self.bound_box.xl)

        dacn_ctrl_xm_tidx = self.grid.coord_to_track(xm_layer, dacn_ctrl[0].middle, RoundMode.NEAREST)
        dacp_ctrl_xm_tidx = self.grid.coord_to_track(xm_layer, dacp_ctrl[0].middle, RoundMode.NEAREST)
        dacn_xm_list = tr_manager.place_wires(xm_layer, ['bias'] * len(dacn_ctrl), align_track=dacn_ctrl_xm_tidx)[1]
        dacp_xm_list = tr_manager.place_wires(xm_layer, ['bias'] * len(dacn_ctrl), align_track=dacp_ctrl_xm_tidx,
                                              align_idx=-1)[1]

        dacn_ctrl_xm = self.connect_matching_tracks(dacn_ctrl, xm_layer, dacn_xm_list, width=tr_w_bias_xm,
                                                    track_upper=self.bound_box.xh)
        dacp_ctrl_xm = self.connect_matching_tracks(dacp_ctrl, xm_layer, dacp_xm_list, width=tr_w_bias_xm,
                                                    track_upper=self.bound_box.xh)
        for idx, pin in enumerate(dacn_ctrl_xm):
            self.add_pin(f'ctrl_biasn<{idx}>', pin)
        for idx, pin in enumerate(dacp_ctrl_xm):
            self.add_pin(f'ctrl_biasp<{idx}>', pin)
        self.add_pin('vb', vb_xm)
        self.add_pin('vin', in_vm)
        self.add_pin('vout', vout_vm)
        self.add_pin('VSS', vss_xm)
        self.add_pin('VDD', vdd_xm)
        self.add_pin('vbp', biasp_hm, hide=True)
        self.add_pin('vbn', biasn_hm, hide=True)

        dacn_w_seg_pair = [(w, seg) for w, seg in zip(w_dict['biasn_dac'], seg_dict['biasn_dac'])]
        dacp_w_seg_pair = [(w, seg) for w, seg in zip(w_dict['biasn_dac'], seg_dict['biasn_dac'])]
        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())
        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            n_type=ntype,
            ndecap=0,
            dacn_w_seg=dacn_w_seg_pair,
            dacp_w_seg=dacp_w_seg_pair,
            replica=replica,
            # ndecap=2*(m_decap_l.d.track_id.num + m_decap_r.d.track_id.num)
        )

    def _get_w_th_dict(self, ridx_dict: dict, w_dict: dict) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_ans = {}
        th_ans = {}
        for name, row_idx in ridx_dict.items():
            rinfo = self.get_row_info(row_idx, 1)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)

    @classmethod
    def get_group_node(cls, group, tx, node):
        nodes = [w for g in group for w in g.get_all_port_pins(tx + node)]
        return nodes
