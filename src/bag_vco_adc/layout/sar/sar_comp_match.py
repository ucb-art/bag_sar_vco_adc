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

from typing import Any, Dict, Type, Optional, Tuple

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.routing.base import TrackID, WireArray
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict, ImmutableList
from pybag.enum import RoundMode, MinLenMode
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from ..util.template import TemplateBaseZL
from ..util.util import connect_conn_dummy_rows, fill_conn_layer_intv, fill_and_collect


class SingleEndTxGroup(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ofst_dict='column offset dictionary',
            ridx_dict='bottom nmos row index.',
            g_on_s_list='add tx name here if want g_on_s',
            conn_pair_list='Pair of connections',
            sig_locs='Optional dictionary of user defined signal locations',
            ndum='Number of dummy at sides',
            fill_dummy='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            ridx_dict={},
            ofst_dict={},
            conn_pair_list=[],
            g_on_s_list=[],
            fill_dummy=False,
            ndum=0,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        ridx_dict_temp: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']
        conn_pair_list: ImmutableList[Tuple[str, str]] = self.params['conn_pair_list']
        g_on_s_list: ImmutableList[Tuple[str, str]] = self.params['g_on_s_list']
        ndum: int = self.params['ndum']

        ridx_dict = ridx_dict_temp.copy().to_dict()
        if not ridx_dict:
            for idx, key in enumerate(seg_dict.keys()):
                ridx_dict[key] = idx

        w_dict, th_dict = self._get_w_th_dict(ridx_dict, w_dict.to_dict())
        tx_name_list = list(seg_dict.keys())

        max_seg = max(seg_dict.values()) + 2 * ndum
        # if seg_in & 1 or (seg_tail % 4 != 0) or seg_cas & 1 or seg_load & 1:
        #     raise ValueError('in, tail, nfb, or pfb must have even number of segments')
        # seg_tail = seg_tail // 2

        # placement
        m_dict = {}
        ofst_dict = self.params['ofst_dict']
        for k in tx_name_list:
            # _col = (max_seg-seg_dict[k])//2
            _col = ndum + ofst_dict.get(k, 0)
            m_dict[k] = self.add_mos(ridx_dict[k], _col, seg_dict[k], w=w_dict[k], g_on_s=k in g_on_s_list)

        self.set_mos_size(num_cols=max_seg + (max_seg & 1))

        for inst0, inst1 in conn_pair_list:
            inst0_conn = getattr(m_dict[inst0[0]], inst0[1])
            inst1_conn = getattr(m_dict[inst1[0]], inst1[1])
            self.connect_wires([inst0_conn, inst1_conn])

        if self.params['fill_dummy']:
            for k in tx_name_list:
                fill_conn_layer_intv(self, 0, ridx_dict[k])

        for k, v in m_dict.items():
            self.add_pin(k + 'd', v.d, hide=True)
            self.add_pin(k + 'g', v.g, hide=True)
            self.add_pin(k + 's', v.s, hide=True)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
        )

    def _get_w_th_dict(self, ridx_dict: dict, w_dict: dict) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_ans = {}
        th_ans = {}
        for name, row_idx in ridx_dict.items():
            rinfo = self.get_row_info(row_idx, 0)
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


class PreAmpMatch(SingleEndTxGroup):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._has_ofst = False

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_preamp')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_dict='bottom nmos row index.',
            conn_pair_list='Pair of connections',
            sig_locs='Optional dictionary of user defined signal locations',
            ndum='Number of dummy at sides',
            ngroups='Number of groups',
            fill_dummy='',
            has_cas_dev='True to add cascade dev',
            flip_np='False to have nmos input',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            ridx_dict={},
            conn_pair_list=[],
            fill_dummy=False,
            has_cas_dev=True,
            flip_np=True,
            ndum=0,
            ngroups=2,
        )

    @property
    def has_ofst(self) -> bool:
        return self._has_ofst

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        ngroups = self.params['ngroups']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']

        # Setup tracks
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())

        flip_np = self.params['flip_np']

        # max_seg = max(seg_dict.values()) + 2*ndum
        ridx_dict_group = ridx_dict.copy(remove=['tail', 'os'])
        seg_dict_group = seg_dict.copy(remove=['tail', 'os'])
        w_dict_group = w_dict.copy(remove=['tail', 'os'])
        seg_tail = seg_dict['tail']

        has_cas_dev = self.params['has_cas_dev']

        if has_cas_dev:
            conn_pair_list = [(('in', 's'), ('cas', 's')), (('cas', 'd'), ('load', 'd'))]
        else:
            conn_pair_list = [(('in', 'd'), ('load', 'd'))]
        group_params = dict(
            pinfo=self.params['pinfo'],
            seg_dict=seg_dict_group,
            w_dict=w_dict_group,
            ridx_dict=ridx_dict_group,
            conn_pair_list=conn_pair_list,
            g_on_s_list=['in'] if not has_cas_dev else [],
            ndum=0,
        )

        group_temp: MOSBase = self.new_template(params=group_params, temp_cls=SingleEndTxGroup)
        if ngroups & 1 or seg_tail % 2 != 0:
            raise ValueError('must have even number groups')

        # placement

        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        group_ncol = group_temp.num_cols
        ncol_group_tot = group_ncol * ngroups + min_sep * (ngroups - 1)

        seg_os = seg_dict.get('os', 0)
        self._has_ofst = bool(seg_os)

        ncol_tail_tot = seg_tail + 2 * (seg_os + min_sep) + min_sep

        ncol_tot = max(ncol_group_tot, ncol_tail_tot)

        tail_ncol = (ncol_tot - seg_tail) // 2
        tail_ncol -= 1
        tail_ncol -= tail_ncol & 1
        tail_col_l, tail_col_r = tail_ncol, ncol_tot - tail_ncol
        m_tail_l = self.add_mos(ridx_dict['tail'], tail_ncol, w=w_dict['tail'], seg=seg_dict['tail'] // 2,
                                g_on_s=True)
        m_tail_r = self.add_mos(ridx_dict['tail'], ncol_tot - tail_ncol, w=w_dict['tail'],
                                seg=seg_dict['tail'] // 2, flip_lr=True, g_on_s=True)
        m_tail_s = [m_tail_l.s, m_tail_r.s]
        m_tail_d = [m_tail_l.d, m_tail_r.d]
        m_tail_g = [m_tail_l.g, m_tail_r.g]
        if not has_cas_dev:
            m_tail_s, m_tail_d = m_tail_d, m_tail_s

        tx_groups = []
        cur_col = (ncol_tot - ncol_group_tot) // 2
        for idx in range(ngroups // 2):
            cur_col += group_temp.num_cols
            tx_groups.append(self.add_tile(group_temp, 0, cur_col, flip_lr=True))
            cur_col += min_sep
        for idx in range(ngroups // 2, ngroups):
            tx_groups.append(self.add_tile(group_temp, 0, cur_col))
            cur_col += group_temp.num_cols + min_sep

        # Get n and p side groups, assume even number of groups
        tx_groups_n, tx_groups_p = tx_groups[::2], tx_groups[1::2]

        self.set_mos_size(ncol_tot)

        # Make new segment dictionary, update based on number of groups
        seg_dict_new = {'tail': seg_tail}
        for k, v in seg_dict_group.items():
            seg_dict_new[k] = v * ngroups // 2

        if self.has_ofst:
            seg_dict_new['os'] = seg_os

        # ===== Routings =====

        # ----- hm layer -----
        # clk hm
        clk_hm_bot_tid = self.get_track_id(ridx_dict['tail'], MOSWireType.G, 'clk', 0)
        clk_hm_top_tid = self.get_track_id(ridx_dict['load'], MOSWireType.G, 'clk', 0)
        clk_hm_bot = self.connect_to_tracks(m_tail_g, clk_hm_bot_tid)
        groups_load_g = self.get_group_node(tx_groups, 'load', 'g')
        clk_hm_top = self.connect_to_tracks(groups_load_g, clk_hm_top_tid)

        # tail node hm
        groups_tail_d = self.get_group_node(tx_groups, 'in', 'd' if has_cas_dev else 's')
        _tail_hm = self.connect_to_tracks(groups_tail_d + m_tail_d,
                                          self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', 1))

        # input pair
        inp_tidx, hm_w = self.get_track_info(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        inn_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        # inn, inp = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'in', 'g'),
        #                                             self.get_group_node(tx_groups_p, 'in', 'g'),
        #                                             hm_layer, inp_tidx, inn_tidx, width=hm_w)
        inn = self.connect_to_tracks(self.get_group_node(tx_groups_n, 'in', 'g'),
                                     TrackID(hm_layer, inn_tidx, width=hm_w))
        inp = self.connect_to_tracks(self.get_group_node(tx_groups_p, 'in', 'g'),
                                     TrackID(hm_layer, inn_tidx, width=hm_w))

        # connection btw input and cascode
        midn_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS, wire_name='sig', wire_idx=0)
        midp_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS, wire_name='sig', wire_idx=1)
        midn = self.connect_to_tracks(self.get_group_node(tx_groups_n, 'in', 'd'),
                                      TrackID(hm_layer, midp_tidx, width=hm_w))
        midp = self.connect_to_tracks(self.get_group_node(tx_groups_p, 'in', 'd'),
                                      TrackID(hm_layer, midp_tidx, width=hm_w))

        # output pair
        if has_cas_dev:
            outn_tidx = self.get_track_index(ridx_dict['cas'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=0)
            outp_tidx = self.get_track_index(ridx_dict['cas'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)
            outp, outn = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'cas', 'd'),
                                                          self.get_group_node(tx_groups_p, 'cas', 'd'),
                                                          hm_layer, outp_tidx, outn_tidx, width=hm_w)
            outp, outn = [outp], [outn]
        else:
            outp_tidx = self.get_track_index(ridx_dict['load'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)
            outn = [self.connect_to_tracks(self.get_group_node(tx_groups_n, 'load', 'd'),
                                           TrackID(hm_layer, outp_tidx, width=hm_w)), midn]
            outp = [self.connect_to_tracks(self.get_group_node(tx_groups_p, 'load', 'd'),
                                           TrackID(hm_layer, outp_tidx, width=hm_w)), midp]

        # cas dev gate
        if has_cas_dev:
            cas_sup_tidx = self.get_track_id(ridx_dict['cas'], MOSWireType.G, wire_name='gate_sup')
            cas_sup = self.connect_to_tracks(self.get_group_node(tx_groups, 'cas', 'g'), cas_sup_tidx)
            self.add_pin('VDD', cas_sup, connect=True)

        if self.has_ofst:
            osn = self.add_mos(ridx_dict['tail'], tail_col_l - min_sep, seg=seg_os, w=w_dict['os'], flip_lr=True)
            osp = self.add_mos(ridx_dict['tail'], tail_col_r + min_sep, seg=seg_os, w=w_dict['os'])
            os_tidx = self.get_track_index(ridx_dict['tail'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)

            outn = outn + [
                self.connect_to_tracks(osn.s if has_cas_dev else osn.d,
                                       self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', 0))]
            outp = outp + [
                self.connect_to_tracks(osp.s if has_cas_dev else osp.d,
                                       self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', 0))]

            ofst_tail_conn = groups_tail_d + [osn.d, osp.d] if has_cas_dev else groups_tail_d + [osn.s, osp.s]
            _tail_os_hm = self.connect_to_tracks(ofst_tail_conn,
                                                 self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', 1))

            osp_hm = self.connect_to_tracks(osp.g, TrackID(hm_layer, os_tidx, hm_w))
            osn_hm = self.connect_to_tracks(osn.g, TrackID(hm_layer, os_tidx, hm_w))

            # osp_vm_tidx = self.grid.coord_to_track(vm_layer, osp_hm.middle, mode=RoundMode.NEAREST)
            # osn_vm_tidx = self.grid.coord_to_track(vm_layer, osn_hm.middle, mode=RoundMode.NEAREST)
            # osn_vm_tidx = tr_manager.get_next_track(vm_layer, inn)
            # osp_vm = self.connect_to_tracks(osp_hm, TrackID(vm_layer, osp_vm_tidx, tr_w_vm),
            #                                 min_len_mode=MinLenMode.MIDDLE)
            # osn_vm = self.connect_to_tracks(osn_hm, TrackID(vm_layer, osn_vm_tidx, tr_w_vm),
            #                                 min_len_mode=MinLenMode.MIDDLE)

            self.add_pin('osp', osp_hm)
            self.add_pin('osn', osn_hm)

        # dummy metal fill
        if self.params['fill_dummy']:
            tx_name_list = list(seg_dict.keys())
            for k in tx_name_list:
                fill_conn_layer_intv(self, 0, ridx_dict[k], extend_to_gate=False)

        # ===== add pins =====
        # - conn
        if flip_np:
            self.add_pin('VSS', self.get_group_node(tx_groups, 'load', 's'), connect=True)
            self.add_pin('VDD', m_tail_s, connect=True)
        else:
            self.add_pin('VDD', self.get_group_node(tx_groups, 'load', 's'), connect=True)
            self.add_pin('VSS', m_tail_s, connect=True)

        # - hm
        self.add_pin('clk_hm', [clk_hm_bot, clk_hm_top], label='clk', connect=True)
        self.add_pin('inn', inn)
        self.add_pin('inp', inp)
        self.add_pin('outp', outp)
        self.add_pin('outn', outn)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict_new,
            w_dict=w_dict,
            th_dict=th_dict,
            has_ofst=self.has_ofst,
            has_cas=has_cas_dev,
            flip_np=flip_np,
        )


class HalfLatchMatch(SingleEndTxGroup):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._has_ofst = False

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_half_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_dict='bottom nmos row index.',
            conn_pair_list='Pair of connections',
            sig_locs='Optional dictionary of user defined signal locations',
            ndum='Number of dummy at sides',
            ngroups='Number of groups',
            fill_dummy='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            ridx_dict={},
            conn_pair_list=[],
            fill_dummy=False,
            ndum=0,
            ngroups=2,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        ngroups = self.params['ngroups']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']

        # Setup tracks
        hm_layer = self.conn_layer + 1
        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())

        # max_seg = max(seg_dict.values()) + 2*ndum
        ridx_dict_group = ridx_dict.copy()
        seg_dict_group = seg_dict.copy()
        w_dict_group = w_dict.copy()
        # seg_tail = seg_dict['tail']
        #

        conn_pair_list = [(('cp', 'd'), ('tail', 'd')), (('in', 'd'), ('cp', 'g'))]
        group_params = dict(
            pinfo=self.params['pinfo'],
            seg_dict=seg_dict_group,
            w_dict=w_dict_group,
            ridx_dict=ridx_dict_group,
            conn_pair_list=conn_pair_list,
            g_on_s_list=['in'],
            ndum=0,
        )

        group_temp: MOSBase = self.new_template(params=group_params, temp_cls=SingleEndTxGroup)
        if ngroups & 1:
            raise ValueError('must have even number groups')

        # placement

        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        group_ncol = group_temp.num_cols
        ncol_group_tot = group_ncol * ngroups + min_sep * (ngroups - 1)
        ncol_tot = ncol_group_tot

        tx_groups = []
        cur_col = (ncol_tot - ncol_group_tot) // 2
        for idx in range(ngroups // 2):
            cur_col += group_temp.num_cols
            tx_groups.append(self.add_tile(group_temp, 0, cur_col, flip_lr=True))
            cur_col += min_sep
        for idx in range(ngroups // 2, ngroups):
            tx_groups.append(self.add_tile(group_temp, 0, cur_col))
            cur_col += group_temp.num_cols + min_sep

        # Get n and p side groups, assume even number of groups
        tx_groups_n, tx_groups_p = tx_groups[::2], tx_groups[1::2]

        self.set_mos_size(ncol_tot)

        # Make new segment dictionary, update based on number of groups
        seg_dict_new = {}
        for k, v in seg_dict_group.items():
            seg_dict_new[k] = v * ngroups // 2 if k != 'tail' else v * ngroups

        # ===== Routings =====
        # ----- hm layer -----
        # clk hm
        clk_hm_tid = self.get_track_id(ridx_dict['tail'], MOSWireType.G, 'clk', 0)
        clk_hm = self.connect_to_tracks(self.get_group_node(tx_groups, 'tail', 'g'), clk_hm_tid)

        # tail node hm
        groups_tail_d = self.get_group_node(tx_groups, 'tail', 'd')
        groups_cp_d = self.get_group_node(tx_groups, 'cp', 'd')
        _tail_hm = self.connect_to_tracks(groups_tail_d + groups_cp_d,
                                          self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', 0))

        # input pair
        inp_tidx, hm_w = self.get_track_info(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=1)
        inn_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        inn, inp = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'in', 'g'),
                                                    self.get_group_node(tx_groups_p, 'in', 'g'),
                                                    hm_layer, inp_tidx, inn_tidx, width=hm_w)

        # output pair
        outn_cp_tidx = self.get_track_index(ridx_dict['cp'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=0)
        outp_cp_tidx = self.get_track_index(ridx_dict['cp'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)
        outp_cp, outn_cp = self.connect_differential_tracks(self.get_group_node(tx_groups_p, 'cp', 's'),
                                                            self.get_group_node(tx_groups_n, 'cp', 's'),
                                                            hm_layer, outp_cp_tidx, outn_cp_tidx, width=hm_w)
        outn_in_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=0)
        outp_in_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)
        outp_in, outn_in = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'in', 'd'),
                                                            self.get_group_node(tx_groups_p, 'in', 'd'),
                                                            hm_layer, outp_in_tidx, outn_in_tidx, width=hm_w)
        # connect floting out if any
        self.connect_to_tracks(self.get_group_node(tx_groups_n, 'cp', 'g'),
                               self.get_track_id(ridx_dict['cp'], MOSWireType.G, wire_name='sig'))
        self.connect_to_tracks(self.get_group_node(tx_groups_p, 'cp', 'g'),
                               self.get_track_id(ridx_dict['cp'], MOSWireType.G, wire_name='sig'))

        # dummy metal fill
        if self.params['fill_dummy']:
            tx_name_list = list(seg_dict.keys())
            for k in tx_name_list:
                fill_conn_layer_intv(self, 0, ridx_dict[k], extend_to_gate=False)

        # ===== add pins =====
        # - conn
        self.add_pin('VDD', self.get_group_node(tx_groups, 'tail', 's'), connect=True)
        self.add_pin('VSS', self.get_group_node(tx_groups, 'in', 's'), connect=True)

        # - hm
        self.add_pin('clkb_hm', clk_hm, label='clkb', connect=True)
        self.add_pin('inn', inn)
        self.add_pin('inp', inp)
        self.add_pin('outp', [outp_cp, outp_in], connect=True)
        self.add_pin('outn', [outn_cp, outn_in], connect=True)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict_new,
            w_dict=w_dict,
            th_dict=th_dict,
        )


class DynLatchMatch(SingleEndTxGroup):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._has_ofst = False

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_dyn_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_dict='bottom nmos row index.',
            conn_pair_list='Pair of connections',
            sig_locs='Optional dictionary of user defined signal locations',
            ndum='Number of dummy at sides',
            ngroups='Number of groups',
            fill_dummy='',
            flip_np='False to have nmos input',
            split_ck='Separate clock and tail nodes'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            ridx_dict={},
            conn_pair_list=[],
            fill_dummy=False,
            flip_np=True,
            split_ck=False,
            ndum=0,
            ngroups=2,
        )

    @property
    def has_ofst(self) -> bool:
        return self._has_ofst

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        ngroups = self.params['ngroups']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']

        # Setup tracks
        hm_layer = self.conn_layer + 1

        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())

        # max_seg = max(seg_dict.values()) + 2*ndum
        ridx_dict_group = ridx_dict.copy(remove=['os'])
        seg_dict_group = seg_dict.copy(remove=['os'])
        w_dict_group = w_dict.copy(remove=['os'])
        seg_tail = seg_dict['tail']
        has_rst = 'rst' in seg_dict.keys()

        flip_np = self.params['flip_np']
        split_ck = self.params['split_ck']
        if flip_np:
            conn_pair_list = [(('pfb', 'g'), ('nfb', 'g')), (('pfb', 'd'), ('in', 'd')), (('pfb', 's'), ('in', 's')),
                              (('nfb', 'd'), ('tail', 's'))]
        else:
            conn_pair_list = [(('pfb', 'g'), ('nfb', 'g')), (('nfb', 'd'), ('in', 'd')), (('nfb', 's'), ('in', 's')),
                              (('pfb', 'd'), ('tail', 's'))]

        group_params = dict(
            pinfo=self.params['pinfo'],
            seg_dict=seg_dict_group,
            w_dict=w_dict_group,
            ridx_dict=ridx_dict_group,
            conn_pair_list=conn_pair_list,
            ofst_dict={'rst': int(seg_dict_group['in'])} if has_rst else None,
            ndum=0,
        )

        group_temp: MOSBase = self.new_template(params=group_params, temp_cls=SingleEndTxGroup)
        if ngroups & 1 or seg_tail % 2 != 0:
            raise ValueError('must have even number groups')

        # placement

        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        group_ncol = group_temp.num_cols
        ncol_group_tot = group_ncol * ngroups + min_sep * (ngroups - 1)

        seg_os = seg_dict.get('os', 0)
        self._has_ofst = bool(seg_os)

        ncol_tot = ncol_group_tot

        tx_groups = []
        cur_col = (ncol_tot - ncol_group_tot) // 2
        for idx in range(ngroups // 2):
            cur_col += group_temp.num_cols
            tx_groups.append(self.add_tile(group_temp, 0, cur_col, flip_lr=True))
            cur_col += min_sep
        for idx in range(ngroups // 2, ngroups):
            tx_groups.append(self.add_tile(group_temp, 0, cur_col))
            cur_col += group_temp.num_cols + min_sep

        # Get n and p side groups, assume even number of groups
        tx_groups_n, tx_groups_p = tx_groups[::2], tx_groups[1::2]

        m_tail_s = self.get_group_node(tx_groups, 'tail', 's')
        m_tail_g = self.get_group_node(tx_groups, 'tail', 'g')
        m_tail_d = self.get_group_node(tx_groups, 'tail', 'd')

        self.set_mos_size(ncol_tot)

        # Make new segment dictionary, update based on number of groups
        seg_dict_new = {'tail': seg_tail}
        for k, v in seg_dict_group.items():
            seg_dict_new[k] = v * ngroups // 2

        # ===== Routings =====

        # ----- hm layer -----
        # clk hm
        clk_hm_tid = self.get_track_id(ridx_dict['tail'], MOSWireType.G, 'clk', 0)
        if split_ck:
            ck_l_hm = self.connect_to_tracks(m_tail_g[0], clk_hm_tid)
            ck_r_hm = self.connect_to_tracks(m_tail_g[1], clk_hm_tid)
            clk_hm = None
        else:
            clk_hm = self.connect_to_tracks(m_tail_g, clk_hm_tid)
            ck_l_hm, ck_r_hm = None, None

        # tail node hm
        groups_tail = self.get_group_node(tx_groups, 'nfb' if flip_np else 'pfb', 'd')

        if split_ck:
            tail_n_hm = [self.connect_to_tracks(
                self.get_group_node(tx_groups_n, 'nfb' if flip_np else 'pfb', 'd') + [m_tail_d[0]],
                self.get_track_id(ridx_dict['nfb'] if flip_np else ridx_dict['pfb'], MOSWireType.DS, 'sig', 0))]
            tail_p_hm = [self.connect_to_tracks(
                self.get_group_node(tx_groups_p, 'nfb' if flip_np else 'pfb', 'd') + [m_tail_d[1]],
                self.get_track_id(ridx_dict['nfb'] if flip_np else ridx_dict['pfb'], MOSWireType.DS, 'sig', 0))]
            tail_hm = None
        else:
            tail_hm = self.connect_to_tracks(groups_tail + m_tail_d,
                                             self.get_track_id(ridx_dict['nfb'] if flip_np else ridx_dict['pfb'],
                                                               MOSWireType.DS, 'sig', 0))
            tail_n_hm, tail_p_hm = [], []

        # input pair
        inn = self.connect_to_tracks(self.get_group_node(tx_groups_n, 'in', 'g'),
                                     self.get_track_id(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='ana_sig'))
        inp = self.connect_to_tracks(self.get_group_node(tx_groups_p, 'in', 'g'),
                                     self.get_track_id(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='ana_sig'))

        # output pair
        outn_tidx, hm_w = self.get_track_info(ridx_dict['nfb'] if flip_np else ridx_dict['pfb'],
                                              MOSWireType.DS, wire_name='sig', wire_idx=1)
        outn_d0 = self.connect_to_tracks(self.get_group_node(tx_groups_n, 'nfb' if flip_np else 'pfb', 's'),
                                         TrackID(hm_layer, outn_tidx, width=hm_w))
        outp_d0 = self.connect_to_tracks(self.get_group_node(tx_groups_p, 'nfb' if flip_np else 'pfb', 's'),
                                         TrackID(hm_layer, outn_tidx, width=hm_w))

        outn_tidx = self.get_track_index(ridx_dict['nfb'] if flip_np else ridx_dict['pfb'],
                                         MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        outp_tidx = self.get_track_index(ridx_dict['pfb'] if flip_np else ridx_dict['nfb'],
                                         MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        outn_nfb, outp_nfb = self.connect_differential_tracks(
            self.get_group_node(tx_groups_n, 'nfb', 'g') + self.get_group_node(tx_groups_n, 'pfb', 'g'),
            self.get_group_node(tx_groups_p, 'nfb', 'g') + self.get_group_node(tx_groups_p, 'pfb', 'g'), hm_layer,
            outp_tidx, outn_tidx, width=hm_w)

        outn_tidx = self.get_track_index(ridx_dict['pfb'] if flip_np else ridx_dict['nfb'],
                                         MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)

        outn_d1 = self.connect_to_tracks(self.get_group_node(tx_groups_n, 'pfb' if flip_np else 'nfb', 'd'),
                                         TrackID(hm_layer, outn_tidx, width=hm_w))
        outp_d1 = self.connect_to_tracks(self.get_group_node(tx_groups_p, 'pfb' if flip_np else 'nfb', 'd'),
                                         TrackID(hm_layer, outn_tidx, width=hm_w))
        if has_rst:
            inn_m = self.connect_to_tracks(self.get_group_node(tx_groups_n, 'rst', 'g'),
                                           self.get_track_id(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='ana_sig'))
            inp_m = self.connect_to_tracks(self.get_group_node(tx_groups_p, 'rst', 'g'),
                                           self.get_track_id(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='ana_sig'))
            rst_ds_tidx = self.get_track_id(ridx_dict['rst'], MOSWireType.DS, wire_name='sig', wire_idx=0)
            tail_n_hm.append(self.connect_to_tracks(self.get_group_node(tx_groups_n, 'rst', 'd'), rst_ds_tidx))
            tail_p_hm.append(self.connect_to_tracks(self.get_group_node(tx_groups_p, 'rst', 'd'), rst_ds_tidx))
        else:
            inn_m, inp_m = None, None
        # dummy metal fill
        if self.params['fill_dummy']:
            tx_name_list = list(seg_dict.keys())
            for k in tx_name_list:
                fill_conn_layer_intv(self, 0, ridx_dict[k], extend_to_gate=False)

        # ===== add pins =====
        # - conn
        if flip_np:
            self.add_pin('VDD', self.get_group_node(tx_groups, 'in', 's') + self.get_group_node(tx_groups, 'pfb', 's'),
                         connect=True)
            self.add_pin('VSS', m_tail_s, connect=True)
        else:
            self.add_pin('VSS', self.get_group_node(tx_groups, 'in', 's') + self.get_group_node(tx_groups, 'nfb', 's'),
                         connect=True)
            self.add_pin('VDD', m_tail_s, connect=True)

        # - hm
        if split_ck:
            self.add_pin('inn', [inn, ck_l_hm])
            self.add_pin('inp', [inp, ck_r_hm])
            self.add_pin('tailn', tail_n_hm, hide=True)
            self.add_pin('tailp', tail_p_hm, hide=True)
        else:
            self.add_pin('clk_hm', clk_hm, label='clk', connect=True)
            self.add_pin('inn', inn)
            self.add_pin('inp', inp)
            self.add_pin('tail', tail_hm, hide=True)
        if has_rst:
            self.add_pin('inn_m', inn_m)
            self.add_pin('inp_m', inp_m)
        self.add_pin('outn', [outp_nfb, outn_d0, outn_d1], connect=True)
        self.add_pin('outp', [outn_nfb, outp_d0, outp_d1], connect=True)
        #
        # self.add_pin('VDD', cas_sup, connect=True)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict_new,
            w_dict=w_dict,
            th_dict=th_dict,
            flip_np=not flip_np,
            split_ck=split_ck,
            has_rst=has_rst,
        )


class StrongArmMatch(SingleEndTxGroup):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._has_ofst = False

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_strongarm_core')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_dict='bottom nmos row index.',
            conn_pair_list='Pair of connections',
            sig_locs='Optional dictionary of user defined signal locations',
            ndum='Number of dummy at sides',
            ngroups='Number of groups',
            fill_dummy='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            ridx_dict={},
            conn_pair_list=[],
            fill_dummy=False,
            ndum=0,
            ngroups=2,
        )

    @property
    def has_ofst(self) -> bool:
        return self._has_ofst

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        ngroups = self.params['ngroups']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']

        # Setup tracks
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())

        # max_seg = max(seg_dict.values()) + 2*ndum
        ridx_dict_group = ridx_dict.copy(remove=['tail', 'swm'])
        seg_dict_group = seg_dict.copy(remove=['tail', 'swm']).to_dict()
        w_dict_group = w_dict.copy(remove=['tail', 'swm'])
        seg_tail = seg_dict['tail']

        group_params = dict(
            pinfo=self.params['pinfo'],
            seg_dict=seg_dict_group,
            w_dict=w_dict_group,
            ridx_dict=ridx_dict_group,
            conn_pair_list=self.params['conn_pair_list'],
            ndum=0,
        )

        group_temp: MOSBase = self.new_template(params=group_params, temp_cls=SingleEndTxGroup)
        if ngroups & 1 or seg_tail % 2 != 0:
            raise ValueError('must have even number groups')

        # placement

        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        if seg_dict['swm'] + seg_dict['swo'] > min_sep + seg_dict['pfb']:
            raise ValueError("sar_comp_match/SenseAmpMatch: currently assume swm+so<min_sep+pfb")

        group_ncol = group_temp.num_cols
        ncol_group_tot = group_ncol * ngroups + min_sep * (ngroups - 1)

        seg_os = seg_dict.get('os', 0)
        self._has_ofst = bool(seg_os)

        ncol_tail_tot = seg_tail + 2 * (seg_os + min_sep)
        ncol_tot = max(ncol_group_tot, ncol_tail_tot)

        tail_ncol = (ncol_tot - seg_tail) // 2
        if tail_ncol & 1:
            tail_ncol -= 1
            m_tail_l = self.add_mos(ridx_dict['tail'], tail_ncol, w=w_dict['tail'], seg=seg_dict['tail'] // 2)
            m_tail_r = self.add_mos(ridx_dict['tail'], ncol_tot - tail_ncol, w=w_dict['tail'],
                                    seg=seg_dict['tail'] // 2,
                                    flip_lr=True)
            m_tail_s = [m_tail_l.s, m_tail_r.s]
            m_tail_d = [m_tail_l.d, m_tail_r.d]
            m_tail_g = [m_tail_l.g, m_tail_r.g]

        else:
            m_tail = self.add_mos(ridx_dict['tail'], (ncol_tot - seg_tail) // 2, w=w_dict['tail'], seg=seg_dict['tail'])
            m_tail_s, m_tail_d, m_tail_g = [m_tail.s], [m_tail.d], [m_tail.g]

        tx_groups = []
        swm_list = []
        cur_col = (ncol_tot - ncol_group_tot) // 2
        for idx in range(ngroups // 2):
            cur_col += group_temp.num_cols
            tx_groups.append(self.add_tile(group_temp, 0, cur_col, flip_lr=True))
            swm_list.append(self.add_mos(ridx_dict['swm'], cur_col - seg_dict['swo'], w=w_dict['swm'],
                                         seg=seg_dict['swm'], flip_lr=True))
            cur_col += min_sep
        for idx in range(ngroups // 2, ngroups):
            tx_groups.append(self.add_tile(group_temp, 0, cur_col))
            swm_list.append(self.add_mos(ridx_dict['swm'], cur_col + seg_dict['swo'], w=w_dict['swm'],
                                         seg=seg_dict['swm'], flip_lr=False))
            cur_col += group_temp.num_cols + min_sep

        # Get n and p side groups, assume even number of groups
        tx_groups_n, tx_groups_p = tx_groups[::2], tx_groups[1::2]

        self.set_mos_size(ncol_tot)

        # Make new segment dictionary, update based on number of groups
        seg_dict_new = {'tail': seg_tail}
        seg_dict_group['swm'] = seg_dict['swm']
        for k, v in seg_dict_group.items():
            seg_dict_new[k] = v * ngroups // 2
        #
        # ===== Routings =====

        # ----- hm layer -----
        # clk hm
        clk_hm_bot_tid = self.get_track_id(ridx_dict['tail'], MOSWireType.G, 'clk', 0)
        clk_hm_top_tid = self.get_track_id(ridx_dict['swm'], MOSWireType.G, 'clk', 0)
        clk_bot_hm = self.connect_to_tracks(m_tail_g, clk_hm_bot_tid)
        clk_top_hm = self.connect_to_tracks(self.get_group_node(tx_groups, 'swo', 'g') + [tx.g for tx in swm_list],
                                            clk_hm_top_tid)

        # tail node hm
        groups_in_d = self.get_group_node(tx_groups, 'in', 'd')
        _tail_hm = self.connect_to_tracks(groups_in_d + m_tail_d,
                                          self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', 0))

        # input pair
        inp_tidx, hm_w = self.get_track_info(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=1)
        inn_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        inn, inp = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'in', 'g'),
                                                    self.get_group_node(tx_groups_p, 'in', 'g'),
                                                    hm_layer, inp_tidx, inn_tidx, width=hm_w)

        # output pair
        outn_tidx = self.get_track_index(ridx_dict['nfb'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=0)
        outp_tidx = self.get_track_index(ridx_dict['nfb'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)
        outp_nfb, outn_nfb = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'nfb', 'd'),
                                                              self.get_group_node(tx_groups_p, 'nfb', 'd'), hm_layer,
                                                              outp_tidx, outn_tidx, width=hm_w)

        outn_tidx = self.get_track_index(ridx_dict['pfb'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=0)
        outp_tidx = self.get_track_index(ridx_dict['pfb'], MOSWireType.DS_MATCH, wire_name='sig', wire_idx=1)
        outp_pfb, outn_pfb = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'pfb', 'd'),
                                                              self.get_group_node(tx_groups_p, 'pfb', 'd'),
                                                              hm_layer, outp_tidx, outn_tidx, width=hm_w)

        outn_tidx = self.get_track_index(ridx_dict['nfb'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=0)
        outp_tidx = self.get_track_index(ridx_dict['nfb'], MOSWireType.G_MATCH, wire_name='sig', wire_idx=1)
        outp_g, outn_g = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'pfb', 'g') +
                                                          self.get_group_node(tx_groups_n, 'nfb', 'g'),
                                                          self.get_group_node(tx_groups_p, 'pfb', 'g')
                                                          + self.get_group_node(tx_groups_p, 'nfb', 'g'),
                                                          hm_layer, outp_tidx, outn_tidx, width=hm_w)

        # mid pair
        midn_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS, wire_name='sig', wire_idx=0)
        midp_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS, wire_name='sig', wire_idx=1)
        midp, midn = self.connect_differential_tracks(self.get_group_node(tx_groups_n, 'in', 's'),
                                                      self.get_group_node(tx_groups_p, 'in', 's'), hm_layer,
                                                      midp_tidx, midn_tidx, width=hm_w)
        mid_vm_list = []
        for g, tx in zip(tx_groups, swm_list):
            mid_coord = (g.bound_box.xl + g.bound_box.xh) // 2
            mid_vm_tidx = self.grid.coord_to_track(vm_layer, mid_coord, mode=RoundMode.NEAREST)
            tx_hm = self.connect_to_tracks(tx.d, self.get_track_id(ridx_dict['swm'], MOSWireType.DS, 'sig', 0))
            tx_vm = self.connect_to_tracks(tx_hm, TrackID(vm_layer, mid_vm_tidx, tr_manager.get_width(vm_layer, 'sig')))
            mid_vm_list.append(tx_vm)

        midn = self.connect_to_track_wires(midn, mid_vm_list[::2])
        midp = self.connect_to_track_wires(midp, mid_vm_list[1::2])
        mid_lower = min([w.lower for w in midn + midp])
        midn = self.extend_wires(midn, lower=mid_lower)
        midp = self.extend_wires(midp, lower=mid_lower)
        # dummy metal fill
        tx_name_list = list(seg_dict.keys())
        for k in tx_name_list:
            fill_conn_layer_intv(self, 0, ridx_dict[k], extend_to_gate=False)

        # ===== add pins =====
        # - conn
        self.add_pin('VDD', self.get_group_node(tx_groups, 'swo', 's') + self.get_group_node(tx_groups, 'pfb', 's') +
                     [tx.s for tx in swm_list], connect=True)
        self.add_pin('VSS', m_tail_s, connect=True)

        # - hm
        self.add_pin('clk', [clk_top_hm, clk_bot_hm], label='clk', connect=True)
        self.add_pin('inn', inn)
        self.add_pin('inp', inp)
        self.add_pin('midn', midn)
        self.add_pin('midp', midp)
        self.add_pin('outp', [outp_nfb, outp_pfb, outn_g], connect=True)
        self.add_pin('outn', [outn_nfb, outn_pfb, outp_g], connect=True)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict_new,
            w_dict=w_dict,
            th_dict=th_dict,
        )


class StrongArmWrap(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_strongarm_core')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = StrongArmMatch.get_params_info()
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = StrongArmMatch.get_default_param_values()
        return ans

    def draw_layout(self):
        master_params = self.params.copy().to_dict()
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        master_pinfo = self.get_draw_base_sub_pattern(1, 2)
        master_params['pinfo'] = master_pinfo
        master: StrongArmMatch = self.new_template(StrongArmMatch, params=master_params)

        core = self.add_tile(master, 1, 0)
        self.set_mos_size()

        # routing
        ncols, nrows = master.num_cols, master.num_rows
        ntap = self.add_substrate_contact(0, 0, seg=ncols, tile_idx=2, port_mode=SubPortMode.EVEN)
        ptap = self.add_substrate_contact(0, 0, seg=ncols, tile_idx=0, port_mode=SubPortMode.EVEN)

        core_vss = core.get_all_port_pins('VSS', layer=self.conn_layer)
        core_vdd = core.get_all_port_pins('VDD', layer=self.conn_layer)
        vss = self.connect_to_tracks([ptap] + core_vss, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0))
        vdd = self.connect_to_tracks([ntap] + core_vdd, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2))

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'ana_sig')
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        core_center_coord = (self.bound_box.xl + self.bound_box.xh) // 2
        _, tid_locs = tr_manager.place_wires(vm_layer, ['sig', 'sig', 'sup', 'clk', 'sup', 'sig', 'sig'],
                                             center_coord=core_center_coord)

        clk_vm = self.connect_to_tracks(core.get_all_port_pins('clk'), TrackID(vm_layer, tid_locs[3], tr_w_clk_vm))

        available_vm_locs = self.get_available_tracks(vm_layer,
                                                      self.grid.coord_to_track(vm_layer, 0, mode=RoundMode.NEAREST),
                                                      self.grid.coord_to_track(vm_layer, self.bound_box.xh,
                                                                               mode=RoundMode.NEAREST),
                                                      lower=self.bound_box.yl, upper=self.bound_box.yh,
                                                      width=tr_w_sig_vm,
                                                      include_last=True)
        available_vm_num = len(available_vm_locs)
        vdd_vm_shield = [
            self.connect_to_tracks(vdd, TrackID(vm_layer, available_vm_locs[available_vm_num // 2], tr_w_sup_vm)),
            self.connect_to_tracks(vdd, TrackID(vm_layer, available_vm_locs[available_vm_num // 2 - 1], tr_w_sup_vm))]
        vdd_vm_shield = self.extend_wires(vdd_vm_shield, lower=self.bound_box.yl, upper=self.bound_box.yh)
        available_vm_locs = self.get_available_tracks(vm_layer,
                                                      self.grid.coord_to_track(vm_layer, 0, mode=RoundMode.NEAREST),
                                                      self.grid.coord_to_track(vm_layer, self.bound_box.xh,
                                                                               mode=RoundMode.NEAREST),
                                                      lower=self.bound_box.yl, upper=self.bound_box.yh,
                                                      width=tr_w_sig_vm,
                                                      include_last=True)
        available_vm_num = len(available_vm_locs)
        outp, outn = self.connect_differential_tracks(core.get_all_port_pins('outp'),
                                                      core.get_all_port_pins('outn'), vm_layer,
                                                      available_vm_locs[available_vm_num // 2],
                                                      available_vm_locs[available_vm_num // 2 - 1],
                                                      width=tr_w_sig_vm)
        inp, inn = self.connect_differential_tracks(core.get_all_port_pins('inp'),
                                                    core.get_all_port_pins('inn'), vm_layer,
                                                    available_vm_locs[available_vm_num // 2 + 1],
                                                    available_vm_locs[available_vm_num // 2 - 2],
                                                    width=tr_w_sig_vm)
        self.add_pin('clk', clk_vm, show=self.show_pins)
        self.add_pin('VSS', vss, show=self.show_pins)
        self.add_pin('VDD', vdd, show=self.show_pins)
        self.add_pin('inp', inp, show=self.show_pins)
        self.add_pin('inn', inn, show=self.show_pins)
        self.add_pin('outp', outp, show=self.show_pins)
        self.add_pin('outn', outn, show=self.show_pins)

        self.add_pin('VDD_shield', vdd_vm_shield, hide=True)

        self.reexport(core.get_port('midp'))
        self.reexport(core.get_port('midn'))
        self.sch_params = master.sch_params


class TriTailWrap(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_tri_tail_core')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            ncols_tot='Total number of cols',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_dict='Number of rows in each sub-blocks',
            ngroups_dict='Number of groups in each sub-blocks',
            vertical_out='True to connect outputs to vm_layer.',
            vm_sup='True to connect supply to vm_layer.',
            even_center='True to force center column to be even.',
            signal_locs='Signal locations',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ncols_tot=0,
            vertical_out=True,
            vm_sup=False,
            even_center=False,
            signal_locs={},
        )

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        seg_dict: Dict[str, Dict] = self.params['seg_dict']
        w_dict: Dict[str, Dict] = self.params['w_dict']
        ridx_dict: Dict[str, Dict] = self.params['ridx_dict']
        ngroups_dict: Dict[str, Dict] = self.params['ngroups_dict']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        # Make templates
        preamp_params = dict(pinfo=self.get_tile_pinfo(1), fill_dummy=False,
                             ridx_dict=ridx_dict['preamp'], ngroups=ngroups_dict['preamp'],
                             seg_dict=seg_dict['preamp'], w_dict=w_dict['preamp'])
        half_latch_params = dict(pinfo=self.get_tile_pinfo(3), fill_dummy=False,
                                 ridx_dict=ridx_dict['half_latch'], ngroups=ngroups_dict['half_latch'],
                                 seg_dict=seg_dict['half_latch'], w_dict=w_dict['half_latch'])
        dyn_latch_params = dict(pinfo=self.get_tile_pinfo(5), fill_dummy=False,
                                ridx_dict=ridx_dict['dyn_latch'], ngroups=ngroups_dict['dyn_latch'],
                                seg_dict=seg_dict['dyn_latch'], w_dict=w_dict['dyn_latch'])

        preamp_master = self.new_template(PreAmpMatch, params=preamp_params)
        half_latch_master = self.new_template(HalfLatchMatch, params=half_latch_params)
        dyn_latch_master = self.new_template(DynLatchMatch, params=dyn_latch_params)

        # floorplanning
        preamp_ncol = preamp_master.num_cols
        half_latch_ncol = half_latch_master.num_cols
        dyn_latch_ncol = dyn_latch_master.num_cols

        tot_ncol_predefined = self.params['ncols_tot']
        tot_ncol = max(preamp_ncol, half_latch_ncol, dyn_latch_ncol, tot_ncol_predefined)

        # placement
        preamp = self.add_tile(preamp_master, 1, (tot_ncol - preamp_ncol) // 2)
        half_latch = self.add_tile(half_latch_master, 3, (tot_ncol - half_latch_ncol) // 2)
        dyn_latch = self.add_tile(dyn_latch_master, 5, (tot_ncol - dyn_latch_ncol) // 2)

        # Add substrate connection
        # port_mode = SubPortMode.ODD if ((tot_ncol - seg_dict['preamp']['tail'])//2) & 1 else SubPortMode.EVEN
        port_mode = SubPortMode.EVEN if tot_ncol % 4 != 0 else SubPortMode.ODD
        ptap0 = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=0, port_mode=port_mode)
        ntap0 = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=2, port_mode=port_mode)
        ptap1 = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=4, port_mode=port_mode)
        ntap1 = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=6, port_mode=port_mode)
        self.set_mos_size()

        preamp_p_dum, preamp_n_dum = [], []
        half_latch_p_dum, half_latch_n_dum = [], []
        dyn_latch_p_dum, dyn_latch_n_dum = [], []

        # Fill preamp
        fill_and_collect(self, 1, preamp_p_dum, preamp_n_dum)
        fill_and_collect(self, 3, half_latch_p_dum, half_latch_n_dum)
        fill_and_collect(self, 5, dyn_latch_p_dum, dyn_latch_n_dum)

        # Fill half latch
        half_latch_p_dum = half_latch_p_dum[::-1]
        half_latch_n_dum = half_latch_n_dum[::-1]

        # Fill dyn latch

        connect_conn_dummy_rows(self, preamp_p_dum, connect_to_sup=True, sup_dum_idx=-1, sup_coord=ntap0[0].middle)
        connect_conn_dummy_rows(self, preamp_n_dum, connect_to_sup=True, sup_dum_idx=0, sup_coord=ptap0[0].middle)
        connect_conn_dummy_rows(self, half_latch_p_dum, connect_to_sup=True, sup_dum_idx=0, sup_coord=ntap0[0].middle)
        connect_conn_dummy_rows(self, half_latch_n_dum, connect_to_sup=True, sup_dum_idx=-1, sup_coord=ptap1[0].middle)
        connect_conn_dummy_rows(self, dyn_latch_p_dum, connect_to_sup=True, sup_dum_idx=-1, sup_coord=ntap1[0].middle)
        connect_conn_dummy_rows(self, dyn_latch_n_dum, connect_to_sup=True, sup_dum_idx=0, sup_coord=ptap1[0].middle)

        # Connect supplies
        vss0 = self.connect_to_tracks(ptap0, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=0))
        vss1 = self.connect_to_tracks(ptap1, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=4))
        vdd0 = self.connect_to_tracks(ntap0, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=2))
        vdd1 = self.connect_to_tracks(ntap1, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=6))
        vdd2 = self.extend_wires(preamp.get_pin('VDD', layer=self.conn_layer + 1), lower=self.bound_box.xl,
                                 upper=self.bound_box.xh)[0]
        self.connect_to_track_wires(vss0, preamp.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vdd0, preamp.get_all_port_pins('VDD', layer=self.conn_layer) +
                                    half_latch.get_all_port_pins('VDD'))
        self.connect_to_track_wires(vss1, dyn_latch.get_all_port_pins('VSS') +
                                    half_latch.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vdd1, dyn_latch.get_all_port_pins('VDD'))

        # -- Inter connection --
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'ana_sig')
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        # ---- clock signals ----
        _, vm_tidx_locs = tr_manager.place_wires(vm_layer, ['sig'] * 2 + ['sup'] + ['clk'] * 5 + ['sup'] + ['sig'] * 2,
                                                 center_coord=(self.bound_box.xh + self.bound_box.xl) // 2)
        clk_hm = preamp.get_all_port_pins('clk_hm') + dyn_latch.get_all_port_pins('clk_hm')
        clkb_hm = half_latch.get_all_port_pins('clkb_hm')
        clk_vm = [self.connect_to_tracks(clk_hm, TrackID(vm_layer, vm_tidx_locs[4], tr_w_clk_vm)),
                  self.connect_to_tracks(clk_hm, TrackID(vm_layer, vm_tidx_locs[-5], tr_w_clk_vm))]
        clkb_vm = [self.connect_to_tracks(clkb_hm, TrackID(vm_layer, vm_tidx_locs[3], tr_w_sig_vm)),
                   self.connect_to_tracks(clkb_hm, TrackID(vm_layer, vm_tidx_locs[-4], tr_w_clk_vm))]
        _sup_shield_vm = [self.connect_to_tracks([vdd0, vdd1, vdd2], TrackID(vm_layer, vm_tidx_locs[2], tr_w_sup_vm)),
                          self.connect_to_tracks([vdd0, vdd1, vdd2], TrackID(vm_layer, vm_tidx_locs[-3], tr_w_sup_vm))]

        # ---- in/out signals ----
        self.connect_differential_tracks(preamp.get_all_port_pins('outp') + half_latch.get_all_port_pins('inp'),
                                         preamp.get_all_port_pins('outn') + half_latch.get_all_port_pins('inn'),
                                         vm_layer, vm_tidx_locs[1], vm_tidx_locs[-2], width=tr_w_sig_vm)
        self.connect_differential_tracks(half_latch.get_all_port_pins('outp') + dyn_latch.get_all_port_pins('inp'),
                                         half_latch.get_all_port_pins('outn') + dyn_latch.get_all_port_pins('inn'),
                                         vm_layer, vm_tidx_locs[0], vm_tidx_locs[-1], width=tr_w_sig_vm)
        inp, inn = self.connect_differential_tracks(preamp.get_all_port_pins('inp'),
                                                    preamp.get_all_port_pins('inn'),
                                                    vm_layer, vm_tidx_locs[-1], vm_tidx_locs[0], width=tr_w_sig_vm)
        outp, outn = self.connect_differential_tracks(dyn_latch.get_all_port_pins('outp'),
                                                      dyn_latch.get_all_port_pins('outn'),
                                                      vm_layer, vm_tidx_locs[1], vm_tidx_locs[-2], width=tr_w_sig_vm)
        self.add_pin('clk', clk_vm)
        self.add_pin('clkb', clkb_vm)

        if self.params['vm_sup']:
            sup_vm_locs_l = self.arr_info.col_to_track(vm_layer, 0, mode=RoundMode.GREATER_EQ)
            _, sup_vm_locs_l = tr_manager.place_wires(vm_layer, ['sup', 'sup'], align_idx=0,
                                                      align_track=sup_vm_locs_l)
            sup_vm_locs_r = self.arr_info.col_to_track(vm_layer, self.num_cols, mode=RoundMode.LESS_EQ)
            _, sup_vm_locs_r = tr_manager.place_wires(vm_layer, ['sup', 'sup'], align_idx=-1,
                                                      align_track=sup_vm_locs_r)
            # sup_vm_locs = sup_vm_locs_l + sup_vm_locs_r
            tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
            vssr, vddr = self.connect_matching_tracks([[vss0, vss1], [vdd0, vdd1, vdd2]], vm_layer,
                                                      [sup_vm_locs_r[0], sup_vm_locs_r[1]], width=tr_w_sup_vm)
            vssl, vddl = self.connect_matching_tracks([[vss0, vss1], [vdd0, vdd1, vdd2]], vm_layer,
                                                      [sup_vm_locs_l[0], sup_vm_locs_l[1]], width=tr_w_sup_vm)

            self.add_pin('VSS', [vssr, vssl], connect=True)
            self.add_pin('VDD', [vddr, vddl], connect=True)
            self.add_pin('VSS_hm', [vss0, vss1], label='VSS', connect=True)
            self.add_pin('VDD_hm', [vdd0, vdd1, vdd2], label='VDD', connect=True)
        else:
            self.add_pin('VSS', [vss0, vss1], connect=True)
            self.add_pin('VDD', [vdd0, vdd1], connect=True)

        self.add_pin('inn', inn)
        self.add_pin('inp', inp)

        self.add_pin('outn', outn)
        self.add_pin('outp', outp)

        if preamp_master.has_ofst:
            self.reexport(preamp.get_port('osp'))
            self.reexport(preamp.get_port('osn'))

        self.sch_params = dict(
            preamp=preamp_master.sch_params,
            half_latch=half_latch_master.sch_params,
            dyn_latch=dyn_latch_master.sch_params,
        )


class DoubleTailSelfTimeWrap(MOSBase, TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._out_col = []

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_double_tail_selftime_core')

    @property
    def out_col(self):
        return self._out_col

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            ncols_tot='Total number of cols',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_dict='Number of rows in each sub-blocks',
            ngroups_dict='Number of groups in each sub-blocks',
            vertical_out='True to connect outputs to vm_layer.',
            vm_sup='True to connect supply to vm_layer.',
            even_center='True to force center column to be even.',
            signal_locs='Signal locations',
            flip_np='False to use pmos input',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ncols_tot=0,
            vertical_out=True,
            vm_sup=False,
            even_center=False,
            signal_locs={},
            flip_np=True,
        )

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        seg_dict: Dict[str, Dict] = self.params['seg_dict']
        w_dict: Dict[str, Dict] = self.params['w_dict']
        ridx_dict: Dict[str, Dict] = self.params['ridx_dict']
        ngroups_dict: Dict[str, Dict] = self.params['ngroups_dict']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        flip_np = self.params['flip_np']
        # Make templates
        preamp_params = dict(pinfo=self.get_tile_pinfo(1), fill_dummy=False, has_cas_dev=False,
                             ridx_dict=ridx_dict['preamp'], ngroups=ngroups_dict['preamp'],
                             seg_dict=seg_dict['preamp'], w_dict=w_dict['preamp'], flip_np=flip_np)
        dyn_latch_params = dict(pinfo=self.get_tile_pinfo(3), fill_dummy=False, flip_np=self.params['flip_np'],
                                split_ck=True, ridx_dict=ridx_dict['dyn_latch'], ngroups=ngroups_dict['dyn_latch'],
                                seg_dict=seg_dict['dyn_latch'], w_dict=w_dict['dyn_latch'])

        preamp_master = self.new_template(PreAmpMatch, params=preamp_params)
        dyn_latch_master = self.new_template(DynLatchMatch, params=dyn_latch_params)

        # floorplanning
        preamp_ncol = preamp_master.num_cols
        dyn_latch_ncol = dyn_latch_master.num_cols

        tot_ncol_predefined = self.params['ncols_tot']
        tot_ncol = max(preamp_ncol, dyn_latch_ncol, tot_ncol_predefined)

        # placement
        preamp = self.add_tile(preamp_master, 1, (tot_ncol - preamp_ncol) // 2)
        dyn_latch = self.add_tile(dyn_latch_master, 3, (tot_ncol - dyn_latch_ncol) // 2)

        # Add substrate connection
        tap0_conn = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=0, port_mode=SubPortMode.BOTH)
        tap1_conn = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=2, port_mode=SubPortMode.BOTH)
        tap2_conn = self.add_substrate_contact(0, 0, seg=tot_ncol, tile_idx=4, port_mode=SubPortMode.BOTH)
        if tot_ncol % 4 != 0:
            tap0_conn, tap1_conn, tap2_conn = tap0_conn[1::2], tap1_conn[::2], tap2_conn[::2]
        else:
            tap0_conn, tap1_conn, tap2_conn = tap0_conn[::2], tap1_conn[1::2], tap2_conn[1::2]
        self.set_mos_size()
        #
        preamp_p_dum, preamp_n_dum = [], []
        # half_latch_p_dum, half_latch_n_dum = [], []
        dyn_latch_p_dum, dyn_latch_n_dum = [], []
        #
        # Fill preamp
        fill_and_collect(self, 1, preamp_p_dum, preamp_n_dum)
        # fill_and_collect(self, 3, half_latch_p_dum, half_latch_n_dum)
        fill_and_collect(self, 3, dyn_latch_p_dum, dyn_latch_n_dum)
        #
        # # Fill half latch
        # half_latch_p_dum = half_latch_p_dum[::-1]
        # half_latch_n_dum = half_latch_n_dum[::-1]
        #
        # Fill dyn latch
        connect_conn_dummy_rows(self, preamp_p_dum, connect_to_sup=True, sup_dum_idx=-1,
                                sup_coord=tap0_conn[0].middle if flip_np else tap1_conn[0].middle)
        connect_conn_dummy_rows(self, preamp_n_dum, connect_to_sup=True, sup_dum_idx=0,
                                sup_coord=tap1_conn[0].middle if flip_np else tap0_conn[0].middle)
        connect_conn_dummy_rows(self, dyn_latch_p_dum[::-1], connect_to_sup=True, sup_dum_idx=0,
                                sup_coord=tap2_conn[0].middle if flip_np else tap1_conn[0].middle)
        connect_conn_dummy_rows(self, dyn_latch_n_dum[::-1], connect_to_sup=True, sup_dum_idx=-1,
                                sup_coord=tap1_conn[0].middle if flip_np else tap2_conn[0].middle)

        # Connect supplies
        tap0 = self.connect_to_tracks(tap0_conn, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=0))
        tap1 = self.connect_to_tracks(tap1_conn, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=2))
        tap2 = self.connect_to_tracks(tap2_conn, self.get_track_id(0, MOSWireType.DS, 'sup', 0, tile_idx=4))
        self.connect_to_track_wires(tap1 if flip_np else tap0, preamp.get_all_port_pins('VSS', layer=self.conn_layer))
        self.connect_to_track_wires(tap0 if flip_np else tap1, preamp.get_all_port_pins('VDD', layer=self.conn_layer))
        self.connect_to_track_wires(tap1 if flip_np else tap2, dyn_latch.get_all_port_pins('VSS'))
        self.connect_to_track_wires(tap2 if flip_np else tap1, dyn_latch.get_all_port_pins('VDD'))

        # -- Inter connection --
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'ana_sig')
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        # ---- clock signals ----
        # In the middle there should be clk, but here it use sup type to leave space for wide clk input
        # clk_vm = [self.connect_to_tracks(clk_hm, TrackID(vm_layer, vm_tidx_locs[3], tr_w_clk_vm)),
        #           self.connect_to_tracks(clk_hm, TrackID(vm_layer, vm_tidx_locs[-4], tr_w_clk_vm))]

        _, vm_tidx_locs = tr_manager.place_wires(vm_layer, ['ana_sig'] * 2 + ['sup'] + ['ana_sig', 'dum', 'ana_sig'] +
                                                 ['sup'] + ['ana_sig'] * 2,
                                                 center_coord=(self.bound_box.xh + self.bound_box.xl) // 2)

        clk_hm = preamp.get_all_port_pins('clk_hm') + dyn_latch.get_all_port_pins('clk_hm')
        clk_vm_sp = self.get_track_sep(vm_layer, tr_w_clk_vm, tr_w_clk_vm)
        clk_vm_sp_margin = self.get_track_sep(vm_layer, tr_w_sup_vm, tr_w_clk_vm)
        clk_tids = self.get_tids_between(vm_layer, vm_tidx_locs[2], vm_tidx_locs[-3],
                                         tr_w_clk_vm, clk_vm_sp, clk_vm_sp_margin, False, mod=2)
        clk_vm = WireArray.list_to_warr([self.connect_to_tracks(clk_hm, tid) for tid in clk_tids])
        sup_shield_vm = [self.connect_to_tracks([tap0], TrackID(vm_layer, vm_tidx_locs[2], tr_w_sup_vm)),
                         self.connect_to_tracks([tap0], TrackID(vm_layer, vm_tidx_locs[-3], tr_w_sup_vm))]
        if dyn_latch_master.sch_params['has_rst']:
            self.connect_to_tracks(dyn_latch.get_all_port_pins('tailn'),
                                   TrackID(vm_layer, vm_tidx_locs[2], tr_w_sup_vm))
            self.connect_to_tracks(dyn_latch.get_all_port_pins('tailp'),
                                   TrackID(vm_layer, vm_tidx_locs[-3], tr_w_sup_vm))
        self.extend_wires(sup_shield_vm, lower=self.bound_box.yl)

        connect_xm_clk = False
        if connect_xm_clk:
            xm_layer = vm_layer + 1
            tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
            # tr_sp_clk_xm = tr_manager.get_sep(xm_layer, ('clk', 'clk'))
            clk_xm_tids = self.get_tids_between(xm_layer,
                                                self.grid.coord_to_track(xm_layer, clk_vm.lower, RoundMode.GREATER),
                                                self.grid.coord_to_track(xm_layer, clk_vm.upper, RoundMode.LESS),
                                                tr_w_clk_xm, 0, 0, False)
            clk_xm = [self.connect_to_tracks(clk_vm, tid) for tid in clk_xm_tids]
            self.add_pin('clk_xm', clk_xm, label='clk')

        # ---- in/out signals ----
        self.connect_to_tracks(preamp.get_all_port_pins('outp') + dyn_latch.get_all_port_pins('inp'),
                               TrackID(vm_layer, vm_tidx_locs[-1], width=tr_w_sig_vm))
        self.connect_to_tracks(preamp.get_all_port_pins('outn') + dyn_latch.get_all_port_pins('inn'),
                               TrackID(vm_layer, vm_tidx_locs[0], width=tr_w_sig_vm))
        inp = self.connect_to_tracks(preamp.get_all_port_pins('inp'),
                                     TrackID(vm_layer, vm_tidx_locs[-2], width=tr_w_sig_vm),
                                     min_len_mode=MinLenMode.MIDDLE)
        inn = self.connect_to_tracks(preamp.get_all_port_pins('inn'),
                                     TrackID(vm_layer, vm_tidx_locs[1], width=tr_w_sig_vm),
                                     min_len_mode=MinLenMode.MIDDLE)

        outp = self.connect_to_tracks(dyn_latch.get_all_port_pins('outp'),
                                      TrackID(vm_layer, vm_tidx_locs[-2], width=tr_w_sig_vm))
        outn = self.connect_to_tracks(dyn_latch.get_all_port_pins('outn'),
                                      TrackID(vm_layer, vm_tidx_locs[1], width=tr_w_sig_vm))

        self.add_pin('clk', clk_vm)
        # self.add_pin('clkb', clkb_vm)
        vss_conn = [tap1] if flip_np else [tap0, tap2]
        vdd_conn = [tap0, tap2] if flip_np else [tap1]

        if self.params['vm_sup']:
            sup_vm_locs_l = self.arr_info.col_to_track(vm_layer, 0, mode=RoundMode.GREATER_EQ)
            _, sup_vm_locs_l = tr_manager.place_wires(vm_layer, ['sup', 'sup'], align_idx=0,
                                                      align_track=sup_vm_locs_l)
            sup_vm_locs_r = self.arr_info.col_to_track(vm_layer, self.num_cols, mode=RoundMode.LESS_EQ)
            _, sup_vm_locs_r = tr_manager.place_wires(vm_layer, ['sup', 'sup'], align_idx=-1,
                                                      align_track=sup_vm_locs_r)
            # sup_vm_locs = sup_vm_locs_l + sup_vm_locs_r
            tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
            vssr, vddr = self.connect_matching_tracks([vss_conn, vdd_conn], vm_layer,
                                                      [sup_vm_locs_r[0], sup_vm_locs_r[1]], width=tr_w_sup_vm)
            vssl, vddl = self.connect_matching_tracks([vss_conn, vdd_conn], vm_layer,
                                                      [sup_vm_locs_l[0], sup_vm_locs_l[1]], width=tr_w_sup_vm)

            self.add_pin('VSS', [vssr, vssl], connect=True)
            self.add_pin('VDD', [vddr, vddl], connect=True)
            self.add_pin('VSS_hm', vss_conn, label='VSS', connect=True)
            self.add_pin('VDD_hm', vdd_conn, label='VDD', connect=True)
        else:
            self.add_pin('VSS', vss_conn, connect=True)
            self.add_pin('VDD', vdd_conn, connect=True)

        self.add_pin('inn', inn)
        self.add_pin('inp', inp)

        self.add_pin('outn', outn)
        self.add_pin('outp', outp)

        tr_w_bias_vm = tr_manager.get_width(vm_layer, 'bias')
        if preamp_master.has_ofst:
            # self.reexport(preamp.get_port('osp'))
            # self.reexport(preamp.get_port('osn'))
            osn_vm_tidx = tr_manager.get_next_track(vm_layer, vm_tidx_locs[0], 'ana_sig', 'bias', up=-1)
            osp_vm_tidx = tr_manager.get_next_track(vm_layer, vm_tidx_locs[-1], 'ana_sig', 'bias', up=1)
            osn_vm = self.connect_to_tracks(preamp.get_pin('osn'), TrackID(vm_layer, osn_vm_tidx, tr_w_bias_vm))
            osp_vm = self.connect_to_tracks(preamp.get_pin('osp'), TrackID(vm_layer, osp_vm_tidx, tr_w_bias_vm))
            self.add_pin('osn', osn_vm)
            self.add_pin('osp', osp_vm)

        tr_sp_sig_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        self._out_col = \
            [self.arr_info.track_to_col(vm_layer, vm_tidx_locs[-4] + tr_sp_sig_vm // 2, mode=RoundMode.GREATER_EQ),
             self.arr_info.track_to_col(vm_layer, vm_tidx_locs[3] - tr_sp_sig_vm // 2, mode=RoundMode.LESS_EQ)]

        self.sch_params = dict(
            preamp=preamp_master.sch_params,
            dyn_latch=dyn_latch_master.sch_params,
        )
