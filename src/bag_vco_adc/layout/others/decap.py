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


from typing import Any, Dict, Type, Optional

from bag.design.database import ModuleDB, Module
from bag.layout.routing import TrackID
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param
from bag_vco_adc.layout.util.template import TemplateBaseZL
from pybag.core import Transform, BBox
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBase
from xbase.layout.mos.placement.data import MOSBasePlaceInfo, MOSArrayPlaceInfo
from ..util.wrapper import GenericWrapper


class DecapUnit(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            min_height='Height to match capdac',
            pinfo='placement information object.',
            seg='segments dictionary.',
            w='widths.',
            ny='',
            type='np --both pmos and nmos, n -- only nmos, p -- only pmos',
            cap='Schematic cap value',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w=4,
            ny=1,
            min_height=0,
            cap=None
        )

    def make_row(self, w, seg, side_ncol, tile_idx, ridx, g_on_s):
        ptap = [self.add_substrate_contact(ridx, 0, seg=2, tile_idx=tile_idx),
                self.add_substrate_contact(ridx, side_ncol + seg + side_ncol, seg=2, tile_idx=tile_idx,
                                           flip_lr=True)]
        n = self.add_mos(ridx, side_ncol, seg, w=w, tile_idx=tile_idx, g_on_s=g_on_s)

        ns_tid = self.get_track_id(ridx, MOSWireType.DS, 'sup', 0, tile_idx=tile_idx)
        ns = self.connect_to_tracks([n.s] + ptap, ns_tid)
        return n, ns, ns_tid

    def get_extra_hm_tids(self, hm_layer, ng_tid, pg_tid, ns_tid, ps_tid, tile_idx):

        if type == 'np':
            gtid_avail = self.get_tids_between(hm_layer, min(pg_tid.base_index, ng_tid.base_index),
                                               max(pg_tid.base_index, ng_tid.base_index), width=1, sep=1,
                                               sep_margin=1, include_last=True, mod=2)
        else:
            gtid_avail = [TrackID(hm_layer, (ng_tid.base_index+pg_tid.base_index)//2, 1)]
        nds_tid = self.get_tids_between(hm_layer, min(ng_tid.base_index, ns_tid.base_index),
                                        max(ng_tid.base_index, ns_tid.base_index), width=1, sep=1,
                                        sep_margin=1, include_last=True, mod=2)
        pds_tid = self.get_tids_between(hm_layer, min(ps_tid.base_index, pg_tid.base_index),
                                        max(ps_tid.base_index, pg_tid.base_index),
                                        width=1, sep=1, sep_margin=1, include_last=True, mod=2)
        return (gtid_avail, nds_tid, pds_tid) if not tile_idx & 1 else (gtid_avail[::-1], nds_tid[::-1], pds_tid[::-1])

    def draw_layout(self):
        """
        This generator assumes type match pinfo
        """
        ny: int = self.params['ny']
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        type = self.params['type']

        seg: int = self.params['seg']
        w: int = self.params['w']

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
        tr_sup_ym_w = tr_manager.get_width(ym_layer, 'sup')

        pin_lower = self.arr_info.col_to_coord(0)
        vdd_list, vss_list = [], []
        sup_bot_tid_list = []
        xm_tid_list = []
        tap_ncol = 2
        tap_sep_col = 2*self.sub_sep_col
        tap_ncol += tap_sep_col

        side_ncol = tap_ncol
        side_ncol += side_ncol & 1
        seg = seg - 2 * side_ncol
        # pds_list, nds_list, pg_list, ng_list = [], [], [], []
        self.set_mos_size(num_cols=seg + 2 * side_ncol, num_tiles=ny)

        vdd_list, vss_list = [], []
        for idx in range(ny):
            if type == 'np':
                n, ns, ns_tid = self.make_row(w, seg, side_ncol, idx, 0, True)
                p, ps, ps_tid = self.make_row(w, seg, side_ncol, idx, 1, False)
                ng_tid = self.get_track_id(0, MOSWireType.G, 'sup', 0, tile_idx=idx)
                pg_tid = self.get_track_id(1, MOSWireType.G, 'sup', 0, tile_idx=idx)
                # pg = self.connect_to_tracks(p.g, ng_tid)
                # ng = self.connect_to_tracks(n.g, pg_tid)
                gtid_avail, nds_tid, pds_tid = self.get_extra_hm_tids(hm_layer, ng_tid, pg_tid, ns_tid, ps_tid, idx)

                nd = [self.connect_to_tracks(n.d, tid) for tid in nds_tid[1::2]]
                pd = [self.connect_to_tracks(p.d, tid) for tid in pds_tid[::2]]
                pd += [self.add_wires(hm_layer, tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                      width=tid.width) for tid in nds_tid[::2]]
                nd += [self.add_wires(hm_layer, tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                      width=tid.width) for tid in pds_tid[1::2]]
                gtid_avail = [pg_tid] + [ng_tid]
                _ng_list = [self.connect_to_tracks(n.g, tid) for tid in gtid_avail[1::2]]
                _pg_list = [self.connect_to_tracks(p.g, tid) for tid in gtid_avail[::2]]
                self.connect_wires([n.g, p.s])
                self.connect_wires([p.g, n.d])
                # nds_list.extend([ns] + nd)
                # pds_list.extend([ps] + pd)
                # ng_list.extend(_ng_list)
                # pg_list.extend(_pg_list)
                vdd_list.extend([ps] + pd + _ng_list)
                vss_list.extend([ns] + nd + _pg_list)
            else:
                b, bs, bs_tid = self.make_row(w, seg, side_ncol, idx, 0, False)
                t, ts, ts_tid = self.make_row(w, seg, side_ncol, idx, 1, False)
                bg_tid = self.get_track_id(0, MOSWireType.G, 'sup', 0, tile_idx=idx)
                tg_tid = self.get_track_id(1, MOSWireType.G, 'sup', 0, tile_idx=idx)
                gtid_avail, nds_tid, pds_tid = self.get_extra_hm_tids(hm_layer, bg_tid, tg_tid, bs_tid, ts_tid, idx)
                pds_tid, nds_tid = pds_tid[::-1], nds_tid[::-1]
                nd = [self.connect_to_tracks(b.d, tid) for tid in nds_tid[::2]]
                pd = [self.connect_to_tracks(t.d, tid) for tid in pds_tid[1::2]]
                pg = [self.add_wires(hm_layer, tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                      width=tid.width) for tid in nds_tid[1::2]]
                ng = [self.add_wires(hm_layer, tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                      width=tid.width) for tid in pds_tid[::2]]
                g = [b.g, t.g]
                gtid_avail = [bg_tid] + gtid_avail + [tg_tid]
                g_list = [self.connect_to_tracks(g, tid) for tid in gtid_avail[::2]]
                ds_list = [self.add_wires(hm_layer, tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                          width=tid.width) for tid in gtid_avail[1::2]]
                _vdd_list = g_list + ng + pg
                _vss_list = ds_list + nd + pd + [bs, ts]
                if type == 'p':
                    _vdd_list, _vss_list = _vss_list, _vdd_list

                vdd_list.extend(_vdd_list)
                vss_list.extend(_vss_list)

        for sup in vdd_list + vss_list:
            self.extend_wires(sup, lower=self.bound_box.xl, upper=self.bound_box.xh)

        # # vm
        # vm_vdd_tidx_list = [self.arr_info.col_to_track(vm_layer, idx, RoundMode.NEAREST) for idx in
        #                     range(0, self.num_cols, 2)]
        # vm_vss_tidx_list = [self.arr_info.col_to_track(vm_layer, idx, RoundMode.NEAREST) for idx in
        #                     range(1, self.num_cols, 2)]
        #
        # vdd_vm = [self.connect_to_tracks(ng_list + pds_list, TrackID(vm_layer, tidx)) for tidx in vm_vdd_tidx_list]
        # vss_vm = [self.connect_to_tracks(pg_list + nds_list, TrackID(vm_layer, tidx)) for tidx in vm_vss_tidx_list]
        # vdd_vm = self.extend_wires(vdd_vm, lower=self.bound_box.yl, upper=self.bound_box.yh)
        # vss_vm = self.extend_wires(vss_vm, lower=self.bound_box.yl, upper=self.bound_box.yh)
        # self.add_pin('VDD', vdd_vm)
        # self.add_pin('VSS', vss_vm)
        #
        # # xm
        # xm_tidx_l = self.grid.coord_to_track(xm_layer, self.bound_box.yl, RoundMode.GREATER_EQ)
        # xm_tidx_h = self.grid.coord_to_track(xm_layer, self.bound_box.yh, RoundMode.LESS_EQ)
        # # Divide by 2 for via separation
        #
        # num_wires = tr_manager.get_num_wires_between(xm_layer, 'sup', xm_tidx_l, 'sup', xm_tidx_h, 'sup') + 2
        # tidx_list = tr_manager.spread_wires(xm_layer, ['sup'] * num_wires, xm_tidx_l, xm_tidx_h, ('sup', 'sup'),
        #                                     alignment=0)[1:-1]
        # tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
        # vdd_xm = [self.connect_to_tracks(vdd_vm, TrackID(xm_layer, tidx, tr_sup_xm_w)) for tidx in tidx_list[::2]]
        # vss_xm = [self.connect_to_tracks(vss_vm, TrackID(xm_layer, tidx, tr_sup_xm_w)) for tidx in tidx_list[1::2]]
        # vdd_xm = self.extend_wires(vdd_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        # vss_xm = self.extend_wires(vss_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        #
        # self.add_pin('VDD', vdd_xm)
        # self.add_pin('VSS', vss_xm)
        self.add_pin('VDD', vdd_list)
        self.add_pin('VSS', vss_list)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            w=w,
            seg=seg * ny,
            intent=self.get_row_info(0, 0).threshold,
            type=type,
            cap=self.params['cap']
        )


class DecapArray(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._actual_width = 0
        self._core = None

    def core(self):
        return self._core

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'decap_array')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            nx='',
            ny='',
            unit_params='',
            top_layer='Top supply layer',
            core_power='True to not extend power to edge'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            nx=1,
            ny=1,
            top_layer=6,
            core_pwoer=False,
        )

    def draw_layout(self) -> None:
        conn_layer = \
            MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                             self.params['unit_params']['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        decap_gen_params = dict(
            cls_name=DecapUnit.get_qualified_name(),
            params=self.params['unit_params'],
        )
        decap_template: GenericWrapper = self.new_template(GenericWrapper, params=decap_gen_params)
        self._core = decap_template

        decap_arr = []
        ny: int = self.params['ny']
        nx: int = self.params['nx']

        w_blk, h_blk = self.grid.get_block_size(decap_template.top_layer)
        core_bound_box = decap_template.core.bound_box
        w_bnd, h_bnd = -(core_bound_box.w - decap_template.bound_box.w) // 2, \
                       -(core_bound_box.h - decap_template.bound_box.h) // 2

        y_place = 0
        x_place = 0
        for idx in range(ny):
            x_place = 0
            decap_list = []
            for jdx in range(nx):
                decap_list.append(self.add_instance(decap_template, xform=Transform(x_place, y_place)))
                x_place += -(-decap_template.bound_box.w // w_blk) * w_blk

            y_place += -(-decap_template.bound_box.h // h_blk) * h_blk
            decap_arr.extend(decap_list)
        self.set_size_from_bound_box(decap_template.top_layer, BBox(0, 0, x_place, y_place))

        # Conenct lower supply wires
        vdd_hm = self.connect_wires([w for inst in decap_arr for w in inst.get_all_port_pins('VDD', layer=hm_layer)],
                                    lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires([w for inst in decap_arr for w in inst.get_all_port_pins('VSS', layer=hm_layer)],
                                    lower=self.bound_box.xl, upper=self.bound_box.xh)

        # # Connect to upper layer supplies
        shrink_power_bbox = BBox(self.bound_box.xl + w_bnd, self.bound_box.yl + h_bnd,
                                 self.bound_box.xh - w_bnd, self.bound_box.yh - h_bnd)
        core_power = self.params['core_power']
        power_bbox = shrink_power_bbox if core_power else self.bound_box
        top_layer = self.params['top_layer']
        if top_layer == 2:
            self.add_pin('VDD', vss_hm)
            self.add_pin('VSS', vdd_hm)
        else:
            power_dict = self.connect_supply_stack_warr(decap_template.core.tr_manager, [vdd_hm, vss_hm], hm_layer,
                                                        top_layer, power_bbox, side_sup=True)
            for idx in range(hm_layer + 1, top_layer + 1):
                self.add_pin(f'PLUS{idx}', power_dict[0][idx], label='PLUS', hide=idx < top_layer)
                self.add_pin(f'MINUS{idx}', power_dict[1][idx], label='MINUS', hide=idx < top_layer)

        self._sch_params = dict(
            nx=nx,
            ny=ny,
            unit=decap_template.sch_params,
        )
