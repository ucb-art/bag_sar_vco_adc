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


import math
from typing import Mapping, Any, Optional, Type, cast, Dict, List, Tuple

from bag3_analog.layout.res.termination import Termination

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.io import read_yaml
from bag.layout.enum import DrawTaps
from bag.layout.routing.base import WireArray, TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.typing import TrackType
from bag.util.immutable import Param, ImmutableSortedDict
from bag_vco_adc.layout.digital import InvCore
from bag_vco_adc.layout.others.decap import DecapArray
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import basefill_bbox
from bag_vco_adc.layout.util.wrapper import GenericWrapper, IntegrationWrapper
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orientation, Direction, MinLenMode, Orient2D
from xbase.layout.array.top import ArrayBaseWrapper
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.guardring import GuardRing
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo, TilePatternElement, TilePattern
from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase, ResTermType


def round_to_blk_pitch(x_loc, y_loc, blk_w, blk_h, round_up_x=True, round_up_y=True):
    x = math.ceil(x_loc / blk_w) * blk_w if round_up_x else math.floor(x_loc / blk_w) * blk_w
    y = math.ceil(y_loc / blk_h) * blk_h if round_up_x else math.floor(y_loc / blk_h) * blk_h
    return (x, y)


def bring_xm_to_xm1(base: TemplateBaseZL, xm_wire: WireArray, tr_manager: TrackManager, xm_layer: int,
                    wire_type: str = 'sig', avoid_short=False, roundmode=RoundMode.NEAREST, ret_ym_wire=None):
    ym_layer = xm_layer + 1
    xm1_layer = ym_layer + 1

    ym_tidx_l = base.grid.coord_to_track(ym_layer, xm_wire.bound_box.xl, RoundMode.GREATER_EQ)
    ym_tidx_m = base.grid.coord_to_track(ym_layer, (xm_wire.bound_box.xl + xm_wire.bound_box.xh) // 2,
                                         RoundMode.NEAREST)
    ym_tidx_r = base.grid.coord_to_track(ym_layer, xm_wire.bound_box.xh, RoundMode.LESS_EQ)

    tr_w_ym = tr_manager.get_width(ym_layer, wire_type)
    if tr_w_ym > 0:
        tr_sp_ym = tr_manager.get_sep(ym_layer, (wire_type, wire_type))
    else:
        tr_sp_ym = base.get_track_sep(ym_layer, tr_w_ym, tr_w_ym)
    if not avoid_short:
        ym_locs = base.get_tids_between(ym_layer, ym_tidx_l, ym_tidx_r, tr_w_ym, tr_sp_ym, tr_sp_ym, include_last=True)
        ym_locs = [tidx.base_index for tidx in ym_locs]
        if len(ym_locs) > 3:
            ym_locs = ym_locs[1:-1]
    else:
        ym_locs = base.get_available_tracks(ym_layer, ym_tidx_l, ym_tidx_m, xm_wire.bound_box.yl,
                                            xm_wire.bound_box.yh, width=tr_w_ym, sep=tr_sp_ym)[:-1] + \
                  base.get_available_tracks(ym_layer, ym_tidx_m, ym_tidx_r, xm_wire.bound_box.yl,
                                            xm_wire.bound_box.yh, width=tr_w_ym, sep=tr_sp_ym, align_to_higer=True)[:-1]
    ym_wires = [base.connect_to_tracks(xm_wire, TrackID(ym_layer, tidx, tr_w_ym, grid=base.grid)) for tidx in ym_locs]

    if xm_wire.track_id.num > 1:
        tidx = xm_wire.track_id.base_index + xm_wire.track_id.pitch * (xm_wire.track_id.num // 2)
        xm1_tidx = base.grid.coord_to_track(xm1_layer, base.grid.track_to_coord(xm_layer, tidx), roundmode)
    else:
        xm1_tidx = base.grid.coord_to_track(xm1_layer, base.grid.track_to_coord(xm_layer, xm_wire.track_id.base_index),
                                            roundmode)

    xm1_wire = base.connect_to_tracks(ym_wires,
                                      TrackID(xm1_layer, xm1_tidx, tr_manager.get_width(xm1_layer, wire_type),
                                              grid=base.grid))
    if ret_ym_wire is not None:
        ret_ym_wire.extend(ym_wires)
    return xm1_wire


class TerminationVert(Termination):
    def draw_layout(self) -> None:
        pinfo = cast(ResBasePlaceInfo, ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))
        self.draw_base(pinfo)

        assert pinfo.top_layer >= pinfo.conn_layer + 3

        # Get hm_layer and vm_layer WireArrays
        warrs, bulk_warrs = self.connect_hm_vm()

        # Connect all dummies
        self.connect_dummies(warrs, bulk_warrs)

        # Supply connections on xm_layer
        self.connect_bulk_xm(bulk_warrs)

        nx_dum: int = self.params['nx_dum']
        ny_dum: int = self.params['ny_dum']
        export_mid: bool = self.params['export_mid']

        # --- Routing of unit resistors --- #
        if pinfo.nx & 1:
            raise ValueError(f'This generator does not support odd number nx')
        bot_l, top_l = self.connect_units(warrs, nx_dum, pinfo.nx // 2, ny_dum, pinfo.ny - ny_dum)
        bot_r, top_r = self.connect_units(warrs, pinfo.nx // 2, pinfo.nx - nx_dum, ny_dum, pinfo.ny - ny_dum)

        pin_list = [('PLUS', top_l, False), ('MINUS', top_r, False),
                    ('MID', self.connect_wires([bot_l, bot_r])[0], not export_mid)]

        # connect to xm_layer
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        w_xm_sig = self.tr_manager.get_width(xm_layer, 'sig')
        for pin_name, warr, _hide in pin_list:
            xm_idx = self.grid.coord_to_track(xm_layer, warr.middle, RoundMode.NEAREST)
            xm_tid = TrackID(xm_layer, xm_idx, w_xm_sig)
            self.add_pin(pin_name, self.connect_to_tracks(warr, xm_tid), hide=_hide)

        self.sch_params = dict(
            w=pinfo.w_res,
            l=pinfo.l_res,
            res_type=pinfo.res_type,
            nx=pinfo.nx,
            ny=pinfo.ny,
            nx_dum=nx_dum,
            ny_dum=ny_dum,
            export_mid=export_mid,
            sup_name='VDD' if pinfo.res_config['sub_type_default'] == 'ntap' else 'VSS'
        )


class DiffInvBuf(MOSBase, TemplateBaseZL):
    """The core of the pseudo differential inverters
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_inv_diff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_p='segments of pmos',
            seg_n='segments of nmos',
            w_p='pmos width.',
            w_n='nmos width.',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            show_pins='True to show pins',
            flip_tile='True to flip all tiles',
            draw_taps='LEFT or RIGHT or BOTH or NONE',
            sig_locs='Signal locations for top horizontal metal layer pins',
            ndum='number of dummy at one side, need even number'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            seg_p=-1,
            seg_n=-1,
            w_p=0,
            w_n=0,
            ridx_p=-1,
            ridx_n=0,
            show_pins=False,
            flip_tile=False,
            draw_taps='NONE',
            sig_locs={},
            ndum=0,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo, flip_tile=self.params['flip_tile'])

        seg_p: int = self.params['seg_p']
        seg_n: int = self.params['seg_n']
        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']
        ndum: int = self.params['ndum']
        draw_taps: DrawTaps = DrawTaps[self.params['draw_taps']]
        sig_locs: Dict[str, TrackType] = self.params['sig_locs']

        for val in [seg_p, seg_n, ndum]:
            if val % 2:
                raise ValueError(f'This generator does not support odd number of segments ')

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        inv_params = self.params.copy(append=dict(is_guarded=True, show_pins=False, vertical_out=False))
        inv_master = self.new_template(InvCore, params=inv_params)
        inv_ncol = inv_master.num_cols

        # taps
        sub_sep = self.sub_sep_col
        sup_info = self.get_supply_column_info(vm_layer)
        num_taps = 0
        tap_offset = 0
        tap_left = tap_right = False
        if draw_taps in DrawTaps.RIGHT | DrawTaps.BOTH:
            num_taps += 1
            tap_right = True
        if draw_taps in DrawTaps.LEFT | DrawTaps.BOTH:
            num_taps += 1
            tap_offset += sup_info.ncol + sub_sep // 2
            tap_left = True

        # set total number of columns
        # Total width can be limited by either transistor size or by vertical metal size
        seg_max = 2 * max(seg_p, seg_n)
        seg_tot = seg_max + (sup_info.ncol + sub_sep // 2) * num_taps + 4 * ndum
        self.set_mos_size(seg_tot)

        # --- Placement --- #
        cur_col = tap_offset
        if ndum > 0:
            nmos_dum_p = self.add_mos(ridx_n, cur_col, ndum, tile_idx=0)
            pmos_dum_p = self.add_mos(ridx_p, cur_col, ndum, tile_idx=0)
            cur_col += ndum
        inst_p = self.add_tile(inv_master, 0, cur_col)

        ntap_mid = self.add_substrate_contact(ridx_n, cur_col+inv_ncol+self.sub_sep_col, tile_idx=0,
                                   seg=2*(ndum-self.sub_sep_col))
        ptap_mid = self.add_substrate_contact(ridx_p, cur_col+inv_ncol+self.sub_sep_col, tile_idx=0,
                                              seg=2*(ndum-self.sub_sep_col))
        cur_col += 2 * inv_ncol + 2 * ndum
        mid_idx = self.arr_info.col_to_track(vm_layer, cur_col - inv_ncol - ndum, mode=RoundMode.NEAREST)
        inst_n = self.add_tile(inv_master, 0, cur_col, flip_lr=True)
        if ndum > 0:
            cur_col += ndum
            nmos_dum_n = self.add_mos(ridx_n, cur_col, ndum, tile_idx=0, flip_lr=True)
            pmos_dum_n = self.add_mos(ridx_p, cur_col, ndum, tile_idx=0, flip_lr=True)
        # add taps
        lay_range = range(self.conn_layer, xm_layer)
        vdd_table: Dict[int, List[WireArray]] = {lay: [] for lay in lay_range}
        vss_table: Dict[int, List[WireArray]] = {lay: [] for lay in lay_range}
        if tap_left:
            self.add_supply_column(sup_info, 0, vdd_table, vss_table, ridx_p, ridx_n)
        if tap_right:
            self.add_supply_column(sup_info, seg_tot, vdd_table, vss_table, ridx_p, ridx_n,
                                   flip_lr=True)

        self.connect_to_track_wires(ntap_mid, vss_table[hm_layer])
        self.connect_to_track_wires(ptap_mid, vdd_table[hm_layer])

        # --- Routing --- #
        # 1. supplies
        vdd_table[hm_layer].append(inst_p.get_pin('VDD', layer=hm_layer))
        vdd_table[hm_layer].append(inst_n.get_pin('VDD', layer=hm_layer))
        self.add_pin('VDD_conn', vdd_table[self.conn_layer], hide=True)
        self.add_pin('VDD_hm', vdd_table[hm_layer], hide=True)
        self.add_pin('VDD_vm', vdd_table[vm_layer], hide=True)

        vss_table[hm_layer].append(inst_p.get_pin('VSS', layer=hm_layer))
        vss_table[hm_layer].append(inst_n.get_pin('VSS', layer=hm_layer))
        self.add_pin('VSS_conn', vss_table[self.conn_layer], hide=True)
        self.add_pin('VSS_hm', vss_table[hm_layer], hide=True)
        self.add_pin('VSS_vm', vss_table[vm_layer], hide=True)
        # self.add_pin('VSS', self.connect_wires(vss_table[xm_layer]))

        vdd = self.connect_wires(vdd_table[hm_layer])[0]
        vss = self.connect_wires(vss_table[hm_layer])[0]
        if ndum > 0:
            # connect dummy
            pmos_port, nmos_port = [], []
            for pmos, nmos in zip([pmos_dum_n, pmos_dum_p], [nmos_dum_n, nmos_dum_p]):
                pmos_port += [pmos.s]
                nmos_port += [nmos.s]
            self.connect_to_track_wires(pmos_port, vdd)
            self.connect_to_track_wires(nmos_port, vss)
            # connect g&d to VDD/VSS with M3
            tr_w_h = self.tr_manager.get_width(hm_layer, 'sig')
            tr_w_v = self.tr_manager.get_width(vm_layer, 'sig')
            nout_tidx = sig_locs.get('nout', self.get_track_index(ridx_n, MOSWireType.DS_GATE,
                                                                  wire_name='sig', wire_idx=0))
            pout_tidx = sig_locs.get('pout', self.get_track_index(ridx_p, MOSWireType.DS_GATE,
                                                                  wire_name='sig', wire_idx=-1))
            nout_tid = TrackID(hm_layer, nout_tidx, tr_w_h)
            pout_tid = TrackID(hm_layer, pout_tidx, tr_w_h)
            # left (p)
            pmos_p_hm = self.connect_to_tracks([pmos_dum_p.g, pmos_dum_p.d], pout_tid, min_len_mode=MinLenMode.LOWER)
            nmos_p_hm = self.connect_to_tracks([nmos_dum_p.g, nmos_dum_p.d], nout_tid, min_len_mode=MinLenMode.LOWER)
            tidx_p_vm = self.grid.coord_to_track(vm_layer, pmos_p_hm.lower, mode=RoundMode.LESS)
            self.connect_to_tracks([pmos_p_hm, vdd], TrackID(vm_layer, tidx_p_vm, width=tr_w_v))
            self.connect_to_tracks([nmos_p_hm, vss], TrackID(vm_layer, tidx_p_vm, width=tr_w_v))
            pmos_n_hm = self.connect_to_tracks([pmos_dum_n.g, pmos_dum_n.d], pout_tid, min_len_mode=MinLenMode.UPPER)
            nmos_n_hm = self.connect_to_tracks([nmos_dum_n.g, nmos_dum_n.d], nout_tid, min_len_mode=MinLenMode.UPPER)
            tidx_n_vm = self.grid.coord_to_track(vm_layer, pmos_n_hm.upper, mode=RoundMode.GREATER)
            self.connect_to_tracks([pmos_n_hm, vdd], TrackID(vm_layer, tidx_n_vm, width=tr_w_v))
            self.connect_to_tracks([nmos_n_hm, vss], TrackID(vm_layer, tidx_n_vm, width=tr_w_v))

        # 2. export inp, inn
        inp_vm = inst_p.get_pin('in')
        inn_vm = inst_n.get_pin('in')
        inp_vm_tidx_extra = tr_manager.get_next_track(vm_layer, inp_vm.track_id.base_index, 'sig', 'sig')
        inn_vm_tidx_extra = tr_manager.get_next_track(vm_layer, inn_vm.track_id.base_index, 'sig', 'sig', up=-1)
        inp_vm = [inp_vm] + [self.connect_to_tracks([inst_p.get_pin('nin'), inst_p.get_pin('pin')],
                                                    TrackID(vm_layer, inp_vm_tidx_extra, tr_w_v))]
        inn_vm = [inn_vm] + [self.connect_to_tracks([inst_n.get_pin('nin'), inst_n.get_pin('pin')],
                                                    TrackID(vm_layer, inn_vm_tidx_extra, tr_w_v))]
        self.add_pin('vip', inp_vm)
        self.add_pin('vin', inn_vm)

        # 3. export von, vop and connect to multiple wires on vm_layer
        von_upper_idx = tr_manager.get_next_track(vm_layer, mid_idx, 'sig', 'sig', up=-2)
        vop_lower_idx = tr_manager.get_next_track(vm_layer, mid_idx, 'sig', 'sig', up=2)
        von_hm = inst_p.get_all_port_pins('out')
        vop_hm = inst_n.get_all_port_pins('out')
        von_vm = self.connect_to_tracks(von_hm, TrackID(vm_layer, von_upper_idx, tr_w_v))
        vop_vm = self.connect_to_tracks(vop_hm, TrackID(vm_layer, vop_lower_idx, tr_w_v))

        self.add_pin('von', von_vm)
        self.add_pin('vop', vop_vm)
        sch_params = inv_master.sch_params
        w_n = sch_params['w_n']
        w_p = sch_params['w_p']
        lch = sch_params['lch']
        if ndum > 0:
            dum_info = [(('nch', w_n, lch, sch_params['th_n'], 'VSS', 'VSS'), ndum * 2),
                        (('pch', w_p, lch, sch_params['th_p'], 'VDDA', 'VDDA'), ndum * 2)]
            sch_params = sch_params.copy(append=dict(dum_info=dum_info))

        vdd_xm, vss_xm = [], []
        vdd_hm = self.connect_wires(vdd_table[hm_layer])
        vss_hm = self.connect_wires(vss_table[hm_layer])
        vdd_hm = [self.add_wires(hm_layer, vdd_hm[0].track_id.base_index, vdd_hm[0].lower, vdd_hm[0].middle,
                                 width=vdd_hm[0].track_id.width, num=vdd_hm[0].track_id.num),
                  self.add_wires(hm_layer, vdd_hm[0].track_id.base_index, vdd_hm[0].middle, vdd_hm[0].upper,
                                 width=vdd_hm[0].track_id.width, num=vdd_hm[0].track_id.num)]
        vss_hm = [self.add_wires(hm_layer, vss_hm[0].track_id.base_index, vss_hm[0].lower, vss_hm[0].middle,
                                 width=vss_hm[0].track_id.width, num=vss_hm[0].track_id.num),
                  self.add_wires(hm_layer, vss_hm[0].track_id.base_index, vss_hm[0].middle, vss_hm[0].upper,
                                 width=vss_hm[0].track_id.width, num=vss_hm[0].track_id.num)]
        for idx, warr in enumerate(vdd_hm):
            vdd_xm.extend(self.export_tap_hm(tr_manager, warr, hm_layer, xm_layer, align_upper=idx & 1))
        # self.add_pin('VDDA', self.connect_wires(vdd_table[xm_layer]))
        for idx, warr in enumerate(vss_hm):
            vss_xm.extend(self.export_tap_hm(tr_manager, warr, hm_layer, xm_layer, align_upper=idx & 1))

        vdd_xm = self.connect_wires(vdd_xm)
        vss_xm = self.connect_wires(vss_xm)

        self.add_pin('VDDA', vdd_xm)
        self.add_pin('VSS', vss_xm)
        # set properties
        self.sch_params = sch_params


class DiffInvBufGR(GuardRing):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        GuardRing.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = DiffInvBuf.get_params_info()
        ans.update(
            pmos_gr='pmos guard ring tile name.',
            nmos_gr='nmos guard ring tile name.',
            edge_ncol='Number of columns on guard ring edge.  Use 0 for default.',
        )
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = DiffInvBuf.get_default_param_values()
        ans.update(
            pmos_gr='pgr',
            nmos_gr='ngr',
            edge_ncol=0,
        )
        return ans

    def get_layout_basename(self) -> str:
        return self.__class__.__name__

    def draw_layout(self) -> None:
        params = self.params
        pmos_gr: str = params['pmos_gr']
        nmos_gr: str = params['nmos_gr']
        edge_ncol: int = params['edge_ncol']

        core_params = params.copy(remove=['pmos_gr', 'nmos_gr', 'edge_ncol'])
        master = self.new_template(DiffInvBuf, params=core_params)

        sub_sep = master.sub_sep_col
        gr_sub_sep = master.gr_sub_sep_col
        sep_ncol_left = sep_ncol_right = sub_sep
        draw_taps: DrawTaps = DrawTaps[params['draw_taps']]
        if draw_taps in DrawTaps.RIGHT | DrawTaps.BOTH:
            sep_ncol_right = gr_sub_sep - sub_sep // 2
        if draw_taps in DrawTaps.LEFT | DrawTaps.BOTH:
            sep_ncol_left = gr_sub_sep - sub_sep // 2
        sep_ncol = (sep_ncol_left, sep_ncol_right)

        inst, sup_list = self.draw_guard_ring(master, pmos_gr, nmos_gr, sep_ncol, edge_ncol)
        vdd_hm_list, vss_hm_list = [], []
        for (vss_list, vdd_list) in sup_list:
            vss_hm_list.extend(vss_list)
            vdd_hm_list.extend(vdd_list)

        self.connect_to_track_wires(vss_hm_list, inst.get_all_port_pins('VSS_vm'))
        self.connect_to_track_wires(vdd_hm_list, inst.get_all_port_pins('VDD_vm'))


class ClkRxRes(ResArrayBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'res')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(nx_dum=0, ny_dum=0)

    def draw_layout(self) -> None:
        pinfo = cast(ResBasePlaceInfo, ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))
        self.draw_base(pinfo)

        sub_type = pinfo.res_config['sub_type_default']
        if sub_type != 'ntap':
            raise ValueError(f'This generator does not support sub_type={sub_type}. Only ntap is supported.')

        # Get hm_layer and vm_layer WireArrays
        warrs, bulk_warrs = self.connect_hm_vm()

        # Connect all dummies
        self.connect_dummies(warrs, bulk_warrs)

        # Supply connections on xm_layer
        self.connect_bulk_xm(bulk_warrs)

        unit_params = dict(
            w=pinfo.w_res,
            l=pinfo.l_res,
            intent=pinfo.res_type,
        )
        nx, ny = pinfo.nx, pinfo.ny
        nx_dum: int = self.params['nx_dum']
        ny_dum: int = self.params['ny_dum']
        npar = nx - 2 * nx_dum
        nser = ny - 2 * ny_dum
        num_dum = nx * ny - npar * nser

        # --- Routing of unit resistors --- #
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        # r_bot, r_top = self.connect_units(warrs, nx_dum, nx - nx_dum, ny_dum, ny - ny_dum)
        l_vm_term_list, r_vm_term_list = [], []
        for idx in range(ny_dum, ny - ny_dum):
            _term_name = ResTermType.BOT if idx & 1 else ResTermType.TOP
            l_vm_term_list.append(warrs[vm_layer][_term_name][nx_dum][idx])
            r_vm_term_list.append(warrs[vm_layer][_term_name][nx - nx_dum - 1][idx])
            for jdx in range(nx_dum, nx - nx_dum - 1):
                if idx & 1:
                    _term_name = ResTermType.TOP if jdx & 1 else ResTermType.BOT
                else:
                    _term_name = ResTermType.BOT if jdx & 1 else ResTermType.TOP
                self.connect_wires(self.connect_wires([warrs[hm_layer][_term_name][jdx][idx],
                                                       warrs[hm_layer][_term_name][jdx + 1][idx]]))

        # connect top vm_layers to xm_layer
        w_xm_sig = self.tr_manager.get_width(xm_layer, 'sig')
        l_xm_term_list, r_xm_term_list = [], []
        for vm_l, vm_r in zip(l_vm_term_list, r_vm_term_list):
            xm_idx = self.grid.coord_to_track(xm_layer, vm_l.middle, RoundMode.NEAREST)
            xm_tid = TrackID(xm_layer, xm_idx, w_xm_sig)
            l_xm_term_list.append(self.connect_to_tracks(vm_l, xm_tid))
            r_xm_term_list.append(self.connect_to_tracks(vm_r, xm_tid))

        # xm_tid1 = TrackID(xm_layer, xm_idx1, w_xm_sig)
        ym_term = False
        if ym_term:
            ym_layer = xm_layer + 1
            w_ym_sig = self.tr_manager.get_width(ym_layer, 'sig')
            l_ym_term_tidx = self.grid.coord_to_track(ym_layer, l_xm_term_list[0].middle, RoundMode.NEAREST)
            r_ym_term_tidx = self.grid.coord_to_track(ym_layer, r_xm_term_list[0].middle, RoundMode.NEAREST)

            plus_ym = self.connect_to_tracks(l_xm_term_list, TrackID(ym_layer, l_ym_term_tidx, w_ym_sig))
            minus_ym = self.connect_to_tracks(r_xm_term_list, TrackID(ym_layer, r_ym_term_tidx, w_ym_sig))
            self.add_pin('PLUS', plus_ym)
            self.add_pin('MINUS', minus_ym)

        else:
            self.add_pin('PLUS', l_xm_term_list, connect=True)
            self.add_pin('MINUS', r_xm_term_list, connect=True)

        nser, npar = npar, nser
        self.sch_params = dict(
            res_params=dict(
                unit_params=unit_params,
                nser=nser,
                npar=npar,
            ),
            dum_params=dict(
                unit_params=unit_params,
                nser=1,
                npar=num_dum,
            ),
            sub_type=sub_type,
        )


class ClkRx(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._sup_wire = {}
        self._cap_top = 0

    @property
    def cap_top(self):
        return self._cap_top

    @property
    def sup_wire(self) -> Dict[WireArray, Any]:
        return self._sup_wire

    # @classmethod
    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc',
                                            'clk_rx_tia' if self.params['res_term_params'] else 'clk_ac_tia')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            buf_params='Parameters for buffer',
            res_params='Parameters for feedback resistor',
            res_term_params='Parameters for termination resistor',
            decap_params='Parameters for DC decoupling cap',
            do_power_fill='True to do power fill from M4 to M6',
            top_layer='Top layer of the process',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            do_power_fill=False,
            res_term_params=None,
            decap_params=None,
        )

    def draw_layout(self) -> None:
        # Routing setting
        buf_params: Mapping[str, Any] = self.params['buf_params']
        res_params: Mapping[str, Any] = self.params['res_params']
        res_term_params: Optional[Mapping[str, Any]] = self.params.get('res_term_params', None)

        if res_term_params:
            if isinstance(res_term_params, str):
                res_term_params = read_yaml(res_term_params)
                res_term_params = res_term_params['params']
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      buf_params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # make masters
        buf_master = self.new_template(GenericWrapper, params=dict(cls_name=DiffInvBufGR.get_qualified_name(),
                                                                   params=buf_params,
                                                                   export_private=False))

        res_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=ClkRxRes.get_qualified_name(),
                                                                     params=res_params))

        # termination
        if res_term_params:
            res_term_master = self.new_template(IntegrationWrapper, params=res_term_params)
            res_term_w, res_term_h = res_term_master.bound_box.w, res_term_master.bound_box.h
        else:
            res_term_master = None
            res_term_w, res_term_h = 0, 0

        # --- Placement --- #
        top_layer = self.params['top_layer']
        top_layer_dir = self.grid.get_direction(top_layer)
        top_vert_layer = top_layer if top_layer_dir == Orient2D.y else top_layer - 1
        w_blk, h_blk = self.grid.get_block_size(top_vert_layer)
        # Use a different grid for placement
        w_blk_place, h_blk_place = self.grid.get_block_size(xm1_layer)
        buf_w, buf_h = buf_master.bound_box.w, buf_master.bound_box.h
        res_w, res_h = res_master.bound_box.w, res_master.bound_box.h

        tot_w = max(buf_w, res_term_w, 2 * res_w)
        tot_h = buf_h + res_h + res_term_h
        tot_w, tot_h = round_to_blk_pitch(tot_w, tot_h, w_blk, h_blk)

        if res_term_params:
            res_term_xl = (tot_w - res_term_w) // 2
            res_term_yl = 0
            res_term_xl, res_term_yl = round_to_blk_pitch(res_term_xl, res_term_yl, w_blk_place, h_blk_place)
            res_term_inst = self.add_instance(res_term_master, xform=Transform(dx=res_term_xl, dy=res_term_yl))
        else:
            res_term_inst = None
        res_margin = res_master.bound_box.w - res_master.core.bound_box.w
        res_yl = res_term_h + h_blk_place
        res_xl, res_yl = round_to_blk_pitch(tot_w // 2 + res_margin // 2, res_yl, w_blk_place, h_blk_place,
                                            round_up_x=False)
        resp_inst = self.add_instance(res_master, xform=Transform(dx=res_xl, dy=res_yl, mode=Orientation.MY))
        res_xr, res_yr = round_to_blk_pitch(tot_w // 2 - res_margin // 2, res_yl, w_blk_place, h_blk_place,
                                            round_up_x=False)
        resn_inst = self.add_instance(res_master, xform=Transform(dx=res_xr, dy=res_yr))

        buf_xl = (tot_w - buf_w) // 2
        buf_yl = res_term_h + res_h
        buf_xl, buf_yl = round_to_blk_pitch(buf_xl, buf_yl, w_blk_place, h_blk_place)
        buf_inst = self.add_instance(buf_master, xform=Transform(dx=buf_xl, dy=buf_yl))
        tr_manager = buf_master.core.tr_manager

        # --- Routing --- #
        w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        w_sig_ym = tr_manager.get_width(ym_layer, 'sig')

        # 3 ym1 at the middle for inp, gnd, inn
        _, ym1_locs = tr_manager.place_wires(ym1_layer, ['sig', 'sup', 'sig'], center_coord=tot_w // 2)

        vlay = (buf_inst.get_port('vip').get_single_layer(), 'drawing')
        vdir = Direction.LOWER

        # set size
        self.set_size_from_bound_box(xm1_layer, BBox(0, 0, tot_w, tot_h))

        # vip_m/ vin_ma
        vip_m_buf = buf_inst.get_all_port_pins('vip')
        vin_m_buf = buf_inst.get_all_port_pins('vin')
        vip_stack_dict = self.via_stack_up(tr_manager, vip_m_buf, vm_layer,
                                           xm1_layer if res_term_params else ym1_layer, 'sig', RoundMode.NEAREST)
        vin_stack_dict = self.via_stack_up(tr_manager, vin_m_buf, vm_layer,
                                           xm1_layer if res_term_params else ym1_layer, 'sig', RoundMode.NEAREST)
        # vop/ von
        vop_m_buf = buf_inst.get_all_port_pins('vop')
        von_m_buf = buf_inst.get_all_port_pins('von')

        vop_stack_dict = self.via_stack_up(tr_manager, vop_m_buf, vm_layer, ym_layer, 'sig', RoundMode.NEAREST)
        von_stack_dict = self.via_stack_up(tr_manager, von_m_buf, vm_layer, ym_layer, 'sig', RoundMode.NEAREST)

        vip_m_res = resp_inst.get_all_port_pins('MINUS')
        vin_m_res = resn_inst.get_all_port_pins('MINUS')
        vop_m_res = resn_inst.get_all_port_pins('PLUS')
        von_m_res = resp_inst.get_all_port_pins('PLUS')

        self.connect_to_track_wires(vop_m_res, vop_stack_dict[ym_layer])
        self.connect_to_track_wires(von_m_res, von_stack_dict[ym_layer])
        self.connect_to_track_wires(vip_m_res, vip_stack_dict[ym_layer])
        self.connect_to_track_wires(vin_m_res, vin_stack_dict[ym_layer])

        self.add_pin('tia_out_p' if res_term_params else 'vop',
                     self.extend_wires(vop_stack_dict[ym_layer], upper=buf_inst.bound_box.yh,
                                       lower=buf_inst.bound_box.yl))
        self.add_pin('tia_out_n' if res_term_params else 'von',
                     self.extend_wires(von_stack_dict[ym_layer], upper=buf_inst.bound_box.yh,
                                       lower=buf_inst.bound_box.yl))

        # Bring supplies to ym1
        buf_vdd_xm1 = bring_xm_to_xm1(self, buf_inst.get_pin('VDDA'), tr_manager, xm_layer, 'sup', True)
        buf_vss_xm1 = bring_xm_to_xm1(self, buf_inst.get_pin('VSS'), tr_manager, xm_layer, 'sup', True)
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')

        res_vdd_xm1 = []
        res_vdd_xm = self.connect_wires(resn_inst.get_all_port_pins('VDD') + resp_inst.get_all_port_pins('VDD'))[0]
        for xm in res_vdd_xm.to_warr_list():
            res_vdd_xm1.append(bring_xm_to_xm1(self, xm, tr_manager, xm_layer, 'sup', True))

        vdd_list, vss_list = [buf_vdd_xm1] + res_vdd_xm1, [buf_vss_xm1]
        self.set_size_from_bound_box(ym1_layer, BBox(0, 0, tot_w, tot_h))
        if res_term_master:
            self.connect_to_track_wires(vin_stack_dict[xm1_layer],
                                        res_term_inst.get_all_port_pins('inn_m', layer=ym1_layer))
            self.connect_to_track_wires(vip_stack_dict[xm1_layer],
                                        res_term_inst.get_all_port_pins('inp_m', layer=ym1_layer))
            self.reexport(res_term_inst.get_port('inn_m'))
            self.reexport(res_term_inst.get_port('inp_m'))
            vdd_list = self.connect_to_track_wires(vdd_list, res_term_inst.get_all_port_pins('VDD', layer=ym1_layer))
            vss_list = self.connect_to_track_wires(vss_list, res_term_inst.get_all_port_pins('VSS', layer=ym1_layer))

            vdd_list = self.extend_wires(vdd_list, upper=self.bound_box.yh)
            vss_list = self.extend_wires(vss_list, upper=self.bound_box.yh)
            self.add_pin(f'VDD{ym1_layer}', vdd_list, label='VDD')
            self.add_pin(f'VSS{ym1_layer}', vss_list, label='VSS')
            vdd_list, vss_list = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], ym1_layer, self.bound_box)
            vdd_list.extend(res_term_inst.get_all_port_pins('VDD', layer=ym1_layer + 1))
            vss_list.extend(res_term_inst.get_all_port_pins('VSS', layer=ym1_layer + 1))

            vdd_list = self.extend_wires(vdd_list, upper=self.bound_box.xh, lower=self.bound_box.xl)
            vss_list = self.extend_wires(vss_list, upper=self.bound_box.xh, lower=self.bound_box.xl)
            self.add_pin('VDD', vdd_list)
            self.add_pin('VSS', vss_list)
            self.reexport(res_term_inst.get_port('inn'), net_name='inn')
            self.reexport(res_term_inst.get_port('inp'), net_name='inp')
            self.reexport(res_term_inst.get_port('mid'))
        else:
            # self.extend_wires(vin_stack_dict[ym1_layer], lower=0)
            self.add_pin('vin', vin_stack_dict[ym1_layer])
            self.add_pin('vip', vip_stack_dict[ym1_layer])
            for idx in range(xm1_layer, ym1_layer):
                sup_w = tr_manager.get_width(idx, 'sup')
                sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
                sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
                sup_bbox_all_l = BBox(self.bound_box.xl, self.bound_box.yl,
                                      (self.bound_box.xl + self.bound_box.xh) // 2, self.bound_box.yh)
                sup_bbox_all_r = BBox((self.bound_box.xl + self.bound_box.xh) // 2, self.bound_box.yl,
                                      self.bound_box.xh, self.bound_box.yh)
                idx_dir = self.grid.get_direction(idx)
                if idx_dir == Orient2D.y:
                    vdd_list, vss_list = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx,
                                                                  self.bound_box)
                else:
                    sup_l = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_l,
                                                     side_sup=False)
                    sup_r = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_r,
                                                     side_sup=False, align_upper=True)
                    vdd_list = sup_l[0] + sup_r[0]
                    vss_list = sup_l[1] + sup_r[1]
                self.add_pin(f'VDD{idx + 1}', vdd_list, label='VDD')
                self.add_pin(f'VSS{idx + 1}', vss_list, label='VSS')

        # vout_xm1_tidx = self.grid.coord_to_track(xm1_layer, buf_inst.bound_box.yh, RoundMode.NEAREST, )
        # von_bbox = BBox(von_stack_dict[ym_layer].bound_box.xl, buf_vdd_xm1.bound_box.yh + buf_vdd_xm1.bound_box.h // 2,
        #                 von_stack_dict[ym_layer].bound_box.xh, self.bound_box.yh)
        # von_xm1 = self.via_up(tr_manager, von_stack_dict[ym_layer], ym_layer, 'sig', bbox=von_bbox)
        # vop_bbox = BBox(vop_stack_dict[ym_layer].bound_box.xl, buf_vdd_xm1.bound_box.yh + buf_vdd_xm1.bound_box.h // 2,
        #                 vop_stack_dict[ym_layer].bound_box.xh, self.bound_box.yh)
        # vop_xm1 = self.via_up(tr_manager, vop_stack_dict[ym_layer], ym_layer, 'sig', bbox=vop_bbox)
        #
        # von_ym1_list, vop_ym1_list = [], []
        # von_ym1 = self.connect_to_tracks(von_xm1, vip_stack_dict[ym1_layer].track_id)
        # vop_ym1 = self.connect_to_tracks(vop_xm1, vin_stack_dict[ym1_layer].track_id)

        # set size
        # set schematic parameters
        if res_term_params:
            self.sch_params = dict(
                tia_params=dict(
                    buf_cmos_cell_params=buf_master.sch_params,
                    res_params=res_master.sch_params,
                    with_cap=False),
                term_params=res_term_master.sch_params,
            )
        else:
            self.sch_params = dict(
                buf_cmos_cell_params=buf_master.sch_params,
                res_params=res_master.sch_params,
                with_cap=False
            )


class DiffInvCoupled(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_diff_inv_coupled')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            vertical_out='True to have vertical output',
            coupling='Remove coupling',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            vertical_out=True,
            coupling=True,
        )

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']

        w_ans = {}
        th_ans = {}
        for name, row_idx in [('nfb', ridx_n), ('nin', ridx_n), ('pfb', ridx_p), ('pin', ridx_p)]:
            rinfo = self.get_row_info(row_idx, 0)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        coupling: bool = self.params['coupling']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_inv_n = seg_dict['nin']
        seg_inv_p = seg_dict['pin']

        seg_fb_n = seg_dict['nfb']
        seg_fb_p = seg_dict['pfb']

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=0)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2, tile_idx=0)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=0)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2, tile_idx=0)

        vertical_out = self.params['vertical_out']
        inv_l_params = dict(pinfo=pinfo, seg_n=seg_inv_n, seg_p=seg_inv_p, w_p=w_p, w_n=w_n,
                            ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={'nin': ng2_tidx}, vertical_sup=True,
                            vertical_out=vertical_out)
        inv_r_params = dict(pinfo=pinfo, seg_n=seg_inv_n, seg_p=seg_inv_p, w_p=w_p, w_n=w_n,
                            ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={'nin': ng2_tidx}, vertical_sup=True,
                            vertical_out=vertical_out)
        fb_l_params = dict(pinfo=pinfo, seg_n=seg_fb_n, seg_p=seg_fb_p, w_p=w_p, w_n=w_n,
                           ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={'nin': ng1_tidx}, vertical_sup=True)
        fb_r_params = dict(pinfo=pinfo, seg_n=seg_fb_n, seg_p=seg_fb_p, w_p=w_p, w_n=w_n,
                           ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={'nin': pg1_tidx}, vertical_sup=True)

        [inv_l_master, inv_r_master, fb_l_master, fb_r_master] = \
            [self.new_template(InvCore, params=_params) for _params
             in [inv_l_params, inv_r_params, fb_l_params, fb_r_params]]
        cur_col = min_sep
        inv_l = self.add_tile(inv_l_master, col_idx=cur_col, tile_idx=0)
        cur_col += inv_l_master.num_cols
        fb_l = self.add_tile(fb_l_master, col_idx=cur_col, tile_idx=0) if coupling else None
        cur_col += fb_l_master.num_cols
        cur_col += 2 * min_sep if seg_fb_n < 4 else 0
        fb_r = self.add_tile(fb_r_master, col_idx=cur_col, tile_idx=0) if coupling else None
        cur_col += fb_r_master.num_cols
        inv_r = self.add_tile(inv_r_master, col_idx=cur_col, tile_idx=0)
        self.set_mos_size(self.num_cols + min_sep)

        if coupling:
            self.connect_wires([inv_l.get_pin('nout'), fb_l.get_pin('nout')])
            self.connect_wires([inv_l.get_pin('pout'), fb_l.get_pin('pout')])
            self.connect_wires([inv_r.get_pin('nout'), fb_r.get_pin('nout')])
            self.connect_wires([inv_r.get_pin('pout'), fb_r.get_pin('pout')])

            self.connect_differential_wires(fb_r.get_pin('nin'), fb_l.get_pin('nin'),
                                            fb_l.get_pin('out'), fb_r.get_pin('out'))
            self.add_pin('outp_fb', fb_r.get_pin('out'), hide=True)
            self.add_pin('outn_fb', fb_l.get_pin('out'), hide=True)

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [inv_l, fb_l, fb_r, inv_r] if coupling else [inv_l, inv_r]
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))

        #
        self.add_pin('VDD', vdd_list)
        self.add_pin('VSS', vss_list)
        self.add_pin('inp', inv_l.get_pin('nin'))
        self.add_pin('inn', inv_r.get_pin('nin'))
        self.add_pin('outp', inv_r.get_all_port_pins('out'))
        self.add_pin('outn', inv_l.get_all_port_pins('out'))

        sch_params_dict = dict(
            inv=inv_r_master.sch_params,
            fb=fb_r_master.sch_params if coupling else None,
        )
        self.sch_params = sch_params_dict


class DiffInvCoupledChain(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_diff_inv_coupled_chain')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            nstage='',
            scale_list='',
            row_list='',
            vertical_out_idx='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            row_list=None,
            vertical_out_idx=None,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        ntap_pinfo = pinfo[1]['ntap_tile']
        logic_pinfo = pinfo[1]['logic_tile']
        ptap_pinfo = pinfo[1]['ptap_tile']

        nstage = self.params['nstage']
        pinfo_list = [TilePatternElement(ptap_pinfo)]
        scale_list = self.params['scale_list']
        row_list = self.params['row_list']
        if row_list:
            scale_list = [s // row_list[idx] for idx, s in enumerate(scale_list)]
        else:
            row_list = [1] * len(scale_list)

        nrows = sum(row_list)

        for idx in range(nrows):
            tap_info = ptap_pinfo if bool(idx & 1) else ntap_pinfo
            # pinfo_list.append(TilePatternElement(logic_pinfo))
            pinfo_list.append(TilePatternElement(logic_pinfo, flip=bool(idx & 1)))
            pinfo_list.append(TilePatternElement(tap_info))

        self.draw_base((TilePattern(pinfo_list), pinfo[1]))
        # setup floorplan

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_inv_n = seg_dict['nin']
        seg_inv_p = seg_dict['pin']

        seg_fb_n = seg_dict['nfb']
        seg_fb_p = seg_dict['pfb']

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        params_list = []

        vertical_out_idx = self.params['vertical_out_idx']
        if vertical_out_idx is None:
            vertical_out_idx = nstage - 1
        for idx in range(nstage):
            _seg_dict = dict()
            for key, val in seg_dict.items():
                _seg_dict[key] = int(scale_list[idx] * val)
            params_list.append(dict(pinfo=(self.get_tile_subpattern(start_idx=1, stop_idx=2), self.tile_table),
                                    seg_dict=_seg_dict, vertical_out=idx <= vertical_out_idx,
                                    coupling=idx < nstage - 1))
        master_list = [self.new_template(DiffInvCoupled, params=_p) for _p in params_list]
        ncol_tot = max([m.num_cols for m in master_list])
        inst_list = []
        for idx in range(nstage):
            for jdx in range(row_list[idx]):
                tile_index = sum(row_list[:idx]) + jdx
                inst_list.append(self.add_tile(master_list[idx], tile_idx=2 * tile_index + 1,
                                               col_idx=(ncol_tot - master_list[idx].num_cols) // 2))

        vdd_sub_list, vss_sub_list = [], []
        vss_sub_list.append(self.add_substrate_contact(0, col_idx=0, tile_idx=0, seg=self.num_cols).to_warr_list())
        for idx in range(1, nrows + 1):
            if idx & 1:
                vdd_sub_list.append(
                    self.add_substrate_contact(0, col_idx=0, tile_idx=2 * idx, seg=self.num_cols).to_warr_list())
            else:
                vss_sub_list.append(
                    self.add_substrate_contact(0, col_idx=0, tile_idx=2 * idx, seg=self.num_cols).to_warr_list())
        self.set_mos_size(num_cols=max([master.num_cols for master in master_list]))

        _stage_in_list, _stage_out_list = [], []
        stage_in_list, stage_out_list = [], []
        for idx in range(nstage):
            start, stop = sum(row_list[:idx]), sum(row_list[:idx + 1])
            _stage_in_list.append(([inst.get_all_port_pins('inp') for inst in inst_list[start:stop]],
                                   [inst.get_all_port_pins('inn') for inst in inst_list[start:stop]]))
            _stage_out_list.append(([inst.get_all_port_pins('outp') for inst in inst_list[start:stop]],
                                    [inst.get_all_port_pins('outn') for inst in inst_list[start:stop]]))
            if idx < nstage - 1:
                self.connect_wires([inst.get_pin('outp_fb') for inst in inst_list[start:stop]] +
                                   [inst.get_pin('outp_fb') for inst in inst_list[start:stop]])
                self.connect_wires([inst.get_pin('outn_fb') for inst in inst_list[start:stop]] +
                                   [inst.get_pin('outn_fb') for inst in inst_list[start:stop]])

        for inn, inp in _stage_in_list:
            inn = [w for warr in inn for w in warr]
            inp = [w for warr in inp for w in warr]
            stage_in_list.append((inn, inp))
        for outn, outp in _stage_out_list:
            outn = [w for warr in outn for w in warr]
            outp = [w for warr in outp for w in warr]
            stage_out_list.append((outn, outp))

        if nstage > 2:
            for idx in range(vertical_out_idx + 1):
                if idx + 1 < nstage:
                    self.connect_to_track_wires(stage_in_list[idx + 1][1], stage_out_list[idx][0])
                    self.connect_to_track_wires(stage_in_list[idx + 1][0], stage_out_list[idx][1])
        #
        vss_hm_list, vdd_hm_list = [], []
        for idx in range(nrows):
            if idx & 1:
                vss_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2 * (idx + 1))
                vdd_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2 * idx)
                vss_hm_list.append(self.connect_to_tracks(inst_list[idx].get_all_port_pins('VSS') +
                                                          vss_sub_list[(idx + 1) // 2], vss_tid))
                vdd_hm_list.append(self.connect_to_tracks(inst_list[idx].get_all_port_pins('VDD') +
                                                          vdd_sub_list[idx // 2], vdd_tid))
            else:
                vss_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2 * idx)
                vdd_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2 * (idx + 1))
                vss_hm_list.append(
                    self.connect_to_tracks(inst_list[idx].get_all_port_pins('VSS') + vss_sub_list[idx // 2],
                                           vss_tid))
                vdd_hm_list.append(
                    self.connect_to_tracks(inst_list[idx].get_all_port_pins('VDD') + vdd_sub_list[idx // 2],
                                           vdd_tid))

        # final stage input
        if vertical_out_idx + 1 < nstage:
            inn_vm_tidx = self.grid.coord_to_track(vm_layer, stage_out_list[-1][0][0].upper, RoundMode.LESS)
            inp_vm_tidx = self.grid.coord_to_track(vm_layer, stage_out_list[-1][1][0].lower, RoundMode.GREATER)
            tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig_w')
            sig_w_sep_half_vm = self.get_track_sep(vm_layer, tr_w_sig_vm, 1)
            inn_vm_tidx -= sig_w_sep_half_vm
            inp_vm_tidx += sig_w_sep_half_vm
            self.connect_to_tracks(stage_in_list[-1][1] + stage_out_list[-2][0],
                                   TrackID(vm_layer, inn_vm_tidx, tr_w_sig_vm, grid=self.grid))
            self.connect_to_tracks(stage_in_list[-1][0] + stage_out_list[-2][1],
                                   TrackID(vm_layer, inp_vm_tidx, tr_w_sig_vm, grid=self.grid))
            final_out_n, final_out_p = stage_out_list[-1][0], stage_out_list[-1][1]
            final_out_n_lower = final_out_n[0].lower
            final_out_p_upper = final_out_p[0].upper
            final_out_n_vm_tidx = self.grid.coord_to_track(vm_layer, final_out_n_lower,
                                                           RoundMode.GREATER) + sig_w_sep_half_vm
            final_out_p_vm_tidx = self.grid.coord_to_track(vm_layer, final_out_p_upper,
                                                           RoundMode.LESS) - sig_w_sep_half_vm
            final_out_n_vm = self.connect_to_tracks(final_out_n, TrackID(vm_layer, final_out_n_vm_tidx, tr_w_sig_vm,
                                                                         grid=self.grid))
            final_out_p_vm = self.connect_to_tracks(final_out_p, TrackID(vm_layer, final_out_p_vm_tidx, tr_w_sig_vm,
                                                                         grid=self.grid))
            final_out_layer = 7
            self.add_pin('outp' if nstage & 1 else 'outn', final_out_n_vm)
            self.add_pin('outn' if nstage & 1 else 'outp', final_out_p_vm)
        else:
            self.add_pin('outp' if nstage & 1 else 'outn', stage_out_list[-1][0])
            self.add_pin('outn' if nstage & 1 else 'outp', stage_out_list[-1][1])

        vss_hm_list = self.extend_wires(vss_hm_list, upper=self.bound_box.xh)
        vdd_hm_list = self.extend_wires(vdd_hm_list, upper=self.bound_box.xh)
        self.add_pin('VSS', vss_hm_list, connect=True)
        self.add_pin('VDD', vdd_hm_list, connect=True)
        self.add_pin('inn', inst_list[0].get_pin('inn'))
        self.add_pin('inp', inst_list[0].get_pin('inp'))

        params_list = []
        for idx in range(nstage):
            _seg_dict = dict()
            for key, val in seg_dict.items():
                _seg_dict[key] = int(scale_list[idx] * val * row_list[idx])
            params_list.append(dict(pinfo=(self.get_tile_subpattern(start_idx=1, stop_idx=2), self.tile_table),
                                    seg_dict=_seg_dict, coupling=idx < nstage - 1))
        master_list = [self.new_template(DiffInvCoupled, params=_p) for _p in params_list]

        self.sch_params = dict(inv_list=[m.sch_params for m in master_list])


class ClkRxBuf(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._sup_wire = {}

    @property
    def sup_wire(self) -> Dict[WireArray, Any]:
        return self._sup_wire

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_rx')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            rx_params='Parameters for buffer x',
            inv_diff_params='',
        )

    def draw_layout(self) -> None:
        rx_params = self.params['rx_params']
        inv_diff_params: Mapping[str, Any] = self.params['inv_diff_params']
        if isinstance(rx_params, str):
            rx_params = read_yaml(rx_params)
            rx_params = rx_params['params']

        if isinstance(inv_diff_params, str):
            inv_diff_params = read_yaml(inv_diff_params)
            inv_diff_params = inv_diff_params['params']['params']

        rx_master = self.new_template(ClkRx, params=rx_params)
        buf_master = self.new_template(GenericWrapper, params=dict(cls_name=DiffInvCoupledChain.get_qualified_name(),
                                                                   params=inv_diff_params,
                                                                   export_private=False))
        # --- Placement --- #
        buf_w, buf_h = buf_master.bound_box.w, buf_master.bound_box.h
        rx_w, rx_h = rx_master.bound_box.w, rx_master.bound_box.h
        top_layer = max(buf_master.top_layer, rx_master.top_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=True, half_blk_y=True)

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      inv_diff_params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        tot_w = max(buf_w, rx_w)
        blk_w_place, blk_h_place = self.grid.get_block_size(xm1_layer, half_blk_x=True, half_blk_y=True)
        tr_manager = buf_master.core.tr_manager

        rx_x = (tot_w - rx_w) // 2
        buf_x = (tot_w - buf_w) // 2
        rx_x = -(-rx_x // blk_w_place) * blk_w_place
        buf_x = -(-buf_x // blk_w_place) * blk_w_place
        rx = self.add_instance(rx_master, xform=Transform(rx_x, 0, mode=Orientation.R0))
        buf_y = rx.bound_box.yh
        buf_y = -(-buf_y // blk_h) * blk_h
        buf = self.add_instance(buf_master, xform=Transform(buf_x, buf_y, mode=Orientation.R0))
        tot_h = buf.bound_box.yh
        tot_h = -(-tot_h // blk_h) * blk_h

        buf_inn, buf_inp = buf.get_pin('inp'), buf.get_pin('inn')
        if rx.has_port('tia_out_n'):
            rx_outn, rx_outp = rx.get_pin('tia_out_n'), rx.get_pin('tia_out_p')
        else:
            rx_outn, rx_outp = rx.get_pin('von'), rx.get_pin('vop')
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        buf_inn_vm_tidx = buf.get_pin('outn').track_id
        buf_inp_vm_tidx = buf.get_pin('outp').track_id
        if inv_diff_params['nstage'] & 1:
            buf_inn_vm_tidx, buf_inp_vm_tidx = buf_inp_vm_tidx, buf_inn_vm_tidx
        buf_inn_vm = self.connect_to_tracks(buf_inn, buf_inp_vm_tidx)
        buf_inp_vm = self.connect_to_tracks(buf_inp, buf_inn_vm_tidx)

        # Connect to xm layer
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        buf_innp_xm_tidx = self.grid.coord_to_track(xm_layer, rx.bound_box.yh, RoundMode.NEAREST)
        buf_inn_xm = self.connect_to_tracks(rx_outn, TrackID(xm_layer, buf_innp_xm_tidx, tr_w_clk_xm))
        buf_inp_xm = self.connect_to_tracks(rx_outp, TrackID(xm_layer, buf_innp_xm_tidx, tr_w_clk_xm))

        self.connect_to_track_wires(buf_inn_xm, buf_inn_vm)
        self.connect_to_track_wires(buf_inp_xm, buf_inp_vm)

        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))
        basefill_bbox(self, BBox(buf.bound_box.xh, buf.bound_box.yl, self.bound_box.xh, self.bound_box.yh))
        basefill_bbox(self, BBox(self.bound_box.xl, buf.bound_box.yl, buf.bound_box.xl, self.bound_box.yh))

        bufn, bufp = buf.get_all_port_pins('outn')[0], buf.get_all_port_pins('outp')[0]
        buf_out_h = bufn.bound_box.h
        if inv_diff_params['nstage'] & 1:
            bufn, bufp = bufp, bufn

        bufn_stack_dict = self.via_stack_up(tr_manager, bufn, vm_layer, xm1_layer, 'sig',
                                            bbox=BBox(bufn.bound_box.xl, bufn.bound_box.yl + buf_out_h // 3,
                                                      bufn.bound_box.xh, bufn.bound_box.yh - buf_out_h // 3))
        bufp_stack_dict = self.via_stack_up(tr_manager, bufp, vm_layer, xm1_layer, 'sig',
                                            bbox=BBox(bufp.bound_box.xl, bufp.bound_box.yl + buf_out_h // 3,
                                                      bufp.bound_box.xh, bufp.bound_box.yh - buf_out_h // 3))

        # power fill
        vdd_hm_inv = buf.get_all_port_pins('VDD')
        vss_hm_inv = buf.get_all_port_pins('VSS')
        vdd_hm_inv = self.extend_wires(vdd_hm_inv, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm_inv = self.extend_wires(vss_hm_inv, lower=self.bound_box.xl, upper=self.bound_box.xh)
        sup_w_hm = tr_manager.get_width(hm_layer, 'sup')
        sup_w_vm = tr_manager.get_width(vm_layer, 'sup')
        vdd_list, vss_list = vdd_hm_inv, vss_hm_inv
        for idx in range(hm_layer, xm1_layer):
            sup_w = tr_manager.get_width(idx, 'sup')
            sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
            sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
            sup_bbox_all_l = BBox(self.bound_box.xl, buf.bound_box.yl,
                                  (self.bound_box.xl + self.bound_box.xh) // 2 - sup_w, buf.bound_box.yh)
            sup_bbox_all_r = BBox((self.bound_box.xl + self.bound_box.xh) // 2 + sup_w, buf.bound_box.yl,
                                  self.bound_box.xh, buf.bound_box.yh)
            idx_dir = self.grid.get_direction(idx)
            if idx_dir == Orient2D.y:
                vdd_list, vss_list = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx,
                                                              buf.bound_box)
            else:
                sup_l = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_l,
                                                 side_sup=False)
                sup_r = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_r,
                                                 side_sup=False, align_upper=True)
                vdd_list = sup_l[0] + sup_r[0]
                vss_list = sup_l[1] + sup_r[1]

        vdd_xm1, vss_xm1 = vdd_list, vss_list

        vss_ym1 = self.connect_to_track_wires(vss_xm1, rx.get_all_port_pins(f'VSS{ym1_layer}'))
        vdd_ym1 = self.connect_to_track_wires(vdd_xm1, rx.get_all_port_pins(f'VDD{ym1_layer}'))

        vss_ym1 = self.extend_wires(vss_ym1, upper=self.bound_box.yh)
        vdd_ym1 = self.extend_wires(vdd_ym1, upper=self.bound_box.yh)
        self.add_pin('VSS', vss_ym1)
        self.add_pin('VDD', vdd_ym1)

        if rx.has_port('tia_out_n'):
            self.add_pin('tia_out_n', rx.get_pin('tia_out_n'))
            self.add_pin('tia_out_p', rx.get_pin('tia_out_p'))
        else:
            self.add_pin('tia_out_n', rx.get_pin('von'))
            self.add_pin('tia_out_p', rx.get_pin('vop'))

        if rx.has_port('inn'):
            self.add_pin('inn', rx.get_pin('inn'))
            self.add_pin('inp', rx.get_pin('inp'))
        else:
            self.add_pin('inn', rx.get_pin('vin'))
            self.add_pin('inp', rx.get_pin('vip'))
        # self.add_pin('outp' if invert_output else 'outn', bufn_stack_dict[xm1_layer])
        # self.add_pin('outn' if invert_output else 'outp', bufp_stack_dict[xm1_layer])

        outn_ym1 = [self.connect_to_tracks(bufn_stack_dict[xm1_layer], w.track_id) for w in
                    rx.get_all_port_pins('inn_m', layer=ym1_layer)]
        outp_ym1 = [self.connect_to_tracks(bufp_stack_dict[xm1_layer], w.track_id) for w in
                    rx.get_all_port_pins('inp_m', layer=ym1_layer)]

        self.add_pin('outn', outn_ym1)
        self.add_pin('outp', outp_ym1)

        if rx.has_port('mid'):
            self.reexport(rx.get_port('mid'))

        vdd_list, vss_list = vdd_ym1, vss_ym1
        for idx in range(ym1_layer, rx_params['top_layer']):
            sup_w = tr_manager.get_width(idx, 'sup')
            sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
            sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
            sup_bbox_all_l = BBox(self.bound_box.xl, self.bound_box.yl,
                                  (self.bound_box.xl + self.bound_box.xh) // 2 - 2 * sup_w, self.bound_box.yh)
            sup_bbox_all_r = BBox((self.bound_box.xl + self.bound_box.xh) // 2 + 2 * sup_w, self.bound_box.yl,
                                  self.bound_box.xh, self.bound_box.yh)
            idx_dir = self.grid.get_direction(idx)
            if idx_dir == Orient2D.y:
                vdd_list, vss_list = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx,
                                                              self.bound_box)
            else:
                sup_l = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_l,
                                                 side_sup=False)
                sup_r = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_r,
                                                 side_sup=False, align_upper=True)
                vdd_list = sup_l[0] + sup_r[0]
                vss_list = sup_l[1] + sup_r[1]
            self.add_pin(f'VDD{idx + 1}', vdd_list, label='VDD')
            self.add_pin(f'VSS{idx + 1}', vss_list, label='VSS')

        # sch_params_dict = rx_master.sch_params.copy(append=dict(buf_params=buf_master.sch_params))
        self.sch_params = dict(
            tia_params=rx_master.sch_params,
            buf_params=buf_master.sch_params,
        )


class TermDiff(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._sup_wire = {}
        self._cap_top = 0
        self._core = None

    @property
    def core(self):
        return self._core

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_diff_term')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            res_term_params='Parameters for termination resistor',
            decap_params='Parameters for DC decoupling cap',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            res_term_params=None,
            decap_params=None,
        )

    def draw_layout(self) -> None:
        # Routing setting
        conn_layer = 1
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # make masters
        res_term_params: Optional[Mapping[str, Any]] = self.params.get('res_term_params', None)
        # termination
        res_term_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=TerminationVert.get_qualified_name(),
                                                                          params=res_term_params))
        res_term_w, res_term_h = res_term_master.bound_box.w, res_term_master.bound_box.h

        # initial decap master, for floorplanning
        decap_params: ImmutableSortedDict = self.params['decap_params']
        cap_gen_params = decap_params.copy()
        decap_master = self.new_template(DecapArray, params=cap_gen_params)

        sd_pitch = decap_master.core().core.arr_info.sd_pitch

        decap_seg = res_term_w // sd_pitch
        unit_params_resized = decap_params['unit_params'].copy().to_dict()
        unit_params_resized['seg'] = decap_seg
        cap_gen_params = dict(nx=1, ny=1, top_layer=cap_gen_params['top_layer'],
                              core_power=cap_gen_params['core_power'],
                              unit_params=unit_params_resized)
        decap_master = self.new_template(DecapArray, params=cap_gen_params)

        decap_w, decap_h = decap_master.bound_box.w, decap_master.bound_box.h

        # --- Placement --- #
        w_blk, h_blk = self.grid.get_block_size(ym1_layer)
        # Use a different grid for placement
        w_blk_place, h_blk_place = self.grid.get_block_size(xm1_layer)

        tot_w = max(res_term_w, decap_w)
        tot_h = res_term_h + decap_h
        tot_w, tot_h = round_to_blk_pitch(tot_w, tot_h, w_blk, h_blk)

        decap_xl = (tot_w - decap_w) // 2
        decap_yl = 0
        decap_xl, decap_yl = round_to_blk_pitch(decap_xl, decap_yl, w_blk_place, h_blk_place)
        decap_inst = self.add_instance(decap_master, xform=Transform(dx=decap_xl, dy=decap_yl, mode=Orientation.R0))

        res_term_xl = (tot_w - res_term_w) // 2
        res_term_yl = decap_h + 2 * h_blk_place
        res_term_xl, res_term_yl = round_to_blk_pitch(res_term_xl, res_term_yl, w_blk_place, h_blk_place)
        res_term_inst = self.add_instance(res_term_master, xform=Transform(dx=res_term_xl, dy=res_term_yl))

        tr_manager = res_term_master.core.tr_manager

        # --- Routing --- #

        # 3 ym1 at the middle for inp, gnd, inn
        _, ym1_locs = tr_manager.place_wires(ym1_layer, ['sig', 'sup', 'sig'], center_coord=tot_w // 2)

        # vip/ vin
        vdd_xm1 = []
        # export to xm1_layer
        vip_xm1 = bring_xm_to_xm1(self, res_term_inst.get_pin('PLUS'), tr_manager, xm_layer, 'sig',
                                  roundmode=RoundMode.GREATER)
        vin_xm1 = bring_xm_to_xm1(self, res_term_inst.get_pin('MINUS'), tr_manager, xm_layer, 'sig',
                                  roundmode=RoundMode.GREATER)
        vip_ym1, vin_ym1 = [], []
        vip_xm1 = self.add_wires(xm1_layer, vip_xm1.track_id.base_index, vip_xm1.middle, vip_xm1.upper,
                                 width=vip_xm1.track_id.width)
        vin_xm1 = self.add_wires(xm1_layer, vin_xm1.track_id.base_index, vin_xm1.lower, vin_xm1.middle,
                                 width=vin_xm1.track_id.width)
        mid_xm1 = bring_xm_to_xm1(self, res_term_inst.get_pin('MID'), tr_manager, xm_layer, 'sig',
                                  roundmode=RoundMode.LESS)
        mid_xm1 = self.add_wires(xm1_layer, mid_xm1.track_id.base_index, vip_xm1.lower, vin_xm1.upper,
                                 width=mid_xm1.track_id.width)
        vip_xm2 = bring_xm_to_xm1(self, vip_xm1, tr_manager, xm1_layer, 'sig', ret_ym_wire=vip_ym1)
        vin_xm2 = bring_xm_to_xm1(self, vin_xm1, tr_manager, xm1_layer, 'sig', ret_ym_wire=vin_ym1)
        vdd_res = res_term_inst.get_all_port_pins('VDD', xm_layer)
        for vdd in vdd_res:
            vdd_xm1.append(bring_xm_to_xm1(self, vdd, tr_manager, xm_layer, 'sup'))

        # mid
        decap_top_layer = decap_params['top_layer']
        mid_xm1_cap = decap_inst.get_all_port_pins(f'PLUS{decap_top_layer}')

        # get mid ym1 tidx
        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        tr_sp_sup_ym1 = tr_manager.get_sep(ym1_layer, ('sup', 'sup'))
        mid_ym1_tidx_l = self.grid.coord_to_track(ym1_layer, mid_xm1.lower, RoundMode.NEAREST)
        mid_ym1_tidx_r = self.grid.coord_to_track(ym1_layer, mid_xm1.upper, RoundMode.NEAREST)
        mid_ym1_tidx_list = self.get_tids_between(ym1_layer, mid_ym1_tidx_l, mid_ym1_tidx_r,
                                                  width=tr_w_sup_ym1, sep=tr_sp_sup_ym1, sep_margin=0,
                                                  include_last=True)[2:-2]
        mid_ym1 = [self.connect_to_tracks(mid_xm1_cap + [mid_xm1], tidx)
                   for tidx in mid_ym1_tidx_list]

        ym2_layer = vip_xm2.layer_id + 1
        tr_w_sig_ym2 = tr_manager.get_width(ym2_layer, 'sig')
        vin_top_tidx = self.grid.coord_to_track(ym2_layer, vin_xm2.middle, RoundMode.NEAREST)
        vip_top_tidx = self.grid.coord_to_track(ym2_layer, vip_xm2.middle, RoundMode.NEAREST)
        vip_ym2 = self.connect_to_tracks([vip_xm2], TrackID(ym2_layer, vip_top_tidx, width=tr_w_sig_ym2),
                                         min_len_mode=MinLenMode.MIDDLE)
        vin_ym2 = self.connect_to_tracks([vin_xm2], TrackID(ym2_layer, vin_top_tidx, width=tr_w_sig_ym2),
                                         min_len_mode=MinLenMode.MIDDLE)
        in_top_layer = ym2_layer + 1
        in_top_tidx = self.grid.coord_to_track(in_top_layer, vip_ym2.middle, RoundMode.NEAREST)
        tr_w_top_sig = tr_manager.get_width(in_top_layer, 'sig')
        vip_top = self.connect_to_tracks(vip_ym2, TrackID(in_top_layer, in_top_tidx, tr_w_top_sig))
        vin_top = self.connect_to_tracks(vin_ym2, TrackID(in_top_layer, in_top_tidx, tr_w_top_sig))
        self.add_pin('inp_m', [vip_xm1] + vip_ym1, hide=True)
        self.add_pin('inn_m', [vin_xm1] + vin_ym1, hide=True)
        self.add_pin('inp', vip_xm2)
        self.add_pin('inn', vin_xm2)

        # set size
        self.set_size_from_bound_box(ym1_layer, BBox(0, 0, tot_w, tot_h))

        # Bring supplies to ym1
        num_side_ym1 = 4
        sup_ym1_l_tidx_start = self.grid.coord_to_track(ym1_layer, 0, RoundMode.GREATER_EQ)
        sup_ym1_r_tidx_start = self.grid.coord_to_track(ym1_layer, self.bound_box.xh, RoundMode.LESS_EQ)
        sup_ym1_locs_l, sup_ym1_locs_r = [sup_ym1_l_tidx_start], [sup_ym1_r_tidx_start]
        for idx in range(num_side_ym1 * 2 - 1):
            sup_ym1_locs_l.append(tr_manager.get_next_track(ym1_layer, sup_ym1_locs_l[-1], 'sup', 'sup'))
            sup_ym1_locs_r.append(tr_manager.get_next_track(ym1_layer, sup_ym1_locs_r[-1], 'sup', 'sup', up=False))

        vdd_ym1_locs = sup_ym1_locs_l[::2] + sup_ym1_locs_r[::2]
        vss_ym1_locs = sup_ym1_locs_l[1::2] + sup_ym1_locs_r[1::2]
        vdd_ym1, vss_ym1 = [], []
        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        for tidx in vdd_ym1_locs:
            vdd_ym1.append(self.connect_to_tracks(vdd_xm1, TrackID(ym1_layer, tidx, tr_w_sup_ym1)))
        for tidx in vss_ym1_locs:
            vss_ym1.append(self.connect_to_tracks(decap_inst.get_all_port_pins(f'MINUS{decap_top_layer}'),
                                                  TrackID(ym1_layer, tidx, tr_w_sup_ym1)))

        # set size
        self.set_size_from_bound_box(ym1_layer, BBox(0, 0, tot_w, tot_h))
        vdd_ym1 = self.extend_wires(vdd_ym1, lower=self.bound_box.yl, upper=self.bound_box.yh)
        vss_ym1 = self.extend_wires(vss_ym1, lower=self.bound_box.yl, upper=self.bound_box.yh)

        self.add_pin('VDD', vdd_ym1)
        self.add_pin('VSS', vss_ym1)
        self.add_pin('mid', mid_ym1)

        self._core = res_term_master.core
        # set schematic parameters
        self.sch_params = dict(
            res_params=res_term_master.sch_params,
            cap_params=decap_master.sch_params,
        )
