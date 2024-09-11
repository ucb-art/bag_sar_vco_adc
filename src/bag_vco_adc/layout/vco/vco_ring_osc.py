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


from typing import Any, Dict, List, Mapping, Union, Type, Optional, Tuple

from bag.design.database import ModuleDB, Module
from bag.io import read_yaml
from bag.layout.routing.base import WireArray, HalfInt, TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param
from bag3_digital.layout.stdcells.gates import InvCore
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.util import fill_conn_layer_intv, connect_hm_sup_ow, draw_dummy_route
from bag_vco_adc.layout.util.wrapper import GenericWrapper
from pybag.core import Transform, BBox
from pybag.enum import MinLenMode, RoundMode, Orient2D
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase, SupplyColumnInfo
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo
from xbase.layout.mos.primitives import MOSConn
from ..util.template import TrackIDZL as TrackID


class RingOscUnit(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._col_tot = 0
        self._dum_info = []
        self._center_col = 0

    @property
    def center_col(self) -> int:
        return self._center_col

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_dict='Width',
            out_buf='True to enable output buffers',
            sig_locs='Signal locations',
            is_dum='Dummy row'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            out_buf=True,
            self_coupled=False,
            sig_locs={},
            is_dum=False
        )

    def get_mos_conn_template(self, tile_idx: int, row_idx: int, stack: int, seg: int):
        pinfo = self.get_tile_pinfo(tile_idx)
        rpinfo = pinfo.get_row_place_info(row_idx)
        row_info = rpinfo.row_info
        w_max = row_info.width

        conn_layer = self.conn_layer
        params = dict(
            row_info=row_info,
            conn_layer=conn_layer,
            seg=seg,
            w=w_max,
            stack=stack,
            arr_options=self.arr_info.arr_options,
            g_on_s=False,
            options={},
        )
        master = self.new_template(MOSConn, params=params)
        return master

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Dict[str, int] = self.params['seg_dict']
        w_dict: Dict[str, int] = self.params['w_dict']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']

        out_buf: bool = self.params['out_buf']
        is_dum: bool = self.params['is_dum']

        seg_inv = seg_dict['inv']
        seg_buf = seg_dict['buf']
        seg_coupler = seg_dict['coupler']

        wn = w_dict['wn']
        wp = w_dict['wp']
        wn_buf = w_dict['wn_buf']
        wp_buf = w_dict['wp_buf']
        wn_coupled = w_dict['wn_coupled']
        wp_coupled = w_dict['wp_coupled']

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        min_sep = self.min_sep_col

        nd0 = self.get_track_index(0, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pd0 = self.get_track_index(1, MOSWireType.DS, wire_name='sig', wire_idx=1)

        ng1 = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2 = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=2)
        pg2 = self.get_track_index(1, MOSWireType.G, wire_name='sig', wire_idx=-3 if out_buf else -2)
        pg1 = self.get_track_index(1, MOSWireType.G, wire_name='sig', wire_idx=-2 if out_buf else -1)

        # Calculate segments

        flip_main_couple = True if sig_locs.get('nout', 0) < -2 else False
        out_locs = sig_locs.get('nout', 0) + 4 if flip_main_couple else sig_locs.get('nout', 0)
        nout_vm_tidx = self.arr_info.col_to_track(vm_layer, out_locs + seg_inv)
        pout_vm_tidx = self.arr_info.col_to_track(vm_layer, out_locs + seg_inv)
        inv_n_params = dict(pinfo=pinfo, seg=seg_inv, w_p=wp, w_n=wn, vertical_out=True, vertical_sup=True,
                            sig_locs={'pout': pd0, 'nout': nd0, 'nin': ng2, 'out': nout_vm_tidx})
        inv_p_params = dict(pinfo=pinfo, seg=seg_inv, w_p=wp, w_n=wn, vertical_out=True, vertical_sup=True,
                            sig_locs={'pout': pd0, 'nout': nd0, 'nin': pg2, 'out': pout_vm_tidx})
        coupler_n_params = dict(pinfo=pinfo, seg=seg_coupler, w_p=wp_coupled, w_n=wn_coupled, vertical_out=False,
                                sig_locs={'pout': pd0, 'nout': nd0, 'nin': ng1}, vertical_sup=True)
        coupler_p_params = dict(pinfo=pinfo, seg=seg_coupler, w_p=wp_coupled, w_n=wn_coupled, vertical_out=False,
                                sig_locs={'pout': pd0, 'nout': nd0, 'nin': pg1}, vertical_sup=True)
        if out_buf:
            ng0 = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=0)
            pg0 = self.get_track_index(1, MOSWireType.G, wire_name='sig', wire_idx=-1)
            buf_n_params = dict(pinfo=pinfo, seg=seg_buf, w_p=wp_buf, w_n=wn_buf, vertical_out=True,
                                sig_locs={'pout': pd0, 'nout': nd0, 'nin': ng0}, vertical_sup=True)
            buf_p_params = dict(pinfo=pinfo, seg=seg_buf, w_p=wp_buf, w_n=wn_buf, vertical_out=True,
                                sig_locs={'pout': pd0, 'nout': nd0, 'nin': pg0}, vertical_sup=True)
        else:
            buf_n_params, buf_p_params = None, None

        tap_ncol = self.min_sub_col
        tap_sep_col = self.sub_sep_col
        # add taps
        vdd_list: List[WireArray] = []
        vss_list: List[WireArray] = []

        cur_loc = 0
        # Make sure all vertical routings area within range
        min_half_ncol = sig_locs.get('half_col_min', 0)  # set minmum half osc ncol
        cur_loc += max(0, min_half_ncol - seg_inv - seg_coupler - min_sep)

        # left bnd of RO
        core_l = cur_loc

        coupler_master = self.new_template(InvCore, params=coupler_n_params)
        inv_master = self.new_template(InvCore, params=inv_n_params)
        if flip_main_couple:
            inv_n = self.add_tile(inv_master, 0, cur_loc)
            coupler_n = self.add_tile(coupler_master, 0, cur_loc + seg_inv + min_sep)
        else:
            coupler_n = self.add_tile(coupler_master, 0, cur_loc)
            inv_n = self.add_tile(inv_master, 0, cur_loc + seg_coupler + min_sep)
        cur_loc += seg_inv + seg_coupler + min_sep  # space taken by two invs
        cur_loc += min_sep + 1  # mid space for supply

        coupler_master = self.new_template(InvCore, params=coupler_p_params)
        inv_master = self.new_template(InvCore, params=inv_p_params)
        # right bnd of RO
        core_r = cur_loc + max(min_half_ncol, seg_coupler + seg_inv + min_sep)
        self._center_col = cur_loc - seg_inv - min_sep // 2
        if flip_main_couple:
            coupler_p = self.add_tile(coupler_master, 0, cur_loc + seg_coupler, flip_lr=True)
            inv_p = self.add_tile(inv_master, 0, cur_loc + seg_inv + min_sep + seg_coupler, flip_lr=True)
        else:
            inv_p = self.add_tile(inv_master, 0, cur_loc + seg_inv, flip_lr=True)
            coupler_p = self.add_tile(coupler_master, 0, cur_loc + seg_inv + seg_coupler + min_sep, flip_lr=True)

        sup_vm_sep_col = self.arr_info.track_to_col(vm_layer, tr_manager.get_sep(vm_layer, ('sup', 'sup')))
        cur_loc = core_r

        if out_buf:
            buf_master = self.new_template(InvCore, params=buf_n_params)
            cur_loc = core_r + min_sep
            buf_n = self.add_tile(buf_master, 0, cur_loc)
            buf_master = self.new_template(InvCore, params=buf_p_params)
            buf_p = self.add_tile(buf_master, 0, cur_loc + 2 * seg_buf + min_sep, flip_lr=True)
            cur_loc += 2 * seg_buf + min_sep
        else:
            buf_n, buf_p = None, None

        cur_loc += tap_ncol + tap_sep_col + 2 * sup_vm_sep_col
        self.add_tap(cur_loc, vdd_list, vss_list, flip_lr=True)
        vdd_list = connect_hm_sup_ow(self, vdd_list, 0, -1, tr_manager)
        vss_list = connect_hm_sup_ow(self, vss_list, 0, 0, tr_manager)

        self.set_mos_size(cur_loc + tap_sep_col)

        fill_conn_layer_intv(self, 0, 0, start_col=0, stop_col=self.num_cols, extend_to_gate=False)
        fill_conn_layer_intv(self, 0, 1, start_col=0, stop_col=self.num_cols, extend_to_gate=False)

        # Connect vco core supply
        core_vdd_list, core_vss_list = [], []
        core_inst_list = [coupler_p, coupler_n, inv_p, inv_n]
        if out_buf:
            core_inst_list.extend([buf_n, buf_p])
        for inst in core_inst_list:
            core_vdd_list.append(inst.get_pin('VDD'))
            core_vss_list.append(inst.get_pin('VSS'))

        core_l_coord = self.arr_info.col_to_coord(core_l)
        core_r_coord = self.arr_info.col_to_coord(core_r)
        core_vdd_list = connect_hm_sup_ow(self, core_vdd_list, 0, -1, tr_manager)
        core_vss_list = connect_hm_sup_ow(self, core_vss_list, 0, 0, tr_manager)
        core_vdd_list = self.extend_wires(core_vdd_list, lower=core_l_coord, upper=core_r_coord)
        core_vss_list = self.extend_wires(core_vss_list, lower=core_l_coord, upper=core_r_coord)
        core_vdd_hm = self.connect_wires(core_vdd_list)
        core_vss_hm = self.connect_wires(core_vss_list)

        if is_dum:
            self.connect_differential_wires([inv_n.get_pin('nin'), coupler_p.get_pin('nin')],
                                            [inv_p.get_pin('nin'), coupler_n.get_pin('nin')],
                                            inv_n.get_pin('out'), inv_p.get_pin('out'))
            self.connect_to_track_wires([inv_n.get_pin('nout'), coupler_n.get_pin('nout'), coupler_n.get_pin('pout')] +
                                        core_vss_hm, inv_n.get_pin('out'))
            self.connect_to_track_wires([inv_p.get_pin('nout'), coupler_p.get_pin('nout'), coupler_p.get_pin('pout')] +
                                        core_vss_hm, inv_p.get_pin('out'))
            if out_buf:
                self.connect_differential_wires(buf_n.get_pin('nin'), buf_p.get_pin('nin'), inv_n.get_pin('out'),
                                                inv_p.get_pin('out'))
                self.connect_to_track_wires([buf_n.get_pin('out'), buf_p.get_pin('out')], core_vss_hm)
            vss_hm = vss_list
            self.add_pin('VSS', vss_hm)
        else:
            self.connect_wires([inv_n.get_pin('nout'), coupler_n.get_pin('nout')])
            self.connect_wires([inv_p.get_pin('nout'), coupler_p.get_pin('nout')])
            self.connect_wires([inv_n.get_pin('pout'), coupler_n.get_pin('pout')])
            self.connect_wires([inv_p.get_pin('pout'), coupler_p.get_pin('pout')])

            inn, inp = inv_n.get_pin('nin'), inv_p.get_pin('nin')
            couplern, couplerp = coupler_n.get_pin('nin'), coupler_p.get_pin('nin')
            in_hm_max_coord = max([pin.upper for pin in [inn, inp, couplern, couplerp]])
            in_hm_min_coord = min([pin.lower for pin in [inn, inp, couplern, couplerp]])

            inn = self.extend_wires(inn, upper=in_hm_max_coord, lower=in_hm_min_coord)
            inp = self.extend_wires(inp, upper=in_hm_max_coord, lower=in_hm_min_coord)
            couplern = self.extend_wires(couplern, upper=in_hm_max_coord, lower=in_hm_min_coord)
            couplerp = self.extend_wires(couplerp, upper=in_hm_max_coord, lower=in_hm_min_coord)
            if out_buf:
                self.connect_differential_wires(buf_n.get_pin('nin'), buf_p.get_pin('nin'), inv_n.get_pin('out'),
                                                inv_p.get_pin('out'))
            self.add_pin('inn', inn, show=self.show_pins)
            self.add_pin('inp', inp, show=self.show_pins)
            self.add_pin('couplern', couplern, show=self.show_pins)
            self.add_pin('couplerp', couplerp, show=self.show_pins)

        # Supply connections
        sup_vm_l = self.grid.coord_to_track(vm_layer, min(inv_n.bound_box.xl, coupler_n.bound_box.xl), RoundMode.LESS)
        sup_vm_r = self.grid.coord_to_track(vm_layer, max(inv_p.bound_box.xh, coupler_p.bound_box.xh),
                                            RoundMode.GREATER)
        sup_vm_m = self.grid.coord_to_track(vm_layer, (inv_p.bound_box.xh + inv_n.bound_box.xl) // 2, RoundMode.NEAREST)
        sup_vm_tidx_list = [sup_vm_l, sup_vm_r, sup_vm_m]

        if out_buf:
            sup_vm_buf_l = self.grid.coord_to_track(vm_layer, buf_n.bound_box.xl, RoundMode.LESS)
            sup_vm_buf_r = self.grid.coord_to_track(vm_layer, buf_p.bound_box.xh, RoundMode.GREATER)
            sup_vm_buf_m = self.grid.coord_to_track(vm_layer, (buf_n.bound_box.xh + buf_p.bound_box.xl) // 2,
                                                    RoundMode.NEAREST)
            sup_vm_tidx_list.extend([sup_vm_buf_l, sup_vm_buf_r, sup_vm_buf_m])

        # nom_sup_vm_r = tr_manager.get_next_track(vm_layer, nom_sup_vm_r, 'sig', 'sup', up=True)
        nom_sup_vm_edge = self.arr_info.col_to_track(vm_layer, self.num_cols, mode=RoundMode.GREATER)
        nom_sup_vm_edge = tr_manager.get_next_track(vm_layer, nom_sup_vm_edge, 'sig', 'sup', up=False)
        nom_sup_vm_inner = self.arr_info.col_to_track(vm_layer, self.num_cols - tap_ncol - 2 * tap_sep_col,
                                                      mode=RoundMode.LESS)
        nom_sup_vm_inner = tr_manager.get_next_track(vm_layer, nom_sup_vm_inner, 'sig', 'sup', up=True)

        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')

        core_vdd_vm, core_vss_vm = [], []
        for _sup_vm in sup_vm_tidx_list:
            core_vdd_vm.append(self.connect_to_tracks(core_vdd_hm, TrackID(vm_layer, _sup_vm, tr_w_sup_vm),
                                                      min_len_mode=MinLenMode.MIDDLE))
            core_vss_vm.append(self.connect_to_tracks(core_vss_hm, TrackID(vm_layer, _sup_vm, tr_w_sup_vm),
                                                      min_len_mode=MinLenMode.MIDDLE))

        vss_vm = [self.connect_to_tracks(vss_list, TrackID(vm_layer, nom_sup_vm_edge, tr_w_sup_vm),
                                         min_len_mode=MinLenMode.MIDDLE),
                  self.connect_to_tracks(vss_list, TrackID(vm_layer, nom_sup_vm_inner, tr_w_sup_vm),
                                         min_len_mode=MinLenMode.MIDDLE)]
        vdd_vm = [self.connect_to_tracks(vdd_list, TrackID(vm_layer, nom_sup_vm_edge, tr_w_sup_vm),
                                         min_len_mode=MinLenMode.MIDDLE),
                  self.connect_to_tracks(vdd_list, TrackID(vm_layer, nom_sup_vm_inner, tr_w_sup_vm),
                                         min_len_mode=MinLenMode.MIDDLE)]

        # Connect supplies to xm layer
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        vss_xm_coord = self.grid.track_to_coord(hm_layer, core_vss_hm[0].track_id.base_index)
        vss_xm_tid = self.grid.coord_to_track(xm_layer, vss_xm_coord, RoundMode.NEAREST)
        core_vss_xm = self.connect_to_tracks(core_vss_vm, TrackID(xm_layer, vss_xm_tid, tr_w_sup_xm, grid=self.grid),
                                             track_lower=core_vss_hm[0].lower, track_upper=core_vdd_hm[0].upper)
        vss_xm = self.connect_to_tracks(vss_vm, TrackID(xm_layer, vss_xm_tid, tr_w_sup_xm, grid=self.grid),
                                        track_lower=vss_list[0].lower, track_upper=vss_list[0].upper)

        vdd_xm_coord = self.grid.track_to_coord(hm_layer, core_vdd_hm[0].track_id.base_index)
        vdd_xm_tid = self.grid.coord_to_track(xm_layer, vdd_xm_coord, RoundMode.NEAREST)
        core_vdd_xm = self.connect_to_tracks(core_vdd_vm, TrackID(xm_layer, vdd_xm_tid, tr_w_sup_xm, grid=self.grid),
                                             track_lower=core_vdd_hm[0].lower, track_upper=core_vdd_hm[0].upper)
        vdd_xm = self.connect_to_tracks(vdd_vm, TrackID(xm_layer, vdd_xm_tid, tr_w_sup_xm, grid=self.grid),
                                        track_lower=vdd_list[0].lower, track_upper=vdd_list[0].upper)

        # Connect supplies to ym layer
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_sp_sup_ym = self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_sup_ym)
        # sup_ym_locs = tr_manager.spread_wires(ym_layer, ['sup'] * 5,
        #                                       self.arr_info.col_to_track(vm_layer, 0),
        #                                       self.arr_info.col_to_track(ym_layer, 2*min_half_ncol+min_sep+1),
        #                                       ('sup', 'sup'))
        sup_ym_core_mid_tidx = self.grid.coord_to_track(ym_layer, (inv_n.bound_box.xh + inv_p.bound_box.xl) // 2,
                                                        RoundMode.NEAREST)
        sup_ym_locs = [sup_ym_core_mid_tidx - 2 * tr_sp_sup_ym, sup_ym_core_mid_tidx - tr_sp_sup_ym,
                       sup_ym_core_mid_tidx,
                       sup_ym_core_mid_tidx + tr_sp_sup_ym, sup_ym_core_mid_tidx + 2 * tr_sp_sup_ym]
        if out_buf:
            sup_ym_locs.extend([sup_ym_locs[-1] + tr_sp_sup_ym, sup_ym_locs[-1] + 2 * tr_sp_sup_ym])

        ret_supply_xm_list = []
        if not is_dum:
            # core
            core_vdd_ym, core_vss_ym = [], []
            for idx in sup_ym_locs[::2]:
                core_vdd_ym.append(self.connect_to_tracks(core_vdd_xm, TrackID(ym_layer, idx, tr_w_sup_ym),
                                                          min_len_mode=MinLenMode.MIDDLE,
                                                          ret_wire_list=ret_supply_xm_list))
            for idx in sup_ym_locs[1::2]:
                core_vss_ym.append(self.connect_to_tracks(core_vss_xm, TrackID(ym_layer, idx, tr_w_sup_ym),
                                                          min_len_mode=MinLenMode.MIDDLE,
                                                          ret_wire_list=ret_supply_xm_list))
            core_vdd_ym = self.extend_wires(core_vdd_ym, lower=self.bound_box.yl, upper=self.bound_box.yh)

            core_vss_ym = self.extend_wires(core_vss_ym, lower=self.bound_box.yl, upper=self.bound_box.yh)
        else:
            core_vdd_ym, core_vss_ym = [], []
            for idx in sup_ym_locs[::2]:
                core_vdd_ym.append(self.connect_to_tracks([core_vdd_xm, core_vss_xm],
                                                          TrackID(ym_layer, idx, tr_w_sup_ym),
                                                          min_len_mode=MinLenMode.MIDDLE,
                                                          ret_wire_list=ret_supply_xm_list))
            core_vdd_ym = self.extend_wires(core_vdd_ym, lower=self.bound_box.yl, upper=self.bound_box.yh)

        self.match_warr_length(ret_supply_xm_list)
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        sup_ym_locs = [tr_manager.get_next_track(ym_layer, sup_ym_locs[-1], 'sup', 'sup', up=2),
                       tr_manager.get_next_track(ym_layer, sup_ym_locs[-1], 'sup', 'sup', up=3)]
        ret_supply_xm_list = []
        vdd_ym = [self.connect_to_tracks(vdd_xm, TrackID(ym_layer, sup_ym_locs[0], tr_w_sup_ym),
                                         ret_wire_list=ret_supply_xm_list)]
        vdd_ym = self.extend_wires(vdd_ym, lower=self.bound_box.yl, upper=self.bound_box.yh)

        vss_ym = [self.connect_to_tracks(vss_xm, TrackID(ym_layer, sup_ym_locs[1], tr_w_sup_ym),
                                         ret_wire_list=ret_supply_xm_list)]
        vss_ym = self.extend_wires(vss_ym, lower=self.bound_box.yl, upper=self.bound_box.yh)

        xm1_tidx_list = [self.grid.coord_to_track(xm1_layer, 0, mode=RoundMode.NEAREST),
                         self.grid.coord_to_track(xm1_layer, self.bound_box.h, mode=RoundMode.NEAREST)]
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        if not is_dum:
            vdd_core_xm1 = self.connect_to_tracks(core_vdd_ym, TrackID(xm1_layer, xm1_tidx_list[1], tr_w_sup_xm1,
                                                                       grid=self.grid))
            vco_xm1 = self.connect_to_tracks(core_vss_ym, TrackID(xm1_layer, xm1_tidx_list[0], tr_w_sup_xm1,
                                                                  grid=self.grid))
            vdd_core_xm1 = self.extend_wires(vdd_core_xm1, lower=self.bound_box.xl)
            vco_xm1 = self.extend_wires(vco_xm1, lower=self.bound_box.xl, upper=vdd_core_xm1[0].upper)
            self.match_warr_length([vdd_core_xm1, vco_xm1])
            self.add_pin('VDD_core_xm1', vdd_core_xm1)
            self.add_pin('VCO_xm1', vco_xm1)
        vdd_xm1 = self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx_list[1], tr_w_sup_xm1, grid=self.grid))
        vss_xm1 = self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx_list[0], tr_w_sup_xm1, grid=self.grid))

        vss_xm1 = self.extend_wires(vss_xm1, lower=vdd_xm1.lower, upper=self.bound_box.xh)
        vdd_xm1 = self.extend_wires(vdd_xm1, lower=vdd_xm1.lower, upper=self.bound_box.xh)

        for warr in [ret_supply_xm_list, vdd_xm1 + vss_xm1]:
            self.match_warr_length(warr)

        self.add_pin('VDD_core', core_vdd_hm, show=self.show_pins)
        self.add_pin('VSS_core', core_vss_hm, show=self.show_pins)
        self.add_pin('VDD_core', core_vdd_vm, show=self.show_pins)
        self.add_pin('VSS_core', core_vss_vm, show=self.show_pins)
        self.add_pin('VDD_r', vdd_list, show=self.show_pins, connect=True)
        self.add_pin('VSS_r', vss_list, show=self.show_pins, connect=True)
        self.add_pin('VDD', vdd_vm, show=self.show_pins)
        self.add_pin('VSS', vss_vm, show=self.show_pins)

        self.add_pin('VDD_xm', vdd_xm, label='VDD')
        self.add_pin('VSS_xm', vss_xm, label='VSS')
        self.add_pin('VDD_core_xm', core_vdd_xm, label='VDD')
        self.add_pin('VSS_core_xm', core_vss_xm, label='VSS')
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD')
        self.add_pin('VSS_xm1', vss_xm1, label='VSS')

        self.add_pin('VDD_ym', vdd_ym, label='VDD')
        self.add_pin('VSS_ym', vss_ym, label='VSS')
        self.add_pin('VDD_core_ym', core_vdd_ym)
        self.add_pin('VSS_core_ym', core_vss_ym)

        self.reexport(inv_n.get_port('out'), net_name='outn', show=self.show_pins)
        self.reexport(inv_p.get_port('out'), net_name='outp', show=self.show_pins)
        if not is_dum and out_buf:
            self.reexport(buf_n.get_port('out'), net_name='buf_outn', show=self.show_pins)
            self.reexport(buf_p.get_port('out'), net_name='buf_outp', show=self.show_pins)

        self._sch_params = dict(
            delay_params=dict(
                lch=self.arr_info.lch,
                wp=wp,
                wn=wn,
                wp_coupled=wp_coupled,
                wn_coupled=wn_coupled,
                pth=self.get_tile_info(0)[0].get_row_place_info(1).row_info.threshold,
                nth=self.get_tile_info(0)[0].get_row_place_info(0).row_info.threshold,
                seg_n=seg_inv,
                seg_p=seg_inv,
                seg_n_coupled=seg_coupler,
                seg_p_coupled=seg_coupler,
                self_coupled=False,
                out_buf=out_buf,
            ),
            buf_params=dict(
                lch=self.arr_info.lch,
                wp=wp_buf,
                wn=wn_buf,
                pth=self.get_tile_info(0)[0].get_row_place_info(1).row_info.threshold,
                nth=self.get_tile_info(0)[0].get_row_place_info(0).row_info.threshold,
                seg_n=seg_buf,
                seg_p=seg_buf,
            ),
            vtop_buf='VTOP',
            vbot_buf='VBOT',
        )

    def get_supply_column_info(self, top_layer: int, tile_idx: int = 0) -> SupplyColumnInfo:
        grid = self.grid
        ainfo = self._arr_info
        tr_manager = ainfo.tr_manager

        pinfo = self.get_tile_pinfo(tile_idx)
        if not pinfo.is_complementary:
            raise ValueError('Currently only works on complementary tiles.')

        if top_layer <= self.conn_layer:
            raise ValueError(f'top_layer must be at least {self.conn_layer + 1}')
        if grid.get_direction(top_layer) == Orient2D.x:
            top_vm_layer = top_layer - 1
        else:
            top_vm_layer = top_layer

        # get total number of columns
        num_col = self.get_tap_ncol() + self.sub_sep_col
        tr_info_list = []
        for vm_lay in range(self.conn_layer + 2, top_vm_layer + 1, 2):
            blk_ncol = ainfo.get_block_ncol(vm_lay)
            tr_w = tr_manager.get_width(vm_lay, 'sup')
            tr_sep = tr_manager.get_sep(vm_lay, ('sup', 'sup'), half_space=False)
            ntr = 2 * tr_sep
            cur_ncol = -(-ainfo.get_column_span(vm_lay, ntr) // blk_ncol) * blk_ncol
            num_col = max(num_col, cur_ncol)
            tr_info_list.append((tr_w, tr_sep))

        # make sure we can draw substrate contact
        num_col += (num_col & 1)
        return SupplyColumnInfo(ncol=num_col, top_layer=top_layer, tr_info=tr_info_list)


class RingOscCol(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._col_tot = 0
        self._dum_info = []
        self._center_col = 0
        self._row_idx = []

    @property
    def row_idx(self) -> List[int]:
        return self._row_idx

    @property
    def center_col(self) -> int:
        return self._center_col

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_ro_diff_coupled')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_dict='Width',
            num_stages='Number of RO stages',
            delta='delta between ro and coupler, delta=0 means self coupled',
            out_buf='True to enable output buffers',
            sig_locs='Signal locations',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            delta=2,
            num_stage=4,
            out_buf=True,
            self_coupled=False,
            sig_locs={},
        )

    def get_mos_conn_template(self, tile_idx: int, row_idx: int, stack: int, seg: int):
        pinfo = self.get_tile_pinfo(tile_idx)
        rpinfo = pinfo.get_row_place_info(row_idx)
        row_info = rpinfo.row_info
        w_max = row_info.width

        conn_layer = self.conn_layer
        params = dict(
            row_info=row_info,
            conn_layer=conn_layer,
            seg=seg,
            w=w_max,
            stack=stack,
            arr_options=self.arr_info.arr_options,
            g_on_s=False,
            options={},
        )
        master = self.new_template(MOSConn, params=params)
        return master

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Dict[str, int] = self.params['seg_dict']
        w_dict: Dict[str, int] = self.params['w_dict']
        num_stages: int = self.params['num_stages']
        out_buf: int = self.params['out_buf']
        delta: int = self.params['delta']
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        # === 1. Compute necessary infomation for RO ===
        num_g_tids = 6  # Necessary tracks for coupled inverter routing
        ng_tidx_list, pg_tidx_list = list(range(num_g_tids)), list(range(num_g_tids))
        ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * num_g_tids + ['sup'])
        half_col_min = self.arr_info.track_to_col(vm_layer, ntr, mode=RoundMode.GREATER_EQ)
        # also leave space that can fit in 5 ym supply
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        # half_col_min_sup = self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_sup_ym)
        # half_col_min_sup = 2 * half_col_min_sup + half_col_min_sup.div2()

        half_col_min = max(half_col_min, 0)

        # -- 1.1 Flip inverters, make flip n,p sides --
        flip_np_list = []
        for idx in range(num_stages // 2):
            if idx & 1:
                flip_np_list.extend([False, True])
            else:
                flip_np_list.extend([True, False])

        sig_locs_list = [{} for _ in range(num_stages)]
        sig_locs_map = [
            {'in_n': (1, ng_tidx_list[2]), 'in_p': (1, ng_tidx_list[1])},
            {'in_n': (0, pg_tidx_list[2]), 'in_p': (1, ng_tidx_list[0])},
            {'in_n': (0, pg_tidx_list[0]), 'in_p': (0, pg_tidx_list[1])}
        ]

        # -- 1.2 Compute signals' list for main ring --
        coupled_index = [1, 2, 1, 0]
        unit_idx_list = [idx * 2 for idx in range(num_stages // 2)] + [idx * 2 + 1 for idx in range(num_stages // 2)][
                                                                      ::-1]
        unit_idx_list = unit_idx_list + unit_idx_list[0:delta]
        for idx in range(num_stages):
            if idx == 0:
                sig_locs_list[idx].update(sig_locs_map[1])
            elif idx == num_stages - 1:
                _idx = idx // 2
                sig_locs_list[idx].update(sig_locs_map[_idx % 3])
            elif idx & 1:
                _idx = (idx - 1) // 2 + 2
                sig_locs_list[idx].update(sig_locs_map[_idx % 3])
            else:
                _idx = (idx // 2) - 1
                sig_locs_list[idx].update(sig_locs_map[_idx % 3])

        # -- 1.3 Find coupled loop, and compute coupled loop signals locs --
        loop0, loop1 = [unit_idx_list[0]], [unit_idx_list[1]]
        _new = 2
        while unit_idx_list[_new] not in loop0:
            loop0.append(unit_idx_list[_new])
            _new += 2
        _new = 3
        while unit_idx_list[_new] not in loop1:
            loop1.append(unit_idx_list[_new])
            _new += 2
        ng_tidx_list = ng_tidx_list[::-1]
        for idx, num in enumerate(loop0):
            out_idx = coupled_index[(idx + 1) % 4]
            sig_locs_list[num].update({'pout': -ng_tidx_list[out_idx],
                                       'nout': -ng_tidx_list[out_idx],
                                       'half_col_min': half_col_min})

        for idx, num in enumerate(loop1):
            out_idx = -coupled_index[(idx + 1) % 4] - 1
            sig_locs_list[num].update({'pout': -ng_tidx_list[out_idx],
                                       'nout': -ng_tidx_list[out_idx],
                                       'half_col_min': half_col_min})

        # === 2. Place instances ===
        stage_list = []
        # -- 2.1 Dummy at top --

        # -- 2.2 Main instances --
        for idx in range(0, num_stages):
            _sig_locs = sig_locs_list[idx]
            _flip_np = flip_np_list[idx]
            _params = dict(pinfo=pinfo, seg_dict=seg_dict, w_dict=w_dict, sig_locs=_sig_locs, out_buf=out_buf)
            _template = self.new_template(RingOscUnit, params=_params)
            stage_list.append(self.add_tile(_template, col_idx=0, tile_idx=idx + 1))
        dum_params = dict(pinfo=pinfo, seg_dict=seg_dict, w_dict=w_dict, is_dum=True, out_buf=out_buf,
                          sig_locs={'half_col_min': half_col_min})
        dum_template = self.new_template(RingOscUnit, params=dum_params)
        dum_b = self.add_tile(dum_template, col_idx=0, tile_idx=0)
        dum_t = self.add_tile(dum_template, col_idx=0, tile_idx=num_stages + 1)
        self.set_mos_size(dum_template.num_cols)
        inc_n_list = [inst.get_pin('couplern') for inst in stage_list]
        inc_p_list = [inst.get_pin('couplerp') for inst in stage_list]
        in_n_list = [inst.get_pin('inn') for inst in stage_list]
        in_p_list = [inst.get_pin('inp') for inst in stage_list]
        out_n_list = [inst.get_pin('outn') for inst in stage_list]
        out_p_list = [inst.get_pin('outp') for inst in stage_list]

        # -- 2.3 Dummy at bottom --
        # === 3. Connections ===

        # -- 3.1 Main connections in RO --
        for idx in range(num_stages):
            idx_prev = unit_idx_list[idx]
            idx_next = unit_idx_list[idx + 1]
            idx_next_coupled = unit_idx_list[idx + 2]
            if idx == num_stages - 1:
                out_p_list[idx_prev], out_n_list[idx_prev] = \
                    self.connect_differential_wires(in_p_list[idx_next], in_n_list[idx_next],
                                                    out_p_list[idx_prev], out_n_list[idx_prev])
            else:
                out_n_list[idx_prev], out_p_list[idx_prev] = \
                    self.connect_differential_wires(in_p_list[idx_next], in_n_list[idx_next],
                                                    out_n_list[idx_prev], out_p_list[idx_prev])
            if idx >= num_stages - 2:
                out_p_list[idx_prev], out_n_list[idx_prev] = \
                    self.connect_differential_wires(inc_p_list[idx_next_coupled], inc_n_list[idx_next_coupled],
                                                    out_p_list[idx_prev], out_n_list[idx_prev])
            else:
                out_p_list[idx_prev], out_n_list[idx_prev] = \
                    self.connect_differential_wires(inc_n_list[idx_next_coupled], inc_p_list[idx_next_coupled],
                                                    out_p_list[idx_prev], out_n_list[idx_prev])

        out_p_list_track_set = set()
        [out_p_list_track_set.add(o.track_id.base_index) for o in out_p_list]
        out_p_list_track_list = list(out_p_list_track_set)
        out_p_list_track_list.sort()
        extend_vm_unit = stage_list[0].bound_box.h
        for idx in range(len(out_p_list)):
            if out_p_list[idx].track_id.base_index == out_p_list_track_list[1]:
                out_p_list[idx] = self.extend_wires(out_p_list[idx],
                                                    upper=out_p_list[idx].upper + extend_vm_unit,
                                                    lower=out_p_list[idx].lower - extend_vm_unit)[0]
            if out_p_list[idx].track_id.base_index == out_p_list_track_list[-2]:
                mid_y = self.bound_box.h // 2
                if out_p_list[idx].lower > mid_y:
                    out_p_list[idx] = self.extend_wires(out_p_list[idx], upper=out_p_list[idx].upper + extend_vm_unit)[
                        0]
                else:
                    out_p_list[idx] = self.extend_wires(out_p_list[idx], lower=out_p_list[idx].lower - extend_vm_unit)[
                        0]

        out_n_list_track_set = set()
        [out_n_list_track_set.add(o.track_id.base_index) for o in out_n_list]
        out_n_list_track_list = list(out_n_list_track_set)
        out_n_list_track_list.sort(reverse=True)
        extend_vm_unit = stage_list[0].bound_box.h
        for idx in range(len(out_n_list)):
            if out_n_list[idx].track_id.base_index == out_n_list_track_list[1]:
                out_n_list[idx] = self.extend_wires(out_n_list[idx],
                                                    upper=out_n_list[idx].upper + extend_vm_unit,
                                                    lower=out_n_list[idx].lower - extend_vm_unit)[0]
            if out_n_list[idx].track_id.base_index == out_n_list_track_list[-2]:
                mid_y = self.bound_box.h // 2
                if out_n_list[idx].lower > mid_y:
                    out_n_list[idx] = self.extend_wires(out_n_list[idx], upper=out_n_list[idx].upper + extend_vm_unit)[
                        0]
                else:
                    out_n_list[idx] = self.extend_wires(out_n_list[idx], lower=out_n_list[idx].lower - extend_vm_unit)[
                        0]

        # === 5. Pins ===
        _idx = 0
        pin_added_list, pin_n_added_list, buf_pin_added_list, buf_pinn_added_list = [], [], [], []
        while out_p_list[unit_idx_list[_idx]] not in pin_added_list:
            pin_added_list.append(out_p_list[unit_idx_list[_idx]])
            pin_n_added_list.append(out_n_list[unit_idx_list[_idx]])
            _idx += 1
        if out_buf:
            buf_out_n_list = [inst.get_pin('buf_outn') for inst in stage_list]
            buf_out_p_list = [inst.get_pin('buf_outp') for inst in stage_list]
            while out_p_list[unit_idx_list[_idx]] not in pin_added_list:
                buf_pin_added_list.append(buf_out_p_list[unit_idx_list[_idx]])
                buf_pinn_added_list.append(buf_out_n_list[unit_idx_list[_idx]])
                _idx += 1
            for idx, (pinp, pinn) in enumerate(zip(buf_pin_added_list, buf_pinn_added_list)):
                self.add_pin(f"phi_buf<{idx}>", pinp, show=self.show_pins)
                self.add_pin(f"phi_buf<{idx + num_stages}>", pinn, show=self.show_pins)

        for idx, (pinp, pinn) in enumerate(zip(pin_added_list, pin_n_added_list)):
            self.add_pin(f"phi<{idx}>", pinp, show=self.show_pins)
            self.add_pin(f"phi<{idx + num_stages}>", pinn, show=self.show_pins)

        vss_core_ym = self.connect_wires([warr for c in stage_list for warr in c.get_all_port_pins('VSS_core_ym')])
        vdd_core_ym = self.connect_wires([warr for c in stage_list for warr in c.get_all_port_pins('VDD_core_ym')])
        full_list = stage_list + [dum_t, dum_b]
        vdd_xm_list = [warr for c in full_list for warr in c.get_all_port_pins('VDD_xm')]
        vss_xm_list = [warr for c in full_list for warr in c.get_all_port_pins('VSS_xm')]

        stage_list += [dum_t, dum_b]
        vdd_vm_list, vss_vm_list = [], []
        for inst in stage_list:
            vdd_vm_list.extend(inst.get_all_port_pins('VDD', layer=vm_layer))
            vss_vm_list.extend(inst.get_all_port_pins('VSS', layer=vm_layer))
        self.add_pin('VDD', vdd_vm_list, connect=True)
        self.add_pin('VSS', vss_vm_list, connect=True)

        vss_ym = self.connect_wires([warr for c in stage_list for warr in c.get_all_port_pins('VSS_ym')])
        vdd_ym = self.connect_wires([warr for c in stage_list for warr in c.get_all_port_pins('VDD_ym')])

        self.extend_wires(vss_core_ym + vdd_core_ym + vss_ym + vdd_ym, lower=self.bound_box.yl, upper=self.bound_box.yh)

        # self.connect_wires(core_vdd_xm_list + vdd_xm_list)
        self.add_pin('VSS_xm', vss_xm_list, hide=True)
        self.add_pin('VDD_xm', vdd_xm_list, hide=True)
        # self.add_pin('VSS_core_xm', core_vss_xm_list, hide=True)
        self.add_pin('VSS_core_ym', vss_core_ym, hide=True)
        self.add_pin('VDD_core_ym', vdd_core_ym, hide=True)
        self.add_pin('vss_ym', vss_ym, hide=True)
        self.add_pin('vdd_ym', vdd_ym, hide=True)

        core_vss_xm1_list = [warr for c in stage_list for warr in c.get_all_port_pins('VCO_xm1')]
        core_vdd_xm1_list = [warr for c in stage_list for warr in c.get_all_port_pins('VDD_core_xm1')]
        vss_xm1_list = [warr for c in stage_list for warr in c.get_all_port_pins('VSS_xm1')]
        vdd_xm1_list = [warr for c in stage_list for warr in c.get_all_port_pins('VDD_xm1')]
        self.add_pin('VDD_core_xm1', core_vdd_xm1_list, label='VTOP')
        self.add_pin('VDD_xm1', vdd_xm1_list, label='VDD')
        self.add_pin('VCO_xm1', core_vss_xm1_list, label='VBOT')
        self.add_pin('VSS_xm1', vss_xm1_list, label='VSS')
        params = dict(pinfo=pinfo, seg_dict=seg_dict, w_dict=w_dict, sig_locs={'half_col_min': half_col_min},
                      out_buf=out_buf)
        template = self.new_template(RingOscUnit, params=params)
        self._center_col = template.center_col
        self._sch_params = template.sch_params
        self._row_idx = unit_idx_list[:-1]

        self._sch_params.update(
            dict(
                num_stage=num_stages,
                ndum=2,
                delta=delta,
                dum_info=self._dum_info,
                vbot_core='VBOT',
                vtop_core='VTOP',
                out_buf=out_buf,
            )
        )


class CtrlUnit(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg='Number of segments.',
            stack='Number of stack',
            w='Width',
            half='True to only draw half active devices',
            row_idx='Odd or Even row, used match ring osc'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            stack=1,
            half=False,
            row_idx=0,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg: int = self.params['seg']
        stack: int = self.params['stack']
        w: int = self.params['w']

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        # Calculate segments
        tap_ncol = self.min_sub_col
        tap_sep_col = self.sub_sep_col

        cur_loc = 0
        # self.add_tap(cur_loc + tap_ncol, vdd_list, vss_list, flip_lr=True)
        sup_list = [self.add_substrate_contact(0, cur_loc, seg=tap_ncol),
                    self.add_substrate_contact(1, cur_loc, seg=tap_ncol)]
        cur_loc += tap_ncol + tap_sep_col
        mosb = self.add_mos(0, cur_loc, seg, w=w, stack=stack)
        most = self.add_mos(1, cur_loc, seg, w=w, stack=stack)
        cur_loc += seg * stack
        tot_cols = cur_loc + tap_sep_col
        tot_cols += tot_cols & 1
        self.set_mos_size(tot_cols)
        sup_list.extend([self.add_substrate_contact(0, self.num_cols, seg=tap_ncol, flip_lr=True),
                         self.add_substrate_contact(1, self.num_cols, seg=tap_ncol, flip_lr=True)])

        sup_list = [self.connect_to_tracks(sup_list, self.get_track_id(0, MOSWireType.DS, 'sup', 0)),
                    self.connect_to_tracks(sup_list, self.get_track_id(1, MOSWireType.DS, 'sup', 0))]
        ctrl_hm = [self.connect_to_tracks([mosb.g, ], self.get_track_id(0, MOSWireType.G, wire_name='ctrl')),
                   self.connect_to_tracks([mosb.g, most.g], self.get_track_id(1, MOSWireType.G, wire_name='ctrl'))]
        vco_hm = connect_hm_sup_ow(self, mosb.s, 0, 0, tr_manager, 'vco')
        vco_hm_t = connect_hm_sup_ow(self, most.s, 0, 1, tr_manager, 'vco')
        # vco_hm = self.connect_to_tracks(mosb.s, self.get_track_id(0, MOSWireType.DS, 'vco'))
        # vco_hm_t = self.connect_to_tracks(most.s, self.get_track_id(1, MOSWireType.DS, 'vco'))
        sup_hm = self.connect_to_tracks([mosb.d], self.get_track_id(0, MOSWireType.DS, 'sup'))
        sup_hm_t = self.connect_to_tracks(most.d, self.get_track_id(1, MOSWireType.DS, 'sup'))

        sup_list_hm = self.connect_wires(sup_list + [sup_hm, sup_hm_t], lower=self.bound_box.xl,
                                         upper=self.bound_box.xh)
        vco_hm = self.extend_wires([vco_hm, vco_hm_t], lower=self.bound_box.xl, upper=self.bound_box.xh)

        # vm_layer
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        tr_w_vco_vm = tr_manager.get_width(vm_layer, 'vco')
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        ctrl_vm_tid = self.grid.coord_to_track(vm_layer, ctrl_hm[0].middle, mode=RoundMode.NEAREST)
        ctrl_vm = self.connect_to_tracks(ctrl_hm, TrackID(vm_layer, ctrl_vm_tid, tr_w_ctrl_vm),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
        _, vm_locs = tr_manager.place_wires(vm_layer, ['vco', 'sup', 'dum', 'ctrl', 'dum', 'sup', 'vco'],
                                            align_track=ctrl_vm_tid, align_idx=3)

        vco_vm_tid = [vm_locs[0], vm_locs[-1]]
        sup_vm_tid = [vm_locs[1], vm_locs[-2]]
        vco_vm = [self.connect_to_tracks(vco_hm, TrackID(vm_layer, vco_vm_tid[0], tr_w_vco_vm)),
                  self.connect_to_tracks(vco_hm, TrackID(vm_layer, vco_vm_tid[1], tr_w_vco_vm))]
        sup_vm = [self.connect_to_tracks(sup_list_hm, TrackID(vm_layer, sup_vm_tid[0], tr_w_sup_vm)),
                  self.connect_to_tracks(sup_list_hm, TrackID(vm_layer, sup_vm_tid[1], tr_w_sup_vm))]
        xm_tidx_list = [self.grid.coord_to_track(xm_layer, 0, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h // 4, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h // 2, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h * 3 // 4, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h, mode=RoundMode.NEAREST)]

        tr_w_ctrl_xm = tr_manager.get_width(xm_layer, 'ctrl')
        tr_w_vco_xm = tr_manager.get_width(xm_layer, 'vco')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        tr_w_vco_ym = tr_manager.get_width(ym_layer, 'vco')
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        ctrl_xm = self.connect_to_tracks(ctrl_vm, TrackID(xm_layer, xm_tidx_list[2], tr_w_ctrl_xm, grid=self.grid))
        vco_xm = [self.connect_to_tracks(vco_vm, TrackID(xm_layer, xm_tidx_list[0], tr_w_vco_xm, grid=self.grid)),
                  self.connect_to_tracks(vco_vm, TrackID(xm_layer, xm_tidx_list[-1], tr_w_vco_xm, grid=self.grid))]
        sup_xm = [self.connect_to_tracks(sup_vm, TrackID(xm_layer, xm_tidx_list[1], tr_w_sup_xm, grid=self.grid)),
                  self.connect_to_tracks(sup_vm, TrackID(xm_layer, xm_tidx_list[-2], tr_w_sup_xm, grid=self.grid))]
        ym_mid_tidx = self.grid.coord_to_track(ym_layer, ctrl_hm[0].middle, RoundMode.NEAREST)
        tr_sp_sup_ctrl = self.get_track_sep(ym_layer, tr_w_sup_ym, 1) + self.get_track_sep(ym_layer, tr_w_ctrl_ym, 1)
        tr_sp_vco_sup = self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_vco_ym)
        ym_locs = [ym_mid_tidx - tr_sp_sup_ctrl - tr_sp_vco_sup, ym_mid_tidx - tr_sp_sup_ctrl, ym_mid_tidx,
                   ym_mid_tidx, ym_mid_tidx + tr_sp_sup_ctrl, ym_mid_tidx + tr_sp_sup_ctrl + tr_sp_vco_sup]
        # _, ym_locs = tr_manager.place_wires(ym_layer, ['vco', 'sup', 'dum', 'ctrl', 'dum', 'sup', 'vco'],
        #                                     align_track=ym_mid_tidx, align_idx=3)

        ret_xm_list = []
        ctrl_ym = self.connect_to_tracks(ctrl_xm, TrackID(ym_layer, ym_locs[3], tr_w_ctrl_ym, grid=self.grid),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh,
                                         ret_wire_list=ret_xm_list)
        vco_ym = [self.connect_to_tracks(vco_xm, TrackID(ym_layer, ym_locs[-1], tr_w_vco_ym, grid=self.grid),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh,
                                         ret_wire_list=ret_xm_list),
                  self.connect_to_tracks(vco_xm, TrackID(ym_layer, ym_locs[0], tr_w_vco_ym, grid=self.grid),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh,
                                         ret_wire_list=ret_xm_list)]
        sup_ym = [self.connect_to_tracks(sup_xm, TrackID(ym_layer, ym_locs[1], tr_w_sup_ym, grid=self.grid),
                                         ret_wire_list=ret_xm_list),
                  self.connect_to_tracks(sup_xm, TrackID(ym_layer, ym_locs[-2], tr_w_sup_ym, grid=self.grid),
                                         ret_wire_list=ret_xm_list)]
        xm1_tidx_bot = self.grid.coord_to_track(xm1_layer, 0, mode=RoundMode.NEAREST)
        xm1_tidx_top = self.grid.coord_to_track(xm1_layer, self.bound_box.yh, mode=RoundMode.NEAREST)
        tr_w_vco_xm1 = tr_manager.get_width(xm1_layer, 'vco')
        # draw_dummy_route(self, ret_xm_list, self.bound_box.xl, self.bound_box.xh, xm_layer,
        #                  min_gap=2 * self.grid.get_line_end_space(xm_layer, ret_xm_list[0].track_id.width))
        vco_xm1 = self.connect_to_tracks(vco_ym, TrackID(xm1_layer, xm1_tidx_bot, tr_w_vco_xm1, grid=self.grid),
                                         track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
        sup_xm1 = self.connect_to_tracks(sup_ym, TrackID(xm1_layer, xm1_tidx_top, tr_w_vco_xm1, grid=self.grid),
                                         track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
        self.add_pin('vco_xm1', vco_xm1)
        self.add_pin('sup_xm1', sup_xm1)

        self.add_pin('vco_xm', vco_xm)
        self.add_pin('ctrl_ym', ctrl_ym)
        self.add_pin('vco_ym', vco_ym)
        self.add_pin('sup_ym', sup_ym)


class CtrlDiffUnit(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_ro_ctrl')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg='Number of segments.',
            stack='Number of stack',
            w='Width',
            half='True to only draw half active devices',
            row_idx='Odd or Even row, used match ring osc'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            stack=1,
            half=False,
            row_idx=0,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg: dict = self.params['seg']
        seg_ctrl = seg['ctrl']
        seg_tail = seg['tail']
        stack: dict = self.params['stack']
        stack_ctrl = stack['ctrl']
        stack_tail = stack['tail']
        w: dict = self.params['w']
        w_ctrl = w['ctrl']
        w_tail = w['tail']

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        # Calculate segments
        tap_ncol = self.min_sub_col
        tap_sep_col = self.sub_sep_col

        cur_loc = 0
        tx_sep = 2 * self.min_sep_col
        sup_list = [self.add_substrate_contact(0, cur_loc, seg=tap_ncol),
                    self.add_substrate_contact(1, cur_loc, seg=tap_ncol)]
        cur_loc += tap_ncol + tap_sep_col
        tailb = self.add_mos(0, cur_loc, seg_tail, w=w_tail, stack=stack_tail)
        tailt = self.add_mos(1, cur_loc, seg_tail, w=w_tail, stack=stack_tail)

        cur_loc += seg_tail * stack_tail + tx_sep
        mosb = self.add_mos(0, cur_loc, seg_ctrl, w=w_ctrl, stack=stack_ctrl)
        most = self.add_mos(1, cur_loc, seg_ctrl, w=w_ctrl, stack=stack_ctrl)

        tail_vm_col = cur_loc - tx_sep // 2

        cur_loc += seg_ctrl * stack_ctrl
        tot_cols = cur_loc + tap_sep_col
        tot_cols += tot_cols & 1
        self.set_mos_size(tot_cols)
        ctrl_hm = [self.connect_to_tracks([mosb.g, most.g], self.get_track_id(0, MOSWireType.G, wire_name='ctrl')),
                   self.connect_to_tracks([mosb.g, most.g], self.get_track_id(1, MOSWireType.G, wire_name='ctrl'))]
        vdd_hm = [self.connect_to_tracks([tailb.g, tailt.g], self.get_track_id(0, MOSWireType.G, wire_name='dum')),
                  self.connect_to_tracks([tailb.g, tailt.g], self.get_track_id(1, MOSWireType.G, wire_name='dum'))]
        vco_hm = connect_hm_sup_ow(self, mosb.s, 0, 0, tr_manager, 'vco')
        vco_hm_t = connect_hm_sup_ow(self, most.s, 0, 1, tr_manager, 'vco')
        sup_hm = connect_hm_sup_ow(self, sup_list + [tailb.s], 0, 0, tr_manager, 'vco')
        sup_hm_t = connect_hm_sup_ow(self, sup_list + [tailt.s], 0, 1, tr_manager, 'vco')

        self.extend_wires([vco_hm, vco_hm_t], upper=self.bound_box.xh)

        tail_hm = self.connect_to_tracks([mosb.d, tailb.d], self.get_track_id(0, MOSWireType.DS, 'sup', 0))
        tail_hm_t = self.connect_to_tracks([most.d, tailt.d], self.get_track_id(1, MOSWireType.DS, 'sup', 0))

        # vm_layer
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        tr_w_vco_vm = tr_manager.get_width(vm_layer, 'vco')
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        ctrl_vm_tid = self.grid.coord_to_track(vm_layer, ctrl_hm[0].middle, mode=RoundMode.NEAREST)
        ctrl_vm = self.connect_to_tracks(ctrl_hm, TrackID(vm_layer, ctrl_vm_tid, tr_w_ctrl_vm))
        vco_vm = self.connect_to_tracks(vco_hm, TrackID(vm_layer, ctrl_vm_tid, tr_w_vco_vm))
        vco_vm_t = self.connect_to_tracks(vco_hm_t, TrackID(vm_layer, ctrl_vm_tid, tr_w_vco_vm))

        sup_vm_tid = self.grid.coord_to_track(vm_layer, sup_hm[0].middle, mode=RoundMode.NEAREST)
        sup_vm = self.connect_to_tracks(sup_hm, TrackID(vm_layer, sup_vm_tid, tr_w_sup_vm))
        sup_vm_t = self.connect_to_tracks(sup_hm_t, TrackID(vm_layer, sup_vm_tid, tr_w_sup_vm))
        vdd_vm = self.connect_to_tracks(vdd_hm, TrackID(vm_layer, sup_vm_tid, tr_w_sup_vm))

        tail_vm_tidx = self.arr_info.col_to_track(vm_layer, tail_vm_col, RoundMode.NEAREST)
        tail_vm = self.connect_to_tracks([tail_hm_t, tail_hm], TrackID(vm_layer, tail_vm_tidx, tr_w_sup_vm),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)

        # xm_layer
        xm_tidx_list = [self.grid.coord_to_track(xm_layer, 0, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h // 2, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h, mode=RoundMode.NEAREST)]

        tr_w_ctrl_xm = tr_manager.get_width(xm_layer, 'ctrl')
        tr_w_vco_xm = tr_manager.get_width(xm_layer, 'vco')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        tr_w_vco_ym = tr_manager.get_width(ym_layer, 'vco')
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        ctrl_xm = self.connect_to_tracks(ctrl_vm, TrackID(xm_layer, xm_tidx_list[1], tr_w_ctrl_xm, grid=self.grid))
        vco_xm = [self.connect_to_tracks(vco_vm, TrackID(xm_layer, xm_tidx_list[0], tr_w_vco_xm, grid=self.grid)),
                  self.connect_to_tracks(vco_vm_t, TrackID(xm_layer, xm_tidx_list[-1], tr_w_vco_xm, grid=self.grid))]
        sup_xm = [self.connect_to_tracks(sup_vm, TrackID(xm_layer, xm_tidx_list[0], tr_w_sup_xm, grid=self.grid)),
                  self.connect_to_tracks(sup_vm_t, TrackID(xm_layer, xm_tidx_list[-1], tr_w_sup_xm, grid=self.grid))]
        vdd_xm = self.connect_to_tracks(vdd_vm, TrackID(xm_layer, xm_tidx_list[1], tr_w_ctrl_xm, grid=self.grid))

        tr_w_tail_xm = tr_manager.get_width(xm_layer, 'tail')
        tail_xm_tidx = [self.grid.coord_to_track(xm_layer, self.bound_box.h // 4, mode=RoundMode.NEAREST),
                        self.grid.coord_to_track(xm_layer, self.bound_box.h * 3 // 4, mode=RoundMode.NEAREST)]
        tail_xm = [self.connect_to_tracks(tail_vm, TrackID(xm_layer, tail_xm_tidx[0], tr_w_tail_xm, grid=self.grid)),
                   self.connect_to_tracks(tail_vm, TrackID(xm_layer, tail_xm_tidx[1], tr_w_tail_xm, grid=self.grid))]
        tail_xm = self.extend_wires(tail_xm, lower=self.bound_box.xl)

        # ym_layer
        ym_mid_tidx = self.grid.coord_to_track(ym_layer, ctrl_hm[0].middle, RoundMode.NEAREST)
        tr_sp_sup_ctrl = self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_ctrl_ym)
        tr_sp_vco_sup = self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_vco_ym).div2()
        ym_locs = [
            ym_mid_tidx - tr_sp_sup_ctrl - tr_sp_vco_sup - self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_sup_ym),
            ym_mid_tidx - tr_sp_sup_ctrl - tr_sp_vco_sup,
            ym_mid_tidx - tr_sp_vco_sup, ym_mid_tidx + tr_sp_vco_sup]
        ret_xm_list = []
        ctrl_ym = self.connect_to_tracks(ctrl_xm, TrackID(ym_layer, ym_locs[2], tr_w_ctrl_ym, grid=self.grid))
        vco_ym = [self.connect_to_tracks(vco_xm, TrackID(ym_layer, ym_locs[3], tr_w_vco_ym, grid=self.grid),
                                         track_lower=self.bound_box.yl, track_upper=self.bound_box.yh,
                                         ret_wire_list=ret_xm_list)]
        sup_ym = [self.connect_to_tracks(sup_xm, TrackID(ym_layer, ym_locs[1], tr_w_sup_ym, grid=self.grid),
                                         ret_wire_list=ret_xm_list)]
        vdd_ym = self.connect_to_tracks(vdd_xm, TrackID(ym_layer, ym_locs[0], tr_w_sup_ym, grid=self.grid),
                                        ret_wire_list=ret_xm_list)
        xm1_tidx_bot = self.grid.coord_to_track(xm1_layer, 0, mode=RoundMode.NEAREST)
        xm1_tidx_top = self.grid.coord_to_track(xm1_layer, self.bound_box.yh, mode=RoundMode.NEAREST)
        tr_w_vco_xm1 = tr_manager.get_width(xm1_layer, 'vco')
        ret_ym_list = []
        vco_xm1 = self.connect_to_tracks(vco_ym, TrackID(xm1_layer, xm1_tidx_bot, tr_w_vco_xm1, grid=self.grid),
                                         track_upper=self.bound_box.xh, ret_wire_list=ret_ym_list)
        sup_xm1 = self.connect_to_tracks(sup_ym, TrackID(xm1_layer, xm1_tidx_top, tr_w_vco_xm1, grid=self.grid),
                                         track_lower=self.bound_box.xl, track_upper=self.bound_box.xh,
                                         ret_wire_list=ret_ym_list)
        vdd_xm1 = self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx_bot, tr_w_vco_xm1, grid=self.grid),
                                         track_lower=self.bound_box.xl, ret_wire_list=ret_ym_list)
        sup_xm1 = self.extend_wires(sup_xm1, lower=vdd_xm1.lower)[0]

        tr_w_ctrl_xm1 = tr_manager.get_width(xm1_layer, 'ctrl')
        ctrl_xm1_tidx = self.grid.coord_to_track(xm1_layer, ctrl_ym.middle, RoundMode.NEAREST)
        ctrl_xm1 = self.connect_to_tracks(ctrl_ym, TrackID(xm1_layer, ctrl_xm1_tidx, tr_w_ctrl_xm1,
                                                           grid=self.grid))
        self.match_warr_length(ret_ym_list)

        draw_dummy_route(self, [sup_xm1, vco_xm1, vdd_xm1], self.bound_box.xl, self.bound_box.xh, xm1_layer,
                         min_gap=2 * self.grid.get_line_end_space(xm1_layer, ctrl_xm1.track_id.width))

        self.add_pin('vco_xm1', vco_xm1, label='d')
        self.add_pin('sup_xm1', sup_xm1, label='VSS')
        self.add_pin('vdd_xm1', vdd_xm1, label='VDD')
        self.add_pin('ctrl_xm1', ctrl_xm1, label='ctrl')
        self.add_pin('tail_hm', [tail_hm, tail_hm_t], label='tail')
        self.add_pin('tail_xm', tail_xm, label='tail')

        self._sch_params = dict(
            lch=self.arr_info.lch,
            intent=self.get_tile_pinfo(0).get_row_place_info(0).row_info.threshold,
            w=w,
            seg={'ctrl': seg_ctrl * 2, 'tail': 2 * seg_tail},
            stack=stack,
        )


class CtrlCol(CtrlUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = CtrlUnit.get_params_info()
        ans['min_height'] = 'minimum unit height'
        ans['num_stages'] = 'number of osc units'
        ans['dev_type'] = 'dev_type'
        ans.pop('half')
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = CtrlUnit.get_default_param_values()
        ans.pop('half')
        return ans

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_ro_ctrl')

    def draw_layout(self) -> None:
        min_height: int = self.params['min_height']
        dev_type: str = self.params['dev_type']
        num_stages: int = self.params['num_stages']
        w: dict = self.params['w']
        seg: dict = self.params['seg']
        stack: dict = self.params['stack']

        pinfo_dict = self.params['pinfo'].to_yaml()
        pinfo_dict['tile_specs']['place_info']['ctrl_tile']['min_height'] = min_height
        pinfo_dict['tile_specs']['place_info']['ctrl_tile']['row_specs'][0]['mos_type'] = dev_type
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        self.draw_base(pinfo)

        # === 2. Place instances ===

        ctrl_unit_list = []
        template_list = []
        # -- 2.2 Main instances --
        for idx in range(0, num_stages):  # +4 for top and bottom osc dummy
            _params = dict(pinfo=pinfo, seg=seg, w=w, stack=stack, row_idx=(1 + idx) // 2)
            _template = self.new_template(CtrlDiffUnit, params=_params)
            template_list.append(_template)
            ctrl_unit_list.append(self.add_tile(_template, col_idx=0, tile_idx=idx + 1))
        self.set_mos_size(template_list[0].num_cols, num_tiles=num_stages + 2)

        # ==== Connection
        sup_ym = self.connect_wires([warr for c in ctrl_unit_list for warr in c.get_all_port_pins('sup_ym')])
        _ctrl_ym = self.connect_wires([warr for c in ctrl_unit_list for warr in c.get_all_port_pins('ctrl_ym')])

        # === Collect xm1 layer vco and sup signals
        sup_xm1_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('sup_xm1')]
        vco_xm1_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('vco_xm1')]
        vdd_xm1_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('vdd_xm1')]
        vco_xm_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('vco_xm')]
        tail_hm_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('tail_hm')]
        tail_xm_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('tail_xm')]
        ctrl_xm1_list = [warr for c in ctrl_unit_list for warr in c.get_all_port_pins('ctrl_xm1')]

        self.add_pin('sup_xm1', sup_xm1_list, label='VSS', connect=True)
        self.add_pin('sup_ym', sup_ym, label='VSS', connect=True)
        self.add_pin('vco_xm1', vco_xm1_list, label='d', connect=True)
        self.add_pin('vco_xm', vco_xm_list, label='d', connect=True)
        self.add_pin('ctrl_xm1', ctrl_xm1_list, label='ctrl', connect=True)
        self.add_pin('vdd_xm1', vdd_xm1_list, label='VDD', connect=True)
        self.add_pin('tail_hm', tail_hm_list, label='tail', connect=True)
        self.add_pin('tail_xm', tail_xm_list, label='tail', connect=True)

        self._sch_params = dict(
            lch=self.arr_info.lch,
            intent=self.get_tile_pinfo(1).get_row_place_info(0).row_info.threshold,
            seg={'ctrl': num_stages * seg['ctrl'] * 2, 'tail': num_stages * seg['tail'] * 2},
            w=self.params['w'],
            stack=stack,
        )


class VCOCore(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._col_tot = 0
        self._dum_info = []
        self._center_col = 0
        self._row_idx = []
        self._tr_manager = None
        self._core = None

    @property
    def core(self):
        return self._core

    @property
    def tr_manager(self):
        return self._tr_manager

    @property
    def row_idx(self) -> List[int]:
        return self._row_idx

    @property
    def center_col(self) -> int:
        return self._center_col

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_vco')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            ro_params='',
            ctrl_params='',
            tr_widths='track width dictionary',
            tr_spaces='track space dictionary',
            top_sup_layer='Top supply layer',
        )

    def draw_layout(self) -> None:
        ctrl_params = self.params['ctrl_params']
        if isinstance(ctrl_params, str):
            ctrl_params = read_yaml(ctrl_params)
            ctrl_params = ctrl_params['params']
            ctrl_params['export_private'] = False
            ctrl_params['export_hidden'] = True
        else:
            ctrl_params = dict(cls_name=CtrlCol.get_qualified_name(), export_private=False,
                               params=ctrl_params)

        ro_params = self.params['ro_params']
        if isinstance(ro_params, str):
            ro_params = read_yaml(ro_params)
            ro_params = ro_params['params']
            ro_params['export_private'] = False
            ro_params['export_hidden'] = True
        else:
            ro_params = dict(cls_name=RingOscCol.get_qualified_name(), export_private=False,
                             export_hidden=True, params=ro_params)
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        ring_master: GenericWrapper = self.new_template(GenericWrapper, params=ro_params)
        self._core = ring_master
        ctrl_master: GenericWrapper = self.new_template(GenericWrapper, params=ctrl_params)
        top_layer = max(ring_master.top_layer, ctrl_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        ctrl = self.add_instance(ctrl_master, xform=Transform(0, 0))
        x_ring = -(-ctrl_master.bound_box.w // w_blk) * w_blk

        ring = self.add_instance(ring_master, xform=Transform(x_ring, 0))
        w_tot = -(-ring.bound_box.xh // w_blk) * w_blk
        h_tot = -(-ring.bound_box.yh // h_blk) * h_blk
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot))

        self._tr_manager = tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      ro_params['params']['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # Connection
        # connect core vdd
        vdd_core_xm1 = ring.get_all_port_pins('VDD_core_xm1')
        vdd_xm1 = ring.get_all_port_pins('VDD_xm1')
        vss_ring_xm1 = ring.get_all_port_pins('VSS_xm1')
        vco_xm1 = ring.get_all_port_pins('VCO_xm1') + ctrl.get_all_port_pins('vco_xm1')
        vss_ctrl_xm1 = ctrl.get_all_port_pins('sup_xm1')
        vdd_ctrl_xm1 = ctrl.get_all_port_pins('vdd_xm1')
        vdd_xm1 = self.connect_wires(vdd_xm1 + vdd_core_xm1)
        vco_xm1 = self.connect_wires(vco_xm1)
        # vco_xm = self.connect_wires(ring.get_all_port_pins('VSS_core_xm') + ctrl.get_all_port_pins('vco_xm'))

        vss_ring_xm1.sort(key=lambda x: x.track_id.base_index)
        vss_ring_xm1.extend(self.extend_wires([vss_ring_xm1[0], vss_ring_xm1[-1]], lower=vdd_xm1[0].lower))

        # ym1 ctrl line
        ctrl_xm1 = ctrl.get_all_port_pins('ctrl_xm1')
        ctrl_ym1_tidx = self.grid.coord_to_track(ym1_layer, ctrl_xm1[0].middle, RoundMode.NEAREST)
        tr_w_ctrl_ym1 = tr_manager.get_width(ym1_layer, 'ctrl')
        ctrl_ym1 = self.connect_to_tracks(ctrl_xm1, TrackID(ym1_layer, ctrl_ym1_tidx, tr_w_ctrl_ym1, grid=self.grid))

        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        ym1_tid_l = self.grid.coord_to_track(ym1_layer, ctrl.bound_box.xl, mode=RoundMode.NEAREST)
        ym1_tid_r = self.grid.coord_to_track(ym1_layer, ctrl.bound_box.xh, mode=RoundMode.LESS)
        tr_sp_ym1 = tr_manager.get_sep(ym_layer, ('sup', 'sup')) if tr_w_sup_ym1 > 0 else \
            self.get_track_sep(ym1_layer, tr_w_sup_ym1, tr_w_sup_ym1)
        vss_ym1_locs = self.get_available_tracks(ym1_layer, ym1_tid_l, ym1_tid_r,
                                                 lower=self.bound_box.yl, upper=self.bound_box.yh, width=tr_w_sup_ym1,
                                                 sep=tr_sp_ym1, sep_margin=None, include_last=False)
        ym1_tid_l = self.grid.coord_to_track(ym1_layer, ring.bound_box.xl, mode=RoundMode.NEAREST)
        ym1_tid_r = self.grid.coord_to_track(ym1_layer, ring.bound_box.xh, mode=RoundMode.LESS)
        vdd_ym1_locs = self.get_available_tracks(ym1_layer, ym1_tid_l, ym1_tid_r,
                                                 lower=self.bound_box.yl, upper=self.bound_box.yh, width=tr_w_sup_ym1,
                                                 sep=tr_sp_ym1, sep_margin=None, include_last=False)

        vss_ym1 = [self.connect_to_tracks(vss_ctrl_xm1, TrackID(ym1_layer, tid, tr_w_sup_ym1, grid=self.grid))
                   for tid in vss_ym1_locs[1:]]
        vdd_ym1 = [self.connect_to_tracks(vdd_xm1, TrackID(ym1_layer, tid, tr_w_sup_ym1, grid=self.grid))
                   for tid in vdd_ym1_locs[1:-1]]
        vss_ym1.append(self.connect_to_tracks(vss_ring_xm1, TrackID(ym1_layer, vdd_ym1_locs[-1], tr_w_sup_ym1,
                                                                    grid=self.grid)))
        sup_xm1_l_ret = []
        vdd_ym1.append(self.connect_to_tracks(vdd_ctrl_xm1, TrackID(ym1_layer, vss_ym1_locs[0], tr_w_sup_ym1,
                                                                    grid=self.grid), ret_wire_list=sup_xm1_l_ret))
        self.extend_wires([vss_ring_xm1[0], vss_ring_xm1[-1]] + vss_ctrl_xm1, lower=sup_xm1_l_ret[0].lower)

        vdd_ym1 = self.extend_wires(vdd_ym1, lower=self.bound_box.yl, upper=self.bound_box.yh)
        vss_ym1 = self.extend_wires(vss_ym1, lower=self.bound_box.yl, upper=self.bound_box.yh)

        self.reexport(ring.get_port('VDD_xm'))
        self.reexport(ring.get_port('VSS_xm'))
        self.add_pin('VSS_l', vss_ctrl_xm1, label='VSS')
        self.add_pin('VSS_r', vss_ring_xm1, label='VSS')
        self.add_pin('VDD', vdd_xm1)
        self.add_pin('vbot', vco_xm1)

        for port in ring.port_names_iter():
            if 'phi' in port:
                self.reexport(ring.get_port(port))
        self.add_pin('vctrl', ctrl_ym1)
        # self.reexport(ctrl.get_port('tail_xm'), connect=True)
        # self.reexport(ctrl.get_port('tail_hm'))
        self.add_pin('tail', ctrl.get_all_port_pins('tail_xm') + ctrl.get_all_port_pins('tail_hm'),
                     connect=True)

        # Bring supply to top layer
        top_sup_layer = self.params['top_sup_layer']
        tile_height = ring_master.core.get_tile_info(0)[0].height
        start_coord = ring_master.core_bound_box.yl
        vss_hor_coord = [start_coord + 2 * idx * tile_height for idx in
                         range(ro_params['params']['num_stages'] // 2 + 2)]
        vdd_hor_coord = [start_coord + tile_height + 2 * idx * tile_height for idx in
                         range(ro_params['params']['num_stages'] // 2 + 1)]
        vss_vert_bnd = [vss_ctrl_xm1[0].lower, vss_ctrl_xm1[0].upper]
        vdd_vert_bnd = [vdd_xm1[0].lower, vdd_xm1[0].upper]

        last_vdd_list, last_vss_list = vdd_ym1, vss_ym1
        for idx in range(ym1_layer + 1, top_sup_layer + 1):
            tr_w_sup = tr_manager.get_width(idx, 'sup')
            lay_dir = self.grid.get_direction(idx)
            tr_w_sep = tr_manager.get_sep(idx, ('sup', 'sup')) if tr_w_sup > 0 else 0
            if lay_dir == Orient2D.x and idx < 10:
                vss_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vss_hor_coord]
                vdd_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vdd_hor_coord]
                last_vss_list = [self.connect_to_tracks(last_vss_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
                                 vss_tidx_list]
                last_vdd_list = [self.connect_to_tracks(last_vdd_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
                                 vdd_tidx_list]
                last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
                last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
            elif lay_dir == Orient2D.x:
                last_vdd_list, last_vss_list = self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list],
                                                                        idx - 1, self.bound_box)
            else:
                vss_tid_l = self.grid.coord_to_track(idx, vss_vert_bnd[0], mode=RoundMode.NEAREST)
                vss_tid_r = self.grid.coord_to_track(idx, vss_vert_bnd[1], mode=RoundMode.NEAREST)
                vss_locs = self.get_tids_between(idx, vss_tid_l, vss_tid_r, width=tr_w_sup, sep=tr_w_sep, sep_margin=0,
                                                 include_last=True)
                vdd_tid_l = self.grid.coord_to_track(idx, vdd_vert_bnd[0], mode=RoundMode.NEAREST)
                vdd_tid_r = self.grid.coord_to_track(idx, vdd_vert_bnd[1], mode=RoundMode.NEAREST)
                vdd_locs = self.get_tids_between(idx, vdd_tid_l, vdd_tid_r, width=tr_w_sup,
                                                 sep=tr_w_sep, sep_margin=0, include_last=False)
                last_vdd_list = [self.connect_to_tracks(last_vdd_list, tid) for tid in vdd_locs]
                last_vss_list = [self.connect_to_tracks(last_vss_list, tid) for tid in vss_locs]
                last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
                last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.yl, upper=self.bound_box.yh)

            self.add_pin(f'VDD{idx}', last_vdd_list, hide=idx < top_sup_layer, label='VDD')
            self.add_pin(f'VSS{idx}', last_vss_list, hide=idx < top_sup_layer, label='VSS')

        # self._ring_ncol = self.num_cols
        self._row_idx = ring_master.core.row_idx
        self._sch_params = dict(
            ro_params=ring_master.sch_params,
            ctrl_params=ctrl_master.sch_params,
            is_pctrl=False,
        )
