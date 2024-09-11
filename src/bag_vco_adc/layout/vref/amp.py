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

from typing import Any, Dict, Optional, Type, Tuple

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from bag_vco_adc.layout.util.wrapper import GenericWrapper
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orient2D
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase


class DiffAmp(MOSBase, TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'diffamp')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            w_dict='widths dictionary.',
            seg_dict='segments dictionary.',
            ridx_dict='bottom nmos row index.',
            fill_dummy='True to fill dummy',
            ndum='Number of dummy at sides',
            ngroups='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_dict={},
            ofst_dict={},
            fill_dummy=False,
            ngroups=1,
            ndum=0,
        )

    def draw_in_tail_group(self, col, seg_in, seg_load, ridx_dict):
        m_in = self.add_mos(ridx_dict['in'], seg=seg_in, col_idx=col, tile_idx=1)
        m_load = self.add_mos(ridx_dict['load'], seg=seg_load, col_idx=col, tile_idx=1)
        return m_in, m_load

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        w_dict: ImmutableSortedDict[str, int] = self.params['w_dict']
        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        ridx_dict: ImmutableSortedDict[str, int] = self.params['ridx_dict']
        ngroups: bool = self.params['ngroups']

        ndum: int = self.params['ndum']

        min_sep = self.min_sep_col
        seg_in, seg_tail, seg_load = seg_dict['in'], seg_dict['tail'], seg_dict['load']
        max_seg = ngroups * max(seg_in, seg_load, seg_tail)
        max_seg += max_seg & 1
        seg_tot = max_seg + 2 * ndum

        col_in = (seg_tot - 2 * seg_in) // 2
        col_load = (seg_tot - 2 * seg_load) // 2
        col_tail = (seg_tot - seg_tail * ngroups) // 2

        m_in_list, m_load_list = [], []
        cur_col = ndum
        for idx in range(ngroups):
            _m_in, _m_load = self.draw_in_tail_group(cur_col, seg_in, seg_load, ridx_dict)
            cur_col += max(seg_in, seg_load, seg_tail)
            m_in_list.append(_m_in)
            m_load_list.append(_m_load)

        m_in_list = m_in_list[:ngroups//2]+m_in_list[ngroups//2:][::-1]
        m_load_list = m_load_list[:ngroups//2]+m_load_list[ngroups//2:][::-1]
        m_tail = self.add_mos(ridx_dict['tail'], seg=seg_tail * ngroups, col_idx=col_tail, tile_idx=1,
                              g_on_s=True)

        self.set_mos_size(seg_tot, num_tiles=3)

        vss_tap = self.add_substrate_contact(0, 0, tile_idx=0, seg=self.num_cols, port_mode=SubPortMode.ODD)
        vdd_tap = self.add_substrate_contact(0, 0, tile_idx=2, seg=self.num_cols)
        vss_tidx = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0)
        vdd_tidx = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2)

        m_load_n_list, m_load_p_list = m_load_list[::2], m_load_list[1::2]
        m_in_n_list, m_in_p_list = m_in_list[::2], m_in_list[1::2]
        outp_conn = self.connect_wires([tx.d for tx in m_load_n_list] + [tx.d for tx in m_in_n_list])
        outn_conn = self.connect_wires([tx.d for tx in m_load_p_list] + [tx.d for tx in m_in_p_list])
        tail_conn = self.connect_wires([m_tail.s] + [tx.s for tx in m_in_list])
        vss_hm = self.connect_to_tracks([vss_tap] + [m_tail.d], vss_tidx)
        vdd_hm = self.connect_to_tracks([vdd_tap] + [tx.s for tx in m_load_list], vdd_tidx)
        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        bbox_m = (self.bound_box.xl + self.bound_box.xh) // 2
        vm_vss_l = self.export_tap_hm(tr_manager, vss_hm, hm_layer, vm_layer, bbox=[0, bbox_m])
        vm_vss_r = self.export_tap_hm(tr_manager, vss_hm, hm_layer, vm_layer,
                                      bbox=[bbox_m, self.bound_box.xh], align_upper=True)
        [fill_conn_layer_intv(self, 1, idx) for idx in range(self.get_tile_pinfo(1).num_rows)]
        #
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        vss_xm_tidx = self.grid.coord_to_track(xm_layer, vm_vss_l[0].middle)
        vss_xm = self.connect_to_tracks(vm_vss_l + vm_vss_r,
                                        TrackID(xm_layer, vss_xm_tidx, tr_w_sup_xm, grid=self.grid))
        self.connect_to_track_wires(vm_vss_l + vm_vss_r, vss_xm)
        vdd_xm_l = self.export_tap_hm(tr_manager, vdd_hm, hm_layer, xm_layer, bbox=[0, bbox_m])
        vdd_xm_r = self.export_tap_hm(tr_manager, vdd_hm, hm_layer, xm_layer, bbox=[bbox_m, self.bound_box.xh])
        vdd_xm = self.connect_wires(vdd_xm_l + vdd_xm_r)

        bn_hm_tid = self.get_track_id(ridx_dict['tail'], MOSWireType.G, 'bias', wire_idx=0, tile_idx=1)
        bn_hm = self.connect_to_tracks(m_tail.g, bn_hm_tid)
        inn_hm_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.G, 'sig', wire_idx=0, tile_idx=1)
        inp_hm_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.G, 'sig', wire_idx=1, tile_idx=1)
        tr_w_hm_sig = tr_manager.get_width(hm_layer, 'sig')
        inn_hm, inp_hm = self.connect_matching_tracks([[tx.g for tx in m_in_n_list],
                                                       [tx.g for tx in m_in_p_list]],
                                                      hm_layer, [inn_hm_tidx, inp_hm_tidx],
                                                      width=tr_w_hm_sig)
        bp_hm_tid = self.get_track_id(ridx_dict['load'], MOSWireType.G, 'bias', wire_idx=0, tile_idx=1)
        bp_hm = self.connect_to_tracks([tx.g for tx in m_load_list], bp_hm_tid)

        tail_hm_tid = self.get_track_id(ridx_dict['tail'], MOSWireType.DS, 'sig', tile_idx=1)
        _tail = self.connect_to_tracks(tail_conn, tail_hm_tid)

        outn_tidx = self.get_track_index(ridx_dict['load'], MOSWireType.DS, 'sig', tile_idx=1)
        outp_tidx = self.get_track_index(ridx_dict['in'], MOSWireType.DS, 'sig', tile_idx=1)
        outn_hm, outp_hm = self.connect_matching_tracks([outn_conn, outp_conn], hm_layer, [outn_tidx, outp_tidx],
                                                        width=tr_w_hm_sig)
        vm_col_list = list(range(ndum+1, self.num_cols-ndum, 2))
        vm_tidx_list = [self.arr_info.col_to_track(vm_layer, tidx, RoundMode.NEAREST) for tidx in vm_col_list]

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        _outn_vm = [self.connect_to_tracks([bp_hm, outn_hm], TrackID(vm_layer, tidx, tr_w_sig_vm)) for tidx in vm_tidx_list]
        outp_vm = [self.connect_to_tracks(outp_hm, TrackID(vm_layer, tidx, tr_w_sig_vm)) for tidx in vm_tidx_list]
        inn_vm = [self.connect_to_tracks(inn_hm, TrackID(vm_layer, tidx, tr_w_sig_vm)) for tidx in vm_tidx_list[::2]]
        inp_vm = [self.connect_to_tracks(inp_hm, TrackID(vm_layer, tidx, tr_w_sig_vm)) for tidx in vm_tidx_list[1::2]]
        inn_vm, inp_vm = self.match_warr_length([inn_vm, inp_vm])
        bn_vm = [self.connect_to_tracks(bn_hm, TrackID(vm_layer, tidx, tr_w_sig_vm)) for tidx in vm_tidx_list]
        out_xm_tidx = self.grid.coord_to_track(xm_layer, outp_vm[0].middle, RoundMode.NEAREST)
        inn_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm[0].upper, RoundMode.NEAREST)
        inp_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm[0].lower, RoundMode.NEAREST)
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        inn_xm, inp_xm = self.connect_matching_tracks([inn_vm, inp_vm], xm_layer, [inn_xm_tidx, inp_xm_tidx],
                                                      width=tr_w_sig_xm)
        out_xm = self.connect_to_tracks(outp_vm, TrackID(xm_layer, out_xm_tidx, tr_w_sig_xm))
        bn_xm_tidx = self.grid.coord_to_track(xm_layer, bn_vm[0].middle, RoundMode.NEAREST)
        bn_xm = self.connect_to_tracks(bn_vm, TrackID(xm_layer, bn_xm_tidx, tr_w_sig_xm))

        self.add_pin('out', out_xm)
        self.add_pin('vbn', bn_xm)
        self.add_pin('inn', inn_xm)
        self.add_pin('inp', inp_xm)
        self.add_pin('VSS', vss_xm)
        self.add_pin('VDD', vdd_xm)
    #
        w_dict, th_dict = self._get_w_th_dict(ridx_dict.to_dict(), w_dict.to_dict())
        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict={'in': ngroups*seg_in//2, 'tail': ngroups*seg_tail, 'load': ngroups*seg_load//2},
            w_dict=w_dict,
            th_dict=th_dict,
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


class AmpWrapper(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBaseZL.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None
        self._core_master = None

    @property
    def tr_manager(self):
        return self._tr_manager

    @property
    def core_master(self):
        return self._core_master

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return self._core_master.get_schematic_class()

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cls_name='Core class name',
            params='Core master params',
            top_layer='Top supply layer',
            top_out_layer='Top output layer',
            num_sup_around='Tuple of x,y direction',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(num_sup_around=(2, 2))

    def draw_layout(self) -> None:
        cls_name = self.params['cls_name']
        core_params = self.params['params']
        top_layer = self.params['top_layer']
        top_out_layer = self.params['top_out_layer']
        num_sup_around = self.params['num_sup_around']

        # gen_cls = cast(Type[MOSBase], import_class(cls_name))
        core_master: GenericWrapper = self.new_template(GenericWrapper, params=dict(export_private=False,
                                                                                    cls_name=cls_name,
                                                                                    params=core_params))
        self._core_master = core_master.core

        tr_manager = core_master.core.tr_manager
        top_sup_dir = self.grid.get_direction(top_layer)
        top_layer_x = top_layer if top_sup_dir == Orient2D.x else top_layer - 1
        top_layer_y = top_layer - 1 if top_sup_dir == Orient2D.x else top_layer

        top_x_w = TrackID(top_layer_x, 0, tr_manager.get_width(top_layer_x, 'sup'), grid=self.grid).width
        top_x_sep = self.get_track_sep(top_layer_x, top_x_w, top_x_w)
        top_y_w = TrackID(top_layer_y, 0, tr_manager.get_width(top_layer_y, 'sup'), grid=self.grid).width
        top_y_sep = self.get_track_sep(top_layer_x, top_y_w, top_y_w)
        num_sup_x, num_sup_y = num_sup_around[0], num_sup_around[1]
        top_x_spec, top_y_spec = self.grid.get_track_info(top_layer_x), self.grid.get_track_info(top_layer_y)
        core_y, core_x = top_x_spec.offset + num_sup_y * top_x_sep * top_x_spec.pitch, \
                         top_y_spec.offset + num_sup_x * top_y_sep * top_y_spec.pitch

        bbox_top_layer = max(top_layer, top_out_layer)
        blk_w, blk_h = self.grid.get_block_size(bbox_top_layer, half_blk_x=False, half_blk_y=False)

        core = self.add_instance(core_master, xform=Transform(-(-core_x // blk_w) * blk_w, -(-core_y // blk_h) * blk_h))

        h_tot = -(-(core.bound_box.yh + num_sup_y * (top_x_sep) * top_x_spec.pitch) // blk_h) * blk_h
        w_tot = -(-(core.bound_box.xh + num_sup_x * (top_y_sep) * top_y_spec.pitch) // blk_w) * blk_w
        self.set_size_from_bound_box(bbox_top_layer, BBox(0, 0, w_tot, h_tot))

        conn_layer = core_master.core.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        # in_vm = core.get_pin('vin')
        # tr_w_ana_sig_xm = tr_manager.get_width(xm_layer, 'ana_sig')
        # in_xm_tidx = self.grid.coord_to_track(xm_layer, in_vm.middle, RoundMode.NEAREST)
        # in_xm = self.connect_to_tracks(in_vm, TrackID(xm_layer, in_xm_tidx, tr_w_ana_sig_xm, grid=self.grid))
        # out_vm = core.get_pin('vout')
        # out_dict = self.via_stack_up(tr_manager, out_vm, vm_layer, top_out_layer, 'ana_sig', bbox=out_vm.bound_box)

        vdd_xm = self.extend_wires(core.get_pin('VDD'), lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm = self.extend_wires(core.get_pin('VSS'), lower=self.bound_box.xl, upper=self.bound_box.xh)
        power_dict = self.connect_supply_stack_warr(tr_manager, [vdd_xm, vss_xm], xm_layer, top_layer, self.bound_box)
        self.add_pin('VDD', power_dict[0][top_layer])
        self.add_pin('VSS', power_dict[1][top_layer])
        # self.add_pin('vout', out_dict[top_layer])
        for pin in core.port_names_iter():
            if pin in ['inn', 'inp', 'vbn', 'bias', 'out']:
                self.reexport(core.get_port(pin))

        # self.add_pin('vin', in_xm)

        self._sch_params = core_master.sch_params
