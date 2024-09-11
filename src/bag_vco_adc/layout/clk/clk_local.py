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


from typing import Any, Dict, Type, Optional, Tuple, Union
from typing import Mapping

from bag.design.database import ModuleDB, Module
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableList
from bag.util.math import HalfInt
from bag_vco_adc.layout.digital import NAND2Core, InvCore, InvTristateCore, FlopCore, NOR2Core, InvChainCore
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from bag_vco_adc.layout.util.wrapper import GenericWrapper, IntegrationWrapper
from bag_vco_adc.layout.vco.vco_cnter_dec import MuxCore
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orientation, MinLenMode, PinMode
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo


def export_vm_diff_wire(base: TemplateBase, vm_wire_n, vm_wire_p, xm_tidx_n, xm_tidx_p, wire_type, tr_manager, vm_layer,
                        export_ym=False, xm_diff=True, ym_tidx_n=None, ym_tidx_p=None):
    xm_layer = vm_layer + 1
    ym_layer = xm_layer + 1
    tr_w_xm = tr_manager.get_width(xm_layer, wire_type)
    tr_w_ym = tr_manager.get_width(ym_layer, wire_type)
    if xm_diff:
        xm_wire_n, xm_wire_p = base.connect_differential_tracks(vm_wire_n, vm_wire_p, xm_layer,
                                                                xm_tidx_n, xm_tidx_p, width=tr_w_xm)
    else:
        xm_wire_n = base.connect_to_tracks(vm_wire_n,
                                           TrackID(xm_layer, xm_tidx_n, tr_w_xm), min_len_mode=MinLenMode.MIDDLE)
        xm_wire_p = base.connect_to_tracks(vm_wire_p,
                                           TrackID(xm_layer, xm_tidx_p, tr_w_xm), min_len_mode=MinLenMode.MIDDLE)
    if export_ym:
        _, ym_locs = tr_manager.place_wires(ym_layer, [wire_type, wire_type], center_coord=xm_wire_n.middle)
        if ym_tidx_n and ym_tidx_p:
            ym_locs = [ym_tidx_n, ym_tidx_p]
        ym_wire_n, ym_wire_p = base.connect_differential_tracks(xm_wire_n, xm_wire_p, ym_layer,
                                                                ym_locs[0], ym_locs[1], width=tr_w_ym)
    else:
        ym_wire_n, ym_wire_p = None, None

    return (xm_wire_n, xm_wire_p), (ym_wire_n, ym_wire_p)


class ClkNonOverlap(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_nonoverlap')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_nor='Number of segments.',
            seg_chain='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']

        seg_nor: int = self.params['seg_nor']
        seg_chain: int = self.params['seg_chain']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        nd_tidx = get_rel_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=1)
        pd_tidx = get_rel_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)
        ng1_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=1)
        ng2_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)
        pinfo = self.get_tile_pinfo(1)
        nor_params = dict(pinfo=pinfo, seg=seg_nor, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          sig_locs={'nin0': ng1_tidx, 'nin1': ng2_tidx, 'nout': nd_tidx, 'pout': pd_tidx},
                          vertical_sup=True, vertical_out=True)
        chain_params = dict(pinfo=pinfo, seg_list=seg_chain, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            sig_locs={}, vertical_sup=True, dual_output=False)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        chain_master = self.new_template(InvChainCore, params=chain_params)

        cur_col = 0
        _, nor_out_vm_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 2,
                                                    center_coord=self.arr_info.col_to_coord(cur_col + seg_nor))
        norn = self.add_tile(nor_master, tile_idx=1, col_idx=cur_col)
        norp = self.add_tile(nor_master, tile_idx=3, col_idx=cur_col)

        cur_col += nor_master.num_cols + min_sep
        cur_col += cur_col & 1

        chainn = self.add_tile(chain_master, tile_idx=1, col_idx=cur_col)
        chainp = self.add_tile(chain_master, tile_idx=3, col_idx=cur_col)
        self.set_mos_size(num_tiles=5)

        # outn_vm, outp_vm = self.connect_differential_tracks([norn.get_pin('nout'), norn.get_pin('pout')],
        #                                                     [norp.get_pin('nout'), norp.get_pin('pout')],
        #                                                     vm_layer, nor_out_vm_tidx[0], nor_out_vm_tidx[1],
        #                                                     width=tr_manager.get_width(vm_layer, 'sig'))
        self.connect_to_track_wires(norn.get_pin('out'), chainn.get_pin('nin'))
        self.connect_to_track_wires(norp.get_pin('out'), chainp.get_pin('nin'))
        _, nor_xcp_input_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3, align_idx=0,
                                                          align_track=norn.get_pin('out').track_id.base_index)
        nor_xcp_input_vm_locs = nor_xcp_input_vm_locs[1:]
        nor_xcp_inn_vm, nor_xcp_inp_vm = self.connect_differential_tracks(norn.get_pin('nin<0>'),
                                                                          norp.get_pin('nin<0>'), vm_layer,
                                                                          nor_xcp_input_vm_locs[0],
                                                                          nor_xcp_input_vm_locs[1],
                                                                          width=tr_w_vm)
        xcp_outn_xm_tidx = self.grid.coord_to_track(xm_layer, chainn.get_pin('out').middle, RoundMode.NEAREST)
        xcp_outp_xm_tidx = self.grid.coord_to_track(xm_layer, chainp.get_pin('out').middle, RoundMode.NEAREST)

        self.connect_to_tracks([nor_xcp_inp_vm, chainn.get_pin('out')], TrackID(xm_layer, xcp_outn_xm_tidx, tr_w_xm))
        self.connect_to_tracks([nor_xcp_inn_vm, chainp.get_pin('out')], TrackID(xm_layer, xcp_outp_xm_tidx, tr_w_xm))
        nor_in_vm_tidx = tr_manager.get_next_track(vm_layer, norn.get_pin('out').track_id.base_index, 'sig', 'sig',
                                                   up=False)
        clkn_vm = self.connect_to_tracks(norn.get_pin('nin<1>'), TrackID(vm_layer, nor_in_vm_tidx, tr_w_vm),
                                         min_len_mode=MinLenMode.MIDDLE)
        clkp_vm = self.connect_to_tracks(norp.get_pin('nin<1>'), TrackID(vm_layer, nor_in_vm_tidx, tr_w_vm),
                                         min_len_mode=MinLenMode.MIDDLE)
        inout_xm_tidx_n = self.grid.coord_to_track(xm_layer, clkn_vm.middle, mode=RoundMode.NEAREST)
        inout_xm_tidx_p = self.grid.coord_to_track(xm_layer, clkp_vm.middle, mode=RoundMode.NEAREST)

        (clkn_xm, clkp_xm), _ = export_vm_diff_wire(self, clkn_vm, clkp_vm, inout_xm_tidx_n,
                                                    inout_xm_tidx_p, 'sig', tr_manager, vm_layer, False, False)
        (outn_xm, outp_xm), (outn_ym, outp_ym) = \
            export_vm_diff_wire(self, chainn.get_pin('out'), chainp.get_pin('out'), inout_xm_tidx_n,
                                inout_xm_tidx_p, 'sig', tr_manager, vm_layer, True, False)

        self.add_pin('outp', [outp_xm, outp_ym])
        self.add_pin('outn', [outn_xm, outn_ym])
        self.add_pin('clkp', clkp_xm)
        self.add_pin('clkn', clkn_xm)

        #
        vdd_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()
        vss_bot_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_top_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_list, vss_list = [], []
        for inst in [norn, norp, chainn, chainp]:
            vdd_list.append(inst.get_pin('VDD'))
        #
        vdd_hm_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2)
        vss_hm_bot_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0)
        vss_hm_top_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=4)

        vss_hm_list = \
            [self.connect_to_tracks(norp.get_all_port_pins('VSS') + chainp.get_all_port_pins('VSS') + vss_top_sub_conn,
                                    vss_hm_top_tid),
             self.connect_to_tracks(norn.get_all_port_pins('VSS') + chainn.get_all_port_pins('VSS') + vss_bot_sub_conn,
                                    vss_hm_bot_tid)]
        vdd_hm = self.connect_to_tracks(vdd_list + vdd_sub_conn, vdd_hm_tid)

        self.add_pin('VDD', vdd_hm, connect=True)
        self.add_pin('VSS', vss_hm_list, connect=True)

        sch_params_dict = dict(
            chain=chain_master.sch_params,
            nor=nor_master.sch_params,
        )
        self.sch_params = sch_params_dict


class ClkRetimer(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_retime_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            sig_locs='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            sig_locs={},
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_tinv = seg_dict['tinv']
        seg_inv = seg_dict['inv']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        ng0_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        ng0_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        ng1_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=1)
        ng2_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)
        pg0_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)
        sig_locs = self.params['sig_locs']
        in_loc = sig_locs.get('in', ng2_tidx)
        pinfo = self.get_tile_pinfo(1)
        tinv_params = dict(pinfo=pinfo, seg=seg_tinv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           sig_locs={'nin': in_loc, 'nen': ng0_tidx, 'pen': pg0_tidx}, vertical_sup=True)
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_out=False, sig_locs={'nin': ng1_tidx}, vertical_sup=True)
        tinv_master = self.new_template(InvTristateCore, params=tinv_params)
        inv_master = self.new_template(InvCore, params=inv_params)

        cur_col = 0
        tinvp = self.add_tile(tinv_master, tile_idx=1, col_idx=cur_col)
        tinvn = self.add_tile(tinv_master, tile_idx=3, col_idx=cur_col)

        cur_col += tinv_master.num_cols + min_sep
        invp = self.add_tile(inv_master, tile_idx=1, col_idx=cur_col)
        invn = self.add_tile(inv_master, tile_idx=3, col_idx=cur_col)
        self.set_mos_size(num_tiles=5)

        self.connect_to_track_wires(tinvn.get_pin('out'), invn.get_pin('nin'))
        self.connect_to_track_wires(tinvp.get_pin('out'), invp.get_pin('nin'))

        _, out_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 2,
                                                center_coord=(invn.bound_box.xl + invn.bound_box.xh) // 2)

        outp_vm, outn_vm = self.connect_matching_tracks(
            [[invn.get_pin('nout'), invn.get_pin('pout'), invp.get_pin('nin')],
             [invp.get_pin('nout'), invp.get_pin('pout'), invn.get_pin('nin')]],
            vm_layer, out_vm_locs, width=tr_manager.get_width(vm_layer, 'sig'))

        outp_hm_tidx = self.get_track_id(ridx_p, MOSWireType.G, 'sig', wire_idx=-2, tile_idx=1)
        outn_hm_tidx = self.get_track_id(ridx_p, MOSWireType.G, 'sig', wire_idx=-2, tile_idx=3)
        outp_hm0 = self.connect_to_tracks([tinvp.get_pin('out'), outp_vm], outp_hm_tidx)
        outn_hm0 = self.connect_to_tracks([tinvn.get_pin('out'), outn_vm], outn_hm_tidx)

        en_hm = [tinvp.get_pin('en'), tinvn.get_pin('en')]
        enb_hm = [tinvp.get_pin('enb'), tinvn.get_pin('enb')]

        # en_vm_tidx = self.grid.coord_to_track(vm_layer, en_hm[0].lower, mode=RoundMode.GREATER)
        tinv_vm_tidx = tinvn.get_pin('out').track_id.base_index
        tr_w_vm_sep = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        # _, en_vm_locs = tr_manager.place_wires(vm_layer, ['clk'] * 2, align_track=en_vm_tidx)
        en_vm_locs = [tinv_vm_tidx - tr_w_vm_sep, tinv_vm_tidx + tr_w_vm_sep]
        en_vm, enb_vm = self.connect_matching_tracks([en_hm, enb_hm], vm_layer, en_vm_locs,
                                                     width=tr_manager.get_width(vm_layer, 'clk'))

        outp_hm0, outp_hm0 = self.extend_wires([outp_hm0, outn_hm0], upper=max(outp_hm0.upper, outn_hm0.upper),
                                               lower=min(outn_hm0.lower, outp_hm0.lower))

        self.add_pin('outp_hm', [outp_hm0, invp.get_pin('nin')], label='outp')
        self.add_pin('outn_hm', [outn_hm0, invn.get_pin('nin')], label='outn')

        self.add_pin('outp', outp_vm)
        self.add_pin('outn', outn_vm)
        self.add_pin('clk', en_vm)
        self.add_pin('clkb', enb_vm)

        self.add_pin('inp', tinvp.get_pin('nin'))
        self.add_pin('inn', tinvn.get_pin('nin'))

        vdd_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()
        vss_bot_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_top_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_list, vss_list = [], []
        for inst in [tinvp, tinvn, invp, invn]:
            vdd_list.append(inst.get_pin('VDD'))

        vdd_hm_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2)
        vss_hm_bot_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0)
        vss_hm_top_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=4)

        vss_hm_list = \
            [self.connect_to_tracks(invp.get_all_port_pins('VSS') + tinvp.get_all_port_pins('VSS') + vss_bot_sub_conn,
                                    vss_hm_bot_tid),
             self.connect_to_tracks(invn.get_all_port_pins('VSS') + tinvn.get_all_port_pins('VSS') + vss_top_sub_conn,
                                    vss_hm_top_tid)]
        vdd_hm = self.connect_to_tracks(vdd_list + vdd_sub_conn, vdd_hm_tid)

        self.add_pin('VDD', vdd_hm, connect=True)
        self.add_pin('VSS', vss_hm_list, connect=True)

        sch_params_dict = dict(
            inv=inv_master.sch_params,
            tinv=tinv_master.sch_params,
        )
        self.sch_params = sch_params_dict


class DiffInvCoupledRow(MOSBase, TemplateBaseZL):
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
            flip_io_hm='True to flip io hm',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            flip_io_hm=False
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        flip_io_hm: bool = self.params['flip_io_hm']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_in = seg_dict['inv']
        seg_cp = seg_dict['cp']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_sp_vm_sig = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        tr_w_sig_hm = tr_manager.get_width(hm_layer, 'sig')
        tr_sp_sig_hm = tr_manager.get_sep(hm_layer, ('sig', 'sig'))

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        nd_tidx = get_rel_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)
        pd_tidx = get_rel_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)
        pinfo = self.get_tile_pinfo(1)
        if seg_cp:
            cp_params = dict(pinfo=pinfo, seg=seg_cp, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             vertical_out=False, vertical_sup=True, conn_in=True,
                             sig_locs={'nout': nd_tidx, 'pout': pd_tidx})
            cp_master = self.new_template(InvCore, params=cp_params)
        else:
            cp_master = None

        inv_params = dict(pinfo=pinfo, seg=seg_in, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=True, conn_in=True, sig_locs={'nout': nd_tidx, 'pout': pd_tidx},
                          vertical_out=cp_master is not None)
        inv_master = self.new_template(InvCore, params=inv_params)

        cur_col = 0
        invp = self.add_tile(inv_master, tile_idx=1, col_idx=cur_col)
        invn = self.add_tile(inv_master, tile_idx=3, col_idx=cur_col)

        if cp_master:
            cur_col += inv_master.num_cols + min_sep
            cpp = self.add_tile(cp_master, tile_idx=1, col_idx=cur_col)
            cpn = self.add_tile(cp_master, tile_idx=3, col_idx=cur_col)
            self.set_mos_size()
        else:
            self.set_mos_size(num_cols=cur_col)
            cpp, cpn = None, None

        # Get tile0 gate connections
        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        pg0_tidx = max(self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1),
                       self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1))

        tile0_hm_tracks = self.get_tids_between(hm_layer, ng0_tidx, pg0_tidx, width=tr_w_sig_hm, sep=tr_sp_sig_hm,
                                                sep_margin=0, include_last=True)

        # Get tile1 gate connections
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=3)
        pg1_tidx = min(self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=3),
                       self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=3))

        tr_w_sig_hm = tr_manager.get_width(hm_layer, 'sig')
        tr_sp_sig_hm = tr_manager.get_sep(hm_layer, ('sig', 'sig'))
        tile1_hm_tracks = self.get_tids_between(hm_layer, pg1_tidx, ng1_tidx, width=tr_w_sig_hm, sep=tr_sp_sig_hm,
                                                sep_margin=0, include_last=True, align_to_higher=True)[::-1]

        num_avail_g_hm = len(tile0_hm_tracks)
        inp_hm_tidx = tile0_hm_tracks[:num_avail_g_hm // 2]
        inn_hm_tidx = tile1_hm_tracks[:num_avail_g_hm // 2]

        cpn_hm_tidx = [tile0_hm_tracks[-num_avail_g_hm // 2]]
        cpp_hm_tidx = [tile1_hm_tracks[-num_avail_g_hm // 2]]

        outn_hm_tidx = tile0_hm_tracks[-(num_avail_g_hm // 2):]
        outp_hm_tidx = tile1_hm_tracks[-(num_avail_g_hm // 2):]

        if flip_io_hm:
            inn_hm_tidx, inp_hm_tidx = outp_hm_tidx, outn_hm_tidx

        inn_hm = [self.connect_to_tracks(invn.get_pin('in'), tidx) for tidx in inn_hm_tidx]
        inp_hm = [self.connect_to_tracks(invp.get_pin('in'), tidx) for tidx in inp_hm_tidx]

        [self.extend_wires(warr, upper=self.bound_box.xh) for warr in [invp.get_pin('nout'), invp.get_pin('pout'),
                                                                       invn.get_pin('nout'), invn.get_pin('pout')]]

        if cp_master:
            outn_vm, outp_vm = [invp.get_pin('out')], [invn.get_pin('out')]
            outn_hm = [self.connect_to_tracks(cpp.get_pin('in'), tidx) for tidx in cpn_hm_tidx]
            outp_hm = [self.connect_to_tracks(cpn.get_pin('in'), tidx) for tidx in cpp_hm_tidx]

            tr_w_vm_sig = tr_manager.get_width(vm_layer, 'sig_thin')
            tr_sp_vm_sig = tr_manager.get_sep(vm_layer, ('sig_thin', 'sig_thin'))

            cp_out_vm_tidx = self.grid.coord_to_track(vm_layer, (cpp.bound_box.xl + cpp.bound_box.xh) // 2,
                                                      RoundMode.NEAREST)
            out_vm_tidx_sep = self.get_track_sep(vm_layer, tr_w_vm_sig, 1)
            outn_vm_tidx = cp_out_vm_tidx - out_vm_tidx_sep
            outp_vm_tidx = cp_out_vm_tidx + out_vm_tidx_sep

            out_hm_ret_list = []
            outn_cp_vm = self.connect_to_tracks(outn_hm + [cpn.get_pin('nout'), cpn.get_pin('pout')],
                                                TrackID(vm_layer, outn_vm_tidx, tr_w_vm_sig),
                                                ret_wire_list=out_hm_ret_list)
            outp_cp_vm = self.connect_to_tracks(outp_hm + [cpp.get_pin('nout'), cpp.get_pin('pout')],
                                                TrackID(vm_layer, outp_vm_tidx, tr_w_vm_sig),
                                                ret_wire_list=out_hm_ret_list)
            outn_cp_vm, outp_cp_vm = self.match_warr_length([outn_cp_vm, outp_cp_vm])
            self.match_warr_length(out_hm_ret_list)
            outn_vm.append(outp_cp_vm)
            outp_vm.append(outn_cp_vm)
        else:
            tr_w_vm_sig = max(tr_manager.get_width(vm_layer, 'sig_w'),
                              tr_manager.get_width(vm_layer, 'sig'))

            cp_out_vm_tidx = self.grid.coord_to_track(vm_layer, (invp.bound_box.xl + invp.bound_box.xh) // 2,
                                                      RoundMode.NEAREST)
            out_vm_tidx_sep = self.get_track_sep(vm_layer, tr_w_vm_sig, tr_w_vm_sig).div2() + HalfInt(1)
            outn_vm_tidx = cp_out_vm_tidx - out_vm_tidx_sep
            outp_vm_tidx = cp_out_vm_tidx + out_vm_tidx_sep
            out_hm_ret_list = []
            outn_vm = self.connect_to_tracks([invp.get_pin('nout'), invp.get_pin('pout')],
                                             TrackID(vm_layer, outn_vm_tidx, tr_w_vm_sig),
                                             ret_wire_list=out_hm_ret_list)
            outp_vm = self.connect_to_tracks([invn.get_pin('nout'), invn.get_pin('pout')],
                                             TrackID(vm_layer, outp_vm_tidx, tr_w_vm_sig),
                                             ret_wire_list=out_hm_ret_list)

            outn_vm, outp_vm = self.match_warr_length([outn_vm, outp_vm])

        self.add_pin('outp', outp_vm)
        self.add_pin('outn', outn_vm)
        self.add_pin('inn', inn_hm)
        self.add_pin('inp', inp_hm)
        #
        vdd_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()
        vss_bot_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_top_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_list, vss_list = [], []
        for inst in [invp, invn]:
            vdd_list.append(inst.get_pin('VDD'))

        if cp_master:
            for inst in [cpp, cpn]:
                vdd_list.append(inst.get_pin('VDD'))

        vdd_hm_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2)
        vss_hm_bot_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0)
        vss_hm_top_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=4)

        vss_hm_list = \
            [self.connect_to_tracks(invp.get_all_port_pins('VSS') + vss_bot_sub_conn, vss_hm_bot_tid),
             self.connect_to_tracks(invn.get_all_port_pins('VSS') + vss_top_sub_conn, vss_hm_top_tid)]

        if cp_master:
            vss_hm_list.extend([self.connect_to_tracks(cpp.get_all_port_pins('VSS'), vss_hm_bot_tid),
                                self.connect_to_tracks(cpn.get_all_port_pins('VSS'), vss_hm_top_tid)])
            vss_hm_list = self.connect_wires(vss_hm_list)
        vdd_hm = self.connect_to_tracks(vdd_list + vdd_sub_conn, vdd_hm_tid)

        self.add_pin('VDD', vdd_hm, connect=True)
        self.add_pin('VSS', vss_hm_list, connect=True)

        sch_params_dict = dict(
            inv=inv_master.sch_params,
            fb=cp_master.sch_params if cp_master else None,
        )
        self.sch_params = sch_params_dict


class DiffInvCoupledChainBuf(MOSBase, TemplateBaseZL):
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
            seg_buf='',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            sig_locs='',
            export_mid0='',
            export_in='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            sig_locs={},
            export_mid0=False,
            export_in=False
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_buf: Dict[str, Any] = self.params['seg_buf']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        tr_manager = self.tr_manager
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_sig_sep = self.get_track_sep(vm_layer, tr_w_sig_vm, tr_w_sig_vm)
        in_vm_ncols = self.arr_info.get_column_span(vm_layer, tr_w_sig_sep)
        # compute track locations
        ng2_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)

        diff_inv_params_list = []
        for idx in range(len(seg_buf['inv'])):
            diff_inv_params_list.append(
                dict(seg_dict=dict(inv=seg_buf['inv'][idx], cp=seg_buf['cp'][idx]), pinfo=pinfo,
                     w_n=w_n, w_p=w_p, flip_io_hm=idx & 1))
        diff_inv_master_list = [self.new_template(DiffInvCoupledRow, params=p) for p in diff_inv_params_list]
        # inv_master = self.new_template(InvCore, params=inv_params)
        #
        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        cur_col = min_sep + in_vm_ncols
        diff_inv_inst_list = []
        for master in diff_inv_master_list:
            diff_inv_inst_list.append(self.add_tile(master, tile_idx=0, col_idx=cur_col))
            cur_col += master.num_cols + 2 * min_sep

        self.set_mos_size(self.num_cols + min_sep)

        if len(diff_inv_inst_list) > 1:
            for idx in range(len(diff_inv_inst_list) - 1):
                for _inn, _inp in zip(diff_inv_inst_list[idx + 1].get_all_port_pins('inn'),
                                      diff_inv_inst_list[idx + 1].get_all_port_pins('inp')):
                    ret_inn_hm = self.connect_to_track_wires(diff_inv_inst_list[idx].get_all_port_pins('outp'), _inn)
                    ret_inp_hm = self.connect_to_track_wires(diff_inv_inst_list[idx].get_all_port_pins('outn'), _inp)
                    self.match_warr_length([ret_inn_hm, ret_inp_hm])

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        vdd_hm = self.connect_wires([inst.get_pin('VDD') for inst in diff_inv_inst_list])
        vss_hm = self.connect_wires([w for inst in diff_inv_inst_list for w in inst.get_all_port_pins('VSS')])[
            0].to_warr_list()

        vdd_xm = self.export_tap_hm(tr_manager, vdd_hm[0], hm_layer, xm_layer)[0]
        vss_xm = [self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0] for vss in vss_hm]
        vdd_xm = self.extend_wires(vdd_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm = self.extend_wires(vss_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)

        vdd_xm1 = self.export_tap_hm(tr_manager, vdd_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_bot = self.export_tap_hm(tr_manager, vss_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_top = self.export_tap_hm(tr_manager, vss_xm[1], xm_layer, xm1_layer)[0]
        in_vm_tidx = self.arr_info.col_to_track(vm_layer, in_vm_ncols)
        inn_vm = self.connect_to_tracks(diff_inv_inst_list[0].get_all_port_pins('inn'),
                                        TrackID(vm_layer, in_vm_tidx, tr_w_sig_vm))
        inp_vm = self.connect_to_tracks(diff_inv_inst_list[0].get_all_port_pins('inp'),
                                        TrackID(vm_layer, in_vm_tidx, tr_w_sig_vm))
        export_in = self.params['export_in']
        if export_in:
            inp_stack = self.via_stack_up(tr_manager, inp_vm, vm_layer, xm1_layer, 'sig', RoundMode.NEAREST)
            inn_stack = self.via_stack_up(tr_manager, inn_vm, vm_layer, xm1_layer, 'sig', RoundMode.NEAREST)
        else:
            inp_stack = self.via_stack_up(tr_manager, inp_vm, vm_layer, xm_layer, 'sig', RoundMode.NEAREST)
            inn_stack = self.via_stack_up(tr_manager, inn_vm, vm_layer, xm_layer, 'sig', RoundMode.NEAREST)

        for idx, inst in enumerate(diff_inv_inst_list[:-1]):
            self.add_pin(f'midp<{idx}>' if idx & 1 else f'midn<{idx}>', [inst.get_pin('outn')])
            self.add_pin(f'midn<{idx}>' if idx & 1 else f'midp<{idx}>', [inst.get_pin('outp')])

        outn_vm = diff_inv_inst_list[-1].get_pin('outn')
        outp_vm = diff_inv_inst_list[-1].get_pin('outp')

        out_vm_h = outn_vm.bound_box.h
        out_vm_xl = min(outn_vm.bound_box.xl, outp_vm.bound_box.xl)
        out_vm_xh = max(outn_vm.bound_box.xh, outp_vm.bound_box.xh)
        outp_vm_bbox = BBox(out_vm_xl, outp_vm.bound_box.yh - out_vm_h // 3, out_vm_xh, outp_vm.bound_box.yh)
        outn_vm_bbox = BBox(out_vm_xl, outn_vm.bound_box.yl, out_vm_xh, outn_vm.bound_box.yl + out_vm_h // 3)

        outp_stack = self.via_stack_up(tr_manager, outp_vm, vm_layer, xm1_layer, 'sig', RoundMode.NEAREST,
                                       bbox=outp_vm_bbox)
        outn_stack = self.via_stack_up(tr_manager, outn_vm, vm_layer, xm1_layer, 'sig', RoundMode.NEAREST,
                                       bbox=outn_vm_bbox)
        if self.params['export_mid0']:
            midn0_vm = diff_inv_inst_list[0].get_pin('outn')
            midp0_vm = diff_inv_inst_list[0].get_pin('outp')

            mid_vm_h = midn0_vm.bound_box.h
            mid_vm_xl = min(midn0_vm.bound_box.xl, midp0_vm.bound_box.xl)
            mid_vm_xh = max(midp0_vm.bound_box.xh, midn0_vm.bound_box.xh)
            midn_vm_bbox = BBox(mid_vm_xl, midp0_vm.bound_box.yh - mid_vm_h // 3, mid_vm_xh,
                                midp0_vm.bound_box.yh)
            midp_vm_bbox = BBox(mid_vm_xl, midp0_vm.bound_box.yl, mid_vm_xh,
                                midp0_vm.bound_box.yl + out_vm_h // 3)

            midp_stack = self.via_stack_up(tr_manager, midp0_vm, vm_layer, xm1_layer, 'sig', RoundMode.NEAREST,
                                           bbox=midp_vm_bbox)
            midn_stack = self.via_stack_up(tr_manager, midn0_vm, vm_layer, xm1_layer, 'sig', RoundMode.NEAREST,
                                           bbox=midn_vm_bbox)
            [self.add_pin('midn<0>', midn_stack[idx]) for idx in range(vm_layer + 1, xm1_layer + 1)]
            [self.add_pin('midp<0>', midp_stack[idx]) for idx in range(vm_layer + 1, xm1_layer + 1)]
        self.add_pin('inp_vm', inp_vm, hide=True)
        self.add_pin('inn_vm', inn_vm, hide=True)
        [self.add_pin('inp', inp_stack[idx]) for idx in
         range(vm_layer + 1, xm1_layer + 1 if export_in else xm_layer + 1)]
        [self.add_pin('inn', inn_stack[idx]) for idx in
         range(vm_layer + 1, xm1_layer + 1 if export_in else xm_layer + 1)]
        [self.add_pin('outp' if len(diff_inv_inst_list) & 1 else 'outn',
                      outp_stack[idx]) for idx in range(vm_layer + 1, xm1_layer + 1)]
        [self.add_pin('outn' if len(diff_inv_inst_list) & 1 else 'outp',
                      outn_stack[idx]) for idx in range(vm_layer + 1, xm1_layer + 1)]

        self.add_pin('VDD_hm', vdd_hm, hide=True)
        self.add_pin('VSS_hm', vss_hm, hide=True)
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', [vss_xm1_bot, vss_xm1_top], label='VSS', show=self.show_pins)
        sch_params_dict = dict(inv_list=[master.sch_params for master in diff_inv_master_list],
                               export_mid=True)
        self.sch_params = sch_params_dict


class ClkLocalRetimer(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._clk_col_alignment = 0
        self._output_cols_alignment = [0, 0]

    @property
    def clk_col_alignment(self):
        return self._clk_col_alignment

    @property
    def output_cols_alignment(self):
        return self._output_cols_alignment

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_retimer')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_ret_s='Number of segments.',
            seg_ret_m='Number of segments.',
            seg_buf='',
            seg_nand='',
            seg_nor='',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            shift_input='',
            sig_locs='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            shift_input=False,
            sig_locs={}
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_ret_m: Dict[str, Any] = self.params['seg_ret_m']
        seg_ret_s: Dict[str, Any] = self.params['seg_ret_s']
        seg_buf: Dict[str, Any] = self.params['seg_buf']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        # Make templates
        ng0_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)
        ng1_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)
        ret_m_params = dict(seg_dict=seg_ret_m, pinfo=pinfo)
        ret_s_params = dict(seg_dict=seg_ret_s, pinfo=pinfo)
        nand_params = dict(seg=self.params['seg_nand'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                           vertical_out=False, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        nor_params = dict(seg=self.params['seg_nor'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                          vertical_out=False, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})

        diff_inv_params = dict(seg_buf=seg_buf, pinfo=pinfo)
        diff_inv_master = self.new_template(DiffInvCoupledChainBuf, params=diff_inv_params)

        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        ret_m_master = self.new_template(ClkRetimer, params=ret_m_params)
        ret_s_master = self.new_template(ClkRetimer, params=ret_s_params)

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        in_vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * 4)
        cur_col = self.arr_info.get_column_span(vm_layer, in_vm_ntr)

        cur_col += min_sep
        nand_in_col = cur_col
        nand = self.add_tile(nand_master, tile_idx=3, col_idx=cur_col)
        nor = self.add_tile(nor_master, tile_idx=1, col_idx=cur_col)
        cur_col += min_sep

        vss_conn_bot = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_conn_top = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()
        cur_col += max(nand_master.num_cols, nor_master.num_cols)
        tap_mid_col = cur_col
        retimer_m = self.add_tile(ret_m_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_m_master.num_cols + min_sep
        retimer_s = self.add_tile(ret_s_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_s_master.num_cols

        diff_buf = self.add_tile(diff_inv_master, tile_idx=0, col_idx=cur_col)
        self.set_mos_size(self.num_cols + min_sep)

        # connect nand/nor to retimer
        _, nandnor_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 2, center_coord=nand.get_pin('nout').middle)
        self.connect_to_tracks([nand.get_pin('nout'), nand.get_pin('pout'), retimer_m.get_pin('inn')],
                               TrackID(vm_layer, nandnor_vm_locs[0], tr_w_vm))
        self.connect_to_tracks([nor.get_pin('nout'), nor.get_pin('pout'), retimer_m.get_pin('inp')],
                               TrackID(vm_layer, nandnor_vm_locs[1], tr_w_vm))
        self.connect_to_track_wires(retimer_m.get_pin('outn'), retimer_s.get_pin('inn'))
        self.connect_to_track_wires(retimer_m.get_pin('outp'), retimer_s.get_pin('inp'))

        # connect nand/nor input
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 5, align_idx=-1,
                                               align_track=self.arr_info.col_to_track(vm_layer, nand_in_col))
        nandnor_in_list = self.connect_matching_tracks([nor.get_pin('nin<0>'), nand.get_pin('nin<0>'),
                                                        nor.get_pin('nin<1>'), nand.get_pin('nin<1>')],
                                                       vm_layer, in_vm_locs[:-1],
                                                       width=tr_manager.get_width(vm_layer, 'sig'))

        self.connect_to_track_wires(retimer_s.get_pin('outn'), diff_buf.get_all_port_pins('inn', layer=xm_layer))
        self.connect_to_track_wires(retimer_s.get_pin('outp'), diff_buf.get_all_port_pins('inp', layer=xm_layer))

        # === hm supply ===
        vdd_hm = self.connect_to_track_wires(vdd_conn, retimer_m.get_pin('VDD'))
        vss_hm_bot = min(retimer_m.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_top = max(retimer_m.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_list = [self.connect_to_track_wires(vss_conn_bot, vss_hm_bot),
                       self.connect_to_track_wires(vss_conn_top, vss_hm_top)]

        self.connect_to_track_wires(nor.get_all_port_pins('VDD') + nand.get_all_port_pins('VDD'),
                                    vdd_hm)
        self.connect_to_track_wires(vss_hm_list[0], nor.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vss_hm_list[1], nand.get_all_port_pins('VSS'))
        vdd_hm = self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_bot = self.extend_wires(vss_hm_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_top = self.extend_wires(vss_hm_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]

        vdd_xm = self.export_tap_hm(tr_manager, vdd_hm[0], hm_layer, xm_layer)
        vss_xm = [self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0] for vss in [vss_hm_top, vss_hm_bot]]
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')

        tr_sp_sup_sig_xm = self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sup_xm)
        mid_xm_tidx = self.grid.coord_to_track(xm_layer, self.bound_box.h // 2, RoundMode.NEAREST)
        clk_xm_tidx_u = mid_xm_tidx + 2 * tr_sp_sup_sig_xm
        clk_xm_tidx_l = mid_xm_tidx - 2 * tr_sp_sup_sig_xm
        clk_xm, clkn_xm = self.connect_differential_tracks([retimer_m.get_pin('clkb'), retimer_s.get_pin('clk')],
                                                           [retimer_m.get_pin('clk'), retimer_s.get_pin('clkb')],
                                                           xm_layer, clk_xm_tidx_u, clk_xm_tidx_l,
                                                           width=tr_w_clk_xm)

        ym_layer = xm_layer + 1
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        sig_locs = self.params['sig_locs']
        if 'clk' in sig_locs.keys():
            clk_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols - sig_locs['clk'], RoundMode.NEAREST)
        else:
            clk_ym_tidx = self.grid.coord_to_track(ym_layer, clkn_xm.middle, RoundMode.NEAREST)
        clkb_ym_tidx = tr_manager.get_next_track(ym_layer, clk_ym_tidx, 'clk', 'clk')
        self._clk_col_alignment = self.num_cols - self.arr_info.track_to_col(ym_layer, clk_ym_tidx)
        clk_ym, clkb_ym = self.connect_differential_tracks(clk_xm, clkn_xm, ym_layer, clk_ym_tidx,
                                                           clkb_ym_tidx, width=tr_w_clk_ym)

        # Connect in0, in1, rst
        _, input_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 4, align_idx=0, align_track=clk_xm_tidx_u)
        in_xm_warrs = self.connect_matching_tracks(nandnor_in_list, xm_layer, input_xm_locs, width=tr_w_clk_xm)
        _, nandnor_in_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 4, center_coord=in_xm_warrs[0].middle)
        in_ym_warrs = self.connect_matching_tracks(in_xm_warrs, ym_layer, nandnor_in_ym_locs, width=tr_w_clk_ym)

        # === xm1 layer ===
        xm1_layer = ym_layer + 1

        _, nandnor_in_xm1_locs = \
            tr_manager.place_wires(xm1_layer, ['sig'] * 4,
                                   center_coord=self.get_tile_info(1)[1] + self.get_tile_pinfo(1).height // 2)
        in_xm1_warrs = self.connect_matching_tracks(in_ym_warrs, xm1_layer, nandnor_in_xm1_locs, width=tr_w_clk_ym)

        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, clk_ym.upper, RoundMode.GREATER)
        clkb_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'clk', 'clk', up=1)
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        clk_xm1, clkb_xm1 = self.connect_matching_tracks([clk_ym, clkb_ym], xm1_layer,
                                                         [clk_xm1_tidx, clkb_xm1_tidx], width=tr_w_clk_xm1)

        self.add_pin('inp_db', in_xm1_warrs[0])
        self.add_pin('inn_db', in_xm1_warrs[1])
        self.add_pin('inp', in_xm1_warrs[2])
        self.add_pin('inn', in_xm1_warrs[3])
        self.add_pin('clkp', clk_xm1)
        self.add_pin('clkn', clkb_xm1)
        tot_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols, RoundMode.NEAREST) + \
                      self.arr_info.col_to_track(ym_layer, 0, RoundMode.NEAREST)
        self._output_cols_alignment = \
            [tot_ym_tidx - diff_buf.get_pin('outn', layer=ym_layer).track_id.base_index,
             tot_ym_tidx - diff_buf.get_pin('outp', layer=ym_layer).track_id.base_index]
        for pin in diff_buf.port_names_iter():
            if 'out' in pin or 'mid' in pin:
                self.reexport(diff_buf.get_port(pin))

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        vdd_xm1 = self.export_tap_hm(tr_manager, vdd_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_bot = self.export_tap_hm(tr_manager, vss_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_top = self.export_tap_hm(tr_manager, vss_xm[1], xm_layer, xm1_layer)[0]
        vdd_xm1 = self.extend_wires(vdd_xm1, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm1_bot = self.extend_wires(vss_xm1_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_xm1_top = self.extend_wires(vss_xm1_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', [vss_xm1_bot, vss_xm1_top], label='VSS', show=self.show_pins)

        fill_conn_layer_intv(self, 1, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 1, 1, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 1, stop_col=nand_in_col)
        sch_params_dict = dict(
            nand_params=nand_master.sch_params,
            nor_params=nor_master.sch_params,
            ret_params=[ret_m_master.sch_params, ret_s_master.sch_params],
            buf_params=diff_inv_master.sch_params,
        )
        self.sch_params = sch_params_dict


class ClkLocalRetimerAmp(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._clk_col_alignment = 0
        self._output_cols_alignment = [0, 0]

    @property
    def clk_col_alignment(self):
        return self._clk_col_alignment

    @property
    def output_cols_alignment(self):
        return self._output_cols_alignment

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_amp')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_nonoverlap='',
            seg_ret_s='Number of segments.',
            seg_ret_m='Number of segments.',
            seg_buf='',
            seg_buf_b='',
            seg_mux='',
            seg_nand='',
            seg_nor='',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            sig_locs='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            seg_buf_b=[],
            seg_nonoverlap={},
            sig_locs={},
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        # seg_nonoverlap: int = self.params['seg_nonoverlap']
        seg_ret_s: Dict[str, Any] = self.params['seg_ret_s']
        seg_ret_m: Dict[str, Any] = self.params['seg_ret_m']
        seg_mux: Dict[str, Any] = self.params['seg_mux']
        seg_buf: Dict[str, Any] = self.params['seg_buf']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        ng0_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)
        ng2_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2, tile_idx=1)
        pg_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)

        # Make templates
        wn = self.get_tile_info(1)[0].get_row_place_info(0).row_info.width
        wp = self.get_tile_info(1)[0].get_row_place_info(1).row_info.width
        nd_tidx = get_rel_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=1)
        pd_tidx = get_rel_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1, tile_idx=1)
        mux_params = dict(pinfo=self.get_tile_pinfo(1), seg_dict=seg_mux, vertical_sup=True,
                          ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={})

        buf_params = dict(pinfo=self.get_tile_pinfo(1), seg_list=self.params['seg_buf'], vertical_sup=True,
                          ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={})
        buf_b_params = dict(pinfo=self.get_tile_pinfo(1), seg_list=self.params['seg_buf_b'], vertical_sup=True,
                            ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={})

        sel_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_mux['tri'], vertical_sup=True,
                          ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={})
        ret_m_params = dict(seg_dict=seg_ret_m, pinfo=pinfo)
        ret_s_params = dict(seg_dict=seg_ret_s, pinfo=pinfo)
        nand_params = dict(seg=self.params['seg_nand'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                           sig_locs={'nin1': ng0_tidx, 'nin0': ng2_tidx, 'nout': nd_tidx, 'pout': pd_tidx})
        nor_params = dict(seg=self.params['seg_nor'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                          sig_locs={'nin1': ng0_tidx, 'nin0': ng2_tidx, 'nout': nd_tidx, 'pout': pd_tidx})
        # nonoverlap_params = dict(pinfo=pinfo, seg_nor=seg_nonoverlap['seg_nor'],
        # seg_chain=seg_nonoverlap['seg_chain'], ridx_n=ridx_n, ridx_p=ridx_p, vertical_sup=True, sig_locs={})

        sel_master = self.new_template(InvCore, params=sel_params)
        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        ret_m_master = self.new_template(ClkRetimer, params=ret_m_params)
        ret_s_master = self.new_template(ClkRetimer, params=ret_s_params)
        mux_master = self.new_template(MuxCore, params=mux_params)
        # nonoverlap_master = self.new_template(ClkNonOverlap, params=nonoverlap_params)
        # buf_master = self.new_template(InvChainCore, params=buf_params)
        # buf_b_master = self.new_template(InvChainCore, params=buf_b_params)
        diff_inv_params = dict(pinfo=pinfo, seg_buf=seg_buf)
        diff_inv_master = self.new_template(DiffInvCoupledChainBuf, params=diff_inv_params)

        # Place insts
        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        cur_col = min_sep
        sel_inv = self.add_tile(sel_master, tile_idx=3, col_idx=cur_col)
        cur_col += sel_master.num_cols
        in_vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * 6)
        vm_ncol = self.arr_info.get_column_span(vm_layer, in_vm_ntr)

        in_col = cur_col + vm_ncol // 2
        cur_col += vm_ncol

        muxp = self.add_tile(mux_master, tile_idx=1, col_idx=cur_col)
        muxn = self.add_tile(mux_master, tile_idx=3, col_idx=cur_col)
        cur_col += mux_master.num_cols + min_sep

        nand = self.add_tile(nand_master, tile_idx=1, col_idx=cur_col)
        nor = self.add_tile(nor_master, tile_idx=3, col_idx=cur_col)
        cur_col += max(nand_master.num_cols, nor_master.num_cols)
        cur_col += min_sep
        nand_inv = self.add_tile(sel_master, tile_idx=1, col_idx=cur_col)
        nor_inv = self.add_tile(sel_master, tile_idx=3, col_idx=cur_col)
        cur_col += min_sep + sel_master.num_cols
        cur_col += cur_col & 1

        vss_conn_bot = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_conn_top = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()

        retimer_m = self.add_tile(ret_m_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_m_master.num_cols + min_sep
        retimer_s = self.add_tile(ret_s_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_s_master.num_cols + min_sep

        diffbuf = self.add_tile(diff_inv_master, tile_idx=0, col_idx=cur_col)
        cur_col += min_sep
        self.set_mos_size(self.num_cols + min_sep)

        self.connect_to_track_wires(nor_inv.get_pin('out'), retimer_m.get_pin('inn'))
        self.connect_to_track_wires(nand_inv.get_pin('out'), retimer_m.get_pin('inp'))
        self.connect_to_track_wires(retimer_m.get_pin('outp'), retimer_s.get_pin('inp'))
        self.connect_to_track_wires(retimer_m.get_pin('outn'), retimer_s.get_pin('inn'))
        self.connect_to_track_wires(muxp.get_pin('out'), nand.get_pin('nin<1>'))
        self.connect_to_track_wires(muxn.get_pin('out'), nor.get_pin('nin<1>'))

        # cpmmecy nand nor input
        _, nandnor_in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 6,
                                                       center_coord=self.arr_info.col_to_coord(in_col))
        muxp_in0_vm = self.connect_to_tracks([muxp.get_pin('in<0>')],
                                             TrackID(vm_layer, nandnor_in_vm_locs[0], tr_w_vm),
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
        muxp_in1_vm = self.connect_to_tracks([muxp.get_pin('in<1>')],
                                             TrackID(vm_layer, nandnor_in_vm_locs[1], tr_w_vm),
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)

        muxn_in0_vm = self.connect_to_tracks([muxn.get_pin('in<0>')],
                                             TrackID(vm_layer, nandnor_in_vm_locs[2], tr_w_vm),
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
        muxn_in1_vm = self.connect_to_tracks([muxn.get_pin('in<1>')],
                                             TrackID(vm_layer, nandnor_in_vm_locs[3], tr_w_vm),
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)

        nor_in0_vm = self.connect_to_tracks([nor.get_pin('nin<0>')],
                                            TrackID(vm_layer, nandnor_in_vm_locs[4], tr_w_vm),
                                            min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
        nand_in0_vm = self.connect_to_tracks([nand.get_pin('nin<0>')],
                                             TrackID(vm_layer, nandnor_in_vm_locs[5], tr_w_vm),
                                             min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)

        self.connect_to_track_wires(nand_inv.get_pin('nin'), nand.get_pin('out'))
        self.connect_to_track_wires(nor_inv.get_pin('nin'), nor.get_pin('out'))

        # sel_out
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        sel_out_xm_tidx = self.grid.coord_to_track(xm_layer, sel_inv.get_pin('out').middle, RoundMode.NEAREST)
        sel_in_xm_tidx = sel_out_xm_tidx + self.get_track_sep(xm_layer, tr_w_xm, tr_w_xm)

        sel_in_vm_tidx = tr_manager.get_next_track(vm_layer, sel_inv.get_pin('out').track_id.base_index,
                                                   'sig', 'sig', up=False)
        sel_in_vm = self.connect_to_tracks(sel_inv.get_pin('nin'), TrackID(vm_layer, sel_in_vm_tidx, tr_w_vm),
                                           min_len_mode=MinLenMode.MIDDLE)

        sel_in_xm = self.connect_to_tracks(sel_in_vm, TrackID(xm_layer, sel_in_xm_tidx, tr_w_xm, grid=self.grid))
        self.connect_to_tracks([sel_in_vm, muxn.get_pin('sel'), muxp.get_pin('sel')],
                               TrackID(xm_layer, sel_in_xm_tidx, tr_w_xm, grid=self.grid))
        self.connect_to_tracks([sel_inv.get_pin('out'), muxn.get_pin('selb'), muxp.get_pin('selb')],
                               TrackID(xm_layer, sel_out_xm_tidx, tr_w_xm, grid=self.grid))
        #
        self.connect_to_track_wires(retimer_s.get_pin('outn'), diffbuf.get_pin('inn', layer=xm_layer))
        self.connect_to_track_wires(retimer_s.get_pin('outp'), diffbuf.get_pin('inp', layer=xm_layer))
        #
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_sp_sup_sig_xm = self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sup_xm)
        mid_xm_tidx = self.grid.coord_to_track(xm_layer, self.bound_box.h // 2, RoundMode.NEAREST)
        clk_xm_tidx_u = mid_xm_tidx + 2 * tr_sp_sup_sig_xm
        clk_xm_tidx_l = mid_xm_tidx - 2 * tr_sp_sup_sig_xm
        clk_xm, clkn_xm = self.connect_differential_tracks([retimer_m.get_pin('clkb'), retimer_s.get_pin('clk')],
                                                           [retimer_m.get_pin('clk'), retimer_s.get_pin('clkb')],
                                                           xm_layer, clk_xm_tidx_u, clk_xm_tidx_l,
                                                           width=tr_w_clk_xm)

        # connect input to ym
        ym_layer = xm_layer + 1
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')

        nandnor_in_list = [muxn_in0_vm, muxn_in1_vm, muxp_in0_vm, muxp_in1_vm,
                           nor_in0_vm, nand_in0_vm]
        _, input_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 6, align_idx=-1, align_track=clk_xm_tidx_l)
        in_xm_warrs = self.connect_matching_tracks(nandnor_in_list, xm_layer, input_xm_locs, width=tr_w_clk_xm)
        _, nandnor_in_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 6,
                                                       center_coord=self.arr_info.col_to_coord(in_col))
        in_ym_warrs = self.connect_matching_tracks(in_xm_warrs, ym_layer, nandnor_in_ym_locs, width=tr_w_clk_ym, )

        sig_locs = self.params['sig_locs']
        if 'clk' in sig_locs.keys():
            clk_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols - sig_locs['clk'], RoundMode.NEAREST)
        else:
            clk_ym_tidx = self.grid.coord_to_track(ym_layer, clkn_xm.middle, RoundMode.NEAREST)
        clkb_ym_tidx = tr_manager.get_next_track(ym_layer, clk_ym_tidx, 'clk', 'clk')
        self._clk_col_alignment = self.num_cols - self.arr_info.track_to_col(ym_layer, clk_ym_tidx)
        # clk_ym = self.connect_to_tracks(clk_xm, TrackID(ym_layer, clk_ym_tidx, tr_w_clk_ym),
        #                                 track_upper=self.bound_box.yh)
        # clkb_ym = self.connect_to_tracks(clkn_xm, TrackID(ym_layer, clkb_ym_tidx, tr_w_clk_ym),
        #                                  track_upper=self.bound_box.yh)
        self._clk_col_alignment = self.arr_info.track_to_col(ym_layer, clk_ym_tidx)
        clk_ym, clkb_ym = self.connect_matching_tracks([clk_xm, clkn_xm], ym_layer,
                                                       [clk_ym_tidx, clkb_ym_tidx],
                                                       width=tr_w_clk_ym)

        # === xm1 layer ===
        xm1_layer = ym_layer + 1

        _, nandnor_in_xm1_locs_l = \
            tr_manager.place_wires(xm1_layer, ['sig'] * 3,
                                   center_coord=self.get_tile_info(1)[1] + self.get_tile_pinfo(1).height // 2)
        _, nandnor_in_xm1_locs_u = \
            tr_manager.place_wires(xm1_layer, ['sig'] * 3,
                                   center_coord=self.get_tile_info(3)[1] + self.get_tile_pinfo(3).height // 2)
        nandnor_in_xm1_locs = nandnor_in_xm1_locs_l + nandnor_in_xm1_locs_u
        in_xm1_warrs = self.connect_matching_tracks(in_ym_warrs, xm1_layer, nandnor_in_xm1_locs, width=tr_w_clk_ym)

        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, clk_ym.upper, RoundMode.GREATER)
        clkb_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'clk', 'clk', up=1)
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        clk_xm1, clkb_xm1 = self.connect_matching_tracks([clk_ym, clkb_ym], xm1_layer,
                                                         [clk_xm1_tidx, clkb_xm1_tidx], width=tr_w_clk_xm1)

        vdd_hm = self.connect_to_track_wires(vdd_conn, [retimer_s.get_pin('VDD'), retimer_m.get_pin('VDD')])
        vdd_hm = self.connect_wires(vdd_hm)[0]
        vss_hm_bot = min(retimer_m.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_top = max(retimer_m.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_list = [self.connect_to_track_wires(vss_conn_bot, vss_hm_bot),
                       self.connect_to_track_wires(vss_conn_top, vss_hm_top)]

        self.connect_to_track_wires(muxp.get_all_port_pins('VDD') + muxn.get_all_port_pins('VDD') +
                                    nand.get_all_port_pins('VDD') + nor.get_all_port_pins('VDD') +
                                    nand_inv.get_all_port_pins('VDD') + nor_inv.get_all_port_pins('VDD') +
                                    sel_inv.get_all_port_pins('VDD'),
                                    vdd_hm)
        self.connect_to_track_wires(vss_hm_list[1], muxn.get_all_port_pins('VSS') + nor.get_all_port_pins('VSS') +
                                    sel_inv.get_all_port_pins('VSS') + nor_inv.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vss_hm_list[0], muxp.get_all_port_pins('VSS') + nand.get_all_port_pins('VSS') +
                                    nand_inv.get_all_port_pins('VSS'))
        self.add_pin('inp_d', in_xm1_warrs[2])
        self.add_pin('inp', in_xm1_warrs[3])
        self.add_pin('inn_d', in_xm1_warrs[0])
        self.add_pin('inn', in_xm1_warrs[1])
        self.add_pin('inn_c', in_xm1_warrs[4])
        self.add_pin('inp_c', in_xm1_warrs[5])
        self.add_pin('sel', sel_in_xm)
        self.add_pin('clkp', clk_xm1)
        self.add_pin('clkn', clkb_xm1)
        self.reexport(diffbuf.get_port('outp'), net_name='amp_p')
        self.reexport(diffbuf.get_port('outn'), net_name='amp_n')

        # xm sup
        vdd_hm = self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_bot = self.extend_wires(vss_hm_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_top = self.extend_wires(vss_hm_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]

        vdd_xm = self.export_tap_hm(tr_manager, vdd_hm[0], hm_layer, xm_layer)
        vss_xm = [self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0] for vss in [vss_hm_top, vss_hm_bot]]
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        vdd_xm1 = self.export_tap_hm(tr_manager, vdd_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_bot = self.export_tap_hm(tr_manager, vss_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_top = self.export_tap_hm(tr_manager, vss_xm[1], xm_layer, xm1_layer)[0]
        vdd_xm1 = self.extend_wires(vdd_xm1, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm1_bot = self.extend_wires(vss_xm1_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_xm1_top = self.extend_wires(vss_xm1_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', [vss_xm1_bot, vss_xm1_top], label='VSS', show=self.show_pins)
        sch_params_dict = dict(
            mux_params=mux_master.sch_params,
            nand_params=nand_master.sch_params,
            nor_params=nor_master.sch_params,
            ret_params=[ret_m_master.sch_params, ret_s_master.sch_params],
            sel_params=sel_master.sch_params,
            buf_params=diff_inv_master.sch_params,
        )
        self.sch_params = sch_params_dict
        #


class ClkLocalHoldQuantize(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._clk_col_alignment = 0

    @property
    def clk_col_alignment(self):
        return self._clk_col_alignment

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_hold')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_nand='',
            seg_nand_inv='',
            seg_hold_inv='',
            seg_nor='',
            seg_nor_inv='',
            seg_quan_inv='',
            seg_flop='',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            sig_locs='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            sig_locs={},
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_nor: Dict[str, Any] = self.params['seg_nor']
        seg_nor_inv: Dict[str, Any] = self.params['seg_nor_inv']
        seg_nand: int = self.params['seg_nand']
        seg_nand_inv: int = self.params['seg_nand_inv']
        seg_hold_inv: int = self.params['seg_hold_inv']
        seg_quan_inv: int = self.params['seg_quan_inv']
        seg_flop: int = self.params['seg_flop']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        pg0_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2, tile_idx=1)
        ng0_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=-2, tile_idx=1)
        nand_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_nand, ridx_n=ridx_n,
                           ridx_p=ridx_p, vertical_sup=True, sig_locs={}, resetable=True)
        nor_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_nor, ridx_n=ridx_n,
                          ridx_p=ridx_p, vertical_sup=True, sig_locs={}, resetable=True)
        nand_inv_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_nand_inv, ridx_n=ridx_n,
                               ridx_p=ridx_p, vertical_sup=True, sig_locs={'nin': pg0_tidx}, resetable=True)
        if isinstance(seg_hold_inv, ImmutableList):
            hold_inv_params = dict(pinfo=self.get_tile_pinfo(1), seg_list=seg_hold_inv, ridx_n=ridx_n,
                                   ridx_p=ridx_p, vertical_sup=True, sig_locs={'nin': ng0_tidx}, resetable=True)
            hold_inv_master = self.new_template(InvChainCore, params=hold_inv_params)
        else:
            hold_inv_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_hold_inv, ridx_n=ridx_n,
                                   ridx_p=ridx_p, vertical_sup=True, sig_locs={'nin': ng0_tidx}, resetable=True)
            hold_inv_master = self.new_template(InvCore, params=hold_inv_params)
        nor_inv_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_nor_inv, ridx_n=ridx_n,
                              ridx_p=ridx_p, vertical_sup=True, sig_locs={'nin': pg0_tidx}, resetable=True)
        quan_inv_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_quan_inv, ridx_n=ridx_n,
                               ridx_p=ridx_p, vertical_sup=True, sig_locs={'nin': ng0_tidx}, resetable=True)

        flop_params = dict(pinfo=self.get_tile_pinfo(1), seg=seg_flop, seg_ck=2, w_p=w_p, w_n=w_n, ridx_n=ridx_n,
                           ridx_p=ridx_p, vertical_sup=True, sig_locs={}, resetable=False)

        # Make templates
        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        nor_inv_master = self.new_template(InvCore, params=nor_inv_params)
        nand_inv_master = self.new_template(InvCore, params=nand_inv_params)
        flop_master = self.new_template(FlopCore, params=flop_params)
        quan_inv_master = self.new_template(InvCore, params=quan_inv_params)

        # Place insts
        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        cur_col = min_sep
        nor = self.add_tile(nor_master, tile_idx=1, col_idx=cur_col)
        nand = self.add_tile(nand_master, tile_idx=3, col_idx=cur_col)
        cur_col += max(nor_master.num_cols, nand_master.num_cols) + min_sep
        nor_inv = self.add_tile(nor_inv_master, tile_idx=1, col_idx=cur_col)
        nand_inv = self.add_tile(nand_inv_master, tile_idx=3, col_idx=cur_col)
        cur_col += max(nor_inv_master.num_cols, nand_inv_master.num_cols) + min_sep
        hold_inv = self.add_tile(hold_inv_master, tile_idx=1, col_idx=cur_col)
        quan_inv = self.add_tile(quan_inv_master, tile_idx=3, col_idx=cur_col)

        cur_col = min_sep
        flop0 = self.add_tile(flop_master, tile_idx=5, col_idx=cur_col)
        cur_col += flop_master.num_cols + min_sep
        cur_col += cur_col & 1
        flop1 = self.add_tile(flop_master, tile_idx=5, col_idx=cur_col)

        # Add sub conn
        vss_conn_bot = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_conn_top = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_conn_bot = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()
        vdd_conn_top = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=6).to_warr_list()

        self.set_mos_size(self.num_cols + min_sep)

        vss_hm_bot = self.connect_to_tracks(vss_conn_bot, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0))
        vss_hm_top = self.connect_to_tracks(vss_conn_top, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=4))
        vdd_hm_bot = self.connect_to_tracks(vdd_conn_bot, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2))
        vdd_hm_top = self.connect_to_tracks(vdd_conn_top, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=6))

        vss_hm_list = [self.connect_to_track_wires(vss_conn_bot + nor.get_all_port_pins('VSS') +
                                                   nor_inv.get_all_port_pins('VSS') + hold_inv.get_all_port_pins('VSS'),
                                                   vss_hm_bot),
                       self.connect_to_track_wires(vss_conn_top + nand.get_all_port_pins('VSS') +
                                                   nand_inv.get_all_port_pins('VSS') + quan_inv.get_all_port_pins(
                           'VSS') +
                                                   flop0.get_all_port_pins('VSS') +
                                                   flop1.get_all_port_pins('VSS'), vss_hm_top)]

        inst_list = [nand, nor, nand_inv, nor_inv, hold_inv, quan_inv]
        vdd_conn_list = []
        for inst in inst_list:
            vdd_conn_list.extend(inst.get_all_port_pins('VDD'))
        self.connect_to_track_wires(vdd_conn_list, vdd_hm_bot)

        self.connect_to_track_wires(flop0.get_all_port_pins('VDD') + flop1.get_all_port_pins('VDD'),
                                    vdd_hm_top)

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        # Connect gates
        self.connect_to_track_wires(nor.get_pin('out'), nor_inv.get_pin('nin'))
        self.connect_to_track_wires(nor_inv.get_pin('out'), hold_inv.get_pin('nin'))
        self.connect_to_track_wires(nand.get_pin('out'), nand_inv.get_pin('nin'))
        self.connect_to_track_wires(nand_inv.get_pin('out'), quan_inv.get_pin('nin'))

        nor_xm_tidx = self.grid.coord_to_track(xm_layer, nor.get_pin('out').middle, RoundMode.NEAREST)
        nand_xm_tidx = self.grid.coord_to_track(xm_layer, nand.get_pin('out').middle, RoundMode.NEAREST)

        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        hold_xm = self.connect_to_tracks(hold_inv.get_pin('outb' if isinstance(seg_hold_inv, ImmutableList) else 'out'),
                                         TrackID(xm_layer, nor_xm_tidx, tr_w_clk_xm),
                                         min_len_mode=MinLenMode.MIDDLE)
        quantize_xm = self.connect_to_tracks(quan_inv.get_pin('out'), TrackID(xm_layer, nand_xm_tidx, tr_w_clk_xm),
                                             min_len_mode=MinLenMode.MIDDLE)
        _, h_q_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=hold_xm.middle)
        hold_ym, quantize_ym = self.connect_matching_tracks([hold_xm, quantize_xm], ym_layer,
                                                            h_q_ym_locs, width=tr_manager.get_width(ym_layer, 'clk'))

        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 5, align_idx=-1,
                                               align_track=min(nor.get_pin('out').track_id.base_index,
                                                               nand.get_pin('out').track_id.base_index))
        in_vm_list = self.connect_matching_tracks([nand.get_pin('nin<1>'), nand.get_pin('nin<0>'),
                                                   nor.get_pin('nin<0>'), nor.get_pin('nin<1>')], vm_layer,
                                                  in_vm_locs[:-1], width=tr_w_vm)
        in_vm_list = self.extend_wires(in_vm_list, upper=self.get_tile_info(4)[1], lower=self.bound_box.yl)
        # Connect hold to quan input
        self.connect_to_track_wires(in_vm_list[0], hold_xm)

        _, in_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 3, align_idx=1, align_track=nand_xm_tidx)
        in_xm_warrs = self.connect_matching_tracks(in_vm_list[1:], xm_layer, in_xm_locs, width=tr_w_clk_xm)

        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        _, input_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 4, align_idx=0,
                                                  align_track=self.grid.coord_to_track(ym_layer, 0))
        input_ym = self.connect_matching_tracks(in_xm_warrs, ym_layer, input_ym_locs[1:], width=tr_w_clk_ym,
                                                track_upper=self.bound_box.yh)
        self.add_pin('sam_d', in_xm_warrs[1])
        self.add_pin('sam', in_xm_warrs[-1])
        self.add_pin('amp_b', in_xm_warrs[0])

        self.add_pin('sam_d', input_ym[1])
        self.add_pin('sam', input_ym[-1])
        self.add_pin('amp_b', input_ym[0])

        self.add_pin('hold', hold_ym)
        self.add_pin('quantize', quantize_ym)

        # Conenct rst synchronizer
        clk_xm_tidx = self.grid.coord_to_track(xm_layer, flop0.get_pin('clk').middle, RoundMode.NEAREST)
        clk_xm = self.connect_to_tracks([flop0.get_pin('clk'), flop1.get_pin('clk')],
                                        TrackID(xm_layer, clk_xm_tidx, tr_w_clk_xm))

        sig_locs = self.params['sig_locs']
        if 'clk' in sig_locs.keys():
            clk_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols - sig_locs['clk'], RoundMode.NEAREST)
        else:
            clk_ym_tidx = self.grid.coord_to_track(ym_layer, clk_xm.middle, RoundMode.NEAREST)
        self._clk_col_alignment = self.num_cols - self.arr_info.track_to_col(ym_layer, clk_ym_tidx)
        clk_ym = self.connect_to_tracks(clk_xm, TrackID(ym_layer, clk_ym_tidx, tr_w_clk_ym),
                                        track_upper=self.bound_box.yh)
        clkb_dummy_ym_tidx = tr_manager.get_next_track(ym_layer, clk_ym_tidx, 'clk', 'clk')
        self.add_wires(ym_layer, clkb_dummy_ym_tidx, clk_ym.lower, clk_ym.upper,
                       width=tr_manager.get_width(ym_layer, 'clk'))

        self.connect_to_track_wires(flop0.get_pin('out'), flop1.get_pin('nin'))

        rst_out_xm_tidx = self.grid.coord_to_track(xm_layer, flop1.get_pin('out').middle, RoundMode.NEAREST)
        rst_out_xm = self.connect_to_tracks(flop1.get_pin('out'), TrackID(xm_layer, rst_out_xm_tidx, tr_w_clk_ym))
        rst_out_ym_tidx = self.grid.coord_to_track(ym_layer, rst_out_xm.upper, RoundMode.NEAREST)
        rst_out_ym = self.connect_to_tracks(rst_out_xm, TrackID(ym_layer, rst_out_ym_tidx, tr_w_clk_ym),
                                            track_upper=self.bound_box.yh, track_lower=self.bound_box.yl)

        in_vm_tidx = tr_manager.get_next_track(vm_layer, flop0.get_pin('clk').track_id.base_index,
                                               'sig', 'sig', up=False)
        rst_in_vm = self.connect_to_tracks(flop0.get_pin('nin'), TrackID(vm_layer, in_vm_tidx, tr_w_vm),
                                           min_len_mode=MinLenMode.MIDDLE)
        rst_in_xm_tidx = self.grid.coord_to_track(xm_layer, rst_in_vm.middle, RoundMode.NEAREST)
        rst_in_xm = self.connect_to_tracks(rst_in_vm, TrackID(xm_layer, rst_in_xm_tidx, tr_w_clk_xm),
                                           min_len_mode=MinLenMode.MIDDLE)
        rst_in_ym = self.connect_to_tracks(rst_in_xm, TrackID(ym_layer, input_ym_locs[0], tr_w_clk_ym),
                                           min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.yh)

        self.add_pin('rst_in', rst_in_ym)
        self.add_pin('clk', clk_ym)
        self.add_pin('clk_xm', clk_xm)
        self.add_pin('rst_out', rst_out_ym)

        # vm sup
        vdd_bot_vm_list, vdd_top_vm_list, vss_bot_vm_list, vss_top_vm_list = [], [], [], []
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        # sup_vm_cols = list(range(0, self.num_cols))[::2]
        sup_vm_cols = self.get_available_tracks(vm_layer, self.arr_info.col_to_track(vm_layer, 0),
                                                self.arr_info.col_to_track(vm_layer, self.num_cols),
                                                lower=self.bound_box.yl,
                                                upper=self.bound_box.yh, width=tr_manager.get_width(vm_layer, 'sup'),
                                                sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')),
                                                sep_margin=HalfInt(1), include_last=True)
        for c in sup_vm_cols:
            vdd_bot_vm_list.append(self.connect_to_tracks(vdd_hm_bot, TrackID(vm_layer, c, tr_w_sup_vm)))
            vdd_top_vm_list.append(self.connect_to_tracks(vdd_hm_top, TrackID(vm_layer, c, tr_w_sup_vm)))
            vss_bot_vm_list.append(self.connect_to_tracks(vss_hm_list[0], TrackID(vm_layer, c, tr_w_sup_vm)))
            vss_top_vm_list.append(self.connect_to_tracks(vss_hm_list[1], TrackID(vm_layer, c, tr_w_sup_vm)))

        # xm sup
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        vdd_bot_xm_coord = self.grid.track_to_coord(hm_layer, vdd_hm_bot.track_id.base_index)
        vdd_bot_xm_tid = self.grid.coord_to_track(xm_layer, vdd_bot_xm_coord, RoundMode.NEAREST)
        vdd_bot_xm = self.connect_to_tracks(vdd_bot_vm_list, TrackID(xm_layer, vdd_bot_xm_tid, tr_w_sup_xm),
                                            track_lower=vss_hm_bot.lower, track_upper=vss_hm_bot.upper)
        vdd_top_xm_coord = self.grid.track_to_coord(hm_layer, vdd_hm_top.track_id.base_index)
        vdd_top_xm_tid = self.grid.coord_to_track(xm_layer, vdd_top_xm_coord, RoundMode.NEAREST)
        vdd_top_xm = self.connect_to_tracks(vdd_top_vm_list, TrackID(xm_layer, vdd_top_xm_tid, tr_w_sup_xm),
                                            track_lower=vss_hm_top.lower, track_upper=vss_hm_top.upper)
        vss_bot_xm_coord = self.grid.track_to_coord(hm_layer, vss_hm_list[0].track_id.base_index)
        vss_bot_xm_tid = self.grid.coord_to_track(xm_layer, vss_bot_xm_coord, RoundMode.NEAREST)
        vss_bot_xm = self.connect_to_tracks(vss_bot_vm_list, TrackID(xm_layer, vss_bot_xm_tid, tr_w_sup_xm),
                                            track_lower=vss_hm_list[0].lower, track_upper=vss_hm_list[0].upper)
        vss_top_xm_coord = self.grid.track_to_coord(hm_layer, vss_hm_list[1].track_id.base_index)
        vss_top_xm_tid = self.grid.coord_to_track(xm_layer, vss_top_xm_coord, RoundMode.NEAREST)
        vss_top_xm = self.connect_to_tracks(vss_top_vm_list, TrackID(xm_layer, vss_top_xm_tid, tr_w_sup_xm),
                                            track_lower=vss_hm_list[1].lower, track_upper=vss_hm_list[1].upper)
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        ym_tid_l = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.GREATER_EQ)
        ym_tid_r = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.LESS_EQ)
        num_ym_sup = tr_manager.get_num_wires_between(ym_layer, 'dum', ym_tid_l, 'dum', ym_tid_r, 'sup')

        ym_sup_tidxs = self.get_available_tracks(ym_layer, ym_tid_l, ym_tid_r, self.bound_box.yl,
                                                 self.bound_box.yh, tr_w_sup_ym,
                                                 sep=tr_manager.get_sep(ym_layer, ('sup', 'sup')),
                                                 sep_margin=tr_manager.get_sep(ym_layer, ('sup', 'sup')))
        ym_sup_tidxs = ym_sup_tidxs[1:-1]

        vdd_ym = [self.connect_to_tracks([vdd_bot_xm, vdd_top_xm], TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[::2]]
        vss_ym = [self.connect_to_tracks([vss_bot_xm, vss_top_xm], TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[1::2]]
        xm1_tidx_list = \
            [self.grid.coord_to_track(xm1_layer, self.grid.track_to_coord(xm1_layer, vss_bot_xm.track_id.base_index)),
             self.grid.coord_to_track(xm1_layer, self.grid.track_to_coord(xm1_layer, vdd_bot_xm.track_id.base_index)),
             self.grid.coord_to_track(xm1_layer, self.grid.track_to_coord(xm1_layer, vss_top_xm.track_id.base_index)),
             self.grid.coord_to_track(xm1_layer, self.grid.track_to_coord(xm1_layer, vdd_top_xm.track_id.base_index))]
        [vss_xm1_bot, vdd_xm1_bot, vss_xm1_top, vdd_xm1_top] = \
            self.connect_matching_tracks([vss_ym, vdd_ym, vss_ym, vdd_ym], xm1_layer, xm1_tidx_list, width=tr_w_sup_xm1)
        self.add_pin('VDD_ym', vdd_ym, label='VDD', show=self.show_pins)
        self.add_pin('VSS_ym', vss_ym, label='VSS', show=self.show_pins)
        self.add_pin('VDD_xm1', [vdd_xm1_bot, vdd_xm1_top], label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', [vss_xm1_bot, vss_xm1_top], label='VSS', show=self.show_pins)
        sch_params_dict = dict(
            ff=flop_master.sch_params,
            nor=nor_master.sch_params,
            nand=nand_master.sch_params,
            nor_inv=nor_inv_master.sch_params,
            hold_inv=hold_inv_master.sch_params,
            quan_inv=quan_inv_master.sch_params,
            nand_inv=nand_inv_master.sch_params,
            hold_inv_chain=isinstance(seg_hold_inv, ImmutableList)
        )
        self.sch_params = sch_params_dict


class ClkLocalTop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_phase_gen')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            clk_local_params='Parameters for buffer x',
            clk_local_vco_params='',
            clk_local_amp_params='',
        )

    def draw_layout(self) -> None:
        clk_local_params: Param = self.params['clk_local_params']
        clk_local_vco_params: Param = self.params['clk_local_vco_params']
        clk_local_amp_params: Param = self.params['clk_local_amp_params']

        if isinstance(clk_local_params, str):
            spec_yaml = read_yaml(clk_local_params)
            clk_local_params = spec_yaml['params']
        if isinstance(clk_local_vco_params, str):
            spec_yaml = read_yaml(clk_local_vco_params)
            clk_local_vco_params = spec_yaml['params']
        if isinstance(clk_local_amp_params, str):
            spec_yaml = read_yaml(clk_local_amp_params)
            clk_local_amp_params = spec_yaml['params']

        for params in [clk_local_params, clk_local_vco_params, clk_local_amp_params]:
            params['export_private'] = False
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        clk_local_sam_master = self.new_template(GenericWrapper, params=clk_local_params)
        clk_local_vco_params['params']['sig_locs'] = {'clk': clk_local_sam_master.core.clk_col_alignment}
        clk_local_amp_params['params']['sig_locs'] = {'clk': clk_local_sam_master.core.clk_col_alignment}
        clk_local_vco_master = self.new_template(GenericWrapper, params=clk_local_vco_params)
        clk_local_amp_master = self.new_template(GenericWrapper, params=clk_local_amp_params)

        # --- Placement --- #

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      clk_local_params['params']['pinfo']['tile_specs']
                                                      ['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        top_layer = max(clk_local_amp_master.top_layer, clk_local_sam_master.top_layer, clk_local_vco_master.top_layer,
                        ym1_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        sam_w, sam_h = clk_local_sam_master.bound_box.w, clk_local_sam_master.bound_box.h
        vco_w, vco_h = clk_local_vco_master.bound_box.w, clk_local_vco_master.bound_box.h
        amp_w, amp_h = clk_local_amp_master.bound_box.w, clk_local_amp_master.bound_box.h

        local_y = 0
        local_x = clk_local_sam_master.bound_box.xh
        local = self.add_instance(clk_local_sam_master, xform=Transform(local_x, local_y, Orientation.MY))
        vco_y = -(-local.bound_box.yh // blk_h) * blk_h
        vco_x = clk_local_vco_master.bound_box.xh
        vco = self.add_instance(clk_local_vco_master, xform=Transform(vco_x, vco_y, Orientation.MY))
        amp_y = -(-vco.bound_box.yh // blk_h) * blk_h
        amp_x = clk_local_amp_master.bound_box.xh
        amp = self.add_instance(clk_local_amp_master, xform=Transform(amp_x, amp_y, Orientation.MY))
        hold_y = -(-amp.bound_box.yh // blk_h) * blk_h

        tot_w = max(amp_w, vco_w, sam_w)
        tot_w = -(-tot_w // blk_w) * blk_w
        tot_h = -(-amp.bound_box.yh // blk_h) * blk_h

        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))

        # = Connection =
        # Connect clock
        clkp_xm1 = self.connect_wires([inst.get_pin('clkp') for inst in [local, vco, amp]])
        clkn_xm1 = self.connect_wires([inst.get_pin('clkn') for inst in [local, vco, amp]])
        clk_ym1_mid_tidx = self.grid.coord_to_track(ym1_layer, clkp_xm1[0].middle, RoundMode.NEAREST)
        tr_w_clk_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        tr_sp_clk_ym1 = self.get_track_sep(ym1_layer, tr_w_clk_ym1, tr_w_clk_ym1).div2(round_up=True)
        clk_ym1_tidx_n = clk_ym1_mid_tidx + tr_sp_clk_ym1
        clk_ym1_tidx_p = clk_ym1_mid_tidx - tr_sp_clk_ym1
        clkn_ym1, clkp_ym1 = \
            self.connect_to_tracks(clkn_xm1, TrackID(ym1_layer, clk_ym1_tidx_n, tr_w_clk_ym1, grid=self.grid)), \
            self.connect_to_tracks(clkp_xm1, TrackID(ym1_layer, clk_ym1_tidx_p, tr_w_clk_ym1, grid=self.grid))
        sam_n_xm2 = self.via_stack_up(tr_manager, local.get_pin('outn', layer=xm1_layer), xm1_layer, xm1_layer + 1,
                                      'sig')
        sam_p_xm2 = self.via_stack_up(tr_manager, local.get_pin('outp', layer=xm1_layer), xm1_layer, xm1_layer + 1,
                                      'sig')
        vco_n_xm2 = self.via_stack_up(tr_manager, vco.get_pin('outn', layer=xm1_layer), xm1_layer, xm1_layer + 1, 'sig')
        vco_p_xm2 = self.via_stack_up(tr_manager, vco.get_pin('outp', layer=xm1_layer), xm1_layer, xm1_layer + 1, 'sig')
        amp_n_xm2 = self.via_stack_up(tr_manager, amp.get_pin('amp_n', layer=xm1_layer), xm1_layer, xm1_layer + 1,
                                      'sig')
        amp_p_xm2 = self.via_stack_up(tr_manager, amp.get_pin('amp_p', layer=xm1_layer), xm1_layer, xm1_layer + 1,
                                      'sig')
        out_list = []
        tr_w_sig_xm2 = tr_manager.get_width(xm1_layer+2, 'sig')
        for via_stack in [sam_n_xm2, sam_p_xm2, vco_n_xm2, vco_p_xm2, amp_n_xm2, amp_p_xm2]:
            tidx = self.grid.coord_to_track(xm1_layer+2, via_stack[xm1_layer+1].middle, RoundMode.NEAREST)
            out_list.append(self.connect_to_tracks(via_stack[xm1_layer+1],
                                                   TrackID(xm1_layer+2, tidx, tr_w_sig_xm2)))
        #
        ph_ym1_tidx = self.grid.coord_to_track(ym1_layer, self.bound_box.xh, RoundMode.NEAREST)
        ph_ym1_locs = [ph_ym1_tidx + 2 * idx * tr_sp_clk_ym1 for idx in range(8)]
        tr_w_clk_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        ph_ym_ins = [self.connect_to_tracks([local.get_pin('inn'), amp.get_pin('inn')],
                                            TrackID(ym1_layer, ph_ym1_locs[0], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([local.get_pin('inp'), amp.get_pin('inp')],
                                            TrackID(ym1_layer, ph_ym1_locs[1], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([local.get_pin('inp_db'), amp.get_pin('inn_d')],
                                            TrackID(ym1_layer, ph_ym1_locs[2], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([local.get_pin('inn_db'), amp.get_pin('inp_d')],
                                            TrackID(ym1_layer, ph_ym1_locs[3], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([vco.get_pin('inn')],
                                            TrackID(ym1_layer, ph_ym1_locs[4], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([vco.get_pin('inp')],
                                            TrackID(ym1_layer, ph_ym1_locs[5], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([vco.get_pin('inp_db'), amp.get_pin('inn_c')],
                                            TrackID(ym1_layer, ph_ym1_locs[6], tr_w_clk_ym1, grid=self.grid)),
                     self.connect_to_tracks([vco.get_pin('inn_db'), amp.get_pin('inp_c')],
                                            TrackID(ym1_layer, ph_ym1_locs[7], tr_w_clk_ym1, grid=self.grid)),
                     ]
        ph_ym_ins = self.extend_wires(ph_ym_ins, upper=self.bound_box.yh, lower=self.bound_box.yl)

        for pinname, pin in zip(['sam_n', 'sam_p', 'vco_n', 'vco_p', 'amp_n', 'amp_p'],out_list):
            self.add_pin(pinname, self.extend_wires(pin, lower=self.bound_box.xl))
        self.add_pin('ph_sam', ph_ym_ins[0], mode=PinMode.LOWER)
        self.add_pin('ph_sam_b', ph_ym_ins[1], mode=PinMode.LOWER)
        self.add_pin('ph_sam_d', ph_ym_ins[2], mode=PinMode.LOWER)
        self.add_pin('ph_sam_db', ph_ym_ins[3], mode=PinMode.LOWER)
        self.add_pin('ph_vco', ph_ym_ins[4], mode=PinMode.LOWER)
        self.add_pin('ph_vco_b', ph_ym_ins[5], mode=PinMode.LOWER)
        self.add_pin('ph_vco_d', ph_ym_ins[6], mode=PinMode.LOWER)
        self.add_pin('ph_vco_db', ph_ym_ins[7], mode=PinMode.LOWER)

        self.add_pin('ctrl_amp', amp.get_pin('sel'))
        self.add_pin('clkp', self.extend_wires(clkp_ym1, upper=self.bound_box.yh))
        self.add_pin('clkn', self.extend_wires(clkn_ym1, upper=self.bound_box.yh))

        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        tr_sp_sup_ym1 = tr_manager.get_sep(ym1_layer, ('sup', 'sup'))
        ym1_tid_l = self.grid.coord_to_track(ym1_layer, self.bound_box.xl, mode=RoundMode.NEAREST)
        ym1_tid_r = self.grid.coord_to_track(ym1_layer, self.bound_box.xh, mode=RoundMode.LESS)
        ym1_sup_locs = self.get_available_tracks(ym1_layer, ym1_tid_l, ym1_tid_r,
                                                 self.bound_box.yl, self.bound_box.yh, width=tr_w_sup_ym1,
                                                 sep=tr_sp_sup_ym1, include_last=True)
        vdd_ym1_list, vss_ym1_list = [], []
        # vdd vss at xm1_layer
        vdd_xm1_list = [vdd for inst in [vco, amp, local] for vdd in inst.get_all_port_pins('VDD_xm1')]
        vss_xm1_list = [vss for inst in [vco, amp, local] for vss in inst.get_all_port_pins('VSS_xm1')]

        vdd_xm1_list = self.extend_wires(vdd_xm1_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm1_list = self.extend_wires(vss_xm1_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        ym1_vdd_locs = ym1_sup_locs[::2]
        ym1_vss_locs = ym1_sup_locs[1::2]
        vdd_ym1_list.extend([self.connect_to_tracks(vdd_xm1_list, TrackID(ym1_layer, tidx, tr_w_sup_ym1)) for tidx in
                             ym1_vdd_locs])
        vss_ym1_list.extend([self.connect_to_tracks(vss_xm1_list, TrackID(ym1_layer, tidx, tr_w_sup_ym1)) for tidx in
                             ym1_vss_locs])

        vdd_ym1_list = self.extend_wires(vdd_ym1_list, upper=self.bound_box.yh, lower=self.bound_box.yl)
        vss_ym1_list = self.extend_wires(vss_ym1_list, upper=self.bound_box.yh, lower=self.bound_box.yl)
        self.add_pin('VDD_ym1', vdd_ym1_list, label='VDD', connect=True)
        self.add_pin('VSS_ym1', vss_ym1_list, label='VSS', connect=True)

        vss_list, vdd_list = self.connect_supply_stack_warr(tr_manager, [vss_ym1_list, vdd_ym1_list], ym1_layer, ym1_layer + 3,
                                       bbox_list=BBox(out_list[0].bound_box.xh, 0,
                                                 self.bound_box.xh, self.bound_box.yh))
        self.add_pin('VSS', vss_list[ym1_layer + 3])
        self.add_pin('VDD', vdd_list[ym1_layer + 3])

        self.sch_params = dict(
            sam=clk_local_sam_master.sch_params,
            vco=clk_local_vco_master.sch_params,
            amp=clk_local_amp_master.sch_params,
        )


class ClkGlobalDivPhase(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_global_div_ph')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            clk_phase_params='',
            clk_div_params='',
        )

    def draw_layout(self) -> None:
        clk_phase_params: Param = self.params['clk_phase_params']
        clk_div_params: Param = self.params['clk_div_params']

        if isinstance(clk_phase_params, str):
            spec_yaml = read_yaml(clk_phase_params)
            clk_phase_params = spec_yaml['params']
        if isinstance(clk_div_params, str):
            spec_yaml = read_yaml(clk_div_params)
            clk_div_params = spec_yaml['params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        # self._tr_manager = tr_manager

        clk_div_master = self.new_template(IntegrationWrapper, params=clk_div_params)
        clk_phase_master = self.new_template(ClkLocalTop, params=clk_phase_params)

        top_layer = max(clk_div_master.top_layer, clk_phase_master.top_layer)

        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        div = self.add_instance(clk_div_master, xform=Transform(0, 0, Orientation.R0))
        ph_y = -(-div.bound_box.yh//blk_h)*blk_h + 5*blk_h
        ph = self.add_instance(clk_phase_master, xform=Transform(0, ph_y, Orientation.R0))

        tot_h = -(-ph.bound_box.yh//blk_h)*blk_h
        tot_w = -(-max(div.bound_box.xh, ph.bound_box.xh)//blk_w)*blk_w
        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))

        for pins in div.port_names_iter():
            self.reexport(div.get_port(pins))
        for pins in ph.port_names_iter():
            self.reexport(ph.get_port(pins))

        self._sch_params=dict(div_params=clk_div_master.sch_params, ph_params=clk_phase_master.sch_params)

