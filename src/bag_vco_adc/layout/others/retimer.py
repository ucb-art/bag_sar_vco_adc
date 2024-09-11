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
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableList
from pybag.core import BBox, Transform
from pybag.enum import RoundMode, MinLenMode, PinMode, Orientation
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from ..digital import NAND2Core, InvCore, LatchCore, InvChainCore
from ..sar.sar_logic import SARRetUnit
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID
from ..util.util import fill_tap, export_xm_sup
from ..util.wrapper import IntegrationWrapper
from ..vco.vco_cnter_dec import MuxCore


class NANDSRCoreVert(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    # @classmethod
    # def get_schematic_class(cls) -> Optional[Type[Module]]:
    #     # noinspection PyTypeChecker
    #     return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_retime_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
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
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_nand = seg_dict['nand']
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
        pg1_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2, tile_idx=1)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)
        pinfo = self.get_tile_pinfo(1)
        nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           sig_locs={'nin0': pg0_tidx, 'nin1': pg1_tidx}, vertical_sup=True, vertical_out=False)
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_out=False, sig_locs={'nin': ng1_tidx}, vertical_sup=True)
        nand_master = self.new_template(NAND2Core, params=nand_params)
        inv_master = self.new_template(InvCore, params=inv_params)

        cur_col = 0
        nands = self.add_tile(nand_master, tile_idx=1, col_idx=cur_col)
        nandr = self.add_tile(nand_master, tile_idx=3, col_idx=cur_col)

        cur_col += nand_master.num_cols + min_sep
        invs = self.add_tile(inv_master, tile_idx=1, col_idx=cur_col)
        invr = self.add_tile(inv_master, tile_idx=3, col_idx=cur_col)
        self.set_mos_size()

        _, out_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 2,
                                                center_coord=(nands.bound_box.xl + nands.bound_box.xh) // 2)

        outs_n_vm, outr_n_vm = self.connect_matching_tracks(
            [[nands.get_pin('nout'), nands.get_pin('pout'), invr.get_pin('nin')],
             [nandr.get_pin('nout'), nandr.get_pin('pout'), invs.get_pin('nin')]],
            vm_layer, out_vm_locs, width=tr_manager.get_width(vm_layer, 'sig'))

        self.connect_to_track_wires(outs_n_vm, nandr.get_pin('nin<1>'))
        self.connect_to_track_wires(outr_n_vm, nands.get_pin('nin<1>'))

        out_vm_tidx = self.grid.coord_to_track(vm_layer, (invr.bound_box.xl + invr.bound_box.xh) // 2,
                                               RoundMode.NEAREST)
        outs_vm = self.connect_to_tracks([invs.get_pin('pout'), invs.get_pin('pout')],
                                         TrackID(vm_layer, out_vm_tidx, tr_w_vm))
        outr_vm = self.connect_to_tracks([invr.get_pin('pout'), invr.get_pin('pout')],
                                         TrackID(vm_layer, out_vm_tidx, tr_w_vm))
        #
        self.add_pin('q', outs_vm)
        self.add_pin('qb', outr_vm)
        self.add_pin('s', nands.get_pin('nin<0>'))
        self.add_pin('r', nandr.get_pin('nin<0>'))
        #
        vdd_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=2).to_warr_list()
        vss_bot_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=0).to_warr_list()
        vss_top_sub_conn = self.add_substrate_contact(0, 0, seg=self.num_cols, tile_idx=4).to_warr_list()
        vdd_list, vss_list = [], []
        for inst in [nands, nandr, invr, invs]:
            vdd_list.append(inst.get_pin('VDD'))

        vdd_hm_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=2)
        vss_hm_bot_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=0)
        vss_hm_top_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=4)

        vss_hm_list = \
            [self.connect_to_tracks(invs.get_all_port_pins('VSS') + nands.get_all_port_pins('VSS') + vss_bot_sub_conn,
                                    vss_hm_bot_tid),
             self.connect_to_tracks(invr.get_all_port_pins('VSS') + nandr.get_all_port_pins('VSS') + vss_top_sub_conn,
                                    vss_hm_top_tid)]
        vdd_hm = self.connect_to_tracks(vdd_list + vdd_sub_conn, vdd_hm_tid)

        self.add_pin('VDD', vdd_hm, connect=True)
        self.add_pin('VSS', vss_hm_list, connect=True)

        sch_params_dict = dict(
            inv=inv_master.sch_params,
            nand=nand_master.sch_params,
        )
        self.sch_params = sch_params_dict


class NANDSRCore(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sr_nand')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
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
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_nand = seg_dict['nand']
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

        ng0_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=0)
        ng1_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=0)
        ng2_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2, tile_idx=0)
        pg0_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=0)
        pg1_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2, tile_idx=0)
        pg2_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-3, tile_idx=0)
        pd_tidx = get_rel_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1, tile_idx=0)
        nd_tidx = get_rel_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=0)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)
        pinfo = self.get_tile_pinfo(1)
        nands_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            sig_locs={'nin0': pg0_tidx, 'nin1': pg1_tidx, 'pout': pd_tidx, 'nout': nd_tidx},
                            vertical_sup=False, vertical_out=True)
        nandr_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            sig_locs={'nin0': ng0_tidx, 'nin1': ng1_tidx, 'pout': pd_tidx, 'nout': nd_tidx},
                            vertical_sup=False, vertical_out=True)
        invs_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_out=False, sig_locs={'nin': ng0_tidx}, vertical_sup=False)
        invr_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_out=False, sig_locs={'nin': pg0_tidx}, vertical_sup=False)
        nands_master = self.new_template(NAND2Core, params=nands_params)
        invs_master = self.new_template(InvCore, params=invs_params)
        nandr_master = self.new_template(NAND2Core, params=nandr_params)
        invr_master = self.new_template(InvCore, params=invr_params)

        cur_col = 0
        nands = self.add_tile(nands_master, tile_idx=0, col_idx=cur_col)
        cur_col += nands_master.num_cols + min_sep
        cur_col += cur_col & 1
        nandr = self.add_tile(nandr_master, tile_idx=0, col_idx=cur_col)
        cur_col += nandr_master.num_cols + min_sep
        cur_col += cur_col & 1
        invs = self.add_tile(invs_master, tile_idx=0, col_idx=cur_col)
        cur_col += invs_master.num_cols
        cur_col += cur_col & 1
        invr = self.add_tile(invr_master, tile_idx=0, col_idx=cur_col)
        self.set_mos_size()

        # input
        s_in_vm_tidx = self.grid.coord_to_track(vm_layer, nands.get_pin('nin<1>').lower, RoundMode.GREATER_EQ)
        r_in_vm_tidx = self.grid.coord_to_track(vm_layer, nandr.get_pin('nin<1>').lower, RoundMode.GREATER_EQ)

        s_in_vm = self.connect_to_tracks(nands.get_pin('nin<1>'), TrackID(vm_layer, s_in_vm_tidx, tr_w_vm),
                                         min_len_mode=MinLenMode.MIDDLE)
        r_in_vm = self.connect_to_tracks(nandr.get_pin('nin<1>'), TrackID(vm_layer, r_in_vm_tidx, tr_w_vm),
                                         min_len_mode=MinLenMode.MIDDLE)

        self.connect_differential_wires([nandr.get_pin('nin<0>'), invs.get_pin('nin')],
                                        [nands.get_pin('nin<0>'), invr.get_pin('nin')],
                                        nands.get_pin('out'), nandr.get_pin('out'))
        #
        out_vm_tidx = self.grid.coord_to_track(vm_layer, (invs.bound_box.xl + invs.bound_box.xh) // 2,
                                               RoundMode.NEAREST)
        outs_vm = self.connect_to_tracks([invs.get_pin('pout'), invs.get_pin('nout')],
                                         TrackID(vm_layer, out_vm_tidx, tr_w_vm))
        out_vm_tidx = self.grid.coord_to_track(vm_layer, (invr.bound_box.xl + invr.bound_box.xh) // 2,
                                               RoundMode.NEAREST)
        outr_vm = self.connect_to_tracks([invr.get_pin('pout'), invr.get_pin('nout')],
                                         TrackID(vm_layer, out_vm_tidx, tr_w_vm))

        self.connect_to_track_wires(invr.get_pin('nin'), nandr.get_pin('out'))
        self.connect_to_track_wires(invs.get_pin('nin'), nands.get_pin('out'))

        vdd = nandr.get_all_port_pins('VDD') + nands.get_all_port_pins('VDD') + invr.get_all_port_pins(
            'VDD') + invs.get_all_port_pins('VDD')
        vss = nandr.get_all_port_pins('VSS') + nands.get_all_port_pins('VSS') + invr.get_all_port_pins(
            'VSS') + invs.get_all_port_pins('VSS')
        vdd = self.connect_wires(vdd)
        vss = self.connect_wires(vss)

        self.add_pin('s', s_in_vm)
        self.add_pin('r', r_in_vm)
        self.add_pin('q', outs_vm)
        self.add_pin('qb', outr_vm)

        self.add_pin('VDD', vdd)
        self.add_pin('VSS', vss)

        self.add_pin('s', s_in_vm)
        self.add_pin('r', r_in_vm)
        sch_params_dict = dict(
            inv=invr_master.sch_params,
            nand=nandr_master.sch_params,
        )
        self.sch_params = sch_params_dict
        #


class RetimerChannel(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'retimer_channel')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            pinfo='The MOSBasePlaceInfo object.',
            nbits='',
            ncol_tot='',
            seg_dict='',
            w_dict='',
            ridx_p='',
            ridx_n=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            tr_widths={},
            tr_spaces={},
            w_dict={'wn': 4, 'wp': 4},
            ridx_n=0,
            ridx_p=-1,
            ncol_tot=0,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_dict = self.params['w_dict']
        w_n: int = w_dict['w_n']
        w_p: int = w_dict['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        ncol_tot: int = self.params['ncol_tot']
        seg_latch: int = seg_dict['latch']
        seg_buf: int = seg_dict['buf']
        seg_out_buf: int = seg_dict['out_buf']
        seg_inv: int = seg_dict['inv']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = TrackManager(self.grid, self.params['tr_widths'], self.params['tr_spaces'])

        def get_rel_track_index(row_idx, wire_type, wire_name, wire_idx, tile_idx):
            yb = self.get_tile_info(tile_idx)[1]
            abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                            tile_idx=tile_idx)
            abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
            rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
            return rel_tidx

        def insert_tap(tile_idx, col_tidx, seg=2):
            tot_ncol = 2 * self.sub_sep_col + seg
            ptap = self.add_substrate_contact(0, col_tidx + self.sub_sep_col, seg=2, tile_idx=tile_idx)
            ntap = self.add_substrate_contact(1, col_tidx + self.sub_sep_col, seg=2, tile_idx=tile_idx)
            vss_hm = self.connect_to_tracks(ptap, self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=tile_idx))
            vdd_hm = self.connect_to_tracks(ntap, self.get_track_id(1, MOSWireType.DS, 'sup', tile_idx=tile_idx))
            mid_coord = int(self.arr_info.col_to_coord(col_tidx + tot_ncol / 2))
            _, vm_locs = tr_manager.place_wires(vm_layer, ['sup'] * 2, center_coord=mid_coord)
            vss_vm, vdd_vm = self.connect_matching_tracks([vss_hm, vdd_hm], vm_layer, vm_locs,
                                                          width=tr_manager.get_width(vm_layer, 'sup'))
            vss_coord = self.grid.track_to_coord(hm_layer, vss_hm.track_id.base_index)
            vss_xm_tid = self.grid.coord_to_track(xm_layer, vss_coord, RoundMode.NEAREST)
            vdd_coord = self.grid.track_to_coord(hm_layer, vdd_hm.track_id.base_index)
            vdd_xm_tid = self.grid.coord_to_track(xm_layer, vdd_coord, RoundMode.NEAREST)
            vss_xm, vdd_xm = self.connect_matching_tracks([vss_vm, vdd_vm], xm_layer, [vss_xm_tid, vdd_xm_tid],
                                                          width=tr_manager.get_width(xm_layer, 'sup'))
            return [vss_hm, vdd_hm], [vss_vm, vdd_vm], [vss_xm, vdd_xm], tot_ncol

        ng0_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=0)
        ng1_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1, tile_idx=0)
        ng2_tidx = get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2, tile_idx=0)
        pg0_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=0)
        pg1_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2, tile_idx=0)
        pg2_tidx = get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-3, tile_idx=0)
        pd_tidx = get_rel_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1, tile_idx=0)
        nd_tidx = get_rel_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=0)

        pinfo = self.get_tile_pinfo(1)
        w_n_latch, w_p_latch = w_dict['w_n_latch'], w_dict['w_p_latch']
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False, sig_locs={})
        buf_params = dict(pinfo=pinfo, seg=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False, sig_locs={}, tr_manager=tr_manager)
        out_buf_params = dict(pinfo=pinfo, seg_list=seg_out_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={'nin0': pg2_tidx, 'nin1': ng2_tidx}, tr_manager=tr_manager)
        latch_params = dict(pinfo=pinfo, seg=seg_latch, w_p=w_p_latch, w_n=w_n_latch, ridx_n=ridx_n, ridx_p=ridx_p,
                            vertical_sup=False, sig_locs={})
        latch_master = self.new_template(LatchCore, params=latch_params)
        inv_master = self.new_template(InvCore, params=inv_params)
        buf_master = self.new_template(InvCore, params=buf_params)
        outbuf_master = self.new_template(InvChainCore, params=out_buf_params)

        cur_col = 0
        nbits = self.params['nbits']
        out_buf_list = []
        inv_stage0, inv_stage1, inv_stage2 = [], [], []
        latch_stage0, latch_stage1, latch_stage2 = [], [], []

        buf0 = self.add_tile(buf_master, tile_idx=3, col_idx=cur_col)
        buf1 = self.add_tile(buf_master, tile_idx=2, col_idx=cur_col)
        buf2 = self.add_tile(buf_master, tile_idx=1, col_idx=cur_col)
        cur_col += buf_master.num_cols + min_sep
        tap_sup_vdd_hm, tap_sup_vss_hm = [], []
        tap_sup_vdd_xm, tap_sup_vss_xm = [], []
        for idx in range(nbits):
            out_buf_list.append(self.add_tile(outbuf_master, tile_idx=0, col_idx=cur_col))

            inv_stage0.append(self.add_tile(inv_master, tile_idx=3, col_idx=cur_col))
            inv_stage1.append(self.add_tile(inv_master, tile_idx=2, col_idx=cur_col))
            inv_stage2.append(self.add_tile(inv_master, tile_idx=1, col_idx=cur_col))
            cur_col += inv_master.num_cols + min_sep
            cur_col += cur_col & 1
            latch_stage0.append(self.add_tile(latch_master, tile_idx=3, col_idx=cur_col))
            latch_stage1.append(self.add_tile(latch_master, tile_idx=2, col_idx=cur_col))
            latch_stage2.append(self.add_tile(latch_master, tile_idx=1, col_idx=cur_col))
            cur_col += latch_master.num_cols
            if idx % 4 == 0:
                tap_ncol = 0
                for jdx in range(4):
                    sup_hm, sup_vm, sup_xm, tap_ncol = insert_tap(jdx, cur_col)
                    tap_sup_vdd_hm.append(sup_hm[1])
                    tap_sup_vss_hm.append(sup_hm[0])
                    tap_sup_vdd_xm.append(sup_xm[1])
                    tap_sup_vss_xm.append(sup_xm[0])
                cur_col += tap_ncol
            else:
                cur_col += min_sep
        self.set_mos_size(num_tiles=self.num_tile_rows, num_cols=max(self.num_cols, ncol_tot))
        #
        # for pname in latch_stage0[0].port_names_iter():
        #     self.reexport(latch_stage0[0].get_port(pname), hide=False, show=True)

        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        clk0_list, clk1_list, clk2_list = [], [], []
        in_vm_list = []
        for idx in range(nbits):
            _int_mid_vm_tidx = self.grid.coord_to_track(vm_layer, latch_stage0[idx].bound_box.xl,
                                                        RoundMode.GREATER_EQ)
            _stage1_mid = self.connect_to_tracks(latch_stage1[idx].get_pin('nin'),
                                                 TrackID(vm_layer, _int_mid_vm_tidx, tr_w_vm))
            _stage2_mid = self.connect_to_tracks(latch_stage2[idx].get_pin('nin'),
                                                 TrackID(vm_layer, _int_mid_vm_tidx, tr_w_vm))

            _int_mid_xm_tidx = self.grid.coord_to_track(xm_layer, latch_stage0[idx].get_pin('out').lower,
                                                        RoundMode.GREATER)
            self.connect_to_tracks([_stage1_mid, latch_stage0[idx].get_pin('out')],
                                   TrackID(xm_layer, _int_mid_xm_tidx, tr_w_xm))
            _int_mid_xm_tidx = self.grid.coord_to_track(xm_layer, latch_stage1[idx].get_pin('out').lower,
                                                        RoundMode.GREATER)
            self.connect_to_tracks([_stage2_mid, latch_stage1[idx].get_pin('out')],
                                   TrackID(xm_layer, _int_mid_xm_tidx, tr_w_xm))

            # connect clk
            self.connect_wires([inv_stage0[idx].get_pin('nin'), latch_stage0[idx].get_pin('nclk')])
            self.connect_wires([inv_stage1[idx].get_pin('nin'), latch_stage1[idx].get_pin('nclk')])
            self.connect_wires([inv_stage2[idx].get_pin('nin'), latch_stage2[idx].get_pin('nclk')])
            self.connect_to_track_wires(inv_stage0[idx].get_pin('out'), latch_stage0[idx].get_pin('pclkb'))
            self.connect_to_track_wires(inv_stage1[idx].get_pin('out'), latch_stage1[idx].get_pin('pclkb'))
            self.connect_to_track_wires(inv_stage2[idx].get_pin('out'), latch_stage2[idx].get_pin('pclkb'))

            # get clk input
            _clk_vm_tidx = self.grid.coord_to_track(vm_layer, inv_stage0[idx].bound_box.xl, RoundMode.NEAREST)
            clk0_list.append(self.connect_to_tracks(inv_stage0[idx].get_pin('in'),
                                                    TrackID(vm_layer, _clk_vm_tidx, tr_w_vm)))
            clk1_list.append(self.connect_to_tracks(inv_stage1[idx].get_pin('in'),
                                                    TrackID(vm_layer, _clk_vm_tidx, tr_w_vm)))
            clk2_list.append(self.connect_to_tracks(inv_stage2[idx].get_pin('in'),
                                                    TrackID(vm_layer, _clk_vm_tidx, tr_w_vm)))
            # in vm
            _in_vm_tidx = self.grid.coord_to_track(vm_layer, latch_stage0[idx].get_pin('nin').middle, RoundMode.NEAREST)
            in_vm_list.append(self.connect_to_tracks(latch_stage0[idx].get_pin('nin'),
                                                     TrackID(vm_layer, _in_vm_tidx, tr_w_vm),
                                                     min_len_mode=MinLenMode.MIDDLE))

        clk_xm0_tid = self.grid.coord_to_track(xm_layer, buf0.get_pin('out').middle, RoundMode.NEAREST)
        clk_xm1_tid = self.grid.coord_to_track(xm_layer, buf1.get_pin('out').middle, RoundMode.NEAREST)
        clk_xm2_tid = self.grid.coord_to_track(xm_layer, buf2.get_pin('out').middle, RoundMode.NEAREST)

        tr_w_xm_clk = tr_manager.get_width(xm_layer, 'clk')
        tr_w_vm_clk = tr_manager.get_width(vm_layer, 'clk')

        clk_xm0 = self.connect_to_tracks(clk0_list + [buf0.get_pin('out')], TrackID(xm_layer, clk_xm0_tid, tr_w_xm_clk))
        _clk_xm1 = self.connect_to_tracks(clk1_list + [buf1.get_pin('out')],
                                          TrackID(xm_layer, clk_xm1_tid, tr_w_xm_clk))
        _clk_xm2 = self.connect_to_tracks(clk2_list + [buf2.get_pin('out')],
                                          TrackID(xm_layer, clk_xm2_tid, tr_w_xm_clk))

        clk_in_vm_tidx_start = self.grid.coord_to_track(vm_layer, buf0.get_pin('nin').lower, RoundMode.GREATER_EQ)
        _, clk_in_vm_locs = tr_manager.place_wires(vm_layer, ['clk'] * 3, align_track=clk_in_vm_tidx_start)

        clk0, clk1, clk2 = self.connect_matching_tracks([buf0.get_pin('nin'), buf1.get_pin('nin'), buf2.get_pin('nin')],
                                                        vm_layer, clk_in_vm_locs, width=tr_w_vm_clk,
                                                        track_upper=self.bound_box.yh)
        clk_in_xm_tidx_list = [self.grid.coord_to_track(xm_layer, warr.bound_box.yl, RoundMode.NEAREST) for warr in
                               [buf0.get_pin('nin'), buf1.get_pin('nin'), buf2.get_pin('nin')]]
        clk0, clk1, clk2 = self.connect_matching_tracks([clk0, clk1, clk2], xm_layer, clk_in_xm_tidx_list,
                                                        width=tr_w_xm_clk)

        _, clk_in_ym_tidx_list = tr_manager.place_wires(ym_layer, ['clk'] * 3, align_idx=0)
        tr_w_ym_clk = tr_manager.get_width(ym_layer, 'clk')
        clk0, clk1, clk2 = self.connect_matching_tracks([clk0, clk1, clk2], ym_layer, clk_in_ym_tidx_list,
                                                        width=tr_w_ym_clk, track_upper=self.bound_box.yh)

        self.add_pin('clk_in0', clk0, mode=PinMode.UPPER)
        self.add_pin('clk_in1', clk1, mode=PinMode.UPPER)
        self.add_pin('clk_in2', clk2, mode=PinMode.UPPER)

        inst_list = inv_stage0 + inv_stage1 + inv_stage2 + latch_stage0 + latch_stage1 + latch_stage2 + \
                    [buf0, buf1, buf2]
        vdd_hm = [w for inst in inst_list for w in inst.get_all_port_pins('VDD')]
        vss_hm = [w for inst in inst_list for w in inst.get_all_port_pins('VSS')]

        vdd_hm = self.connect_wires(vdd_hm + tap_sup_vdd_hm)
        vss_hm = self.connect_wires(vss_hm + tap_sup_vss_hm)

        vdd_xm = self.connect_wires(tap_sup_vdd_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm = self.connect_wires(tap_sup_vss_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)

        self.add_pin('VDD', vdd_hm)
        self.add_pin('VSS', vss_hm)
        self.add_pin('VDD', vdd_xm)
        self.add_pin('VSS', vss_xm)

        in_xm_tidx_top = self.grid.coord_to_track(xm_layer, in_vm_list[0].lower, RoundMode.NEAREST)
        in_xm_list = [self.connect_to_tracks(vm, TrackID(xm_layer, in_xm_tidx_top, tr_w_xm),
                                             min_len_mode=MinLenMode.MIDDLE) for vm in in_vm_list]

        in_ym_list = []
        tr_w_ym = tr_manager.get_width(ym_layer, 'sig')
        for idx in range(nbits):
            _ym_coord = self.grid.track_to_coord(vm_layer, in_vm_list[idx].track_id.base_index)
            _ym_tid = self.grid.coord_to_track(ym_layer, _ym_coord, RoundMode.NEAREST)
            in_ym_list.append(self.connect_to_tracks(in_xm_list[idx], TrackID(ym_layer, _ym_tid, tr_w_ym),
                                                     track_upper=self.bound_box.yh))

        for idx, lat in enumerate(latch_stage2):
            self.connect_to_track_wires(out_buf_list[idx].get_pin('nin'), lat.get_pin('out'))
            self.add_pin(f'data_out7<{idx}>',
                         self.extend_wires(out_buf_list[idx].get_pin('out'), lower=self.bound_box.yl),
                         mode=PinMode.LOWER)
            self.add_pin(f'd_in<{idx}>', in_vm_list[idx], mode=PinMode.UPPER)
            self.add_pin(f'data_in7<{idx}>', in_ym_list[idx], mode=PinMode.UPPER)
            self.add_pin(f'd_in<{idx}>', in_xm_list[idx])

        # Connect supplies to ym
        ym_lo_tidx = self.grid.coord_to_track(ym_layer, self.bound_box.xl, RoundMode.GREATER)
        ym_hi_tidx = self.grid.coord_to_track(ym_layer, self.bound_box.xh, RoundMode.LESS)
        ym_sup_locs = self.get_available_tracks(ym_layer, ym_lo_tidx, ym_hi_tidx, self.bound_box.yl,
                                                self.bound_box.yh, width=tr_manager.get_width(ym_layer, 'sup'),
                                                sep=tr_manager.get_sep(ym_layer, ('sup', 'sup')))[::2]
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        vdd_ym_locs, vss_ym_locs = ym_sup_locs[::2], ym_sup_locs[1::2]
        vdd_ym = [self.connect_to_tracks(vdd_xm, TrackID(ym_layer, tid, tr_w_sup_ym)) for tid in vdd_ym_locs]
        vss_ym = [self.connect_to_tracks(vss_xm, TrackID(ym_layer, tid, tr_w_sup_ym)) for tid in vss_ym_locs]

        vdd_hm = vdd_hm[0].to_warr_list()
        vss_hm = vss_hm[0].to_warr_list()
        vdd_xm = vdd_xm[0].to_warr_list()
        vss_xm = vss_xm[0].to_warr_list()

        vm_lo_tidx = self.grid.coord_to_track(vm_layer, self.bound_box.xl, RoundMode.GREATER)
        vm_hi_tidx = self.grid.coord_to_track(vm_layer, self.bound_box.xh, RoundMode.LESS)
        vm_sup_locs = self.get_available_tracks(vm_layer, vm_lo_tidx, vm_hi_tidx, self.bound_box.yl,
                                                self.bound_box.yh, width=tr_manager.get_width(vm_layer, 'sup'),
                                                sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')))
        for hm, xm in zip(vdd_hm + vss_hm, vdd_xm + vss_xm):
            for tidx in vm_sup_locs:
                self.connect_to_tracks(hm, TrackID(vm_layer, tidx, tr_manager.get_width(vm_layer, 'sup')),
                                       min_len_mode=MinLenMode.MIDDLE)
                self.connect_to_tracks(xm, TrackID(vm_layer, tidx, tr_manager.get_width(vm_layer, 'sup')),
                                       min_len_mode=MinLenMode.MIDDLE)

        self.add_pin('VDD', vdd_ym)
        self.add_pin('VSS', vss_ym)
        self._sch_params = dict(
            nbits=nbits,
            latch_params=latch_master.sch_params,
            inv_params=inv_master.sch_params,
            buf_params=buf_master.sch_params,
            outbuf_params=outbuf_master.sch_params,
        )


class RetimerFlopsChannel(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'retimer_flops')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            tr_widths='',
            tr_spaces='',
            nbits='',
            ncol_tot='',
            seg_dict='',
            w_dict='',
            ridx_p='',
            ridx_n=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={'w_n': 4, 'w_p': 4},
            ridx_n=0,
            ridx_p=-1,
            ncol_tot=0,
            tr_widths={},
            tr_spaces={},
        )

    def export_xm_sup(self, tile_idx, export_top=False, export_bot=False, given_locs=None):
        """
        This function export top or bot supply to xm layer
        """

        # Setup routing information
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        tr_manager = self.tr_manager
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_sp_sup_vm = tr_manager.get_sep(vm_layer, ('sup', 'sup'))

        # Get tile info, get coord and row_idx
        pinfo, tile_yb, flip_tile = self.used_array.get_tile_info(tile_idx)
        bot_coord = tile_yb
        top_coord = tile_yb + pinfo.height
        bot_ridx = -1 if flip_tile else 0
        top_ridx = 0 if flip_tile else -1

        # Get track info
        vm_min_len = self.grid.get_next_length(vm_layer, tr_w_sup_vm, 0, even=True)
        vm_min_le_sp = self.grid.get_line_end_space(vm_layer, tr_w_sup_vm, even=True)
        xm_bnd_l, xm_bnd_u = self.grid.get_wire_bounds_htr(xm_layer, 0, tr_w_sup_xm)
        # via_ext_xm1, via_ext_ym1 = self.grid.get_via_extensions(Direction.LOWER, xm1_layer, sup_w_xm1, sup_w_ym1)

        xm_w = xm_bnd_u - xm_bnd_l

        # Get xm layer info, calculate margin
        sep_margin = self.grid.get_sep_tracks(xm_layer, tr_w_sup_xm, 1, same_color=False)
        tr_info = self.grid.get_track_info(xm_layer)
        margin = tr_info.pitch * sep_margin // 2

        # Get vm layer info, calculate margin
        vm_margin = max(vm_min_len, xm_w) // 2 + 2 * vm_min_le_sp  # If not perfect align, add margin by *2
        margin = max(vm_margin, margin) + 2 * vm_min_len
        vm_sep_margin = self.grid.get_sep_tracks(vm_layer, tr_w_sup_vm, tr_w_sup_vm, same_color=False)
        vm_tr_info = self.grid.get_track_info(vm_layer)
        vm_margin = vm_tr_info.pitch * vm_sep_margin // 2

        # Get start and end track locs, avoid short with adjacent blocks
        vm_ti_lo = self.grid.coord_to_track(vm_layer, self.bound_box.xl + vm_margin, mode=RoundMode.GREATER_EQ)
        vm_ti_hi = self.grid.coord_to_track(vm_layer, self.bound_box.xh - vm_margin, mode=RoundMode.LESS_EQ)

        if export_bot:
            if not given_locs:
                bot_vm_locs = self.get_available_tracks(vm_layer, vm_ti_lo, vm_ti_hi,
                                                        bot_coord - margin, bot_coord + margin,
                                                        width=tr_w_sup_vm, sep=tr_sp_sup_vm, include_last=False,
                                                        sep_margin=tr_sp_sup_vm)
            else:
                bot_vm_locs = given_locs

            bot_hm_tid = self.get_track_id(bot_ridx, MOSWireType.DS, 'sup', 0, tile_idx=tile_idx)
            bot_sup_hm = self.add_wires(hm_layer, bot_hm_tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                        width=bot_hm_tid.width)
            bot_sup_vm = []
            for tid in bot_vm_locs:
                bot_sup_vm.append(self.connect_to_tracks(bot_sup_hm, TrackID(vm_layer, tid, tr_w_sup_vm),
                                                         min_len_mode=MinLenMode.MIDDLE))

            xm_sup_coord = self.grid.track_to_coord(hm_layer, bot_hm_tid.base_index)
            xm_sup_tid = self.grid.coord_to_track(xm_layer, xm_sup_coord, mode=RoundMode.NEAREST)
            if bot_sup_vm:
                bot_sup_xm = self.connect_to_tracks(bot_sup_vm, TrackID(xm_layer, xm_sup_tid, tr_w_sup_xm))
            else:
                bot_sup_xm = self.add_wires(xm_layer, xm_sup_tid, width=tr_w_sup_xm, lower=self.bound_box.xl,
                                            upper=self.bound_box.xh)
        else:
            bot_sup_xm = None

        if export_top:
            if not given_locs:
                top_vm_locs = self.get_available_tracks(vm_layer, vm_ti_lo, vm_ti_hi,
                                                        top_coord - margin, top_coord + margin,
                                                        width=tr_w_sup_vm, sep=tr_sp_sup_vm, include_last=False,
                                                        sep_margin=tr_sp_sup_vm)
            else:
                top_vm_locs = given_locs

            top_hm_tid = self.get_track_id(top_ridx, MOSWireType.DS, 'sup', 0, tile_idx=tile_idx)
            top_sup_hm = self.add_wires(hm_layer, top_hm_tid.base_index, self.bound_box.xl, self.bound_box.xh,
                                        width=top_hm_tid.width)
            top_sup_vm = []
            for tid in top_vm_locs:
                top_sup_vm.append(self.connect_to_tracks(top_sup_hm, TrackID(vm_layer, tid, tr_w_sup_vm),
                                                         min_len_mode=MinLenMode.MIDDLE))

            xm_sup_coord = self.grid.track_to_coord(hm_layer, top_hm_tid.base_index)
            xm_sup_tid = self.grid.coord_to_track(xm_layer, xm_sup_coord, mode=RoundMode.NEAREST)

            if top_sup_vm:
                top_sup_xm = self.connect_to_tracks(top_sup_vm, TrackID(xm_layer, xm_sup_tid, tr_w_sup_xm))
            else:
                top_sup_xm = self.add_wires(xm_layer, xm_sup_tid, width=tr_w_sup_xm, lower=self.bound_box.xl,
                                            upper=self.bound_box.xh)
        else:
            top_sup_xm = None

        return bot_sup_xm, top_sup_xm

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_dict = self.params['w_dict']
        w_n: int = w_dict['w_n']
        w_p: int = w_dict['w_p']
        w_d_buf_n = w_dict.get('w_d_buf_n', w_n)
        w_d_buf_p = w_dict.get('w_d_buf_p', w_p)
        w_latch_n = w_dict.get('w_latch_n', w_n)
        w_latch_p = w_dict.get('w_latch_p', w_p)

        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        ncol_tot: int = self.params['ncol_tot']
        seg_ret_unit: int = seg_dict['ret_unit']
        seg_buf: int = seg_dict['buf']
        seg_buf_out: int = seg_dict['buf_out']
        seg_inbuf: int = seg_dict['inbuf']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = TrackManager(self.grid, self.params['tr_widths'], self.params['tr_spaces'])
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

        pinfo = self.get_tile_pinfo(1)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        inbuf_params = dict(pinfo=pinfo, seg_list=seg_inbuf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            vertical_sup=False, sig_locs={'nin0': ng1_tidx, 'nin1': ng2_tidx})
        buf_in_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_d_buf_p, w_n=w_d_buf_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             vertical_sup=False, sig_locs={}, tr_manager=tr_manager)
        buf_out_params = dict(pinfo=pinfo, seg_list=seg_buf_out, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={}, tr_manager=tr_manager)
        ret_params = dict(pinfo=pinfo, seg_dict=seg_ret_unit, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False, sig_locs={})
        inbuf_master = self.new_template(InvChainCore, params=inbuf_params)
        ret_master = self.new_template(SARRetUnit, params=ret_params)
        buf_in_master = self.new_template(InvChainCore, params=buf_in_params)
        buf_out_master = self.new_template(InvChainCore, params=buf_out_params)

        cur_col = 0
        nbits = self.params['nbits']
        if isinstance(nbits, ImmutableList):
            nrow = len(nbits)
        else:
            nrow = 1
            nbits = [nbits]

        max_num_unit_per_row = 0
        xm_rt_span_coord = 0
        while xm_rt_span_coord < self.get_tile_info(0)[0].height:
            max_num_unit_per_row += 1
            _xm_ntr, _ = tr_manager.place_wires(xm_layer, ['sup'] + ['sig'] * max_num_unit_per_row + ['clk'] + ['sup'])
            xm_rt_span_coord = self.grid.track_to_coord(xm_layer, _xm_ntr)

        # max_num_unit_per_row = 2 * (max_num_unit_per_row - 1)
        max_num_unit_per_row -= 1
        ret_list_list = []
        inbuf_list_list = []
        inbuf_in_list_list = []
        input_ym_ntr, _ = tr_manager.place_wires(ym_layer, ['sig'] * (sum(nbits) + 1))
        start_col = self.arr_info.track_to_col(ym_layer, input_ym_ntr)
        for idx in range(nrow):
            ret_list = []
            inbuf_list = []
            inbuf_in_list = []
            cur_col = start_col
            if nbits[idx] > max_num_unit_per_row:
                raise ValueError("too many units in one row, max: ", max_num_unit_per_row)
            for jdx in range(nbits[idx]):
                inbuf_list.append(self.add_tile(inbuf_master, tile_idx=idx, col_idx=cur_col))
                cur_col += inbuf_master.num_cols + min_sep
                ret_list.append(self.add_tile(ret_master, tile_idx=idx, col_idx=cur_col))
                cur_col += ret_master.num_cols
                conn_mid_hm_tidx = self.grid.coord_to_track(hm_layer, inbuf_list[-1].get_pin('out').middle,
                                                            RoundMode.LESS)
                self.connect_to_track_wires(inbuf_list[-1].get_pin('out'), ret_list[-1].get_pin('in_hm'))
                tidx = self.grid.coord_to_track(vm_layer, inbuf_list[-1].bound_box.xl, RoundMode.NEAREST)
                inbuf_in_list.append(self.connect_to_tracks(inbuf_list[-1].get_pin('nin'),
                                                            TrackID(vm_layer, tidx)))
            inbuf_in_list_list.append(inbuf_in_list)
            ret_list_list.append(ret_list)
            inbuf_list_list.append(inbuf_list)

        cur_col = 0
        buf_in = self.add_tile(buf_in_master, tile_idx=self.num_tile_rows, col_idx=start_col)
        buf_out = self.add_tile(buf_out_master, tile_idx=self.num_tile_rows - 1,
                                col_idx=start_col+buf_in_master.num_cols+min_sep)

        self.set_mos_size(num_tiles=self.num_tile_rows, num_cols=max(self.num_cols, ncol_tot))
        tr_w_xm_clk = tr_manager.get_width(xm_layer, 'clk')
        xm_in_list_list = []
        xm_out_list_list = []
        clk_xm_list = []
        vdd_xm_list, vss_xm_list = [], []
        for idx in range(self.num_tile_rows):
            _sup, _ = self.export_xm_sup(idx, export_bot=True)
            if idx & 1:
                vdd_xm_list.append(_sup)
            else:
                vss_xm_list.append(_sup)
        _, _sup = self.export_xm_sup(self.num_tile_rows - 1, export_top=True)
        if self.num_tile_rows & 1:
            vdd_xm_list.append(_sup)
        else:
            vss_xm_list.append(_sup)

        for idx in range(nrow):
            tile_top_coord = self.get_tile_info(idx)[1] + self.get_tile_info(idx)[0].height
            tile_bot_coord = self.get_tile_info(idx)[1]
            xm_mid_coord = (tile_top_coord + tile_bot_coord) // 2
            clk_xm_tidx = self.grid.coord_to_track(xm_layer, xm_mid_coord, RoundMode.NEAREST)
            clk_xm = self.connect_to_tracks([inst.get_pin('clk') for inst in ret_list_list[idx]],
                                            TrackID(xm_layer, clk_xm_tidx, tr_w_xm_clk))
            tile_top_xm_tidx = self.grid.coord_to_track(xm_layer, tile_top_coord, RoundMode.NEAREST)
            tile_bot_xm_tidx = self.grid.coord_to_track(xm_layer, tile_bot_coord, RoundMode.NEAREST)
            xm_sig_locs = self.get_available_tracks(xm_layer, tile_bot_xm_tidx, tile_top_xm_tidx,
                                                    self.bound_box.xl, self.bound_box.xh, width=tr_w_xm,
                                                    sep=tr_manager.get_sep(xm_layer, ('sig', 'sig')),
                                                    sep_margin=tr_manager.get_sep(xm_layer, ('sig', 'sig')))

            xm_sig_list = self.connect_matching_tracks(inbuf_in_list_list[idx],
                                                       xm_layer, xm_sig_locs[1:nbits[idx] + 1], width=tr_w_xm)
            xm_in_list_list.append(xm_sig_list)
            clk_xm_list.append(clk_xm)
            xm_out_tidx = xm_sig_locs[nbits[idx] + 1]
            xm_out_list_list.append(
                [self.connect_to_tracks(inst.get_pin('out'), TrackID(xm_layer, xm_out_tidx, tr_w_xm)) for inst in
                 ret_list_list[idx]])

        clk_int_xm_tidx = self.grid.coord_to_track(xm_layer, buf_in.get_pin('out').middle, RoundMode.NEAREST)
        clk_int_xm = self.connect_to_tracks(buf_in.get_pin('out'), TrackID(xm_layer, clk_int_xm_tidx, tr_w_xm_clk))
        tr_w_ym_clk = tr_manager.get_width(ym_layer, 'clk')
        clk_int_ym_tidx = self.grid.coord_to_track(ym_layer, clk_int_xm.middle, RoundMode.GREATER_EQ)
        self.connect_to_tracks(clk_xm_list + [clk_int_xm], TrackID(ym_layer, clk_int_ym_tidx, tr_w_ym_clk))

        clk_out = self.via_stack_up(tr_manager, buf_out.get_all_port_pins('out'), vm_layer, ym1_layer, 'clk_out')
        clk_out = self.extend_wires(clk_out[ym1_layer], lower=self.bound_box.yl)

        # connect to buf out
        self.connect_to_track_wires(buf_in.get_pin('out'), buf_out.get_pin('nin'))
        _, input_ym_locs = tr_manager.place_wires(ym_layer, ['sig'] * (sum(nbits) + 1))
        ym_in_list_list = []
        tr_w_ym = tr_manager.get_width(ym_layer, 'sig')
        for idx in range(len(nbits)):
            ym_locs = input_ym_locs[sum(nbits[:idx + 1]) - nbits[idx]:sum(nbits[:idx + 1])]
            ym_in_list_list.append(self.connect_matching_tracks(xm_in_list_list[idx], ym_layer, ym_locs,
                                                                width=tr_w_ym, track_upper=self.bound_box.yh))

        # figure out out ym locs
        max_ncol = max(nbits)
        nrow = len(nbits)
        out_ym_tidx_list_list = []
        for idx in range(max_ncol):
            ref_col = (idx + 1) * (ret_master.num_cols + inbuf_master.num_cols + min_sep) + start_col
            ref_tidx = self.arr_info.col_to_track(ym_layer, ref_col, RoundMode.NEAREST)
            _, out_ym_locs = tr_manager.place_wires(ym_layer, ['sig'] * nrow, align_track=ref_tidx, align_idx=-1)
            out_ym_tidx_list_list.append(out_ym_locs)

        ym_out_list_list = []
        for idx in range(len(nbits)):
            ym_out_list = []
            for jdx in range(nbits[idx]):
                ym_out_list.append(self.connect_to_tracks(xm_out_list_list[idx][jdx],
                                                          TrackID(ym_layer, out_ym_tidx_list_list[jdx][idx], tr_w_ym),
                                                          track_lower=self.bound_box.yl))
            ym_out_list_list.append(ym_out_list)

        # clock in
        clk_vm_tidx = self.arr_info.col_to_track(vm_layer, start_col, RoundMode.LESS)
        clk_vm = self.connect_to_tracks(buf_in.get_pin('nin'),
                                        TrackID(vm_layer, clk_vm_tidx, tr_manager.get_width(vm_layer, 'sig')))
        clk_xm_tidx = self.grid.coord_to_track(xm_layer, clk_vm.middle, RoundMode.NEAREST)
        clk_xm = self.connect_to_tracks(clk_vm, TrackID(xm_layer, clk_xm_tidx, tr_w_xm_clk))
        clk_ym_tidx = self.grid.coord_to_track(ym_layer, clk_xm.upper, RoundMode.NEAREST)
        clk_ym = self.connect_to_tracks(clk_xm, TrackID(ym_layer, clk_ym_tidx, tr_w_ym_clk),
                                        track_upper=self.bound_box.yh)

        bit_cnt = 0
        for idx in range(nrow):
            for jdx in range(nbits[idx]):
                self.add_pin(f'd_in<{bit_cnt}>', ym_in_list_list[idx][jdx], mode=PinMode.UPPER)
                self.add_pin(f'd_out<{bit_cnt}>', ym_out_list_list[idx][jdx], mode=PinMode.LOWER)
                bit_cnt += 1

        self.add_pin('clk', clk_xm)
        self.add_pin('VDD_xm', vdd_xm_list, label='VDD')
        self.add_pin('VSS_xm', vss_xm_list, label='VSS')

        vdd_hm, vss_hm = [], []
        for inst in [buf_in, buf_out] + [inst for inst_list in ret_list_list for inst in inst_list]:
            vdd_hm.append(inst.get_pin('VDD'))
            vss_hm.append(inst.get_pin('VSS'))

        self.add_pin('VDD_hm', self.connect_wires(vdd_hm), label='VDD')
        self.add_pin('VSS_hm', self.connect_wires(vss_hm), label='VSS')
        self.add_pin('clk_out', clk_out, mode=PinMode.LOWER)

        self._sch_params = dict(
            nbits=sum(nbits),
            ret_params=ret_master.sch_params,
            buf_in_params=buf_in_master.sch_params,
            buf_out_params=buf_out_master.sch_params,
            inbuf_params=inbuf_master.sch_params,
        )


class RetimerCLK(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'retimer_clk')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            mux_params='',
            buf_in_params='',
            buf_out_params='',
            sr_params='',
            tr_widths='',
            tr_spaces='',
            top_layer='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            top_layer=0,
            tr_widths={},
            tr_spaces={},
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = TrackManager(self.grid, self.params['tr_widths'], self.params['tr_spaces'])
        top_layer = self.params['top_layer']
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        pinfo = self.get_tile_pinfo(1)

        buf_gen_params = self.params['buf_in_params'].copy(append=dict(pinfo=pinfo, dual_output=True))
        bufout_gen_params = self.params['buf_out_params'].copy(append=dict(pinfo=pinfo, tr_manager=tr_manager))
        mux_gen_params = self.params['mux_params'].copy(append=dict(pinfo=pinfo, tr_manager=tr_manager))
        inv_mux_gen_params = dict(pinfo=pinfo, seg_list=[4, 4], dual_output=True)
        sr_gen_params = self.params['sr_params'].copy(append=dict(pinfo=pinfo))
        buf_temp = self.new_template(InvChainCore, params=buf_gen_params)
        bufout_temp = self.new_template(InvChainCore, params=bufout_gen_params)
        sr_temp = self.new_template(NANDSRCore, params=sr_gen_params)
        mux_temp = self.new_template(MuxCore, params=mux_gen_params)
        inv_mux_temp = self.new_template(InvChainCore, params=inv_mux_gen_params)

        cur_col = 10
        sr_col = cur_col + min_sep
        sr_col += sr_col & 1
        top_tile_idx = 2
        sr = self.add_tile(sr_temp, top_tile_idx, sr_col)
        mux_col = max(sr_temp.num_cols + 2 * min_sep + buf_temp.num_cols + inv_mux_temp.num_cols + 6 * min_sep,
                      mux_temp.num_cols)
        clk_in_buf = self.add_tile(buf_temp, top_tile_idx - 1, mux_col, flip_lr=True)
        mux_col -= buf_temp.num_cols + min_sep
        clk_mux = self.add_tile(mux_temp, top_tile_idx - 1, mux_col, flip_lr=True)
        inv_mux_col = mux_col - (inv_mux_temp.num_cols + min_sep + mux_temp.num_cols)
        inv_mux = self.add_tile(inv_mux_temp, top_tile_idx - 1, inv_mux_col)
        outbuf_col = max(bufout_temp.num_cols, self.num_cols) - bufout_temp.num_cols - 4 * min_sep
        clk_out_buf = self.add_tile(bufout_temp, top_tile_idx - 2, outbuf_col, flip_lr=False)
        self.set_mos_size(self.num_cols + 10)

        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        # Connect buf in to mux
        buf_in_in_vm_tidx = self.grid.coord_to_track(vm_layer, clk_in_buf.bound_box.xh, RoundMode.GREATER)
        buf_in_in_vm = self.connect_to_tracks(clk_in_buf.get_pin('nin'),
                                              TrackID(vm_layer, buf_in_in_vm_tidx,
                                                      clk_in_buf.get_pin('out').track_id.width))
        self.connect_differential_wires(clk_in_buf.get_pin('out'), clk_in_buf.get_pin('outb'),
                                        clk_mux.get_pin('in<1>'), clk_mux.get_pin('in<0>'))

        # sr latch to buf in
        sr_out_xm_tidx = self.grid.coord_to_track(xm_layer, sr.get_pin('q').middle, RoundMode.NEAREST)
        self.connect_to_track_wires(buf_in_in_vm,
                                    self.connect_to_tracks(sr.get_pin('q'), TrackID(xm_layer, sr_out_xm_tidx,
                                                                                    tr_w_clk_xm)))

        # clk out
        clk_out_buf_in_vm_tidx = self.grid.coord_to_track(vm_layer, clk_out_buf.bound_box.xl, RoundMode.GREATER)
        clk_out_buf_inv_vm = self.connect_to_tracks(clk_out_buf.get_pin('nin'),
                                                    TrackID(vm_layer, clk_out_buf_in_vm_tidx,
                                                            inv_mux.get_pin('out').track_id.width))
        # mux to buf_out
        _, clk_sel_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 4,
                                                    center_coord=(inv_mux.bound_box.yl + inv_mux.bound_box.yh) // 2)
        clk_out_buf_in_xm_tidx = self.grid.coord_to_track(xm_layer,
                                                          clk_out_buf.bound_box.yl + clk_out_buf.bound_box.h // 2,
                                                          RoundMode.NEAREST)
        clk_out_buf_in_xm = self.connect_to_tracks(clk_out_buf_inv_vm, TrackID(xm_layer, clk_out_buf_in_xm_tidx,
                                                                               tr_w_clk_xm))
        mux_out_xm = self.connect_to_tracks(clk_mux.get_pin('out'),
                                            TrackID(xm_layer, clk_sel_xm_locs[-1], tr_w_clk_xm))
        #
        clk_out_ym_tidx = self.grid.coord_to_track(ym_layer, mux_out_xm.middle, RoundMode.NEAREST)
        _clk_out_ym = self.connect_to_tracks([mux_out_xm, clk_out_buf_in_xm],
                                             TrackID(ym_layer, clk_out_ym_tidx, tr_w_clk_ym))
        # self.add_pin('clk_out', clk_out_ym)

        clk_sel_vm_tidx = self.grid.coord_to_track(vm_layer, inv_mux.bound_box.xl, RoundMode.NEAREST)
        clk_sel_vm = self.connect_to_tracks(inv_mux.get_pin('nin'), TrackID(vm_layer, clk_sel_vm_tidx,
                                                                            inv_mux.get_pin('out').track_id.width))
        self.connect_matching_tracks([[clk_mux.get_pin('sel'), inv_mux.get_pin('outb')],
                                      [clk_mux.get_pin('selb'), inv_mux.get_pin('out')]],
                                     xm_layer, clk_sel_xm_locs[1:-1],
                                     width=tr_manager.get_width(xm_layer, 'clk'))
        clk_sel_xm = self.connect_to_tracks(clk_sel_vm, TrackID(xm_layer, clk_sel_xm_locs[-1], tr_w_clk_xm))
        clk_sel_ym_tidx = self.grid.coord_to_track(ym_layer, clk_sel_xm.middle, RoundMode.NEAREST)
        clk_sel_ym = self.connect_to_tracks(clk_sel_xm, TrackID(ym_layer, clk_sel_ym_tidx, tr_w_clk_ym),
                                            track_upper=self.bound_box.yh)
        self.add_pin('clk_sel', clk_sel_ym)

        # sr input
        _, sr_in_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 3, align_track=sr_out_xm_tidx, align_idx=1)
        sr_in_xm_locs.pop(1)
        s_xm, r_xm = self.connect_matching_tracks([sr.get_pin('s'), sr.get_pin('r')], xm_layer, sr_in_xm_locs,
                                                  width=tr_w_clk_xm)
        _, sr_in_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=s_xm.middle)
        s_ym, r_ym = self.connect_matching_tracks([s_xm, r_xm], ym_layer, sr_in_ym_locs,
                                                  width=tr_w_clk_ym, track_upper=self.bound_box.yh)

        clk_out = clk_out_buf.get_pin('out')
        clk_out = self.add_wires(vm_layer, clk_out.track_id.base_index, clk_out.middle - clk_out.bound_box.h // 4,
                                 clk_out.middle + clk_out.bound_box.h // 4, width=clk_out.track_id.width)
        out_stack_up = self.via_stack_up(tr_manager, clk_out, vm_layer, top_layer, 'clk')
        self.add_pin('clk_mid', s_ym)
        self.add_pin('clk_last', r_ym)
        self.add_pin('clk_out', out_stack_up[top_layer])

        # supplies
        for idx in range(self.num_tile_rows):
            fill_tap(self, idx, SubPortMode.EVEN)

        # Supply
        vdd_hm_list, vss_hm_list = [], []
        inst_list = [clk_mux, clk_in_buf, clk_out_buf, inv_mux, sr]
        for inst in inst_list:
            vdd_hm_list.extend(inst.get_all_port_pins('VDD'))
            vss_hm_list.extend(inst.get_all_port_pins('VSS'))

        self.add_pin('VDD_hm', vdd_hm_list, label='VDD')
        self.add_pin('VSS_hm', vss_hm_list, label='VSS')

        xm_sup_list = []
        for idx in range(self.num_tile_rows):
            xm_sup_list.append(export_xm_sup(self, idx, export_bot=True)[0])
        xm_sup_list.append(export_xm_sup(self, self.num_tile_rows - 1, export_top=True)[1])
        self.add_pin('VDD_xm', xm_sup_list[1::2], label='VDD')
        self.add_pin('VSS_xm', xm_sup_list[::2], label='VSS')

        self._sch_params = dict(
            # retimer_params=retimer_ch_temp.sch_params,
            ck_in_buf_params=buf_temp.sch_params,
            ck_out_buf_params=bufout_temp.sch_params,
            sr_params=sr_temp.sch_params,
            # num_channels=num_channels,
            mux_params=mux_temp.sch_params,
            inv_mux_params=inv_mux_temp.sch_params
        )


class RetimerBuffer(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'retimer_channel_clk')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            buf_params='',
            mux_params='',
            tr_widths='',
            tr_spaces='',
            top_layer='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            top_layer=0,
            tr_widths={},
            tr_spaces={},
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = TrackManager(self.grid, self.params['tr_widths'], self.params['tr_spaces'])
        top_layer = self.params['top_layer']
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        pinfo = self.get_tile_pinfo(1)

        mux_gen_params = self.params['mux_params'].copy(append=dict(pinfo=pinfo))
        inv_mux_gen_params = dict(pinfo=pinfo, seg_list=[4, 4], dual_output=True)
        buf_gen_params = self.params['buf_params'].copy(append=dict(pinfo=pinfo, tr_manager=tr_manager))
        mux_temp = self.new_template(MuxCore, params=mux_gen_params)
        inv_mux_temp = self.new_template(InvChainCore, params=inv_mux_gen_params)
        buf_temp = self.new_template(InvChainCore, params=buf_gen_params)

        inv_mux = self.add_tile(inv_mux_temp, 0, 10)
        cur_col = 10 + inv_mux_temp.num_cols + 4
        clk_mux = self.add_tile(mux_temp, 0, cur_col)
        cur_col += mux_temp.num_cols + 4
        clk_buf = self.add_tile(buf_temp, 0, cur_col, flip_lr=False)
        self.set_mos_size(self.num_cols + 10)

        # mux to buf_out
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        _, clk_sel_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 4,
                                                    center_coord=(inv_mux.bound_box.yl + inv_mux.bound_box.yh) // 2)

        clk_sel_vm_tidx = self.grid.coord_to_track(vm_layer, inv_mux.bound_box.xl, RoundMode.NEAREST)
        clk_sel_vm_tidx = tr_manager.get_next_track(vm_layer, clk_sel_vm_tidx, 'clk', 'clk', up=False)
        clk_sel_vm = self.connect_to_tracks(inv_mux.get_pin('nin'), TrackID(vm_layer, clk_sel_vm_tidx,
                                                                            inv_mux.get_pin('out').track_id.width))
        self.connect_matching_tracks([[clk_mux.get_pin('sel'), inv_mux.get_pin('outb')],
                                      [clk_mux.get_pin('selb'), inv_mux.get_pin('out')]],
                                     xm_layer, clk_sel_xm_locs[1:-1],
                                     width=tr_manager.get_width(xm_layer, 'clk'))
        clk_sel_xm = self.connect_to_tracks(clk_sel_vm, TrackID(xm_layer, clk_sel_xm_locs[-1], tr_w_clk_xm))
        clk_sel_ym_tidx = self.grid.coord_to_track(ym_layer, clk_sel_xm.middle, RoundMode.NEAREST)
        clk_sel_ym = self.connect_to_tracks(clk_sel_xm, TrackID(ym_layer, clk_sel_ym_tidx, tr_w_clk_ym, grid=self.grid),
                                            track_upper=self.bound_box.yh)
        self.add_pin('clk_sel', clk_sel_ym)

        # connect clkbuf
        self.connect_to_track_wires(clk_buf.get_pin('nin'), clk_mux.get_pin('out'))
        clk_out = clk_buf.get_pin('outb')
        clk_out = self.add_wires(vm_layer, clk_out.track_id.base_index, clk_out.middle - clk_out.bound_box.h // 4,
                                 clk_out.middle + clk_out.bound_box.h // 4, width=clk_out.track_id.width)

        out_stack_up = self.via_stack_up(tr_manager, clk_out, vm_layer, top_layer, 'clk')
        self.add_pin('clk_out', out_stack_up[top_layer])

        _, mux_in_vm_locs = tr_manager.place_wires(vm_layer, ['clk', 'clk', 'clk'],
                                                    center_coord=(clk_mux.get_pin('selb').bound_box.xh+clk_mux.get_pin('sel').bound_box.xl)//2)
        mux_in_vm_locs.pop(1)
        mux_in0_vm, mux_in1_vm = self.connect_matching_tracks([clk_mux.get_pin('in<0>'), clk_mux.get_pin('in<1>')],
                                                              vm_layer, mux_in_vm_locs, width=tr_w_clk_vm)
        mux_in0_xm, mux_in1_xm = self.connect_matching_tracks([mux_in0_vm, mux_in1_vm], xm_layer,
                                                              [clk_sel_xm_locs[0], clk_sel_xm_locs[-1]],
                                                              width=tr_w_clk_xm)

        mux_in0_ym_tidx = self.grid.coord_to_track(ym_layer, mux_in0_xm.lower, mode=RoundMode.NEAREST)
        mux_in1_ym_tidx = self.grid.coord_to_track(ym_layer, mux_in1_xm.upper, mode=RoundMode.NEAREST)
        mux_in0_ym = self.connect_to_tracks(mux_in0_xm, TrackID(ym_layer, mux_in0_ym_tidx, tr_w_clk_ym, grid=self.grid),
                                            track_lower=mux_in0_vm.lower, track_upper=mux_in0_vm.upper)
        mux_in1_ym = self.connect_to_tracks(mux_in1_xm, TrackID(ym_layer, mux_in1_ym_tidx, tr_w_clk_ym, grid=self.grid),
                                            track_lower=mux_in0_vm.lower, track_upper=mux_in0_vm.upper)
        in0_stack_up = self.via_stack_up(tr_manager, mux_in0_ym, ym_layer, top_layer, 'clk')
        self.add_pin('in0', in0_stack_up[top_layer])
        in1_stack_up = self.via_stack_up(tr_manager, mux_in1_ym, ym_layer, top_layer, 'clk')
        self.add_pin('in1', in1_stack_up[top_layer])
        #
        # supplies
        for idx in range(self.num_tile_rows):
            fill_tap(self, idx, SubPortMode.EVEN)

        # Supply
        vdd_hm_list, vss_hm_list = [], []
        inst_list = [clk_buf, clk_mux, inv_mux]
        for inst in inst_list:
            vdd_hm_list.extend(inst.get_all_port_pins('VDD'))
            vss_hm_list.extend(inst.get_all_port_pins('VSS'))

        self.add_pin('VDD_hm', vdd_hm_list, label='VDD')
        self.add_pin('VSS_hm', vss_hm_list, label='VSS')

        xm_sup_list = []
        for idx in range(self.num_tile_rows):
            xm_sup_list.append(export_xm_sup(self, idx, export_bot=True)[0])
        xm_sup_list.append(export_xm_sup(self, self.num_tile_rows - 1, export_top=True)[1])
        self.add_pin('VDD_xm', xm_sup_list[1::2], label='VDD')
        self.add_pin('VSS_xm', xm_sup_list[::2], label='VSS')

        self._sch_params = dict(buf_params=buf_temp.sch_params,
                                inv_mux_params=inv_mux_temp.sch_params,
                                mux_params=mux_temp.sch_params)


class Retimer(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'retimer')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            num_channels='',
            retimer_params='',
            buf_params='',
            clk_params='',
            tr_widths='',
            tr_spaces='',
            top_layer='',
            ch_space='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            top_layer=0,
            num_channels=1,
            tr_widths={},
            tr_spaces={},
            ch_space=0,
        )

    def draw_layout(self) -> None:
        num_channels = self.params['num_channels']
        ch_space = self.params['ch_space']
        conn_layer = 1
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        retimer_params = self.params['retimer_params']
        buf_params = self.params['buf_params']
        clk_params = self.params['clk_params']

        if isinstance(retimer_params, str):
            retimer_params = read_yaml(retimer_params)
            retimer_params = retimer_params['params']

        if isinstance(buf_params, str):
            buf_params = read_yaml(buf_params)
            buf_params = buf_params['params']

        if isinstance(clk_params, str):
            clk_params = read_yaml(clk_params)
            clk_params = clk_params['params']

        buf_temp = self.new_template(IntegrationWrapper, params=buf_params)
        clk_temp = self.new_template(IntegrationWrapper, params=clk_params)
        ret_temp = self.new_template(IntegrationWrapper, params=retimer_params)
        top_layer = max(buf_temp.top_layer, clk_temp.top_layer, ret_temp.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        # compute track locations
        tr_manager = TrackManager(self.grid, self.params['tr_widths'], self.params['tr_spaces'])
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ch_space = max(ch_space // self.grid.resolution, ret_temp.bound_box.w)
        ch_space = int(ch_space)

        ch_list = []
        ch_x_coord_list = []
        for idx in range(num_channels):
            ch_x_coord_list.append(idx * ch_space)
            ch_list.append(self.add_instance(ret_temp, xform=Transform(idx * ch_space, 0, Orientation.R0)))

        y_ret_top = -(-(ret_temp.bound_box.h + clk_temp.bound_box.h) // h_blk) * h_blk
        clk_buf = self.add_instance(clk_temp, xform=Transform(ch_x_coord_list[-1] - w_blk, y_ret_top, Orientation.MY))

        ch_index = [num_channels - idx // 2 - 1 if idx & 1 else idx // 2 for idx in range(num_channels)][::-1]
        clk_index = list({num_channels // 2 - 1, num_channels // 2 + 1, 1, num_channels - 1})

        clk_x_coord_list = []
        for idx in clk_index:
            _idx = ch_index.index(idx)
            clk_x_coord_list.append(ch_x_coord_list[_idx])

        buf_list = []
        for x in clk_x_coord_list:
            buf_list.append(self.add_instance(buf_temp, xform=Transform(x - w_blk, y_ret_top, Orientation.MY)))

        w_tot = -(-(num_channels * ch_space) // w_blk) * w_blk
        h_tot = -(-ret_temp.bound_box.h // h_blk) * h_blk
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot), half_blk_x=False)


        self._sch_params = dict(
            num_channels=num_channels,
            retimer_params=ret_temp.sch_params,
            buf_params=buf_temp.sch_params,
            clk_params=clk_temp.sch_params,
        )
