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
from bag.util.immutable import Param
from bag.util.math import HalfInt
from bag_vco_adc.layout.clk.clk_delay_tune import ClkDelay
from bag_vco_adc.layout.clk.clk_local import DiffInvCoupledChainBuf, ClkRetimer
from bag_vco_adc.layout.digital import NAND2Core, NOR2Core, InvChainCore
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv
from bag_vco_adc.layout.util.wrapper import GenericWrapper
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, Orientation, MinLenMode, PinMode
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo


class ClkLocalRetimer(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._clk_col_alignment = 0
        self._output_cols_alignment = [0, 0]

    @property
    def clk_col_alignment(self):
        return self._clk_col_alignmenClkLocalTopt

    @property
    def output_cols_alignment(self):
        return self._output_cols_alignment

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_retimer_sam')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_ret='Number of segments.',
            seg_buf='',
            seg_nand='',
            seg_nor='',
            seg_en='',
            seg_ff='',
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

    def get_rel_track_index(self, row_idx, wire_type, wire_name, wire_idx, tile_idx):
        hm_layer = self.conn_layer + 1
        yb = self.get_tile_info(tile_idx)[1]
        abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                        tile_idx=tile_idx)
        abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
        rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
        return rel_tidx

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_ret: Dict[str, Any] = self.params['seg_ret']
        seg_buf: Dict[str, Any] = self.params['seg_buf']
        seg_en = self.params['seg_en']
        seg_ff = self.params['seg_ff']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        # Make templates
        ng0_tidx = self.get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)
        ng1_tidx = self.get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)

        ret_params = dict(seg_dict=seg_ret, w_p=w_p, w_n=w_n, pinfo=pinfo)
        nand_params = dict(seg=self.params['seg_nand'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                           vertical_out=False, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        nor_params = dict(seg=self.params['seg_nor'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                          vertical_out=False, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        en_buf_params = dict(seg_list=seg_en, pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                             vertical_out=True, dual_output=True, sig_locs={'nin0': ng0_tidx, 'nin1': ng1_tidx})

        diff_inv_params = dict(seg_buf=seg_buf, w_p=w_p, w_n=w_n, pinfo=pinfo)
        diff_inv_master = self.new_template(DiffInvCoupledChainBuf, params=diff_inv_params)

        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        ret_master = self.new_template(ClkRetimer, params=ret_params)
        en_buf_master = self.new_template(InvChainCore, params=en_buf_params)

        ff_m_params = dict(seg_dict={'tinv': seg_ff['tinv'][0], 'inv': seg_ff['inv'][0]}, pinfo=pinfo)
        ff_s_params = dict(seg_dict={'tinv': seg_ff['tinv'][1], 'inv': seg_ff['inv'][1]}, pinfo=pinfo)

        ff_m_master = self.new_template(ClkRetimer, params=ff_m_params)
        ff_s_master = self.new_template(ClkRetimer, params=ff_s_params)

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        in_vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * 4)
        cur_col = self.arr_info.get_column_span(vm_layer, in_vm_ntr)
        ff_m = self.add_tile(ff_m_master, tile_idx=0, col_idx=cur_col)
        cur_col += ff_m_master.num_cols + min_sep
        ff_s = self.add_tile(ff_s_master, tile_idx=0, col_idx=cur_col)
        cur_col += ff_s_master.num_cols + min_sep

        tap_mid_col_start = cur_col
        cur_col += min_sep
        en_buf = self.add_tile(en_buf_master, tile_idx=1, col_idx=cur_col)
        cur_col += en_buf_master.num_cols + min_sep

        nand_in_col = cur_col
        nand = self.add_tile(nand_master, tile_idx=3, col_idx=cur_col)
        nor = self.add_tile(nor_master, tile_idx=1, col_idx=cur_col)
        cur_col += min_sep

        cur_col += max(nand_master.num_cols, nor_master.num_cols)
        vss_conn_bot = self.add_substrate_contact(0, tap_mid_col_start, seg=self.num_cols - tap_mid_col_start,
                                                  tile_idx=0).to_warr_list()
        vss_conn_top = self.add_substrate_contact(0, tap_mid_col_start, seg=self.num_cols - tap_mid_col_start,
                                                  tile_idx=4).to_warr_list()
        vdd_conn = self.add_substrate_contact(0, tap_mid_col_start, seg=self.num_cols - tap_mid_col_start,
                                              tile_idx=2).to_warr_list()
        # tap_mid_col_stop = cur_col
        retimer = self.add_tile(ret_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_master.num_cols

        diff_buf = self.add_tile(diff_inv_master, tile_idx=0, col_idx=cur_col)
        self.set_mos_size(self.num_cols + min_sep)

        # === vm layer ===
        en_vm_tidx = en_buf.get_pin('outb').track_id.base_index - tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        en_vm = self.connect_to_tracks(en_buf.get_pin('nin'), TrackID(vm_layer, en_vm_tidx, tr_w_vm),
                                       min_len_mode=MinLenMode.MIDDLE)
        # Make flip flop
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3, align_idx=0)
        inn_vm, inp_vm = self.connect_matching_tracks([ff_m.get_pin('inn'), ff_m.get_pin('inp')], vm_layer,
                                                      in_vm_locs[1:], width=tr_w_vm)

        self.connect_to_track_wires(ff_m.get_pin('outn'), ff_s.get_pin('inn'))
        self.connect_to_track_wires(ff_m.get_pin('outp'), ff_s.get_pin('inp'))

        # connect nand/nor to retimer
        nandnor_vm_tidx = self.grid.coord_to_track(vm_layer, nand.get_pin('nout').upper, RoundMode.NEAREST)
        self.connect_to_tracks([nand.get_pin('nout'), nand.get_pin('pout'), retimer.get_pin('inn')],
                               TrackID(vm_layer, nandnor_vm_tidx, tr_w_vm))
        self.connect_to_tracks([nor.get_pin('nout'), nor.get_pin('pout'), retimer.get_pin('inp')],
                               TrackID(vm_layer, nandnor_vm_tidx, tr_w_vm))

        # connect nand/nor input
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3, align_idx=0,
                                               align_track=self.arr_info.col_to_track(vm_layer, nand_in_col))
        nandnor_in_list = self.connect_matching_tracks([nor.get_pin('nin<1>'), nand.get_pin('nin<1>')],
                                                       vm_layer, in_vm_locs[1:],
                                                       width=tr_manager.get_width(vm_layer, 'sig'))
        self.connect_to_track_wires(en_buf.get_pin('outb'), nor.get_pin('nin<0>'))
        self.connect_to_track_wires(en_buf.get_pin('out'), nand.get_pin('nin<0>'))

        # Retimer to output

        self.connect_differential_wires(diff_buf.get_all_port_pins('inn', layer=xm_layer),
                                        diff_buf.get_all_port_pins('inp', layer=xm_layer),
                                        retimer.get_pin('outp'), retimer.get_pin('outn'))

        # ==== xmlayer ====

        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        inn_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm.upper, RoundMode.NEAREST)
        inp_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm.lower, RoundMode.NEAREST)
        inn_xm, inp_xm = self.connect_matching_tracks([inn_vm, inp_vm], xm_layer, [inn_xm_tidx, inp_xm_tidx],
                                                      width=tr_w_clk_xm)

        bot_tile_ymid = self.get_tile_info(1)[1] + self.get_tile_info(1)[0].height // 2
        top_tile_ymid = self.get_tile_info(3)[1] + self.get_tile_info(3)[0].height // 2
        clk_xm_tidx_l = self.grid.coord_to_track(xm_layer, bot_tile_ymid, RoundMode.NEAREST)
        clk_xm_tidx_u = self.grid.coord_to_track(xm_layer, top_tile_ymid, RoundMode.NEAREST)
        sig_xm_tidx_l = clk_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_u = clk_xm_tidx_u - self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)

        en_xm_tidx = sig_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        en_xm = self.connect_to_tracks(en_vm, TrackID(xm_layer, en_xm_tidx, tr_w_sig_xm, grid=self.grid),
                                       min_len_mode=MinLenMode.MIDDLE)

        clk_xm, clkn_xm = \
            self.connect_differential_tracks([ff_m.get_pin('clkb'), ff_s.get_pin('clk'), retimer.get_pin('clk')],
                                             [ff_m.get_pin('clk'), ff_s.get_pin('clkb'), retimer.get_pin('clkb')],
                                             xm_layer, clk_xm_tidx_u, clk_xm_tidx_l, width=tr_w_clk_xm)

        # shift n mid
        shift_out_n_mid_xm = self.connect_to_tracks([ff_m.get_pin('outn'), nandnor_in_list[1]],
                                                    TrackID(xm_layer, sig_xm_tidx_l, tr_w_sig_xm, grid=self.grid))
        # shift p mid
        shift_out_p_mid_xm = self.connect_to_tracks([ff_m.get_pin('outp'), nandnor_in_list[0]],
                                                    TrackID(xm_layer, sig_xm_tidx_u, tr_w_sig_xm, grid=self.grid))

        sig_xm_tidx_l = clk_xm_tidx_l - self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_u = clk_xm_tidx_u + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        # shift n
        shift_out_n_xm = self.connect_to_tracks(ff_s.get_pin('outn'),
                                                TrackID(xm_layer, sig_xm_tidx_l, tr_w_sig_xm, grid=self.grid))
        # shift p
        shift_out_p_xm = self.connect_to_tracks(ff_s.get_pin('outp'),
                                                TrackID(xm_layer, sig_xm_tidx_u, tr_w_sig_xm, grid=self.grid))

        ym_layer = xm_layer + 1
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        sig_locs = self.params['sig_locs']
        if 'clk' in sig_locs.keys():
            clk_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols - sig_locs['clk'], RoundMode.NEAREST)
        else:
            clk_ym_tidx = self.grid.coord_to_track(ym_layer, clkn_xm.upper, RoundMode.NEAREST)
        clkb_ym_tidx = tr_manager.get_next_track(ym_layer, clk_ym_tidx, 'clk', 'clk')
        self._clk_col_alignment = self.num_cols - self.arr_info.track_to_col(ym_layer, clk_ym_tidx)
        clk_ym, clkb_ym = self.connect_differential_tracks(clk_xm, clkn_xm, ym_layer, clk_ym_tidx,
                                                           clkb_ym_tidx, width=tr_w_clk_ym)

        # === ym_layer ===
        _, input_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=inn_xm.middle)
        inn_ym, inp_ym = self.connect_matching_tracks([inn_xm, inp_xm],
                                                      ym_layer, input_ym_locs, width=tr_w_clk_ym)

        _, output_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=shift_out_n_xm.middle)
        outn_ym, outp_ym = self.connect_matching_tracks([shift_out_n_xm, shift_out_p_xm],
                                                        ym_layer, output_ym_locs, width=tr_w_clk_ym)
        tot_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols, RoundMode.NEAREST) + \
                      self.arr_info.col_to_track(ym_layer, 0, RoundMode.NEAREST)
        self._output_cols_alignment = \
            [tot_ym_tidx - diff_buf.get_pin('outn', layer=ym_layer).track_id.base_index,
             tot_ym_tidx - diff_buf.get_pin('outp', layer=ym_layer).track_id.base_index]

        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        en_ym_tidx = output_ym_locs[0] - self.get_track_sep(ym_layer, tr_w_clk_ym, tr_w_sig_ym)
        en_ym = self.connect_to_tracks(en_xm, TrackID(ym_layer, en_ym_tidx, tr_w_sig_ym, grid=self.grid))

        # === xm1 layer ===
        xm1_layer = ym_layer + 1
        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, clk_ym.upper, RoundMode.GREATER)
        clkb_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'clk', 'clk', up=1)
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        clk_xm1_u, clkb_xm1_u = self.connect_matching_tracks([clk_ym, clkb_ym], xm1_layer,
                                                             [clk_xm1_tidx, clkb_xm1_tidx], width=tr_w_clk_xm1)

        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, clk_ym.lower, RoundMode.GREATER)
        clkb_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'clk', 'clk', up=-1)
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        clk_xm1_l, clkb_xm1_l = self.connect_matching_tracks([clk_ym, clkb_ym], xm1_layer,
                                                             [clk_xm1_tidx, clkb_xm1_tidx], width=tr_w_clk_xm1)

        inn_xm1_tidx = self.grid.coord_to_track(xm1_layer, inn_ym.upper, RoundMode.GREATER)
        inp_xm1_tidx = tr_manager.get_next_track(xm1_layer, inn_xm1_tidx, 'sig', 'sig', up=1)
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        inn_xm1, inp_xm1 = self.connect_matching_tracks([inn_ym, inp_ym], xm1_layer,
                                                        [inn_xm1_tidx, inp_xm1_tidx], width=tr_w_sig_xm1)

        # outn_xm1_tidx = self.grid.coord_to_track(xm1_layer, outn_ym.lower, RoundMode.GREATER)
        outp_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'sig', 'sig', up=1)
        outn_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'sig', 'sig', up=2)
        outn_xm1, outp_xm1 = self.connect_matching_tracks([outn_ym, outp_ym], xm1_layer,
                                                          [outn_xm1_tidx, outp_xm1_tidx], width=tr_w_sig_xm1)

        en_xm1_tidx = inn_xm1_tidx - (inp_xm1_tidx - inn_xm1_tidx)
        en_xm1 = self.connect_to_tracks(en_ym, TrackID(xm1_layer, en_xm1_tidx, tr_w_clk_xm1))
        # === supply ===
        vdd_hm = self.connect_to_track_wires(vdd_conn, retimer.get_pin('VDD'))
        vss_hm_bot = min(retimer.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_top = max(retimer.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_list = [self.connect_to_track_wires(vss_conn_bot, vss_hm_bot),
                       self.connect_to_track_wires(vss_conn_top, vss_hm_top)]

        vdd_hm = self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_bot = self.extend_wires(vss_hm_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_top = self.extend_wires(vss_hm_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        self.connect_to_track_wires(nor.get_all_port_pins('VDD') + nand.get_all_port_pins('VDD') +
                                    en_buf.get_all_port_pins('VDD'), vdd_hm)
        self.connect_to_track_wires(vss_hm_list[0], nor.get_all_port_pins('VSS') + en_buf.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vss_hm_list[1], nand.get_all_port_pins('VSS'))

        vdd_xm = self.export_tap_hm(tr_manager, vdd_hm[0], hm_layer, xm_layer)
        vss_xm = [self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0] for vss in [vss_hm_top, vss_hm_bot]]

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        vdd_xm1 = self.export_tap_hm(tr_manager, vdd_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_bot = self.export_tap_hm(tr_manager, vss_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_top = self.export_tap_hm(tr_manager, vss_xm[1], xm_layer, xm1_layer)[0]
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', [vss_xm1_bot, vss_xm1_top], label='VSS', show=self.show_pins)
        fill_conn_layer_intv(self, 1, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 1, 1, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 1, stop_col=nand_in_col)

        for pinname in diff_buf.port_names_iter():
            if 'mid' in pinname:
                self.reexport(diff_buf.get_port(pinname))

        self.add_pin('shift_inn', inn_xm1)
        self.add_pin('shift_inp', inp_xm1)
        self.add_pin('shift_outn', outn_xm1)
        self.add_pin('shift_outp', outp_xm1)
        self.add_pin('clkn', [clkb_xm1_l, clkb_xm1_u])
        self.add_pin('clkp', [clk_xm1_l, clk_xm1_u])
        self.add_pin('en', en_xm1)
        self.reexport(diff_buf.get_port('outn'))
        self.reexport(diff_buf.get_port('outp'))
        self.reexport(diff_buf.get_port('inn'), net_name='retn')
        self.reexport(diff_buf.get_port('inp'), net_name='retp')
        self.reexport(diff_buf.get_port('inn_vm'), net_name='retn_vm', hide=True)
        self.reexport(diff_buf.get_port('inp_vm'), net_name='retp_vm', hide=True)

        sch_params_dict = dict(
            nand_params=nand_master.sch_params,
            nor_params=nor_master.sch_params,
            ret_params=ret_master.sch_params,
            ff_m_params=ff_m_master.sch_params,
            ff_s_params=ff_s_master.sch_params,
            enbuf_params=en_buf_master.sch_params,
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
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_retimer_amp')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_ret='Number of segments.',
            seg_buf='',
            seg_nand='',
            seg_nor='',
            seg_nand_amp='',
            seg_nor_amp='',
            seg_en='',
            seg_ff='',
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

    def get_rel_track_index(self, row_idx, wire_type, wire_name, wire_idx, tile_idx):
        hm_layer = self.conn_layer + 1
        yb = self.get_tile_info(tile_idx)[1]
        abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                        tile_idx=tile_idx)
        abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
        rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
        return rel_tidx

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_ret: Dict[str, Any] = self.params['seg_ret']
        seg_buf: Dict[str, Any] = self.params['seg_buf']
        seg_en = self.params['seg_en']
        seg_ff = self.params['seg_ff']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        # Make templates
        ng0_tidx = self.get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)
        ng1_tidx = self.get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)

        nand_params = dict(seg=self.params['seg_nand'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                           vertical_out=False, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        nor_params = dict(seg=self.params['seg_nor'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                          vertical_out=False, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        en_buf_params = dict(seg_list=seg_en, pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                             vertical_out=True, dual_output=True, sig_locs={'nin0': ng0_tidx, 'nin1': ng1_tidx})

        diff_inv_params = dict(seg_buf=seg_buf, pinfo=pinfo)
        diff_inv_master = self.new_template(DiffInvCoupledChainBuf, params=diff_inv_params)

        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        en_buf_master = self.new_template(InvChainCore, params=en_buf_params)

        ff_m_params = dict(seg_dict={'tinv': seg_ff['tinv'][0], 'inv': seg_ff['inv'][0]}, pinfo=pinfo)
        ff_s_params = dict(seg_dict={'tinv': seg_ff['tinv'][1], 'inv': seg_ff['inv'][1]}, pinfo=pinfo)
        ret_m_params = dict(seg_dict={'tinv': seg_ret['tinv'][0], 'inv': seg_ret['inv'][0]}, pinfo=pinfo)
        ret_s_params = dict(seg_dict={'tinv': seg_ret['tinv'][1], 'inv': seg_ret['inv'][1]}, pinfo=pinfo)

        ret_m_master = self.new_template(ClkRetimer, params=ret_m_params)
        ret_s_master = self.new_template(ClkRetimer, params=ret_s_params)
        ff_m_master = self.new_template(ClkRetimer, params=ff_m_params)
        ff_s_master = self.new_template(ClkRetimer, params=ff_s_params)

        nand_amp_params = dict(seg=self.params['seg_nand_amp'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                               vertical_out=True, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        nor_amp_params = dict(seg=self.params['seg_nor_amp'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                              vertical_out=True, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})

        nand_amp_master = self.new_template(NAND2Core, params=nand_amp_params)
        nor_amp_master = self.new_template(NOR2Core, params=nor_amp_params)

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        in_vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * 4)
        cur_col = self.arr_info.get_column_span(vm_layer, in_vm_ntr)
        ff_m = self.add_tile(ff_m_master, tile_idx=0, col_idx=cur_col)
        cur_col += ff_m_master.num_cols + min_sep
        ff_s = self.add_tile(ff_s_master, tile_idx=0, col_idx=cur_col)
        cur_col += ff_s_master.num_cols + min_sep

        tap_mid_col_start = cur_col
        cur_col += min_sep
        en_buf = self.add_tile(en_buf_master, tile_idx=1, col_idx=cur_col)
        cur_col += en_buf_master.num_cols + min_sep

        nand_in_col = cur_col
        nand = self.add_tile(nand_master, tile_idx=3, col_idx=cur_col)
        nor = self.add_tile(nor_master, tile_idx=1, col_idx=cur_col)
        cur_col += min_sep

        cur_col += max(nand_master.num_cols, nor_master.num_cols)
        vss_conn_bot = self.add_substrate_contact(0, tap_mid_col_start, seg=self.num_cols - tap_mid_col_start,
                                                  tile_idx=0).to_warr_list()
        vss_conn_top = self.add_substrate_contact(0, tap_mid_col_start, seg=self.num_cols - tap_mid_col_start,
                                                  tile_idx=4).to_warr_list()
        vdd_conn = self.add_substrate_contact(0, tap_mid_col_start, seg=self.num_cols - tap_mid_col_start,
                                              tile_idx=2).to_warr_list()
        # tap_mid_col_stop = cur_col
        retimer_m = self.add_tile(ret_m_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_m_master.num_cols + min_sep
        retimer_s = self.add_tile(ret_s_master, tile_idx=0, col_idx=cur_col)
        cur_col += ret_s_master.num_cols + min_sep

        nand_amp_in_col = cur_col
        nand_amp = self.add_tile(nand_amp_master, tile_idx=1, col_idx=cur_col)
        nor_amp = self.add_tile(nor_amp_master, tile_idx=3, col_idx=cur_col)
        cur_col += max(nand_amp_master.num_cols, nor_amp_master.num_cols) + min_sep

        tap_ncol = max(nand_amp_master.num_cols, nor_amp_master.num_cols) + min_sep
        vss_conn_bot += self.add_substrate_contact(0, nand_amp_in_col, seg=tap_ncol, tile_idx=0).to_warr_list()
        vss_conn_top += self.add_substrate_contact(0, nand_amp_in_col, seg=tap_ncol, tile_idx=4).to_warr_list()
        vdd_conn += self.add_substrate_contact(0, nand_amp_in_col, seg=tap_ncol, tile_idx=2).to_warr_list()

        diff_buf = self.add_tile(diff_inv_master, tile_idx=0, col_idx=cur_col)
        self.set_mos_size(self.num_cols + min_sep)

        # === vm layer ===
        en_vm_tidx = en_buf.get_pin('outb').track_id.base_index - tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        en_vm = self.connect_to_tracks(en_buf.get_pin('nin'), TrackID(vm_layer, en_vm_tidx, tr_w_vm),
                                       min_len_mode=MinLenMode.MIDDLE)
        # Make flip flop
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3, align_idx=0)
        inn_vm, inp_vm = self.connect_matching_tracks([ff_m.get_pin('inn'), ff_m.get_pin('inp')], vm_layer,
                                                      in_vm_locs[1:], width=tr_w_vm)

        self.connect_to_track_wires(ff_m.get_pin('outn'), ff_s.get_pin('inn'))
        self.connect_to_track_wires(ff_m.get_pin('outp'), ff_s.get_pin('inp'))

        # connect nand/nor to retimer
        nandnor_vm_tidx = self.grid.coord_to_track(vm_layer, nand.get_pin('nout').upper, RoundMode.NEAREST)
        self.connect_to_tracks([nand.get_pin('nout'), nand.get_pin('pout'), retimer_m.get_pin('inn')],
                               TrackID(vm_layer, nandnor_vm_tidx, tr_w_vm))
        self.connect_to_tracks([nor.get_pin('nout'), nor.get_pin('pout'), retimer_m.get_pin('inp')],
                               TrackID(vm_layer, nandnor_vm_tidx, tr_w_vm))
        self.connect_to_track_wires(retimer_m.get_pin('outn'), retimer_s.get_pin('inn'))
        self.connect_to_track_wires(retimer_m.get_pin('outp'), retimer_s.get_pin('inp'))

        # connect nand/nor input
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3, align_idx=0,
                                               align_track=self.arr_info.col_to_track(vm_layer, nand_in_col))
        nandnor_in_list = self.connect_matching_tracks([nor.get_pin('nin<1>'), nand.get_pin('nin<1>')],
                                                       vm_layer, in_vm_locs[1:],
                                                       width=tr_manager.get_width(vm_layer, 'sig'))
        self.connect_to_track_wires(en_buf.get_pin('outb'), nor.get_pin('nin<0>'))
        self.connect_to_track_wires(en_buf.get_pin('out'), nand.get_pin('nin<0>'))

        # Retimer to output
        self.connect_to_track_wires(diff_buf.get_all_port_pins('inn', layer=xm_layer), nor_amp.get_pin('out'))
        self.connect_to_track_wires(diff_buf.get_all_port_pins('inp', layer=xm_layer), nand_amp.get_pin('out'))

        nand_amp_vm_tidx_l = tr_manager.get_next_track(vm_layer, nand_amp.get_pin('out').track_id.base_index,
                                                       'sig', 'sig', up=-1)
        nand_amp_vm_tidx_u = tr_manager.get_next_track(vm_layer, nand_amp.get_pin('out').track_id.base_index,
                                                       'sig', 'sig', up=1)
        nor_amp_vm_tidx_l = tr_manager.get_next_track(vm_layer, nor_amp.get_pin('out').track_id.base_index,
                                                      'sig', 'sig', up=-1)
        nor_amp_vm_tidx_u = tr_manager.get_next_track(vm_layer, nor_amp.get_pin('out').track_id.base_index,
                                                      'sig', 'sig', up=1)

        nand_amp_in0_vm = self.connect_to_tracks(nand_amp.get_pin('nin<0>'),
                                                 TrackID(vm_layer, nand_amp_vm_tidx_l, tr_w_vm),
                                                 min_len_mode=MinLenMode.MIDDLE)
        nand_amp_in1_vm = self.connect_to_tracks(nand_amp.get_pin('nin<1>'),
                                                 TrackID(vm_layer, nand_amp_vm_tidx_u, tr_w_vm),
                                                 min_len_mode=MinLenMode.MIDDLE)

        nor_amp_in0_vm = self.connect_to_tracks(nor_amp.get_pin('nin<0>'),
                                                TrackID(vm_layer, nor_amp_vm_tidx_l, tr_w_vm),
                                                min_len_mode=MinLenMode.MIDDLE)
        nor_amp_in1_vm = self.connect_to_tracks(nor_amp.get_pin('nin<1>'),
                                                TrackID(vm_layer, nor_amp_vm_tidx_u, tr_w_vm),
                                                min_len_mode=MinLenMode.MIDDLE)

        # ==== xmlayer ====

        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        inn_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm.upper, RoundMode.NEAREST)
        inp_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm.lower, RoundMode.NEAREST)
        inn_xm, inp_xm = self.connect_matching_tracks([inn_vm, inp_vm], xm_layer, [inn_xm_tidx, inp_xm_tidx],
                                                      width=tr_w_clk_xm)

        bot_tile_ymid = self.get_tile_info(1)[1] + self.get_tile_info(1)[0].height // 2
        top_tile_ymid = self.get_tile_info(3)[1] + self.get_tile_info(3)[0].height // 2
        clk_xm_tidx_l = self.grid.coord_to_track(xm_layer, bot_tile_ymid, RoundMode.NEAREST)
        clk_xm_tidx_u = self.grid.coord_to_track(xm_layer, top_tile_ymid, RoundMode.NEAREST)
        sig_xm_tidx_l = clk_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_u = clk_xm_tidx_u - self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_ll = sig_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_sig_xm, tr_w_sig_xm)
        sig_xm_tidx_uu = sig_xm_tidx_u - self.get_track_sep(xm_layer, tr_w_sig_xm, tr_w_sig_xm)

        en_xm_tidx = sig_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        en_xm = self.connect_to_tracks(en_vm, TrackID(xm_layer, en_xm_tidx, tr_w_sig_xm, grid=self.grid),
                                       min_len_mode=MinLenMode.MIDDLE)

        clk_xm, clkn_xm = \
            self.connect_differential_tracks([ff_m.get_pin('clkb'), ff_s.get_pin('clk'),
                                              retimer_m.get_pin('clk'), retimer_s.get_pin('clkb')],
                                             [ff_m.get_pin('clk'), ff_s.get_pin('clkb'),
                                              retimer_m.get_pin('clkb'), retimer_s.get_pin('clk')],
                                             xm_layer, clk_xm_tidx_u, clk_xm_tidx_l, width=tr_w_clk_xm)

        # shift n mid
        shift_out_n_mid_xm = self.connect_to_tracks([ff_m.get_pin('outp'), nandnor_in_list[0]],
                                                    TrackID(xm_layer, sig_xm_tidx_l, tr_w_sig_xm, grid=self.grid))
        # shift p mid
        shift_out_p_mid_xm = self.connect_to_tracks([ff_m.get_pin('outn'), nandnor_in_list[1]],
                                                    TrackID(xm_layer, sig_xm_tidx_u, tr_w_sig_xm, grid=self.grid))

        sig_xm_tidx_l = clk_xm_tidx_l - self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_u = clk_xm_tidx_u + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        # shift n
        shift_out_n_xm = self.connect_to_tracks(ff_s.get_pin('outn'),
                                                TrackID(xm_layer, sig_xm_tidx_l, tr_w_sig_xm, grid=self.grid))
        # shift p
        shift_out_p_xm = self.connect_to_tracks(ff_s.get_pin('outp'),
                                                TrackID(xm_layer, sig_xm_tidx_u, tr_w_sig_xm, grid=self.grid))
        ym_layer = xm_layer + 1
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        sig_locs = self.params['sig_locs']
        if 'clk' in sig_locs.keys():
            clk_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols - sig_locs['clk'], RoundMode.NEAREST)
        else:
            clk_ym_tidx = self.grid.coord_to_track(ym_layer, clkn_xm.upper, RoundMode.NEAREST)
        clkb_ym_tidx = tr_manager.get_next_track(ym_layer, clk_ym_tidx, 'clk', 'clk')
        self._clk_col_alignment = self.num_cols - self.arr_info.track_to_col(ym_layer, clk_ym_tidx)
        clk_ym, clkb_ym = self.connect_differential_tracks(clk_xm, clkn_xm, ym_layer, clk_ym_tidx,
                                                           clkb_ym_tidx, width=tr_w_clk_ym)
        self.connect_to_tracks([nor_amp_in0_vm, retimer_s.get_pin('outp')],
                               TrackID(xm_layer, sig_xm_tidx_u, tr_w_sig_xm, grid=self.grid))
        self.connect_to_tracks([nor_amp_in1_vm, retimer_m.get_pin('outn')],
                               TrackID(xm_layer, sig_xm_tidx_uu, tr_w_sig_xm, grid=self.grid))
        self.connect_to_tracks([nand_amp_in0_vm, retimer_s.get_pin('outn')],
                               TrackID(xm_layer, sig_xm_tidx_l, tr_w_sig_xm, grid=self.grid))
        self.connect_to_tracks([nand_amp_in1_vm, retimer_m.get_pin('outp')],
                               TrackID(xm_layer, sig_xm_tidx_ll, tr_w_sig_xm, grid=self.grid))

        # === ym_layer ===
        _, input_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=inn_xm.middle)
        inn_ym, inp_ym = self.connect_matching_tracks([inn_xm, inp_xm],
                                                      ym_layer, input_ym_locs, width=tr_w_clk_ym)

        _, output_ym_locs = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=shift_out_n_xm.middle)
        outn_ym, outp_ym = self.connect_matching_tracks([shift_out_n_xm, shift_out_p_xm],
                                                        ym_layer, output_ym_locs, width=tr_w_clk_ym)
        tot_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols, RoundMode.NEAREST) + \
                      self.arr_info.col_to_track(ym_layer, 0, RoundMode.NEAREST)
        self._output_cols_alignment = \
            [tot_ym_tidx - diff_buf.get_pin('outn', layer=ym_layer).track_id.base_index,
             tot_ym_tidx - diff_buf.get_pin('outp', layer=ym_layer).track_id.base_index]

        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        en_ym_tidx = output_ym_locs[0] - self.get_track_sep(ym_layer, tr_w_clk_ym, tr_w_sig_ym)
        en_ym = self.connect_to_tracks(en_xm, TrackID(ym_layer, en_ym_tidx, tr_w_sig_ym, grid=self.grid))
        # === xm1 layer ===

        xm1_layer = ym_layer + 1
        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, clk_ym.upper, RoundMode.LESS)
        clkb_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'clk', 'clk', up=1)
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        clk_xm1_u, clkb_xm1_u = self.connect_matching_tracks([clk_ym, clkb_ym], xm1_layer,
                                                             [clk_xm1_tidx, clkb_xm1_tidx], width=tr_w_clk_xm1)

        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, clk_ym.lower, RoundMode.GREATER)
        clkb_xm1_tidx = tr_manager.get_next_track(xm1_layer, clk_xm1_tidx, 'clk', 'clk', up=-1)
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        clk_xm1_l, clkb_xm1_l = self.connect_matching_tracks([clk_ym, clkb_ym], xm1_layer,
                                                             [clk_xm1_tidx, clkb_xm1_tidx], width=tr_w_clk_xm1)

        inn_xm1_tidx = self.grid.coord_to_track(xm1_layer, inn_ym.upper, RoundMode.LESS)
        inp_xm1_tidx = tr_manager.get_next_track(xm1_layer, inn_xm1_tidx, 'sig', 'sig', up=1)
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        inn_xm1, inp_xm1 = self.connect_matching_tracks([inn_ym, inp_ym], xm1_layer,
                                                        [inn_xm1_tidx, inp_xm1_tidx], width=tr_w_sig_xm1)

        outn_xm1_tidx = self.grid.coord_to_track(xm1_layer, outn_ym.lower, RoundMode.GREATER)
        outp_xm1_tidx = tr_manager.get_next_track(xm1_layer, outn_xm1_tidx, 'sig', 'sig', up=-1)
        outn_xm1, outp_xm1 = self.connect_matching_tracks([outn_ym, outp_ym], xm1_layer,
                                                          [outn_xm1_tidx, outp_xm1_tidx], width=tr_w_sig_xm1)
        en_xm1_tidx = inn_xm1_tidx - (inp_xm1_tidx - inn_xm1_tidx)
        en_xm1 = self.connect_to_tracks(en_ym, TrackID(xm1_layer, en_xm1_tidx, tr_w_clk_xm1))

        # === supply ===
        vdd_hm = self.connect_to_track_wires(vdd_conn, retimer_m.get_pin('VDD'))
        vss_hm_bot = min(retimer_m.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_top = max(retimer_m.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        vss_hm_list = [self.connect_to_track_wires(vss_conn_bot, vss_hm_bot),
                       self.connect_to_track_wires(vss_conn_top, vss_hm_top)]

        vdd_hm = self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_bot = self.extend_wires(vss_hm_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_top = self.extend_wires(vss_hm_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]

        # enable gates supply
        self.connect_to_track_wires(nor.get_all_port_pins('VDD') + nand.get_all_port_pins('VDD') +
                                    en_buf.get_all_port_pins('VDD'), vdd_hm)
        self.connect_to_track_wires(vss_hm_list[0], nor.get_all_port_pins('VSS') + en_buf.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vss_hm_list[1], nand.get_all_port_pins('VSS'))
        # amp gates supply
        self.connect_to_track_wires(nor_amp.get_all_port_pins('VDD') + nand_amp.get_all_port_pins('VDD'), vdd_hm)
        self.connect_to_track_wires(vss_hm_list[1], nor_amp.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vss_hm_list[0], nand_amp.get_all_port_pins('VSS'))

        vdd_xm = self.export_tap_hm(tr_manager, vdd_hm[0], hm_layer, xm_layer)
        vss_xm = [self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0] for vss in [vss_hm_top, vss_hm_bot]]

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        vdd_xm1 = self.export_tap_hm(tr_manager, vdd_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_bot = self.export_tap_hm(tr_manager, vss_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_top = self.export_tap_hm(tr_manager, vss_xm[1], xm_layer, xm1_layer)[0]
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', [vss_xm1_bot, vss_xm1_top], label='VSS', show=self.show_pins)
        fill_conn_layer_intv(self, 1, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 1, 1, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 1, stop_col=nand_in_col)

        # for pinname in diff_buf.port_names_iter():
        #     if 'mid' in pinname:
        #         self.reexport(diff_buf.get_port(pinname))

        self.add_pin('shift_inn', inn_xm1)
        self.add_pin('shift_inp', inp_xm1)
        self.add_pin('shift_outn', outn_xm1)
        self.add_pin('shift_outp', outp_xm1)
        self.add_pin('clkn', [clkb_xm1_l, clkb_xm1_u])
        self.add_pin('clkp', [clk_xm1_l, clk_xm1_u])
        self.add_pin('en', en_xm1)
        self.reexport(diff_buf.get_port('outn'))
        self.reexport(diff_buf.get_port('outp'))

        sch_params_dict = dict(
            nand_params=nand_master.sch_params,
            nor_params=nor_master.sch_params,
            nand_amp_params=nand_amp_master.sch_params,
            nor_amp_params=nor_amp_master.sch_params,
            ret_params=[ret_m_master.sch_params, ret_s_master.sch_params],
            ff_m_params=ff_m_master.sch_params,
            ff_s_params=ff_s_master.sch_params,
            enbuf_params=en_buf_master.sch_params,
            buf_params=diff_inv_master.sch_params,
        )
        self.sch_params = sch_params_dict


class ClkLocalHold(MOSBase, TemplateBaseZL):
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
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_retimer_hold')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
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

    def get_rel_track_index(self, row_idx, wire_type, wire_name, wire_idx, tile_idx):
        hm_layer = self.conn_layer + 1
        yb = self.get_tile_info(tile_idx)[1]
        abs_tidx = self.get_track_index(row_idx, wire_type, wire_name=wire_name, wire_idx=wire_idx,
                                        tile_idx=tile_idx)
        abs_coord = self.grid.track_to_coord(hm_layer, abs_tidx)
        rel_tidx = self.grid.coord_to_track(hm_layer, abs_coord - yb, RoundMode.NEAREST)
        return rel_tidx

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

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        # Make templates
        ng0_tidx = self.get_rel_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1, tile_idx=1)
        ng1_tidx = self.get_rel_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0, tile_idx=1)

        diff_inv_params = dict(seg_buf=seg_buf, pinfo=pinfo)
        diff_inv_master = self.new_template(DiffInvCoupledChainBuf, params=diff_inv_params)

        nand_params = dict(seg=self.params['seg_nand'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                           vertical_out=True, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})
        nor_params = dict(seg=self.params['seg_nor'], pinfo=self.get_tile_pinfo(1), vertical_sup=True,
                          vertical_out=True, sig_locs={'nin1': ng0_tidx, 'nin0': ng1_tidx})

        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        in_vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * 4)
        cur_col = self.arr_info.get_column_span(vm_layer, in_vm_ntr)

        nand_in_col = cur_col
        nand = self.add_tile(nand_master, tile_idx=3, col_idx=cur_col)
        nor = self.add_tile(nor_master, tile_idx=1, col_idx=cur_col)
        cur_col += max(nand_master.num_cols, nor_master.num_cols) + min_sep

        tap_ncol = max(nand_master.num_cols, nor_master.num_cols) + min_sep
        vss_conn_bot = self.add_substrate_contact(0, 0, seg=tap_ncol + nand_in_col, tile_idx=0).to_warr_list()
        vss_conn_top = self.add_substrate_contact(0, 0, seg=tap_ncol + nand_in_col, tile_idx=4).to_warr_list()
        vdd_conn = self.add_substrate_contact(0, 0, seg=tap_ncol + nand_in_col, tile_idx=2).to_warr_list()

        diff_buf = self.add_tile(diff_inv_master, tile_idx=0, col_idx=cur_col)
        self.set_mos_size(self.num_cols + min_sep)

        # === vm layer ===

        # connect nand/nor input
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 5, align_idx=-1,
                                               align_track=self.arr_info.col_to_track(vm_layer, nand_in_col))
        nandnor_in_list = self.connect_matching_tracks([nor.get_pin('nin<1>'), nor.get_pin('nin<0>'),
                                                        nand.get_pin('nin<1>'), nand.get_pin('nin<0>')],
                                                       vm_layer, in_vm_locs[:-1],
                                                       width=tr_manager.get_width(vm_layer, 'sig'))

        self.connect_to_track_wires(diff_buf.get_all_port_pins('inp', layer=xm_layer), nor.get_pin('out'))
        self.connect_to_track_wires(diff_buf.get_all_port_pins('inn', layer=xm_layer), nand.get_pin('out'))

        # ==== xmlayer ====

        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        #
        bot_tile_ymid = self.get_tile_info(1)[1] + self.get_tile_info(1)[0].height // 2
        top_tile_ymid = self.get_tile_info(3)[1] + self.get_tile_info(3)[0].height // 2
        clk_xm_tidx_l = self.grid.coord_to_track(xm_layer, bot_tile_ymid, RoundMode.NEAREST)
        clk_xm_tidx_u = self.grid.coord_to_track(xm_layer, top_tile_ymid, RoundMode.NEAREST)
        sig_xm_tidx_l = clk_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_u = clk_xm_tidx_u - self.get_track_sep(xm_layer, tr_w_clk_xm, tr_w_sig_xm)
        sig_xm_tidx_ll = sig_xm_tidx_l + self.get_track_sep(xm_layer, tr_w_sig_xm, tr_w_sig_xm)
        sig_xm_tidx_uu = sig_xm_tidx_u - self.get_track_sep(xm_layer, tr_w_sig_xm, tr_w_sig_xm)
        #
        ym_layer = xm_layer + 1
        nandnor_in_list_xm = [
            self.connect_to_tracks(nandnor_in_list[0],
                                   TrackID(xm_layer, sig_xm_tidx_l, tr_w_sig_xm, grid=self.grid)),
            self.connect_to_tracks(nandnor_in_list[1],
                                   TrackID(xm_layer, sig_xm_tidx_ll, tr_w_sig_xm, grid=self.grid)),
            self.connect_to_tracks(nandnor_in_list[2],
                                   TrackID(xm_layer, sig_xm_tidx_u, tr_w_sig_xm, grid=self.grid)),
            self.connect_to_tracks(nandnor_in_list[3],
                                   TrackID(xm_layer, sig_xm_tidx_uu, tr_w_sig_xm, grid=self.grid))]

        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        nandnor_in_list_xm = self.match_warr_length(nandnor_in_list_xm)
        _, tr_w_ym_locs = tr_manager.place_wires(ym_layer, ['sig'] * 4,
                                                 align_track=self.grid.coord_to_track(ym_layer, self.bound_box.xl,
                                                                                      RoundMode.NEAREST))
        nandnor_in_list_ym = self.connect_matching_tracks(nandnor_in_list_xm, ym_layer,
                                                          tr_w_ym_locs, width=tr_w_sig_ym)
        # # === ym_layer ===

        xm1_layer = ym_layer + 1
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        _, tr_w_xm1_locs = tr_manager.place_wires(xm1_layer, ['sig'] * 5, align_idx=2,
                                                  align_track=self.grid.coord_to_track(xm1_layer,
                                                                                       nandnor_in_list_ym[0].upper,
                                                                                       RoundMode.NEAREST))
        nandnor_in_list_xm1 = self.connect_matching_tracks(nandnor_in_list_ym, xm1_layer,
                                                           tr_w_xm1_locs[1:], width=tr_w_sig_xm1)

        #
        # === supply ===
        vdd_hm = self.connect_to_track_wires(vdd_conn, diff_buf.get_pin('VDD_hm'))
        vss_hm_bot = min(diff_buf.get_all_port_pins('VSS_hm'), key=lambda x: x.track_id.base_index)
        vss_hm_top = max(diff_buf.get_all_port_pins('VSS_hm'), key=lambda x: x.track_id.base_index)
        vss_hm_list = [self.connect_to_track_wires(vss_conn_bot, vss_hm_bot),
                       self.connect_to_track_wires(vss_conn_top, vss_hm_top)]
        #
        vdd_hm = self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_bot = self.extend_wires(vss_hm_bot, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]
        vss_hm_top = self.extend_wires(vss_hm_top, lower=self.bound_box.xl, upper=self.bound_box.xh)[0]

        # amp gates supply
        self.connect_to_track_wires(nor.get_all_port_pins('VDD') + nand.get_all_port_pins('VDD'), vdd_hm)
        self.connect_to_track_wires(vss_hm_list[1], nand.get_all_port_pins('VSS'))
        self.connect_to_track_wires(vss_hm_list[0], nor.get_all_port_pins('VSS'))

        vdd_xm = self.export_tap_hm(tr_manager, vdd_hm[0], hm_layer, xm_layer)
        vss_xm = [self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0] for vss in [vss_hm_top, vss_hm_bot]]

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        vdd_xm1 = self.export_tap_hm(tr_manager, vdd_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_bot = self.export_tap_hm(tr_manager, vss_xm[0], xm_layer, xm1_layer)[0]
        vss_xm1_top = self.export_tap_hm(tr_manager, vss_xm[1], xm_layer, xm1_layer)[0]
        vdd_xm1 = self.extend_wires(vdd_xm1, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm1 = self.extend_wires([vss_xm1_bot, vss_xm1_top], lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm1', vss_xm1, label='VSS', show=self.show_pins)
        fill_conn_layer_intv(self, 1, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 1, 1, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 0, stop_col=nand_in_col)
        fill_conn_layer_intv(self, 3, 1, stop_col=nand_in_col)

        self.add_pin('sam', nandnor_in_list_xm1[0], mode=PinMode.LOWER)
        self.add_pin('sam_e', nandnor_in_list_xm1[1], mode=PinMode.LOWER)
        self.add_pin('sam_b', nandnor_in_list_xm1[2], mode=PinMode.LOWER)
        self.add_pin('sam_e_b', nandnor_in_list_xm1[3], mode=PinMode.LOWER)
        self.reexport(diff_buf.get_port('outn'))
        self.reexport(diff_buf.get_port('outp'))

        sch_params_dict = dict(
            nand_params=nand_master.sch_params,
            nor_params=nor_master.sch_params,
            buf_params=diff_inv_master.sch_params,
        )
        self.sch_params = sch_params_dict


class ClkLocalTop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_local_top')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            clk_local_params='Parameters for buffer x',
            clk_local_buf_params='',
            clk_delay_params='',
            clk_local_vco_params='',
            clk_local_hold_params='',
            clk_local_amp_params='',
            flip_phase_inout='',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(flip_phase_inout=False)

    def draw_layout(self) -> None:
        clk_delay_params: Param = self.params['clk_delay_params']
        clk_local_params: Param = self.params['clk_local_params']
        clk_local_buf_params: Param = self.params['clk_local_buf_params']
        clk_local_vco_params: Param = self.params['clk_local_vco_params']
        clk_local_hold_params: Param = self.params['clk_local_hold_params']
        clk_local_amp_params: Param = self.params['clk_local_amp_params']

        if isinstance(clk_delay_params, str):
            spec_yaml = read_yaml(clk_delay_params)
            clk_delay_params = spec_yaml['params']
        if isinstance(clk_local_params, str):
            spec_yaml = read_yaml(clk_local_params)
            clk_local_params = spec_yaml['params']
        if isinstance(clk_local_buf_params, str):
            spec_yaml = read_yaml(clk_local_buf_params)
            clk_local_buf_params = spec_yaml['params']
        if isinstance(clk_local_vco_params, str):
            spec_yaml = read_yaml(clk_local_vco_params)
            clk_local_vco_params = spec_yaml['params']
        if isinstance(clk_local_hold_params, str):
            spec_yaml = read_yaml(clk_local_hold_params)
            clk_local_hold_params = spec_yaml['params']
        if isinstance(clk_local_amp_params, str):
            spec_yaml = read_yaml(clk_local_amp_params)
            clk_local_amp_params = spec_yaml['params']

        # for params in [clk_local_params, clk_local_buf_params, clk_local_vco_params,
        #                clk_local_amp_params]:
        for params in [clk_local_params, clk_local_buf_params, clk_local_hold_params, clk_local_vco_params,
                       clk_local_amp_params]:
            params['export_private'] = False
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        # self._tr_manager = tr_manager

        clk_delay_master = self.new_template(ClkDelay, params=clk_delay_params)
        clk_local_params['export_hidden'] = True
        clk_local_sam_master = self.new_template(GenericWrapper, params=clk_local_params)
        # clk_local_vco_params['params']['sig_locs'] = {'clk': clk_local_sam_master.core.clk_col_alignment}
        # clk_local_amp_params['params']['sig_locs'] = {'clk': clk_local_sam_master.core.clk_col_alignment}
        clk_local_buf_params['params']['sig_locs'] = {'inn': clk_local_sam_master.core.output_cols_alignment[0],
                                                      'inp': clk_local_sam_master.core.output_cols_alignment[1]}
        clk_local_buf_master = self.new_template(GenericWrapper, params=clk_local_buf_params)
        clk_local_hold_master = self.new_template(GenericWrapper, params=clk_local_hold_params)
        clk_local_vco_master = self.new_template(GenericWrapper, params=clk_local_vco_params)
        clk_local_amp_master = self.new_template(GenericWrapper, params=clk_local_amp_params)

        # --- Placement --- #

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      clk_local_buf_params['params']['pinfo']['tile_specs']
                                                      ['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        top_layer = max(clk_local_buf_master.top_layer, clk_local_amp_master.top_layer,
                        clk_local_sam_master.top_layer, clk_local_vco_master.top_layer,
                        ym1_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        buf_w, buf_h = clk_local_buf_master.bound_box.w, clk_local_buf_master.bound_box.h
        delay_w, delay_h = clk_delay_master.bound_box.w, clk_delay_master.bound_box.h
        sam_w, sam_h = clk_local_sam_master.bound_box.w, clk_local_sam_master.bound_box.h
        vco_w, vco_h = clk_local_vco_master.bound_box.w, clk_local_vco_master.bound_box.h
        amp_w, amp_h = clk_local_amp_master.bound_box.w, clk_local_amp_master.bound_box.h
        hold_w, hold_h = clk_local_hold_master.bound_box.w, clk_local_hold_master.bound_box.h

        # delay_shift = clk_delay_master.core.arr_info.col_to_coord(clk_delay_params['ndum_side'])
        # delay_x = (-delay_shift // blk_w) * blk_w
        # buf_y = -(-dvelay_h // blk_h) * blk_h
        hold_y = 2 * blk_h
        hold_x = -(-(buf_w - hold_w) // blk_w) * blk_w
        hold = self.add_instance(clk_local_hold_master, xform=Transform(hold_x, hold_y, Orientation.R0))
        buf_y = -(-hold.bound_box.yh // blk_h) * blk_h + 2 * blk_h
        buf_x = 0
        buf = self.add_instance(clk_local_buf_master, xform=Transform(buf_x, buf_y, Orientation.R0))
        local_y = -(-buf.bound_box.yh // blk_h - 2) * blk_h + 2 * blk_h
        # local_x = -(-clk_local_sam_master.bound_box.xh // blk_h) * blk_h
        local_x = clk_local_sam_master.bound_box.xh
        local = self.add_instance(clk_local_sam_master, xform=Transform(local_x, local_y, Orientation.MY))
        vco_y = -(-local.bound_box.yh // blk_h) * blk_h
        # vco_x = -(-clk_local_vco_master.bound_box.xh // blk_h) * blk_h
        vco_x = clk_local_vco_master.bound_box.xh
        vco = self.add_instance(clk_local_vco_master, xform=Transform(vco_x, vco_y, Orientation.MY))
        amp_y = -(-vco.bound_box.yh // blk_h) * blk_h
        # amp_x = -(-clk_local_amp_master.bound_box.xh // blk_h) * blk_h
        amp_x = clk_local_amp_master.bound_box.xh
        amp = self.add_instance(clk_local_amp_master, xform=Transform(amp_x, amp_y, Orientation.MY))
        hold_y = -(-amp.bound_box.yh // blk_h) * blk_h
        # hold_x = -(-clk_local_hold_master.bound_box.xh // blk_w) * blk_w
        # hold = self.add_instance(clk_local_hold_master, xform=Transform(hold_x, hold_y, Orientation.MY))

        # tot_w = max(amp_w, vco_w, hold_w, buf_w, sam_w)
        tot_w = max(amp_w, vco_w, buf_w, sam_w)
        tot_w = -(-tot_w // blk_w) * blk_w
        # tot_h = -(-max(hold.bound_box.yh, delay.bound_box.yh) // blk_h) * blk_h
        tot_h = -(-hold_y // blk_h) * blk_h

        delay_x = - clk_delay_master.bound_box.w - 4 * blk_w  # TODO: Fix this, supply line-end spacing
        delay_x = -(-delay_x // blk_w) * blk_w
        delay_y = buf.bound_box.yl + buf.bound_box.h // 2 - clk_delay_master.bound_box.h // 2
        delay_y = delay_y//blk_h * blk_h
        delay = self.add_instance(clk_delay_master, xform=Transform(delay_x, delay_y, mode=Orientation.R0))
        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))

        # ==== Connection ====

        # Delay node
        buf_inn = buf.get_pin('inn', layer=xm1_layer)
        buf_inp = buf.get_pin('inp', layer=xm1_layer)
        buf_ym1_tidx_n = self.grid.coord_to_track(ym1_layer, buf_inn.lower, RoundMode.NEAREST)
        tr_w_clk_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        tr_w_clk_out_ym1 = tr_manager.get_width(ym1_layer, 'clk_out')
        tr_sp_clk_ym1 = self.get_track_sep(ym1_layer, tr_w_clk_ym1, tr_w_clk_ym1)
        tr_sp_clk_out_ym1 = self.get_track_sep(ym1_layer, tr_w_clk_out_ym1, tr_w_clk_out_ym1)
        buf_ym1_tidx_p = buf_ym1_tidx_n - tr_sp_clk_ym1

        ret_xm1_list = []
        buf_inn_ym1 = self.connect_to_tracks(buf_inn,
                                             TrackID(ym1_layer, buf_ym1_tidx_n, tr_w_clk_ym1, grid=self.grid),
                                             ret_wire_list=ret_xm1_list)
        buf_inp_ym1 = self.connect_to_tracks(buf_inp,
                                             TrackID(ym1_layer, buf_ym1_tidx_p, tr_w_clk_ym1, grid=self.grid),
                                             ret_wire_list=ret_xm1_list)
        self.match_warr_length(ret_xm1_list)

        ret_xm1_list = []
        buf_outn_ym1 = self.connect_to_tracks(local.get_pin('outn', layer=xm1_layer),
                                              TrackID(ym1_layer, buf_ym1_tidx_n, tr_w_clk_ym1, grid=self.grid),
                                              ret_wire_list=ret_xm1_list)
        buf_outp_ym1 = self.connect_to_tracks(local.get_pin('outp', layer=xm1_layer),
                                              TrackID(ym1_layer, buf_ym1_tidx_p, tr_w_clk_ym1, grid=self.grid),
                                              ret_wire_list=ret_xm1_list)
        self.match_warr_length(ret_xm1_list)

        xm2_layer = ym1_layer + 1
        delay_outn_stack_up = self.via_stack_up(tr_manager, delay.get_pin('outn'), xm1_layer, xm2_layer, 'sig')
        delay_outp_stack_up = self.via_stack_up(tr_manager, delay.get_pin('outp'), xm1_layer, xm2_layer, 'sig')
        self.connect_differential_wires([buf_inn_ym1, buf_outn_ym1], [buf_inp_ym1, buf_outp_ym1],
                                        delay_outn_stack_up[xm2_layer], delay_outp_stack_up[xm2_layer])

        # clock ym1
        clkp_xm1 = [w for inst in [local, amp, vco] for w in inst.get_all_port_pins('clkp')]
        clkn_xm1 = [w for inst in [local, amp, vco] for w in inst.get_all_port_pins('clkn')]
        ym1_clk_mid_tidx = self.grid.coord_to_track(ym1_layer, local.get_pin('clkp').middle, RoundMode.NEAREST)
        tr_w_clk_in_ym1 = tr_manager.get_width(ym1_layer, 'clk_in')
        tr_sp_clk_in_ym1 = self.get_track_sep(ym1_layer, tr_w_clk_in_ym1, tr_w_clk_in_ym1).div2(round_up=True)
        ym1_clk_n_tidx = ym1_clk_mid_tidx - tr_sp_clk_in_ym1
        ym1_clk_p_tidx = ym1_clk_mid_tidx + tr_sp_clk_in_ym1

        clk_ym1_n = self.connect_to_tracks(clkn_xm1, TrackID(ym1_layer, ym1_clk_n_tidx, tr_w_clk_in_ym1))
        clk_ym1_p = self.connect_to_tracks(clkp_xm1, TrackID(ym1_layer, ym1_clk_p_tidx, tr_w_clk_in_ym1))
        clk_ym1_n, clk_ym1_p = self.match_warr_length([clk_ym1_n, clk_ym1_p])

        enable_ym1_tidx = ym1_clk_p_tidx + 2 * tr_sp_clk_in_ym1
        enable_ym1 = self.connect_to_tracks([inst.get_pin('en') for inst in [local, amp, vco]],
                                            TrackID(ym1_layer, enable_ym1_tidx, tr_manager.get_width(ym1_layer, 'clk')))

        # clock in xm2
        xm2_clk_mid_tidx = self.grid.coord_to_track(xm2_layer, local.bound_box.yl + local.bound_box.h // 2,
                                                    RoundMode.NEAREST)
        tr_w_clk_in_xm2 = tr_manager.get_width(xm2_layer, 'clk_in')
        tr_sp_clk_in_xm2 = self.get_track_sep(xm2_layer, tr_w_clk_in_xm2, tr_w_clk_in_xm2).div2(round_up=True)
        xm2_clk_n_tidx = xm2_clk_mid_tidx - tr_sp_clk_in_xm2
        xm2_clk_p_tidx = xm2_clk_mid_tidx + tr_sp_clk_in_xm2
        clk_xm2_n = self.connect_to_tracks(clk_ym1_n, TrackID(xm2_layer, xm2_clk_n_tidx, tr_w_clk_in_ym1))
        clk_xm2_p = self.connect_to_tracks(clk_ym1_p, TrackID(xm2_layer, xm2_clk_p_tidx, tr_w_clk_in_ym1))
        clk_xm2_n, clk_xm2_p = self.match_warr_length([clk_xm2_n, clk_xm2_p])

        # clock in ym2
        ym2_layer = xm2_layer + 1
        ym2_clk_mid_tidx = self.grid.coord_to_track(ym2_layer, clk_xm2_n.middle, RoundMode.NEAREST)
        tr_w_clk_in_ym2 = tr_manager.get_width(ym2_layer, 'clk_in')
        tr_sp_clk_in_ym2 = self.get_track_sep(ym2_layer, tr_w_clk_in_ym2, tr_w_clk_in_xm2).div2(round_up=True)
        ym2_clk_n_tidx = ym2_clk_mid_tidx - tr_sp_clk_in_ym2
        ym2_clk_p_tidx = ym2_clk_mid_tidx + tr_sp_clk_in_ym2
        clk_ym2_n = self.connect_to_tracks(clk_xm2_n, TrackID(ym2_layer, ym2_clk_n_tidx, tr_w_clk_in_ym2))
        clk_ym2_p = self.connect_to_tracks(clk_xm2_p, TrackID(ym2_layer, ym2_clk_p_tidx, tr_w_clk_in_ym2))
        clk_ym2_n, clk_ym2_p = self.match_warr_length([clk_ym2_n, clk_ym2_p])

        # vco and amp output
        vco_n_stackup = self.via_stack_up(tr_manager, vco.get_pin('outn', layer=xm1_layer), xm1_layer, ym1_layer,
                                          'clk_out')
        vco_p_stackup = self.via_stack_up(tr_manager, vco.get_pin('outp', layer=xm1_layer), xm1_layer, ym1_layer,
                                          'clk_out')

        amp_n_stackup = self.via_stack_up(tr_manager, amp.get_pin('outn', layer=xm1_layer), xm1_layer, xm2_layer,
                                          'clk_out')
        amp_p_stackup = self.via_stack_up(tr_manager, amp.get_pin('outp', layer=xm1_layer), xm1_layer, xm2_layer,
                                          'clk_out')

        vco_p_xm2_tdix = self.grid.coord_to_track(xm2_layer, vco_p_stackup[ym1_layer].middle, RoundMode.NEAREST)
        vco_p_xm2 = \
            self.connect_to_tracks(vco_p_stackup[ym1_layer], TrackID(xm2_layer, vco_p_xm2_tdix,
                                                                     amp_n_stackup[xm2_layer][0].track_id.width),
                                   min_len_mode=MinLenMode.MIDDLE)
        vco_p_ym2_tidx = self.grid.coord_to_track(ym2_layer, self.bound_box.xl, RoundMode.NEAREST)
        vco_p_ym2 = self.connect_to_tracks(vco_p_xm2, TrackID(ym2_layer, vco_p_ym2_tidx, tr_w_clk_in_ym2),
                                           track_lower=self.bound_box.xl)

        amp_ym2_tidx = self.grid.coord_to_track(ym2_layer, self.bound_box.xh, RoundMode.LESS)
        amp_ym2_tidx = amp_ym2_tidx + 2 * tr_sp_clk_in_ym2
        amp_b_ym2_tidx = amp_ym2_tidx + 2 * tr_sp_clk_in_ym2
        amp_ym2, amp_b_ym2 = self.connect_matching_tracks([amp_p_stackup[xm2_layer], amp_n_stackup[xm2_layer]],
                                                          ym2_layer, [amp_ym2_tidx, amp_b_ym2_tidx],
                                                          width=tr_w_clk_in_ym2, track_lower=self.bound_box.yl)
        # ym1 layer
        _, ym1_locs = tr_manager.place_wires(ym1_layer, ['clk'] * 13, center_coord=self.bound_box.xh)
        flip_phase_inout = self.params['flip_phase_inout']
        if flip_phase_inout:
            ym1_locs = ym1_locs[::-1]

        ret_xm1_list = []
        sam_shift_inn_xm1, sam_shift_inp_xm1 = local.get_pin('shift_inn'), local.get_pin('shift_inp')
        sam_shift_outn_xm1, sam_shift_outp_xm1 = local.get_pin('shift_outn'), local.get_pin('shift_outp')
        vco_shift_inn_xm1, vco_shift_inp_xm1 = vco.get_pin('shift_inn'), vco.get_pin('shift_inp')
        vco_shift_outn_xm1, vco_shift_outp_xm1 = vco.get_pin('shift_outn'), vco.get_pin('shift_outp')
        amp_shift_inn_xm1, amp_shift_inp_xm1 = amp.get_pin('shift_inn'), amp.get_pin('shift_inp')
        amp_shift_outn_xm1, amp_shift_outp_xm1 = amp.get_pin('shift_outn'), amp.get_pin('shift_outp')

        tr_w_clk_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        shift_in_ym1_list = [self.connect_to_tracks(warr, TrackID(ym1_layer, ym1_locs[-idx - 1], tr_w_clk_ym1),
                                                    track_upper=self.bound_box.yh, ret_wire_list=ret_xm1_list)
                             for idx, warr in enumerate([sam_shift_inn_xm1, sam_shift_inp_xm1,
                                                         vco_shift_inn_xm1, vco_shift_inp_xm1,
                                                         amp_shift_inn_xm1, amp_shift_inp_xm1])]
        shift_out_ym1_list = [self.connect_to_tracks(warr, TrackID(ym1_layer, ym1_locs[idx], tr_w_clk_ym1),
                                                     track_upper=self.bound_box.yh, ret_wire_list=ret_xm1_list)
                              for idx, warr in enumerate([sam_shift_outn_xm1, sam_shift_outp_xm1,
                                                          vco_shift_outn_xm1, vco_shift_outp_xm1,
                                                          amp_shift_outn_xm1, amp_shift_outp_xm1])]
        self.match_warr_length(shift_in_ym1_list + shift_out_ym1_list)

        # hold output
        hold_out_ret_xm1 = []

        hold_outn_tidx = self.grid.coord_to_track(ym1_layer, self.bound_box.xh, RoundMode.NEAREST)
        hold_outn_tidx = hold_outn_tidx + 2 * tr_sp_clk_in_ym1
        hold_outp_tidx = hold_outn_tidx + 2 * tr_sp_clk_in_ym1
        hold_outn_ym1 = self.connect_to_tracks(hold.get_all_port_pins('outn', xm1_layer),
                                               TrackID(ym1_layer, hold_outn_tidx, tr_w_clk_in_ym1),
                                               ret_wire_list=hold_out_ret_xm1)
        hold_outp_ym1 = self.connect_to_tracks(hold.get_all_port_pins('outp', xm1_layer),
                                               TrackID(ym1_layer, hold_outp_tidx, tr_w_clk_in_ym1),
                                               ret_wire_list=hold_out_ret_xm1)
        self.match_warr_length(hold_out_ret_xm1)
        hold_outn_ym1, hold_outp_ym1 = self.match_warr_length([hold_outn_ym1, hold_outp_ym1])
        hold_outn_tidx = self.grid.coord_to_track(xm2_layer, hold_outp_ym1.middle, RoundMode.NEAREST)
        tr_w_clk_out_xm2 = tr_manager.get_width(xm2_layer, 'clk_out')
        tr_sp_clk_out_xm2 = self.get_track_sep(xm2_layer, tr_w_clk_in_xm2, tr_w_clk_in_xm2).div2(round_up=True)

        hold_outn_tidx = hold_outn_tidx + 2 * tr_sp_clk_out_xm2
        hold_outp_tidx = hold_outn_tidx - 4 * tr_sp_clk_out_xm2
        hold_outn, hold_outp = self.connect_differential_tracks(hold_outn_ym1, hold_outp_ym1, xm2_layer,
                                                                hold_outn_tidx, hold_outp_tidx, width=tr_w_clk_out_xm2)
        hold_outn, hold_outp = self.match_warr_length([hold_outn, hold_outp])

        # sam output
        sam_outn_tidx = self.grid.coord_to_track(ym1_layer, self.bound_box.xh, RoundMode.NEAREST)
        sam_outn_tidx = sam_outn_tidx - 2 * tr_sp_clk_in_ym1
        sam_outp_tidx = sam_outn_tidx - 2 * tr_sp_clk_in_ym1

        sam_out_ret_xm1 = []
        sam_outn_ym1 = self.connect_to_tracks(buf.get_all_port_pins('outn', xm1_layer),
                                              TrackID(ym1_layer, sam_outn_tidx, tr_w_clk_in_ym1),
                                              ret_wire_list=sam_out_ret_xm1)
        sam_outp_ym1 = self.connect_to_tracks(buf.get_all_port_pins('outp', xm1_layer),
                                              TrackID(ym1_layer, sam_outp_tidx, tr_w_clk_in_ym1),
                                              ret_wire_list=sam_out_ret_xm1)
        self.match_warr_length(sam_out_ret_xm1)
        sam_outn_ym1, sam_outp_ym1 = self.match_warr_length([sam_outn_ym1, sam_outp_ym1])
        sam_outn_tidx = self.grid.coord_to_track(xm2_layer, sam_outp_ym1.middle, RoundMode.NEAREST)
        tr_w_clk_out_xm2 = 2*tr_manager.get_width(xm2_layer, 'clk_out')
        tr_sp_clk_out_xm2 = self.get_track_sep(xm2_layer, tr_w_clk_in_xm2, tr_w_clk_in_xm2).div2(round_up=True)

        sam_outn_tidx = sam_outn_tidx + 2 * tr_sp_clk_out_xm2
        sam_outp_tidx = sam_outn_tidx - 4 * tr_sp_clk_out_xm2
        sam_outn, sam_outp = self.connect_differential_tracks(sam_outn_ym1, sam_outp_ym1, xm2_layer,
                                                              sam_outn_tidx, sam_outp_tidx, width=tr_w_clk_out_xm2)
        sam_outn, sam_outp = self.match_warr_length([sam_outn, sam_outp])

        sam_e_out_tidx = self.grid.coord_to_track(ym1_layer,
                                                  buf.get_all_port_pins('midn<0>', layer=xm1_layer)[0].middle,
                                                  RoundMode.NEAREST)
        sam_e_outn_tidx = sam_e_out_tidx - tr_sp_clk_out_ym1.div2(round_up=True)
        sam_e_outp_tidx = sam_e_out_tidx + tr_sp_clk_out_ym1.div2(round_up=True)
        sam_e_outn, sam_e_outp = self.connect_differential_tracks(buf.get_all_port_pins('midn<0>', layer=xm1_layer),
                                                                  buf.get_all_port_pins('midp<0>', layer=xm1_layer),
                                                                  ym1_layer, sam_e_outn_tidx, sam_e_outp_tidx,
                                                                  width=tr_w_clk_out_ym1)

        sam_e_outn = self.connect_to_tracks(buf.get_all_port_pins('midn<0>', layer=xm1_layer),
                                            TrackID(ym1_layer, sam_e_outn_tidx, tr_w_clk_out_ym1))
        sam_e_outp = self.connect_to_tracks(buf.get_all_port_pins('midp<0>', layer=xm1_layer),
                                            TrackID(ym1_layer, sam_e_outp_tidx, tr_w_clk_out_ym1))

        sam_e_outn_xm2 = self.via_up(tr_manager, sam_e_outn, ym1_layer, 'clk_out')
        sam_e_outp_xm2 = self.via_up(tr_manager, sam_e_outp, ym1_layer, 'clk_out')

        sam_e_outn_xm2, sam_e_outp_xm2 = self.match_warr_length([sam_e_outn_xm2, sam_e_outp_xm2])
        # sam_e_outn_xm2 = self.via_up(tr_manager, sam_e_outn_xm2, ym1_layer + 1, 'clk_out', alignment=RoundMode.NEAREST)
        # sam_e_outp_xm2 = self.via_up(tr_manager, sam_e_outp_xm2, ym1_layer + 1, 'clk_out', alignment=RoundMode.NEAREST)

        ym2_sam_e_tidx = ym2_clk_n_tidx - 2*tr_sp_clk_in_ym2
        sam_e_outn_ym2 = self.connect_to_tracks(sam_e_outn_xm2, TrackID(ym2_layer, ym2_sam_e_tidx, tr_w_clk_in_ym2))
        sam_e_outp_ym2 = self.connect_to_tracks(sam_e_outp_xm2, TrackID(ym2_layer, ym2_sam_e_tidx, tr_w_clk_in_ym2))

        sam_e_outn_xm2 = self.via_up(tr_manager, sam_e_outn_ym2, ym2_layer, 'clk_out', alignment=RoundMode.NEAREST)
        sam_e_outp_xm2 = self.via_up(tr_manager, sam_e_outp_ym2, ym2_layer, 'clk_out', alignment=RoundMode.NEAREST)
        sam_e_outn_xm2, sam_e_outp_xm2 = self.match_warr_length([sam_e_outn_xm2, sam_e_outp_xm2],
                                                                warr_upper=self.bound_box.xh)

        # retimer to hold ===
        retp_vm = local.get_pin('retp_vm')
        retn_vm = local.get_pin('retn_vm')
        local_outn_xm = local.get_pin('outn', layer=xm_layer)
        local_outp_xm = local.get_pin('outp', layer=xm_layer)

        retn_xm_tidx, retp_xm_tidx = \
            local_outp_xm.track_id.base_index - local_outp_xm.track_id.pitch, \
            local_outn_xm.track_id.base_index + local_outn_xm.track_id.num * local_outn_xm.track_id.pitch

        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        retp_xm = self.connect_to_tracks(retp_vm, TrackID(xm_layer, retp_xm_tidx, tr_w_sig_xm, grid=self.grid))
        retn_xm = self.connect_to_tracks(retn_vm, TrackID(xm_layer, retn_xm_tidx, tr_w_sig_xm, grid=self.grid))

        _, [retp_ym_tidx, retn_ym_tidx] = tr_manager.place_wires(ym_layer, ['sig', 'sig'],
                                                                 center_coord=local.bound_box.xl)
        retp_ym, retn_ym = self.connect_matching_tracks([retp_xm, retn_xm], ym_layer,
                                                        [retp_ym_tidx, retn_ym_tidx],
                                                        width=tr_manager.get_width(ym_layer, 'sig'))

        self.connect_differential_wires(retn_ym, retp_ym, hold.get_pin('sam_e_b'), hold.get_pin('sam_e'))

        # ===
        hold_sam_ym1 = self.connect_to_tracks(hold.get_pin('sam'), TrackID(ym1_layer, ym1_clk_p_tidx, tr_w_clk_out_ym1))
        hold_sam_b_ym1 = self.connect_to_tracks(hold.get_pin('sam_b'),
                                                TrackID(ym1_layer, ym1_clk_n_tidx, tr_w_clk_out_ym1))
        self.match_warr_length([hold_sam_b_ym1, hold_sam_ym1])

        self.connect_differential_wires(buf.get_all_port_pins('outp', layer=xm1_layer),
                                        buf.get_all_port_pins('outn', layer=xm1_layer), hold_sam_ym1, hold_sam_b_ym1)

        delay_vdd_list, delay_vss_list = delay.get_all_port_pins('VDD'), delay.get_all_port_pins('VSS')
        delay_vdd_list, delay_vss_list = \
            self.connect_supply_stack_warr(tr_manager, [delay_vdd_list, delay_vss_list], xm1_layer, ym2_layer,
                                           delay.bound_box)
        clock_vss_list = local.get_all_port_pins('VSS_xm1') + amp.get_all_port_pins('VSS_xm1') + \
                         vco.get_all_port_pins('VSS_xm1') + buf.get_all_port_pins('VSS_xm1') + \
                         hold.get_all_port_pins('VSS_xm1')
        clock_vdd_list = local.get_all_port_pins('VDD_xm1') + amp.get_all_port_pins('VDD_xm1') + \
                         vco.get_all_port_pins('VDD_xm1') + buf.get_all_port_pins('VDD_xm1') + \
                         hold.get_all_port_pins('VDD_xm1')
        clock_vss_list = self.extend_wires(clock_vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        clock_vdd_list = self.extend_wires(clock_vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        clock_vdd_list, clock_vss_list = \
            self.connect_supply_stack_warr(tr_manager, [clock_vdd_list, clock_vss_list], xm1_layer, ym2_layer,
                                           BBox(0, 0, self.bound_box.xh, amp.bound_box.yh))

        vdd_list, vss_list = \
            self.connect_supply_stack_warr(tr_manager, [clock_vdd_list[ym2_layer] + delay_vdd_list[ym2_layer],
                                                        clock_vss_list[ym2_layer] + delay_vss_list[ym2_layer]],
                                           ym2_layer, ym2_layer + 1,
                                           BBox(delay.bound_box.xl, 0, self.bound_box.xh, amp.bound_box.yh),
                                           extend_lower_layer=True)

        for pin in delay.port_names_iter():
            if 'in' in pin:
                pin_warr = self.extend_wires(delay.get_pin(pin), upper=self.bound_box.yh)
                self.add_pin(pin.replace('in', 'ctrl_delay'), pin_warr, mode=PinMode.UPPER)

        self.add_pin('shift_sam_n_in', shift_in_ym1_list[0], mode=PinMode.UPPER)
        self.add_pin('shift_sam_p_in', shift_in_ym1_list[1], mode=PinMode.UPPER)
        self.add_pin('shift_vco_n_in', shift_in_ym1_list[2], mode=PinMode.UPPER)
        self.add_pin('shift_vco_p_in', shift_in_ym1_list[3], mode=PinMode.UPPER)
        self.add_pin('shift_amp_n_in', shift_in_ym1_list[4], mode=PinMode.UPPER)
        self.add_pin('shift_amp_p_in', shift_in_ym1_list[5], mode=PinMode.UPPER)

        self.add_pin('shift_sam_n_out', shift_out_ym1_list[0], mode=PinMode.UPPER)
        self.add_pin('shift_sam_p_out', shift_out_ym1_list[1], mode=PinMode.UPPER)
        self.add_pin('shift_vco_n_out', shift_out_ym1_list[2], mode=PinMode.UPPER)
        self.add_pin('shift_vco_p_out', shift_out_ym1_list[3], mode=PinMode.UPPER)
        self.add_pin('shift_amp_n_out', shift_out_ym1_list[4], mode=PinMode.UPPER)
        self.add_pin('shift_amp_p_out', shift_out_ym1_list[5], mode=PinMode.UPPER)

        self.add_pin('sam_n', sam_outn, mode=PinMode.UPPER)
        self.add_pin('sam_p', sam_outp, mode=PinMode.UPPER)
        self.add_pin('sam_e_n', sam_e_outn_xm2, mode=PinMode.UPPER)
        self.add_pin('sam_e_p', sam_e_outp_xm2, mode=PinMode.UPPER)
        self.add_pin('vco_n', vco_n_stackup[ym1_layer], mode=PinMode.UPPER)
        self.add_pin('vco_p', vco_p_ym2, mode=PinMode.LOWER)

        self.add_pin('amp_n', amp_b_ym2, mode=PinMode.LOWER)
        self.add_pin('amp_p', amp_ym2, mode=PinMode.LOWER)

        self.add_pin('hold_p', hold_outp)
        self.add_pin('hold_n', hold_outn)

        self.reexport(local.get_port('retn'), net_name='ret_n')
        self.reexport(local.get_port('retp'), net_name='ret_p')

        self.add_pin('en', self.extend_wires(enable_ym1, upper=self.bound_box.yh))
        self.add_pin('clkp', self.extend_wires(clk_ym2_p, upper=self.bound_box.yh))
        self.add_pin('clkn', self.extend_wires(clk_ym2_n, upper=self.bound_box.yh))

        self.add_pin('VSS', vss_list[xm2_layer + 2])
        self.add_pin('VDD', vdd_list[xm2_layer + 2])

        self.sch_params = dict(
            sam=clk_local_sam_master.sch_params,
            vco=clk_local_vco_master.sch_params,
            amp=clk_local_amp_master.sch_params,
            hold=clk_local_hold_master.sch_params,
            delay=clk_delay_master.sch_params,
            buf=clk_local_buf_master.sch_params
        )
