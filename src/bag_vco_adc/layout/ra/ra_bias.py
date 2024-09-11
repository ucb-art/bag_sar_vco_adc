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



from typing import Any, Dict
from typing import Optional, Type

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.routing import TrackID
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param
from bag_vco_adc.layout.digital import InvCore, NAND2Core
from bag_vco_adc.layout.sar.sar_logic import SARLogicUnit
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, MinLenMode, PinMode, Orientation, Orient2D
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo
from xbase.layout.mos.top import GenericWrapper
from .ra_cap import MOSBaseCap
from ..util.util import fill_conn_layer_intv, export_xm_sup, fill_tap
from ..util.wrapper import MOSBaseTapWrapper


class RABiasLogicUnit(SARLogicUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_bias_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            msb='True en its msb',
            pbias='True to generate pmos bias'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            msb=False,
            pbias=False,
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
        msb: bool = self.params['msb']
        pbias: bool = self.params['pbias']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_nand: int = seg_dict['nand']
        seg_inv_in: int = seg_dict['inv_in']
        seg_inv_out: int = seg_dict['inv_out']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)

        nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=False)

        inv_in_params = dict(pinfo=pinfo, seg=seg_inv_in, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             vertical_sup=False, sig_locs={'nin': ng2_tidx})
        inv_out_params = dict(pinfo=pinfo, seg=seg_inv_out, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={'nin': pg0_tidx})
        inv_buf_params = dict(pinfo=pinfo, seg=1, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, vertical_out=False)

        nand_master = self.new_template(NAND2Core, params=nand_params)
        inv_in_master = self.new_template(InvCore, params=inv_in_params)
        inv_out_master = self.new_template(InvCore, params=inv_out_params)
        inv_buf_master = self.new_template(InvCore, params=inv_buf_params)

        # Place insts
        cur_col = 0
        inv_in = self.add_tile(inv_in_master, 1, cur_col) if msb else None
        if msb:
            cur_col += inv_in_master.num_cols + min_sep
        cur_col += cur_col & 1

        nand = self.add_tile(nand_master, 1, cur_col)
        cur_col += nand_master.num_cols + min_sep
        cur_col += cur_col & 1

        has_inv_out = (pbias and not msb) or (not pbias and msb)
        inv_out = self.add_tile(inv_out_master, 1, cur_col) if has_inv_out else None

        in_buf = True
        if in_buf:
            ctrl_buf = self.add_tile(inv_buf_master, 0, 0)
            en_buf = self.add_tile(inv_buf_master, 0, cur_col, flip_lr=True)
        else:
            ctrl_buf, en_buf = None, None
        self.set_mos_size()

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        if msb:
            self.connect_to_track_wires(inv_in.get_pin('out'), nand.get_pin('nin<0>'))
            in_vm_tidx = self.grid.coord_to_track(vm_layer, inv_in.bound_box.xl, RoundMode.NEAREST)
            in_vm = self.connect_to_tracks(inv_in.get_pin('nin'), TrackID(vm_layer, in_vm_tidx, tr_w_sig_vm),
                                           min_len_mode=MinLenMode.MIDDLE)
            en_vm_tidx = self.grid.coord_to_track(vm_layer, nand.bound_box.xh, RoundMode.NEAREST)
            en_vm = self.connect_to_tracks(nand.get_pin('nin<1>'), TrackID(vm_layer, en_vm_tidx, tr_w_sig_vm),
                                           min_len_mode=MinLenMode.MIDDLE)
        else:
            in_vm_tidx = self.grid.coord_to_track(vm_layer, nand.bound_box.xl, RoundMode.NEAREST)
            in_vm = self.connect_to_tracks(nand.get_pin('nin<0>'), TrackID(vm_layer, in_vm_tidx, tr_w_sig_vm),
                                           min_len_mode=MinLenMode.MIDDLE)
            # en_vm_tidx = tr_manager.get_next_track(vm_layer, in_vm_tidx, 'sig', 'sig')
            en_vm_tidx = self.grid.coord_to_track(vm_layer, nand.bound_box.xh, RoundMode.NEAREST)
            en_vm = self.connect_to_tracks(nand.get_pin('nin<1>'), TrackID(vm_layer, en_vm_tidx, tr_w_sig_vm),
                                           min_len_mode=MinLenMode.MIDDLE)
        if has_inv_out:
            self.connect_to_track_wires(nand.get_pin('out'), inv_out.get_pin('nin'))

        if in_buf:
            self.connect_to_track_wires([ctrl_buf.get_pin('nout'), ctrl_buf.get_pin('pout')], in_vm)
            self.connect_to_track_wires([en_buf.get_pin('nout'), en_buf.get_pin('pout')], en_vm)
            in_vm_new_tidx = tr_manager.get_next_track(vm_layer, in_vm.track_id.base_index, 'sig', 'sig', up=-1)
            en_vm_new_tidx = tr_manager.get_next_track(vm_layer, en_vm.track_id.base_index, 'sig', 'sig', up=-1)
            in_vm = self.connect_to_tracks(ctrl_buf.get_pin('nin'), TrackID(vm_layer, in_vm_new_tidx, tr_w_sig_vm))
            en_vm = self.connect_to_tracks(en_buf.get_pin('nin'), TrackID(vm_layer, en_vm_new_tidx, tr_w_sig_vm))

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [nand]
        if msb:
            inst_list.append(inv_in)
        if has_inv_out:
            inst_list.append(inv_out)
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]

        self.add_pin('en', en_vm)
        self.add_pin('in', in_vm)
        self.add_pin('out', inv_out.get_pin('out') if has_inv_out else nand.get_pin('out'))
        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)
        self.sch_params = dict(
            nand=nand_master.sch_params,
            inv_in=inv_in_master.sch_params,
            inv_out=inv_out_master.sch_params,
            pbias=self.params['pbias'],
            msb=self.params['msb'],
            inv_buf=inv_buf_master.sch_params if in_buf else None
        )


class RABiasLogic(SARLogicUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    # @classmethod
    # def get_schematic_class(cls) -> Optional[Type[Module]]:
    #     # noinspection PyTypeChecker
    #     return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_unit_bot')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            pbias='True to generate pmos bias',
            nbits='Number of bits',
            ncols='Number of columns to match cap width'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            pbias=False,
            nbits=1,
            ncols=0,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        nbits: int = self.params['nbits']
        pbias: bool = self.params['pbias']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_dict: Dict[str, Any] = self.params['seg_dict']
        logic_unit_parmas_list = []

        for idx in range(nbits):
            logic_params = dict(pinfo=pinfo, seg_dict=seg_dict, pbias=pbias, msb=idx == nbits - 1)
            logic_unit_parmas_list.append(logic_params)

        logic_master_list = []
        for params in logic_unit_parmas_list:
            logic_master_list.append(self.new_template(RABiasLogicUnit, params=params))

        # Place insts
        min_sep = self.min_sep_col
        cur_col = min_sep
        cur_col += cur_col & 1
        logic_unit_list = []
        for master in logic_master_list:
            logic_unit_list.append(self.add_tile(master, 0, cur_col))
            cur_col += master.num_cols + min_sep
            cur_col += cur_col & 1

        ncols = max(self.params['ncols'], self.num_cols)
        self.set_mos_size(num_cols=ncols)

        # expot vm out to top
        out_list = [self.extend_wires(unit.get_pin('out'), upper=self.bound_box.yh)[0] for unit in logic_unit_list]

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = logic_unit_list
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]

        _, vdd_xm_list = export_xm_sup(self, 0, export_bot=False, export_top=True)
        vss_xm_list_l, _ = export_xm_sup(self, 0, export_bot=True, export_top=False,
                                         given_locs=[warr.track_id.base_index for warr in out_list])
        _, vss_xm_tidx_u = export_xm_sup(self, 1, export_bot=False, export_top=True)

        ymid_coord = self.bound_box.yl + self.get_tile_pinfo(0).height // 2
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        num_sig = tr_manager.get_num_wires_between(xm_layer, 'sup', vss_xm_list_l.track_id.base_index,
                                                   'sup', vdd_xm_list.track_id.base_index, 'sig')
        _, sig_xm_locs = tr_manager.place_wires(xm_layer, ['sup', 'sup', 'clk']+['sig'] * num_sig, center_coord=ymid_coord)
        sig_xm_locs = sig_xm_locs[2:-1]
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        en_xm = self.connect_to_tracks([unit.get_pin('en') for unit in logic_unit_list],
                                       TrackID(xm_layer, sig_xm_locs[0], tr_w_clk_xm), track_lower=self.bound_box.xl)
        in_xm_list = [self.connect_to_tracks(unit.get_pin('in'), TrackID(xm_layer, sig_xm_locs[idx + 1], tr_w_sig_xm),
                                             track_lower=self.bound_box.xl)
                      for idx, unit in enumerate(logic_unit_list)]

        self.add_pin('en', en_xm)
        [self.add_pin(f'in<{idx}>', pin) for idx, pin in enumerate(in_xm_list)]
        [self.add_pin(f'out<{idx}>', pin) for idx, pin in enumerate(out_list)]
        self.add_pin('VDD_hm', vdd_hm, hide=True)
        self.add_pin('VSS_hm', vss_hm, hide=True)
        self.add_pin('VDD', vdd_xm_list)
        self.add_pin('VSS', [vss_xm_tidx_u, vss_xm_list_l])
        self._sch_params = [inst.sch_params for inst in logic_master_list]


class RABias(SARLogicUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_bias')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            pbias='True to generate pmos bias',
            nbits='Number of bits',
            ncols='Number of columns to match cap width',
            nlsb_first_row='Number of lsbs in the first row',
            nmsb_in_col='Number of msbs arrange in column',
            msb_ow='Non-zero value to override msb, to tune the overall range',
            ncols_tot='Total number of columns',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            pbias=False,
            nbits=1,
            ncols=0,
            nlsb_first_row=6,
            nmsb_in_col=2,
            ncols_tot=0,
            msb_ow=0,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        nbits: int = self.params['nbits']
        pbias: bool = self.params['pbias']
        ncols_tot: int = self.params['ncols_tot']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        seg_dict: Dict[str, Any] = self.params['seg_dict']
        logic_params = dict(pinfo=pinfo, seg_dict=seg_dict, pbias=pbias, nbits=nbits)
        logic_master = self.new_template(RABiasLogic, params=logic_params)

        # Place insts
        cur_col = 0
        min_sep = self.min_sep_col
        nlsb_first_row = self.params['nlsb_first_row']
        nmsb_in_col = self.params['nmsb_in_col']
        if nbits != nlsb_first_row + nmsb_in_col:
            raise ValueError('Total bits doesnt match arrangement')

        logic = self.add_tile(logic_master, 0, 0)
        cap_seg_list = [2 ** idx for idx in range(1, nlsb_first_row + 1)]
        for idx in range(nlsb_first_row, nbits):
            for jdx in range(2 ** (idx - nlsb_first_row)):
                cap_seg_list.append(2 ** nlsb_first_row)
        msb_ow = self.params['msb_ow']
        if msb_ow:
            if isinstance(msb_ow, int):
                msb_ow = [msb_ow]
            len_msb_ow = len(msb_ow)
            cap_seg_list[-len_msb_ow:] = msb_ow
        cap_params_list = []
        for idx, seg in enumerate(cap_seg_list):
            cap_params_list.append(dict(pinfo=pinfo, seg=seg, half=False))
        cap_master_list = []
        for params in cap_params_list:
            cap_master_list.append(self.new_template(MOSBaseCap, params=params))

        cur_tile = logic_master.num_tile_rows + 1
        cap_cur_col = 0
        cap_inst_list = []
        for idx in range(nlsb_first_row):
            cap_inst_list.append(self.add_tile(cap_master_list[idx], cur_tile, cap_cur_col))
            cap_cur_col += cap_master_list[idx].num_cols
            if cap_master_list[idx].num_cols < 4:
                cap_cur_col += self.min_sep_col
        first_row_col = cap_cur_col
        cur_tile += 1

        def get_tile_ymid(tile_idx):
            tile_pinfo = self.get_tile_info(tile_idx)
            return tile_pinfo[1] + tile_pinfo[0].height // 2

        cap_msb_xm_locs = []
        master_idx = nlsb_first_row
        for idx in range(nmsb_in_col):
            _nrow = 2 ** idx
            _bit_list = []
            _xm_locs_list = []
            for jdx in range(_nrow):
                _bit_list.append(self.add_tile(cap_master_list[master_idx], cur_tile, 0))
                _xm_locs_list.append(self.grid.coord_to_track(xm_layer, get_tile_ymid(cur_tile), RoundMode.NEAREST))
                cur_tile += 1
                master_idx += 1
            cap_msb_xm_locs.append(_xm_locs_list)
            cap_inst_list.append(_bit_list)

        vm_l_tidx = self.arr_info.col_to_track(vm_layer,
                                               max(max([inst.num_cols for inst in cap_master_list]), first_row_col))
        vm_l_tidx = tr_manager.get_next_track(vm_layer, vm_l_tidx, 'sig', 'cap', up=True)
        vm_bnd_tidx = tr_manager.get_next_track(vm_layer, vm_l_tidx, 'cap', 'cap', up=True)
        bias_vm_col = self.arr_info.track_to_col(vm_layer, vm_bnd_tidx)
        self.set_mos_size(num_cols=max(logic_master.num_cols + self.get_tap_ncol(seg=4), bias_vm_col, ncols_tot))

        for idx in range(1, self.num_tile_rows):
            fill_conn_layer_intv(self, idx, 0)
            fill_conn_layer_intv(self, idx, 1)

        # connect lsbs' top plate

        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')
        _, cap_lsb_xm_locs = tr_manager.place_wires(xm_layer, ['cap'] * nlsb_first_row, center_coord=get_tile_ymid(3))
        cap_lsb_xm_list = []
        for idx in range(nlsb_first_row):
            cap_lsb_xm_list.append(self.connect_to_tracks(cap_inst_list[idx].get_all_port_pins('minus'),
                                                          TrackID(xm_layer, cap_lsb_xm_locs[idx], tr_w_cap_xm),
                                                          min_len_mode=MinLenMode.MIDDLE))
        cap_msb_xm_list = []
        for idx in range(nmsb_in_col):
            _xm_list = []
            for _xm_loc, cap in zip(cap_msb_xm_locs[idx], cap_inst_list[nlsb_first_row + idx]):
                _xm_list.append(self.connect_to_tracks(cap.get_all_port_pins('minus'),
                                                       TrackID(xm_layer, _xm_loc, tr_w_cap_xm),
                                                       min_len_mode=MinLenMode.MIDDLE))
            cap_msb_xm_list.append(_xm_list)

        # Connect to ym
        cap_ym_list = []
        tr_w_cap_ym = tr_manager.get_width(ym_layer, 'cap')
        for xm in cap_lsb_xm_list[:-1]:
            _ym_tidx = self.grid.coord_to_track(ym_layer, xm.middle, RoundMode.NEAREST)
            cap_ym_list.append(self.connect_to_tracks(xm, TrackID(ym_layer, _ym_tidx, tr_w_cap_ym)))

        ym1_tid_l = self.grid.coord_to_track(ym_layer, cap_lsb_xm_list[-1].lower, mode=RoundMode.GREATER)
        ym1_tid_r = self.grid.coord_to_track(ym_layer, cap_lsb_xm_list[-1].upper, mode=RoundMode.LESS)
        ym_locs = tr_manager.spread_wires(ym_layer, ['cap'] * (nmsb_in_col + 3), ym1_tid_l, ym1_tid_r,
                                          ('cap', 'cap'))
        ym_locs = ym_locs[1:-1]
        cap_ym_list.append(self.connect_to_tracks(cap_lsb_xm_list[-1], TrackID(ym_layer, ym_locs[0], tr_w_cap_ym)))
        cap_ym_list.extend([self.connect_to_tracks(_xm, TrackID(ym_layer, ym_locs[idx + 1], tr_w_cap_ym))
                            for idx, _xm in enumerate(cap_msb_xm_list)])

        # Route to logic
        _, xm_logic_locs = tr_manager.place_wires(xm_layer, ['cap'] * nbits, center_coord=get_tile_ymid(2))
        xm_logic_locs = xm_logic_locs[::-1]
        cap_xm_logic_list = []
        for ym, xm_locs in zip(cap_ym_list, xm_logic_locs):
            cap_xm_logic_list.append(self.connect_to_tracks(ym, TrackID(xm_layer, xm_locs, tr_w_cap_xm),
                                                            min_len_mode=MinLenMode.MIDDLE))

        for idx, xm in enumerate(cap_xm_logic_list):
            self.connect_to_track_wires(logic.get_pin(f'out<{idx}>'), xm)

        tr_w_cap_vm = tr_manager.get_width(vm_layer, 'cap')
        cap_inst_list_flatten = []
        for cap in cap_inst_list:
            if type(cap) is list:
                cap_inst_list_flatten.extend(cap)
            else:
                cap_inst_list_flatten.append(cap)
        bias_vm = self.connect_to_tracks([_w for cap in cap_inst_list_flatten for _w in cap.get_all_port_pins('plus')],
                                         TrackID(vm_layer, vm_l_tidx, tr_w_cap_vm))
        bias_xm_tidx = self.grid.coord_to_track(xm_layer, bias_vm.middle, RoundMode.NEAREST)
        tr_w_bias_xm = tr_manager.get_width(xm_layer, 'bias')
        tr_w_bias_ym = tr_manager.get_width(ym_layer, 'bias')
        bias_xm = self.connect_to_tracks(bias_vm, TrackID(xm_layer, bias_xm_tidx, tr_w_bias_xm))
        bias_ym_tidx = self.grid.coord_to_track(ym_layer, bias_xm.middle, RoundMode.NEAREST)
        bias_ym = self.connect_to_tracks(bias_xm, TrackID(ym_layer, bias_ym_tidx, tr_w_bias_ym))

        xm1_layer = ym_layer + 1
        bias_xm1_tidx = self.grid.coord_to_track(xm1_layer, bias_ym.middle, RoundMode.NEAREST)
        tr_w_bias_xm1 = tr_manager.get_width(xm1_layer, 'bias')
        bias_xm1 = self.connect_to_tracks(bias_ym, TrackID(xm1_layer, bias_xm1_tidx, tr_w_bias_xm1),
                                          min_len_mode=MinLenMode.MIDDLE, track_upper=self.bound_box.xh)

        fill_tap(self, 0, SubPortMode.EVEN, extra_margin=False)
        vdd_hm, vss_hm = logic.get_pin('VDD_hm', layer=hm_layer), logic.get_pin('VSS_hm', layer=hm_layer)
        vdd_xm, vss_xm = logic.get_all_port_pins('VDD', layer=xm_layer),\
                         logic.get_all_port_pins('VSS', layer=xm_layer)
        vdd_hm = self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.extend_wires(vss_hm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_xm = self.extend_wires(vdd_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm = self.extend_wires(vss_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)

        en_xm = logic.get_pin('en')
        en_xm = self.extend_wires(en_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)

        [self.add_pin(f'in<{idx}>', logic.get_pin(f'in<{idx}>'), mode=PinMode.LOWER) for idx in range(nbits)]
        self.add_pin('bias', [bias_xm, bias_ym, bias_xm1])
        self.add_pin('en', en_xm)

        self.add_pin('VDD_hm', vdd_hm, label='VDD')
        self.add_pin('VSS_hm', vss_hm, label='VSS')
        self.add_pin('VDD_xm', vdd_xm, label='VDD')
        self.add_pin('VSS_xm', vss_xm, label='VSS')

        cap_m_list = [1] * nlsb_first_row + list(range(1, nmsb_in_col + 1))
        self._sch_params = dict(
            logic_params_list=logic_master.sch_params,
            cap_params_list=[cap.sch_params for cap in cap_master_list],
            cap_m_list=cap_m_list,
        )


class RABiasTop(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._bias_ycoord = (0, 0)

    @property
    def bias_ycoord(self):
        return self._bias_ycoord

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_bias_top')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            bias_gen_params='',
        )

    def draw_layout(self) -> None:
        pbias_params: Param = self.params['bias_gen_params']
        nbias_params: Dict = pbias_params.copy().to_dict()
        pbias_params: dict = pbias_params.to_dict()
        nbias_params['pbias'] = False

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      pbias_params['pinfo']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        pbias_params['pbias'] = True

        pbias_gen_params = dict(
            cls_name=RABias.get_qualified_name(),
            ndum_side=4,
            params=pbias_params,
            top_sup_layer=xm1_layer,
            cover_sup=True,
            fill_tap=False,
        )

        pbias_master: GenericWrapper = self.new_template(MOSBaseTapWrapper, params=pbias_gen_params)
        nbias_params['ncols_tot'] = pbias_master.core.num_cols
        nbias_gen_params = dict(
            cls_name=RABias.get_qualified_name(),
            ndum_side=4,
            params=nbias_params,
            top_sup_layer=xm1_layer,
            cover_sup=True,
            fill_tap=False,
        )
        nbias_master: GenericWrapper = self.new_template(MOSBaseTapWrapper, params=nbias_gen_params)

        tr_manager = pbias_master.core.tr_manager
        bias_ym_locs_start = self.grid.coord_to_track(vm_layer, 0, RoundMode.NEAREST)
        bias_ym_locs = [bias_ym_locs_start]
        nbits = pbias_params['nbits']
        for idx in range(2 * nbits + 1):
            bias_ym_locs.append(tr_manager.get_next_track(vm_layer, bias_ym_locs[-1], 'ctrl', 'ctrl', up=True))

        bias_ym_locs = bias_ym_locs[1:]

        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        bias_x_rt = self.grid.track_to_coord(ym_layer, bias_ym_locs[-1])

        # ----- floorplan -----
        top_layer = max(pbias_master.top_layer, nbias_master.top_layer, xm1_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        w_pbias, h_pbias = pbias_master.bound_box.w, pbias_master.bound_box.h
        w_nbias, h_nbias = nbias_master.bound_box.w, nbias_master.bound_box.h

        # ----- add blocks -----
        bias_x = -(-bias_x_rt // w_blk) * w_blk
        nbias = self.add_instance(nbias_master, xform=Transform(bias_x, 0, Orientation.R0))
        h_bias = -(-h_pbias // h_blk - 2) * h_blk
        pbias = self.add_instance(pbias_master, xform=Transform(bias_x, h_bias, Orientation.R0))

        bbox_w = max(nbias.bound_box.xh, pbias.bound_box.xh, ym1_layer)
        bbox_h = pbias.bound_box.yh
        bbox_w = -(-bbox_w // w_blk) * w_blk
        bbox_h = -(-bbox_h // h_blk) * h_blk
        bbox = BBox(0, 0, bbox_w, bbox_h)
        self.set_size_from_bound_box(top_layer, bbox)

        ctrl_p_list, ctrl_n_list = [], []
        for idx in range(nbits):
            ctrl_p_list.append(self.connect_to_tracks(pbias.get_pin(f'in<{idx}>'),
                                                      TrackID(ym_layer, bias_ym_locs[idx + 1], tr_w_ctrl_ym),
                                                      track_upper=self.bound_box.yh))
        for idx in range(nbits):
            ctrl_n_list.append(self.connect_to_tracks(nbias.get_pin(f'in<{idx}>'),
                                                      TrackID(ym_layer, bias_ym_locs[idx + nbits + 1], tr_w_ctrl_ym),
                                                      track_upper=self.bound_box.yh))

        # self.add_pin('amp', ctrl_en_vm, mode=PinMode.LOWER)
        bias_ym1_tid_l = self.grid.coord_to_track(ym1_layer, 0, RoundMode.GREATER_EQ)
        bias_ym1_tid_r = self.grid.coord_to_track(ym1_layer, bbox_w, RoundMode.LESS_EQ)

        num_ym1_sup = tr_manager.get_num_wires_between(ym1_layer, 'dum', bias_ym1_tid_l, 'dum', bias_ym1_tid_r,
                                                       'sup')
        _, locs = tr_manager.place_wires(ym1_layer, ['sup'] * num_ym1_sup, center_coord=self.bound_box.xh // 2)

        # vdd vss at xm1_layer
        tr_w_sup_ym1 = tr_manager.get_width(ym1_layer, 'sup')
        vdd_xm1_list = nbias.get_all_port_pins('VDD', layer=xm1_layer) + pbias.get_all_port_pins('VDD', layer=xm1_layer)
        vss_xm1_list = nbias.get_all_port_pins('VSS', layer=xm1_layer) + pbias.get_all_port_pins('VSS', layer=xm1_layer)
        self.extend_wires(vss_xm1_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.extend_wires(vdd_xm1_list, lower=self.bound_box.xl, upper=self.bound_box.xh)

        ym1_vdd_locs = locs[::2]
        ym1_vss_locs = locs[1::2]
        vdd_ym1_list = [self.connect_to_tracks(vdd_xm1_list, TrackID(ym1_layer, tidx, tr_w_sup_ym1)) for tidx in
                        ym1_vdd_locs]
        vss_ym1_list = [self.connect_to_tracks(vss_xm1_list, TrackID(ym1_layer, tidx, tr_w_sup_ym1)) for tidx in
                        ym1_vss_locs]
        ym_sup_top = max(vdd_ym1_list + vss_ym1_list, key=lambda x: x.upper).upper
        ym_sup_bot = min(vdd_ym1_list + vss_ym1_list, key=lambda x: x.lower).lower

        vdd_bias_ym1_list = self.extend_wires(vdd_ym1_list, lower=ym_sup_bot, upper=ym_sup_top)
        vss_bias_ym1_list = self.extend_wires(vss_ym1_list, lower=ym_sup_bot, upper=ym_sup_top)

        self._bias_ycoord = (nbias.get_pin('bias', layer=xm1_layer).bound_box.yh,
                             pbias.get_pin('bias', layer=xm1_layer).bound_box.yh)

        for idx, ctrl in enumerate(ctrl_p_list):
            self.add_pin(f'ctrl_biasp<{idx}>', ctrl, mode=PinMode.UPPER)
        for idx, ctrl in enumerate(ctrl_n_list):
            self.add_pin(f'ctrl_biasn<{idx}>', ctrl, mode=PinMode.UPPER)

        for layer in range(hm_layer, pbias_master.top_layer):
            lay_dir = self.grid.get_direction(layer)
            if lay_dir == Orient2D.y:
                self.connect_wires(pbias.get_all_port_pins('VDD', layer=layer) +
                                   nbias.get_all_port_pins('VDD', layer=layer))
                self.connect_wires(pbias.get_all_port_pins('VSS', layer=layer) +
                                   nbias.get_all_port_pins('VSS', layer=layer))
        self.add_pin('VDD', vdd_bias_ym1_list, connect=True)
        self.add_pin('VSS', vss_bias_ym1_list, connect=True)
        self.reexport(pbias.get_port('bias'), net_name='v_biasp')
        self.reexport(nbias.get_port('bias'), net_name='v_biasn')
        self.add_pin('amp', [pbias.get_pin('en'), nbias.get_pin('en')], connect=True)
        self._sch_params = dict(
            pbias_params=pbias_master.sch_params,
            nbias_params=nbias_master.sch_params,
            nbits=nbias_params['nbits'],
        )
