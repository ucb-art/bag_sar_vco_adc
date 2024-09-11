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

import copy
from itertools import chain
from typing import Any, Dict, Type, Optional, List, Mapping, Union

from bag.design.database import ModuleDB, Module
from bag.layout.routing.base import WireArray, TrackManager
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict, ImmutableList
from bag.util.math import HalfInt
from pybag.enum import MinLenMode, RoundMode, PinMode
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from .sar_async_clkgen import SARAsyncClkSimple
from .sar_logic_gates import FlopCore, RstLatchCore, DynLatchCore
from ..digital import InvChainCore, NOR3Core, InvCore, PassGateCore
from ..digital import LatchCore
from ..digital import get_adj_tid_list
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID
from ..util.util import export_xm_sup, fill_tap


class OAICore(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'oai')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg='segments of transistors',
            seg_pstack0='segments of stack input <3:2>',
            seg_pstack1='segments of stack input <1:0>',
            seg_n0='segments of nmos input <3:2>',
            seg_n1='segments of nmos input <1:0>',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            stack_p='number of transistors in a stack.',
            stack_n='number of transistors in a stack.',
            vertical_sup='True to enable vertical supply (mos_conn layer)',
            vertical_out='True to enable vertical output',
            sig_locs='Optional dictionary of user defined signal locations',
            min_len_mode='A Dictionary specfiying min_len_mode for connections',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            stack_p=1,
            stack_n=1,
            seg=-1,
            seg_pstack0=-1,
            seg_pstack1=-1,
            seg_n0=-1,
            seg_n1=-1,
            vertical_sup=False,
            vertical_out=True,
            sig_locs={},
            min_len_mode=dict(
                in0=MinLenMode.NONE,
                in1=MinLenMode.NONE,
                in2=MinLenMode.NONE,
                in3=MinLenMode.NONE,
                out=MinLenMode.MIDDLE,
            ),
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        stack_n: int = self.params['stack_n']
        stack_p: int = self.params['stack_p']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']
        mlm: Dict[str, MinLenMode] = self.params['min_len_mode']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        seg: int = self.params['seg']
        seg_pstack0: int = self.params['seg_pstack0']
        seg_pstack1: int = self.params['seg_pstack1']
        seg_n0: int = self.params['seg_n0']
        seg_n1: int = self.params['seg_n1']

        if seg_pstack0 <= 0:
            seg_pstack0 = seg
        if seg_pstack1 <= 0:
            seg_pstack1 = seg
        if seg_n0 <= 0:
            seg_n0 = seg
        if seg_n1 <= 0:
            seg_n1 = seg
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        pports0 = self.add_nand2(ridx_p, 0, seg_pstack0, w=w_p, stack=stack_p)
        nports0 = self.add_nand2(ridx_n, 0, seg_n0, w=w_n, stack=stack_n, other=True)
        port1_col = max(min_sep + 2 * seg_pstack0 * stack_p, min_sep + 2 * seg_n0 * stack_n)
        pports1 = self.add_nand2(ridx_p, port1_col, seg_pstack1, w=w_p, stack=stack_p)
        nports1 = self.add_nand2(ridx_n, port1_col, seg_n1, w=w_n, stack=stack_n, other=True)
        self.set_mos_size()
        xr = self.bound_box.xh

        tr_manager = self.tr_manager
        tr_w_h = tr_manager.get_width(hm_layer, 'sig')

        nin_tid = get_adj_tid_list(self, ridx_n, sig_locs, MOSWireType.G, 'nin', True, tr_w_h)
        pin_tid = get_adj_tid_list(self, ridx_p, sig_locs, MOSWireType.G, 'pin', False, tr_w_h)

        in0 = self.connect_to_tracks(list(chain(nports0.g0, pports0.g0)), nin_tid[0], min_len_mode=mlm.get('in0', None))
        in1 = self.connect_to_tracks(list(chain(nports0.g1, pports0.g1)), nin_tid[1], min_len_mode=mlm.get('in1', None))
        in2 = self.connect_to_tracks(list(chain(nports1.g0, pports1.g0)), pin_tid[0], min_len_mode=mlm.get('in2', None))
        in3 = self.connect_to_tracks(list(chain(nports1.g1, pports1.g1)), pin_tid[1], min_len_mode=mlm.get('in3', None))

        for p, pname in zip([in0, in1, in2, in3], ['in<0>', 'in<1>', 'in<2>', 'in<3>']):
            self.add_pin(pname, p)

        pd_tidx = sig_locs.get('pout', self.get_track_index(ridx_p, MOSWireType.DS_GATE, wire_name='sig'))
        nd_tidx0 = sig_locs.get('nout', self.get_track_index(ridx_n, MOSWireType.DS_GATE, wire_name='sig', wire_idx=-1))
        nd_tidx1 = sig_locs.get('nout', self.get_track_index(ridx_n, MOSWireType.DS_GATE, wire_name='sig', wire_idx=0))
        nd_tid0 = TrackID(hm_layer, nd_tidx0, width=tr_w_h)
        nd_tid1 = TrackID(hm_layer, nd_tidx1, width=tr_w_h)
        pd_tid = TrackID(hm_layer, pd_tidx, width=tr_w_h)

        pout = self.connect_to_tracks([pports0.d, pports1.d], pd_tid, min_len_mode=mlm.get('out', None))
        nout = self.connect_to_tracks([nports1.d], nd_tid0, min_len_mode=mlm.get('out', None))
        self.connect_to_tracks([nports1.s, nports0.d], nd_tid1, min_len_mode=mlm.get('out', None))

        vm_tidx = sig_locs.get('out', self.grid.coord_to_track(vm_layer, pout.middle, mode=RoundMode.GREATER_EQ))

        if vertical_out:
            out = self.connect_to_tracks([pout, nout], TrackID(vm_layer, vm_tidx))
            self.add_pin('pout', pout, hide=True)
            self.add_pin('nout', nout, hide=True)
            self.add_pin('out', out)
        else:
            self.add_pin('pout', pout, label='out:')
            self.add_pin('nout', nout, label='out:')

        if vertical_sup:
            self.add_pin('VDD', list(chain(pports0.s, pports1.s)), connect=True)
            self.add_pin('VSS', list(chain(nports0.s, nports1.s)), connect=True)
        else:
            ns_tid = self.get_track_id(ridx_n, MOSWireType.DS_GATE, wire_name='sup')
            ps_tid = self.get_track_id(ridx_p, MOSWireType.DS_GATE, wire_name='sup')
            vdd = self.connect_to_tracks([pports0.s, pports1.s], ps_tid, track_lower=0, track_upper=xr)
            vss = self.connect_to_tracks([nports0.s], ns_tid, track_lower=0, track_upper=xr)
            self.add_pin('VDD', vdd)
            self.add_pin('VSS', vss)

        self.sch_params = dict(
            seg_pstack0=seg_pstack0,
            seg_pstack1=seg_pstack1,
            seg_n0=seg_n0,
            seg_n1=seg_n1,
            lch=self.place_info.lch,
            w_p=self.place_info.get_row_place_info(ridx_p).row_info.width if w_p == 0 else w_p,
            w_n=self.place_info.get_row_place_info(ridx_n).row_info.width if w_n == 0 else w_n,
            th_n=self.place_info.get_row_place_info(ridx_n).row_info.threshold,
            th_p=self.place_info.get_row_place_info(ridx_p).row_info.threshold,
            stack_p=stack_p,
            stack_n=stack_n,
        )


class SARLogicUnit(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_unit_bot')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            substrate_row='True to add substrate row',
            has_pmos_sw='True to add differential signal to drive pmos switch in CDAC',
            msb='True to add initial flops to set bit',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            substrate_row=False,
            has_pmos_sw=False,
            msb=False,
        )

    def fill_tap(self, tile_idx, port_mode=SubPortMode.EVEN) -> None:
        """
        This method fill empty region with sub contact
        """
        min_fill_ncols = self.tech_cls.min_sub_col + 2 * self.tech_cls.min_sep_col + 4
        _, _, flip_tile = self.used_array.get_tile_info(tile_idx)
        intv_list = self.used_array.get_complement(tile_idx, 0, 0, self.num_cols)

        def get_diff_port(pmode):
            return SubPortMode.EVEN if pmode == SubPortMode.ODD else SubPortMode.ODD

        for intv in intv_list:
            intv_pair = intv[0]
            nspace = intv_pair[1] - intv_pair[0]
            if nspace < min_fill_ncols:
                continue
            else:
                _port_mode = get_diff_port(port_mode) if (intv_pair[0] + self.min_sep_col) & 1 else port_mode
                tap0 = self.add_substrate_contact(0, intv_pair[0] + self.min_sep_col + 2,
                                                  seg=nspace - 2 * self.min_sep_col - 4,
                                                  tile_idx=tile_idx, port_mode=_port_mode)
            tid0 = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=tile_idx)
            self.connect_to_tracks(tap0, tid0)
            intv_list = self.used_array.get_complement(tile_idx, 1, 0, self.num_cols)
        for intv in intv_list:
            intv_pair = intv[0]
            nspace = intv_pair[1] - intv_pair[0]
            if nspace < min_fill_ncols:
                continue
            else:
                _port_mode = get_diff_port(port_mode) if (intv_pair[0] + self.min_sep_col) & 1 else port_mode
                tap1 = self.add_substrate_contact(1, intv_pair[0] + self.min_sep_col + 2,
                                                  seg=nspace - 2 * self.min_sep_col - 4,
                                                  tile_idx=tile_idx, port_mode=_port_mode)
            tid1 = self.get_track_id(1, MOSWireType.DS, 'sup', tile_idx=tile_idx)
            self.connect_to_tracks(tap1, tid1)

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        substrate_row: bool = self.params['substrate_row']
        has_pmos_sw: bool = self.params['has_pmos_sw']
        msb: bool = self.params['msb']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        seg_oai: Dict[str, int] = seg_dict['oai']
        seg_lat: Dict[str, int] = seg_dict['latch']
        seg_flop: Dict[str, int] = seg_dict['flop']
        seg_buf: ImmutableList = seg_dict['buf']
        seg_inv_fb = seg_dict['oai_fb']
        seg_nor = seg_dict['nor']
        # seg_nand_done = seg_dict['nand_done']
        # seg_nand_state = seg_dict['nand_state']
        # seg_inv_done = seg_dict['inv_done']
        seg_buf_state = seg_dict['buf_state']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)
        nd0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nd1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pd0_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pd1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)

        oai_params = dict(pinfo=pinfo, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=substrate_row, sig_locs={},
                          min_len_mode={'in3': MinLenMode.MIDDLE, 'in2': MinLenMode.MIDDLE,
                                        'in1': MinLenMode.MIDDLE, 'in0': MinLenMode.MIDDLE})
        oai_params.update(**seg_oai)
        latch_params = dict(pinfo=pinfo, seg=seg_lat, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            vertical_sup=substrate_row, sig_locs={'nclkb': ng0_tidx, 'nclk': ng1_tidx, 'nin': ng2_tidx})
        inv_fb_n_params = dict(pinfo=pinfo, seg=seg_inv_fb, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                               vertical_sup=substrate_row, sig_locs={'nin': ng2_tidx, 'out': d_fb_tidx[0]})
        inv_fb_p_params = dict(pinfo=pinfo, seg=seg_inv_fb, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                               vertical_sup=substrate_row, sig_locs={'nin': ng2_tidx, 'out': d_fb_tidx[1]})
        flop_params = dict(pinfo=pinfo, seg=seg_flop, seg_ck=2, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=substrate_row, sig_locs={'nclkb': ng0_tidx, 'nclk': ng1_tidx, 'nin': ng2_tidx},
                           resetable=True)
        if has_pmos_sw:
            seg_buf = seg_buf if len(seg_buf) & 1 else seg_buf.to_list() + [seg_buf[-1]]
            buf_params = dict(pinfo=pinfo, seg_list=seg_buf[:-1], w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=substrate_row, sig_locs={'nin0': ng2_tidx, 'nin1': ng1_tidx},
                              vertical_out=False)
            buf_np_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                 vertical_sup=substrate_row,
                                 sig_locs={'nin0': ng2_tidx, 'nin1': ng1_tidx, 'nout0': nd0_tidx, 'pout0': pd1_tidx,
                                           'nout1': nd1_tidx, 'pout1': pd0_tidx},
                                 vertical_out=False, dual_output=True)
            pg_params = dict(pinfo=pinfo, seg=max(seg_buf[-1] - min_sep, 2),
                             w_p=w_p, wn=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             vertical_sup=substrate_row, vertical_in=False, vertical_out=False,
                             sig_locs={'nd': nd0_tidx, 'pd': pd1_tidx}, is_guarded=True)
            pg_master = self.new_template(PassGateCore, params=pg_params)
        else:
            # check buf seg length and make it even stage
            seg_buf = seg_buf[:-1] if len(seg_buf) & 1 else seg_buf
            buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=substrate_row, sig_locs={'nin0': ng2_tidx, 'nin1': ng1_tidx},
                              vertical_out=False)
            buf_np_params = buf_params
            pg_master = None

        inv_state_params = dict(pinfo=pinfo, seg=seg_buf_state[0], w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                vertical_sup=substrate_row, sig_locs={'nin': ng1_tidx})
        inv_init_params = dict(pinfo=pinfo, seg=seg_buf_state[0], w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                               vertical_sup=substrate_row, sig_locs={'nin': pg0_tidx})

        oai_master = self.new_template(OAICore, params=oai_params)
        inv_fb_n_master = self.new_template(InvCore, params=inv_fb_n_params)
        inv_fb_p_master = self.new_template(InvCore, params=inv_fb_p_params)
        inv_state_master = self.new_template(InvCore, params=inv_state_params)
        inv_init_master = self.new_template(InvCore, params=inv_init_params)
        latch_master = self.new_template(LatchCore, params=latch_params)
        flop_master = self.new_template(FlopCore, params=flop_params)

        buf_master = self.new_template(InvChainCore, params=buf_params)
        buf_np_master = self.new_template(InvChainCore, params=buf_np_params)

        # Row 0 - bit and retimer
        cur_col = 0
        inv_state = self.add_tile(inv_state_master, 0, cur_col)
        cur_col += inv_state_master.num_cols + min_sep
        cur_col += cur_col & 1
        latch = self.add_tile(latch_master, 0, cur_col)
        cur_col += latch_master.num_cols + min_sep
        cur_col += cur_col & 1
        if msb:
            flop_init = self.add_tile(flop_master, 0, cur_col)
            cur_col += flop_master.num_cols + min_sep
            cur_col += cur_col & 1
            inv_init = self.add_tile(inv_init_master, 0, cur_col)
            cur_col += inv_init_master.num_cols + min_sep
            cur_col += cur_col & 1
            self.connect_to_tracks(flop_init.get_pin('in'), self.get_track_id(1, MOSWireType.DS, 'sup'))
            self.connect_to_track_wires(flop_init.get_pin('out'), inv_init.get_pin('nin'))
        else:
            flop_init, inv_init = None, None

        flop_state = self.add_tile(flop_master, 0, cur_col)
        row0_ncol = cur_col + flop_master.num_cols
        row0_ncol += row0_ncol & 1

        # Row 1,2 - dn/p signal
        cur_col = 0
        oai_inv_n = self.add_tile(inv_fb_n_master, 1, cur_col)
        oai_inv_p = self.add_tile(inv_fb_p_master, 2, cur_col)
        cur_col += inv_fb_n_master.num_cols + min_sep
        cur_col += cur_col & 1
        oai_n = self.add_tile(oai_master, 1, cur_col)
        oai_p = self.add_tile(oai_master, 2, cur_col)
        cur_col += oai_master.num_cols + min_sep
        cur_col += cur_col & 1
        buf_n = self.add_tile(buf_np_master, 1, cur_col)
        buf_p = self.add_tile(buf_np_master, 2, cur_col)
        buf_m = self.add_tile(buf_master, 3, cur_col)
        row12_ncol = cur_col + buf_np_master.num_cols
        row12_ncol += row12_ncol & 1
        if has_pmos_sw:
            pg_n = self.add_tile(pg_master, 1, row12_ncol + min_sep)
            pg_p = self.add_tile(pg_master, 2, row12_ncol + min_sep)
            row12_ncol += pg_master.num_cols + min_sep
        else:
            pg_n, pg_p = None, None

        oai_out_tidx = oai_n.get_pin('out').track_id.base_index

        # Row 3 - dm signal
        cur_col = 1
        nor_params = dict(pinfo=pinfo, seg=seg_nor,
                          w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p, vertical_sup=substrate_row,
                          sig_locs={'nin0': ng0_tidx, 'nin1': ng1_tidx, 'out': oai_out_tidx})
        nor3_master = self.new_template(NOR3Core, params=nor_params)
        nor = self.add_tile(nor3_master, 3, cur_col)
        tot_seg = max(row0_ncol, row12_ncol)
        self.set_mos_size(tot_seg)

        # Route transistor parts

        # Connection between dn/dp
        # -- feedback inv --
        self.connect_to_track_wires(oai_n.get_pin('in<3>'), oai_inv_n.get_pin('out'))
        self.connect_to_track_wires(oai_p.get_pin('in<3>'), oai_inv_p.get_pin('out'))
        # -- oai to buf --
        self.connect_to_track_wires([buf_n.get_pin('in'), oai_inv_n.get_pin('in')], oai_n.get_pin('out'))
        self.connect_to_track_wires([buf_p.get_pin('in'), oai_inv_p.get_pin('in')], oai_p.get_pin('out'))
        # Connection for dm
        self.connect_to_track_wires(nor.get_pin('out'), buf_m.get_pin('in'))
        _, nor_in_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 5,
                                                align_track=oai_inv_p.get_pin('out').track_id.base_index, align_idx=0)
        _dn_fb_vm = self.connect_to_tracks([oai_inv_n.get_pin('in'), nor.get_pin('nin<1>')],
                                           TrackID(vm_layer, nor_in_tidx[2], tr_w_vm))
        _dp_fb_vm = self.connect_to_tracks([oai_inv_p.get_pin('in'), nor.get_pin('nin<0>')],
                                           TrackID(vm_layer, nor_in_tidx[3], tr_w_vm))
        comp_tidx_upper = self.grid.coord_to_track(vm_layer, buf_n.bound_box.xl, mode=RoundMode.GREATER_EQ)
        tidx_list = self.get_available_tracks(vm_layer, oai_p.get_pin('out').track_id.base_index,
                                              comp_tidx_upper, self.get_tile_info(1)[1], self.bound_box.yh,
                                              width=tr_w_vm, sep=tr_sp_vm)

        comp_n, comp_p = self.connect_differential_tracks(oai_n.get_pin('in<1>'), oai_p.get_pin('in<1>'), vm_layer,
                                                          tidx_list[0], tidx_list[1], width=tr_w_vm)
        self.connect_to_track_wires(latch.get_pin('nclkb'), inv_state.get_pin('out'))
        self.connect_wires([latch.get_pin('nclk'), inv_state.get_pin('nin')])

        flop_hm_list = [oai_n.get_pin('in<2>'), oai_p.get_pin('in<2>'), nor.get_pin('nin<2>'),
                        flop_state.get_pin('prst')]
        if msb:
            flop_hm_list.append(flop_init.get_pin('prst'))
        tidx_list = self.get_available_tracks(vm_layer, oai_p.get_pin('out').track_id.base_index,
                                              comp_tidx_upper, 0, self.bound_box.yh,
                                              width=tr_w_vm, sep=tr_sp_vm)
        rst_vm = self.connect_to_tracks(flop_hm_list, TrackID(vm_layer, tidx_list[0], tr_w_vm))
        _write_vm = self.connect_to_tracks([oai_n.get_pin('in<0>'), oai_p.get_pin('in<0>'),
                                            inv_state.get_pin('nout'), inv_state.get_pin('pout')],
                                           TrackID(vm_layer, nor_in_tidx[1], tr_w_vm))
        # wire_lower=nand_done_in_upper_coord)
        # Connect output dn/dp/dm
        if has_pmos_sw:
            out_vm_tidx = self.grid.coord_to_track(vm_layer, buf_n.bound_box.xh, mode=RoundMode.LESS_EQ)
            _, out_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 4, align_track=out_vm_tidx, align_idx=-1)
            out_vm_list = [self.connect_to_tracks([buf_m.get_pin('nout'), buf_m.get_pin('pout')],
                                                  TrackID(vm_layer, out_vm_locs[0], tr_w_vm)),
                           self.connect_to_tracks([buf_n.get_pin('noutb'), buf_n.get_pin('poutb')],
                                                  TrackID(vm_layer, out_vm_locs[1], tr_w_vm)),
                           self.connect_to_tracks([buf_p.get_pin('noutb'), buf_p.get_pin('poutb')],
                                                  TrackID(vm_layer, out_vm_locs[2], tr_w_vm))]
            self.connect_wires([buf_n.get_pin('nout'), pg_n.get_pin('ns'), buf_n.get_pin('pout'), pg_n.get_pin('ps')])
            self.connect_wires([buf_p.get_pin('nout'), pg_p.get_pin('ns'), buf_p.get_pin('pout'), pg_p.get_pin('ps')])
            self.add_pin('dm', out_vm_list[0])
            self.add_pin('dn_b', out_vm_list[1])
            self.add_pin('dp_b', out_vm_list[2])
            _, out_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3 + ['sig'] * 2, align_track=out_vm_tidx,
                                                    align_idx=0)
            out_vm_list.extend([self.connect_to_tracks([pg_n.get_pin('nd'), pg_n.get_pin('pd')],
                                                       TrackID(vm_layer, out_vm_locs[1], tr_w_vm)),
                                self.connect_to_tracks([pg_p.get_pin('nd'), pg_p.get_pin('pd')],
                                                       TrackID(vm_layer, out_vm_locs[2], tr_w_vm)), ])
            en_vm = self.connect_to_tracks([pg_p.get_pin('en'), pg_n.get_pin('en')],
                                           TrackID(vm_layer, out_vm_locs[-1], tr_w_vm))
            enb_vm = self.connect_to_tracks([pg_p.get_pin('enb'), pg_n.get_pin('enb')],
                                            TrackID(vm_layer, out_vm_locs[-2], tr_w_vm))
            self.connect_to_track_wires(en_vm, [pg_n.get_pin('VDD'), pg_p.get_pin('VDD')])
            self.connect_to_track_wires(enb_vm, [pg_n.get_pin('VSS'), pg_p.get_pin('VSS')])
            self.add_pin('dn', out_vm_list[3])
            self.add_pin('dp', out_vm_list[4])
        else:
            out_vm_tidx = self.grid.coord_to_track(vm_layer, self.bound_box.xh, mode=RoundMode.LESS_EQ)
            _, out_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 4, align_idx=-1, align_track=out_vm_tidx)
            out_vm_list = []
            for inst, tidx in zip([buf_m, buf_n, buf_p], out_vm_locs[:-1]):
                out_vm_list.append(self.connect_to_tracks([inst.get_pin('nout'), inst.get_pin('pout')],
                                                          TrackID(vm_layer, tidx, tr_w_vm)))
            self.add_pin('dm', out_vm_list[0])
            self.add_pin('dn', out_vm_list[1])
            self.add_pin('dp', out_vm_list[2])

        # Connect retimer's input
        # dp_vm = out_vm_list[4] if has_pmos_sw else out_vm_list[2]
        ret_in_tidx = nor_in_tidx[4]
        dp_m_hm = [buf_p.get_pin('nout'), buf_p.get_pin('pout')]
        dp_m_vm_tidx = self.grid.coord_to_track(vm_layer, dp_m_hm[0].lower, RoundMode.NEAREST)
        dp_m_vm = self.connect_to_tracks(dp_m_hm, TrackID(vm_layer, dp_m_vm_tidx, tr_w_vm))
        dp_m_hm = self.connect_to_tracks(dp_m_vm, self.get_track_id(1, MOSWireType.G, 'sig', 0, tile_idx=2))
        self.connect_to_tracks([dp_m_hm, latch.get_pin('nin')], TrackID(vm_layer, ret_in_tidx, tr_w_vm))

        bit_flop_vm_tidx = self.grid.coord_to_track(vm_layer, flop_state.get_pin('nin').lower, RoundMode.NEAREST)
        bit_vm_prev = self.connect_to_tracks(flop_state.get_pin('nin'), TrackID(vm_layer, bit_flop_vm_tidx, tr_w_vm))

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [inv_state, oai_p, oai_n, oai_inv_p, oai_inv_n, latch, nor, buf_m, buf_n, buf_p, flop_state]
        if msb:
            inst_list.extend([inv_init, flop_init])
        if has_pmos_sw:
            inst_list.extend([pg_n, pg_p])
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]
        # vss_hm = vss_hm[1:]

        # --- Fill tap --
        self.fill_tap(3, SubPortMode.EVEN)
        self.fill_tap(0, SubPortMode.EVEN)

        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        # vm_sep_margin = self.grid.get_sep_tracks(vm_layer, tr_w_sup_vm, tr_w_sup_vm, same_color=False)
        # vm_tr_info = self.grid.get_track_info(vm_layer)
        # vm_margin = vm_tr_info.pitch * vm_sep_margin // 2
        if msb:
            bit_prev = inv_init.get_pin('out')
            self.add_pin('bit_prev', bit_prev, connect=True)
        self.add_pin('bit', [latch.get_pin('clk'), flop_state.get_pin('out')], connect=True)

        if msb:
            self.add_pin('clk', [flop_state.get_pin('clk'), flop_init.get_pin('clk')], connect=True)
        else:
            self.add_pin('clk', flop_state.get_pin('clk'))

        self.add_pin('comp_p', comp_p)
        self.add_pin('comp_n', comp_n)
        self.add_pin('out_ret', latch.get_pin('out'))
        # self.add_pin('rst', flop_state.get_pin('rst'))
        self.add_pin('rst', rst_vm)
        self.add_pin('bit_prev', bit_vm_prev, connect=True)
        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)

        # vdd_xm_list, vss_xm_list = [], []
        # for idx in range(self.num_tile_rows):
        #     _sup, _ = self.export_xm_sup(idx, export_bot=True, given_locs=vm_locs)
        #     if idx & 1:
        #         vdd_xm_list.append(_sup)
        #     else:
        #         vss_xm_list.append(_sup)
        # self.add_pin('VDD_xm', vdd_xm_list, label='VDD', show=self.show_pins, connect=True)
        # self.add_pin('VSS_xm', vss_xm_list, label='VSS', show=self.show_pins, connect=True)

        sch_params_dict = dict(
            buf_state=inv_state_master.sch_params,
            latch=latch_master.sch_params,
            oai=oai_master.sch_params,
            oai_fb=inv_fb_n_master.sch_params,
            nor=nor3_master.sch_params,
            buf=buf_master.sch_params,
            buf_np=buf_np_master.sch_params,
            flop=flop_master.sch_params,
            has_pmos_sw=has_pmos_sw,
            msb=self.params['msb'],
        )
        if has_pmos_sw:
            sch_params_dict.update(pg=pg_master.sch_params)
        self.sch_params = sch_params_dict


class SARRetUnit(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_ret_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            substrate_row='True to add substrate row',
            sig_locs='Signal locations',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            substrate_row=False,
            sig_locs={},
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Dict[str, Any] = self.params['seg_dict']
        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        substrate_row: bool = self.params['substrate_row']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        seg_buf: List = seg_dict['buf']
        seg_lat: int = seg_dict['latch']
        seg_inv: int = seg_dict['inv']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm_sig = tr_manager.get_width(vm_layer, 'sig')
        tr_w_vm_clk = tr_manager.get_width(vm_layer, 'clk')

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        nd0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nd1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)

        latch_params = dict(pinfo=pinfo, seg=seg_lat, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            vertical_sup=substrate_row, sig_locs={'nclkb': ng1_tidx, 'nclk': ng0_tidx, 'pin': pg0_tidx})

        invb_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=substrate_row, sig_locs={'nin': ng0_tidx})
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=substrate_row, sig_locs={'nin': pg0_tidx})
        buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=substrate_row,
                          sig_locs={'nin0': pg0_tidx, 'nin1': pg1_tidx, 'nout0': nd0_tidx, 'nout1': nd1_tidx},
                          vertical_out=False)

        inv_master = self.new_template(InvCore, params=inv_params)
        invb_master = self.new_template(InvCore, params=invb_params)
        buf_master = self.new_template(InvChainCore, params=buf_params)
        latch_master = self.new_template(LatchCore, params=latch_params)

        cur_col = 0
        invb = self.add_tile(invb_master, 0, cur_col)
        cur_col += invb_master.num_cols + min_sep
        inv = self.add_tile(inv_master, 0, cur_col)
        cur_col += invb_master.num_cols + min_sep
        latch = self.add_tile(latch_master, 0, cur_col)
        cur_col += latch_master.num_cols
        buf = self.add_tile(buf_master, 0, cur_col)
        tot_seg = cur_col + buf_master.num_cols
        self.set_mos_size(tot_seg)

        self.connect_to_track_wires(invb.get_pin('out'), latch.get_pin('pclkb'))
        self.connect_to_track_wires(inv.get_pin('out'), latch.get_pin('nclk'))
        self.connect_wires([inv.get_pin('nin'), latch.get_pin('nclk')])
        self.connect_to_track_wires(latch.get_pin('out'), buf.get_pin('nin'))
        vdd_gate_list, vss_gate_list = [], []
        for inst in [invb, inv, latch, buf]:
            vdd_gate_list.append(inst.get_pin('VDD'))
            vss_gate_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_gate_list)
        vss_hm = self.connect_wires(vss_gate_list)
        out_tidx = sig_locs.get('out', 0)

        out_vm_tidx = self.grid.coord_to_track(vm_layer, buf.get_pin('nout').middle, mode=RoundMode.NEAREST)
        out_vm = self.connect_to_tracks([buf.get_pin('nout'), buf.get_pin('pout')],
                                        TrackID(vm_layer, out_vm_tidx + out_tidx, tr_w_vm_sig))
        clk_in_vm_tidx = tr_manager.get_next_track(vm_layer, invb.get_pin('out').track_id.base_index, 'clk', 'clk')
        in_vm_tidx = tr_manager.get_next_track(vm_layer, inv.get_pin('out').track_id.base_index, 'clk', 'clk')
        in_vm_tidx += sig_locs.get('in', 0)

        clk_in_vm = self.connect_to_tracks(invb.get_pin('nin'), TrackID(vm_layer, clk_in_vm_tidx, tr_w_vm_clk))
        in_vm = self.connect_to_tracks(latch.get_pin('nin'), TrackID(vm_layer, in_vm_tidx, tr_w_vm_sig),
                                       min_len_mode=MinLenMode.MIDDLE)

        self.add_pin('in', in_vm)
        self.add_pin('in_hm', latch.get_pin('nin'))
        self.add_pin('out', out_vm)
        self.add_pin('clk', clk_in_vm)
        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)

        self.sch_params = dict(
            latch=latch_master.sch_params,
            inv=inv_master.sch_params,
            buf=buf_master.sch_params,
        )


class SARLogicArray(SARLogicUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._lower_layer_routing = False

    @property
    def lower_layer_routing(self):
        return self._lower_layer_routing

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_array')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            logic_unit_row_arr='Array of unit cells',
            ret_unit_row_arr='Array of retimer cells',
            substrate_row='True to add substrate row',
            has_pmos_sw='True to add differential signal to drive pmos switch in CDAC',
            lower_layer_routing='Avoid use metal layer above xm for routing'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            logic_unit_row_arr=[],
            ret_unit_row_arr=[],
            substrate_row=False,
            has_pmos_sw=False,
            lower_layer_routing=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Dict[str, Any] = self.params['seg_dict']
        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        substrate_row: bool = self.params['substrate_row']
        has_pmos_sw: bool = self.params['has_pmos_sw']
        lower_layer_routing: bool = self.params['lower_layer_routing']
        logic_unit_row_arr: List[int] = self.params['logic_unit_row_arr']
        ret_unit_row_arr: List[int] = self.params['ret_unit_row_arr']
        self._lower_layer_routing = lower_layer_routing

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        seg_ret: Dict[str, int] = seg_dict['retimer']
        seg_buf_int: List[int] = seg_dict['buf_int']
        seg_buf_out: List[int] = seg_dict['buf_out']
        seg_logic: ImmutableSortedDict[str, Any] = seg_dict['logic']
        logic_scale_list: List[int] = seg_dict['logic_scale_list']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # Check logic unit arrangement
        num_bits = len(logic_scale_list)
        if not logic_unit_row_arr:
            logic_unit_row_arr = [num_bits]
        if not ret_unit_row_arr:
            ret_unit_row_arr = [num_bits]
        if num_bits != sum(logic_unit_row_arr):
            raise ValueError("Logic unit array arrangement doesn't match number of units")
        if num_bits != sum(ret_unit_row_arr):
            raise ValueError("Logic unit array arrangement doesn't match number of units")

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm_clk = tr_manager.get_width(vm_layer, 'clk')
        tr_w_vm_sig = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm_sig = tr_manager.get_width(xm_layer, 'sig')
        tr_w_xm_clk = tr_manager.get_width(xm_layer, 'clk')
        tr_sp_xm_sig = tr_manager.get_sep(xm_layer, ('sig', 'sig'))
        tr_sp_vm_clk = tr_manager.get_sep(vm_layer, ('clk', 'clk'))
        tr_sp_vm_sig = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        # tr_sp_ym_sig = tr_manager.get_sep(ym_layer, ('sig', 'sig'))
        # tr_w_ym_clk = tr_manager.get_width(ym_layer, 'clk')
        rt_layer = vm_layer if lower_layer_routing else ym_layer
        tr_w_rt_clk = tr_manager.get_width(rt_layer, 'clk')
        tr_w_rt_sig = tr_manager.get_width(rt_layer, 'sig')
        tr_sp_rt_sig = tr_manager.get_sep(rt_layer, ('sig', 'sig'))

        buf_int_params = dict(pinfo=pinfo, seg_list=seg_buf_int, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=substrate_row, sig_locs={}, dual_output=True, vertical_out=False)
        buf_out_params = dict(pinfo=pinfo, seg_list=seg_buf_out, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=substrate_row, sig_locs={}, dual_output=True, vertical_out=False)
        logic_unit_params_list = []
        for idx, scale in enumerate(logic_scale_list):
            _seg_dict = copy.deepcopy(seg_logic.to_dict())
            for key, val in _seg_dict.items():
                if key == 'nor' or key == 'oai_fb':
                    _seg_dict[key] = val * scale
                elif key == 'buf' or key == 'buf_state':
                    _seg_dict[key] = [_seg * scale for _seg in val]
                elif key == 'oai':
                    for _key, _val in val.items():
                        _seg_dict[key] = dict(_seg_dict[key])
                        _seg_dict[key][_key] = _seg_dict[key][_key] * scale
            msb = (idx == len(logic_scale_list) - 1)
            logic_unit_params_list.append(dict(pinfo=pinfo, seg_dict=_seg_dict, substrate_row=substrate_row,
                                               has_pmos_sw=has_pmos_sw, w_n=w_n, w_p=w_p, msb=msb))

        logic_unit_master_list = [self.new_template(SARLogicUnit, params=_params) for _params in logic_unit_params_list]
        buf_int_master = self.new_template(InvChainCore, params=buf_int_params)
        buf_out_master = self.new_template(InvChainCore, params=buf_out_params)

        if lower_layer_routing:
            tot_wires = 2 * num_bits if has_pmos_sw else num_bits
            wire_type = 'clk' if tr_w_vm_clk + tr_sp_vm_clk > tr_sp_vm_sig + tr_w_vm_sig else 'sig'
            ntr, _ = tr_manager.place_wires(vm_layer, [wire_type] * tot_wires)
            ncol_rt = self.arr_info.get_column_span(vm_layer, ntr)
        else:
            ncol_rt = 0

        origin_col = ncol_rt // 2

        # Add clock buffer
        buf_int = self.add_tile(buf_int_master, 1, origin_col)
        buf_out = self.add_tile(buf_out_master, 0, origin_col)

        # Add retimers
        retimer_ncol, retimer_row_list = 0, []
        ret_params = dict(pinfo=pinfo, seg_dict=seg_ret, w_n=w_n, w_p=w_p, substrate_row=substrate_row)
        ret_master = self.new_template(SARRetUnit, params=ret_params)

        ret_start_ncol = max(buf_out_master.num_cols, buf_int_master.num_cols) + min_sep + origin_col
        for idx, row_num in enumerate(ret_unit_row_arr):
            ret_shift_params = dict(pinfo=pinfo, seg_dict=seg_ret, w_n=w_n, w_p=w_p, substrate_row=substrate_row,
                                    sig_locs={'out': idx, 'in': idx})
            ret_shift_master = self.new_template(SARRetUnit, params=ret_shift_params)
            _retimer_list = []
            if idx == 0:
                cur_col_ret = ret_start_ncol
            elif idx == 1:
                cur_col_ret = ret_start_ncol
            else:
                cur_col_ret = origin_col
            for jdx in range(row_num):
                _retimer_list.append(self.add_tile(ret_shift_master, idx, cur_col_ret))
                cur_col_ret += ret_shift_master.num_cols
            retimer_row_list.append(_retimer_list)
            retimer_ncol = max(cur_col_ret, retimer_ncol)

        logic_row = max(1, len(ret_unit_row_arr)) + 1

        # Add logic cells
        logic_ncol, logic_row_list = 0, []
        num_logic_rows = logic_unit_master_list[0].num_tile_rows
        for idx, row_num in enumerate(logic_unit_row_arr):
            _logic_list = []
            cur_col, cur_row = origin_col, logic_row + idx * num_logic_rows
            cur_row += (cur_row & 1) * 4
            for jdx in range(row_num):
                if idx & 1:
                    _master = logic_unit_master_list[sum(logic_unit_row_arr[:idx + 1]) - jdx - 1]
                else:
                    _master = logic_unit_master_list[sum(logic_unit_row_arr[:idx]) + jdx]
                if idx & 1:
                    _logic_list.append(self.add_tile(_master, cur_row, cur_col + _master.num_cols, flip_lr=True))
                else:
                    _logic_list.append(self.add_tile(_master, cur_row, cur_col))
                cur_col += _master.num_cols + min_sep
            _logic_list = _logic_list[::-1] if idx & 1 else _logic_list
            logic_row_list.append(_logic_list)
            logic_ncol = max(cur_col, logic_ncol)
        tot_ncol = ncol_rt // 2 + self.num_cols if lower_layer_routing else self.num_cols
        self.set_mos_size(tot_ncol)

        # Get higehr layer supplies
        vdd_xm_list, vss_xm_list = [], []
        for idx in range(logic_row, self.num_tile_rows):
            _sup, _ = export_xm_sup(self, idx, export_bot=True)
            if idx & 1:
                vdd_xm_list.append(_sup)
            else:
                vss_xm_list.append(_sup)

        # Connect clk signal together
        retimer_flatten_list = [item for sublist in retimer_row_list for item in sublist]
        logic_flatten_list = [item for sublist in logic_row_list for item in sublist]

        ret_clk_list = [inst.get_pin('clk') for inst in retimer_flatten_list]
        ret_clk_vm = self.connect_wires(ret_clk_list)
        clk_int_vm_tidx = self.grid.coord_to_track(vm_layer, buf_int.get_pin('nout').upper, mode=RoundMode.NEAREST)
        clk_out_vm_tidx = tr_manager.get_next_track(vm_layer, clk_int_vm_tidx, 'clk', 'clk', up=False)
        buf_int_out_vm = self.connect_to_tracks([buf_int.get_pin('nout'), buf_int.get_pin('pout')],
                                                TrackID(vm_layer, clk_int_vm_tidx, tr_w_vm_clk))
        buf_out_out_vm = self.connect_to_tracks([buf_out.get_pin('nout'), buf_out.get_pin('pout')],
                                                TrackID(vm_layer, clk_out_vm_tidx, tr_w_vm_clk))

        ret_clk_xm_tidx = self.grid.coord_to_track(xm_layer, ret_clk_vm[0].upper, mode=RoundMode.NEAREST)
        ret_clk_xm = self.connect_to_tracks(ret_clk_vm, TrackID(xm_layer, ret_clk_xm_tidx, tr_w_xm_clk, grid=self.grid),
                                            min_len_mode=MinLenMode.MIDDLE)
        self.connect_to_track_wires(buf_int_out_vm, ret_clk_xm)
        self.connect_to_track_wires(buf_int_out_vm, buf_out.get_pin('nin'))

        ret_out_list = [inst.get_pin('out') for inst in retimer_flatten_list]
        ret_in_list = [inst.get_pin('in') for inst in retimer_flatten_list]
        ret_out_list.sort(key=lambda x: x.track_id.base_index)
        ret_in_list.sort(key=lambda x: x.track_id.base_index)

        ret_out_list = self.extend_wires(ret_out_list, lower=self.get_tile_info(0)[1])
        ret_in_list = self.extend_wires(ret_in_list, upper=self.get_tile_info(len(ret_unit_row_arr))[1])

        # Connection for logic cells
        # -- Rst signal --
        logic_rst_list = []
        for inst_row in logic_row_list:
            rst_vm_list = [inst.get_pin('rst') for inst in inst_row]
            rst_xm_tidx = self.grid.coord_to_track(xm_layer, rst_vm_list[0].lower, mode=RoundMode.NEAREST)
            logic_rst_list.append(self.connect_to_tracks(rst_vm_list,
                                                         TrackID(xm_layer, rst_xm_tidx, tr_w_xm_clk, grid=self.grid),
                                                         min_len_mode=MinLenMode.MIDDLE))

        # rst_buf_vm_tidx = self.grid.coord_to_track(vm_layer, buf_int.get_pin('nin').lower, mode=RoundMode.LESS_EQ)
        rst_buf_vm_tidx = tr_manager.get_next_track(vm_layer, buf_int.get_pin('outb').track_id.base_index, 'sig', 'sig',
                                                    up=False)
        rst_buf_vm = self.connect_to_tracks(buf_int.get_pin('nin'), TrackID(vm_layer, rst_buf_vm_tidx, tr_w_vm_clk))
        rst_buf_xm_tidx = self.grid.coord_to_track(xm_layer, rst_buf_vm.middle, mode=RoundMode.NEAREST)
        rst_buf_xm = self.connect_to_tracks(rst_buf_vm, TrackID(xm_layer, rst_buf_xm_tidx, tr_w_xm_clk),
                                            min_len_mode=MinLenMode.MIDDLE)

        # rt layer connection
        rt_tidx_start = self.arr_info.col_to_track(rt_layer, 0)
        rt_tidx_stop = self.arr_info.col_to_track(rt_layer, self.num_cols)
        if lower_layer_routing:
            rt_tidx_list = self.get_available_tracks(rt_layer, rt_tidx_start, rt_tidx_stop, self.bound_box.yl,
                                                     self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
            rt_tidx_coord_list = [self.grid.track_to_coord(rt_layer, x) for x in rt_tidx_list]
            rst_rt_tidx = self.get_nearest_tidx(rst_buf_xm, rt_tidx_list, rt_tidx_coord_list)
            clk_rt_tidx = self.get_nearest_tidx(rst_buf_xm, rt_tidx_list, rt_tidx_coord_list)
        else:
            rst_rt_tidx = self.grid.coord_to_track(rt_layer, rst_buf_xm.lower, mode=RoundMode.LESS_EQ)
            clk_midb_rt_tidx = tr_manager.get_next_track(rt_layer, rst_rt_tidx, 'clk', 'clk')
            clk_rt_tidx = tr_manager.get_next_track(rt_layer, clk_midb_rt_tidx, 'clk', 'clk')
            ym_tidx_start = tr_manager.get_next_track(rt_layer, clk_rt_tidx, 'clk', 'clk')

        rst_rt = self.connect_to_tracks(logic_rst_list + [rst_buf_xm], TrackID(rt_layer, rst_rt_tidx, tr_w_rt_clk))
        # -- clk signal --
        logic_clk_list = []
        for inst_row in logic_row_list:
            clk_vm_list = [inst.get_all_port_pins('clk') for inst in inst_row]
            clk_vm_list = [warr for warr_list in clk_vm_list for warr in warr_list]
            clk_xm_tidx = self.grid.coord_to_track(xm_layer, clk_vm_list[0].upper, mode=RoundMode.NEAREST)
            logic_clk_list.append(self.connect_to_tracks(clk_vm_list, TrackID(xm_layer, clk_xm_tidx, tr_w_xm_clk),
                                                         min_len_mode=MinLenMode.MIDDLE))

        clk_rt = self.connect_to_tracks(logic_clk_list, TrackID(rt_layer, clk_rt_tidx, tr_w_rt_clk))

        # -- Compn/Compp signal --
        logic_compn_list, logic_compp_list = [], []
        for inst_row in logic_row_list:
            rst_vm_list = [inst.get_pin('rst') for inst in inst_row]
            compn_list = [inst.get_pin('comp_n') for inst in inst_row]
            compp_list = [inst.get_pin('comp_p') for inst in inst_row]
            rst_xm_tidx = self.grid.coord_to_track(xm_layer, rst_vm_list[0].lower, mode=RoundMode.NEAREST)
            _, comp_xm_locs = tr_manager.place_wires(xm_layer, ['clk', 'sig', 'sig'], rst_xm_tidx, 0)
            logic_compn_list.append(self.connect_to_tracks(compn_list, TrackID(xm_layer, comp_xm_locs[1], tr_w_xm_sig),
                                                           min_len_mode=MinLenMode.MIDDLE))
            logic_compp_list.append(self.connect_to_tracks(compp_list, TrackID(xm_layer, comp_xm_locs[2], tr_w_xm_sig),
                                                           min_len_mode=MinLenMode.MIDDLE))

        if lower_layer_routing:

            comp_p_ym = logic_compp_list
            comp_n_ym = logic_compn_list
        else:
            # ---- Connect comp/n to ym ----
            middle_coord = self.arr_info.col_to_coord(self.num_cols // 2)
            tr_w_rt_ana_sig = tr_manager.get_width(rt_layer, 'ana_sig')
            _, comp_ym_locs = tr_manager.place_wires(ym_layer, ['ana_sig'] * 2, center_coord=middle_coord)
            comp_p_ym = self.connect_to_tracks(logic_compp_list, TrackID(rt_layer, comp_ym_locs[0], tr_w_rt_ana_sig),
                                               track_upper=self.bound_box.yh)
            comp_n_ym = self.connect_to_tracks(logic_compn_list, TrackID(rt_layer, comp_ym_locs[1], tr_w_rt_ana_sig),
                                               track_upper=self.bound_box.yh)

        # -- bit/bit_nxt --
        out_ret_list = []
        last_bit = None
        bit_nxt_list = []
        ym_tidx_stop = self.grid.coord_to_track(ym_layer, self.bound_box.xh, mode=RoundMode.LESS)
        for idx, inst_row in enumerate(logic_row_list):
            _row = logic_row + (idx + 1) * num_logic_rows
            _coord = self.get_tile_info(_row)[1]
            _tidx_start = self.grid.coord_to_track(xm_layer, _coord, mode=RoundMode.GREATER_EQ)
            num_units = len(inst_row)
            _, xm_locs = tr_manager.place_wires(xm_layer, ['sup'] + ['sig'] * (num_units + 3), _tidx_start, 0)
            xm_locs = xm_locs[1:]  # Avoid put  on top of sup
            bit_sig_list = []
            bit_nxt_sig_list = []
            # inst_row = inst_row[::-1] if idx & 1 else inst_row
            for jdx, inst in enumerate(inst_row):
                if jdx & 1:
                    _tidx, _tidx_nxt = xm_locs[1], xm_locs[2]
                else:
                    _tidx, _tidx_nxt = xm_locs[2], xm_locs[1]
                if idx == len(logic_row_list) - 1 and jdx == len(inst_row) - 1:
                    _tidx = xm_locs[0]
                bit_sig_list.append(self.connect_to_tracks(inst.get_all_port_pins('bit_prev'),
                                                           TrackID(xm_layer, _tidx, tr_w_xm_sig)))
                bit_nxt_sig_list.append(self.connect_to_tracks(inst.get_all_port_pins('bit'),
                                                               TrackID(xm_layer, _tidx_nxt, tr_w_xm_sig)))
            bit_nxt_list.extend(bit_nxt_sig_list)
            for _bit, _bitnxt in zip(bit_nxt_sig_list[1:], bit_sig_list[:-1]):
                self.connect_wires([_bit, _bitnxt])
            out_ret_list.extend([self.connect_to_tracks(inst_row[idx].get_pin('out_ret'),
                                                        TrackID(xm_layer, xm_locs[3 + idx], tr_w_xm_sig))
                                 for idx in range(num_units)])
            if idx > 0:
                _coord = max(bit_nxt_sig_list[0].middle, last_bit.middle) if idx & 1 else \
                    min(bit_nxt_sig_list[0].middle, last_bit.middle)
                if lower_layer_routing:
                    rt_tidx_list = self.get_available_tracks(rt_layer, rt_tidx_start, rt_tidx_stop,
                                                             last_bit.bound_box.yh,
                                                             self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                    rt_tidx_coord_list = [self.grid.track_to_coord(rt_layer, x) for x in rt_tidx_list]
                    _y_wire = self.connect_to_tracks([last_bit, bit_nxt_sig_list[0]],
                                                     TrackID(rt_layer, self.get_nearest_tidx(last_bit, rt_tidx_list,
                                                                                             rt_tidx_coord_list,
                                                                                             _coord),
                                                             tr_w_rt_sig))
                else:
                    ym_tidx_list = self.get_available_tracks(ym_layer, ym_tidx_start, ym_tidx_stop,
                                                             last_bit.bound_box.yh,
                                                             self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                    ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                    _y_wire = self.connect_to_tracks([last_bit, bit_nxt_sig_list[0]],
                                                     TrackID(ym_layer, self.get_nearest_tidx(last_bit, ym_tidx_list,
                                                                                             ym_tidx_coord_list,
                                                                                             _coord),
                                                             tr_w_rt_sig))
                # _bit_conn_tidx = self.grid.coord_to_track(ym_layer, _coord, mode=RoundMode.NEAREST)
            last_bit = bit_sig_list[-1]

        # # Connect clk midb
        # self.connect_to_tracks([clk_midb_xm, last_bit], TrackID(rt_layer, clk_midb_rt_tidx, tr_w_rt_clk))

        # -- retimer input --
        ret_top_tileinfo, ret_top_y, _ = self.get_tile_info(len(ret_unit_row_arr))
        # retimer_in_tidx_start = self.grid.coord_to_track(xm_layer, self.get_tile_info(len(ret_unit_row_arr))[1],
        #                                                  mode=RoundMode.NEAREST)
        _, retimer_in_xm_locs = tr_manager.place_wires(xm_layer, ['sig'] * (num_bits + 2),
                                                       center_coord=ret_top_y + ret_top_tileinfo.height // 2)
        retimer_in_xm_list = []
        for tidx, ret_in in zip(retimer_in_xm_locs[1:-1], ret_in_list):
            retimer_in_xm_list.append(self.connect_to_tracks(ret_in, TrackID(xm_layer, tidx, tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))

        # Connect from logic to retimer
        # start_ym_tidx = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.NEAREST)
        if lower_layer_routing:
            rt_tidx_core_l = self.arr_info.col_to_track(rt_layer, ncol_rt // 2)
            rt_tidx_core_h = self.arr_info.col_to_track(rt_layer, self.num_cols - ncol_rt // 2)
            rt_tidx_list_lower = self.get_available_tracks(rt_layer, rt_tidx_start, rt_tidx_core_l, self.bound_box.yl,
                                                           self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
            rt_tidx_list_upper = self.get_available_tracks(rt_layer, rt_tidx_core_h, rt_tidx_stop, self.bound_box.yl,
                                                           self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
            retimer_in_tidx = rt_tidx_list_lower[-1 - num_bits // 2:] + rt_tidx_list_upper[:num_bits // 2]
            for idx, (ret_in, logic_out) in enumerate(zip(retimer_in_xm_list, out_ret_list)):
                self.connect_to_tracks([ret_in, logic_out], TrackID(rt_layer, retimer_in_tidx[idx], tr_w_rt_sig))

        else:
            stop_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.NEAREST)
            tidx_list = self.get_available_tracks(ym_layer, ym_tidx_start, stop_ym_tidx,
                                                  0, self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
            retimer_in_tidx = tidx_list[:num_bits // 2] + tidx_list[-1 - num_bits // 2:]
            for idx, (ret_in, logic_out) in enumerate(zip(retimer_in_xm_list, out_ret_list)):
                self.connect_to_tracks([ret_in, logic_out], TrackID(ym_layer, retimer_in_tidx[idx], tr_w_rt_sig))

        # Connect dm/dn/dp dp_b/dn_b to xm_layer
        dm_xm_list, dn_xm_list, dp_xm_list, dpb_xm_list, dnb_xm_list = [], [], [], [], []
        for idx, inst_row in enumerate(logic_row_list):
            _row_start = logic_row + idx * num_logic_rows + 1
            _row_stop = logic_row + (idx + 1) * num_logic_rows + 1
            _tidx_start = self.grid.coord_to_track(xm_layer, self.get_tile_info(_row_start)[1],
                                                   mode=RoundMode.GREATER)
            _tidx_stop = self.grid.coord_to_track(xm_layer, self.get_tile_info(_row_stop)[1], mode=RoundMode.LESS_EQ)
            tidx_list = self.get_available_tracks(xm_layer, _tidx_start, _tidx_stop, self.bound_box.xl,
                                                  self.bound_box.xh, width=tr_w_xm_sig, sep=tr_sp_xm_sig)[1:-1]
            # inst_row = inst_row[::-1] if idx & 1 else inst_row
            if has_pmos_sw:
                for jdx, inst in enumerate(inst_row):
                    dm_xm_list.append(self.connect_to_tracks(inst.get_pin('dm'),
                                                             TrackID(xm_layer, tidx_list[5 * jdx], tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))
                    dn_xm_list.append(self.connect_to_tracks(inst.get_pin('dn'),
                                                             TrackID(xm_layer, tidx_list[5 * jdx + 1], tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))
                    dp_xm_list.append(self.connect_to_tracks(inst.get_pin('dp'),
                                                             TrackID(xm_layer, tidx_list[5 * jdx + 2], tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))
                    dnb_xm_list.append(self.connect_to_tracks(inst.get_pin('dn_b'),
                                                              TrackID(xm_layer, tidx_list[5 * jdx + 3], tr_w_xm_sig),
                                                              min_len_mode=MinLenMode.MIDDLE))
                    dpb_xm_list.append(self.connect_to_tracks(inst.get_pin('dp_b'),
                                                              TrackID(xm_layer, tidx_list[5 * jdx + 4], tr_w_xm_sig),
                                                              min_len_mode=MinLenMode.MIDDLE))
            else:
                for jdx, inst in enumerate(inst_row):
                    dm_xm_list.append(self.connect_to_tracks(inst.get_pin('dm'),
                                                             TrackID(xm_layer, tidx_list[3 * jdx], tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))
                    dn_xm_list.append(self.connect_to_tracks(inst.get_pin('dn'),
                                                             TrackID(xm_layer, tidx_list[3 * jdx + 1], tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))
                    dp_xm_list.append(self.connect_to_tracks(inst.get_pin('dp'),
                                                             TrackID(xm_layer, tidx_list[3 * jdx + 2], tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))

        #
        _tidx_start = self.grid.coord_to_track(ym_layer, self.bound_box.xl, mode=RoundMode.GREATER_EQ)
        _tidx_stop = self.grid.coord_to_track(ym_layer, self.bound_box.xh, mode=RoundMode.LESS)
        if has_pmos_sw:
            for idx, pin in enumerate(dm_xm_list):
                if lower_layer_routing:
                    _y_wire = pin
                else:
                    ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                             self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                    ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                    _y_wire = self.connect_to_tracks(pin, TrackID(ym_layer,
                                                                  self.get_nearest_tidx(pin, ym_tidx_list,
                                                                                        ym_tidx_coord_list),
                                                                  tr_w_rt_sig), track_upper=self.bound_box.yh)
                self.add_pin(f'dm<{idx}>', _y_wire, mode=PinMode.UPPER)
            for d_list, pname in zip([dpb_xm_list, dn_xm_list], ['dp_b', 'dn']):
                for idx, pin in enumerate(d_list):
                    if lower_layer_routing:
                        _y_wire = pin
                    else:
                        ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                                 self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                        ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                        _ym_wire_tidx = self.get_nearest_tidx(pin, ym_tidx_list, ym_tidx_coord_list)
                        _y_wire = self.connect_to_tracks(pin, TrackID(ym_layer, _ym_wire_tidx,
                                                                      tr_w_rt_sig), track_upper=self.bound_box.yh)
                    self.add_pin(f'{pname}<{idx}>', _y_wire, mode=PinMode.UPPER)
            for d_list, pname in zip([dnb_xm_list, dp_xm_list], ['dn_b', 'dp']):
                for idx, pin in enumerate(d_list):
                    if lower_layer_routing:
                        _y_wire = pin
                    else:
                        ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                                 self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                        ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                        _ym_wire_tidx = self.get_nearest_tidx(pin, ym_tidx_list, ym_tidx_coord_list)
                        _y_wire = self.connect_to_tracks(pin, TrackID(ym_layer, _ym_wire_tidx,
                                                                      tr_w_rt_sig), track_upper=self.bound_box.yh)
                    self.add_pin(f'{pname}<{idx}>', _y_wire, mode=PinMode.UPPER)

        else:
            for d_list, pname in zip([dm_xm_list, dn_xm_list, dp_xm_list], ['dm', 'dn', 'dp']):
                for idx, pin in enumerate(d_list):
                    if lower_layer_routing:
                        _y_wire = pin
                    else:
                        ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                                 self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                        ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                        _y_wire = self.connect_to_tracks(pin,
                                                         TrackID(ym_layer,
                                                                 self.get_nearest_tidx(pin, ym_tidx_list,
                                                                                       ym_tidx_coord_list),
                                                                 tr_w_rt_sig), track_upper=self.bound_box.yh)
                    self.add_pin(f'{pname}<{idx}>', _y_wire, mode=PinMode.UPPER)
        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [buf_int, buf_out] + retimer_flatten_list + logic_flatten_list
        for inst in inst_list:
            vdd_list.append(inst.get_all_port_pins('VDD'))
            vss_list.append(inst.get_all_port_pins('VSS'))
        vdd_list = [vdd for sublist in vdd_list for vdd in sublist]
        vss_list = [vss for sublist in vss_list for vss in sublist]
        vdd_hm = self.connect_wires(vdd_list)
        vss_hm = self.connect_wires(vss_list)
        # for pname, plist in zip(['dm', 'dn', 'dp'], [dm_xm_list, dn_xm_list, dp_xm_list]):
        #     for idx, pin in enumerate(plist):
        #         self.add_pin(f'{pname}<{idx}>', pin)
        for idx, pin in enumerate(bit_nxt_list):
            self.add_pin(f'state<{idx}>', pin)
        self.add_pin('VDD_hm', vdd_hm, show=self.show_pins)
        self.add_pin('VSS_hm', vss_hm, show=self.show_pins)

        _sup, _ = export_xm_sup(self, 0, export_bot=True)
        vss_xm_list.append(_sup)
        _sup, _ = export_xm_sup(self, 1, export_bot=True)
        vdd_xm_list.append(_sup)
        _sup, _ = export_xm_sup(self, 2, export_bot=True)
        vss_xm_list.append(_sup)
        _sup, _ = export_xm_sup(self, 3, export_bot=True)
        vdd_xm_list.append(_sup)
        _sup, _ = export_xm_sup(self, 4, export_bot=True)
        vss_xm_list.append(_sup)
        self.add_pin('VDD_xm', vdd_xm_list, label='VDD', show=self.show_pins, connect=True)
        self.add_pin('VSS_xm', vss_xm_list, label='VSS', show=self.show_pins, connect=True)

        self.add_pin('rst', rst_rt)
        self.add_pin('sar_clk', clk_rt)
        self.add_pin('comp_p', comp_p_ym)
        self.add_pin('comp_n', comp_n_ym)
        self.add_pin('clk_out', buf_out_out_vm)
        [self.add_pin(f'data_out<{idx}>', pin) for idx, pin in enumerate(ret_out_list)]
        self.sch_params = dict(
            ret=ret_master.sch_params,
            buf_clk=buf_int_master.sch_params,
            buf_out=buf_out_master.sch_params,
            logic_list=[master.sch_params for master in logic_unit_master_list][::-1],
            nbits=num_bits,
            has_pmos_sw=has_pmos_sw,
        )

    @classmethod
    def get_nearest_tidx(cls, wire: WireArray, aval_tidx_list: List[HalfInt], aval_tidx_coord_list: List[float],
                         coord: Union[int, None] = None):
        wire_coord = wire.middle if coord is None else coord
        nearest_track_coord = min(aval_tidx_coord_list, key=lambda x: abs(x - wire_coord))
        nearest_idx = aval_tidx_coord_list.index(nearest_track_coord)
        nearest_track = aval_tidx_list[nearest_idx]
        aval_tidx_list.pop(nearest_idx)
        aval_tidx_coord_list.pop(nearest_idx)
        return nearest_track


class SARLogic(SARLogicUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_bot')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            w_n='nmos width',
            w_p='pmos width',
            pinfo='The MOSBasePlaceInfo object.',
            clkgen='Parameter for clock generator',
            logic_array='Parameter for logic array',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        w_p = self.params['w_p']
        w_n = self.params['w_n']
        gate_tr_manager_override = TrackManager(self.grid, self.params['tr_widths'], self.params['tr_spaces'])
        self.draw_base(pinfo)

        clk_gen_param: ImmutableSortedDict[str, Any] = self.params['clkgen']
        logic_array_param: ImmutableSortedDict[str, Any] = self.params['logic_array']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm_sig = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm_clk = tr_manager.get_width(xm_layer, 'clk')
        tr_w_ym_sig = tr_manager.get_width(ym_layer, 'sig')
        tr_sp_vm_sig = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        clk_gen_param = clk_gen_param.copy(append={'pinfo': pinfo, 'w_n': w_n, 'w_p': w_p,
                                                   'tr_manager': gate_tr_manager_override})
        logic_array_param = logic_array_param.copy(append={'pinfo': pinfo, 'w_n': w_n, 'w_p': w_p})
        logic_array_master = self.new_template(SARLogicArray, params=logic_array_param)
        clk_gen_master = self.new_template(SARAsyncClkSimple, params=clk_gen_param)

        # Add clock buffer
        logic_ntile = logic_array_master.num_tile_rows
        logic_array = self.add_tile(logic_array_master, 0, 0)
        clk_gen = self.add_tile(clk_gen_master, logic_ntile, 0)

        self.set_mos_size()

        clk_start_hm = clk_gen.get_pin('start')
        clk_stop_hm = clk_gen.get_pin('stop')
        start_vm_tidx = self.grid.coord_to_track(vm_layer, clk_start_hm.lower, mode=RoundMode.NEAREST)
        stop_vm_tidx = self.grid.coord_to_track(vm_layer, clk_stop_hm.lower, mode=RoundMode.LESS_EQ)
        clk_start_vm = self.connect_to_tracks(clk_start_hm, TrackID(vm_layer, start_vm_tidx, tr_w_vm_sig))
        clk_stop_vm = self.connect_to_tracks(clk_stop_hm, TrackID(vm_layer, stop_vm_tidx, tr_w_vm_sig))
        _, start_xm_tidxs = tr_manager.place_wires(xm_layer, ['clk', 'clk'], align_idx=0,
                                                   align_track=self.grid.coord_to_track(xm_layer, clk_stop_vm.middle,
                                                                                        mode=RoundMode.NEAREST))
        clk_start_xm = self.connect_to_tracks(clk_start_vm, TrackID(xm_layer, start_xm_tidxs[0], tr_w_xm_clk),
                                              min_len_mode=MinLenMode.MIDDLE)
        clk_stop_xm = self.connect_to_tracks(clk_stop_vm, TrackID(xm_layer, start_xm_tidxs[1], tr_w_xm_clk),
                                             min_len_mode=MinLenMode.MIDDLE)

        sar_clk_xm_tidx = tr_manager.get_next_track(xm_layer, start_xm_tidxs[0], 'sig', 'clk', up=False)
        sar_clk_xm = self.connect_to_tracks(clk_gen.get_pin('logic_clk'),
                                            TrackID(xm_layer, sar_clk_xm_tidx, tr_w_xm_clk),
                                            min_len_mode=MinLenMode.MIDDLE)
        self.connect_to_track_wires(logic_array.get_pin('sar_clk'), sar_clk_xm)

        # Connect rst
        # self.connect_to_track_wires(logic_array.get_pin('rst'), clk_start_xm)
        rst_comp_tidx = tr_manager.get_next_track(ym_layer, logic_array.get_pin('rst').track_id.base_index,
                                                  'clk', 'clk')
        rst_comp = self.connect_to_tracks(clk_start_xm, TrackID(ym_layer, rst_comp_tidx,
                                                                tr_manager.get_width(ym_layer, 'clk')),
                                          track_upper=self.bound_box.yh)

        # Connect state<0> top stop
        if logic_array_master.lower_layer_routing:
            rt_tidx_start = self.grid.coord_to_track(vm_layer, self.bound_box.xl)
            rt_tidx_stop = self.grid.coord_to_track(vm_layer, self.bound_box.xh)
            rt_tidx_list = self.get_available_tracks(vm_layer, rt_tidx_start, rt_tidx_stop, self.bound_box.yl,
                                                     self.bound_box.yh, width=tr_w_vm_sig, sep=tr_sp_vm_sig)
            rt_tidx_coord_list = [self.grid.track_to_coord(vm_layer, x) for x in rt_tidx_list]
            stop_vm_tidx = SARLogicArray.get_nearest_tidx(clk_stop_xm, rt_tidx_list, rt_tidx_coord_list)
            self.connect_to_tracks([clk_stop_xm, logic_array.get_pin('state<0>')],
                                   TrackID(vm_layer, stop_vm_tidx, tr_w_vm_sig))
        else:
            stop_ym_tidx = tr_manager.get_next_track(ym_layer, logic_array.get_pin('rst').track_id.base_index,
                                                     'clk', 'sig', up=False)
            self.connect_to_tracks([clk_stop_xm, logic_array.get_pin('state<0>')],
                                   TrackID(ym_layer, stop_ym_tidx, tr_w_ym_sig))

        # self.reexport(clk_gen.get_port('clk_outb'), net_name='comp_clkb')
        clk_out_bbox = clk_gen.get_pin('clk_out').bound_box
        clk_out = self.via_stack_up(tr_manager, clk_gen.get_pin('clk_out'), vm_layer, xm_layer, 'clk',
                                    bbox=clk_out_bbox)

        top_tile_info, top_yb, _ = self.get_tile_info(self.num_tile_rows - 1)

        # FIXME: fix clock short with dig signals
        tr_w_ym_clk = tr_manager.get_width(ym_layer, 'clk')
        clk_out_ym_locs = self.get_available_tracks(ym_layer,
                                                    self.grid.coord_to_track(ym_layer, self.bound_box.xl,
                                                                             RoundMode.GREATER),
                                                    self.grid.coord_to_track(ym_layer, self.bound_box.xh,
                                                                             RoundMode.LESS),
                                                    top_yb - top_tile_info.height, top_yb,
                                                    width=tr_w_ym_clk, sep=tr_manager.get_sep(ym_layer, ('clk', 'dum')))
        clk_out_ym_tid = min(clk_out_ym_locs,
                             key=lambda x: abs(self.grid.track_to_coord(ym_layer, x) - clk_out_bbox.xl))
        clk_out_ym = self.connect_to_tracks(clk_out[xm_layer], TrackID(ym_layer, clk_out_ym_tid, tr_w_ym_clk))
        clk_out = self.via_stack_up(tr_manager, clk_out_ym, ym_layer, xm1_layer, 'clk',
                                    bbox=clk_out_ym.bound_box)
        self.add_pin('comp_clk', clk_out[xm1_layer])
        self.reexport(clk_gen.get_port('clk_out'), net_name='comp_clk')
        # self.reexport(logic_array.get_port('sar_clk'), net_name='comp_clk', connect=True)

        self.reexport(clk_gen.get_port('comp_p'), net_name='comp_p_m')
        self.reexport(clk_gen.get_port('comp_n'), net_name='comp_n_m')
        # self.reexport(clk_gen.get_port('done'))
        # self.reexport(clk_gen.get_port('start'))
        # self.reexport(clk_gen.get_port('stop'))

        nbits = logic_array_master.sch_params['nbits']
        lower_layer_routing = logic_array_master.lower_layer_routing
        rt_layer = vm_layer
        rt_tidx_start = self.arr_info.col_to_track(rt_layer, 0)
        rt_tidx_stop = self.arr_info.col_to_track(rt_layer, self.num_cols)
        tr_w_rt_sig = tr_manager.get_width(rt_layer, 'sig')
        tr_sp_rt_sig = tr_manager.get_sep(rt_layer, ('sig', 'sig'))

        # # Fill tap
        for tidx in range(self.num_tile_rows):
            self.fill_tap(tidx)

        if lower_layer_routing:
            middle_coord = self.arr_info.col_to_coord(self.num_cols // 2)
            rt_tidx_list = self.get_available_tracks(rt_layer, rt_tidx_start, rt_tidx_stop, self.bound_box.yl,
                                                     self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
            rt_tidx_coord_list = [self.grid.track_to_coord(rt_layer, x) for x in rt_tidx_list]
            compn_rt_tidx = SARLogicArray.get_nearest_tidx(logic_array.get_pin('comp_p'), rt_tidx_list,
                                                           rt_tidx_coord_list, middle_coord)
            compp_rt_tidx = SARLogicArray.get_nearest_tidx(logic_array.get_pin('comp_p'), rt_tidx_list,
                                                           rt_tidx_coord_list, middle_coord)
            comp_p_ym = self.connect_to_tracks(logic_array.get_all_port_pins('comp_p'),
                                               TrackID(rt_layer, compp_rt_tidx, tr_w_rt_sig),
                                               track_upper=self.bound_box.yh)
            comp_n_ym = self.connect_to_tracks(logic_array.get_all_port_pins('comp_n'),
                                               TrackID(rt_layer, compn_rt_tidx, tr_w_rt_sig),
                                               track_upper=self.bound_box.yh)
            self.add_pin('comp_p', comp_p_ym)
            self.add_pin('comp_n', comp_n_ym)
            for pname in ['rst', 'clk_out']:
                self.reexport(logic_array.get_port(pname))
            for pname in ['state']:
                [self.reexport(logic_array.get_port(f'{pname}<{idx}>')) for idx in range(nbits)]
            for pname in ['dn', 'dp', 'dm']:
                for idx in range(nbits):
                    pin = logic_array.get_pin(f'{pname}<{idx}>')
                    rt_tidx_list = self.get_available_tracks(rt_layer, rt_tidx_start, rt_tidx_stop, pin.bound_box.yh,
                                                             self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                    rt_tidx_coord_list = [self.grid.track_to_coord(rt_layer, x) for x in rt_tidx_list]
                    _y_wire = self.connect_to_tracks(pin, TrackID(rt_layer,
                                                                  SARLogicArray.get_nearest_tidx(pin, rt_tidx_list,
                                                                                                 rt_tidx_coord_list),
                                                                  tr_w_rt_sig), track_upper=self.bound_box.yh)
                    self.add_pin(f'{pname}<{idx}>', _y_wire, mode=PinMode.UPPER)
            if logic_array_master.sch_params['has_pmos_sw']:
                for pname in ['dn_b', 'dp_b']:
                    for idx in range(nbits):
                        pin = logic_array.get_pin(f'{pname}<{idx}>')
                        rt_tidx_list = self.get_available_tracks(rt_layer, rt_tidx_start, rt_tidx_stop,
                                                                 pin.bound_box.yh,
                                                                 self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                        rt_tidx_coord_list = [self.grid.track_to_coord(rt_layer, x) for x in rt_tidx_list]
                        _y_wire = self.connect_to_tracks(pin, TrackID(rt_layer,
                                                                      SARLogicArray.get_nearest_tidx(pin, rt_tidx_list,
                                                                                                     rt_tidx_coord_list),
                                                                      tr_w_rt_sig), track_upper=self.bound_box.yh)
                        self.add_pin(f'{pname}<{idx}>', _y_wire, mode=PinMode.UPPER)

        else:
            for pname in ['rst', 'comp_p', 'comp_n', 'clk_out']:
                self.reexport(logic_array.get_port(pname))
            for pname in ['dn', 'dp', 'dm', 'state']:
                [self.reexport(logic_array.get_port(f'{pname}<{idx}>')) for idx in range(nbits)]
            if logic_array_master.sch_params['has_pmos_sw']:
                for pname in ['dn_b', 'dp_b']:
                    [self.reexport(logic_array.get_port(f'{pname}<{idx}>')) for idx in range(nbits)]

        _, last_sup = export_xm_sup(self, self.num_tile_rows - 1, export_top=True)
        self.add_pin('VDD_xm' if self.num_tile_rows & 1 else 'VSS_xm', last_sup,
                     label='VDD' if self.num_tile_rows & 1 else 'VSS', connect=True)
        _, last_sup = export_xm_sup(self, self.num_tile_rows - 2, export_top=True)
        self.add_pin('VSS_xm' if self.num_tile_rows & 1 else 'VDD_xm', last_sup,
                     label='VSS' if self.num_tile_rows & 1 else 'VDD', connect=True)

        # vss_xm_list.append(_sup)
        self.reexport(logic_array.get_port('VDD_xm'))
        self.reexport(logic_array.get_port('VSS_xm'))

        [self.reexport(logic_array.get_port(f'data_out<{idx}>')) for idx in range(nbits)]
        self.reexport(logic_array.get_port('VDD_hm'), label='VDD')
        self.reexport(logic_array.get_port('VSS_hm'), label='VSS')
        self.add_pin('VDD_hm', clk_gen.get_all_port_pins('VDD'), label='VDD')
        self.add_pin('VSS_hm', clk_gen.get_all_port_pins('VSS'), label='VSS')

        self.add_pin('rst_comp_clk', rst_comp)
        self.sch_params = dict(
            nbits=logic_array_master.sch_params['nbits'],
            sar_logic=logic_array_master.sch_params,
            clkgen=clk_gen_master.sch_params,
            lower_layer_routing=lower_layer_routing,
        )


class SARLogicUnitRev(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_unit_rev1')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_dict='Width dict',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            substrate_row='True to add substrate row',
            split_dac='True to use split dac, only two logic cells',
            msb='True to add initial flops to set bit',
            logicb='True to take low logic -- comparator output toggles btw 1->0/1,'
                   'False to take high logic -- comparator output toggles btw 0->0/1',
            logic_rt_layer='logic output layer, 4 or 5',
            has_buf='True to add output buffer to the logic cell, dyn latch drives by default',
            np_coupled='True to have np coupled',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            substrate_row=False,
            has_pmos_sw=False,
            msb=False,
            logicb=False,
            split_dac=False,
            logic_rt_layer=5,
            has_buf=False,
            np_coupled=False,
        )

    def fill_tap(self, tile_idx, port_mode=SubPortMode.EVEN) -> None:
        """
        This method fill empty region with sub contact
        """
        min_fill_ncols = self.tech_cls.min_sub_col + 2 * self.tech_cls.min_sep_col
        min_fill_ncols += min_fill_ncols & 1
        _, _, flip_tile = self.used_array.get_tile_info(tile_idx)
        intv_list = self.used_array.get_complement(tile_idx, 0, 0, self.num_cols)
        min_sep_col = self.min_sep_col
        min_sep_col += min_sep_col & 1

        def get_diff_port(pmode):
            return SubPortMode.EVEN if pmode == SubPortMode.ODD else SubPortMode.ODD

        for intv in intv_list:
            intv_pair = intv[0]
            nspace = intv_pair[1] - intv_pair[0]
            if nspace < min_fill_ncols:
                continue
            else:
                _port_mode = get_diff_port(port_mode) if (intv_pair[0] + self.min_sep_col) & 1 else port_mode
                tap0 = self.add_substrate_contact(0, intv_pair[0] + min_sep_col, seg=nspace - 2 * min_sep_col,
                                                  tile_idx=tile_idx, port_mode=_port_mode)
            tid0 = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=tile_idx)
            self.connect_to_tracks(tap0, tid0)
            intv_list = self.used_array.get_complement(tile_idx, 1, 0, self.num_cols)
        for intv in intv_list:
            intv_pair = intv[0]
            nspace = intv_pair[1] - intv_pair[0]
            if nspace < min_fill_ncols:
                continue
            else:
                _port_mode = get_diff_port(port_mode) if (intv_pair[0] + self.min_sep_col) & 1 else port_mode
                tap1 = self.add_substrate_contact(1, intv_pair[0] + min_sep_col,
                                                  seg=nspace - 2 * min_sep_col,
                                                  tile_idx=tile_idx, port_mode=_port_mode)
            tid1 = self.get_track_id(1, MOSWireType.DS, 'sup', tile_idx=tile_idx)
            self.connect_to_tracks(tap1, tid1)

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        logicb: bool = self.params['logicb']
        split_dac: bool = self.params['split_dac']
        substrate_row: bool = self.params['substrate_row']
        logic_rt_layer: bool = self.params['logic_rt_layer']
        has_buf: bool = self.params['has_buf']
        np_coupled: bool = self.params['np_coupled']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_lat: int = seg_dict['latch']
        seg_flop: Dict[str, int] = seg_dict['flop']
        seg_buf: ImmutableList = seg_dict['buf']
        seg_rst = seg_dict['rst_buf']
        seg_dyn_latch = seg_dict['dyn_latch'].to_dict()
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        # Check buffer length

        # Get some hm tidx
        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        # nd0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        # nd1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        # pd0_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        # pd1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)

        # Make params
        w_dyn_latch = self.params['w_dict']['dyn_latch'].to_dict()
        dyn_lat_params = dict(pinfo=pinfo, w_dict=w_dyn_latch, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=substrate_row, sig_locs={}, seg_dict=seg_dyn_latch, nand=logicb,
                              np_coupled=np_coupled)
        latch_params = dict(pinfo=pinfo, seg=seg_lat, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p, resetable=False,
                            vertical_sup=substrate_row, sig_locs={'nclkb': ng1_tidx, 'nclk': ng0_tidx, 'nin': ng2_tidx})
        flop_params = dict(pinfo=pinfo, seg=seg_flop, seg_ck=2, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=substrate_row, sig_locs={'nclkb': ng0_tidx, 'nclk': ng1_tidx, 'nin': ng2_tidx},
                           resetable=True, passgate=True, resetb=logicb)

        if logicb and not (len(seg_buf) & 1):
            raise ValueError("need odd number of buffer for logicb configuration")
        elif not logicb and len(seg_buf) & 1:
            seg_buf = seg_buf[:-1]

        buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=substrate_row, sig_locs={'nin0': pg0_tidx, 'nin1': ng1_tidx},
                          vertical_out=False)
        buf_np_params = buf_params

        inv_state_params = dict(pinfo=pinfo, seg=seg_lat, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                vertical_sup=substrate_row, sig_locs={'nin': ng1_tidx if logicb else ng0_tidx})
        rst_buf_params = dict(pinfo=pinfo, seg=seg_rst, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=substrate_row, sig_locs={'nin': ng1_tidx})

        # Make templates
        inv_state_master = self.new_template(InvCore, params=inv_state_params)
        inv_rst_master = self.new_template(InvCore, params=rst_buf_params)
        latch_master = self.new_template(RstLatchCore, params=latch_params)
        flop_master = self.new_template(FlopCore, params=flop_params)

        buf_master = self.new_template(InvChainCore, params=buf_params)
        buf_np_master = self.new_template(InvChainCore, params=buf_np_params)

        dyn_latch_master = self.new_template(DynLatchCore, params=dyn_lat_params)
        # Make a different dm latch for logicb
        if logicb:
            dyn_lat_params['nand'] = False
            dm_dyn_latch_master = self.new_template(DynLatchCore, params=dyn_lat_params)
        else:
            dm_dyn_latch_master = dyn_latch_master

        ncols_tot_logic = 6 * min_sep + 3 * dyn_latch_master.num_cols + buf_master.num_cols + 2 * buf_np_master.num_cols
        if not has_buf and split_dac:
            ncols_tot_logic -= dyn_latch_master.num_cols + buf_master.num_cols + 2 * buf_np_master.num_cols
        elif split_dac:
            ncols_tot_logic -= dyn_latch_master.num_cols + buf_master.num_cols
        elif not has_buf:
            ncols_tot_logic -= buf_master.num_cols + 2 * buf_np_master.num_cols

        ncols_tot = seg_lat + seg_rst + \
                    max(latch_master.num_cols + flop_master.num_cols + 2 * min_sep, ncols_tot_logic)
        # Row 0 - bit and retimer
        cur_col = 0
        flop_state = self.add_tile(flop_master, 0, cur_col)
        cur_col = ncols_tot - seg_lat - seg_rst
        inv_state = self.add_tile(inv_state_master, 0, cur_col)
        cur_col -= min_sep
        cur_col -= cur_col & 1
        latch = self.add_tile(latch_master, 0, cur_col, flip_lr=True)

        # Row 1,2 - dn/p signal
        cur_col = ncols_tot - (
                6 * min_sep + 3 * dyn_latch_master.num_cols + buf_master.num_cols + 2 * buf_np_master.num_cols)
        if not has_buf:
            cur_col += buf_master.num_cols + 2 * buf_np_master.num_cols + 3 *min_sep
        cur_col -= seg_lat + seg_rst
        dyn_lat_list, buf_list = [], []
        dyn_latch_master_list = [dm_dyn_latch_master, dyn_latch_master, dyn_latch_master]
        buf_master_list = [buf_master, buf_np_master, buf_np_master]
        if split_dac:
            dyn_latch_master_list.pop(0)
            buf_master_list.pop(0)
            cur_col += dyn_latch_master.num_cols + min_sep
            cur_col += cur_col & 1
            if has_buf:
                cur_col += buf_master.num_cols + min_sep
                cur_col += cur_col & 1
        for lat, buf in zip(dyn_latch_master_list, buf_master_list):
            dyn_lat_list.append(self.add_tile(lat, 1, cur_col))
            cur_col += dyn_latch_master.num_cols + min_sep
            cur_col += cur_col & 1
            if has_buf:
                buf_list.append(self.add_tile(buf, 1, cur_col))
                cur_col += buf.num_cols + min_sep
                cur_col += cur_col & 1
        cur_col = ncols_tot - inv_rst_master.num_cols
        inv_rst = self.add_tile(inv_rst_master, 1, cur_col)
        self.set_mos_size()

        if has_buf:
            for lat, buf in zip(dyn_lat_list, buf_list):
                self.connect_to_track_wires(lat.get_pin('o'), buf.get_pin('nin'))

            bufm, bufn, bufp = [None] + buf_list if split_dac else buf_list
        else:
            bufm, bufn, bufp = [None] * 3
        latm, latn, latp = [None] + dyn_lat_list if split_dac else dyn_lat_list
        if logicb and not split_dac:
            dyn_lat_list.pop(0)

        sig_list = ['clk', 'clk', 'clk', 'sig']
        _, xm_locs = tr_manager.place_wires(xm_layer, sig_list, center_coord=self.get_tile_info(0)[0].height // 2)
        # Connecct reset/bit and clk
        # -- get all vm
        bit_vm_tidx = self.grid.coord_to_track(vm_layer, inv_state.bound_box.xl, RoundMode.NEAREST)
        rst_vm_tidx = self.grid.coord_to_track(vm_layer, inv_rst.bound_box.xl, RoundMode.NEAREST)
        dp_vm_tidx = self.grid.coord_to_track(vm_layer, latch.bound_box.xh, RoundMode.NEAREST)

        bit_vm = self.connect_to_tracks(inv_state.get_pin('nin'), TrackID(vm_layer, bit_vm_tidx, tr_w_vm))
        rst_vm = self.connect_to_tracks(inv_rst.get_pin('nin'), TrackID(vm_layer, rst_vm_tidx, tr_w_vm))

        if has_buf:
            dp_vm = self.connect_to_tracks([latch.get_pin('nin'), bufp.get_pin('noutb' if logicb else 'nout'),
                                            bufp.get_pin('poutb' if logicb else 'pout')],
                                           TrackID(vm_layer, dp_vm_tidx, tr_w_vm))
        else:
            dp_vm = latp.get_pin('o')
            self.connect_to_tracks([latch.get_pin('nin')]+latp.get_all_port_pins('ob_hm'),
                                   TrackID(vm_layer, dp_vm_tidx, tr_w_vm))
        # rstb to dyn_latches
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_w_xm_clk = tr_manager.get_width(xm_layer, 'clk')
        self.connect_to_track_wires(inv_rst.get_pin('out'),
                                    [inst.get_pin('rst' if logicb else 'rstb') for inst in dyn_lat_list])
        rst_xm, rstb_xm = self.connect_matching_tracks([flop_state.get_pin('rst'), flop_state.get_pin('rstb')],
                                                       xm_layer, xm_locs[:2], width=tr_w_xm_clk)
        if logicb:
            rst_xm, rstb_xm = rstb_xm, rst_xm
        self.connect_to_track_wires(rst_xm, rst_vm)
        self.connect_to_track_wires(rstb_xm, inv_rst.get_pin('out'))
        self.connect_wires([latch.get_pin('nclkb' if logicb else 'nclk'), inv_state.get_pin('nin')])
        self.connect_to_track_wires(latch.get_pin('pclk' if logicb else 'pclkb'), inv_state.get_pin('out'))

        self.connect_to_track_wires(bit_vm, [inst.get_pin('en') for inst in dyn_lat_list])
        # bit_xm = self.connect_to_tracks([flop_state.get_pin('out'), bit_vm],
        #                                 TrackID(xm_layer, xm_locs[2], tr_w_xm))
        # clk and compn
        # dyn_clk_vm_tidx_l = self.grid.coord_to_track(vm_layer, latm.get_pin('d').lower, RoundMode.LESS)
        # dyn_clk_vm_tidx_h = self.grid.coord_to_track(vm_layer, latm.get_pin('d').upper, RoundMode.GREATER)
        # clk_vm = self.connect_to_tracks([flop_state.get_pin('nclk'), latm.get_pin('d')],
        #                                 TrackID(vm_layer, dyn_clk_vm_tidx_aval[len(dyn_clk_vm_tidx_aval) // 2],
        #                                         tr_w_vm))
        if not split_dac:
            self.connect_to_track_wires(flop_state.get_pin('clk'), latm.get_pin('d'))
            xm_sup_mid = self.grid.coord_to_track(xm_layer, latm.bound_box.yl, RoundMode.NEAREST)
            if logicb:
                latm_rst_vm_tidx = tr_manager.get_next_track(vm_layer, flop_state.get_pin('clk').track_id.base_index,
                                                             'clk', 'clk')
                latm_rst = self.connect_to_tracks(latm.get_pin('rstb'), TrackID(vm_layer, latm_rst_vm_tidx, tr_w_vm))
                self.connect_to_track_wires(rst_xm, latm_rst)
                # latm_en_vm_tidx = self.grid.coord_to_track(vm_layer, latm.get_pin('en').upper, RoundMode.NEAREST)
                latm_en_vm_tidx = tr_manager.get_next_track(vm_layer, latm_rst_vm_tidx, 'clk', 'sig')
                latm_en_vm = self.connect_to_tracks(latm.get_pin('en'), TrackID(vm_layer, latm_en_vm_tidx, tr_w_vm))
                # Connect enable to bitb
                en_xm_tidx = tr_manager.get_next_track(xm_layer, xm_sup_mid, 'sup', 'sig', up=1)
                self.connect_to_tracks([latm_en_vm, inv_state.get_pin('out')], TrackID(xm_layer, en_xm_tidx, tr_w_xm))
                en_xm_tidx = tr_manager.get_next_track(xm_layer, en_xm_tidx, 'sig', 'sig', up=1)
            else:
                en_xm_tidx = tr_manager.get_next_track(xm_layer, xm_sup_mid, 'sup', 'sig', up=1)
        else:
            en_xm_tidx = None

        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        clk_xm = self.connect_to_tracks(flop_state.get_pin('clk'), TrackID(xm_layer, xm_locs[2], tr_w_clk_xm))

        compn_tidx = self.grid.coord_to_track(vm_layer, latn.get_pin('d').middle, RoundMode.NEAREST)
        compn_tidx = tr_manager.get_next_track(vm_layer, compn_tidx, 'sig', 'sig', up = -2)
        compp_tidx = self.grid.coord_to_track(vm_layer, latp.get_pin('d').middle, RoundMode.NEAREST)
        compp_tidx = tr_manager.get_next_track(vm_layer, compp_tidx, 'sig', 'sig', up = -2)

        compn_vm = self.connect_to_tracks(latn.get_pin('d'), TrackID(vm_layer, compn_tidx, tr_w_vm),
                                          min_len_mode=MinLenMode.MIDDLE)
        compp_vm = self.connect_to_tracks(latp.get_pin('d'), TrackID(vm_layer, compp_tidx, tr_w_vm),
                                          min_len_mode=MinLenMode.MIDDLE)

        if has_buf:
            dn_tidx = self.grid.coord_to_track(vm_layer, bufn.get_pin('noutb' if logicb else 'nout').middle,
                                               RoundMode.NEAREST)
            dn_vm = self.connect_to_tracks([bufn.get_pin('noutb' if logicb else 'nout'),
                                            bufn.get_pin('poutb' if logicb else 'pout')],
                                           TrackID(vm_layer, dn_tidx, tr_w_vm),
                                           min_len_mode=MinLenMode.MIDDLE)
        else:
            dn_vm = latn.get_pin('o')
        if not split_dac:

            if has_buf:
                dm_tidx = self.grid.coord_to_track(vm_layer, bufm.get_pin('noutb' if logicb else 'nout').middle,
                                                   RoundMode.NEAREST)
                dm_vm = self.connect_to_tracks([bufm.get_pin('noutb' if logicb else 'nout'),
                                                bufm.get_pin('poutb' if logicb else 'pout')],
                                               TrackID(vm_layer, dm_tidx, tr_w_vm),
                                               min_len_mode=MinLenMode.MIDDLE)
            else:
                dm_vm = latm.get_pin('o')
        else:
            dm_vm = None
        bit_prev_vm_tidx = self.grid.coord_to_track(vm_layer, flop_state.bound_box.xl, RoundMode.NEAREST)
        bit_prev_vm = self.connect_to_tracks(flop_state.get_pin('in'), TrackID(vm_layer, bit_prev_vm_tidx, tr_w_vm),
                                             min_len_mode=MinLenMode.MIDDLE)
        ret_xm = self.connect_to_tracks(latch.get_pin('out'), TrackID(xm_layer, xm_locs[3], tr_w_xm))

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [inv_state, inv_rst, latch, flop_state] + dyn_lat_list
        if has_buf:
            inst_list += buf_list
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]
        # vss_hm = vss_hm[1:]

        ym_layer = xm_layer + 1
        if logic_rt_layer < ym_layer:
            comp_p = self.connect_to_tracks(compp_vm, TrackID(xm_layer, en_xm_tidx, tr_w_xm))
            en_xm_tidx = tr_manager.get_next_track(xm_layer, en_xm_tidx, 'sig', 'sig', up=1)
            comp_n = self.connect_to_tracks(compn_vm, TrackID(xm_layer, en_xm_tidx, tr_w_xm))
            self.add_pin('comp_n', comp_n)
            self.add_pin('comp_p', comp_p)
        else:
            self.add_pin('comp_n', compn_vm)
            self.add_pin('comp_p', compp_vm)

        if logic_rt_layer < ym_layer:
            self.add_pin('dp', dp_vm)
            self.add_pin('dn', dn_vm)
            if not split_dac:
                self.add_pin('dm', dm_vm)
        else:
            d_xm = []
            if split_dac:
                for d_vm in [dp_vm, dn_vm]:
                    xm_tidx = self.grid.coord_to_track(xm_layer, d_vm.upper, RoundMode.LESS_EQ)
                    d_xm.append(self.connect_to_tracks(d_vm, TrackID(xm_layer, xm_tidx, tr_w_xm),
                                                       min_len_mode=MinLenMode.MIDDLE))
                self.add_pin('dp', d_xm[0])
                self.add_pin('dn', d_xm[1])
            else:
                for d_vm in [dp_vm, dn_vm, dm_vm]:
                    xm_tidx = self.grid.coord_to_track(xm_layer, d_vm.upper, RoundMode.LESS_EQ)
                    d_xm.append(self.connect_to_tracks(d_vm, TrackID(xm_layer, xm_tidx, tr_w_xm),
                                                       min_len_mode=MinLenMode.MIDDLE))
                self.add_pin('dp', d_xm[0])
                self.add_pin('dn', d_xm[1])
                self.add_pin('dm', d_xm[2])

        if np_coupled:
            _, xm_tidx = tr_manager.place_wires(xm_layer, ['dig']*2, center_coord=latn.get_pin('o').lower)
            self.connect_matching_tracks([[latn.get_pin('ob_fb'), latp.get_pin('ob')],
                                          [latp.get_pin('ob_fb'), latn.get_pin('ob')]], xm_layer, xm_tidx)

        self.add_pin('clk', clk_xm)
        self.add_pin('rstb' if logicb else 'rst', rst_xm)
        self.add_pin('bit_prev', bit_prev_vm)
        self.add_pin('bit', [bit_vm, flop_state.get_pin('out')], connect=True)
        self.add_pin('out_ret', ret_xm)

        # --- Fill tap --
        fill_tap(self, 1, SubPortMode.EVEN)
        fill_tap(self, 0, SubPortMode.EVEN)

        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)
        #
        sch_params_dict = dict(
            inv_state=inv_state_master.sch_params,
            inv_rst=inv_state_master.sch_params,
            latch=latch_master.sch_params,
            buf=buf_master.sch_params,
            buf_np=buf_np_master.sch_params,
            flop=flop_master.sch_params,
            dyn_lat=dyn_latch_master.sch_params,
            dyn_latm=dm_dyn_latch_master.sch_params,
            logicb=logicb,
            split_dac=split_dac,
            has_buf=has_buf,
            np_coupled=np_coupled,
        )
        self.sch_params = sch_params_dict


class SARLogicArrayRev(SARLogicUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._lower_layer_routing = False

    @property
    def lower_layer_routing(self):
        return self._lower_layer_routing

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_array_rev1')

    @classmethod
    def get_nearest_tidx(cls, wire: WireArray, aval_tidx_list: List[HalfInt], aval_tidx_coord_list: List[float],
                         coord: Union[int, None] = None):
        wire_coord = wire.middle if coord is None else coord
        nearest_track_coord = min(aval_tidx_coord_list, key=lambda x: abs(x - wire_coord))
        nearest_idx = aval_tidx_coord_list.index(nearest_track_coord)
        nearest_track = aval_tidx_list[nearest_idx]
        aval_tidx_list.pop(nearest_idx)
        aval_tidx_coord_list.pop(nearest_idx)
        return nearest_track

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_dict='Width dict',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            logic_unit_row_arr='Array of unit cells',
            ret_unit_row_arr='Array of retimer cells',
            logicb='True to take low input from comparator',
            logic_rt_layer='logic output layer, 4 or 5',
            split_dac='True to use split dac, only two logic cells',
            has_buf='',
            np_coupled='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            logic_unit_row_arr=[],
            ret_unit_row_arr=[],
            logicb=False,
            split_dac=False,
            has_buf=False,
            logic_rt_layer=5,
            np_coupled=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Dict[str, Any] = self.params['seg_dict']
        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        lower_layer_routing: bool = False
        logic_unit_row_arr: List[int] = self.params['logic_unit_row_arr']
        ret_unit_row_arr: List[int] = self.params['ret_unit_row_arr']
        logicb: bool = self.params['logicb']
        split_dac: bool = self.params['split_dac']
        has_buf: bool = self.params['has_buf']
        np_coupled: bool = self.params['np_coupled']
        logic_rt_layer: int = self.params['logic_rt_layer']
        self._lower_layer_routing = lower_layer_routing

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        seg_ret: Dict[str, int] = seg_dict['retimer']
        seg_buf_int: List[int] = seg_dict['buf_int']
        seg_buf_out: List[int] = seg_dict['buf_out']
        seg_logic = seg_dict['logic']
        seg_logic_size_idx = seg_dict['logic_unit_idx']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # Check logic unit arrangement
        num_bits = sum(ret_unit_row_arr)
        if not logic_unit_row_arr:
            logic_unit_row_arr = [num_bits]
        if not ret_unit_row_arr:
            ret_unit_row_arr = [num_bits]
        if num_bits != sum(logic_unit_row_arr):
            raise ValueError("Logic unit array arrangement doesn't match number of units")

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm_clk = tr_manager.get_width(vm_layer, 'clk')
        # tr_w_vm_sig = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm_sig = tr_manager.get_width(xm_layer, 'sig')
        tr_w_xm_clk = tr_manager.get_width(xm_layer, 'clk')
        tr_sp_xm_sig = tr_manager.get_sep(xm_layer, ('sig', 'sig'))
        # tr_sp_vm_clk = tr_manager.get_sep(vm_layer, ('clk', 'clk'))
        # tr_sp_vm_sig = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
        # tr_sp_vm_sig_clk = tr_manager.get_sep(vm_layer, ('sig', 'clk'))
        # tr_sp_ym_sig = tr_manager.get_sep(ym_layer, ('sig', 'sig'))
        tr_w_ym_clk = tr_manager.get_width(ym_layer, 'clk')
        rt_layer = vm_layer if lower_layer_routing else ym_layer
        tr_w_rt_clk = tr_manager.get_width(rt_layer, 'clk')
        tr_w_rt_sig = tr_manager.get_width(rt_layer, 'sig')
        tr_sp_rt_sig = tr_manager.get_sep(rt_layer, ('sig', 'sig'))

        ####################
        # Make templates
        ####################

        logic_unit_params_list = [ ]
        for idx in range(num_bits):
            logic_unit_params = dict(pinfo=pinfo, seg_dict=seg_logic[seg_logic_size_idx[idx]],
                                     w_dict=self.params['w_dict'][seg_logic_size_idx[idx]],
                                     substrate_row=False, has_pmos_sw=False, w_n=w_n, w_p=w_p, logicb=logicb,
                                     split_dac=split_dac, logic_rt_layer=logic_rt_layer, has_buf=has_buf,
                                     np_coupled=np_coupled)
            logic_unit_params_list.append(logic_unit_params)
        pg2_tidx = self.get_track_index(1, MOSWireType.G, 'sig', wire_idx=2)
        buf_int_params = dict(pinfo=pinfo, seg_list=seg_buf_int, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={}, dual_output=logicb, vertical_out=True)
        buf_out_params = dict(pinfo=pinfo, seg_list=seg_buf_out, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={}, dual_output=False, vertical_out=False)
        seg_lat, seg_flop = seg_logic[0]['latch'], seg_logic[0]['flop']
        inv_rst_params = dict(pinfo=pinfo, seg=seg_lat, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={'nin': pg2_tidx})
        flop_params = dict(pinfo=pinfo, seg=seg_flop, seg_ck=2, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=False, resetable=True, passgate=True, resetb=not logicb)

        logic_unit_master_list = []
        for idx in range(num_bits):
            logic_unit_master = self.new_template(SARLogicUnitRev, params=logic_unit_params_list[idx])
            logic_unit_master_list.append(logic_unit_master)
        buf_int_master = self.new_template(InvChainCore, params=buf_int_params)
        buf_out_master = self.new_template(InvChainCore, params=buf_out_params)

        inv_rst_master = self.new_template(InvCore, params=inv_rst_params)
        flop_master = self.new_template(FlopCore, params=flop_params)

        ####################
        # Add insts
        ####################
        origin_col = 0
        # Add retimers
        retimer_ncol, retimer_row_list = 0, []
        ret_params = dict(pinfo=pinfo, seg_dict=seg_ret, w_n=w_n, w_p=w_p, substrate_row=False)
        ret_master = self.new_template(SARRetUnit, params=ret_params)

        ret_start_ncol = max(buf_out_master.num_cols, buf_int_master.num_cols) + min_sep + origin_col
        for idx, row_num in enumerate(ret_unit_row_arr):
            ret_shift_params = dict(pinfo=pinfo, seg_dict=seg_ret, w_n=w_n, w_p=w_p, substrate_row=False,
                                    sig_locs={'out': idx, 'in': idx})
            ret_shift_master = self.new_template(SARRetUnit, params=ret_shift_params)
            _retimer_list = []
            if idx == 0:
                cur_col_ret = ret_start_ncol
            elif idx == 1:
                cur_col_ret = ret_start_ncol
            else:
                cur_col_ret = origin_col
            for jdx in range(row_num):
                _retimer_list.append(self.add_tile(ret_shift_master, idx, cur_col_ret))
                cur_col_ret += ret_shift_master.num_cols
            retimer_row_list.append(_retimer_list)
            retimer_ncol = max(cur_col_ret, retimer_ncol)

        # Add clock buffer
        logic_row = max(1, len(ret_unit_row_arr))
        buf_int = self.add_tile(buf_int_master, 1, origin_col)
        buf_out = self.add_tile(buf_out_master, 0, origin_col)
        inv_rst = self.add_tile(inv_rst_master, logic_row + 1, origin_col)
        flop_in = self.add_tile(flop_master, logic_row + 1, origin_col + inv_rst_master.num_cols + min_sep)
        cur_col = max(buf_int_master.num_cols + buf_out_master.num_cols,
                      flop_master.num_cols + inv_rst_master.num_cols + min_sep) + origin_col

        # Add logic cells
        logic_ncol, logic_row_list = 0, []
        num_logic_rows = logic_unit_master_list[0].num_tile_rows
        num_logic_cols = logic_unit_master_list[0].num_cols
        cur_col = max(cur_col, num_logic_cols + min_sep + origin_col)
        for idx, row_num in enumerate(logic_unit_row_arr):
            _logic_list = []
            cur_col, cur_row = origin_col if idx else cur_col, logic_row + idx * num_logic_rows
            cur_row += (cur_row & 1) * 2
            for jdx in range(row_num):
                if not idx & 1:
                    _logic_list.append(
                        self.add_tile(logic_unit_master_list[sum(logic_unit_row_arr[:idx])+jdx],
                                      cur_row, cur_col + num_logic_cols, flip_lr=True))
                else:
                    _logic_list.append(self.add_tile(logic_unit_master_list[sum(logic_unit_row_arr[:idx])+jdx],
                                                     cur_row, cur_col))
                cur_col += logic_unit_master_list[sum(logic_unit_row_arr[:idx])+jdx].num_cols + min_sep
            _logic_list = _logic_list[::-1] if idx & 1 else _logic_list
            logic_row_list.append(_logic_list)
            logic_ncol = max(cur_col, logic_ncol)
        logic_row_list = logic_row_list[::-1]
        self.set_mos_size()

        ####################
        # Connections
        ####################

        # Connect initia state lflop
        flop_in_vm_tidx = self.grid.coord_to_track(vm_layer, flop_in.get_pin('nin').upper, RoundMode.NEAREST)
        self.connect_to_tracks([flop_in.get_pin('VDD' if logicb else 'VSS'),
                                flop_in.get_pin('nin')], TrackID(vm_layer, flop_in_vm_tidx))

        # Connect clk signal together
        retimer_flatten_list = [item for sublist in retimer_row_list for item in sublist]
        logic_flatten_list = [item for sublist in logic_row_list for item in sublist]

        # ----------
        # Retimer connections
        # ----------
        # -- Clock connections --
        clk_int_vm_tidx = self.grid.coord_to_track(vm_layer, buf_int.get_pin('nout').middle, mode=RoundMode.NEAREST)
        clk_out_vm_tidx = tr_manager.get_next_track(vm_layer, clk_int_vm_tidx, 'clk', 'clk', up=False)
        # buf_int_out_vm = self.connect_to_tracks([buf_int.get_pin('nout'), buf_int.get_pin('pout')],
        #                                         TrackID(vm_layer, clk_int_vm_tidx, tr_w_vm_clk))
        buf_int_out_vm = buf_int.get_pin('out')
        buf_out_out_vm = self.connect_to_tracks([buf_out.get_pin('nout'), buf_out.get_pin('pout')],
                                                TrackID(vm_layer, clk_out_vm_tidx, tr_w_vm_clk))

        ret_clk_xm = []
        for ret_row in retimer_row_list:
            ret_clk_vm = [inst.get_pin('clk') for inst in ret_row]
            ret_clk_xm_tidx = self.grid.coord_to_track(xm_layer, ret_clk_vm[0].upper, mode=RoundMode.NEAREST)
            ret_clk_xm.append(self.connect_to_tracks(ret_clk_vm, TrackID(xm_layer, ret_clk_xm_tidx, tr_w_xm_clk,
                                                                         grid=self.grid),
                                                     min_len_mode=MinLenMode.MIDDLE))
        self.connect_to_track_wires(buf_int_out_vm, ret_clk_xm)
        self.connect_to_track_wires(buf_int_out_vm, buf_out.get_pin('nin'))

        ret_out_list = [inst.get_pin('out') for inst in retimer_flatten_list]
        ret_in_list = [inst.get_pin('in') for inst in retimer_flatten_list]
        ret_out_list.sort(key=lambda x: x.track_id.base_index)
        ret_in_list.sort(key=lambda x: x.track_id.base_index)

        ret_out_list = self.extend_wires(ret_out_list, lower=self.get_tile_info(0)[1])
        ret_in_list = self.extend_wires(ret_in_list,
                                        lower=self.get_tile_info(len(ret_unit_row_arr) - 1)[1],
                                        upper=self.get_tile_info(len(ret_unit_row_arr))[1])
        # ----------
        # Logic cell connections
        # ----------
        # Connection for logic cells
        # -- Rst signal (retimer clk) --
        logic_rst_list = []
        for inst_row in logic_row_list:
            rst_xm_list = [inst.get_pin('rstb' if logicb else 'rst') for inst in inst_row]
            logic_rst_list.extend(self.connect_wires(rst_xm_list))

        rst_buf_vm_tidx = self.grid.coord_to_track(vm_layer, buf_int.bound_box.xl, RoundMode.NEAREST)
        rst_buf_vm = self.connect_to_tracks(buf_int.get_pin('nin'), TrackID(vm_layer, rst_buf_vm_tidx, tr_w_vm_clk))
        rst_buf_xm_tidx = self.grid.coord_to_track(xm_layer, rst_buf_vm.middle, mode=RoundMode.NEAREST)
        rst_buf_xm = self.connect_to_tracks(rst_buf_vm, TrackID(xm_layer, rst_buf_xm_tidx, tr_w_xm_clk),
                                            min_len_mode=MinLenMode.MIDDLE)
        ret_clkb_xm_tidx = rst_buf_xm_tidx - self.get_track_sep(xm_layer, tr_w_xm_clk, tr_w_xm_clk)
        # rt layer connection
        rst_rt_tidx = self.grid.coord_to_track(rt_layer, rst_buf_xm.lower, mode=RoundMode.LESS_EQ)
        # clk_midb_rt_tidx = tr_manager.get_next_track(rt_layer, rst_rt_tidx, 'clk', 'clk')
        # clk_rt_tidx = tr_manager.get_next_track(rt_layer, clk_midb_rt_tidx, 'clk', 'clk')
        clk_ym_sep_tidx = self.get_track_sep(rt_layer, tr_w_rt_clk, tr_w_rt_clk)
        clk_midb_rt_tidx = rst_rt_tidx + clk_ym_sep_tidx
        clk_rt_tidx = clk_midb_rt_tidx + clk_ym_sep_tidx
        ym_tidx_start = clk_rt_tidx + clk_ym_sep_tidx
        # ym_tidx_start = tr_manager.get_next_track(rt_layer, clk_rt_tidx, 'clk', 'clk')

        if logicb:
            rst_rt = self.connect_to_tracks([rst_buf_xm], TrackID(rt_layer, rst_rt_tidx, tr_w_rt_clk,
                                                                  grid=self.grid))
            rstb_xm = self.connect_to_tracks(buf_int.get_pin('outb'), TrackID(xm_layer, ret_clkb_xm_tidx, tr_w_xm_clk))
            self.connect_to_tracks([rstb_xm] + logic_rst_list, TrackID(rt_layer, clk_midb_rt_tidx, tr_w_rt_clk))
        else:
            rst_rt = self.connect_to_tracks(logic_rst_list + [rst_buf_xm], TrackID(rt_layer, rst_rt_tidx, tr_w_rt_clk,
                                                                                   grid=self.grid))

        # -- clk signal --
        logic_clk_list = []
        for inst_row in logic_row_list:
            clk_xm_list = [inst.get_pin('clk') for inst in inst_row]
            logic_clk_list.extend(self.connect_wires(clk_xm_list))

        clk_rt = self.connect_to_tracks(logic_clk_list, TrackID(rt_layer, clk_rt_tidx, tr_w_rt_clk,
                                                                grid=self.grid))
        flop_clk_xm_tidx = self.grid.coord_to_track(xm_layer, flop_in.get_pin('clk').middle, RoundMode.NEAREST)
        flop_clk = self.connect_to_tracks(flop_in.get_pin('clk'), TrackID(xm_layer, flop_clk_xm_tidx, tr_w_xm_clk,
                                                                          grid=self.grid))
        self.connect_to_track_wires(flop_clk, clk_rt)
        # -- rst signal
        inv_rst_in_vm_tidx = tr_manager.get_next_track(vm_layer, inv_rst.get_pin('out').track_id.base_index, 'sig',
                                                       'clk', up=-1)
        inv_rst_in_vm = self.connect_to_tracks(inv_rst.get_pin('nin'), TrackID(vm_layer, inv_rst_in_vm_tidx,
                                                                               tr_w_vm_clk))

        _, flop_rst_locs = tr_manager.place_wires(xm_layer, ['clk'] * 3, align_idx=-1, align_track=flop_clk_xm_tidx)
        rst_xm, rstb_xm = self.connect_matching_tracks([[inv_rst_in_vm, flop_in.get_pin('rst')],
                                                        [inv_rst.get_pin('out'), flop_in.get_pin('rstb')]],
                                                       xm_layer, flop_rst_locs[:-1], width=tr_w_xm_clk)
        self.connect_to_track_wires(rst_xm, rst_rt)
        # -- Compn/Compp signal --
        logic_compn_list, logic_compp_list = [], []
        tr_w_xm_anasig = tr_manager.get_width(xm_layer, 'ana_sig')
        for inst_row in logic_row_list:
            # rst_vm_list = [inst.get_pin('rst') for inst in inst_row]
            compn_list = [inst.get_pin('comp_n') for inst in inst_row]
            compp_list = [inst.get_pin('comp_p') for inst in inst_row]
            if logic_rt_layer != ym_layer:
                logic_compn_list.extend(self.connect_wires(compn_list))
                logic_compp_list.extend(self.connect_wires(compp_list))
            else:
                rst_xm_tidx = self.grid.coord_to_track(xm_layer, compn_list[0].lower, mode=RoundMode.NEAREST)
                _, comp_xm_locs = tr_manager.place_wires(xm_layer, ['clk', 'ana_sig', 'ana_sig'], rst_xm_tidx, 0)
                logic_compn_list.append(
                    self.connect_to_tracks(compn_list, TrackID(xm_layer, comp_xm_locs[1], tr_w_xm_anasig),
                                           min_len_mode=MinLenMode.MIDDLE))
                logic_compp_list.append(
                    self.connect_to_tracks(compp_list, TrackID(xm_layer, comp_xm_locs[2], tr_w_xm_anasig),
                                           min_len_mode=MinLenMode.MIDDLE))

        # ---- Connect comp/n to ym ----
        # middle_coord = self.arr_info.col_to_coord(self.num_cols // 2)
        tr_w_rt_ana_sig = tr_manager.get_width(rt_layer, 'ana_sig')
        comp_ym_mid_tidx = self.grid.coord_to_track(ym_layer, self.bound_box.w // 2, RoundMode.NEAREST)
        ym_tr_sep = self.get_track_sep(ym_layer, tr_w_rt_ana_sig, 1)
        comp_ym_locs = [comp_ym_mid_tidx - ym_tr_sep, comp_ym_mid_tidx + ym_tr_sep]
        comp_p_ym = self.connect_to_tracks(logic_compp_list,
                                           TrackID(rt_layer, comp_ym_locs[0], tr_w_rt_ana_sig, grid=self.grid),
                                           track_upper=self.bound_box.yh)
        comp_n_ym = self.connect_to_tracks(logic_compn_list,
                                           TrackID(rt_layer, comp_ym_locs[1], tr_w_rt_ana_sig, grid=self.grid),
                                           track_upper=self.bound_box.yh)
        [comp_p_ym, comp_n_ym] = self.match_warr_length([comp_p_ym, comp_n_ym])

        tr_w_xrt_ana_sig = tr_manager.get_width(rt_layer + 1, 'ana_sig')
        xrt_layer = rt_layer + 1
        xrt_middle_tidx = self.grid.coord_to_track(xrt_layer, comp_p_ym.middle, RoundMode.NEAREST)
        xrt_ana_sig_sep = self.get_track_sep(xrt_layer, tr_w_xrt_ana_sig, 1)
        comp_n_xm1 = self.connect_to_tracks(comp_n_ym, TrackID(xrt_layer, xrt_middle_tidx - xrt_ana_sig_sep,
                                                               tr_w_xrt_ana_sig, grid=self.grid))
        comp_p_xm1 = self.connect_to_tracks(comp_p_ym, TrackID(xrt_layer, xrt_middle_tidx + xrt_ana_sig_sep,
                                                               tr_w_xrt_ana_sig, grid=self.grid))
        comp_p_xm1, comp_n_xm1 = self.match_warr_length([comp_p_xm1, comp_n_xm1])

        for idx in range(self.num_tile_rows):
            _sup, _ = export_xm_sup(self, idx, export_bot=True, xm_only=True)
        export_xm_sup(self, self.num_tile_rows - 1, export_bot=False, export_top=True, xm_only=True)
        # -- bit/bit_nxt --
        out_ret_list = []
        last_bit = None
        bit_nxt_list = []
        ym_tidx_stop = self.grid.coord_to_track(ym_layer, self.bound_box.xh, mode=RoundMode.LESS)
        bit_init = None
        for idx, inst_row in enumerate(logic_row_list):
            _tidx_start = self.grid.coord_to_track(xm_layer, inst_row[0].bound_box.yl, mode=RoundMode.GREATER_EQ)
            _tidx_stop = self.grid.coord_to_track(xm_layer, inst_row[0].bound_box.yh, mode=RoundMode.GREATER_EQ)
            xm_locs = self.get_available_tracks(xm_layer, _tidx_start, _tidx_stop, self.bound_box.xl,
                                                self.bound_box.xh, width=tr_w_xm_sig, sep=tr_sp_xm_sig)
            xm_locs = [None] + xm_locs
            last_xm = xm_locs[3]
            # xm_locs = xm_locs[1:]  # Avoid put on top of sup
            bit_sig_list = []
            bit_nxt_sig_list = []
            # inst_row = inst_row[::-1] if idx & 1 else inst_row
            for jdx, inst in enumerate(inst_row):
                _tidx, _tidx_nxt = (xm_locs[1], xm_locs[2]) if jdx & 1 else (xm_locs[2], xm_locs[1])
                if idx == len(logic_row_list) - 1 and jdx == len(inst_row) - 1:
                    _tidx = last_xm
                bit_sig_list.append(self.connect_to_tracks(inst.get_all_port_pins('bit_prev'),
                                                           TrackID(xm_layer, _tidx, tr_w_xm_sig)))
                bit_nxt_sig_list.append(self.connect_to_tracks(inst.get_all_port_pins('bit'),
                                                               TrackID(xm_layer, _tidx_nxt, tr_w_xm_sig)))
                if idx == len(logic_row_list) - 1 and jdx == len(inst_row) - 1:
                    bit_init = bit_sig_list[-1]
            bit_nxt_list.extend(bit_nxt_sig_list)
            for _bitnxt, _bit in zip(bit_nxt_sig_list[1:], bit_sig_list[:-1]):
                self.connect_wires([_bit, _bitnxt])

            tr_w_ym_dig = tr_manager.get_width(ym_layer, 'dig')
            for inst in inst_row:
                _ret_ym_tidx = self.grid.coord_to_track(ym_layer, inst.get_pin('out_ret').middle, RoundMode.NEAREST)
                _ret_ym_tidx = tr_manager.get_next_track(ym_layer, _ret_ym_tidx, 'sig', 'sig', up=idx)
                out_ret_list.append(self.connect_to_tracks(inst.get_pin('out_ret'),
                                                           TrackID(ym_layer, _ret_ym_tidx, tr_w_ym_dig)))

            if idx > 0:
                _coord = max(bit_nxt_sig_list[0].middle, last_bit.middle) if idx & 1 else \
                    min(bit_nxt_sig_list[0].middle, last_bit.middle)
                ym_tidx_list = self.get_available_tracks(ym_layer, ym_tidx_start, ym_tidx_stop, last_bit.bound_box.yh,
                                                         self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                _nearest_tidx = self.get_nearest_tidx(last_bit, ym_tidx_list, ym_tidx_coord_list, _coord)
                _y_wire = self.connect_to_tracks([last_bit, bit_nxt_sig_list[0]], TrackID(ym_layer, _nearest_tidx,
                                                                                          tr_w_rt_sig))
            last_bit = bit_sig_list[-1]

        self.connect_to_track_wires(bit_init, flop_in.get_pin('out'))

        # -- retimer input --
        ret_top_tileinfo, ret_top_y, _ = self.get_tile_info(len(ret_unit_row_arr))
        retimer_in_tidx_start = self.grid.coord_to_track(xm_layer, self.get_tile_info(0)[1],
                                                         mode=RoundMode.NEAREST)
        retimer_in_tidx_stop = self.grid.coord_to_track(xm_layer, self.get_tile_info(len(ret_unit_row_arr) + 1)[1],
                                                        mode=RoundMode.NEAREST)

        retimer_in_xm_locs = self.get_available_tracks(xm_layer, retimer_in_tidx_start, retimer_in_tidx_stop,
                                                       self.bound_box.xl, self.bound_box.xh,
                                                       width=tr_w_xm_sig, sep=tr_sp_xm_sig)
        retimer_in_xm_list = []
        for tidx, ret_in in zip(retimer_in_xm_locs[1:-1], ret_in_list):
            retimer_in_xm_list.append(self.connect_to_tracks(ret_in, TrackID(xm_layer, tidx, tr_w_xm_sig),
                                                             min_len_mode=MinLenMode.MIDDLE))
        stop_ym_tidx = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.NEAREST)
        for ret_in, logic_out in zip(retimer_in_xm_list, out_ret_list):
            self.connect_to_track_wires(ret_in, logic_out)

        # Connect dm/dn/dp dp_b/dn_b to xm_layer
        logic_tile_height = self.get_tile_pinfo(0).height
        if logic_rt_layer < ym_layer:
            tr_w_xm_sig = tr_manager.get_width(xm_layer, 'sig')
            tr_sp_xm_sig = tr_manager.get_sep(xm_layer, ('sig', 'sig'))
            for idx, inst in enumerate(logic_flatten_list):
                _tidx_start = self.grid.coord_to_track(xm_layer, inst.bound_box.yh - logic_tile_height,
                                                       mode=RoundMode.GREATER_EQ)
                _tidx_stop = self.grid.coord_to_track(xm_layer, inst.bound_box.yh, mode=RoundMode.LESS_EQ)
                xm_tidx_list = self.get_available_tracks(xm_layer, _tidx_start, _tidx_stop, self.bound_box.xl,
                                                         self.bound_box.xh, width=tr_w_xm_sig, sep=tr_sp_xm_sig)
                xm_tidx_coord_list = [self.grid.track_to_coord(xm_layer, x) for x in xm_tidx_list]
                _nearest_tidx = self.get_nearest_tidx(inst.get_pin('dm'), xm_tidx_list, xm_tidx_coord_list)
                _x_wire = self.connect_to_tracks(inst.get_pin('dm'), TrackID(xm_layer, _nearest_tidx, tr_w_rt_sig),
                                                 track_upper=self.bound_box.yh)
                if not split_dac:
                    self.add_pin(f'dm<{idx}>', _x_wire, mode=PinMode.UPPER)
                    _nearest_tidx = self.get_nearest_tidx(inst.get_pin('dn'), xm_tidx_list, xm_tidx_coord_list)
                    _x_wire = self.connect_to_tracks(inst.get_pin('dn'), TrackID(xm_layer, _nearest_tidx, tr_w_rt_sig),
                                                     track_upper=self.bound_box.yh)
                self.add_pin(f'dn<{idx}>', _x_wire, mode=PinMode.UPPER)
                _nearest_tidx = self.get_nearest_tidx(inst.get_pin('dp'), xm_tidx_list, xm_tidx_coord_list)
                _x_wire = self.connect_to_tracks(inst.get_pin('dp'), TrackID(xm_layer, _nearest_tidx, tr_w_rt_sig),
                                                 track_upper=self.bound_box.yh)
                self.add_pin(f'dp<{idx}>', _x_wire, mode=PinMode.UPPER)

        else:
            _tidx_start = self.grid.coord_to_track(ym_layer, self.bound_box.xl, mode=RoundMode.GREATER_EQ)
            _tidx_stop = self.grid.coord_to_track(ym_layer, self.bound_box.xh, mode=RoundMode.LESS)
            if not split_dac:
                dm_xm_list = [inst.get_pin('dm') for inst in logic_flatten_list]
                for idx, pin in enumerate(dm_xm_list):
                    ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                             self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                    ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                    _nearest_tidx = self.get_nearest_tidx(pin, ym_tidx_list, ym_tidx_coord_list)
                    _y_wire = self.connect_to_tracks(pin, TrackID(ym_layer, _nearest_tidx, tr_w_rt_sig),
                                                     track_upper=self.bound_box.yh)
                    self.add_pin(f'dm<{idx}>', _y_wire, mode=PinMode.UPPER)
            dn_xm_list = [inst.get_pin('dn') for inst in logic_flatten_list]
            dp_xm_list = [inst.get_pin('dp') for inst in logic_flatten_list]
            # Connect dm/dn/dp dp_b/dn_b to ym_layer
            for idx, pin in enumerate(dn_xm_list):
                ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                         self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                _nearest_tidx = self.get_nearest_tidx(pin, ym_tidx_list, ym_tidx_coord_list)
                _y_wire = self.connect_to_tracks(pin, TrackID(ym_layer, _nearest_tidx, tr_w_rt_sig),
                                                 track_upper=self.bound_box.yh)
                self.add_pin(f'dn<{idx}>', _y_wire, mode=PinMode.UPPER)
            for idx, pin in enumerate(dp_xm_list):
                ym_tidx_list = self.get_available_tracks(ym_layer, _tidx_start, _tidx_stop, pin.bound_box.yh,
                                                         self.bound_box.yh, width=tr_w_rt_sig, sep=tr_sp_rt_sig)
                ym_tidx_coord_list = [self.grid.track_to_coord(ym_layer, x) for x in ym_tidx_list]
                _nearest_tidx = self.get_nearest_tidx(pin, ym_tidx_list, ym_tidx_coord_list)
                _y_wire = self.connect_to_tracks(pin, TrackID(ym_layer, _nearest_tidx, tr_w_rt_sig),
                                                 track_upper=self.bound_box.yh)
                self.add_pin(f'dp<{idx}>', _y_wire, mode=PinMode.UPPER)

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [buf_int, buf_out, flop_in, inv_rst] + retimer_flatten_list + logic_flatten_list
        for inst in inst_list:
            vdd_list.append(inst.get_all_port_pins('VDD'))
            vss_list.append(inst.get_all_port_pins('VSS'))
        vdd_list = [vdd for sublist in vdd_list for vdd in sublist]
        vss_list = [vss for sublist in vss_list for vss in sublist]
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)

        # Bring the final state up
        last_state_l_tidx = self.grid.coord_to_track(ym_layer, bit_nxt_list[0].lower, RoundMode.NEAREST)
        last_state_h_tidx = self.grid.coord_to_track(ym_layer, bit_nxt_list[0].upper, RoundMode.NEAREST)
        tr_w_dig_ym = tr_manager.get_width(ym_layer, 'dig')
        tr_sp_dig_ym = tr_manager.get_sep(ym_layer, ('dig', 'dig'))

        last_bit_locs = self.get_available_tracks(ym_layer, last_state_l_tidx, last_state_h_tidx, self.bound_box.yl,
                                                  self.bound_box.yh, tr_w_dig_ym, tr_sp_dig_ym)
        last_bit_tidx = last_bit_locs[len(last_bit_locs) // 2]
        last_bit_ym = self.connect_to_tracks(bit_nxt_list[0], TrackID(ym_layer, last_bit_tidx, tr_w_dig_ym))

        for idx, pin in enumerate(bit_nxt_list[1:]):
            self.add_pin(f'state<{idx + 1}>', pin)
        self.add_pin(f'state<0>', last_bit_ym)
        self.add_pin('VDD_hm', vdd_hm, label='VDD', show=self.show_pins, connect=True)
        self.add_pin('VSS_hm', vss_hm, label='VSS', show=self.show_pins, connect=True)

        vdd_xm_list, vss_xm_list = [], []
        for idx in range(self.num_tile_rows):
            _sup, _ = export_xm_sup(self, idx, export_bot=True, xm_only=False)
            if idx & 1:
                vdd_xm_list.append(_sup)
            else:
                vss_xm_list.append(_sup)
        _, _sup = export_xm_sup(self, self.num_tile_rows - 1, export_bot=False, export_top=True, xm_only=False)
        if self.num_tile_rows & 1:
            vdd_xm_list.append(_sup)
        else:
            vss_xm_list.append(_sup)

        vdd_xm_list = self.extend_wires(vdd_xm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm_list = self.extend_wires(vss_xm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.add_pin('VDD_xm', vdd_xm_list, label='VDD', show=self.show_pins, connect=True)
        self.add_pin('VSS_xm', vss_xm_list, label='VSS', show=self.show_pins, connect=True)

        # # Fill tap
        for tidx in range(self.num_tile_rows):
            self.fill_tap(tidx)
        self.add_pin('rst', rst_rt)
        self.add_pin('sar_clk', clk_rt)
        self.add_pin('comp_p', comp_p_xm1)
        self.add_pin('comp_n', comp_n_xm1)
        self.add_pin('clk_out', buf_out_out_vm)
        [self.add_pin(f'data_out<{idx}>', pin) for idx, pin in enumerate(ret_out_list)]
        self.sch_params = dict(
            ret=ret_master.sch_params,
            logic=[master.sch_params for master in logic_unit_master_list],
            buf_clk=buf_int_master.sch_params,
            buf_out=buf_out_master.sch_params,
            flop_in=flop_master.sch_params,
            inv_rst=inv_rst_master.sch_params,
            nbits=num_bits,
            logicb=logicb,
            split_dac=split_dac,
        )
#
