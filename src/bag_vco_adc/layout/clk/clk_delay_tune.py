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
from bag.layout.routing import TrackID
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict, ImmutableList
from bag.util.math import HalfInt
from bag_vco_adc.layout.digital import NAND2Core, InvChainCore, InvCore
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.wrapper import MOSBaseTapWrapper
from pybag.core import Transform, BBox
from pybag.enum import RoundMode, PinMode, Orientation, Orient2D
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo


def export_vm_diff_wire(base: TemplateBase, vm_wire_n, vm_wire_p, xm_tidx_n, xm_tidx_p, wire_type, tr_manager, vm_layer,
                        export_ym=False):
    xm_layer = vm_layer + 1
    ym_layer = xm_layer + 1
    tr_w_xm = tr_manager.get_width(xm_layer, wire_type)
    tr_w_ym = tr_manager.get_width(ym_layer, wire_type)
    xm_wire_n, xm_wire_p = base.connect_differential_tracks(vm_wire_n, vm_wire_p, xm_layer,
                                                            xm_tidx_n, xm_tidx_p, width=tr_w_xm)
    if export_ym:
        _, ym_locs = tr_manager.place_wires(ym_layer, [wire_type, wire_type], center_coord=xm_wire_n.middle)
        ym_wire_n, ym_wire_p = base.connect_differential_tracks(xm_wire_n, xm_wire_p, ym_layer,
                                                                ym_locs[0], ym_locs[1], width=tr_w_ym)
    else:
        ym_wire_n, ym_wire_p = None, None

    return (xm_wire_n, xm_wire_p), (ym_wire_n, ym_wire_p)


class Binary2ThermalDecoder(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_bi2th_decoder')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            nbits='Number of bits',
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

        seg_buf = seg_dict['buf']
        seg_nand = seg_dict['nand']
        seg_inv = seg_dict['inv']
        nbits = self.params['nbits']

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
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        nd0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nd1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pd0_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pd1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)

        _, d_fb_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                              self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST),
                                              align_idx=0)

        buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          dual_output=True, sig_locs={'nin0': ng2_tidx, 'nin1': ng1_tidx})
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          sig_locs={'nin': pg1_tidx})
        nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p, )

        buf_master = self.new_template(InvChainCore, params=buf_params)
        inv_master = self.new_template(InvCore, params=inv_params)
        nand_master = self.new_template(NAND2Core, params=nand_params)

        def vm_sig_ntr_to_col(num_sig):
            vm_ntr, _ = tr_manager.place_wires(vm_layer, (num_sig + 1) * ['sig'])
            return self.arr_info.track_to_col(vm_layer, vm_ntr)

        cur_col = 0
        # place buffers for all input first
        # cur_col += vm_sig_ntr_to_col(nbits + 1)
        buf_list = []
        for idx in range(nbits):
            buf_list.append(self.add_tile(buf_master, col_idx=cur_col, tile_idx=idx))

        cur_col += min_sep + buf_master.num_cols
        cur_col += cur_col & 1
        sig_vm_locs_tidx_start = self.arr_info.col_to_track(vm_layer, cur_col)
        _, sig_vm_locs = tr_manager.place_wires(vm_layer, (nbits * 2 + 1) * ['sig'], align_track=sig_vm_locs_tidx_start)
        cur_col += vm_sig_ntr_to_col(nbits * 2)
        cur_col += cur_col & 1

        nand_list_list, inv_list_list = [], []
        mid_sig_vm_locs_list = []
        for idx in range(nbits - 1):
            _nand_list, _inv_list = [], []
            for jdx in range(2 ** (idx + 2)):
                _nand_list.append(self.add_tile(nand_master, tile_idx=jdx, col_idx=cur_col))
                _inv_list.append(
                    self.add_tile(inv_master, tile_idx=jdx, col_idx=cur_col + nand_master.num_cols + min_sep))

            for _nand, _inv in zip(_nand_list, _inv_list):
                self.connect_to_track_wires(_nand.get_pin('out'), _inv.get_pin('nin'))
            nand_list_list.append(_nand_list)
            inv_list_list.append(_inv_list)
            cur_col += nand_master.num_cols + inv_master.num_cols + min_sep
            cur_col += cur_col & 1

            sig_vm_locs_tidx_start = self.arr_info.col_to_track(vm_layer, cur_col)
            mid_sig_vm_locs_list.append(tr_manager.place_wires(vm_layer, (2 ** (idx + 2) + 3) * ['sig'],
                                                               align_track=sig_vm_locs_tidx_start)[1][1:])
            cur_col += vm_sig_ntr_to_col(2 ** (idx + 2) + 3)
            cur_col += cur_col & 1

        self.set_mos_size()

        buf_out_list, buf_outn_list = [], []
        for idx, buf in enumerate(buf_list):
            _, xm_locs = tr_manager.place_wires(xm_layer, ['sig'] * 2,
                                                center_coord=(buf.bound_box.yl + buf.bound_box.yh) // 2)
            buf_out_list.append(self.connect_to_tracks(buf.get_pin('out'), TrackID(xm_layer, xm_locs[0])))
            buf_outn_list.append(self.connect_to_tracks(buf.get_pin('outb'), TrackID(xm_layer, xm_locs[1])))
        buf_out_vm = self.connect_matching_tracks(buf_out_list + buf_outn_list, vm_layer, sig_vm_locs[1:],
                                                  track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
        buf_out_vm, buf_outn_vm = buf_out_vm[:nbits], buf_out_vm[nbits:]

        # Connect nands input
        nand_list = nand_list_list[0]
        nand_in0_list = [_nand.get_pin('nin<0>') for _nand in nand_list]
        nand_in1_list = [_nand.get_pin('nin<1>') for _nand in nand_list]
        for idx, inwarr in enumerate(nand_in0_list):
            self.connect_to_track_wires(buf_out_vm[0] if idx & 1 else buf_outn_vm[0], inwarr)
        for idx, inwarr in enumerate(nand_in1_list):
            self.connect_to_track_wires(buf_outn_vm[1] if idx < 2 else buf_out_vm[1], inwarr)

        for idx in range(nbits - 2):
            nand_prv_list = inv_list_list[idx]
            vm_locs = mid_sig_vm_locs_list[idx]
            xm_out = []
            for nand in nand_prv_list:
                nand_prv_out = nand.get_pin('out')
                xm_tidx = self.grid.coord_to_track(xm_layer, nand_prv_out.middle, RoundMode.NEAREST)
                xm_out.append(self.connect_to_tracks(nand_prv_out, TrackID(xm_layer, xm_tidx)))
            vm_warr = self.connect_matching_tracks(xm_out, vm_layer, vm_locs[:len(xm_out)],
                                                   track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)

            nand_nxt_list = nand_list_list[1 + idx]
            nand_in0_list = [_nand.get_pin('nin<0>') for _nand in nand_nxt_list]
            nand_in1_list = [_nand.get_pin('nin<1>') for _nand in nand_nxt_list]
            num_vm_prev = len(vm_warr)
            for jdx, in0 in enumerate(nand_in0_list):
                self.connect_to_track_wires(in0, vm_warr[jdx % num_vm_prev])
            bit_in = self.connect_matching_tracks([nand_in1_list[:num_vm_prev], nand_in1_list[num_vm_prev:]],
                                                  vm_layer, vm_locs[num_vm_prev:num_vm_prev + 2],
                                                  track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
            tile_info, yb, _ = self.get_tile_info(num_vm_prev)
            bit_xm = self.grid.coord_to_track(xm_layer, yb + tile_info.height // 2, RoundMode.NEAREST)
            tile_info, yb, _ = self.get_tile_info(num_vm_prev + 1)
            bitb_xm = self.grid.coord_to_track(xm_layer, yb + tile_info.height // 2, RoundMode.NEAREST)
            self.connect_matching_tracks([[buf_out_vm[idx + 2], bit_in[1]], [buf_outn_vm[idx + 2], bit_in[0]]],
                                         xm_layer, [bit_xm, bitb_xm])

        # Get output
        nand_list = inv_list_list[-1]
        xm_out = []
        for nand in nand_list:
            nand_prv_out = nand.get_pin('out')
            xm_tidx = self.grid.coord_to_track(xm_layer, nand_prv_out.lower, RoundMode.NEAREST)
            xm_out.append(self.connect_to_tracks(nand_prv_out, TrackID(xm_layer, xm_tidx)))

        [self.add_pin(f'out<{idx}>', out) for idx, out in enumerate(xm_out)]
        # Get input
        _, in_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * (nbits + 1),
                                               align_track=self.arr_info.col_to_track(vm_layer, 0))
        for idx in range(nbits):
            self.add_pin(f'in<{idx}>', buf_list[idx].get_pin('nin'), mode=PinMode.LOWER)

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = buf_list + [inst for inst_list in nand_list_list + inv_list_list for inst in inst_list]
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]

        self.add_pin('VDD_hm', vdd_hm, label='VDD', show=self.show_pins, connect=True)
        self.add_pin('VSS_hm', vss_hm, label='VSS', show=self.show_pins, connect=True)

        sch_params_dict = dict(
            buf=buf_master.sch_params,
            inv=inv_master.sch_params,
            nand=nand_master.sch_params,
            nbits=nbits
        )
        self.sch_params = sch_params_dict


class ClkDelayCapUnit(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_delay_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            prebuf='True to add one inverter before, use invchain in middle',
            pinfo='placement information object.',
            seg_dict='Optional dictionary of user defined signal locations',
            w_dict='',
            se=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(prebuf=True, se=False)

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']
        tx_names = list(w_dict.keys())
        tx_name_row_pair = [(tx, ridx_n if tx.startswith('n') else ridx_p) for tx in tx_names]

        w_ans = {}
        th_ans = {}
        for name, row_idx in tx_name_row_pair:
            rinfo = self.get_row_info(row_idx, 1)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)

    def make_decap_unit(self, ndecap, pdecap, se=False, pside=False):
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        tr_w_sig_hm = tr_manager.get_width(hm_layer, 'sig')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')

        out_conn = self.connect_wires([ndecap.g, pdecap.g])
        out_hm_tidx = self.grid.coord_to_track(hm_layer, out_conn[0].middle, RoundMode.NEAREST)
        out_hm_tidx_u = out_hm_tidx + 1
        out_hm_tidx_l = out_hm_tidx - 1

        out_hm_tidx = out_hm_tidx if se else (out_hm_tidx_l if pside else out_hm_tidx_u)
        out_hm = self.connect_to_tracks(out_conn, TrackID(hm_layer, out_hm_tidx, tr_w_sig_hm))

        # Connect n/pcap source
        pg0 = self.get_track_id(-1, MOSWireType.G, wire_name='sig', wire_idx=-1)
        ng0 = self.get_track_id(0, MOSWireType.G, wire_name='sig', wire_idx=0)
        ns_hm = self.connect_to_tracks(ndecap.s, ng0)
        ps_hm = self.connect_to_tracks(pdecap.s, pg0)

        # Connect n/pcap drain
        pd0 = self.get_track_id(-1, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        nd0 = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nd_hm = self.connect_to_tracks(ndecap.d, nd0)
        pd_hm = self.connect_to_tracks(pdecap.d, pd0)
        ns_hm, nd_hm = self.match_warr_length([ns_hm, nd_hm])
        ps_hm, pd_hm = self.match_warr_length([ps_hm, pd_hm])
        s_vm_coord = [self.grid.track_to_coord(vm_layer, s.track_id.base_index) for s in ndecap.s.to_warr_list()]
        g_vm_coord = [self.grid.track_to_coord(vm_layer, s.track_id.base_index) for s in ndecap.g.to_warr_list()]
        s_vm_tidx = [self.grid.coord_to_track(vm_layer, coord, RoundMode.NEAREST) for coord in s_vm_coord]
        g_vm_tidx = [self.grid.coord_to_track(vm_layer, coord, RoundMode.NEAREST) for coord in g_vm_coord]

        nds_vm = [self.connect_to_tracks([ns_hm, nd_hm], TrackID(vm_layer, tidx)) for tidx in s_vm_tidx]
        pds_vm = [self.connect_to_tracks([ps_hm, pd_hm], TrackID(vm_layer, tidx)) for tidx in s_vm_tidx]

        out_vm = [self.connect_to_tracks(out_hm, TrackID(vm_layer, tidx)) for tidx in g_vm_tidx]
        return pd_hm, ps_hm, nd_hm, ns_hm, out_hm, out_vm

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        seg_dict: dict = self.params['seg_dict']
        w_dict: dict = self.params['w_dict']
        se: bool = self.params['se']

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        seg_inv = seg_dict['inv']
        seg_ncap = seg_dict['ncap']
        seg_pcap = seg_dict['pcap']

        w_inv = w_dict['inv']
        w_ncap = w_dict['ncap']
        w_pcap = w_dict['pcap']

        inv_in_tidx = self.get_track_index(1, MOSWireType.G, wire_name='sig', wire_idx=-1)
        prebuf = self.params['prebuf']
        if prebuf:
            inv_in_tidx = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=0)
            inv_in_tidx2 = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=1)
            inv_out_tidx = self.get_track_index(0, MOSWireType.DS, wire_name='sig', wire_idx=-1)
            inv_out_tidx2 = self.get_track_index(0, MOSWireType.DS, wire_name='sig', wire_idx=-2)
            inv_outp_tidx = self.get_track_index(1, MOSWireType.DS, wire_name='sig', wire_idx=-1)
            inv_outp_tidx2 = self.get_track_index(1, MOSWireType.DS, wire_name='sig', wire_idx=-2)
            inv_params = dict(pinfo=pinfo, seg_list=[2 if seg_inv < 3 else seg_inv // 2, seg_inv],
                              w_p=w_inv, w_n=w_inv, vertical_out=True,
                              dual_output=True, sig_locs={'nin1': inv_in_tidx, 'nin0': inv_in_tidx2,
                                                          'nout1': inv_out_tidx, 'nout0': inv_out_tidx2,
                                                          'pout1': inv_outp_tidx, 'pout0': inv_outp_tidx2})
            inv_master = self.new_template(InvChainCore, params=inv_params)
        else:
            inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_inv, w_n=w_inv, vertical_out=False,
                              sig_locs={'nin': inv_in_tidx})
            inv_master = self.new_template(InvCore, params=inv_params)

        cur_col = 0
        inv = self.add_tile(inv_master, col_idx=cur_col, tile_idx=0)

        cur_col += min_sep + inv_master.num_cols
        cur_col += max(seg_ncap, seg_pcap)

        ncap_p = self.add_mos(0, cur_col, seg_ncap, w=w_ncap, flip_lr=True)
        pcap_p = self.add_mos(1, cur_col, seg_pcap, w=w_pcap, flip_lr=True)
        pd_p, ps_p, nd_p, ns_p, out_p, out_p_vm = self.make_decap_unit(ncap_p, pcap_p, se=se, pside=False)
        if not se:
            ncap_n = self.add_mos(0, cur_col, seg_ncap, w=w_ncap)
            pcap_n = self.add_mos(1, cur_col, seg_pcap, w=w_pcap)
            pd_n, ps_n, nd_n, ns_n, out_n, out_n_vm = self.make_decap_unit(ncap_n, pcap_n, se=se, pside=True)
        else:
            pd_n, ps_n, nd_n, ns_n, out_n, out_n_vm = [None] * 6
        self.set_mos_size()

        # Connect gate to output

        tr_w_sig_hm = tr_manager.get_width(hm_layer, 'sig')
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        if se:
            out_xm_tidx = self.grid.coord_to_track(xm_layer, out_p_vm[0].middle, RoundMode.NEAREST)
            out_xm = self.connect_to_tracks(out_p_vm, TrackID(xm_layer, out_xm_tidx,
                                                              width=tr_manager.get_width(xm_layer, 'clk')))
            self.add_pin('out', out_xm)
        else:
            _, out_xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 2,
                                                    center_coord=(inv.bound_box.yl + inv.bound_box.yh) // 2)
            out_n_xm, out_p_xm = self.connect_matching_tracks([out_n_vm, out_p_vm], xm_layer, out_xm_locs,
                                                              width=tr_manager.get_width(xm_layer, 'clk'))
            self.add_pin('outn', out_n_xm)
            self.add_pin('outp', out_p_xm)

        if prebuf:
            self.connect_to_track_wires(ns_p if se else [ns_n, ns_p], inv.get_pin('outb'))
            self.connect_to_track_wires(ps_p if se else [ps_n, ps_p], inv.get_pin('out'))
            inv_in_vm_tidx = self.grid.coord_to_track(vm_layer, inv.bound_box.xl, RoundMode.NEAREST)
            inv_in = self.connect_to_tracks(inv.get_pin('nin'), TrackID(vm_layer, inv_in_vm_tidx, tr_w_sig_vm))

        else:
            _, inv_inout_locs = tr_manager.place_wires(vm_layer, ['sig'] * 3,
                                                       center_coord=(inv.bound_box.xl + inv.bound_box.xh) // 2)

            inv_in_hm = self.connect_wires([ps_p, inv.get_pin('nin')] if se else [ps_n, ps_p, inv.get_pin('nin')])
            inv_out_hm = self.connect_wires(ns_p if se else [ns_n, ns_p])
            inv_in, inv_out = self.connect_matching_tracks([inv_in_hm, inv_out_hm], vm_layer, inv_inout_locs[1:],
                                                           width=tr_manager.get_width(vm_layer, 'sig'))
            self.connect_to_tracks([inv.get_pin('nout'), inv.get_pin('pout')] + inv_out_hm,
                                   TrackID(vm_layer, inv_inout_locs[0], tr_w_sig_vm))
            self.connect_to_track_wires([inv.get_pin('nout'), inv.get_pin('pout')], inv_out)

        vdd_hm = inv.get_pin('VDD')
        vss_hm = inv.get_pin('VSS')
        self.add_pin('VDD', self.extend_wires(vdd_hm, lower=self.bound_box.xl, upper=self.bound_box.xh))
        self.add_pin('VSS', self.extend_wires(vss_hm, lower=self.bound_box.xl, upper=self.bound_box.xh))
        self.add_pin('in', inv_in)

        w_dict, th_dict = self._get_w_th_dict(0, 1)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            inv=inv_master.sch_params,
            seg_dict=self.params['seg_dict'],
            w_dict=w_dict,
            th_dict=th_dict,
            prebuf=prebuf,
            se=se,
        )


class ClkDelayBinary(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._nbits = 0

    @property
    def nbits(self):
        return self._nbits

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_delay_binary')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            buf_seg_list='',
            cap_seg_list='',
            inv_seg_list='',
            seg_dict='Number of segments.',
            w_dict='',
            prebuf_idx_list='',
            nlsb_first_row='Number of lsbs in the first row',
            nmsb_in_col='Numbaer of msbs arrange in column',
            nlsb_groups='',
            num_cols_tot='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            prebuf_idx_list=[],
            nlsb_groups=[],
            num_cols_tot=0,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        nlsb_first_row: int = self.params['nlsb_first_row']
        nmsb_in_col: int = self.params['nmsb_in_col']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        w_dict: Dict[str, Any] = self.params['w_dict']
        cap_seg_list: ImmutableList[int] = self.params['cap_seg_list']
        inv_seg_list: ImmutableList[int] = self.params['inv_seg_list']
        buf_seg_list: ImmutableList[int] = self.params['buf_seg_list']

        num_cols_tot = self.params['num_cols_tot']
        prebuf_idx_list = self.params['prebuf_idx_list']

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        tr_manager = self.tr_manager

        nbits = nlsb_first_row + nmsb_in_col
        # routing space for input siganls
        in_vm_ntr, in_vm_locs = tr_manager.place_wires(vm_layer, ['dig'] * (nbits + 1),
                                                       align_track=self.arr_info.col_to_track(vm_layer, 0))
        in_vm_locs = in_vm_locs[1:]
        in_vm_ncol = self.arr_info.track_to_col(vm_layer, in_vm_ntr)
        if len(cap_seg_list) < nbits:
            cap_seg_gen_list = cap_seg_list.to_list() + [cap_seg_list[-1]] * (nbits - len(cap_seg_list))
        else:
            cap_seg_gen_list = cap_seg_list.to_list()
        if len(inv_seg_list) < nbits:
            inv_seg_gen_list = inv_seg_list.to_list() + [inv_seg_list[-1]] * (nbits - len(inv_seg_list))
        else:
            inv_seg_gen_list = inv_seg_list.to_list()
        if len(buf_seg_list) < nmsb_in_col:
            buf_gen_list = buf_seg_list.to_list() + [buf_seg_list[-1]] * (nbits - len(buf_seg_list))
        else:
            buf_gen_list = buf_seg_list.to_list()
        unit_params_list = []
        for idx in range(nbits):
            _seg_dict = dict(ncap=cap_seg_gen_list[idx], pcap=cap_seg_gen_list[idx], inv=inv_seg_gen_list[idx])
            unit_params_list.append(dict(pinfo=pinfo, seg_dict=_seg_dict, w_dict=w_dict,
                                         prebuf=not bool(idx in prebuf_idx_list)))

        buf_params_list = [dict(seg_list=[seg, 2 * seg], pinfo=pinfo) for seg in buf_gen_list]

        unit_master_list = [self.new_template(ClkDelayCapUnit, params=unit_params) for unit_params in unit_params_list]
        buf_master_list = [self.new_template(InvChainCore, params=buf_params) for buf_params in buf_params_list]

        cur_col = 0
        lsb_units, msb_units = [], []

        buf_list_list = []
        cur_row = 0
        for idx in range(nmsb_in_col):
            msb_unit_list = []
            buf_list = []
            for jdx in range(2 ** idx):
                cur_col = max(0, num_cols_tot - buf_master_list[idx].num_cols -
                              unit_master_list[idx + nlsb_first_row].num_cols - min_sep)
                buf_list.append(self.add_tile(buf_master_list[idx], col_idx=cur_col, tile_idx=cur_row))
                cur_col += buf_master_list[idx].num_cols + min_sep
                msb_unit_list.append(self.add_tile(unit_master_list[idx + nlsb_first_row],
                                                   col_idx=cur_col, tile_idx=cur_row))
                tile_yb = self.get_tile_info(cur_row)[1]
                in_xm_tidx = self.grid.coord_to_track(xm_layer, msb_unit_list[-1].get_pin('in').lower,
                                                      RoundMode.LESS)
                in_xm = self.connect_to_tracks(buf_list[-1].get_pin('out'), TrackID(xm_layer, in_xm_tidx))
                self.connect_to_track_wires(in_xm, msb_unit_list[-1].get_pin('in'))
                cur_row += 1
            msb_units.append(msb_unit_list)
            buf_list_list.append(buf_list)

        cur_col = 0
        lsb_ncols = sum([inst.num_cols for inst in unit_master_list[:nlsb_first_row]]) + (nlsb_first_row - 1) * min_sep
        max_ncols = max(lsb_ncols, self.num_cols, num_cols_tot)
        cur_col = max_ncols - lsb_ncols
        for idx in range(nlsb_first_row):
            lsb_units.append(self.add_tile(unit_master_list[idx], col_idx=cur_col, tile_idx=cur_row))
            cur_col += unit_master_list[idx].num_cols + min_sep

        # out_ym_ntr, out_ym_locs = \
        #     tr_manager.place_wires(ym_layer, ['dum'] + ['clk'] * 2 + ['dum'], align_idx=-1,
        #                            align_track=self.arr_info.col_to_track(vm_layer, self.num_cols))
        # tot_cols = self.num_cols + self.arr_info.track_to_col(vm_layer, out_vm_ntr)
        self.set_mos_size(num_cols=self.num_cols)

        msb_flatten_list = [inst for inst_list in msb_units for inst in inst_list]

        unit_outp_list, unit_outn_list = [], []
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        for unit in msb_flatten_list + lsb_units:
            unit_outn_list.append(unit.get_pin(f'outn'))
            unit_outp_list.append(unit.get_pin(f'outp'))

        unit_outp_list = self.connect_wires(unit_outp_list)
        unit_outn_list = self.connect_wires(unit_outn_list)
        # outn_ym, outp_ym = self.connect_matching_tracks([unit_outn_list, unit_outp_list], ym_layer,
        #                                                 out_ym_locs[1:-1], width=tr_w_clk_ym)
        # Connect lsb inputs
        tile_info, tile_yb, _ = self.get_tile_info(self.num_tile_rows - 1)
        tr_w_dig_xm = tr_manager.get_width(xm_layer, 'dig')
        in_xm_lower_tid = self.grid.coord_to_track(xm_layer, tile_yb, mode=RoundMode.NEAREST)
        in_xm_lower_tid = tr_manager.get_next_track(xm_layer, in_xm_lower_tid, 'sup', 'dig')
        in_xm_upper_tid = self.grid.coord_to_track(xm_layer, tile_yb + tile_info.height, mode=RoundMode.NEAREST)
        in_xm_locs = self.get_available_tracks(xm_layer, in_xm_lower_tid, in_xm_upper_tid,
                                               self.bound_box.xl, self.bound_box.xh, width=tr_w_dig_xm,
                                               sep=tr_manager.get_sep(xm_layer, ('dig', 'dig')))
        tile_info, tile_yb, _ = self.get_tile_info(1)
        in_xm_lower_tid = self.grid.coord_to_track(xm_layer, tile_yb, mode=RoundMode.NEAREST)
        in_xm_lower_tid = tr_manager.get_next_track(xm_layer, in_xm_lower_tid, 'sup', 'dig')
        in_xm_upper_tid = self.grid.coord_to_track(xm_layer, tile_yb + tile_info.height, mode=RoundMode.NEAREST)

        in_xm_locs.extend(self.get_available_tracks(xm_layer, in_xm_lower_tid, in_xm_upper_tid,
                                                    self.bound_box.xl, self.bound_box.xh, width=tr_w_dig_xm,
                                                    sep=tr_manager.get_sep(xm_layer, ('dig', 'dig'))))

        nlsb_groups = self.params['nlsb_groups']
        if nlsb_groups:
            inst_group_list = []
            in_lsb_xm = []
            for idx in range(len(nlsb_groups)):
                start_num = sum(nlsb_groups[:idx])
                stop_num = sum(nlsb_groups[:idx + 1])
                inst_group_list.append(lsb_units[start_num: stop_num])
                in_lsb_xm.append(self.connect_to_tracks([inst.get_pin('in') for inst in lsb_units[start_num: stop_num]],
                                                        TrackID(xm_layer, in_xm_locs[idx], tr_w_dig_xm)))
            in_lsb_xm = self.match_warr_length(in_lsb_xm)
        else:
            in_lsb_xm = self.connect_matching_tracks([inst.get_pin('in') for inst in lsb_units], xm_layer,
                                                     in_xm_locs[:nlsb_first_row], width=tr_w_dig_xm)
        in_sig_list = []
        tr_w_dig_vm = tr_manager.get_width(vm_layer, 'dig')
        for in_xm, vm_locs in zip(in_lsb_xm, in_vm_locs):
            in_sig_list.append(in_xm)
        for buf_list, vm_locs in zip(buf_list_list, in_vm_locs[nlsb_first_row:]):
            in_sig_list.append([inst.get_pin('in') for inst in buf_list])

        inst_list = msb_flatten_list + lsb_units
        vdd_hm_list, vss_hm_list = [], []

        for inst in inst_list:
            vdd_hm_list.extend(inst.get_all_port_pins('VDD'))
            vss_hm_list.extend(inst.get_all_port_pins('VSS'))

        vdd_hm = self.connect_wires(vdd_hm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_hm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]
        self.add_pin('VDD_hm', vdd_hm, label='VDD', show=self.show_pins)
        self.add_pin('VSS_hm', vss_hm, label='VSS', show=self.show_pins)
        vdd_xm_list, vss_xm_list = [], []

        for vdd in vdd_hm:
            vdd_xm_list.append(self.export_tap_hm(tr_manager, vdd, hm_layer, xm_layer)[0])
        for vss in vss_hm:
            vss_xm_list.append(self.export_tap_hm(tr_manager, vss, hm_layer, xm_layer)[0])

        self.add_pin('VDD_xm', vdd_xm_list, label='VDD', show=self.show_pins)
        self.add_pin('VSS_xm', vss_xm_list, label='VSS', show=self.show_pins)

        self.add_pin('outn', unit_outn_list, connect=True)
        self.add_pin('outp', unit_outp_list, connect=True)
        if nlsb_groups:
            for idx in range(len(nlsb_groups)):
                self.add_pin(f'in<{idx}>', in_sig_list[idx], mode=PinMode.LOWER, connect=True)
        else:
            for idx in range(nbits):
                self.add_pin(f'in<{idx}>', in_sig_list[idx], mode=PinMode.LOWER, connect=True)
        #
        # for idx in range(nbits):
        #     self.reexport(dec.get_port(f'in<{idx}>'))
        self._nbits = nbits = len(nlsb_groups) if nlsb_groups else nmsb_in_col + nlsb_first_row
        sch_params_dict = dict(
            nlsb=nlsb_first_row,
            nmsb=nmsb_in_col,
            buf_list=[buf.sch_params for buf in buf_master_list],
            unit_list=[unit.sch_params for unit in unit_master_list],
            nlsb_groups=nlsb_groups,
            nbits=nbits
        )
        self.sch_params = sch_params_dict


class ClkDelay(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_delay_tune')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            binary_col_params='',
            binary_row_params='',
            unary_params='',
            top_sup_layer=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(num_unary_rows=1)

    def draw_layout(self) -> None:
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

        # setup floorplan
        binary_row_params: ImmutableSortedDict = self.params['binary_row_params']
        binary_col_params: ImmutableSortedDict = self.params['binary_col_params']
        unary_params: ImmutableSortedDict = self.params['unary_params']
        if isinstance(unary_params, str):
            spec_yaml = read_yaml(unary_params)
            unary_params = spec_yaml['params']
        if isinstance(binary_col_params, str):
            spec_yaml = read_yaml(binary_col_params)
            binary_col_params = spec_yaml['params']
        if isinstance(binary_row_params, str):
            spec_yaml = read_yaml(binary_row_params)
            binary_row_params = spec_yaml['params']

        binary_col_master = self.new_template(MOSBaseTapWrapper, params=binary_col_params)
        unary_master = self.new_template(MOSBaseTapWrapper, params=unary_params)
        binary_row_master = self.new_template(MOSBaseTapWrapper, params=binary_row_params)
        max_ncols = max(binary_col_master.core.num_cols, binary_row_master.core.num_cols, unary_master.core.num_cols)
        for params in [binary_row_params, binary_col_params, unary_params]:
            params['params']['num_cols_tot'] = max_ncols
        binary_col_master = self.new_template(MOSBaseTapWrapper, params=binary_col_params)
        unary_master = self.new_template(MOSBaseTapWrapper, params=unary_params)
        binary_row_master = self.new_template(MOSBaseTapWrapper, params=binary_row_params)

        top_layer = binary_col_master.top_layer
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        unary_w, unary_h = unary_master.bound_box.w, unary_master.bound_box.h
        bi_col_w, bi_col_h = binary_col_master.bound_box.w, binary_col_master.bound_box.h
        bi_row_w, bi_row_h = binary_row_master.bound_box.w, binary_row_master.bound_box.h

        w_tot = max(binary_col_master.bound_box.w, binary_row_master.bound_box.w, unary_master.bound_box.w)
        w_tot = -(-w_tot // blk_w) * blk_w
        bi_col_x = -(-(w_tot - bi_col_w) // blk_w) * blk_w
        bi_row_x = -(-(w_tot - bi_row_w) // blk_w) * blk_w
        unary_x = -(-(w_tot - unary_w) // blk_w) * blk_w
        cur_y = 0
        binary_col_l = self.add_instance(binary_col_master, xform=Transform(bi_col_x, cur_y, Orientation.R0))
        cur_y += -(-bi_col_h // blk_h) * blk_h
        binary_row_l = self.add_instance(binary_row_master, xform=Transform(bi_row_x, cur_y, Orientation.R0))
        cur_y += -(-bi_row_h // blk_h) * blk_h
        unary = self.add_instance(unary_master, xform=Transform(unary_x, cur_y, Orientation.R0))
        cur_y += -(-unary_h // blk_h) * blk_h
        cur_y += -(-bi_row_h // blk_h) * blk_h
        binary_row_u = self.add_instance(binary_row_master, xform=Transform(bi_row_x, cur_y, Orientation.MX))
        cur_y += -(-bi_col_h // blk_h) * blk_h
        binary_col_u = self.add_instance(binary_col_master, xform=Transform(bi_col_x, cur_y, Orientation.MX))

        h_tot = -(-binary_col_u.bound_box.yh // blk_h) * blk_h
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot))

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      params['params']['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        nbits_tot = unary_master.core.nbits + binary_col_master.core.nbits + binary_row_master.core.nbits
        _, vm_ctrl_locs = tr_manager.place_wires(vm_layer, ['sig'] * (nbits_tot + 1), align_idx=-1,
                                                 align_track=self.grid.coord_to_track(vm_layer, self.bound_box.xl,
                                                                                      RoundMode.NEAREST))
        lsb = unary_master.core.nbits
        msb_m = binary_row_master.core.nbits
        msb_l = binary_col_master.core.nbits
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        for idx in range(lsb):
            in_ym = self.connect_to_tracks(unary.get_all_port_pins(f'in<{idx}>'),
                                           TrackID(vm_layer, vm_ctrl_locs[idx], tr_w_sig_vm),
                                           track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
            self.add_pin(f'in<{idx}>', in_ym, mode=PinMode.UPPER)

        for idx in range(msb_m):
            in_ym = self.connect_to_tracks(binary_row_u.get_all_port_pins(f'in<{idx}>')+\
                                           binary_row_l.get_all_port_pins(f'in<{idx}>'),
                                           TrackID(vm_layer, vm_ctrl_locs[idx+lsb], tr_w_sig_vm),
                                           track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
            self.add_pin(f'in<{idx+lsb}>', in_ym, mode=PinMode.UPPER)

        for idx in range(msb_l):
            in_ym = self.connect_to_tracks(binary_col_u.get_all_port_pins(f'in<{idx}>')+\
                                           binary_col_l.get_all_port_pins(f'in<{idx}>'),
                                           TrackID(vm_layer, vm_ctrl_locs[idx+lsb+msb_m], tr_w_sig_vm),
                                           track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)
            self.add_pin(f'in<{idx+lsb+msb_m}>', in_ym, mode=PinMode.UPPER)

        ym_tidx_bnd_l = self.grid.coord_to_track(ym_layer, unary.get_pin('outn').middle, RoundMode.NEAREST)
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        outn_ym_tidx = ym_tidx_bnd_l - self.get_track_sep(ym_layer, tr_w_clk_ym, 1)
        outp_ym_tidx = outn_ym_tidx - self.get_track_sep(ym_layer, tr_w_clk_ym, tr_w_clk_ym)

        inst_list = [binary_col_l, binary_row_l, binary_row_u, binary_col_u]
        outn_xm = [inst.get_all_port_pins('outn') for inst in inst_list]
        outp_xm = [inst.get_all_port_pins('outp') for inst in inst_list]
        outn_ym, outp_ym = self.connect_differential_tracks(unary.get_all_port_pins('outn'),
                                                            unary.get_all_port_pins('outp'),
                                                            ym_layer, outn_ym_tidx, outp_ym_tidx, width=tr_w_clk_ym)
        self.connect_differential_wires([w for wlist in outn_xm for w in wlist],
                                        [w for wlist in outp_xm for w in wlist],
                                        outn_ym, outp_ym)

        # # might not need xm1 routings
        #
        xm1_layer = ym_layer + 1
        tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        _, out_xm1_locs = tr_manager.place_wires(xm1_layer, ('clk', 'dum', 'sup', 'dum', 'clk'),
                                                 center_coord=outn_ym[0].middle)

        out_xm1_locs.pop(1)
        out_n_xm1, out_p_xm1 = self.connect_differential_tracks(outn_ym, outp_ym, xm1_layer,
                                                                out_xm1_locs[0], out_xm1_locs[-1], width=tr_w_clk_xm1)

        self.add_pin('outn', out_n_xm1)
        self.add_pin('outp', out_p_xm1)
        top_sup_layer = self.params['top_sup_layer']
        inst_list.append(unary)
        last_vdd_list = [w for inst in inst_list for w in inst.get_all_port_pins('VDD', layer=xm_layer)]
        last_vss_list = [w for inst in inst_list for w in inst.get_all_port_pins('VSS', layer=xm_layer)]
        for idx in range(xm_layer + 1, top_sup_layer + 1):
            last_vdd_list, last_vss_list = \
                self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list], idx - 1, self.bound_box,
                                         side_sup=False, extend_lower_layer=False)
        self.add_pin('VDD', last_vdd_list)
        self.add_pin('VSS', last_vss_list)

        sch_params_dict = dict(
            binary_l=binary_row_master.sch_params,
            binary_u=binary_col_master.sch_params,
            unary=unary_master.sch_params,
        )
        self.sch_params = sch_params_dict
