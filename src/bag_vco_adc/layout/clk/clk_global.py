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


import itertools
from typing import Mapping, Any, Optional, Type, Dict, List, Tuple, Union

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from bag.util.math import HalfInt
from bag_vco_adc.layout.clk.clk_rx import ClkRxBuf
from bag_vco_adc.layout.clk.clk_rx import DiffInvCoupledChain
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.wrapper import GenericWrapper, IntegrationWrapper
from pybag.core import BBox, Transform
from pybag.enum import RoundMode, Direction, MinLenMode, Orientation, Orient2D
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo, TilePatternElement, TilePattern
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID


class InvRow4(MOSBase, TemplateBaseZL):
    """Inverter whose pmos and nmos both split into two rows
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls):
        # type: () -> Dict[str, str]
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            sig_locs='Signal track location dictionary.',
            show_pins='True to draw pin geometries.',
            ridx_p0='bottom pmos row index.',
            ridx_p1='top pmos row index.',
            ridx_n0='bottom nmos row index.',
            ridx_n1='top nmos row index.',
            seg='number of segments.',
            stack_p='number of transistors in a stack.',
            stack_n='number of transistors in a stack.',
            w_p='pmos width, can be list or integer if all widths are the same.',
            w_n='pmos width, can be list or integer if all widths are the same.',
            vertical_in='True to have input connected to M3',
            vertical_out='True to have output connected to M3',
            vdd_on_hm='True to connect vdd on hm',
            vss_on_hm='True to connect vss on hm',
            num_out_vm='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            sig_locs={},
            show_pins=True,
            ridx_p1=-1,
            ridx_p0=-2,
            ridx_n1=1,
            ridx_n0=0,
            stack_p=1,
            stack_n=1,
            w_p=0,
            w_n=0,
            vertical_in=False,
            vertical_out=False,
            vdd_on_hm=True,
            vss_on_hm=True,
            num_out_vm=1,
        )

    def get_layout_basename(self):
        return 'load_%x' % self.params['seg']

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_2 = self.params['seg']
        stack_p = self.params['stack_p']
        stack_n = self.params['stack_n']
        sig_locs = self.params['sig_locs']
        show_pins = self.params['show_pins']
        ridx_n0 = self.params['ridx_n0']
        ridx_n1 = self.params['ridx_n1']
        ridx_p0 = self.params['ridx_p0']
        ridx_p1 = self.params['ridx_p1']
        w_p = self.params['w_p']
        w_n = self.params['w_n']
        vertical_in = self.params['vertical_in']
        vertical_out = self.params['vertical_out']
        vdd_on_hm = self.params['vdd_on_hm']
        vss_on_hm = self.params['vss_on_hm']

        row_info_n = self.place_info.get_row_place_info(ridx_n0).row_info
        row_info_p = self.place_info.get_row_place_info(ridx_p0).row_info
        wp_row = row_info_p.width
        wn_row = row_info_n.width

        if seg_2 & 1:
            raise ValueError('seg has to be even to split into two rows.')
        seg = seg_2 // 2

        if w_p == 0:
            w_p = wp_row
        if w_n == 0:
            w_n = wn_row
        if w_p < 0 or w_p > wp_row or w_n < 0 or w_n > wn_row:
            raise ValueError('Invalid choice of wp and/or wn.')

        # out_tidx = sig_locs.get('out', self.get_track_index(ridx_n1, MOSWireType.DS, wire_name='clk2', wire_idx=-1))
        pout_tidx = sig_locs.get('pout', self.get_track_index(ridx_p1, MOSWireType.DS, wire_name='clk', wire_idx=0))
        nout_tidx = sig_locs.get('nout', self.get_track_index(ridx_n0, MOSWireType.DS, wire_name='clk', wire_idx=-1))
        nin1_tidx = sig_locs.get('nin1', self.get_track_index(ridx_n1, MOSWireType.G, wire_name='clk2', wire_idx=0))
        nin0_tidx = sig_locs.get('nin0', self.get_track_index(ridx_n0, MOSWireType.G, wire_name='clk', wire_idx=0))
        pin1_tidx = sig_locs.get('pin1', self.get_track_index(ridx_p1, MOSWireType.G, wire_name='clk', wire_idx=-1))
        pin0_tidx = sig_locs.get('pin0', self.get_track_index(ridx_p0, MOSWireType.G, wire_name='clk2', wire_idx=-1))
        out_tidx = self.grid.get_middle_track(pout_tidx, nout_tidx, round_up=False)

        tr_manager = self.tr_manager

        # get track information
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        sig_w_h = tr_manager.get_width(hm_layer, 'sig')
        sup_w_h = tr_manager.get_width(hm_layer, 'sup')
        sig_w_v = tr_manager.get_width(vm_layer, 'sig')

        # add blocks and collect wires
        p0 = self.add_mos(ridx_p0, 0, seg, w=w_p, stack=stack_p)
        p1 = self.add_mos(ridx_p1, 0, seg, w=w_p, stack=stack_p)
        n0 = self.add_mos(ridx_n0, 0, seg, w=w_n, stack=stack_n)
        n1 = self.add_mos(ridx_n1, 0, seg, w=w_n, stack=stack_n)

        # compute overall block size and fill spaces
        self.set_mos_size()

        # connect output
        out = self.connect_to_tracks([p0.d, n1.d], TrackID(hm_layer, out_tidx, width=sig_w_h))
        pout = self.connect_to_tracks(p1.d, TrackID(hm_layer, pout_tidx, width=sig_w_h))
        nout = self.connect_to_tracks(n0.d, TrackID(hm_layer, nout_tidx, width=sig_w_h))
        num_out_vm = self.params['num_out_vm']
        ret_out_hm, ret_in_hm = [], []
        if vertical_out:
            out_tidx_vm = [self.grid.coord_to_track(vm_layer, out.upper, mode=RoundMode.NEAREST)]
            if num_out_vm > 1:
                out_tidx_vm.append(tr_manager.get_next_track(vm_layer, out_tidx_vm[-1], 'sig', 'sig', up= -1))
            out_vm = [self.connect_to_tracks([out, pout, nout], TrackID(vm_layer, idx, width=sig_w_v),
                                            ret_wire_list=ret_out_hm) for idx in out_tidx_vm]

        # connect input
        nin0 = self.connect_to_tracks(n0.g, TrackID(hm_layer, nin0_tidx, width=sig_w_h))
        nin1 = self.connect_to_tracks(n1.g, TrackID(hm_layer, nin1_tidx, width=sig_w_h))
        pin0 = self.connect_to_tracks(p0.g, TrackID(hm_layer, pin0_tidx, width=sig_w_h))
        pin1 = self.connect_to_tracks(p1.g, TrackID(hm_layer, pin1_tidx, width=sig_w_h))
        if vertical_in:
            in_tidx_vm = self.grid.coord_to_track(vm_layer, nin0.lower, mode=RoundMode.NEAREST)
            if vertical_out:
                in_tidx_vm = [min(tr_manager.get_next_track(vm_layer, out_tidx_vm[-1], 'sig', 'sig', up=False), in_tidx_vm)]
            else:
                in_tidx_vm = [in_tidx_vm]
            if num_out_vm > 1:
                in_tidx_vm.append(tr_manager.get_next_track(vm_layer, in_tidx_vm[-1], 'sig', 'sig', up=1))

            in_vm = [self.connect_to_tracks([nin0, nin1, pin0, pin1], TrackID(vm_layer, idx, width=sig_w_v),
                                           ret_wire_list=ret_in_hm) for idx in in_tidx_vm]

        # connect supplies
        if vss_on_hm:
            vss_tidx = sig_locs.get('vss', self.get_track_index(ridx_n0, MOSWireType.G, wire_name='sup', wire_idx=-1))
            vss = self.connect_to_tracks([n0.s, n1.s], TrackID(hm_layer, vss_tidx, width=sup_w_h))
        else:
            vss = [n0.s, n1.s]
        if vdd_on_hm:
            vdd_tidx = sig_locs.get('vdd', self.get_track_index(ridx_p1, MOSWireType.G, wire_name='sup', wire_idx=0))
            vdd = self.connect_to_tracks([p0.s, p1.s], TrackID(hm_layer, vdd_tidx, width=sup_w_h))
        else:
            vdd = [p0.s, p1.s]

        self.match_warr_length(ret_out_hm+ret_in_hm)
        # export
        self.add_pin('VSS', vss, show=show_pins)
        self.add_pin('VDD', vdd, show=show_pins)
        out_dict = {'out': out, 'pout': pout, 'nout': nout}
        in_dict = {'nin0': nin0, 'nin1': nin1, 'pin0': pin0, 'pin1': pin1}
        if vertical_in:
            in_dict.update(in_vm=in_vm)
        if vertical_out:
            out_dict.update(out_vm=out_vm)
        for pin, warr in out_dict.items():
            self.add_pin(pin, warr, label='out', connect=True, show=show_pins)
        for pin, warr in in_dict.items():
            self.add_pin(pin, warr, label='in', connect=True, show=show_pins)

        # set properties
        self._sch_params = dict(
            lch=self.place_info.lch,
            w_p=w_p,
            w_n=w_n,
            th_p=row_info_p.threshold,
            th_n=row_info_n.threshold,
            seg_p=seg_2,
            seg_n=seg_2,
            stack_p=stack_p,
            stack_n=stack_n,
        )


class InvChainRow4(MOSBase):
    """Two inverter chains composed of InvRow4 from left to right
       """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._out_invert = None

    @property
    def out_invert(self) -> bool:
        return self._out_invert

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            inv_chain_params='inverter chain parameters',
            ridx_p0='bottom pmos row index.',
            ridx_p1='top pmos row index.',
            ridx_n0='bottom nmos row index.',
            ridx_n1='top nmos row index.',
            add_tap='(tap_left, tap_right) True to add substrate tap columns at left/ right',
            sig_locs='Signal track location dictionary.',
            dual_output='True to export complementary outputs  Ignored if only one stage.',
            num_out_vm='',
            w_n='',
            w_p='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ridx_p1=-1,
            ridx_p0=-2,
            ridx_n1=1,
            ridx_n0=0,
            add_tap=(False, False),
            sig_locs={},
            dual_output=False,
            num_out_vm=1,
            w_n=4,
            w_p=4,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        ridx_n0: int = self.params['ridx_n0']
        ridx_n1: int = self.params['ridx_n1']
        ridx_p0: int = self.params['ridx_p0']
        ridx_p1: int = self.params['ridx_p1']
        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']

        seg_list: List = self.params['inv_chain_params']['seg_list']
        dual_output: bool = self.params['dual_output']

        hm_layer = self.conn_layer + 1

        _pinfo_logic, _, _ = self.get_tile_info(0)

        # --- Placement --- #
        # add tap column

        cur_col = 0

        pout_tidx_odd = self.get_track_index(ridx_p1, MOSWireType.DS, wire_name='clk', wire_idx=0)
        pout_tidx_even = self.get_track_index(ridx_p1, MOSWireType.DS, wire_name='clk', wire_idx=-1)
        out_tidx_odd = self.get_track_index(ridx_n1, MOSWireType.DS, wire_name='clk2', wire_idx=-1)
        out_tidx_even = self.get_track_index(ridx_p0, MOSWireType.DS, wire_name='clk2', wire_idx=0)
        nout_tidx_odd = self.get_track_index(ridx_n0, MOSWireType.DS, wire_name='clk', wire_idx=-1)
        nout_tidx_even = self.get_track_index(ridx_n0, MOSWireType.DS, wire_name='clk', wire_idx=0)
        sig_locs_odd = dict(pout=pout_tidx_odd, out=out_tidx_odd, nout=nout_tidx_odd)
        sig_locs_even = dict(pout=pout_tidx_even, out=out_tidx_even, nout=nout_tidx_even)
        inv_params = dict(pinfo=_pinfo_logic, vertical_in=True, vertical_out=True, ridx_n0=ridx_n0, ridx_n1=ridx_n1,
                          ridx_p0=ridx_p0, ridx_p1=ridx_p1, num_out_vm=self.params['num_out_vm'], w_n=w_n, w_p=w_p)
        inv_master, inv_inst = [], []
        for idx, seg in enumerate(seg_list):
            sig_locs = sig_locs_odd if idx & 1 else sig_locs_even
            inv_master.append(self.new_template(InvRow4, params=dict(**inv_params, seg=seg, vdd_on_hm=False,
                                                                     vss_on_hm=False, sig_locs=sig_locs)))
            inv_inst.append(self.add_tile(inv_master[idx], 0, cur_col))
            cur_col += inv_master[idx].num_cols

        self.set_mos_size(self.num_cols, num_tiles=self.num_tile_rows)

        tr_manager = _pinfo_logic.tr_manager
        w_sup_hm = tr_manager.get_width(hm_layer, 'sup')

        # # --- Routing --- #
        # VDD/ VSS
        for inst, pin in itertools.product(inv_inst, ['VDD', 'VSS']):
            self.reexport(inst.get_port(pin), connect=True)
        # in
        for pin in ['nin0', 'nin1', 'pin0', 'pin1', 'in_vm']:
            self.reexport(inv_inst[0].get_port(pin), label='in', connect=True)
        out_name = 'outb' if len(inv_master) & 1 else 'out'
        outb_name = 'out' if len(inv_master) & 1 else 'outb'
        # out
        for pin in ['out', 'pout', 'nout', 'out_vm']:
            self.reexport(inv_inst[-1].get_port(pin), label=out_name, connect=True)

        # outb (if dual_output)
        if len(inv_inst) > 1 and dual_output:
            for pin in ['nin0', 'nin1', 'pin0', 'pin1', 'in_vm']:
                self.reexport(inv_inst[-1].get_port(pin), label=outb_name, connect=True)

        # connect inverters
        for idx in range(len(inv_inst) - 1):
            pre_out = [inv_inst[idx].get_all_port_pins(pin)[0] for pin in ['out', 'pout', 'nout']]
            post_in = inv_inst[idx + 1].get_all_port_pins('in_vm')[0]
            self.connect_to_track_wires(pre_out, post_in, min_len_mode=MinLenMode.MIDDLE)

        self.sch_params = dict(inv_params=[master.sch_params for master in inv_master], dual_output=dual_output)


class C2mosLatch(MOSBase):
    """A c2mos latch in clock path
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    # @classmethod
    # def get_schematic_class(cls) -> Optional[Type[Module]]:
    #     # noinspection PyTypeChecker
    #     return bag_vco__c2mos_latch

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']

        w_ans = {}
        th_ans = {}
        for name, row_idx in [('nfb', ridx_n), ('nin', ridx_n), ('pfb', ridx_p), ('pin', ridx_p), ('ntail', ridx_n),
                              ('ptail', ridx_p)]:
            rinfo = self.get_row_info(row_idx, 0)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        """Returns a dictionary containing parameter descriptions.

        Override this method to return a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : dict[str, str]
            dictionary from parameter name to description.
        """
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            ridx_tailn='tailn row index',
            seg_dict='',
            w_dict='',
            ndum_side='min dummy number at one side, need even number',
            show_pins='True to show pins',
            in_track_offset='input track offset',
            vdd_on_hm='True to connect vdd on hm',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        """Returns a dictionary containing default parameter values.

        Override this method to define default parameter values.  As good practice,
        you should avoid defining default values for technology-dependent parameters
        (such as channel length, transistor width, etc.), but only define default
        values for technology-independent parameters (such as number of tracks).

        Returns
        -------
        default_params : dict[str, any]
            dictionary of default parameter values.
        """
        return dict(
            show_pins=False,
            in_track_offset=0,
            ndum_side=2,
            ridx_tailn=0,
            vdd_on_hm=True,
            w_dict={},
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        # parameters
        params = self.params
        ridx_tailn: int = params['ridx_tailn']
        ridx_coren = ridx_tailn + 1
        ridx_corep = ridx_coren + 1
        ridx_tailp = ridx_corep + 1
        seg_dict = params['seg_dict']
        ndum_side = params['ndum_side']
        show_pins = params['show_pins']
        in_track_offset = params['in_track_offset']
        vdd_on_hm = params['vdd_on_hm']

        nfn_tail = seg_dict['ntail']
        nfp_tail = seg_dict['ptail']
        nf_inv = seg_dict['nin']
        nf_cp = seg_dict['pfb']

        # # check number of fingers
        # if nfn % 2 != 0:
        #     raise ValueError("Need nfn divisible by 4")
        # if nfp % 2 != 0:
        #     raise ValueError("Need nfp divisible by 4")
        # if ndum_cen % 2 != 0:
        #     raise ValueError("We need ndum_cen being even to make layout symmetric.")
        # if ndum_side % 2 != 0:
        #     raise ValueError("We need ndum_side being even to make layout symmetric.")
        if (nf_inv * 2 - nfp_tail) % 4 != 0 or (nf_inv * 2 - nfn_tail) % 4 != 0:
            raise ValueError(
                "We need (seg_inv*2-segp_tail) % 4 == 0 and (seg_inv*2-segn_tail) % 4 == 0 to make layout symmetric.")

        seg_tot = nf_inv * 2 + nf_cp * 2 + ndum_side * 2 + 2 * 2
        grid = self.grid
        tr_manager = self.tr_manager

        # get track id
        ntail_g_tid = self.get_track_id(ridx_tailn, MOSWireType.G, wire_name='clk')
        ptail_g_tid = self.get_track_id(ridx_tailp, MOSWireType.G, wire_name='clk')
        ntail_d_tid = self.get_track_id(ridx_tailn, MOSWireType.DS, wire_name='clk', wire_idx=0)
        ptail_d_tid = self.get_track_id(ridx_tailp, MOSWireType.DS, wire_name='clk', wire_idx=-1)
        ninv_g_tid = self.get_track_id(ridx_coren, MOSWireType.G, wire_name='clk2', wire_idx=0)
        pinv_g_tid = self.get_track_id(ridx_corep, MOSWireType.G, wire_name='clk2', wire_idx=-1)
        ninv_d_tid = self.get_track_id(ridx_coren, MOSWireType.DS, wire_name='clk2', wire_idx=0)
        pinv_d_tid = self.get_track_id(ridx_corep, MOSWireType.DS, wire_name='clk2', wire_idx=0)

        # --- placement ---#
        # wn_cen = self.place_info.get_row_place_info(ridx_coren).row_info.width
        # wp_cen = self.place_info.get_row_place_info(ridx_corep).row_info.width
        # wn_tail = self.place_info.get_row_place_info(ridx_tailn).row_info.width
        # wp_tail = self.place_info.get_row_place_info(ridx_tailp).row_info.width
        w_dict, th_dict = self._get_w_th_dict(0, -1)
        wn_tail = w_dict['ntail']
        wp_tail = w_dict['ptail']
        wn_inv = w_dict['nin']
        wp_inv = w_dict['pin']
        wn_fb = w_dict['nfb']
        wp_fb = w_dict['pfb']

        # FIXME: fliplr or not
        # center two rows
        min_sep = self.min_sep_col + 1 if self.min_sep_col & 1 else self.min_sep_col
        cur_col = ndum_side
        nmos0_cp = self.add_mos(ridx_coren, cur_col, nf_cp, w=wn_tail)
        pmos0_cp = self.add_mos(ridx_corep, cur_col, nf_cp, w=wp_tail)

        cur_col += nf_cp + min_sep
        nmos0_inv = self.add_mos(ridx_coren, cur_col, nf_inv, w=wn_inv)
        pmos0_inv = self.add_mos(ridx_corep, cur_col, nf_inv, w=wp_inv)

        cur_col += nf_inv
        nf_cen = cur_col
        nmos1_inv = self.add_mos(ridx_coren, cur_col, nf_inv, w=wn_inv)
        pmos1_inv = self.add_mos(ridx_corep, cur_col, nf_inv, w=wp_inv)

        cur_col += nf_inv + min_sep
        nmos1_cp = self.add_mos(ridx_coren, cur_col, nf_cp, w=wn_fb)
        pmos1_cp = self.add_mos(ridx_corep, cur_col, nf_cp, w=wp_fb)

        # dummy
        if ndum_side > 0:
            cur_col += nf_cp
            n_dum_l = self.add_mos(ridx_coren, 0, ndum_side, w=wn_inv)
            p_dum_l = self.add_mos(ridx_corep, 0, ndum_side, w=wp_inv)
            n_dum_r = self.add_mos(ridx_coren, cur_col, ndum_side, w=wn_inv)
            p_dum_r = self.add_mos(ridx_corep, cur_col, ndum_side, w=wp_inv)
        else:
            n_dum_l, p_dum_l, n_dum_r, p_dum_r = None, None, None, None
        # tail rows
        cur_col = nf_cen - nfp_tail // 2
        tail_ps = 's' if nfp_tail % 4 == 0 else 'd'
        tail_ns = 's' if nfn_tail % 4 == 0 else 'd'
        pmos0_tail = self.add_mos(ridx_tailp, cur_col, nfp_tail, w=wp_tail, g_on_s=(tail_ps == 's'))

        cur_col = nf_cen - nfn_tail // 2
        nmos0_tail = self.add_mos(ridx_tailn, cur_col, nfn_tail, w=wn_tail, g_on_s=(tail_ns == 's'))
        self.set_mos_size()

        # --- routing --- #
        tr_manager = pinfo.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        w_clk2_vm = tr_manager.get_width(vm_layer, 'clk2')
        # connect drain
        inv0_d = self.connect_to_tracks([nmos0_inv.d, pmos0_inv.d,
                                         nmos1_cp.d, pmos1_cp.d], ninv_d_tid)
        inv1_d = self.connect_to_tracks([nmos1_inv.d, pmos1_inv.d,
                                         nmos0_cp.d, pmos0_cp.d], pinv_d_tid)

        # connect source
        tail_ps_net = pmos0_tail.s if (tail_ps == 's') else pmos0_tail.d
        tail_ns_net = nmos0_tail.s if (tail_ns == 's') else nmos0_tail.d
        inv01_ns = self.connect_to_tracks([nmos0_inv.s, nmos1_inv.s,
                                           tail_ns_net], ntail_d_tid)
        inv01_ps = self.connect_to_tracks([pmos0_inv.s, pmos1_inv.s,
                                           tail_ps_net], ptail_d_tid)

        # connect gate
        inv0_ng = self.connect_to_tracks([nmos0_inv.g], ninv_g_tid)
        inv0_pg = self.connect_to_tracks([pmos0_inv.g], pinv_g_tid)
        inv1_ng = self.connect_to_tracks([nmos1_inv.g], ninv_g_tid)
        inv1_pg = self.connect_to_tracks([pmos1_inv.g], pinv_g_tid)

        cp0_ng = self.connect_to_tracks([nmos0_cp.g], ninv_g_tid)
        cp0_pg = self.connect_to_tracks([pmos0_cp.g], pinv_g_tid)
        cp1_ng = self.connect_to_tracks([nmos1_cp.g], ninv_g_tid)
        cp1_pg = self.connect_to_tracks([pmos1_cp.g], pinv_g_tid)

        tail0_ng = self.connect_to_tracks(nmos0_tail.g, ntail_g_tid)
        tail0_pg = self.connect_to_tracks([pmos0_tail.g], ptail_g_tid)

        # connect to supply
        tail_vdd_net = pmos0_tail.d if (tail_ps == 's') else pmos0_tail.s
        tail_vss_net = nmos0_tail.d if (tail_ns == 's') else nmos0_tail.s
        vdd_conn_layer = [tail_vdd_net]
        vss_conn_layer = [tail_vss_net]
        if vdd_on_hm:
            vdd_tid = self.get_track_id(ridx_tailp, MOSWireType.G, wire_name='sup')
            vss_tid = self.get_track_id(ridx_tailn, MOSWireType.G, wire_name='sup')
            vss_hm = self.connect_to_tracks(vss_conn_layer, vss_tid)
            vdd_hm = self.connect_to_tracks(vdd_conn_layer, vdd_tid)
        else:
            vss_hm, vdd_hm = None, None

        # connect inv gates
        vm_ntr_shift, _ = tr_manager.place_wires(vm_layer, ['clk2'] * 2)
        vm_coord_shift = self.grid.track_to_coord(vm_layer, vm_ntr_shift)
        inp_tidx_vm = grid.coord_to_track(vm_layer, inv0_ng.middle + vm_coord_shift // 2 * in_track_offset)
        inp = self.connect_to_tracks([inv0_ng, inv0_pg], TrackID(vm_layer, inp_tidx_vm, w_clk2_vm))
        inn_tidx_vm = grid.coord_to_track(vm_layer, inv1_ng.middle - vm_coord_shift // 2 * in_track_offset)
        inn = self.connect_to_tracks([inv1_ng, inv1_pg], TrackID(vm_layer, inn_tidx_vm, w_clk2_vm))

        # connect between inv and cp
        von_tidx_vm = grid.coord_to_track(vm_layer, cp0_ng.middle)
        von = self.connect_to_tracks([cp0_ng, cp0_pg, inv0_d], TrackID(vm_layer, von_tidx_vm, w_clk2_vm))
        # von_x = self.connect_wires(inv0_d, lower=von.get_bbox_array(self.grid).left_unit,
        #                            upper=von.get_bbox_array(self.grid).right_unit, unit_mode=True)
        vop_tidx_vm = grid.coord_to_track(vm_layer, cp1_ng.middle)
        vop = self.connect_to_tracks([cp1_ng, cp1_pg, inv1_d], TrackID(vm_layer, vop_tidx_vm, w_clk2_vm))
        # vop_x = self.connect_wires(inv1_d, lower=vop.get_bbox_array(self.grid).left_unit,
        #                            upper=vop.get_bbox_array(self.grid).right_unit, unit_mode=True)

        # # draw dummies
        # ptap_wire_arrs, ntap_wire_arrs = self.fill_dummy()

        # # add pins
        self.add_pin('OUTP', [vop, inv1_d], show=show_pins)
        self.add_pin('OUTN', [von, inv0_d], show=show_pins)
        # self.add_pin(self.get_pin_name('OUTP'), [vop]+vop_x, show=show_pins)
        # self.add_pin(self.get_pin_name('OUTN'), [von]+von_x, show=show_pins)
        self.add_pin('INP', inp, show=show_pins)
        self.add_pin('INN', inn, show=show_pins)
        self.add_pin('CK', tail0_ng, show=show_pins)
        self.add_pin('CKB', tail0_pg, show=show_pins)

        # # export supplies
        vss_list, vdd_list = [nmos0_cp.s, nmos1_cp.s], [pmos0_cp.s, pmos1_cp.s]
        vss_list += [vss_hm] if vdd_on_hm else vss_conn_layer
        vdd_list += [vdd_hm] if vdd_on_hm else vdd_conn_layer
        if ndum_side > 0:
            for n_dum, p_dum in zip([n_dum_l, n_dum_r], [p_dum_l, p_dum_r]):
                vss_list += [n_dum.g, n_dum.d, n_dum.s]
                vdd_list += [p_dum.g, p_dum.d, p_dum.s]

        self.add_pin('VSS', vss_list, label='VSS:', show=show_pins)
        self.add_pin('VDD', vdd_list, label='VDD:', show=show_pins)
        self.add_pin('ns', inv01_ns, hide=True)
        self.add_pin('ps', inv01_ps, hide=True)

        # schmatic parameters
        lch = self.place_info.lch
        intent = self.place_info.get_row_place_info(ridx_coren).row_info.threshold
        dum_info = []
        if ndum_side > 0:
            dum_info += [(('nch', wn_inv, lch, intent, 'VSS', 'VSS'), ndum_side * 2),
                         (('pch', wp_inv, lch, intent, 'VDD', 'VDD'), ndum_side * 2)]

        self._sch_params = dict(
            lch=lch,
            w_dict=w_dict,
            th_dict=th_dict,
            seg_dict=seg_dict,
            dum_info=dum_info,
        )


class C2mosClkDivBuf(MOSBase, TemplateBaseZL):
    """A c2mos clock divider composed of 2 c2mos latches and buffers.
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._div_bbox = None

    @property
    def div_bbox(self):
        return self._div_bbox

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_c2mos_div')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            latch_params='buffer cmos cell parameters',
            buf_params='inv_chain parameters',
            tile0='Tile index of logic tile0',
            tile1='Tile index of logic tile1',
            tile_vss0='Tile index of ptap tile0',
            tile_vss1='Tile index of ptap tile1',
            tile_vdd='Tile index of ntap tile',
            show_pins='True to draw pins.',
            close_loop='',
            flip_stage=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            show_pins=False,
            tile0=1,
            tile1=3,
            tile_vss0=0,
            tile_vss1=4,
            tile_vdd=2,
            close_loop=True,
            flip_stage=False
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        tile0: int = self.params['tile0']
        tile1: int = self.params['tile1']
        tile_vss0: int = self.params['tile_vss0']
        tile_vss1: int = self.params['tile_vss1']
        tile_vdd: int = self.params['tile_vdd']
        latch_params: Param = self.params['latch_params']
        buf_params: Param = self.params['buf_params']
        show_pins: bool = self.params['show_pins']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        zm_layer = ym_layer + 1

        _pinfo_logic, _, _ = self.get_tile_info(tile0)

        # --- Placement --- #
        latch_master0 = self.new_template(C2mosLatch, params=dict(**latch_params, in_track_offset=-1, vdd_on_hm=False,
                                                                  pinfo=_pinfo_logic))
        latch_master1 = self.new_template(C2mosLatch, params=dict(**latch_params, in_track_offset=-2, vdd_on_hm=False,
                                                                  pinfo=_pinfo_logic))
        buf_master = self.new_template(InvChainRow4, params=dict(**buf_params, pinfo=_pinfo_logic))

        buf0_inst00 = self.add_tile(buf_master, tile0, buf_master.num_cols, flip_lr=True)
        buf0_inst10 = self.add_tile(buf_master, tile1, buf_master.num_cols, flip_lr=True)
        cur_col = buf_master.num_cols
        latch_inst0 = self.add_tile(latch_master0, tile0, cur_col)
        latch_inst1 = self.add_tile(latch_master1, tile1, cur_col)
        cur_col += latch_master0.num_cols
        buf0_inst01 = self.add_tile(buf_master, tile0, cur_col)
        buf0_inst11 = self.add_tile(buf_master, tile1, cur_col)

        ncol_tot = self.num_cols
        latch_toplay = latch_master0.top_layer

        sub_vss_bot_both = self.add_substrate_contact(0, 0, tile_idx=tile_vss0, seg=ncol_tot,
                                                      port_mode=SubPortMode.BOTH)
        sub_vdd_both = self.add_substrate_contact(0, 0, tile_idx=tile_vdd, seg=ncol_tot, port_mode=SubPortMode.BOTH)
        sub_vss_top_both = self.add_substrate_contact(0, 0, tile_idx=tile_vss1, seg=ncol_tot,
                                                      port_mode=SubPortMode.BOTH)

        sub_vss_bot = sub_vss_bot_both[1::2] if (latch_params['seg_dict']['ntail'] % 4 == 0) else sub_vss_bot_both[0::2]
        sub_vss_top = sub_vss_top_both[1::2] if (latch_params['seg_dict']['ntail'] % 4 == 0) else sub_vss_top_both[0::2]
        sub_vdd = sub_vdd_both[1::2] if latch_params['seg_dict']['ptail'] % 4 == 0 else sub_vdd_both[0::2]

        self.set_mos_size(self.num_cols, num_tiles=self.num_tile_rows)

        tr_manager = _pinfo_logic.tr_manager
        w_sup_hm = tr_manager.get_width(hm_layer, 'sup')

        # --- Routing --- #
        # connect vip/vin
        vip0 = latch_inst0.get_all_port_pins('INP')[0]
        vin0 = latch_inst0.get_all_port_pins('INN')[0]
        vip1 = latch_inst1.get_all_port_pins('INP')[0]
        vin1 = latch_inst1.get_all_port_pins('INN')[0]

        vop0 = latch_inst0.get_all_port_pins('OUTP', layer=hm_layer)[0]  # -- out2
        von0 = latch_inst0.get_all_port_pins('OUTN', layer=hm_layer)[0]  # -- out0
        vop1 = latch_inst1.get_all_port_pins('OUTP', layer=hm_layer)[0]  # -- out1
        von1 = latch_inst1.get_all_port_pins('OUTN', layer=hm_layer)[0]  # -- out3

        if self.params['close_loop']:
            self.connect_to_track_wires(vop0, vip1)  # -- out2
            self.connect_to_track_wires(von0, vin1)  # -- out0
            self.connect_to_track_wires(vop1, vin0)  # -- out1
            self.connect_to_track_wires(von1, vip0)  # -- out3
            von1 = self.extend_wires(vip0, lower=0, upper=self.bound_box.yh)
            vop1 = self.extend_wires(vin0, lower=0, upper=self.bound_box.yh)
            von0 = self.extend_wires(vin1, lower=0, upper=self.bound_box.yh)
            vop0 = self.extend_wires(vip1, lower=0, upper=self.bound_box.yh)
        elif not self.params['flip_stage']:
            self.connect_to_track_wires(vop0, vip1)  # -- out2
            self.connect_to_track_wires(von0, vin1)  # -- out0
            vop1, von1 = self.connect_differential_tracks(vop1, von1, vm_layer, vip0.track_id.base_index,
                                                          vin0.track_id.base_index, width=vip0.track_id.width)
            von0 = self.extend_wires(vin1, lower=0, upper=self.bound_box.yh)
            vop0 = self.extend_wires(vip1, lower=0, upper=self.bound_box.yh)
            self.add_pin('inp', vip0)
            self.add_pin('inn', vin0)
        else:
            self.connect_to_track_wires(vop1, vip0)  # -- out2
            self.connect_to_track_wires(von1, vin0)  # -- out0
            vop0, von0 = self.connect_differential_tracks(vop0, von0, vm_layer, vip1.track_id.base_index,
                                                          vin1.track_id.base_index, width=vip1.track_id.width)
            von1 = self.extend_wires(vin0, lower=0, upper=self.bound_box.yh)
            vop1 = self.extend_wires(vip0, lower=0, upper=self.bound_box.yh)
            self.add_pin('inp', vip1)
            self.add_pin('inn', vin1)

        mid2 = latch_inst0.get_pin('OUTP', layer=hm_layer)
        mid0 = latch_inst0.get_pin('OUTN', layer=hm_layer)
        mid3 = latch_inst1.get_pin('OUTP', layer=hm_layer)
        mid1 = latch_inst1.get_pin('OUTN', layer=hm_layer)

        # midn0=von0 -- buf0_01/ buf1_01
        self.connect_to_track_wires(buf0_inst00.get_all_port_pins('in_vm'), mid0)
        # midp1=vop1 -- buf0_10/ buf1_10
        self.connect_to_track_wires(buf0_inst11.get_all_port_pins('in_vm'), mid3)
        # midp0=vop0 -- buf0_00/ buf1_00
        self.connect_to_track_wires(buf0_inst01.get_all_port_pins('in_vm'), mid2)
        # midn1=von1 -- buf0_11/ buf1_11
        self.connect_to_track_wires(buf0_inst10.get_all_port_pins('in_vm'), mid1)

        # Connect clock signals
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
        clk_vm_mid_tidx = self.grid.coord_to_track(vm_layer, self.bound_box.w//2, RoundMode.NEAREST)
        clk_vm_locs = [tr_manager.get_next_track(vm_layer, clk_vm_mid_tidx, 'clk', 'clk', up=-2),
                       tr_manager.get_next_track(vm_layer, clk_vm_mid_tidx, 'clk', 'clk', up=-1),
                       tr_manager.get_next_track(vm_layer, clk_vm_mid_tidx, 'clk', 'clk', up=1),
                       tr_manager.get_next_track(vm_layer, clk_vm_mid_tidx, 'clk', 'clk', up=2)]
        clkp_hm = [latch_inst0.get_pin('CK'), latch_inst1.get_pin('CKB')]
        clkn_hm = [latch_inst0.get_pin('CKB'), latch_inst1.get_pin('CK')]
        clkp_vm = [self.connect_to_tracks(clkp_hm, TrackID(vm_layer, clk_vm_locs[0], tr_w_clk_vm),
                                          track_lower=self.bound_box.yl, track_upper=self.bound_box.yh),
                   self.connect_to_tracks(clkp_hm, TrackID(vm_layer, clk_vm_locs[-1], tr_w_clk_vm),
                                          track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)]
        clkn_vm = [self.connect_to_tracks(clkn_hm, TrackID(vm_layer, clk_vm_locs[1], tr_w_clk_vm),
                                          track_lower=self.bound_box.yl, track_upper=self.bound_box.yh),
                   self.connect_to_tracks(clkn_hm, TrackID(vm_layer, clk_vm_locs[-2], tr_w_clk_vm),
                                          track_lower=self.bound_box.yl, track_upper=self.bound_box.yh)]
        # VDD/ VSS
        vdd_tidx = self.get_track_index(0, MOSWireType.DS, wire_name='sup', wire_idx=-1, tile_idx=tile_vdd)
        vss_top_tidx = self.get_track_index(0, MOSWireType.DS, wire_name='sup', wire_idx=-1, tile_idx=tile_vss1)
        vss_bot_tidx = self.get_track_index(0, MOSWireType.DS, wire_name='sup', wire_idx=-1, tile_idx=tile_vss0)

        vdd_tid = TrackID(hm_layer, vdd_tidx, w_sup_hm)
        vss_top_tid = TrackID(hm_layer, vss_top_tidx, w_sup_hm)
        vss_bot_tid = TrackID(hm_layer, vss_bot_tidx, w_sup_hm)

        inst_vss_bot, inst_vss_top, inst_vdd = [], [], []
        for inst in [latch_inst0, latch_inst1, buf0_inst00, buf0_inst01, buf0_inst10, buf0_inst11]:
            if inst in [latch_inst0, buf0_inst00, buf0_inst01]:
                inst_vss_bot += inst.get_all_port_pins('VSS', layer=self.conn_layer)
            else:
                inst_vss_top += inst.get_all_port_pins('VSS', layer=self.conn_layer)
            inst_vdd += inst.get_all_port_pins('VDD', layer=self.conn_layer)

        self.connect_wires(inst_vss_bot + [sub_vss_bot_both])
        self.connect_wires(inst_vss_top + [sub_vss_top_both])
        self.connect_wires(inst_vdd + [sub_vdd_both])

        vss_top = self.connect_to_tracks([sub_vss_top], vss_top_tid, min_len_mode=MinLenMode.MIDDLE)
        vss_bot = self.connect_to_tracks([sub_vss_bot], vss_bot_tid, min_len_mode=MinLenMode.MIDDLE)
        vdd = self.connect_to_tracks([sub_vdd], vdd_tid, min_len_mode=MinLenMode.MIDDLE)

        self.add_pin('VDD', vdd, connect=True)
        self.add_pin('VSS', [vss_bot, vss_top], connect=True)

        # power fill
        vdd_l_hm = self.add_wires(hm_layer, vdd.track_id.base_index, vdd.lower, vdd.middle, width=vdd.track_id.width)
        vdd_r_hm = self.add_wires(hm_layer, vdd.track_id.base_index, vdd.middle, vdd.upper, width=vdd.track_id.width)
        vdd_xm = self.export_tap_hm(tr_manager, vdd_l_hm, hm_layer, xm_layer) + \
                 self.export_tap_hm(tr_manager, vdd_r_hm, hm_layer, xm_layer, align_upper=True)
        vss_bot_l_hm = self.add_wires(hm_layer, vss_bot.track_id.base_index, vss_bot.lower, vss_bot.middle,
                                      width=vss_bot.track_id.width)
        vss_bot_r_hm = self.add_wires(hm_layer, vss_bot.track_id.base_index, vss_bot.middle,
                                      vss_bot.upper, width=vss_bot.track_id.width)
        vss_top_l_hm = self.add_wires(hm_layer, vss_top.track_id.base_index, vss_top.lower, vss_top.middle,
                                      width=vss_top.track_id.width)
        vss_top_r_hm = self.add_wires(hm_layer, vss_top.track_id.base_index, vss_top.middle, vss_top.upper,
                                      width=vss_top.track_id.width)
        vss_xm = self.export_tap_hm(tr_manager, vss_bot_l_hm, hm_layer, xm_layer)+\
                 self.export_tap_hm(tr_manager, vss_top_l_hm, hm_layer, xm_layer)+ \
                 self.export_tap_hm(tr_manager, vss_bot_r_hm, hm_layer, xm_layer, align_upper=True) + \
                 self.export_tap_hm(tr_manager, vss_top_r_hm, hm_layer, xm_layer, align_upper=True)
        vdd_xm = self.extend_wires(vdd_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm = self.extend_wires(vss_xm, lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.add_pin('VDD_xm', vdd_xm, label='VDD')
        self.add_pin('VSS_xm', vss_xm, label='VSS')

        # this is added later
        if self.params['flip_stage']:
            self.add_pin('midp<0>', vop1, show=show_pins)
            self.add_pin('midn<0>', von1, show=show_pins)
            self.add_pin('midp<1>', vop0, show=show_pins)
            self.add_pin('midn<1>', von0, show=show_pins)
            buf_list = [buf0_inst10, buf0_inst11, buf0_inst00, buf0_inst01]
            self.add_pin('clkp', clkn_vm)
            self.add_pin('clkn', clkp_vm)

        else:
            self.add_pin('midp<0>', vop0, show=show_pins)
            self.add_pin('midn<0>', von0, show=show_pins)
            self.add_pin('midp<1>', vop1, show=show_pins)
            self.add_pin('midn<1>', von1, show=show_pins)
            buf_list = [buf0_inst00, buf0_inst01, buf0_inst10, buf0_inst11]
            self.add_pin('clkp', clkp_vm)
            self.add_pin('clkn', clkn_vm)

        # export output
        for idx, inst in enumerate(buf_list):
            for pin in ['out_vm']:
                self.reexport(inst.get_port(pin), net_name=f'outp<{idx // 2}>' if idx & 1 else f'outn<{idx // 2}>',
                              connect=True)
        self.sch_params = dict(
            latch_params_list=latch_master0.sch_params,
            buf_params=buf_master.sch_params,
            num_stages=2,
            close_loop=self.params['close_loop'],
        )


class ClkGlobalDiv(MOSBase, TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_global')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            inv_diff_params='',
            div_params0='',
            div_params1='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def draw_layout(self) -> None:
        # Make templates
        inv_diff_params = self.params['inv_diff_params']
        div_params0 = self.params['div_params0']
        div_params1 = self.params['div_params1']
        inv_diff_gen_params = inv_diff_params.copy(append=dict(pinfo=self.params['pinfo']))
        div2_gen_params = div_params0.copy(append=dict(pinfo=self.params['pinfo']))
        div4_gen_params = div_params1.copy(append=dict(pinfo=self.params['pinfo'], close_loop=False))
        inv_diff_master = self.new_template(DiffInvCoupledChain, params=inv_diff_gen_params)
        div2_master = self.new_template(C2mosClkDivBuf, params=div2_gen_params)
        div4_0_master = self.new_template(C2mosClkDivBuf, params=div4_gen_params)
        div4_gen_params = div_params1.copy(append=dict(pinfo=self.params['pinfo'], close_loop=False, flip_stage=True))
        div4_1_master = self.new_template(C2mosClkDivBuf, params=div4_gen_params)

        dummy_pinfo = inv_diff_master.tile_table['ptap_tile']
        dummy_pinfo = TilePatternElement(dummy_pinfo)

        tile_ele = [inv_diff_master.get_tile_pattern_element(), dummy_pinfo, div2_master.get_tile_pattern_element(),
                    dummy_pinfo, dummy_pinfo, div4_0_master.get_tile_pattern_element()]
        self.draw_base((TilePattern(tile_ele), inv_diff_master.draw_base_info[1]))

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      self.params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        diff_ncols, inv_ncols = div4_0_master.num_cols, inv_diff_master.num_cols
        ncol_tot = 2 * div4_0_master.num_cols + 10 * min_sep
        cur_row = 0
        inv_diff = self.add_tile(inv_diff_master, tile_idx=0, col_idx=(ncol_tot - inv_ncols) // 2)
        cur_row += inv_diff_master.num_tile_rows + 1
        div2 = self.add_tile(div2_master, tile_idx=cur_row, col_idx=(ncol_tot - div2_master.num_cols) // 2)
        cur_row += div2_master.num_tile_rows + 2
        div4_0 = self.add_tile(div4_0_master, tile_idx=cur_row, col_idx=2*min_sep)
        div4_1 = self.add_tile(div4_1_master, tile_idx=cur_row, col_idx=diff_ncols + 8 * min_sep)
        self.set_mos_size(ncol_tot)

        tr_manager = self.tr_manager

        def get_tile_ymid(tile_idx):
            tile_pinfo = self.get_tile_info(tile_idx)
            return tile_pinfo[1] + tile_pinfo[0].height // 2

        def get_tile_ybot(tile_idx):
            tile_pinfo = self.get_tile_info(tile_idx)
            return tile_pinfo[1]

        # Connect inv diff to div2
        _, xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 2,
                                            center_coord=inv_diff.get_pin('outn').middle)
        inv_outp_xm, inv_outn_xm = \
            self.connect_matching_tracks([inv_diff.get_pin('outn'), inv_diff.get_pin('outp')],
                                         xm_layer, xm_locs, width=tr_manager.get_width(xm_layer, 'clk'))

        # export inv input
        clkp = self.connect_to_tracks(inv_diff.get_all_port_pins('inn'), inv_diff.get_pin('outn').track_id)
        clkn = self.connect_to_tracks(inv_diff.get_all_port_pins('inp'), inv_diff.get_pin('outp').track_id)

        clkp_stack_dict = self.via_stack_up(tr_manager, clkp, vm_layer, xm1_layer, 'clk')
        clkn_stack_dict = self.via_stack_up(tr_manager, clkn, vm_layer, xm1_layer, 'clk')

        div2_clk_lower = div2.get_all_port_pins('clkn')[0].lower
        div2_clk_height = div2.get_all_port_pins('clkn')[0].upper - div2_clk_lower

        _, xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 2, center_coord=div2_clk_lower+div2_clk_height//4)
        div_outn_xm0, div_outp_xm0 = \
            self.connect_matching_tracks([div2.get_all_port_pins('clkn'), div2.get_all_port_pins('clkp')],
                                         xm_layer, xm_locs, width=tr_manager.get_width(xm_layer, 'clk'))
        _, xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 2, center_coord=div2_clk_lower+3*div2_clk_height//4)
        div_outn_xm1, div_outp_xm1 = \
            self.connect_matching_tracks([div2.get_all_port_pins('clkn'), div2.get_all_port_pins('clkp')],
                                         xm_layer, xm_locs, width=tr_manager.get_width(xm_layer, 'clk'))

        ym_mid_tidx = self.grid.coord_to_track(ym_layer, self.bound_box.w//2, RoundMode.NEAREST)
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        clk_ym_sep = self.get_track_sep(ym_layer, tr_w_clk_ym, 1)

        clk_bufn_ym = self.connect_to_tracks([div_outn_xm0, div_outn_xm1, inv_outn_xm],
                                             TrackID(ym_layer, ym_mid_tidx + clk_ym_sep, tr_w_clk_ym, grid=self.grid))
        clk_bufp_ym = self.connect_to_tracks([div_outp_xm0, div_outp_xm1, inv_outp_xm],
                                             TrackID(ym_layer, ym_mid_tidx - clk_ym_sep, tr_w_clk_ym, grid=self.grid))
        self.match_warr_length([clk_bufn_ym, clk_bufp_ym])

        # Connect div2 to div4
        _, xm_locs = tr_manager.place_wires(xm_layer, ['clk'] * 2,
                                            center_coord=get_tile_ybot(inv_diff_master.num_tile_rows +
                                                                       div2_master.num_tile_rows + 2))
        inv_outp_xm, inv_outn_xm = \
            self.connect_matching_tracks([div2.get_pin('outp<1>'), div2.get_pin('outn<1>')],
                                         xm_layer, xm_locs, width=tr_manager.get_width(xm_layer, 'clk'))

        self.connect_differential_wires(div4_0.get_all_port_pins('clkn') + div4_1.get_all_port_pins('clkn'),
                                        div4_0.get_all_port_pins('clkp') + div4_1.get_all_port_pins('clkp'),
                                        inv_outn_xm, inv_outp_xm)

        # Connect div4
        _, xm_locs0 = tr_manager.place_wires(xm_layer, ['clk'] * 6, center_coord=get_tile_ymid(cur_row + 1))
        _, xm_locs1 = tr_manager.place_wires(xm_layer, ['clk'] * 6, center_coord=get_tile_ymid(cur_row + 3))
        div_internal_xm_bot = xm_locs0[2:4]
        div_internal_xm_top = xm_locs1[2:4]
        self.connect_matching_tracks([[div4_0.get_pin('midp<1>'), div4_1.get_pin('inp')],
                                      [div4_0.get_pin('midn<1>'), div4_1.get_pin('inn')]],
                                     xm_layer, div_internal_xm_top, width=tr_manager.get_width(xm_layer, 'clk'))
        self.connect_matching_tracks([[div4_1.get_pin('midn<1>'), div4_0.get_pin('inp')],
                                      [div4_1.get_pin('midp<1>'), div4_0.get_pin('inn')]],
                                     xm_layer, div_internal_xm_bot, width=tr_manager.get_width(xm_layer, 'clk'))
        xm_out_bot = self.connect_matching_tracks([div4_0.get_all_port_pins('outn<0>'), div4_0.get_all_port_pins('outp<0>'),
                                                   div4_1.get_all_port_pins('outn<1>'), div4_1.get_all_port_pins('outp<1>')],
                                                  xm_layer, xm_locs0[:2] + xm_locs0[4:],
                                                  width=tr_manager.get_width(xm_layer, 'clk'))
        xm_out_top = self.connect_matching_tracks([div4_0.get_all_port_pins('outn<1>'), div4_0.get_all_port_pins('outp<1>'),
                                                   div4_1.get_all_port_pins('outn<0>'), div4_1.get_all_port_pins('outp<0>')],
                                                  xm_layer, xm_locs1[:2] + xm_locs1[4:],
                                                  width=tr_manager.get_width(xm_layer, 'clk'))

        # power fill
        vdd_hm_inv = inv_diff.get_all_port_pins('VDD')
        vss_hm_inv = inv_diff.get_all_port_pins('VSS')
        vdd_xm_list, vss_xm_list =[], []
        for _vdd in vdd_hm_inv:
            try:
                vdd_xm_list.extend(self.export_tap_hm(tr_manager, _vdd, hm_layer, xm_layer))
            except:
                pass
        for _vss in vss_hm_inv:
            try:
                vss_xm_list.extend(self.export_tap_hm(tr_manager, _vss, hm_layer, xm_layer))
            except:
                pass

        [vdd_xm_list.extend(inst.get_all_port_pins('VDD_xm')) for inst in [div4_0, div4_1, div2]]
        [vss_xm_list.extend(inst.get_all_port_pins('VSS_xm')) for inst in [div4_0, div4_1, div2]]
        vdd_xm_list = self.extend_wires(vdd_xm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm_list = self.extend_wires(vss_xm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        #
        xm_out = xm_out_bot[:2] + xm_out_top + xm_out_bot[2:]
        odd_idx, even_idx = 0, 0
        _, ym1_clk_locs = tr_manager.place_wires(ym1_layer, ['clk']*8, center_coord=self.bound_box.w//2)
        tr_w_ym1_clk = tr_manager.get_width(ym1_layer, 'clk')
        ym1_out_list, xm1_out_list = [], []
        for idx in range(4):
            xm_out0 = xm_out[2*idx]
            xm_out1 = xm_out[2*idx+1]
            # _, ym_locs = tr_manager.place_wires(ym_layer, ['clk']*2, center_coord=xm_out0.middle)
            ym_mid_tidx = self.grid.coord_to_track(ym_layer, xm_out0.middle, RoundMode.NEAREST)
            ym_locs = [ym_mid_tidx - clk_ym_sep, ym_mid_tidx + clk_ym_sep]
            # ym_out0, ym_out1 = self.connect_matching_tracks([xm_out0, xm_out1], ym_layer, ym_locs,
            #                                                 width = tr_manager.get_width(ym_layer, 'clk'))
            ym_out0 = self.connect_to_tracks(xm_out0, TrackID(ym_layer, ym_locs[0], tr_w_clk_ym, grid=self.grid))
            ym_out1 = self.connect_to_tracks(xm_out1, TrackID(ym_layer, ym_locs[1], tr_w_clk_ym, grid=self.grid))
            ym_out0, ym_out1 = self.match_warr_length([ym_out0, ym_out1])
            _, xm1_locs = tr_manager.place_wires(xm1_layer, ['clk']*2, center_coord=ym_out0.middle)
            xm1_out0, xm1_out1 = self.connect_matching_tracks([ym_out0, ym_out1], xm1_layer, xm1_locs,
                                                            width=tr_manager.get_width(xm1_layer, 'clk'))
            [xm1_out0, xm1_out1] = self.extend_wires([xm1_out0, xm1_out1], lower=self.bound_box.xl,
                                                     upper=self.bound_box.xh)
            ym1_out0 = self.connect_to_tracks(xm1_out0, TrackID(ym1_layer, ym1_clk_locs[odd_idx + 4 if idx & 1 else even_idx], tr_w_ym1_clk),
                                              track_upper=self.bound_box.yh, track_lower=div4_1.bound_box.yl, ret_wire_list=xm1_out_list)
            ym1_out1 = self.connect_to_tracks(xm1_out1, TrackID(ym1_layer, ym1_clk_locs[odd_idx + 6 if idx & 1 else even_idx + 2], tr_w_ym1_clk),
                                              track_upper=self.bound_box.yh, track_lower=div4_1.bound_box.yl, ret_wire_list=xm1_out_list)
            ym1_out_list.extend([ym1_out0, ym1_out1])
            self.add_pin(f'out<{odd_idx+4}>' if idx & 1 else f'out<{even_idx}>', ym1_out0)
            self.add_pin(f'out<{odd_idx+6}>' if idx & 1 else f'out<{even_idx+2}>', ym1_out1)
            if idx & 1:
                odd_idx += 1
            else:
                even_idx += 1

        self.match_warr_length(ym1_out_list)
        self.match_warr_length(xm1_out_list)
        vdd_list, vss_list = vdd_xm_list, vss_xm_list
        for idx in range(xm_layer, ym1_layer):
            sup_w = tr_manager.get_width(idx,'sup')
            sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
            sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
            sup_bbox_all_l = BBox(self.bound_box.xl, self.bound_box.yl,
                                  (self.bound_box.xl + self.bound_box.xh) // 2 - sup_w, self.bound_box.yh)
            sup_bbox_all_r = BBox((self.bound_box.xl + self.bound_box.xh) // 2 + sup_w, self.bound_box.yl,
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

        self.add_pin('VDD', vdd_list)
        self.add_pin('VSS', vss_list)

        self.add_pin('clkp', clkp_stack_dict[xm1_layer])
        self.add_pin('clkn', clkn_stack_dict[xm1_layer])
        self.sch_params = dict(
            inv_diff=inv_diff_master.sch_params,
            div2=div2_master.sch_params,
            div4=div4_1_master.sch_params,
        )


class ClkGlobalTop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'clk_global_top')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='',
            tr_spaces='',
            clk_rx_params='',
            clk_div_params='',
        )

    def draw_layout(self) -> None:
        # setup templates
        clk_rx_params: Param = self.params['clk_rx_params']
        clk_div_params: Param = self.params['clk_div_params']

        if isinstance(clk_rx_params, str):
            spec_yaml = read_yaml(clk_rx_params)
            clk_rx_params = spec_yaml['params']
        if isinstance(clk_div_params, str):
            spec_yaml = read_yaml(clk_div_params)
            clk_div_params = spec_yaml['params']
            clk_div_params['export_private'] = False

        clk_div_master = self.new_template(IntegrationWrapper, params=clk_div_params)
        clk_rx_master = self.new_template(ClkRxBuf, params=clk_rx_params)

        # track setting
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        self._tr_manager = tr_manager

        conn_layer = \
            MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                             clk_div_params['params']['params']['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1
        xm2_layer = ym1_layer + 1

        # --- Placement --- #
        blk_w, blk_h = self.grid.get_block_size(10, half_blk_x=False, half_blk_y=False)
        div_w, div_h = clk_div_master.bound_box.w, clk_div_master.bound_box.h
        rx_w, rx_h = clk_rx_master.bound_box.w, clk_rx_master.bound_box.h

        tot_w = max(div_w, rx_w)
        rx_x = -(-(tot_w + rx_w) // 2 // blk_w) * blk_w
        div_x = -(-(tot_w + div_w) // 2 // blk_w) * blk_w

        tr_w_clk_xm2 = tr_manager.get_width(xm2_layer, 'clk')
        clk_xm2_sp_ntr = self.get_track_sep(xm2_layer, tr_w_clk_xm2, 1)*self.grid.get_track_pitch(xm2_layer)

        rx_div_gap = clk_xm2_sp_ntr.dbl_value*2

        rx = self.add_instance(clk_rx_master, xform=Transform(rx_x, 0, mode=Orientation.MY))
        div_y = -(-(rx.bound_box.yh+rx_div_gap) // blk_h) * blk_h
        div = self.add_instance(clk_div_master, xform=Transform(div_x, div_y, mode=Orientation.MY))

        top_layer = max(clk_div_master.top_layer, clk_rx_master.top_layer, ym1_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        tot_w = max(rx.bound_box.xh, div.bound_box.xh)
        tot_w = -(-tot_w // blk_w) * blk_w
        tot_h = -(-div.bound_box.yh // blk_h) * blk_h

        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))

        mid_xm_tidx = self.grid.coord_to_track(xm_layer, (rx.bound_box.yh+div.bound_box.yl)//2, RoundMode.NEAREST)
        tr_w_xm = tr_manager.get_width(xm_layer, 'clk')
        midn = self.connect_to_tracks(rx.get_all_port_pins('outn'), TrackID(xm_layer, mid_xm_tidx, tr_w_xm))
        midp = self.connect_to_tracks(rx.get_all_port_pins('outp'), TrackID(xm_layer, mid_xm_tidx, tr_w_xm))

        midn_xm = self.connect_wires(midn)
        midp_xm = self.connect_wires(midp)

        tr_w_vm = tr_manager.get_width(vm_layer, 'clk')
        div_inn_vm_tidx = self.grid.coord_to_track(vm_layer, div.get_pin('clkn').lower, RoundMode.LESS)
        div_inp_vm_tidx = self.grid.coord_to_track(vm_layer, div.get_pin('clkp').upper, RoundMode.GREATER)

        self.connect_to_track_wires(midn_xm, div.get_pin('clkn'))
        self.connect_to_track_wires(midp_xm, div.get_pin('clkp'))

        tr_w_ym = tr_manager.get_width(ym_layer, 'clk')
        midn_ym_tidx = self.grid.coord_to_track(ym_layer, midn_xm[0].middle, RoundMode.NEAREST)
        midp_ym_tidx = self.grid.coord_to_track(ym_layer, midp_xm[0].middle, RoundMode.NEAREST)
        midn_ym = self.connect_to_tracks(midn_xm, TrackID(ym_layer, midn_ym_tidx, tr_w_ym))
        midp_ym = self.connect_to_tracks(midp_xm, TrackID(ym_layer, midp_ym_tidx, tr_w_ym))
        tr_w_xm1 = tr_manager.get_width(xm1_layer, 'clk')
        tr_w_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        xm2_layer = ym1_layer + 1
        tr_w_xm2 = tr_manager.get_width(xm2_layer, 'clk')
        clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, midp_ym.middle, RoundMode.NEAREST)
        midn_xm1 = self.connect_to_tracks(midn_ym, TrackID(xm1_layer, clk_xm1_tidx, tr_w_xm1, grid=self.grid))
        midp_xm1 = self.connect_to_tracks(midp_ym, TrackID(xm1_layer, clk_xm1_tidx, tr_w_xm1, grid=self.grid))
        clk_ym1_tidx_n =self.grid.coord_to_track(ym1_layer, midn_xm1.middle, RoundMode.NEAREST)
        clk_ym1_tidx_p =self.grid.coord_to_track(ym1_layer, midp_xm1.middle, RoundMode.NEAREST)
        midn_ym1 = self.connect_to_tracks(midn_xm1, TrackID(ym1_layer, clk_ym1_tidx_n, tr_w_ym1, grid=self.grid))
        midp_ym1 = self.connect_to_tracks(midp_xm1, TrackID(ym1_layer, clk_ym1_tidx_p, tr_w_ym1, grid=self.grid))

        clk_xm2_tidx = self.grid.coord_to_track(xm2_layer, (rx.bound_box.yh+div.bound_box.yl)//2, RoundMode.NEAREST)
        clk_xm2_tidx_n = clk_xm2_tidx + self.get_track_sep(xm2_layer, tr_w_clk_xm2, 1)
        clk_xm2_tidx_p = clk_xm2_tidx - self.get_track_sep(xm2_layer, tr_w_clk_xm2, 1)

        ret_ym1_list = []
        midn_xm2 = self.connect_to_tracks(midn_ym1, TrackID(xm2_layer, clk_xm2_tidx_n, tr_w_xm2, grid=self.grid),
                                          ret_wire_list=ret_ym1_list)
        midp_xm2 = self.connect_to_tracks(midp_ym1, TrackID(xm2_layer, clk_xm2_tidx_p, tr_w_xm2, grid=self.grid),
                                          ret_wire_list=ret_ym1_list)
        self.match_warr_length(ret_ym1_list)
        midn_xm2, midp_xm2 = self.match_warr_length([midn_xm2, midp_xm2])

        # midn_ym1 =

        vdd_list, vss_list = div.get_all_port_pins('VDD', layer=xm2_layer+2),\
                             div.get_all_port_pins('VSS', layer=xm2_layer+2)
        for idx in range(xm2_layer+2, 11):
            sup_w = tr_manager.get_width(idx,'sup')
            sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
            sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
            sup_bbox_all_l = BBox(self.bound_box.xl - 2*sup_w, div.bound_box.yl,
                                  (self.bound_box.xl + self.bound_box.xh) // 2, self.bound_box.yh)
            sup_bbox_all_r = BBox((self.bound_box.xl + self.bound_box.xh) // 2, div.bound_box.yl,
                                  self.bound_box.xh + 2*sup_w, self.bound_box.yh)
            idx_dir = self.grid.get_direction(idx)
            if idx_dir == Orient2D.y:
                vdd_list, vss_list = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx,
                                                              div.bound_box)
            else:
                sup_l = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_l,
                                                 side_sup=False)
                sup_r = self.connect_supply_warr(tr_manager, [vdd_list, vss_list], idx, sup_bbox_all_r,
                                                 side_sup=False, align_upper=True)
                vdd_list = sup_l[0] + sup_r[0]
                vss_list = sup_l[1] + sup_r[1]

        self.add_pin('VDD', vdd_list)
        self.add_pin('VSS', vss_list)

        self.add_pin('clkn', midn_xm2)
        self.add_pin('clkp', midp_xm2)
        for pin in div.port_names_iter():
            if 'out' in pin:
                self.reexport(div.get_port(pin))

        self.reexport(rx.get_port('inn'))
        self.reexport(rx.get_port('inp'))
        self.sch_params = dict(
            rx=clk_rx_master.sch_params,
            div=clk_div_master.sch_params,
        )
        #
