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


from itertools import chain
from typing import Any, Dict, Type, Optional, Tuple
from typing import Mapping, Union

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.template import TemplateDB
from bag.util.immutable import ImmutableSortedDict
from bag.util.immutable import Param
from bag.util.math import HalfInt
from bag_vco_adc.layout.util.template import TemplateBaseZL
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from bag_vco_adc.layout.util.util import fill_conn_layer_intv, connect_hm_sup_ow
from bag_vco_adc.layout.vco.vco_ring_osc import RingOscUnit
from pybag.enum import RoundMode, MinLenMode
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase

""" 
Generators for StrongArm flops used in VCO-based ADC

- Because the VCO is arrayed vertically, all flops are 
designed to match VCO height (1 PMOS and 1 NMOS row)
- Schematic generators reuse preamp and dynamic latch in 
SAR comparator
"""


class PreAmpDigHalf(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            vertical_out='True to connect outputs to vm_layer.',
            vertical_sup='True to connect outputs to vm_layer.',
            sig_locs='Optional dictionary of user defined signal locations',
            equalizer='True to add equalizer to preamp',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            sig_locs={},
            ridx_n=0,
            ridx_p=-1,
            vertical_out=True,
            vertical_sup=True,
            equalizer=True
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        vertical_sup: bool = self.params['vertical_sup']
        equalizer: bool = self.params['equalizer']

        w_dict, th_dict = self._get_w_th_dict(ridx_n, ridx_p)

        seg_in = seg_dict['in']
        seg_tail = seg_dict['tail']
        seg_load = seg_dict['load']

        w_in = w_dict['in']
        w_tail = w_dict['tail']
        w_load = w_dict['load']

        if seg_in & 1 or (seg_tail % 4 != 0) or seg_load & 1:
            raise ValueError('in, tail, nfb, or pfb must have even number of segments')
        seg_tail = seg_tail // 2
        seg_in = seg_in
        seg_load = seg_load

        # placement
        m_tail = self.add_mos(ridx_n, 1, seg_tail, w=w_tail)
        m_in = self.add_mos(ridx_n, 1 + seg_tail, seg_in, w=w_in)
        m_load = self.add_mos(ridx_p, 0, seg_load, w=w_load, g_on_s=True)
        tail_conn = [m_tail.s, m_in.s]
        clk_conn = m_tail.g

        tail_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig')
        pclk_tid = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1)
        nout_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        pout_tid = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1)

        # routing
        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        clk_vm_w = self.tr_manager.get_width(vm_layer, 'clk')
        sig_vm_w = self.tr_manager.get_width(vm_layer, 'sig')
        supply_shield_tidx_h = tr_manager.get_next_track(hm_layer, pclk_tid.base_index, 'sig', 'sig', up=-1)
        supply_shield_tidx_l = tr_manager.get_next_track(hm_layer, self.get_track_index(0, MOSWireType.G, 'sig'),
                                                         'sig', 'sig', up=1)
        tr_w_hm = tr_manager.get_width(hm_layer, 'sig')
        supply_shield_tid = [TrackID(hm_layer, supply_shield_tidx_h, tr_w_hm),
                             TrackID(hm_layer, supply_shield_tidx_l, tr_w_hm)]

        # NOTE: force even number of columns to make sure VDD conn_layer wires are on even columns.
        ncol_tot = self.num_cols
        self.set_mos_size(num_cols=ncol_tot + (ncol_tot & 1))

        tail = self.connect_to_tracks(tail_conn, tail_tid)

        # -- Connect middle node --
        nclk = self.connect_to_tracks(clk_conn, pclk_tid)
        pclk = self.connect_to_tracks(m_load.g, pclk_tid)
        nout = self.connect_to_tracks(m_in.d, nout_tid, min_len_mode=MinLenMode.UPPER)
        pout = self.connect_to_tracks(m_load.d, pout_tid, min_len_mode=MinLenMode.UPPER)

        clk_vm_tidx = self.arr_info.col_to_track(vm_layer, 0, mode=RoundMode.NEAREST)
        clk_vm_tidx = sig_locs.get('clk', clk_vm_tidx)
        clk_vm = self.connect_to_tracks([nclk, pclk], TrackID(vm_layer, clk_vm_tidx, width=clk_vm_w))

        out_vm_tidx = self.arr_info.col_to_track(vm_layer, seg_tail + seg_in // 2, mode=RoundMode.NEAREST)
        out_vm_tidx = sig_locs.get('out', out_vm_tidx)
        out_vm = self.connect_to_tracks([nout, pout], TrackID(vm_layer, out_vm_tidx, width=sig_vm_w))
        in_vm_tidx = tr_manager.get_next_track(vm_layer, out_vm_tidx, 'sig', 'sig', up=-1)
        self.add_wires(vm_layer, in_vm_tidx, width=sig_vm_w, lower=m_in.g.lower, upper=m_in.g.upper)

        # Shield output
        if vertical_sup:
            vdd = m_load.s
            vss = m_tail.d
        else:
            vdd = connect_hm_sup_ow(self, m_load.s, 0, ridx_p, tr_manager)
            vss = connect_hm_sup_ow(self, m_tail.d, 0, ridx_n, tr_manager)
            tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
            sup_vm_tidx = \
                self.get_available_tracks(vm_layer,
                                          self.arr_info.col_to_track(vm_layer, 0, RoundMode.NEAREST),
                                          self.arr_info.col_to_track(vm_layer, self.num_cols, RoundMode.NEAREST),
                                          lower=self.bound_box.yl, upper=self.bound_box.yh,
                                          width=tr_w_sup_vm, sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')))
            vdd_vm_list, vss_vm_list = [], []
            for tidx in sup_vm_tidx:
                vdd_vm_list.append(
                    self.connect_to_tracks(vdd, TrackID(vm_layer, tidx, tr_w_sup_vm), min_len_mode=MinLenMode.MIDDLE))
                vss_vm_list.append(
                    self.connect_to_tracks(vss, TrackID(vm_layer, tidx, tr_w_sup_vm), min_len_mode=MinLenMode.MIDDLE))
            self.add_pin('VDD_vm', vdd_vm_list, label='VDD')
            self.add_pin('VSS_vm', vss_vm_list, label='VSS')
        conn_layer_shiled_tidx = self.arr_info.col_to_track(conn_layer, self.num_cols, mode=RoundMode.NEAREST)
        conn_layer_shield = self.connect_to_tracks(vdd, TrackID(conn_layer, conn_layer_shiled_tidx))
        vdd_shield = [self.connect_to_tracks(conn_layer_shield, supply_shield_tid[0], track_lower=self.bound_box.xl),
                      self.connect_to_tracks(conn_layer_shield, supply_shield_tid[1], track_lower=self.bound_box.xl)]

        seg_eq = seg_dict['eq']
        w_eq = w_dict['eq']
        if equalizer:
            eq = self.add_mos(1, 1 + seg_tail, seg=seg_eq // 2, w=w_eq, tile_idx=0)
            self.connect_to_track_wires(eq.g, vdd_shield[0])
            self.add_pin('out_conn', eq.d)
            self.add_pin('outc_conn', eq.s)

        self.add_pin('clk_vm', clk_vm)
        self.add_pin('tail', tail)
        self.add_pin('clk', clk_vm)
        self.add_pin('in', m_in.g)
        self.add_pin('pout', pout)
        self.add_pin('nout', nout)
        self.add_pin('out', out_vm)

        self.add_pin('VSS', vss)
        self.add_pin('VDD', vdd, connect=True)
        self.add_pin('vdd_shield', vdd_shield, connect=True)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            has_cas=False,
            equalizer=equalizer
        )

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int, ) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']

        w_ans = {}
        th_ans = {}
        name_row_pair = [('tail', ridx_n), ('in', ridx_n), ('load', ridx_p)]
        if self.params['equalizer']:
            name_row_pair.append(['eq', ridx_n + 1])
        for name, row_idx in name_row_pair:
            rinfo = self.get_row_info(row_idx, 0)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)


class PreAmpDig(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_preamp')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = PreAmpDigHalf.get_params_info()
        ans['even_center'] = 'True to force center column to be even.'
        ans['flip_preamp_io'] = 'True to flip preamp input output, easy to connect to cnter'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = PreAmpDigHalf.get_default_param_values()
        ans['even_center'] = False
        ans['flip_preamp_io'] = False
        return ans

    def draw_layout(self):
        master: PreAmpDigHalf = self.new_template(PreAmpDigHalf, params=self.params)
        self.draw_base(master.draw_base_info)

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        # placement
        nsep = 0
        nhalf = master.num_cols
        corel = self.add_tile(master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(master, 0, nhalf + nsep)
        self.set_mos_size(num_cols=nsep + 2 * nhalf)

        ridx_n: int = self.params['ridx_n']
        equalizer = self.params['equalizer']
        # Routing
        # -- Get track index --
        inn_tidx, hm_w = self.get_track_info(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        inp_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        out_base_index = corel.get_all_port_pins('vdd_shield')[1]
        _, [_, outn_tidx, outp_tidx] = tr_manager.place_wires(hm_layer, ['sig', 'sig', 'sig'], align_idx=0,
                                                              align_track=out_base_index.track_id.base_index)
        if self.params['flip_preamp_io']:
            inn_tidx, inp_tidx, outn_tidx, outp_tidx = outn_tidx, outp_tidx, inn_tidx, inp_tidx

        inp_hm = self.connect_to_tracks(corel.get_pin('in'), TrackID(hm_layer, inp_tidx, hm_w),
                                        min_len_mode=MinLenMode.MIDDLE)
        inn_hm = self.connect_to_tracks(corer.get_pin('in'), TrackID(hm_layer, inn_tidx, hm_w),
                                        min_len_mode=MinLenMode.MIDDLE)
        outp, outn = self.connect_differential_tracks(corer.get_pin('out'), corel.get_pin('out'),
                                                      hm_layer, outp_tidx, outn_tidx, width=hm_w)
        if equalizer:
            self.connect_differential_wires([corel.get_pin('out_conn'), corer.get_pin('outc_conn')],
                                            [corel.get_pin('outc_conn'), corer.get_pin('out_conn')], outp, outn)
        inp_vm_tidx = tr_manager.get_next_track(vm_layer, corel.get_pin('out').track_id.base_index,
                                                'sig', 'sig', up=True)
        inn_vm_tidx = tr_manager.get_next_track(vm_layer, corer.get_pin('out').track_id.base_index,
                                                'sig', 'sig', up=False)

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        inp_vm = self.connect_to_tracks(inp_hm, TrackID(vm_layer, inp_vm_tidx, tr_w_sig_vm),
                                        min_len_mode=MinLenMode.MIDDLE)
        inn_vm = self.connect_to_tracks(inn_hm, TrackID(vm_layer, inn_vm_tidx, tr_w_sig_vm),
                                        min_len_mode=MinLenMode.MIDDLE)

        self.connect_wires([corel.get_pin('tail'), corer.get_pin('tail')])

        self.add_pin('inp', inp_vm)
        self.add_pin('inn', inn_vm)

        self.add_pin('outp', outp)
        self.add_pin('outn', outn)

        self.add_pin('VDD', self.connect_wires([corel.get_pin('VDD'), corer.get_pin('VDD')]))
        self.add_pin('VSS', self.connect_wires([corel.get_pin('VSS'), corer.get_pin('VSS')]))
        self.add_pin('VDD_vm', corel.get_all_port_pins('VDD_vm') + corer.get_all_port_pins('VDD_vm'))
        self.add_pin('VSS_vm', corel.get_all_port_pins('VSS_vm') + corer.get_all_port_pins('VSS_vm'))
        self.reexport(corel.get_port('clk'))
        self.sch_params = master.sch_params


class DynLatchDigHalf(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            sig_locs='Optional dictionary of user defined signal locations',
            flip_np='True to flip nmos and pmos',
            has_rst='True to add reset devices and connect tail to output of previous stage',
            vertical_sup='True to connect outputs to vm_layer.',
            vertical_out='True to connect outputs to vm_layer.',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_n=0,
            ridx_p=-1,
            sig_locs={},
            has_rst=False,
            flip_np=False,
            vertical_out=True,
            vertical_sup=True,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        flip_np: bool = self.params['flip_np']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']

        ridx_tail = ridx_p if flip_np else ridx_n
        ridx_nfb = ridx_n
        ridx_pfb = ridx_p
        ridx_in = ridx_n if flip_np else ridx_p

        w_dict, th_dict = self._get_w_th_dict(ridx_tail, ridx_nfb, ridx_pfb, ridx_in)
        seg_in = seg_dict['in']
        seg_nfb = seg_dict['nfb']
        seg_pfb = seg_dict['pfb']
        seg_tail = seg_dict['tail']
        w_in = w_dict['in']
        w_tail = w_dict['tail']
        w_nfb = w_dict['nfb']
        w_pfb = w_dict['pfb']

        if seg_in & 1 or (seg_tail % 2 != 0) or seg_nfb & 1 or seg_pfb & 1:
            raise ValueError('in, tail, nfb, or pfb must have even number of segments')
        seg_tail = seg_tail

        # placement
        m_nfb = self.add_mos(ridx_nfb, 0 if flip_np else 0, seg_nfb, g_on_s=not flip_np, w=w_nfb)
        m_pfb = self.add_mos(ridx_pfb, 0 if flip_np else 0, seg_pfb, g_on_s=not flip_np, w=w_pfb)
        m_tail = self.add_mos(ridx_tail, seg_pfb if flip_np else seg_nfb + 1, seg_tail, w=w_tail)
        m_in = self.add_mos(ridx_in, seg_nfb if flip_np else seg_pfb, seg_in, g_on_s=not flip_np, w=w_in)

        # Routing
        # - Signal locations
        nout_tid = self.get_track_id(ridx_nfb, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pout_tid = self.get_track_id(ridx_pfb, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        clk_tid = self.get_track_id(ridx_tail, MOSWireType.G, wire_name='sig', wire_idx=-1 if flip_np else 0)
        tail_tid = self.get_track_id(ridx_pfb if flip_np else ridx_nfb, MOSWireType.DS, wire_name='sig',
                                     wire_idx=0 if flip_np else -1)
        clk_conn = m_tail.g

        # NOTE: force even number of columns to make sure VDD conn_layer wires are on even columns.
        ncol_tot = self.num_cols
        self.set_mos_size(num_cols=ncol_tot + (ncol_tot & 1))
        conn_layer = self.conn_layer
        vm_layer = conn_layer + 2
        sig_vm_w = self.tr_manager.get_width(vm_layer, 'sig')

        clk = self.connect_to_tracks(clk_conn, clk_tid)
        out = self.connect_wires([m_nfb.g, m_pfb.g])
        nout = self.connect_to_tracks([m_in.d, m_nfb.d] if flip_np else [m_nfb.d], nout_tid)
        pout = self.connect_to_tracks([m_pfb.d] if flip_np else [m_in.d, m_pfb.d], pout_tid,
                                      min_len_mode=MinLenMode.UPPER)
        tail = self.connect_to_tracks([m_tail.s, m_pfb.s] if flip_np else [m_tail.s, m_nfb.s], tail_tid)

        if vertical_out:
            vm_tidx = self.arr_info.col_to_track(vm_layer, 2, mode=RoundMode.NEAREST)
            vm_tidx = sig_locs.get('out', vm_tidx)
            out_vm = self.connect_to_tracks([nout, pout], TrackID(vm_layer, vm_tidx, width=sig_vm_w))
            self.add_pin('out_vm', out_vm)
        else:
            self.add_pin('pout', pout)
            self.add_pin('nout', nout)

        if flip_np:
            vdd_conn = m_tail.d
            vss_conn = [m_nfb.s, m_in.s]
        else:
            vdd_conn = [m_pfb.s, m_in.s]
            vss_conn = m_tail.d
        if vertical_sup:
            vss = vss_conn
            vdd = vdd_conn
        else:
            vdd = connect_hm_sup_ow(self, vdd_conn, 0, ridx_p, self.tr_manager)
            vss = connect_hm_sup_ow(self, vss_conn, 0, ridx_n, self.tr_manager)
            tr_manager = self.tr_manager
            # find available vm locs
            tr_w_vm = tr_manager.get_width(vm_layer, 'sup')
            sup_vm_tidx = \
                self.get_available_tracks(vm_layer,
                                          self.arr_info.col_to_track(vm_layer, 0, RoundMode.NEAREST),
                                          self.arr_info.col_to_track(vm_layer, self.num_cols - 1, RoundMode.NEAREST),
                                          lower=self.bound_box.yl, upper=self.bound_box.yh,
                                          width=tr_w_vm, sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')))
            vdd_vm_list, vss_vm_list = [], []
            for tidx in sup_vm_tidx:
                vdd_vm_list.append(
                    self.connect_to_tracks(vdd, TrackID(vm_layer, tidx, tr_w_vm), min_len_mode=MinLenMode.MIDDLE))
                vss_vm_list.append(
                    self.connect_to_tracks(vss, TrackID(vm_layer, tidx, tr_w_vm), min_len_mode=MinLenMode.MIDDLE))
            self.add_pin('VDD_vm', vdd_vm_list, label='VDD')
            self.add_pin('VSS_vm', vss_vm_list, label='VSS')

        self.add_pin('VSS', vss)
        self.add_pin('VDD', vdd)
        self.add_pin('tail', tail)
        self.add_pin('clk', clk)
        self.add_pin('in', m_in.g)
        self.add_pin('out', out)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            flip_np=flip_np,
        )

    def _get_w_th_dict(self, ridx_tail: int, ridx_nfb: int, ridx_pfb: int, ridx_in: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']
        has_rst: bool = self.params['has_rst']

        w_ans = {}
        th_ans = {}
        for name, row_idx in [('nfb', ridx_nfb), ('in', ridx_in), ('pfb', ridx_pfb), ('tail', ridx_tail)]:
            rinfo = self.get_row_info(row_idx, 0)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        if has_rst:
            rinfo = self.get_row_info(ridx_in, 0)
            w = w_dict.get('rst', 0)
            if w == 0:
                w = rinfo.width
            w_ans['rst'] = w
            th_ans['rst'] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)


class DynLatchDig(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_dyn_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = DynLatchDigHalf.get_params_info()
        ans['even_center'] = 'True to force center column to be even.'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = DynLatchDigHalf.get_default_param_values()
        ans['even_center'] = False
        return ans

    def draw_layout(self):
        master: DynLatchDigHalf = self.new_template(DynLatchDigHalf, params=self.params)
        self.draw_base(master.draw_base_info)
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        # Output vm next to input vm
        out_vm_tidx = self.arr_info.col_to_track(vm_layer, 2, mode=RoundMode.NEAREST)
        half_params = self.params.copy(append=dict(sig_locs={'out': out_vm_tidx}))
        master: DynLatchDigHalf = self.new_template(DynLatchDigHalf, params=half_params)

        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        flip_np: bool = self.params['flip_np']

        # placement
        nhalf = master.num_cols
        corel = self.add_tile(master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(master, 0, nhalf)
        self.set_mos_size(num_cols=2 * nhalf)

        # routing
        ridx_nfb = ridx_n
        ridx_pfb = ridx_p
        hm_w = tr_manager.get_width(hm_layer, 'sig')
        out_base_index = self.get_track_index(ridx_nfb if flip_np else ridx_pfb, MOSWireType.G, wire_name='sig',
                                              wire_idx=0 if flip_np else -1)
        _, [outp_tidx, outn_tidx] = tr_manager.place_wires(hm_layer, ['sig'] * 2, out_base_index,
                                                           align_idx=0 if flip_np else -1)

        hm_layer = self.conn_layer + 1
        outp, outn = self.connect_differential_tracks(corer.get_all_port_pins('out'),
                                                      corel.get_all_port_pins('out'),
                                                      hm_layer, outp_tidx, outn_tidx, width=hm_w)
        if vertical_out:
            outp_vm = corel.get_pin('out_vm')
            outn_vm = corer.get_pin('out_vm')
            self.connect_differential_wires(outp_vm, outn_vm, outp, outn)
            self.add_pin('outn', outp)
            self.add_pin('outp', outn)
            self.add_pin('outn', outp_vm)
            self.add_pin('outp', outn_vm)
        else:
            self.add_pin('outp', [corel.get_pin('pout'), corel.get_pin('nout'), outp], connect=True)
            self.add_pin('outn', [corer.get_pin('pout'), corer.get_pin('nout'), outn], connect=True)

        if vertical_sup:
            vss = list(chain(corel.get_all_port_pins('VSS', layer=self.conn_layer),
                             corer.get_all_port_pins('VSS', layer=self.conn_layer)))
            vdd = list(chain(corel.get_all_port_pins('VDD', layer=self.conn_layer),
                             corer.get_all_port_pins('VDD', layer=self.conn_layer)))
        else:
            vss = self.connect_wires(list(chain(corel.get_all_port_pins('VSS'), corer.get_all_port_pins('VSS'))))
            vdd = self.connect_wires(list(chain(corel.get_all_port_pins('VDD'), corer.get_all_port_pins('VDD'))))

        clk_vm_tidx = self.arr_info.col_to_track(vm_layer, self.num_cols // 2, mode=RoundMode.NEAREST)
        clk_vm = self.connect_to_tracks([corel.get_pin('clk'), corer.get_pin('clk')],
                                        TrackID(vm_layer, clk_vm_tidx, tr_manager.get_width(vm_layer, 'clk')))

        self.connect_wires([corel.get_pin('tail'), corer.get_pin('tail')])
        self.add_pin('VDD', vdd)
        self.add_pin('VSS', vss)
        self.add_pin('inp', corel.get_pin('in'))
        self.add_pin('inn', corer.get_pin('in'))
        self.add_pin('clk', clk_vm)
        self.add_pin('VDD_vm', corel.get_all_port_pins('VDD_vm') + corer.get_all_port_pins('VDD_vm'),
                     label='VDD')
        self.add_pin('VSS_vm', corel.get_all_port_pins('VSS_vm') + corer.get_all_port_pins('VSS_vm'),
                     label='VSS')

        self.sch_params = master.sch_params


class DynLatchSADigHalf(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            sig_locs='Optional dictionary of user defined signal locations',
            flip_np='True to flip nmos and pmos',
            has_rst='True to add reset devices and connect tail to output of previous stage',
            vertical_sup='True to connect outputs to vm_layer.',
            vertical_out='True to connect outputs to vm_layer.',
            self_time='True to have self-time design',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_n=0,
            ridx_p=-1,
            sig_locs={},
            has_rst=False,
            flip_np=False,
            vertical_out=True,
            vertical_sup=True,
            self_time=False
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        flip_np: bool = self.params['flip_np']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        self_time: bool = self.params['self_time']

        ridx_ck = ridx_n if flip_np else ridx_p
        ridx_nfb = ridx_n
        ridx_pfb = ridx_p
        ridx_in = ridx_p if flip_np else ridx_n

        w_dict, th_dict = self._get_w_th_dict(ridx_ck, ridx_nfb, ridx_pfb, ridx_in)
        seg_in = seg_dict['in']
        seg_nfb = seg_dict['nfb']
        seg_pfb = seg_dict['pfb']
        seg_ck = seg_dict['ck']
        w_in = w_dict['in']
        w_ck = w_dict['ck']
        w_nfb = w_dict['nfb']
        w_pfb = w_dict['pfb']

        if seg_in & 1 or (seg_ck % 2 != 0) or seg_nfb & 1 or seg_pfb & 1:
            raise ValueError('in, tail, nfb, or pfb must have even number of segments')

        # placement
        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        m_nfb = self.add_mos(ridx_nfb, 0, seg_nfb, g_on_s=not flip_np, w=w_nfb)
        m_pfb = self.add_mos(ridx_pfb, 0, seg_pfb, g_on_s=not flip_np, w=w_pfb)
        m_ck = self.add_mos(ridx_ck, seg_nfb if flip_np else seg_pfb, seg_ck, w=w_ck)
        m_in = self.add_mos(ridx_in, seg_pfb + min_sep if flip_np else seg_nfb + min_sep, seg_in,
                            g_on_s=flip_np, w=w_in)
        ncol_tot = self.num_cols
        self.set_mos_size(num_cols=ncol_tot + (ncol_tot & 1))

        # Routing
        # -- signal locations
        nout_tid = self.get_track_id(ridx_nfb, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pout_tid = self.get_track_id(ridx_pfb, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        in_tid = self.get_track_id(ridx_pfb, MOSWireType.G, wire_name='sig', wire_idx=-1)

        clk_tidx = self.get_track_index(ridx_ck, MOSWireType.G, wire_name='sig', wire_idx=-1 if flip_np else 0)
        clk_tidx = self.tr_manager.get_next_track(self.conn_layer + 1, clk_tidx, 'sig', 'sig', up=4)
        clk_tid = TrackID(self.conn_layer + 1, clk_tidx, self.tr_manager.get_width(self.conn_layer + 1, 'sig'))
        tail_tid = self.get_track_id(ridx_pfb if flip_np else ridx_nfb, MOSWireType.DS, wire_name='sig',
                                     wire_idx=0 if flip_np else -1)
        clk_conn = m_ck.g

        # NOTE: force even number of columns to make sure VDD conn_layer wires are on even columns.
        conn_layer = self.conn_layer
        vm_layer = conn_layer + 2
        sig_vm_w = self.tr_manager.get_width(vm_layer, 'sig')

        out = self.connect_wires([m_nfb.g, m_pfb.g])
        nout = self.connect_to_tracks([m_ck.d, m_nfb.d] if flip_np else [m_nfb.d], nout_tid)
        pout = self.connect_to_tracks([m_in.s] if flip_np else [m_in.d, m_pfb.d], pout_tid,
                                      min_len_mode=MinLenMode.UPPER)
        # Input connect to vm
        in_hm = self.connect_to_tracks(m_in.g, in_tid)
        in_vm_tidx = self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST)
        in_vm_tidx = sig_locs.get('in', in_vm_tidx)
        in_vm = self.connect_to_tracks(in_hm, TrackID(vm_layer, in_vm_tidx, sig_vm_w), min_len_mode=MinLenMode.MIDDLE)
        self.connect_to_tracks([m_in.d, m_pfb.d] if flip_np else [m_in.d, m_nfb.d], tail_tid)

        if vertical_out:
            vm_tidx = self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST)
            vm_tidx = sig_locs.get('out', vm_tidx)
            out_vm = self.connect_to_tracks([nout, pout], TrackID(vm_layer, vm_tidx, width=sig_vm_w))
            self.add_pin('out_vm', out_vm)
            if not self_time:
                clk = self.connect_to_tracks(clk_conn, clk_tid)
                clk_vm_tidx = self.grid.coord_to_track(vm_layer, self.bound_box.xl, RoundMode.LESS_EQ)
                clk_vm = self.connect_to_tracks(clk, TrackID(vm_layer, clk_vm_tidx, sig_vm_w),
                                                min_len_mode=MinLenMode.MIDDLE)
                self.add_pin('clk', clk_vm)
            else:
                self.add_pin('in_conn', clk_conn)
        else:
            if not self_time:
                clk = self.connect_to_tracks(clk_conn, clk_tid)
                self.add_pin('clk', clk)
            else:
                self.add_pin('in_conn', clk_conn)

            self.add_pin('pout', pout)
            self.add_pin('nout', nout)

        if flip_np:
            vdd_conn = m_pfb.s
            vss_conn = [m_nfb.s, m_ck.s]
        else:
            vdd_conn = [m_pfb.s, m_in.s]
            vss_conn = m_nfb.d
        if vertical_sup:
            vss = vss_conn
            vdd = vdd_conn
        else:
            vdd = connect_hm_sup_ow(self, vdd_conn, 0, ridx_p, self.tr_manager)
            vss = connect_hm_sup_ow(self, vss_conn, 0, ridx_n, self.tr_manager)
            tr_manager = self.tr_manager

            # find available vm locs
            tr_w_vm = tr_manager.get_width(vm_layer, 'sup')
            sup_vm_tidx = \
                self.get_available_tracks(vm_layer,
                                          self.arr_info.col_to_track(vm_layer, 0, RoundMode.NEAREST),
                                          self.arr_info.col_to_track(vm_layer, self.num_cols, RoundMode.NEAREST),
                                          lower=self.bound_box.yl, upper=self.bound_box.yh,
                                          width=tr_w_vm, sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')))
            vdd_vm_list, vss_vm_list = [], []
            for tidx in sup_vm_tidx:
                vdd_vm_list.append(
                    self.connect_to_tracks(vdd, TrackID(vm_layer, tidx, tr_w_vm), min_len_mode=MinLenMode.MIDDLE))
                vss_vm_list.append(
                    self.connect_to_tracks(vss, TrackID(vm_layer, tidx, tr_w_vm), min_len_mode=MinLenMode.MIDDLE))
            self.add_pin('VDD_vm', vdd_vm_list, label='VDD')
            self.add_pin('VSS_vm', vss_vm_list, label='VSS')

        self.add_pin('VSS', vss)
        self.add_pin('VDD', vdd)
        self.add_pin('in', in_vm)
        self.add_pin('out', out)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
            flip_np=flip_np,
            self_time=self_time,
        )

    def _get_w_th_dict(self, ridx_ck: int, ridx_nfb: int, ridx_pfb: int, ridx_in: int) \
            -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']
        has_rst: bool = self.params['has_rst']

        w_ans = {}
        th_ans = {}
        for name, row_idx in [('nfb', ridx_nfb), ('in', ridx_in), ('pfb', ridx_pfb), ('ck', ridx_ck)]:
            rinfo = self.get_row_info(row_idx, 0)
            w = w_dict.get(name, 0)
            if w == 0:
                w = rinfo.width
            w_ans[name] = w
            th_ans[name] = rinfo.threshold

        if has_rst:
            rinfo = self.get_row_info(ridx_in, 0)
            w = w_dict.get('rst', 0)
            if w == 0:
                w = rinfo.width
            w_ans['rst'] = w
            th_ans['rst'] = rinfo.threshold

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)


class DynLatchSADig(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'comp_dyn_latch_sa')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = DynLatchSADigHalf.get_params_info()
        ans['even_center'] = 'True to force center column to be even.'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = DynLatchSADigHalf.get_default_param_values()
        ans['even_center'] = False
        return ans

    def draw_layout(self):
        master: DynLatchSADigHalf = self.new_template(DynLatchSADigHalf, params=self.params)
        self.draw_base(master.draw_base_info)
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        out_vm_tidx = self.arr_info.col_to_track(vm_layer, 2, mode=RoundMode.NEAREST)
        in_vm_tidx = self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST)
        half_params = self.params.copy(append=dict(sig_locs={'out': out_vm_tidx, 'in': in_vm_tidx}))
        master: DynLatchSADigHalf = self.new_template(DynLatchSADigHalf, params=half_params)

        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        flip_np: bool = self.params['flip_np']

        # placement
        nhalf = master.num_cols
        corel = self.add_tile(master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(master, 0, nhalf)
        self.set_mos_size(num_cols=2 * nhalf)

        # routing
        ridx_nfb = ridx_n
        ridx_pfb = ridx_p
        inn_tidx, hm_w = \
            self.get_track_info(ridx_nfb if flip_np else ridx_pfb, MOSWireType.G, wire_name='sig', wire_idx=-1)
        out_base_index = self.get_track_index(ridx_nfb if flip_np else ridx_pfb, MOSWireType.G, wire_name='sig',
                                              wire_idx=0 if flip_np else -1)
        _, [outn_tidx, outp_tidx] = tr_manager.place_wires(hm_layer, ['sig'] * 2, out_base_index,
                                                           align_idx=0 if flip_np else -1)
        hm_layer = self.conn_layer + 1
        outp, outn = self.connect_differential_tracks(corer.get_all_port_pins('out'),
                                                      corel.get_all_port_pins('out'),
                                                      hm_layer, outp_tidx, outn_tidx, width=hm_w)
        if vertical_out:
            outp_vm = corel.get_pin('out_vm')
            outn_vm = corer.get_pin('out_vm')
            self.connect_differential_wires(outp_vm, outn_vm, outp, outn)
            # self.connect_to_track_wires(outp, outp_vm)
            # self.connect_to_track_wires(outn, outn_vm)
            self.add_pin('outp', outp)
            self.add_pin('outn', outn)
            self.add_pin('outp', outp_vm)
            self.add_pin('outn', outn_vm)
        else:
            self.add_pin('outp', [corel.get_pin('pout'), corel.get_pin('nout'), outp], connect=True)
            self.add_pin('outn', [corer.get_pin('pout'), corer.get_pin('nout'), outn], connect=True)

        if vertical_sup:
            vss = list(chain(corel.get_all_port_pins('VSS', layer=self.conn_layer),
                             corer.get_all_port_pins('VSS', layer=self.conn_layer)))
            vdd = list(chain(corel.get_all_port_pins('VDD', layer=self.conn_layer),
                             corer.get_all_port_pins('VDD', layer=self.conn_layer)))
        else:
            vss = self.connect_wires(list(chain(corel.get_all_port_pins('VSS'), corer.get_all_port_pins('VSS'))))
            vdd = self.connect_wires(list(chain(corel.get_all_port_pins('VDD'), corer.get_all_port_pins('VDD'))))

        self.add_pin('VDD', vdd)
        self.add_pin('VSS', vss)
        self.add_pin('inn', corel.get_pin('in'), connect=True)
        self.add_pin('inp', corer.get_pin('in'), connect=True)
        self.add_pin('VDD_vm', corel.get_all_port_pins('VDD_vm') + corer.get_all_port_pins('VDD_vm'),
                     label='VDD')
        self.add_pin('VSS_vm', corel.get_all_port_pins('VSS_vm') + corer.get_all_port_pins('VSS_vm'),
                     label='VSS')

        if self.params['self_time']:
            self.add_pin('inn_conn', corel.get_pin('in_conn'), connect=True)
            self.add_pin('inp_conn', corer.get_pin('in_conn'), connect=True)
        else:
            self.add_pin('clk', [corel.get_pin('clk'), corer.get_pin('clk')], connect=True)

        self.sch_params = master.sch_params


class CnterLatchHalf(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            sig_locs='Optional dictionary of user defined signal locations',
            vertical_sup='True to connect outputs to vm_layer.',
            vertical_out='True to connect outputs to vm_layer.',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_n=0,
            ridx_p=-1,
            sig_locs={},
            vertical_out=True,
            vertical_sup=True,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']

        tr_manager = self.tr_manager

        w_dict, th_dict = self._get_w_th_dict(ridx_n, ridx_p)
        seg_nin = seg_dict['nin']
        seg_pin = seg_dict['pin']
        seg_nfb = seg_dict['nfb']
        seg_pfb = seg_dict['pfb']
        seg_ptail = seg_dict['ptail']
        seg_ntail = seg_dict['ntail']
        w_nin = w_dict['nin']
        w_pin = w_dict['pin']
        w_ntail = w_dict['ntail']
        w_ptail = w_dict['ptail']
        w_nfb = w_dict['nfb']
        w_pfb = w_dict['pfb']

        if seg_nin & 1 or seg_pin & 1:
            # if seg_nin & 1 or seg_pin & 1 or (seg_ntail % 4 != 0) or (seg_ptail % 4 != 0):
            raise ValueError('in, tail, nfb, or pfb must have even number of segments')
        seg_ptail = seg_ptail // 2
        seg_ntail = seg_ntail // 2

        # placement
        min_sep = self.min_sep_col
        m_ntail = self.add_mos(ridx_n, 0, seg_ntail, w=w_ntail, g_on_s=bool(seg_ntail & 1))
        m_ptail = self.add_mos(ridx_p, 0, seg_ptail, w=w_ptail, g_on_s=bool(seg_ptail & 1))
        m_nin = self.add_mos(ridx_n, seg_ntail, seg_nin, w=w_nin)
        m_pin = self.add_mos(ridx_p, seg_ptail, seg_pin, w=w_pin)

        m_nfb = self.add_mos(ridx_n, seg_nin + seg_ntail + min_sep, seg_nfb, w=w_nfb, g_on_s=bool(seg_nfb & 1))
        m_pfb = self.add_mos(ridx_p, seg_pin + seg_ptail + min_sep, seg_pfb, w=w_pfb, g_on_s=bool(seg_pfb & 1))

        # NOTE: force even number of columns to make sure VDD conn_layer wires are on even columns.
        ncol_tot = self.num_cols
        self.set_mos_size(ncol_tot)

        # routing
        nout_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pout_tid = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        ntail_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        ptail_tid = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2)
        nclk_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        pclk_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)

        hm_layer = self.conn_layer + 1
        tr_w_hm_sig = tr_manager.get_width(hm_layer, 'sig')
        nclk_tidx = tr_manager.get_next_track(hm_layer, nclk_tidx, 'sig', 'sig', up=sig_locs.get('clk', 0))
        pclk_tidx = tr_manager.get_next_track(hm_layer, pclk_tidx, 'sig', 'sig', up=-sig_locs.get('clk', 0))
        nclk_tid = TrackID(hm_layer, nclk_tidx, tr_w_hm_sig)
        pclk_tid = TrackID(hm_layer, pclk_tidx, tr_w_hm_sig)
        in_tid = TrackID(hm_layer, (nclk_tidx + pclk_tidx).div2(), tr_w_hm_sig)
        conn_layer = self.conn_layer
        vm_layer = conn_layer + 2
        sig_vm_w = self.tr_manager.get_width(vm_layer, 'sig')

        nclk = self.connect_to_tracks([m_ntail.g], nclk_tid)
        pclk = self.connect_to_tracks([m_ptail.g], pclk_tid)
        out = self.connect_wires([m_nfb.g, m_pfb.g])
        nout = self.connect_to_tracks([m_nin.d, m_nfb.s], nout_tid)
        pout = self.connect_to_tracks([m_pin.d, m_pfb.s], pout_tid)
        ntail = self.connect_to_tracks([m_nin.s, m_ntail.d] if seg_ntail & 1 else [m_nin.s, m_ntail.s], ntail_tid)
        ptail = self.connect_to_tracks([m_pin.s, m_ptail.d] if seg_ptail & 1 else [m_pin.s, m_ptail.s], ptail_tid)
        fb_g = self.connect_wires([m_nfb.g, m_pfb.g])
        in_g = self.connect_wires([m_nin.g, m_pin.g])
        in_hm = self.connect_to_tracks(in_g, in_tid)

        vdd_conn = [m_pfb.d, m_ptail.s] if seg_ptail & 1 else [m_pfb.d, m_ptail.d]
        vss_conn = [m_nfb.d, m_ntail.s] if seg_ntail & 1 else [m_nfb.d, m_ntail.d]
        if vertical_out:
            vm_tidx = self.arr_info.col_to_track(vm_layer, 0, mode=RoundMode.NEAREST)
            vm_tidx = sig_locs.get('out', vm_tidx)
            out_vm = self.connect_to_tracks([nout, pout], TrackID(vm_layer, vm_tidx, width=sig_vm_w))
            vm_tidx = self.arr_info.col_to_track(vm_layer, 1, mode=RoundMode.NEAREST)
            vm_tidx = sig_locs.get('in', vm_tidx)
            in_vm = self.connect_to_tracks(in_hm, TrackID(vm_layer, vm_tidx, width=sig_vm_w))
            self.add_pin('in_hm', in_hm)
            self.add_pin('out_vm', out_vm)
            self.add_pin('in_vm', in_vm)
        else:
            self.add_pin('in', in_hm)
            self.add_pin('pout', pout)
            self.add_pin('nout', nout)

        if vertical_sup:
            tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
            vdd_hm = connect_hm_sup_ow(self, vdd_conn, 0, ridx_p, self.tr_manager)
            vss_hm = connect_hm_sup_ow(self, vss_conn, 0, ridx_n, self.tr_manager)
            # export to vm, find available tracks
            sup_vm_tidx = \
                self.get_available_tracks(vm_layer,
                                          self.arr_info.col_to_track(vm_layer, 0, RoundMode.NEAREST),
                                          self.arr_info.col_to_track(vm_layer, self.num_cols - 1, RoundMode.NEAREST),
                                          lower=self.bound_box.yl, upper=self.bound_box.yh,
                                          width=tr_w_sup_vm,
                                          sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')))
            vdd_vm_list, vss_vm_list = [], []
            for tidx in sup_vm_tidx:
                vdd_vm_list.append(self.connect_to_tracks(vdd_hm, TrackID(vm_layer, tidx, tr_w_sup_vm),
                                                          min_len_mode=MinLenMode.MIDDLE))
                vss_vm_list.append(self.connect_to_tracks(vss_hm, TrackID(vm_layer, tidx, tr_w_sup_vm),
                                                          min_len_mode=MinLenMode.MIDDLE))

            self.add_pin('VSS', vss_vm_list)
            self.add_pin('VDD', vdd_vm_list)
        else:
            vdd_hm = connect_hm_sup_ow(self, vdd_conn, 0, ridx_p, self.tr_manager)
            vss_hm = connect_hm_sup_ow(self, vss_conn, 0, ridx_n, self.tr_manager)

        self.add_pin('ntail', ntail)
        self.add_pin('ptail', ptail)

        # extend clk
        clk_ext_x = self.arr_info.col_to_coord(max(seg_ptail, seg_ntail) + max(seg_nin, seg_pin) // 2)
        nclk = self.extend_wires(nclk, upper=clk_ext_x)
        pclk = self.extend_wires(pclk, upper=clk_ext_x)

        self.add_pin('VDD_hm', vdd_hm)
        self.add_pin('VSS_hm', vss_hm)
        self.add_pin('nclk', nclk)
        self.add_pin('pclk', pclk)
        self.add_pin('out', out)
        self.add_pin('fb_in', fb_g)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            th_dict=th_dict,
        )

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


class CnterLatch(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = CnterLatchHalf.get_params_info()
        ans['even_center'] = 'True to force center column to be even.'
        ans['flip_io'] = 'True to flip input/output, easier for inter-connection'
        ans['vertical_clk'] = 'True to add vertical clock signals'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = CnterLatchHalf.get_default_param_values()
        ans['even_center'] = False
        ans['flip_io'] = False
        ans['vertical_clk'] = False
        return ans

    def draw_layout(self):
        master: CnterLatchHalf = self.new_template(CnterLatchHalf, params=self.params)
        self.draw_base(master.draw_base_info)

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        flip_io: bool = self.params['flip_io']
        vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        vertical_clk: bool = self.params['vertical_clk']
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']

        # placement
        nsep = 0
        nsep += nsep & 1
        nhalf = master.num_cols
        mid_sup_vm_tidx = self.arr_info.col_to_track(vm_layer, 0)
        out_vm_tidx = tr_manager.get_next_track(vm_layer, mid_sup_vm_tidx, 'sup', 'sig')
        in_vm_tidx = tr_manager.get_next_track(vm_layer, out_vm_tidx, 'sig', 'sig')
        sig_locs_new = {'out': sig_locs.get('out', out_vm_tidx), 'in': sig_locs.get('in', in_vm_tidx),
                        'sup': [mid_sup_vm_tidx]}

        # shift hm clk idx
        if flip_io:
            sig_locs_new['clk'] = 1
        master_params = self.params.copy(append=dict(sig_locs=sig_locs_new))
        master: CnterLatchHalf = self.new_template(CnterLatchHalf, params=master_params)
        corel = self.add_tile(master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(master, 0, nhalf + nsep)
        self.set_mos_size(num_cols=nsep + 2 * nhalf)

        # routing
        hm_w = tr_manager.get_width(hm_layer, 'sig')
        in_hm_tidx = corel.get_pin('in_hm').track_id.base_index
        outn_tidx = tr_manager.get_next_track(hm_layer, in_hm_tidx, 'sig', 'sig', up=1)
        outp_tidx = tr_manager.get_next_track(hm_layer, in_hm_tidx, 'sig', 'sig', up=-1)
        outp, outn = self.connect_differential_tracks(corer.get_all_port_pins('out'),
                                                      corel.get_all_port_pins('out'),
                                                      hm_layer, outp_tidx, outn_tidx, width=hm_w)
        if vertical_out:
            outp_vm = corel.get_pin('out_vm')
            outn_vm = corer.get_pin('out_vm')
            inp_vm = corel.get_pin('in_vm')
            inn_vm = corer.get_pin('in_vm')
            outp, outn = self.connect_differential_wires(outp_vm, outn_vm, outp, outn)

            inp_vm = self.extend_wires(inp_vm, upper=outp_vm.upper, lower=outp_vm.lower)
            inn_vm = self.extend_wires(inn_vm, upper=outp_vm.upper, lower=outp_vm.lower)

            self.add_pin('d', inp_vm)
            self.add_pin('dn', inn_vm)

            self.add_pin('outn', outp)
            self.add_pin('outp', outn)
            self.add_pin('outn', outp_vm)
            self.add_pin('outp', outn_vm)
        else:
            self.reexport(corel.get_port('in'), net_name='d')
            self.reexport(corer.get_port('in'), net_name='dn')
            self.add_pin('outp', [corel.get_pin('pout'), corel.get_pin('nout'), outp], connect=True)
            self.add_pin('outn', [corer.get_pin('pout'), corer.get_pin('nout'), outn], connect=True)

        if vertical_clk:
            outp_vm, outn_vm = corel.get_pin('out_vm'), corer.get_pin('out_vm')
            inp_vm, inn_vm = corel.get_pin('in_vm'), corer.get_pin('in_vm')
            vm_min_l = min(outp_vm.track_id.base_index, inp_vm.track_id.base_index)
            vm_max_r = max(outn_vm.track_id.base_index, inn_vm.track_id.base_index)
            clkn_vm_tidx = tr_manager.get_next_track(vm_layer, vm_min_l, 'sig', 'clk', up=-1)
            clkp_vm_tidx = tr_manager.get_next_track(vm_layer, vm_max_r, 'sig', 'clk', up=1)
            tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
            if flip_io:
                clkn_vm_tidx, clkp_vm_tidx = clkp_vm_tidx, clkn_vm_tidx
            clk_vm = self.connect_to_tracks([corel.get_pin('pclk'), corer.get_pin('pclk')],
                                            TrackID(vm_layer, clkp_vm_tidx, tr_w_clk_vm))
            nclk_vm = self.connect_to_tracks([corel.get_pin('nclk'), corer.get_pin('nclk')],
                                             TrackID(vm_layer, clkn_vm_tidx, tr_w_clk_vm))
            self.add_pin('clkn_vm', clk_vm)
            self.add_pin('clkp_vm', nclk_vm)
        self.add_pin('clkn', self.connect_wires([corel.get_pin('pclk'), corer.get_pin('pclk')]))
        self.add_pin('clkp', self.connect_wires([corel.get_pin('nclk'), corer.get_pin('nclk')]))

        self.connect_wires([corel.get_pin('ntail'), corer.get_pin('ntail')])
        self.connect_wires([corel.get_pin('ptail'), corer.get_pin('ptail')])

        self.add_pin('VDD_hm', [corel.get_pin('VDD_hm'), corer.get_pin('VDD_hm')], hide=True)
        self.add_pin('VSS_hm', [corel.get_pin('VSS_hm'), corer.get_pin('VSS_hm')], hide=True)

        if vertical_sup:
            vss = list(chain(corel.get_all_port_pins('VSS'), corer.get_all_port_pins('VSS')))
            vdd = list(chain(corel.get_all_port_pins('VDD'), corer.get_all_port_pins('VDD')))
            vdd_xm_tid = self.grid.coord_to_track(xm_layer, vdd[0].middle, mode=RoundMode.NEAREST)
            vss_xm_tid = self.grid.coord_to_track(xm_layer, vss[0].middle, mode=RoundMode.NEAREST)
            tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
            vdd_xm = self.connect_to_tracks(vdd, TrackID(xm_layer, vdd_xm_tid, tr_w_sup_xm))
            vss_xm = self.connect_to_tracks(vss, TrackID(xm_layer, vss_xm_tid, tr_w_sup_xm))
            self.add_pin('VDD', vdd_xm)
            self.add_pin('VSS', vss_xm)
        else:
            vss = self.connect_wires(list(chain(corel.get_all_port_pins('VSS'), corer.get_all_port_pins('VSS'))))
            vdd = self.connect_wires(list(chain(corel.get_all_port_pins('VDD'), corer.get_all_port_pins('VDD'))))
            self.add_pin('VDD', vdd)
            self.add_pin('VSS', vss)

        self.sch_params = master.sch_params


class SRLatchSymmetricHalf(MOSBase):
    """Half of symmetric SR latch
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        zero = HalfInt(0)
        self._q_tr_info = (0, zero, zero)
        self._sr_hm_tr_info = self._q_tr_info
        self._sr_vm_tr_info = self._q_tr_info

    @property
    def q_tr_info(self) -> Tuple[int, HalfInt, HalfInt]:
        return self._q_tr_info

    @property
    def sr_hm_tr_info(self) -> Tuple[int, HalfInt, HalfInt]:
        return self._sr_hm_tr_info

    @property
    def sr_vm_tr_info(self) -> Tuple[int, HalfInt, HalfInt]:
        return self._sr_vm_tr_info

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            ridx_n='bottom nmos row index.',
            ridx_p='pmos row index.',
            has_rstb='True to add rstb functionality.',
            has_outbuf='True to add output buffers.',
            has_inbuf='True to add input buffers.',
            out_pitch='output wire pitch from center.',
            sig_locs='Signal locations',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            ridx_n=0,
            ridx_p=-1,
            has_rstb=False,
            has_outbuf=True,
            has_inbuf=True,
            sig_locs={},
            out_pitch=0.5,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        seg_dict: ImmutableSortedDict[str, int] = self.params['seg_dict']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        has_rstb: bool = self.params['has_rstb']
        has_outbuf: bool = self.params['has_outbuf']
        has_inbuf: bool = self.params['has_inbuf']
        out_pitch: HalfInt = HalfInt.convert(self.params['out_pitch'])
        sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']

        w_dict, th_dict = self._get_w_th_dict(ridx_n, ridx_p, has_rstb)

        seg_fb = seg_dict['fb']
        seg_ps = seg_dict['ps']
        seg_nr = seg_dict['nr']
        seg_obuf = seg_dict['obuf'] if has_outbuf else 0
        seg_ibuf = seg_dict['ibuf'] if has_inbuf else 0

        w_pfb = w_dict['pfb']
        w_nfb = w_dict['nfb']
        w_ps = w_dict['ps']
        w_nr = w_dict['nr']
        w_rst = w_dict.get('pr', 0)
        w_nbuf = w_nr
        w_pbuf = w_ps

        sch_seg_dict = dict(nfb=seg_fb, pfb=seg_fb, ps=seg_ps, nr=seg_nr)
        if has_rstb:
            sch_seg_dict['pr'] = seg_rst = seg_dict['rst']
        else:
            seg_rst = 0

        if seg_ps & 1 or seg_nr & 1 or seg_rst & 1 or seg_obuf & 1:
            raise ValueError('ps, nr, rst, and buf must have even number of segments')

        # placement
        min_sep = self.min_sep_col
        min_sep += (min_sep & 1)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        grid = self.grid
        arr_info = self.arr_info
        tr_manager = self.tr_manager
        hm_w = tr_manager.get_width(hm_layer, 'sig')
        vm_w = tr_manager.get_width(vm_layer, 'sig')
        hm_sep_col = self.get_hm_sp_le_sep_col(ntr=hm_w)
        mid_sep = max(hm_sep_col - 2, 0)
        mid_sep = (mid_sep + 1) // 2

        if has_inbuf:
            m_nibuf = self.add_mos(ridx_n, mid_sep, seg_ibuf, w=w_nbuf)
            m_pibuf = self.add_mos(ridx_p, mid_sep, seg_ibuf, w=w_pbuf)
            cur_col = mid_sep + seg_ibuf
            nsr_list = [m_pibuf.g, m_nibuf.g]
        else:
            m_nibuf = m_pibuf = None
            cur_col = mid_sep
            nsr_list = []

        nr_col = cur_col
        m_nr = self.add_mos(ridx_n, cur_col, seg_nr, w=w_nr)
        m_ps = self.add_mos(ridx_p, cur_col, seg_ps, w=w_ps)
        nsr_list.append(m_nr.g)
        pcol = cur_col + seg_ps
        if has_rstb:
            m_rst = self.add_mos(ridx_p, pcol, seg_rst, w=w_rst)
            pcol += seg_rst
        else:
            m_rst = None

        cur_col = max(cur_col + seg_nr, pcol)
        if has_outbuf:
            m_pinv = self.add_mos(ridx_p, cur_col, seg_obuf, w=w_pbuf)
            m_ninv = self.add_mos(ridx_n, cur_col, seg_obuf, w=w_nbuf)
            cur_col += seg_obuf
        else:
            m_pinv = m_ninv = None

        cur_col += min_sep
        fb_col = cur_col
        m_pfb = self.add_mos(ridx_p, cur_col, seg_fb, w=w_pfb, g_on_s=True, stack=2, sep_g=True)
        m_nfb = self.add_mos(ridx_n, cur_col, seg_fb, w=w_nfb, g_on_s=True, stack=2, sep_g=True)
        self.set_mos_size()

        # track planning
        nbuf_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=-2)
        nq_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        pq_tid = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pbuf_tid = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)

        nsrb_tid = sig_locs.get('srb', self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0))
        nsr_tid = sig_locs.get('sr', self.get_track_id(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0))

        # try to spread out gate wires to lower parasitics on differential Q wires
        pg_lower = sig_locs.get('pg_l', self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=-1))
        pg_upper = sig_locs.get('pg_h', self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1))
        g_idx_list = tr_manager.spread_wires(hm_layer, ['sig', 'sig', 'sig', 'sig'], pg_lower, pg_upper,
                                             ('sig', 'sig'), alignment=-1)
        self._q_tr_info = (hm_w, g_idx_list[3], g_idx_list[0])
        self._sr_hm_tr_info = (hm_w, g_idx_list[2], g_idx_list[1])

        if has_rstb:
            rst_tid = self.get_track_id(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)
            pq_conn_list = [m_ps.d, m_rst.d, m_pfb.s]
            vdd_list = [m_ps.s, m_rst.s, m_pfb.d]
            vss_list = [m_nr.s, m_nfb.d]

            rstb = self.connect_to_tracks(m_rst.g, rst_tid, min_len_mode=MinLenMode.MIDDLE)
            rst_vm_tidx = grid.coord_to_track(vm_layer, rstb.middle, mode=RoundMode.GREATER_EQ)
            rstb_vm = self.connect_to_tracks(rstb, TrackID(vm_layer, rst_vm_tidx, width=vm_w),
                                             min_len_mode=MinLenMode.MIDDLE)
            self.add_pin('rstb', rstb_vm)
        else:
            pq_conn_list = [m_ps.d, m_pfb.s]
            vdd_list = [m_ps.s, m_pfb.d]
            vss_list = [m_nr.s, m_nfb.d]

        self.add_pin('psrb', m_ps.g)
        self.add_pin('psr', m_pfb.g[0::2])
        nq = self.connect_to_tracks([m_nr.d, m_nfb.s], nq_tid)
        pq = self.connect_to_tracks(pq_conn_list, pq_tid)
        nsr_ret_wire_list = []
        nsr = self.connect_to_tracks(nsr_list, nsr_tid, min_len_mode=MinLenMode.UPPER,
                                     ret_wire_list=nsr_ret_wire_list)
        nsrb = self.connect_to_tracks(m_nfb.g[0::2], nsrb_tid, min_len_mode=MinLenMode.LOWER)
        qb = self.connect_wires([m_nfb.g[1::2], m_pfb.g[1::2]])
        self.add_pin('qb', qb)

        if has_inbuf:
            vdd_list.append(m_pibuf.s)
            vss_list.append(m_nibuf.s)

            nbuf = self.connect_to_tracks(m_nibuf.d, nbuf_tid, min_len_mode=MinLenMode.UPPER)
            pbuf = self.connect_to_tracks(m_pibuf.d, pbuf_tid, min_len_mode=MinLenMode.UPPER)
            vm_tidx = grid.coord_to_track(vm_layer, nbuf.middle, mode=RoundMode.LESS_EQ)
            buf = self.connect_to_tracks([nbuf, pbuf], TrackID(vm_layer, vm_tidx, width=vm_w))
            self.add_pin('srb_buf', buf)

        out_p_htr = out_pitch.dbl_value
        vm_ref = grid.coord_to_track(vm_layer, 0)
        srb_vm_tidx = arr_info.col_to_track(vm_layer, nr_col + 1, mode=RoundMode.GREATER_EQ)
        if has_outbuf:
            vdd_list.append(m_pinv.s)
            vss_list.append(m_ninv.s)

            nbuf = self.connect_to_tracks(m_ninv.d, nbuf_tid, min_len_mode=MinLenMode.MIDDLE)
            pbuf = self.connect_to_tracks(m_pinv.d, pbuf_tid, min_len_mode=MinLenMode.MIDDLE)
            vm_delta = grid.coord_to_track(vm_layer, nbuf.middle, mode=RoundMode.LESS_EQ) - vm_ref
            vm_htr = -(-vm_delta.dbl_value // out_p_htr) * out_p_htr
            vm_tidx = vm_ref + HalfInt(vm_htr)
            buf = self.connect_to_tracks([nbuf, pbuf], TrackID(vm_layer, vm_tidx, width=vm_w))
            self.add_pin('buf_out', buf)
            buf_in = self.connect_wires([m_ninv.g, m_pinv.g])
            self.add_pin('buf_in', buf_in)

            q_vm_tidx = tr_manager.get_next_track(vm_layer, srb_vm_tidx, 'sig', 'sig')
        else:
            vm_delta = tr_manager.get_next_track(vm_layer, srb_vm_tidx, 'sig', 'sig') - vm_ref
            vm_htr = -(-vm_delta.dbl_value // out_p_htr) * out_p_htr
            q_vm_tidx = vm_ref + HalfInt(vm_htr)

        sr_vm_tidx = arr_info.col_to_track(vm_layer, fb_col, mode=RoundMode.LESS_EQ)
        self._sr_vm_tr_info = (vm_w, sr_vm_tidx, srb_vm_tidx)

        q_vm = self.connect_to_tracks([nq, pq], TrackID(vm_layer, q_vm_tidx, width=vm_w))
        self.add_pin('q_vm', q_vm)
        self.add_pin('nsr', nsr)
        self.add_pin('nsrb', nsrb)
        self.add_pin('nsr_conn', nsr_ret_wire_list[-1])

        vdd = connect_hm_sup_ow(self, vdd_list, 0, ridx_p, tr_manager)
        vss = connect_hm_sup_ow(self, vss_list, 0, ridx_n, tr_manager)

        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        sup_vm_tidx = \
            self.get_available_tracks(vm_layer,
                                      self.arr_info.col_to_track(vm_layer, 0, RoundMode.NEAREST),
                                      self.arr_info.col_to_track(vm_layer, self.num_cols, RoundMode.NEAREST),
                                      lower=self.bound_box.yl, upper=self.bound_box.yh,
                                      width=tr_w_sup_vm,
                                      sep=tr_manager.get_sep(vm_layer, ('sup', 'sup')))[1:]
        vdd_vm_list, vss_vm_list = [], []
        for tidx in sup_vm_tidx:
            vdd_vm_list.append(
                self.connect_to_tracks(vdd, TrackID(vm_layer, tidx, tr_w_sup_vm), min_len_mode=MinLenMode.MIDDLE))
            vss_vm_list.append(
                self.connect_to_tracks(vss, TrackID(vm_layer, tidx, tr_w_sup_vm), min_len_mode=MinLenMode.MIDDLE))
        self.add_pin('VDD_vm', vdd_vm_list, label='VDD')
        self.add_pin('VSS_vm', vss_vm_list, label='VSS')

        self.add_pin('VDD', vdd)
        self.add_pin('VSS', vss)

        lch = arr_info.lch
        buf_params = ImmutableSortedDict(dict(
            lch=lch,
            w_p=w_pbuf,
            w_n=w_nbuf,
            th_p=th_dict['ps'],
            th_n=th_dict['nr'],
            seg=seg_obuf,
        ))
        obuf_params = buf_params if has_outbuf else None
        ibuf_params = buf_params.copy(append=dict(seg=seg_ibuf)) if has_inbuf else None
        self.sch_params = dict(
            core_params=ImmutableSortedDict(dict(
                lch=lch,
                seg_dict=ImmutableSortedDict(sch_seg_dict),
                w_dict=w_dict,
                th_dict=th_dict,
            )),
            outbuf_params=obuf_params,
            inbuf_params=ibuf_params,
            has_rstb=has_rstb,
        )

    def _get_w_th_dict(self, ridx_n: int, ridx_p: int, has_rstb: bool
                       ) -> Tuple[ImmutableSortedDict[str, int], ImmutableSortedDict[str, str]]:
        w_dict: Mapping[str, int] = self.params['w_dict']

        w_ans = {}
        th_ans = {}
        for row_idx, name_list in [(ridx_n, ['nfb', 'nr']),
                                   (ridx_p, ['pfb', 'ps'])]:
            rinfo = self.get_row_info(row_idx, 0)
            for name in name_list:
                w = w_dict.get(name, 0)
                if w == 0:
                    w = rinfo.width
                w_ans[name] = w
                th_ans[name] = rinfo.threshold

        if has_rstb:
            w_ans['pr'] = w_ans['ps']
            th_ans['pr'] = th_ans['ps']

        return ImmutableSortedDict(w_ans), ImmutableSortedDict(th_ans)


class SRLatchSymmetric(MOSBase, TemplateBaseZL):
    """Symmetric SR latch.  Mainly designed to be used with strongarm.
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sr_latch_symmetric')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = SRLatchSymmetricHalf.get_params_info()
        ans['swap_outbuf'] = 'True to swap output buffers, so outp is on opposite side of inp.'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = SRLatchSymmetricHalf.get_default_param_values()
        ans['swap_outbuf'] = False
        return ans

    def draw_layout(self) -> None:
        master: SRLatchSymmetricHalf = self.new_template(SRLatchSymmetricHalf, params=self.params)
        self.draw_base(master.draw_base_info)

        swap_outbuf: bool = self.params['swap_outbuf']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        arr_info = self.arr_info
        # placement
        inn_tidx = self.get_track_index(0, MOSWireType.G, 'sig', 0)
        inp_tidx = tr_manager.get_next_track(hm_layer, inn_tidx, 'sig', 'sig')
        sr_n_base_tidx = tr_manager.get_next_track(hm_layer, inp_tidx, 'sig', 'sig')
        tr_w_hm = tr_manager.get_width(hm_layer, 'sig')

        pg_l_tidx = tr_manager.get_next_track(hm_layer, sr_n_base_tidx, 'sig', 'sig')
        pg_h_tidx = self.get_track_index(-1, MOSWireType.G, 'sig', -1)
        n_master_params = self.params.copy(append={'sig_locs': {'srb': TrackID(hm_layer, sr_n_base_tidx),
                                                                'sr': TrackID(hm_layer, inn_tidx, tr_w_hm),
                                                                'pg_l': pg_l_tidx, 'pg_h': pg_h_tidx}})
        p_master_params = self.params.copy(append={'sig_locs': {'srb': TrackID(hm_layer, sr_n_base_tidx),
                                                                'sr': TrackID(hm_layer, inp_tidx, tr_w_hm),
                                                                'pg_l': pg_l_tidx, 'pg_h': pg_h_tidx}})

        n_master: SRLatchSymmetricHalf = self.new_template(SRLatchSymmetricHalf, params=n_master_params)
        p_master: SRLatchSymmetricHalf = self.new_template(SRLatchSymmetricHalf, params=p_master_params)

        hm_w, q_tidx, qb_tidx = n_master.q_tr_info
        _, sr_hm_top, sr_hm_bot = n_master.sr_hm_tr_info
        vm_w, sr_vm_tidx, srb_vm_tidx = n_master.sr_vm_tr_info

        nhalf = master.num_cols
        corel = self.add_tile(n_master, 0, nhalf, flip_lr=True)
        corer = self.add_tile(p_master, 0, nhalf)
        self.set_mos_size(num_cols=2 * nhalf)

        vm0 = arr_info.col_to_track(vm_layer, 0)
        vmh = arr_info.col_to_track(vm_layer, nhalf)
        vmdr = vmh - vm0
        vmdl = vmh + vm0

        pr = corel.get_pin('psr')
        psb = corel.get_pin('psrb')
        nr = corel.get_pin('nsr')
        nsb = corel.get_pin('nsrb')
        ps = corer.get_pin('psr')
        prb = corer.get_pin('psrb')
        ns = corer.get_pin('nsr')
        nrb = corer.get_pin('nsrb')

        pr, psb = self.connect_differential_tracks(pr, psb, hm_layer, sr_hm_top, sr_hm_bot,
                                                   width=hm_w)
        ps, prb = self.connect_differential_tracks(ps, prb, hm_layer, sr_hm_bot, sr_hm_top,
                                                   width=hm_w)
        sb = self.connect_to_tracks([psb, nsb], TrackID(vm_layer, vmdl - sr_vm_tidx, width=vm_w))
        r = self.connect_to_tracks([pr, nr], TrackID(vm_layer, vmdl - srb_vm_tidx, width=vm_w))
        s = self.connect_to_tracks([ps, ns], TrackID(vm_layer, vmdr + srb_vm_tidx, width=vm_w))
        rb = self.connect_to_tracks([prb, nrb], TrackID(vm_layer, vmdr + sr_vm_tidx, width=vm_w))
        self.match_warr_length([s, r])
        self.match_warr_length([sb, rb])

        if ns.track_id.base_index != nr.track_id.base_index:
            ns_conn = corel.get_pin('nsr_conn')
            nr_conn = corer.get_pin('nsr_conn')
            self.extend_wires([ns_conn, nr_conn], lower=min(ns_conn.lower, nr_conn.lower))

        self.add_pin('s', s)
        self.add_pin('r', r)
        self.add_pin('s', ns)
        self.add_pin('r', nr)
        if corel.has_port('srb_buf'):
            sbbuf = corer.get_pin('srb_buf')
            rbbuf = corel.get_pin('srb_buf')
            self.connect_to_track_wires(sbbuf, psb)
            self.connect_to_track_wires(rbbuf, prb)
        else:
            self.add_pin('sb', sb)
            self.add_pin('rb', rb)

        q_list = [corel.get_pin('q_vm'), corer.get_pin('qb')]
        qb_list = [corer.get_pin('q_vm'), corel.get_pin('qb')]
        if corel.has_port('buf_out'):
            if swap_outbuf:
                self.reexport(corel.get_port('buf_out'), net_name='qb')
                self.reexport(corer.get_port('buf_out'), net_name='q')
                q_list.append(corel.get_pin('buf_in'))
                qb_list.append(corer.get_pin('buf_in'))
            else:
                self.reexport(corel.get_port('buf_out'), net_name='q')
                self.reexport(corer.get_port('buf_out'), net_name='qb')
                q_list.append(corer.get_pin('buf_in'))
                qb_list.append(corel.get_pin('buf_in'))
        else:
            self.add_pin('q', q_list[0])
            self.add_pin('qb', qb_list[0])

        self.connect_differential_tracks(q_list, qb_list, self.conn_layer + 1,
                                         q_tidx, qb_tidx, width=hm_w)

        if corel.has_port('rstb'):
            self.reexport(corel.get_port('rstb'), net_name='rsthb')
            self.reexport(corer.get_port('rstb'), net_name='rstlb')

        self.add_pin('VDD', self.connect_wires([corel.get_pin('VDD'), corer.get_pin('VDD')]))
        self.add_pin('VSS', self.connect_wires([corel.get_pin('VSS'), corer.get_pin('VSS')]))
        self.add_pin('VDD_vm', corel.get_all_port_pins('VDD_vm') + corer.get_all_port_pins('VDD_vm'))
        self.add_pin('VSS_vm', corel.get_all_port_pins('VSS_vm') + corer.get_all_port_pins('VSS_vm'))

        self.sch_params = master.sch_params


class SAFFCore(RingOscUnit):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_saff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths dictionary.',
            vertical_out='True to connect outputs to vm_layer.',
            even_center='True to force center column to be even.',
            signal_locs='Signal locations',
            shift_input='True to shift preamp input for easy connection with outside',
            self_time='True to have selftime design',
            export_sup_to_xm1='',
            export_clk_to_xm1='',
            export_sr=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            vertical_out=True,
            even_center=False,
            shift_input=False,
            signal_locs={},
            self_time=False,
            export_sup_to_xm1=True,
            export_clk_to_xm1=True,
            export_sr=False,
        )

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        seg_dict: Dict[str, Dict] = self.params['seg_dict']
        w_dict: Dict[str, Dict] = self.params['w_dict']
        sig_locs = self.params['signal_locs']
        self_time: bool = self.params['self_time']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        # Make templates
        preamp_params = dict(pinfo=pinfo, vertical_sup=False, seg_dict=seg_dict['preamp'], w_dict=w_dict['preamp'],
                             flip_preamp_io=self.params['shift_input'])
        dyn_latch_params = dict(pinfo=pinfo, vertical_sup=False, seg_dict=seg_dict['dynlatch'],
                                w_dict=w_dict['dynlatch'], flip_np=True, self_time=self_time)
        sr_params = dict(pinfo=pinfo, seg_dict=seg_dict['sr'], w_dict=w_dict.get('sr', {}))

        preamp_master = self.new_template(PreAmpDig, params=preamp_params)
        dyn_latch_master = self.new_template(DynLatchSADig, params=dyn_latch_params)
        sr_master = self.new_template(SRLatchSymmetric, params=sr_params)

        # floorplanning
        preamp_ncol = preamp_master.num_cols
        dyn_latch_ncol = dyn_latch_master.num_cols
        _sr_latch_ncol = sr_master.num_cols
        min_sep = self.min_sep_col

        # placement
        preamp = self.add_tile(preamp_master, 0, min_sep)
        dynlatch = self.add_tile(dyn_latch_master, 0, preamp_ncol + 2 * min_sep)
        sr = self.add_tile(sr_master, 0, preamp_ncol + dyn_latch_ncol + 3 * min_sep)

        sup_vm_cols = [min_sep // 2, preamp_ncol + min_sep + min_sep // 2,
                       preamp_ncol + dyn_latch_ncol + 2 * min_sep + min_sep // 2]  # Collect some col idx for vm supply
        vdd_list, vss_list = [], []
        tap_sep_col = self.sub_sep_col
        sup_vm_cols.append(self.num_cols + tap_sep_col // 2)
        self.add_tap(self.num_cols + tap_sep_col + min_sep, vdd_list, vss_list)
        sup_vm_cols.append(self.num_cols + tap_sep_col // 2)
        self.set_mos_size(self.num_cols)

        preamp_o_n, preamp_o_p = preamp.get_pin('outn'), preamp.get_pin('outp')
        dynlatch_i_n, dynlatch_i_p = dynlatch.get_all_port_pins('inn'), dynlatch.get_all_port_pins('inp')
        dynlatch_o_n, dynlatch_o_p = dynlatch.get_pin('outn', layer=hm_layer), dynlatch.get_pin('outp', layer=hm_layer)
        sr_i_n, sr_i_p = sr.get_pin('r', layer=hm_layer), sr.get_pin('s', layer=hm_layer)
        self.connect_differential_wires(dynlatch_i_n, dynlatch_i_p, preamp_o_n, preamp_o_p)
        self.connect_differential_wires(dynlatch.get_all_port_pins('inn_conn'),
                                        dynlatch.get_all_port_pins('inp_conn'), preamp_o_n, preamp_o_p)
        self.connect_wires([dynlatch_o_n, sr_i_n])
        self.connect_wires([dynlatch_o_p, sr_i_p])

        # xm in/out
        _, xm_inout_tid = tr_manager.place_wires(xm_layer, ['sig'] * 2, center_coord=self.bound_box.h // 2)

        xm_inout_tid[0] = sig_locs.get('inn', xm_inout_tid[0])
        xm_inout_tid[1] = sig_locs.get('inp', xm_inout_tid[1])
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        inn, inp = self.connect_differential_tracks(preamp.get_pin('inn'), preamp.get_pin('inp'), xm_layer,
                                                    xm_inout_tid[0], xm_inout_tid[1], width=tr_w_sig_xm)
        outp, outn = self.connect_differential_tracks(sr.get_pin('q'), sr.get_pin('qb'), xm_layer,
                                                      xm_inout_tid[0], xm_inout_tid[1], width=tr_w_sig_xm)

        # Connect supplies
        # hm sup
        inst_list = [preamp, dynlatch, sr]
        vdd_hm_list, vss_hm_list = [], []
        for inst in inst_list:
            vdd_hm_list.append(inst.get_pin('VDD'))
            vss_hm_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_hm_list, upper=self.bound_box.xh, lower=self.bound_box.xl)
        vss_hm = self.connect_wires(vss_hm_list, upper=self.bound_box.xh, lower=self.bound_box.xl)
        self.connect_to_track_wires(vdd_list, vdd_hm)
        self.connect_to_track_wires(vss_list, vss_hm)

        # vm sup
        vdd_vm_list, vss_vm_list = [], []
        for inst in inst_list:
            vdd_vm_list.extend(inst.get_all_port_pins('VDD_vm'))
            vss_vm_list.extend(inst.get_all_port_pins('VSS_vm'))
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        for c in sup_vm_cols:
            _tidx = self.arr_info.col_to_track(vm_layer, c, mode=RoundMode.NEAREST)
            vdd_vm_list.append(self.connect_to_tracks(vdd_hm, TrackID(vm_layer, _tidx, tr_w_sup_vm)))
            vss_vm_list.append(self.connect_to_tracks(vss_hm, TrackID(vm_layer, _tidx, tr_w_sup_vm)))

        # xm sup
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        vdd_xm_coord = self.grid.track_to_coord(hm_layer, vdd_hm_list[0].track_id.base_index)
        vdd_xm_tid = self.grid.coord_to_track(xm_layer, vdd_xm_coord, RoundMode.NEAREST)
        vdd_xm = self.connect_to_tracks(vdd_vm_list, TrackID(xm_layer, vdd_xm_tid, tr_w_sup_xm, grid=self.grid),
                                        track_lower=vdd_hm_list[0].lower, track_upper=vdd_hm_list[0].upper)
        vss_xm_coord = self.grid.track_to_coord(hm_layer, vss_hm_list[0].track_id.base_index)
        vss_xm_tid = self.grid.coord_to_track(xm_layer, vss_xm_coord, RoundMode.NEAREST)
        vss_xm = self.connect_to_tracks(vss_vm_list, TrackID(xm_layer, vss_xm_tid, tr_w_sup_xm, grid=self.grid),
                                        track_lower=vss_hm_list[0].lower, track_upper=vss_hm_list[0].upper)

        # export clk to xm
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        clk_xm_tidx = self.grid.coord_to_track(xm_layer, preamp.get_pin('clk').middle, mode=RoundMode.NEAREST)
        clk_xm = self.connect_to_tracks(preamp.get_pin('clk'), TrackID(xm_layer, clk_xm_tidx, tr_w_clk_xm),
                                        min_len_mode=MinLenMode.MIDDLE)
        clk_ym_tidx = self.grid.coord_to_track(ym_layer, self.grid.track_to_coord(vm_layer, preamp.get_pin(
            'clk').track_id.base_index), RoundMode.NEAREST)
        clk_ym = self.connect_to_tracks(clk_xm, TrackID(ym_layer, clk_ym_tidx, tr_w_clk_ym))

        if self.params['export_clk_to_xm1']:
            tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
            clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, self.get_tile_pinfo(0).height // 2, RoundMode.NEAREST)
            clk_xm1 = self.connect_to_tracks(clk_ym, TrackID(xm1_layer, clk_xm1_tidx, tr_w_clk_xm1),
                                             min_len_mode=MinLenMode.MIDDLE)
            self.add_pin('clk', clk_xm1)
        if not self_time:
            clkb_xm = self.connect_to_tracks(dynlatch.get_all_port_pins('clk'),
                                             TrackID(xm_layer, clk_xm_tidx, tr_w_clk_xm),
                                             min_len_mode=MinLenMode.MIDDLE)
            clkb_ym_tidx = self.grid.coord_to_track(ym_layer, clkb_xm.middle, RoundMode.NEAREST)
            clkb_ym = self.connect_to_tracks(clkb_xm, TrackID(ym_layer, clkb_ym_tidx, tr_w_clk_ym))
            if self.params['export_clk_to_xm1']:
                tr_w_clk_xm1 = tr_manager.get_width(xm1_layer, 'clk')
                clk_xm1_tidx = self.grid.coord_to_track(xm1_layer, self.get_tile_pinfo(0).height // 2,
                                                        RoundMode.NEAREST)
                clkb_xm1 = self.connect_to_tracks(clkb_ym, TrackID(xm1_layer, clk_xm1_tidx, tr_w_clk_xm1),
                                                  min_len_mode=MinLenMode.MIDDLE)
                self.add_pin('clkb', clkb_xm1)
            self.add_pin('clkb', dynlatch.get_all_port_pins('clk') + [clkb_ym])

        # Connect supplies
        # ym layer
        if self.params['export_sup_to_xm1']:
            tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
            tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
            ym_tid_l = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.GREATER_EQ)
            ym_tid_r = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.LESS_EQ)
            ym_sup_tidxs = self.get_available_tracks(ym_layer, ym_tid_l, ym_tid_r, self.bound_box.yl,
                                                     self.bound_box.yh, width=tr_w_sup_ym,
                                                     sep=tr_manager.get_sep(ym_layer, ('sup', 'sup')))

            ym_sup_tidxs = ym_sup_tidxs[1:-1]

            vdd_ym = [self.connect_to_tracks(vdd_xm, TrackID(ym_layer, tid, tr_w_sup_ym))
                      for tid in ym_sup_tidxs[::2]]
            vss_ym = [self.connect_to_tracks(vss_xm, TrackID(ym_layer, tid, tr_w_sup_ym))
                      for tid in ym_sup_tidxs[1::2]]
            xm1_tidx_list = [self.grid.coord_to_track(xm1_layer, 0, mode=RoundMode.NEAREST),
                             self.grid.coord_to_track(xm1_layer, self.bound_box.h, mode=RoundMode.NEAREST)]
            vdd_xm1 = self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx_list[1], tr_w_sup_xm1, grid=self.grid),
                                             track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            vss_xm1 = self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx_list[0], tr_w_sup_xm1, grid=self.grid),
                                             track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            self.add_pin('VDD_ym', vdd_ym, label='VDD', show=self.show_pins)
            self.add_pin('VSS_ym', vss_ym, label='VSS', show=self.show_pins)
            self.add_pin('VDD_xm1', vdd_xm1, label='VDD', show=self.show_pins)
            self.add_pin('VSS_xm1', vss_xm1, label='VSS', show=self.show_pins)
            self.add_pin('VDD_xm', vdd_xm, label='VDD', show=self.show_pins)
            self.add_pin('VSS_xm', vss_xm, label='VSS', show=self.show_pins)
        else:
            self.add_pin('VDD', vdd_xm, label='VDD', show=self.show_pins)
            self.add_pin('VSS', vss_xm, label='VSS', show=self.show_pins)

        fill_conn_layer_intv(self, 0, 0, start_col=0, stop_col=self.num_cols, extend_to_gate=False)
        fill_conn_layer_intv(self, 0, -1, start_col=0, stop_col=self.num_cols, extend_to_gate=False)

        if self.params['export_sr']:
            self.add_pin('r', sr_i_n)
            self.add_pin('s', sr_i_p)
        self.add_pin('inn', inn)
        self.add_pin('inp', inp)
        self.add_pin('outn', outn)
        self.add_pin('outp', outp)
        self.add_pin('clk', [preamp.get_pin('clk'), clk_ym])

        # Schematic params
        self.sch_params = dict(
            preamp=preamp_master.sch_params,
            dynlatch=dyn_latch_master.sch_params,
            sr=sr_master.sch_params,
            export_sr=self.params['export_sr']
        )


class SAFFCol(RingOscUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            num_stages='Number of stages',
            saff_params='strongArm flops parameter',
            pinfo='Pinfo for unit row strongArm flop',
            topbot_dummy='Add empty topbot dummy row',
            signal_locs=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(topbot_dummy=True, signal_locs={})

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        saff_params: Param = self.params['saff_params']
        num_stages: int = self.params['num_stages']
        topbot_dummy: int = self.params['topbot_dummy']
        saff_template: SAFFCore = self.new_template(SAFFCore,
                                                    params=saff_params.copy(append=dict(pinfo=pinfo,
                                                                                        signal_locs=self.params[
                                                                                            'signal_locs'])))
        self.draw_base(saff_template.draw_base_info)

        tot_rows = num_stages + 2 if topbot_dummy else num_stages  # Two match bottom and top dummy of ring

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        saff_list = [self.add_tile(saff_template, idx + 1 if topbot_dummy else idx, 0) for idx in range(num_stages)]
        self.set_mos_size(num_cols=saff_template.num_cols, num_tiles=tot_rows)

        vdd_ym_list = [w for inst in saff_list for w in inst.get_all_port_pins('VDD_ym')]
        vss_ym_list = [w for inst in saff_list for w in inst.get_all_port_pins('VSS_ym')]
        self.connect_wires(vdd_ym_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
        self.connect_wires(vss_ym_list, lower=self.bound_box.yl, upper=self.bound_box.yh)

        [self.reexport(inst.get_port('VDD_xm1'), net_name='VDD') for inst in saff_list]
        [self.reexport(inst.get_port('VSS_xm1'), net_name='VSS') for inst in saff_list]

        [self.add_pin(f'inn<{idx}>', saff_list[idx].get_pin('inn')) for idx in range(num_stages)]
        [self.add_pin(f'inp<{idx}>', saff_list[idx].get_pin('inp')) for idx in range(num_stages)]

        [self.add_pin(f'outn<{idx}>', saff_list[idx].get_pin('outn')) for idx in range(num_stages)]
        [self.add_pin(f'outp<{idx}>', saff_list[idx].get_pin('outp')) for idx in range(num_stages)]

        clk_ym = self.connect_wires([inst.get_pin('clk', layer=ym_layer) for inst in saff_list])
        self.add_pin('clk', clk_ym)

        if saff_list[0].has_port('clkb'):
            clkb_ym = self.connect_wires([inst.get_pin('clkb', layer=ym_layer) for inst in saff_list])
            self.add_pin('clkb', clkb_ym)

        if saff_list[0].get_all_port_pins('clk', layer=xm1_layer):
            for inst in saff_list:
                self.add_pin('clk_xm1', inst.get_all_port_pins('clk', layer=xm1_layer))
                self.add_pin('clkb_xm1', inst.get_all_port_pins('clkb', layer=xm1_layer))

        self._sch_params = saff_template.sch_params
