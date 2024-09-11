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

from typing import Any, Dict, Type, Optional, List

from bag.design.database import ModuleDB, Module
from bag.layout.routing.base import TrackID
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableList
from bag3_digital.layout.stdcells.gates import NOR2Core
from pybag.enum import MinLenMode, RoundMode, PinMode, Orient2D
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from ..digital import InvCore, InvTristateCore, NAND2Core, PassGateCore, InvChainCore
from ..util.util import export_xm_sup, fill_tap


class DynLatchCore(MOSBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sar_logic_dyn_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='segments of transistors',
            w_dict='width of transistros',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            sig_locs='Optional dictionary of user defined signal locations',
            vertical_out='True to draw output on vertical metal layer.',
            vertical_sup='True to have supply unconnected on conn_layer.',
            vertical_in='False to not draw the vertical input wire when is_guarded = True.',
            min_len_mode='A Dictionary specfiying min_len_mode for connections',
            tr_manager='override track manager',
            nand='True to generate nand gate',
            np_coupled='True to export fb port',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            seg_dict={},
            w_dict={},
            ridx_p=-1,
            ridx_n=0,
            is_guarded=False,
            sig_locs={},
            vertical_out=True,
            vertical_sup=False,
            vertical_in=True,
            min_len_mode=dict(
                in0=MinLenMode.NONE,
                in1=MinLenMode.NONE,
                out=MinLenMode.MIDDLE,
            ),
            tr_manager=None,
            nand=False,
            np_coupled=False,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        # grid = self.grid

        seg_dict: Dict = self.params['seg_dict']
        w_dict: Dict = self.params['w_dict']
        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']
        # sig_locs: Mapping[str, Union[float, HalfInt]] = self.params['sig_locs']
        # mlm: Dict[str, MinLenMode] = self.params['min_len_mode']
        # vertical_out: bool = self.params['vertical_out']
        vertical_sup: bool = self.params['vertical_sup']
        # vertical_in: bool = self.params['vertical_in']

        is_nand = self.params['nand']
        np_coupled = self.params['np_coupled']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        if self.top_layer < vm_layer:
            raise ValueError(f'MOSBasePlaceInfo top layer must be at least {vm_layer}')

        # Placement
        seg_pull, seg_d, seg_en, seg_inv, seg_inv_fb = \
            seg_dict['pull'], seg_dict['d'], seg_dict['en'], seg_dict['inv'], seg_dict['inv_fb']

        # Total number of cols
        int_g_tidx = self.get_track_id(ridx_n if is_nand else ridx_p, MOSWireType.G, wire_name='sig',
                                       wire_idx=1 if is_nand else -2)
        pd_tid = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pd_tid2 = self.get_track_id(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)
        nd_tid = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        nd_tid2 = self.get_track_id(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=-2)

        en_tid = self.get_track_id(ridx_p if is_nand else ridx_n,
                                   MOSWireType.G, wire_name='sig', wire_idx=-1 if is_nand else 0)
        d_tid = self.get_track_id(ridx_p if is_nand else ridx_n,
                                  MOSWireType.G, wire_name='sig', wire_idx=-2 if is_nand else 1)
        rst_tid = self.get_track_id(ridx_n if is_nand else ridx_p,
                                    MOSWireType.G, wire_name='sig', wire_idx=0 if is_nand else -1)

        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_dict['inv']['wp'], w_n=w_dict['inv']['wn'],
                          ridx_n=ridx_n, ridx_p=ridx_p, sig_locs={'nin': d_tid.base_index},
                          vertical_out=False, vertical_sup=vertical_sup)
        inv_fb_params = dict(pinfo=pinfo, seg=seg_inv_fb, w_p=w_dict['inv_fb']['wp'], w_n=w_dict['inv_fb']['wn'],
                             ridx_n=ridx_n, ridx_p=ridx_p, vertical_out=False, vertical_sup=vertical_sup,
                             sig_locs={'nin': int_g_tidx.base_index, 'nout': nd_tid.base_index,
                                       'pout': pd_tid.base_index})
        inv_master = self.new_template(InvCore, params=inv_params)
        inv_fb_master = self.new_template(InvCore, params=inv_fb_params)

        lat_main_col = max(seg_d + seg_en, seg_pull)
        if is_nand:
            pull = self.add_mos(ridx_n, lat_main_col - seg_pull, seg_pull, w=w_dict['pull'])
            lat_main_col += 1
            d = self.add_mos(ridx_p, lat_main_col - seg_d, seg=seg_d, w=w_dict['d'])
            en_col = lat_main_col - seg_d if seg_en & 1 else lat_main_col - seg_d - seg_en
            en = self.add_mos(ridx_p, en_col, seg=seg_en, w=w_dict['en'], flip_lr=seg_en & 1)
            self.connect_to_tracks([d.s, en.s], pd_tid2)
        else:
            pull = self.add_mos(ridx_p, lat_main_col - seg_pull, seg=seg_pull, w=w_dict['pull'])
            lat_main_col += 1
            d = self.add_mos(ridx_n, lat_main_col - seg_d, seg=seg_d, w=w_dict['d'])
            en_col = lat_main_col - seg_d if seg_en & 1 else lat_main_col - seg_d - seg_en
            en = self.add_mos(ridx_n, en_col, seg=seg_en, w=w_dict['en'], flip_lr=seg_en & 1)
            self.connect_to_tracks([d.s, en.s], nd_tid2)
        cur_col = lat_main_col + self.min_sep_col
        cur_col += inv_fb_master.num_cols
        if not (seg_inv + seg_inv_fb) & 1:
            cur_col += 1

        if np_coupled:
            if is_nand:
                fb = self.add_mos(ridx_n, cur_col, seg=seg_inv_fb, w=w_dict['inv_fb']['wn'], flip_lr=True)
            else:
                fb = self.add_mos(ridx_p, cur_col, seg=seg_inv_fb, w=w_dict['inv_fb']['wp'], flip_lr=True)
            inv_fb = None
        else:
            inv_fb = self.add_tile(inv_fb_master, 0, cur_col, flip_lr=True)
            fb = None
        inv = self.add_tile(inv_master, 0, cur_col)

        self.set_mos_size()

        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')

        if is_nand:
            d_hm = self.connect_to_tracks(d.g, d_tid, min_len_mode=MinLenMode.MIDDLE)
            en_hm = self.connect_to_tracks(en.g, en_tid, min_len_mode=MinLenMode.MIDDLE)
            rstb_hm = self.connect_to_tracks(pull.g, rst_tid, min_len_mode=MinLenMode.MIDDLE)

            pd_hm = self.connect_to_tracks(d.d, pd_tid)
            nd_hm = self.connect_to_tracks(pull.d, nd_tid)
        else:
            d_hm = self.connect_to_tracks(d.g, d_tid, min_len_mode=MinLenMode.MIDDLE)
            en_hm = self.connect_to_tracks(en.g, en_tid, min_len_mode=MinLenMode.MIDDLE)
            rstb_hm = self.connect_to_tracks(pull.g, rst_tid, min_len_mode=MinLenMode.MIDDLE)

            pd_hm = self.connect_to_tracks(pull.d, pd_tid)
            nd_hm = self.connect_to_tracks(d.d, nd_tid)

        if inv_fb:
            ob_vm_tidx = self.grid.coord_to_track(vm_layer, (inv_fb.bound_box.xl + inv_fb.bound_box.xh) // 2,
                                                  RoundMode.NEAREST)
        else:
            fb_hm = self.connect_to_tracks(fb.d, nd_tid if is_nand else pd_tid)
            ob_vm_tidx = self.grid.coord_to_track(vm_layer,
                                                  self.grid.track_to_coord(vm_layer, fb.d.track_id.base_index),
                                                  RoundMode.NEAREST)
            self.connect_to_tracks(fb_hm, TrackID(vm_layer, ob_vm_tidx, tr_w_vm))
            fb_in_hm = self.connect_to_tracks(fb.g, int_g_tidx)
            fb_vm_tidx = tr_manager.get_next_track(vm_layer, ob_vm_tidx, 'sig', 'sig')
            fb_in_vm = self.connect_to_tracks(fb_in_hm, TrackID(vm_layer, fb_vm_tidx, tr_w_vm))
            self.add_pin('ob_fb', fb_in_vm)

        o_vm_tidx = self.grid.coord_to_track(vm_layer, (inv.bound_box.xl + inv.bound_box.xh) // 2,
                                             RoundMode.NEAREST)

        ob_vm = self.connect_to_tracks([pd_hm, nd_hm, inv.get_pin('nin')], TrackID(vm_layer, ob_vm_tidx, tr_w_vm))
        # Get another obvm for output connection
        ob_vm_tidx = self.grid.coord_to_track(vm_layer, pd_hm.middle if is_nand else nd_hm.middle, RoundMode.NEAREST)
        ob_vm_gate = self.connect_to_tracks([pd_hm, nd_hm], TrackID(vm_layer, ob_vm_tidx, tr_w_vm))
        if inv_fb:
            o_vm = self.connect_to_tracks([inv.get_pin('nout'), inv.get_pin('pout'), inv_fb.get_pin('nin')],
                                          TrackID(vm_layer, o_vm_tidx, tr_w_vm))
            inv_vdd = inv.get_all_port_pins('VDD') + inv_fb.get_all_port_pins('VDD')
            inv_vss = inv.get_all_port_pins('VSS') + inv_fb.get_all_port_pins('VSS')
        else:
            o_vm = self.connect_to_tracks([inv.get_pin('nout'), inv.get_pin('pout')],
                                          TrackID(vm_layer, o_vm_tidx, tr_w_vm))
            inv_vdd = inv.get_all_port_pins('VDD')
            inv_vss = inv.get_all_port_pins('VSS')

        if vertical_sup:
            vdd = [en.d if is_nand else pull.s] + inv_vdd
            vss = [pull.s if is_nand else en.d] + inv_vss
            if fb:
                if is_nand:
                    vss += [fb.s]
                else:
                    vdd += [fb.s]
            self.add_pin('VDD', vdd, connect=True)
            self.add_pin('VSS', vss, connect=True)
        else:
            vdd = self.connect_to_track_wires([en.d if is_nand else pull.s], inv_vdd)
            vss = self.connect_to_track_wires([pull.s if is_nand else en.d], inv_vss)
            if fb:
                if is_nand:
                    self.connect_to_track_wires(fb.s, inv_vss)
                else:
                    self.connect_to_track_wires(fb.s, inv_vdd)

            self.add_pin('VDD', vdd)
            self.add_pin('VSS', vss)

        self.add_pin('o', o_vm)
        self.add_pin('ob_gate', ob_vm, hide=True)
        self.add_pin('ob', ob_vm_gate)
        self.add_pin('ob_hm', [pd_hm, nd_hm], label='ob')
        # [d_hm, en_hm, rstb_hm] = self.extend_wires([d_hm, en_hm, rstb_hm], lower=self.bound_box.xl)
        self.add_pin('d', d_hm)
        self.add_pin('en', en_hm)
        self.add_pin('rst' if is_nand else 'rstb', rstb_hm)

        # default_wp = self.place_info.get_row_place_info(ridx_p).row_info.width
        # default_wn = self.place_info.get_row_place_info(ridx_n).row_info.width
        thp = self.place_info.get_row_place_info(ridx_p).row_info.threshold
        thn = self.place_info.get_row_place_info(ridx_n).row_info.threshold
        lch = self.place_info.lch
        self.sch_params = dict(
            seg_d=seg_d,
            seg_en=seg_en,
            seg_pull=seg_pull,
            seg_inv=seg_inv,
            seg_inv_fb=seg_inv_fb,
            w_dict=w_dict,
            lch=lch,
            th_n=thn,
            th_p=thp,
            nand=is_nand,
            np_coupled=np_coupled,
        )


class RstLatchCore(MOSBase):
    """A transmission gate based latch with reset pin."""

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._seg_in = None

    @property
    def seg_in(self) -> int:
        return self._seg_in

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rst_latch')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg='number of segments of output NOR.',
            seg_dict='Dictionary of number of segments',
            w_p='pmos width.',
            w_n='nmos width.',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            sig_locs='Signal track location dictionary.',
            fanout_in='input stage fanout.',
            fanout_kp='keeper stage fanout.',
            vertical_clk='True to have vertical clk and clkb',
            vertical_sup='True to have vertical supply',
            resetb='True to low reset, reset to logic 1',
            passgate='True to use passgate',
            resetable='True to make latch resetable'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_p=0,
            w_n=0,
            ridx_p=-1,
            ridx_n=0,
            sig_locs=None,
            fanout_in=4,
            fanout_kp=4,
            seg_dict=None,
            seg=1,
            vertical_clk=True,
            vertical_sup=2,
            resetb=False,
            passgate=False,
            resetable=True
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])

        self.draw_base(pinfo)
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        if self.top_layer < vm_layer:
            raise ValueError(f'MOSBasePlaceInfo top layer must be at least {vm_layer}')
        # setup floorplan

        seg: int = self.params['seg']
        seg_dict: Optional[Dict[str, int]] = self.params['seg_dict']
        w_p: int = self.params['w_p']
        w_n: int = self.params['w_n']
        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']
        sig_locs: Optional[Dict[str, float]] = self.params['sig_locs']
        fanout_in: float = self.params['fanout_in']
        fanout_kp: float = self.params['fanout_kp']
        vertical_clk: bool = self.params['vertical_clk']
        vertical_sup: bool = self.params['vertical_sup']

        # compute track locations and create masters
        tr_manager = self.tr_manager
        tr_w_v = tr_manager.get_width(vm_layer, 'sig')
        if sig_locs is None:
            sig_locs = {}

        ng0 = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1 = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg0 = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)
        pg1 = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg2 = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=2)
        nd0 = self.get_track_index(ridx_n, MOSWireType.DS_GATE, wire_name='sig', wire_idx=0)
        nd1 = self.get_track_index(ridx_n, MOSWireType.DS_GATE, wire_name='sig', wire_idx=1)
        pd0 = self.get_track_index(ridx_p, MOSWireType.DS_GATE, wire_name='sig', wire_idx=0)
        pd1 = self.get_track_index(ridx_p, MOSWireType.DS_GATE, wire_name='sig', wire_idx=1)
        if seg_dict is None:
            seg_t1 = max(1, int(round(seg / fanout_kp)))
            seg_t0 = self._seg_in = max(2 * seg_t1, max(2, int(round(seg / fanout_in))))
        else:
            seg = seg_dict['nor']
            seg_t1 = seg_dict['keep']
            seg_t0 = seg_dict['in']

        resetb: bool = self.params['resetb']
        resetable: bool = self.params['resetable']
        if resetable:
            nor_sig_locs = dict(
                nin0=sig_locs.get('rst', pg2),
                nin1=sig_locs.get('nclk', ng0),
                pout=pd1,
                nout=nd1,
            )
        else:
            nor_sig_locs = dict(
                nin=sig_locs.get('nclk', ng0),
                pout=pd1,
                nout=nd1,
            )
        params = dict(pinfo=pinfo, seg=seg, w_p=w_p, w_n=w_n, ridx_p=ridx_p, ridx_n=ridx_n,
                      sig_locs=nor_sig_locs, vertical_sup=vertical_sup)
        if resetable:
            rst_gate_master = self.new_template(NAND2Core if resetb else NOR2Core, params=params)
        else:
            rst_gate_master = self.new_template(InvCore, params=params)

        key = 'in' if 'in' in sig_locs else ('nin' if 'nin' in sig_locs else 'pin')

        passgate = self.params['passgate']
        t0_sig_locs = dict()
        if passgate:
            t0_sig_locs.update(**dict(en=sig_locs.get('nclk', ng0), enb=sig_locs.get('pclkb', pg1),
                                      pd=pd0, nd=nd0, ns=sig_locs.get(key, pg0)))
        else:
            t0_sig_locs.update(**dict(nen=sig_locs.get('nclk', ng0), pen=sig_locs.get('pclkb', pg1),
                                      pout=pd0, nout=nd0, nin=sig_locs.get(key, pg0)))
        params = dict(pinfo=pinfo, seg=seg_t0, w_p=w_p, w_n=w_n, ridx_p=ridx_p, ridx_n=ridx_n,
                      vertical_out=False, sig_locs=t0_sig_locs, vertical_sup=vertical_sup)
        t0_master = self.new_template(PassGateCore if passgate else InvTristateCore, params=params)

        t1_sig_locs = dict(
            nin=sig_locs.get('pout', pg1),
            pout=pd0,
            nout=nd0,
            nen=sig_locs.get('nclkb', ng1),
            pen=sig_locs.get('pclk', pg0),
        )
        mlm_dict = dict(
            enb=MinLenMode.MIDDLE,
            en=MinLenMode.MIDDLE,
        )
        params = dict(pinfo=pinfo, seg=seg_t1, w_p=w_p, w_n=w_n, ridx_p=ridx_p, ridx_n=ridx_n,
                      vertical_out=False, sig_locs=t1_sig_locs, vertical_sup=vertical_sup, min_len_mode=mlm_dict)
        t1_master = self.new_template(InvTristateCore, params=params)

        # set size
        blk_sp = max(self.min_sep_col, self.get_hm_sp_le_sep_col())
        t0_ncol = t0_master.num_cols
        t1_ncol = t1_master.num_cols
        rst_ncol = rst_gate_master.num_cols
        num_cols = t0_ncol + t1_ncol + rst_ncol + blk_sp * 2
        self.set_mos_size(num_cols)

        # add instances
        t1_col = t0_ncol + blk_sp
        nor_col = num_cols - rst_ncol
        t0 = self.add_tile(t0_master, 0, 0)
        t1 = self.add_tile(t1_master, 0, t1_col)
        rst_inst = self.add_tile(rst_gate_master, 0, nor_col)

        # routing
        # connect/export VSS/VDD
        vss_list, vdd_list = [], []
        for inst in (t0, t1, rst_inst):
            vss_list.extend(inst.get_all_port_pins('VSS'))
            vdd_list.extend(inst.get_all_port_pins('VDD'))
        self.add_pin('VSS', self.connect_wires(vss_list))
        self.add_pin('VDD', self.connect_wires(vdd_list))

        # export input
        self.reexport(t0.get_port('ns' if passgate else 'nin'), net_name='nin', hide=True)
        self.reexport(t0.get_port('ps' if passgate else 'pin'), net_name='pin', hide=True)
        self.reexport(t0.get_port('s' if passgate else 'in'), net_name='in')

        # connect output
        out = rst_inst.get_pin('out')
        in2 = t1.get_pin('nin')
        self.connect_to_track_wires(in2, out)
        self.add_pin('out', out)
        self.add_pin('nout', in2, hide=True)
        self.add_pin('pout', in2, hide=True)

        # connect middle node
        mid_coord = (t1.bound_box.xh + rst_inst.bound_box.xl) // 2
        mid_tidx = self.grid.coord_to_track(vm_layer, mid_coord, RoundMode.LESS)
        mid_tid = TrackID(vm_layer, mid_tidx, width=tr_w_v)
        if out.layer_id == vm_layer:
            next_tidx = tr_manager.get_next_track(vm_layer, mid_tidx, 'sig', 'sig', up=True)
            if next_tidx >= out.track_id.base_index:
                raise ValueError('oops!')

        if passgate:
            warrs = [t0.get_pin('nd'), t0.get_pin('pd'), t1.get_pin('pout'), t1.get_pin('nout'),
                     rst_inst.get_pin('nin<1>' if resetable else 'nin')]
        else:
            warrs = [t0.get_pin('pout'), t0.get_pin('nout'), t1.get_pin('pout'), t1.get_pin('nout'),
                     rst_inst.get_pin('nin<1>' if resetable else 'nin')]
        mid_vm_warr = self.connect_to_tracks(warrs, mid_tid)

        # connect clocks
        clk_tidx = sig_locs.get('clk', self.place_info.get_source_track(t1_col + 1))
        clkb_tidx = sig_locs.get('clkb', self.place_info.get_source_track(t1_col - blk_sp - t0_master.num_cols // 2))
        clk_tid = TrackID(vm_layer, clk_tidx, width=tr_w_v)
        clkb_tid = TrackID(vm_layer, clkb_tidx, width=tr_w_v)
        t0_en = t0.get_pin('en')
        t1_en = t1.get_pin('en')
        t1_enb = t1.get_pin('enb')
        t0_enb = t0.get_pin('enb')
        clk_hm = [t0_en, t1_enb]
        clkb_hm = [t0_enb, t1_en]
        if vertical_clk:
            clk = self.connect_to_tracks(clk_hm, clk_tid)
            clkb = self.connect_to_tracks(clkb_hm, clkb_tid)
            self.add_pin('clk', clk)
            self.add_pin('clkb', clkb)

        self.add_pin('outb', [rst_inst.get_pin('in<1>' if resetable else 'in'), mid_vm_warr])
        self.add_pin('noutb', rst_inst.get_pin('nin<1>' if resetable else 'nin'), hide=True)
        self.add_pin('poutb', rst_inst.get_pin('pin<1>' if resetable else 'pin'), hide=True)
        if resetable:
            self.add_pin('rstb' if resetb else 'rst', rst_inst.get_pin('in<0>'))
            self.add_pin('nrstb' if resetb else 'nrst', rst_inst.get_pin('nin<0>'), hide=True)
            self.add_pin('prstb' if resetb else 'prst', rst_inst.get_pin('nin<0>'), hide=True)

        self.add_pin('mid_vm', mid_vm_warr, hide=True)
        self.add_pin('nclk', t0_en, label='clk:', hide=vertical_clk)
        self.add_pin('pclk', t1_enb, label='clk:', hide=vertical_clk)
        self.add_pin('nclkb', t1_en, label='clkb:', hide=vertical_clk)
        self.add_pin('pclkb', t0_enb, label='clkb:', hide=vertical_clk)

        # set properties
        self.sch_params = dict(
            tin=t0_master.sch_params,
            tfb=t1_master.sch_params,
            rst=rst_gate_master.sch_params,
            resetb=resetb,
            passgate=passgate,
            resetable=resetable
        )


class FlopCore(MOSBase):
    """A transmission gate based flip-flop."""

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._cntr_col_clk = None

    @property
    def seg_in(self):
        return self.sch_params['seg_m']['in']

    @property
    def cntr_col_clk(self):
        return self._cntr_col_clk

    def get_schematic_class_inst(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rst_flop')
        # noinspection PyTypeChecker

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg='number of segments of output inverter.',
            w_p='pmos width.',
            w_n='nmos width.',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            sig_locs='Signal track location dictionary.',
            fanout_in='latch input stage fanout.',
            fanout_kp='latch keeper stage fanout.',
            fanout_lat='fanout between latches.',
            fanout_mux='fanout of scan mux, if present.',
            seg_ck='number of segments for clock inverter.  0 to disable.',
            seg_mux='Dictionary of segments for scan mux, if scanable',
            resetable='True if flop is resetable, default is False',
            resetb='True if flop is reset low, reset to logic 1',
            passgate='True to use passgate',
            extra_sp='This parameter is added to the min value of one of the separations '
                     '(mostly used to make power vertical stripes aligned)',
            vertical_sup='True to have vertical supply',
            rstbuf='True to add rst buffer'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_p=0,
            w_n=0,
            ridx_p=-1,
            ridx_n=0,
            sig_locs=None,
            fanout_in=4,
            fanout_kp=8,
            fanout_lat=4,
            fanout_mux=4,
            seg_ck=0,
            seg_mux=None,
            resetable=False,
            resetb=False,
            passgate=False,
            rstbuf=False,
            extra_sp=0,
            vertical_sup=False
        )

    def draw_layout(self):
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])

        seg: int = self.params['seg']
        w_p: int = self.params['w_p']
        w_n: int = self.params['w_n']
        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']
        sig_locs: Optional[Dict[str, float]] = self.params['sig_locs']
        fanout_in: float = self.params['fanout_in']
        fanout_kp: float = self.params['fanout_kp']
        fanout_lat: float = self.params['fanout_lat']
        seg_ck: int = self.params['seg_ck']
        rst: bool = self.params['resetable']
        extra_sp: int = self.params['extra_sp']
        vertical_sup: bool = self.params['vertical_sup']
        passgate: bool = self.params['passgate']
        rstbuf: bool = self.params['rstbuf']

        # setup floorplan
        self.draw_base(pinfo)

        # compute track locations
        if sig_locs is None:
            sig_locs = {}
        key = 'in' if 'in' in sig_locs else ('nin' if 'nin' in sig_locs else 'pin')
        in_tidx = sig_locs.get(key, self.get_track_index(ridx_p, MOSWireType.G,
                                                         wire_name='sig', wire_idx=0))
        pclkb_tidx = sig_locs.get('pclkb', self.get_track_index(ridx_p, MOSWireType.G,
                                                                wire_name='sig', wire_idx=0))
        nclk_idx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        nclkb_idx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        pclk_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        rst_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=2)
        # Might not need rst tidx when not rstable
        clk_idx = sig_locs.get('clk', None)
        clkb_idx = sig_locs.get('clkb', None)

        # make masters
        lat_params = dict(pinfo=pinfo, seg=seg, w_p=w_p, w_n=w_n, ridx_p=ridx_p, ridx_n=ridx_n,
                          sig_locs={'nclk': nclk_idx, 'nclkb': nclkb_idx, 'pclk': pclk_tidx,
                                    'nin': pclk_tidx, 'pclkb': pclkb_tidx, 'rst': rst_tidx,
                                    'pout': sig_locs.get('pout', pclkb_tidx)},
                          fanout_in=fanout_in, fanout_kp=fanout_kp, vertical_sup=vertical_sup,
                          resetable=rst, resetb=self.params['resetb'], passgate=passgate)

        s_master = self.new_template(RstLatchCore, params=lat_params)
        seg_m = max(2, int(round(s_master.seg_in / fanout_lat)))
        lat_params['seg'] = seg_m
        if passgate:
            lat_params['resetb'] = not self.params['resetb']
        lat_params['sig_locs'] = lat_sig_locs = {'nclk': nclkb_idx, 'nclkb': nclk_idx,
                                                 'pclk': pclkb_tidx, 'nin': in_tidx,
                                                 'pclkb': pclk_tidx}
        if clk_idx is not None:
            lat_sig_locs['clkb'] = clk_idx
        if clkb_idx is not None:
            lat_sig_locs['clk'] = clkb_idx

        m_master = self.new_template(RstLatchCore, params=lat_params)

        cur_col = 0
        blk_sp = self.min_sep_col
        pd0_tidx = self.get_track_index(ridx_p, MOSWireType.DS_GATE, wire_name='sig', wire_idx=0)
        pd1_tidx = self.get_track_index(ridx_p, MOSWireType.DS_GATE, wire_name='sig', wire_idx=1)
        nd0_tidx = self.get_track_index(ridx_n, MOSWireType.DS_GATE, wire_name='sig', wire_idx=0)
        nd1_tidx = self.get_track_index(ridx_n, MOSWireType.DS_GATE, wire_name='sig', wire_idx=1)

        inst_list = []

        m_ncol = m_master.num_cols
        s_ncol = s_master.num_cols
        m_inv_sp = blk_sp if rst else 0
        inv_master = None
        if seg_ck > 0:
            params = dict(pinfo=pinfo, seg=seg_ck, w_p=w_p, w_n=w_n, ridx_p=ridx_p,
                          ridx_n=ridx_n, sig_locs={'nin': nclk_idx, 'pout': pd0_tidx,
                                                   'nout': nd0_tidx}, vertical_sup=vertical_sup)
            if passgate and rstbuf:
                rst_buf_params = dict(pinfo=pinfo, seg=seg_ck, w_p=w_p, w_n=w_n, ridx_p=ridx_p,
                                      ridx_n=ridx_n, sig_locs={'nin': rst_tidx, 'pout': pd1_tidx,
                                                               'nout': nd1_tidx},
                                      vertical_sup=vertical_sup, vertical_out=False)
                rst_buf_master = self.new_template(InvCore, params=rst_buf_params)
            else:
                rst_buf_master = None

            inv_master = self.new_template(InvCore, params=params)
            ncol = cur_col + m_ncol + s_ncol + blk_sp + inv_master.num_cols + m_inv_sp
            scol = cur_col + m_ncol + inv_master.num_cols + blk_sp + m_inv_sp + extra_sp
            if passgate and rstbuf:
                ncol += rst_buf_master.num_cols
                scol += rst_buf_master.num_cols
                rst_buf_inst = self.add_tile(rst_buf_master, 0, cur_col + m_ncol + m_inv_sp + rst_buf_master.num_cols)
            else:
                rst_buf_inst = None
            b_inst = self.add_tile(inv_master, 0, cur_col + m_ncol + m_inv_sp)
            self._cntr_col_clk = scol - (blk_sp + extra_sp) // 2
            inst_list.append(b_inst)
        else:
            if passgate:
                raise ValueError("not implemented")
            ncol = cur_col + m_ncol + s_ncol + blk_sp
            scol = cur_col + m_ncol + blk_sp + extra_sp
            self._cntr_col_clk = scol - (blk_sp + extra_sp) // 2
            b_inst = None
            rst_buf_inst = None
            rst_buf_master = None

        # set size
        self.set_mos_size(ncol)

        # add instances
        m_inst = self.add_tile(m_master, 0, cur_col)
        s_inst = self.add_tile(s_master, 0, scol)
        inst_list.append(m_inst)
        inst_list.append(s_inst)

        # connect/export VSS/VDD
        vss_list, vdd_list = [], []
        for inst in inst_list:
            vss_list.extend(inst.get_all_port_pins('VSS'))
            vdd_list.extend(inst.get_all_port_pins('VDD'))
        self.add_pin('VSS', self.connect_wires(vss_list))
        self.add_pin('VDD', self.connect_wires(vdd_list))

        # connect intermediate node
        self.connect_wires([s_inst.get_pin('nin'), m_inst.get_pin('nout')])
        # connect clocks
        pclkb = self.connect_wires([s_inst.get_pin('pclkb'), m_inst.get_pin('pclk')])
        if b_inst is None:
            self.connect_wires([s_inst.get_pin('nclk'), m_inst.get_pin('nclkb')])
            self.reexport(m_inst.get_port('clk'), net_name='clkb')
            self.reexport(m_inst.get_port('nclk'), net_name='nclkb')
        else:
            self.connect_wires([s_inst.get_pin('nclk'), m_inst.get_pin('nclkb'),
                                b_inst.get_pin('nin')])
            self.connect_to_track_wires(pclkb, b_inst.get_pin('out'))

        # connect rst if rst is True
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        if rst:
            net_name = 'rstb' if self.params['resetb'] else 'rst'
            net_nameb = 'rst' if self.params['resetb'] else 'rstb'
            if passgate and rstbuf:
                self.connect_wires([rst_buf_inst.get_pin('nin'), s_inst.get_pin('n' + net_name)])
                rstb_vm_tidx = self.grid.coord_to_track(vm_layer, b_inst.bound_box.xl, RoundMode.NEAREST)
                self.connect_to_tracks(
                    [m_inst.get_pin('n' + net_nameb), rst_buf_inst.get_pin('nout'), rst_buf_inst.get_pin('pout')],
                    TrackID(vm_layer, rstb_vm_tidx))
                rst_vm_tidx = self.grid.coord_to_track(vm_layer,
                                                       rst_buf_inst.bound_box.xl + rst_buf_inst.bound_box.w // 2,
                                                       RoundMode.NEAREST)
                rst_vm = self.connect_to_tracks(s_inst.get_pin('n' + net_name), TrackID(vm_layer, rst_vm_tidx))
                self.add_pin(net_name, rst_vm)
            elif passgate:
                rst_hm = s_inst.get_pin('n' + net_name)
                rstb_hm = m_inst.get_pin('n' + net_nameb)
                rst_vm_tidx = self.grid.coord_to_track(vm_layer, s_inst.bound_box.xh, RoundMode.NEAREST)
                rstb_vm_tidx = self.grid.coord_to_track(vm_layer, m_inst.bound_box.xh, RoundMode.NEAREST)
                rst_vm = self.connect_to_tracks(rst_hm, TrackID(vm_layer, rst_vm_tidx))
                rstb_vm = self.connect_to_tracks(rstb_hm, TrackID(vm_layer, rstb_vm_tidx))
                self.add_pin(net_name, rst_vm)
                self.add_pin(net_nameb, rstb_vm)
            else:
                rst_warr = self.connect_wires([s_inst.get_pin('n' + net_name), m_inst.get_pin('n' + net_name)])
                self.add_pin('n' + net_name, rst_warr, hide=True)
                self.add_pin('p' + net_name, rst_warr, hide=True)
                self.add_pin(net_name, [s_inst.get_pin(net_name), m_inst.get_pin(net_name)], label=net_name + ':')

        # connect mux output to flop input if scan is true

        # add pins
        # NOTE: use reexport so hidden pins propagate correctly
        self.reexport(m_inst.get_port('in'))
        self.reexport(m_inst.get_port('nin'))
        self.reexport(m_inst.get_port('pin'))
        self.reexport(s_inst.get_port('out'))
        self.reexport(s_inst.get_port('nout'))
        self.reexport(s_inst.get_port('pout'))
        self.reexport(m_inst.get_port('clkb'), net_name='clk')
        self.reexport(m_inst.get_port('nclkb'), net_name='nclk')
        self.reexport(s_inst.get_port('outb'), net_name='outb')
        self.reexport(s_inst.get_port('noutb'), net_name='noutb')
        self.reexport(s_inst.get_port('poutb'), net_name='poutb')
        if rst:
            self.reexport(m_inst.get_port('mid_vm'), net_name='mid_vm_m', hide=True)
            self.reexport(s_inst.get_port('mid_vm'), net_name='mid_vm_s', hide=True)

        # set properties
        # if rst:
        self.sch_params = dict(
            m_params=m_master.sch_params,
            s_params=s_master.sch_params,
            inv_params=inv_master.sch_params if inv_master else None,
            rst_buf_params=rst_buf_master.sch_params if rst_buf_master else None,
            rst=rst,
            resetb=self.params['resetb'],
            passgate=passgate,
            rstbuf=rstbuf,
        )


class BufferGroup(MOSBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'buffer_group')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_list='buffer seg list',
            segn_list='buffer seg list',
            bufn_index_list='Index to use 1 more stage',
            buffer_arr='Buffer arangement',
            num_per_bit='Number of buffers per bit',
            w_p='pmos width, can be list or integer if all widths are the same.',
            w_n='pmos width, can be list or integer if all widths are the same.',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            tr_manager='override track manager',
            in_rt_layer='Input routing layer',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_p=0,
            w_n=0,
            ridx_p=-1,
            ridx_n=0,
            tr_manager=None,
            bufn_index_list=[],
            in_rt_layer=3,
            segn_list=[]
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        # grid = self.grid
        seg_list: List[ImmutableList[int]] = self.params['seg_list']
        segn_list: List[ImmutableList[int]] = self.params['segn_list']
        buffer_arr: List[int] = self.params['buffer_arr']
        w_p: int = self.params['w_p']
        w_n: int = self.params['w_n']
        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']
        in_rt_layer: int = self.params['in_rt_layer']
        num_per_bit: int = self.params['num_per_bit']
        # vertical_in: bool = self.params['vertical_in']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')

        num_in_rt = sum(buffer_arr[1:]) * num_per_bit
        num_rt_ntr, in_rt_locs = tr_manager.place_wires(in_rt_layer, ['sig'] * (num_in_rt + 1), align_idx=0,
                                                        align_track=self.arr_info.col_to_track(in_rt_layer, 0))
        in_rt_locs = in_rt_locs[1:]

        if self.top_layer < vm_layer:
            raise ValueError(f'MOSBasePlaceInfo top layer must be at least {vm_layer}')

        # Placement
        # Make templates
        if len(seg_list) != sum(buffer_arr):
            raise ValueError("Number of inst doesnt match")
        buf_temp_list = []
        bufn_temp_list = []

        for idx, buf_seg in enumerate(seg_list):
            buf_params = dict(pinfo=pinfo, seg_list=buf_seg, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_out=True, vertical_sup=False, sig_locs={}, mid_int_vm=False)
            bufn_seg = buf_seg.to_list()
            bufn_seg = segn_list[idx] if segn_list else bufn_seg[:1] + bufn_seg
            buf_temp_list.append(self.new_template(InvChainCore, params=buf_params))
            bufn_params = dict(pinfo=pinfo, seg_list=bufn_seg, w_p=w_p, w_n=w_n, ridx_n=ridx_n,
                               ridx_p=ridx_p, vertical_out=True, vertical_sup=False, sig_locs={},
                               mid_int_vm=False)
            bufn_temp_list.append(self.new_template(InvChainCore, params=bufn_params))

        # Track settings

        min_sep = self.min_sep_col
        min_sep += min_sep & 1
        buf_inst_list_list = []
        start_col = self.arr_info.track_to_col(vm_layer, num_rt_ntr) if in_rt_layer < xm_layer else 0
        start_col += start_col & 1
        # start_col = 0
        num_bits = sum(buffer_arr)
        bufn_index_list = self.params['bufn_index_list']
        tot_row = len(buffer_arr) * num_per_bit
        sig_type = ['sig'] * num_per_bit
        ntr_vm, _ = tr_manager.place_wires(vm_layer, ['sig' * (num_per_bit - 1 if bufn_index_list else num_per_bit)])
        ntr_vm_ncol = self.arr_info.track_to_col(vm_layer, ntr_vm)
        ntr_vm_ncol -= ntr_vm_ncol & 1
        first_rt_locs = []
        if isinstance(bufn_index_list[0], int):
            bufn_index_list = [bufn_index_list] * num_bits

        for idx, num_col in enumerate(buffer_arr):
            _buf_list = []
            cur_col = start_col + ntr_vm_ncol if not idx else start_col
            cur_row = tot_row - idx * num_per_bit - 1
            temp_idx = sum(buffer_arr[:idx])
            for jdx in range(num_col):
                bufn_index = bufn_index_list[temp_idx + jdx]
                _buf_col = []
                max_ncol = max(buf_temp_list[temp_idx + jdx].num_cols, bufn_temp_list[temp_idx + jdx].num_cols)
                for kdx in range(num_per_bit):
                    temp = buf_temp_list[temp_idx + jdx] if kdx not in bufn_index else bufn_temp_list[temp_idx + jdx]
                    _buf_col.append(self.add_tile(temp, cur_row - kdx, cur_col + max_ncol - temp.num_cols))
                _buf_list.append(_buf_col)
                if not idx:
                    _align_col = self.arr_info.col_to_track(vm_layer, cur_col)
                    _, locs = tr_manager.place_wires(vm_layer, sig_type, align_track=_align_col,
                                                     align_idx=-1)
                    first_rt_locs.extend(locs[::-1])
                cur_col += max(buf_temp_list[temp_idx + jdx].num_cols,
                               bufn_temp_list[temp_idx + jdx].num_cols)
                cur_col += max(min_sep, ntr_vm_ncol) if not idx else min_sep
                cur_col += cur_col & 1
            buf_inst_list_list.append(_buf_list)
        self.set_mos_size()
        in_rt_locs = first_rt_locs + in_rt_locs

        tile_info = self.get_tile_info(0)[0]
        tile_bot_xm_tidx = self.grid.coord_to_track(xm_layer, 0, RoundMode.GREATER_EQ)
        tile_top_xm_tidx = self.grid.coord_to_track(xm_layer, tile_info.height, RoundMode.LESS)
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        tr_sp_xm = tr_manager.get_sep(xm_layer, ('sig', 'sig'))
        tr_sp_xm_sig_sup = tr_manager.get_sep(xm_layer, ('sig', 'sup'))
        xm_avail_locs = self.get_available_tracks(xm_layer, tile_bot_xm_tidx, tile_top_xm_tidx,
                                                  self.bound_box.xl, self.bound_box.xh, width=tr_w_xm,
                                                  sep=tr_sp_xm, sep_margin=tr_sp_xm_sig_sup, include_last=True)
        num_xm_avail = len(xm_avail_locs) - 1
        for num_row in buffer_arr:
            if num_row > num_xm_avail:
                raise ValueError("Too many buffer in one row")

        in_xm_list_list, out_xm_list_list = [], []
        inst_list = []
        for buf_idx, buf_inst_list in enumerate(buf_inst_list_list):
            num_col = len(buf_inst_list)
            for idx in range(num_per_bit):
                center_coord = buf_inst_list[0][idx].bound_box.yl + buf_inst_list[0][idx].bound_box.h // 2
                sig_type_list = ['sup'] + ['sig'] * (num_col + 1)
                _, xm_locs = tr_manager.place_wires(xm_layer, sig_type_list, center_coord=center_coord)
                xm_locs = xm_locs[1:]
                in_xm_row_list, out_xm_row_list = [], []
                for jdx, inst_col in enumerate(buf_inst_list):
                    if buf_idx or in_rt_layer == xm_layer:
                        in_vm_tidx = self.grid.coord_to_track(vm_layer, inst_col[idx].bound_box.xl, RoundMode.NEAREST)
                    else:
                        in_vm_tidx = in_rt_locs[num_per_bit * jdx + idx]
                    in_vm = self.connect_to_tracks(inst_col[idx].get_pin('nin'), TrackID(vm_layer, in_vm_tidx, tr_w_vm),
                                                   min_len_mode=MinLenMode.MIDDLE,
                                                   track_upper=self.bound_box.yh if not (
                                                               buf_idx or in_rt_layer == xm_layer) else 0)
                    if buf_idx or in_rt_layer == xm_layer:
                        _in_xm = self.connect_to_tracks(in_vm, TrackID(xm_layer, xm_locs[jdx + 1], tr_w_xm),
                                                        min_len_mode=MinLenMode.MIDDLE)
                        in_xm_row_list.append(_in_xm)
                    else:
                        in_xm_row_list.append(None)
                        self.add_pin(f'din{idx}<{jdx}>', in_vm, mode=PinMode.UPPER)
                    outvm = inst_col[idx].get_pin('out' if inst_col[idx].has_port('out') else 'outb', layer=vm_layer)
                    _out_xm = self.connect_to_tracks(outvm, TrackID(xm_layer, xm_locs[jdx], tr_w_xm),
                                                     track_upper=self.bound_box.xh, min_len_mode=MinLenMode.MIDDLE)
                    out_xm_row_list.append(_out_xm)
                    inst_list.extend(inst_col)
                in_xm_list_list.append(in_xm_row_list)
                out_xm_list_list.append(out_xm_row_list)

        in_xm_flat_list, out_xm_flat_list = [], []
        for idx in range(num_per_bit):
            in_xm_flat_list.append([w for wlist in in_xm_list_list[idx::num_per_bit] for w in wlist])
            out_xm_flat_list.append([w for wlist in out_xm_list_list[idx::num_per_bit] for w in wlist])

        # Connect input to vertical layer
        tr_w_rt = tr_manager.get_width(in_rt_layer, 'sig')
        num_bits = sum(buffer_arr)
        for idx in range(num_bits):
            for jdx in range(num_per_bit):
                if idx >= buffer_arr[0] or in_rt_layer == xm_layer:
                    rt_idx = jdx + num_per_bit * idx
                    if self.grid.get_direction(in_rt_layer) == Orient2D.y:
                        din = self.connect_to_tracks(in_xm_flat_list[jdx][idx],
                                                     TrackID(in_rt_layer, in_rt_locs[rt_idx], tr_w_rt),
                                                     track_upper=self.bound_box.yh)
                    else:
                        din = self.extend_wires(in_xm_flat_list[jdx][idx], lower=self.bound_box.xl)
                    # din = self.extend_wires(in_xm_flat_list[jdx][idx], lower=self.bound_box.xl)
                    self.add_pin(f'din{jdx}<{idx}>', din, mode=PinMode.LOWER)
                self.add_pin(f'dout{jdx}<{idx}>', out_xm_flat_list[jdx][idx])

        # Get higehr layer supplies
        vdd_xm_list, vss_xm_list = [], []
        for idx in range(self.num_tile_rows):
            _sup, _ = export_xm_sup(self, idx, export_bot=True)
            if idx & 1:
                vdd_xm_list.append(_sup)
            else:
                vss_xm_list.append(_sup)
        _, _sup = export_xm_sup(self, self.num_tile_rows - 1, export_bot=False, export_top=True)
        if self.num_tile_rows & 1:
            vdd_xm_list.append(_sup)
        else:
            vss_xm_list.append(_sup)

        # # Fill tap
        for tidx in range(self.num_tile_rows):
            fill_tap(self, tidx, extra_margin=True)

        vss_list, vdd_list = [], []
        for inst in inst_list:
            vdd_list.append(inst.get_pin('VDD'))
            vss_list.append(inst.get_pin('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.add_pin('VDD_hm', vdd_hm, label='VDD', connect=True)
        self.add_pin('VSS_hm', vss_hm, label='VSS', connect=True)
        self.add_pin('VDD_xm', vdd_xm_list, label='VDD', connect=True)
        self.add_pin('VSS_xm', vss_xm_list, label='VSS', connect=True)

        # Make param list
        name_conn_tuple_list = []
        param_list = []
        for idx in range(num_bits):
            bufn_index = bufn_index_list[idx]
            for jdx in range(num_per_bit):
                temp = buf_temp_list[idx] if jdx not in bufn_index else bufn_temp_list[idx]
                param_list.append(temp.sch_params)
                name = f'XBUF{jdx}{idx}'
                out_name = 'out' if temp.has_port('out') else 'outb'
                conn = {'in': f'din{jdx}<{idx}>', out_name: f'dout{jdx}<{idx}>', 'VDD': 'VDD', 'VSS': 'VSS'}
                name_conn_tuple_list.append((name, conn))
        self.sch_params = dict(
            param_list=param_list,
            name_conn_tuple_list=name_conn_tuple_list,
            num_bits=num_bits,
            num_per_bit=num_per_bit,
        )
