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


from typing import Any, Dict, Type, Optional, List, Tuple, Union, Sequence, Mapping

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.io import read_yaml
from bag.layout.core import PyLayInstance
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableList
from bag.util.math import HalfInt
from bag3_digital.layout.stdcells.mux import Mux2to1Matched
from bag_vco_adc.layout.util.template import TrackIDZL as TrackID
from pybag.core import BBox
from pybag.enum import RoundMode, MinLenMode, PinMode
from xbase.layout.enum import MOSWireType, SubPortMode
from xbase.layout.mos.base import MOSBase, MOSBasePlaceInfo
from .vco_flops import CnterLatch
from .vco_ring_osc import RingOscUnit
from ..digital import InvCore, XORCore, NAND2Core, InvChainCore, InvTristateCore, NOR2Core
from ..util.util import export_xm_sup, fill_tap, connect_hm_sup_ow, fill_conn_layer_intv

""" 
Generators for StrongArm flops used in VCO-based ADC

- Because the VCO is arrayed vertically, all flops are 
designed to match VCO height (1 PMOS and 1 NMOS row)
- Schematic generators reuse preamp and dynamic latch in 
SAR comparator
"""


class CnterDiv(MOSBase):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._mid_col = 0
        self._mid_sp = 0

    @property
    def mid_sp(self):
        return self._mid_sp

    @property
    def mid_col(self):
        return self._mid_col

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_div')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = CnterLatch.get_params_info()
        ans['nbits'] = 'Number of bits'
        ans['mid_sp'] = 'middle space'
        ans['shift_clk'] = 'True to shift clock for easier layout routing'
        ans['has_clkbuf'] = 'Has clock buffer'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = CnterLatch.get_default_param_values()
        ans['nbits'] = 1
        ans['mid_sp'] = 0
        ans['shift_clk'] = False
        ans['has_clkbuf'] = False
        return ans

    def draw_layout(self):
        has_clkbuf = self.params['has_clkbuf']
        seg_dict = self.params['seg_dict']
        w_dict = self.params['w_dict']
        pinfo = self.params['pinfo']
        nbits: int = self.params['nbits']
        shift_clk: int = self.params['shift_clk']
        seg_dict_list, w_dict_list = [], []
        for idx in range(4):
            _seg_dict = dict(
                nin=seg_dict['nin'][idx],
                pin=seg_dict['pin'][idx],
                ntail=seg_dict['ntail'][idx],
                ptail=seg_dict['ptail'][idx],
                nfb=seg_dict['nfb'][idx],
                pfb=seg_dict['pfb'][idx],
            )
            seg_dict_list.append(_seg_dict)
            _w_dict = dict(
                nin=w_dict['nin'][idx],
                pin=w_dict['pin'][idx],
                ntail=w_dict['ntail'][idx],
                ptail=w_dict['ptail'][idx],
                nfb=w_dict['nfb'][idx],
                pfb=w_dict['pfb'][idx],
            )
            w_dict_list.append(_w_dict)

        cnter_master_params = dict(nbits=nbits, seg_dict=seg_dict_list[0], w_dict=w_dict_list[0], pinfo=pinfo,
                                   vertical_clk=True)
        master: CnterLatch = self.new_template(CnterLatch, params=cnter_master_params)
        self.draw_base(master.draw_base_info)

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        mid_sup_vm_tidx = self.arr_info.col_to_track(vm_layer, 0)
        out_vm_tidx = tr_manager.get_next_track(vm_layer, mid_sup_vm_tidx, 'sup', 'sig')
        in_vm_tidx = tr_manager.get_next_track(vm_layer, out_vm_tidx, 'sig', 'sig')

        sig_locs_orig = {'out': in_vm_tidx, 'in': out_vm_tidx}
        sig_locs_new = {'out': out_vm_tidx, 'in': in_vm_tidx}
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')

        # Make templates
        min_sep = self.min_sep_col
        cnter_master_params = dict(nbits=nbits, seg_dict=seg_dict_list[0], w_dict=w_dict_list[0], pinfo=pinfo,
                                   sig_locs=sig_locs_orig, vertical_clk=False)
        master: CnterLatch = self.new_template(CnterLatch, params=cnter_master_params)
        master_w_double_shift_params = dict(nbits=nbits, seg_dict=seg_dict_list[1], w_dict=w_dict_list[1],
                                            pinfo=pinfo, flip_io=True, sig_locs=sig_locs_new, vertical_clk=False)
        master_w_double_shift = self.new_template(CnterLatch, params=master_w_double_shift_params)
        master_seg_double_params = dict(nbits=nbits, seg_dict=seg_dict_list[2], w_dict=w_dict_list[2], pinfo=pinfo,
                                        sig_locs=sig_locs_orig, vertical_clk=False)
        master_seg_double = self.new_template(CnterLatch, params=master_seg_double_params)
        master_both_double_shift_params = dict(nbits=nbits, seg_dict=seg_dict_list[3], w_dict=w_dict_list[3],
                                               pinfo=pinfo, flip_io=True, sig_locs=sig_locs_new, vertical_clk=False)
        master_both_double_shift = self.new_template(CnterLatch, params=master_both_double_shift_params)

        # Placement
        master_list = [master, master_w_double_shift, master_seg_double, master_both_double_shift]
        l_lat_list, r_lat_list = [], []
        nrow = 2 ** (nbits - 1)
        ncol_lat = master.num_cols
        ncol_lat2 = master_list[-1].num_cols

        # Routing
        clk_vm_ntr, _ = self.tr_manager.place_wires(vm_layer, ['clk'] * (6 if has_clkbuf else 4))
        lat_sep = self.arr_info.get_column_span(vm_layer, clk_vm_ntr)
        lat_sep += (lat_sep & 1)
        if has_clkbuf:
            g_tidx = (self.get_track_index(0, MOSWireType.G, 'sig', 0) +
                      self.get_track_index(-1, MOSWireType.G, 'sig', 0)).div2()
            seg_inv, w_inv = seg_dict['inv'], w_dict['inv']
            if isinstance(seg_inv, ImmutableList) and len(seg_inv) > 1:
                params = dict(seg_list=seg_inv, w_p=w_inv['wp'], w_n=w_inv['wn'], pinfo=pinfo, vertical_out=False,
                              sig_locs={'nin': g_tidx}, vertical_sup=True)
                inv_template = self.new_template(InvChainCore, params=params)
            else:
                params = dict(seg=seg_inv[0] if isinstance(seg_inv, ImmutableList) else seg_inv,
                              w_p=w_inv['wp'], w_n=w_inv['wn'], pinfo=pinfo, vertical_out=False,
                              sig_locs={'nin': g_tidx}, vertical_sup=True)
                inv_template = self.new_template(InvCore, params=params)
            lat_sep = max(lat_sep, inv_template.num_cols + 2 * min_sep)
            invn = self.add_tile(inv_template, 0, ncol_lat + lat_sep // 2 - inv_template.num_cols // 2)
            invp = self.add_tile(inv_template, 1, ncol_lat + lat_sep // 2 - inv_template.num_cols // 2)
            connect_hm_sup_ow(self, invn.get_all_port_pins('VDD'), 0, -1, tr_manager)
            connect_hm_sup_ow(self, invn.get_all_port_pins('VSS'), 0, 0, tr_manager)
            connect_hm_sup_ow(self, invp.get_all_port_pins('VDD'), 1, -1, tr_manager)
            connect_hm_sup_ow(self, invp.get_all_port_pins('VSS'), 1, 0, tr_manager)
            _, clk_tidxs = self.tr_manager.place_wires(vm_layer, ['clk', 'clk', 'clk', 'dum', 'clk', 'clk', 'clk'],
                                                       center_coord=self.arr_info.col_to_coord(ncol_lat + lat_sep // 2))
            clk_tidxs.pop(3)
        else:
            lat_sep = max(lat_sep, self.params['mid_sp'])
            _, clk_tidxs = self.tr_manager.place_wires(vm_layer, ['clk', 'clk', 'dum', 'clk', 'clk'],
                                                       center_coord=self.arr_info.col_to_coord(ncol_lat + lat_sep // 2))
            # clk_tidxs.pop(2)
            # clk_tidxs.pop(2)
            inv_template = None
            invn, invp = None, None

        self._mid_sp = lat_sep
        if nrow <= 2:
            for ridx in range(nrow):
                l_lat_list.append(self.add_tile(master_list[ridx], ridx, ncol_lat if ridx & 1 else 0,
                                                flip_lr=bool(ridx & 1)))

                r_lat_list.append(self.add_tile(master_list[ridx + nrow], nrow - ridx - 1,
                                                ncol_lat2 + ncol_lat + lat_sep if ridx & 1 else ncol_lat + lat_sep,
                                                flip_lr=bool(ridx & 1)))
        else:
            # FIXME: This part is not updated, might have issues
            pass

        self.set_mos_size()

        # Connect clock signals
        nclk_list, pclk_list = [], []
        vdd_list, vss_list = [], []
        vdd_hm_list, vss_hm_list = [], []
        for idx in range(nrow):
            pinfo, yb, _ = self.get_tile_info(idx)
            _, io_xm_locs = tr_manager.place_wires(xm_layer, ['sig'] * 4, center_coord=yb + pinfo.height // 2)
            if nrow > 1:
                io_xm_locs_0, io_xm_locs_1 = io_xm_locs[1:-1], io_xm_locs[1:-1]
            else:
                io_xm_locs_0, io_xm_locs_1 = io_xm_locs[1:-1], [io_xm_locs[0], io_xm_locs[-1]]

            if idx == nrow - 1:
                self.connect_matching_tracks([[l_lat_list[idx].get_pin('outp', layer=vm_layer),
                                               r_lat_list[-idx - 1].get_pin('d')],
                                              [l_lat_list[idx].get_pin('outn', layer=vm_layer),
                                               r_lat_list[-idx - 1].get_pin('dn')]],
                                             xm_layer, io_xm_locs_0, width=tr_manager.get_width(xm_layer, 'sig'))
            if idx == 0:
                outn_xm, outp_xm = \
                    self.connect_matching_tracks([[l_lat_list[idx].get_pin('d'),
                                                   r_lat_list[-idx - 1].get_pin('outn', layer=vm_layer)],
                                                  [l_lat_list[idx].get_pin('dn'),
                                                   r_lat_list[-idx - 1].get_pin('outp', layer=vm_layer)]],
                                                 xm_layer, io_xm_locs_1, width=tr_manager.get_width(xm_layer, 'sig'))
                self.add_pin(f'outn<{2 ** nbits - 1}>', outn_xm)
                self.add_pin(f'outp<{2 ** nbits - 1}>', outp_xm)
            if nrow > 1:
                pinst, ninst = (r_lat_list[idx], l_lat_list[-1 - idx]) if idx & 1 else (
                    l_lat_list[-1 - idx], r_lat_list[idx])
            else:
                pinst, ninst = (r_lat_list[idx], l_lat_list[-1 - idx])

            nclk_list.extend([ninst.get_pin('clkn'), pinst.get_pin('clkp')])
            pclk_list.extend([ninst.get_pin('clkp'), pinst.get_pin('clkn')])
            vdd_list.extend(self.connect_wires([ninst.get_pin('VDD'), pinst.get_pin('VDD')]))
            vss_list.extend(self.connect_wires([ninst.get_pin('VSS'), pinst.get_pin('VSS')]))
            vdd_hm_list.extend(self.connect_wires([ninst.get_pin('VDD_hm'), pinst.get_pin('VDD_hm')]))
            vss_hm_list.extend(self.connect_wires([ninst.get_pin('VSS_hm'), pinst.get_pin('VSS_hm')]))

        for idx in range(nrow - 1):
            self.connect_wires([r_lat_list[idx].get_pin('outn', layer=vm_layer), r_lat_list[idx + 1].get_pin('dn')])
            self.connect_wires([r_lat_list[idx].get_pin('outp', layer=vm_layer), r_lat_list[idx + 1].get_pin('d')])
            self.connect_wires([l_lat_list[idx].get_pin('outn', layer=vm_layer), l_lat_list[idx + 1].get_pin('dn')])
            self.connect_wires([l_lat_list[idx].get_pin('outp', layer=vm_layer), l_lat_list[idx + 1].get_pin('d')])

        if has_clkbuf:
            self.connect_matching_tracks([[invn.get_pin('nout'), invn.get_pin('pout')],
                                          [invp.get_pin('nout'), invp.get_pin('pout')]], vm_layer,
                                         [clk_tidxs[2], clk_tidxs[3]], width=tr_w_clk_vm)
            self.connect_matching_tracks([pclk_list, nclk_list], vm_layer,
                                         [clk_tidxs[2], clk_tidxs[3]], width=tr_w_clk_vm)
            clk_tid_pair = [clk_tidxs[0], clk_tidxs[-1]] if shift_clk else [clk_tidxs[1], clk_tidxs[-2]]
            nclk, pclk = self.connect_matching_tracks([[invn.get_pin('nin')], [invp.get_pin('nin')]],
                                                      vm_layer, clk_tid_pair, width=tr_w_clk_vm)

        else:
            clk_tid_pair = [clk_tidxs[0], clk_tidxs[-1]] if shift_clk else [clk_tidxs[1], clk_tidxs[-2]]
            nclk, pclk = self.connect_matching_tracks([nclk_list, pclk_list], vm_layer, clk_tid_pair, width=tr_w_clk_vm)

        self.add_pin('VDD', vdd_list, connect=True)
        self.add_pin('VSS', vss_list, connect=True)
        self.add_pin('clkn', nclk)
        self.add_pin('clkp', pclk)

        for idx in range(nrow):
            self.reexport(l_lat_list[idx].get_port('outp'), net_name=f'outp<{idx}>')
            self.reexport(l_lat_list[idx].get_port('outn'), net_name=f'outn<{idx}>')
            self.reexport(r_lat_list[-idx - 1].get_port('outp'), net_name=f'outp<{2 ** nbits - 1 - idx}>')
            self.reexport(r_lat_list[-idx - 1].get_port('outn'), net_name=f'outn<{2 ** nbits - 1 - idx}>')

        self._mid_col = master.num_cols + min_sep // 2
        self.sch_params = dict(
            latch_params_list=[m.sch_params for m in master_list],
            num_stages=2 ** nbits,
            clkbuf=inv_template.sch_params if has_clkbuf else None
        )


class CnterAsync(CnterDiv):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._mid_col = 0

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_async')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = CnterDiv.get_params_info()
        ans['ndivs'] = 'Number of dividers'
        ans['export_output'] = 'True to export final output'
        ans['top_sup_layer'] = 'Top supply layer'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = CnterDiv.get_default_param_values()
        ans['ndivs'] = 1
        ans['export_output'] = False
        ans['top_sup_layer'] = 6
        return ans

    def draw_layout(self):
        master: CnterDiv = self.new_template(CnterDiv, params=self.params)
        self._mid_col = master.mid_col
        self.draw_base(master.draw_base_info)
        ndivs: int = self.params['ndivs']
        nbits: int = self.params['nbits']

        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        # placement
        div_nrow = master.num_tile_rows
        params = self.params.copy(append=dict(has_clkbuf=True))
        master_clkbuf: CnterDiv = self.new_template(CnterDiv, params=params)
        div_mid_sp = master_clkbuf.mid_sp
        params = params.copy(append=dict(shift_clk=True))
        master_shift_clkbuf: CnterDiv = self.new_template(CnterDiv, params=params)
        first_params = self.params.copy(append=dict(mid_sp=div_mid_sp))
        first_master: CnterDiv = self.new_template(CnterDiv, params=first_params)

        master_list = [first_master] + [master_shift_clkbuf if ridx & 1 else master_clkbuf for ridx in range(1, ndivs)]
        div_list = []
        for ridx in range(ndivs):
            div_list.append(self.add_tile(master_list[ridx], (ndivs - ridx - 1) * div_nrow, 0))

        self.set_mos_size(max([inst_temp.num_cols for inst_temp in master_list]))

        vdd_list, vss_list, nclk_list, pclk_list = [], [], [], []
        for idx, inst in enumerate(div_list):
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
            for b in range(2 ** nbits):
                self.reexport(inst.get_port(f'outn<{b}>'), net_name=f'outn<{b + idx * 2 ** nbits}>')
                self.reexport(inst.get_port(f'outp<{b}>'), net_name=f'outp<{b + idx * 2 ** nbits}>')
            if idx < ndivs - 1:
                outn = div_list[idx].get_pin(f'outn<{2 ** nbits - 1}>', layer=xm_layer)
                outp = div_list[idx].get_pin(f'outp<{2 ** nbits - 1}>', layer=xm_layer)
                clkn = div_list[idx + 1].get_pin('clkn')
                clkp = div_list[idx + 1].get_pin('clkp')
                self.connect_differential_wires(outn, outp, clkn, clkp)
        self.reexport(div_list[0].get_port('clkn'))
        self.reexport(div_list[0].get_port('clkp'))

        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')

        export_output = self.params['export_output']
        if export_output:
            if len(div_list) > 1:
                final_output_tid = [div_list[-2].get_pin('clkn').track_id, div_list[-2].get_pin('clkp').track_id]
            else:
                final_output_tid = [
                    tr_manager.get_next_track(vm_layer, div_list[-2].get_pin('clkn').track_id.base_index,
                                              'clk', 'clk', up=False),
                    tr_manager.get_next_track(vm_layer, div_list[-2].get_pin('clkp').track_id.base_index,
                                              'clk', 'clk', up=True)]
                tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')
                final_output_tid = [TrackID(vm_layer, final_output_tid[0], tr_w_clk_vm),
                                    TrackID(vm_layer, final_output_tid[1], tr_w_clk_vm)]

            final_outn = self.connect_to_tracks(div_list[-1].get_pin(f'outn<{2 ** nbits - 1}>', layer=hm_layer),
                                                final_output_tid[0], min_len_mode=MinLenMode.MIDDLE)
            final_outp = self.connect_to_tracks(div_list[-1].get_pin(f'outp<{2 ** nbits - 1}>', layer=hm_layer),
                                                final_output_tid[1], min_len_mode=MinLenMode.MIDDLE)
            self.add_pin('final_outn', final_outn, hide=True)
            self.add_pin('final_outp', final_outp, hide=True)

        self.add_pin('VDD_xm', vdd_list, label='VDD', connect=True)
        self.add_pin('VSS_xm', vss_list, label='VSS', connect=True)
        if self.params['top_sup_layer'] > 4:
            tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')

            ym_tid_l = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.GREATER_EQ)
            ym_tid_r = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.LESS_EQ)
            num_ym_sup = tr_manager.get_num_wires_between(ym_layer, 'dum', ym_tid_l, 'dum', ym_tid_r, 'sup')
            _, ym_sup_tidxs = tr_manager.place_wires(ym_layer, ['dum'] + ['sup'] * num_ym_sup + ['dum'],
                                                     center_coord=self.bound_box.w // 2)
            ym_sup_tidxs = ym_sup_tidxs[1:-1]

            vdd_ym = [self.connect_to_tracks(vdd_list, TrackID(ym_layer, tid, tr_w_sup_ym))
                      for tid in ym_sup_tidxs[::2]]
            vss_ym = [self.connect_to_tracks(vss_list, TrackID(ym_layer, tid, tr_w_sup_ym))
                      for tid in ym_sup_tidxs[1::2]]
            self.add_pin('VDD_ym', vdd_ym, label='VDD', connect=True)
            self.add_pin('VSS_ym', vss_ym, label='VSS', connect=True)
            if self.params['top_sup_layer'] > 5:
                vss_xm1_list, vdd_xm1_list = [], []
                for vss in vss_list:
                    xm1_tidx = self.grid.coord_to_track(xm1_layer,
                                                        self.grid.track_to_coord(xm_layer, vss.track_id.base_index),
                                                        mode=RoundMode.NEAREST)
                    vss_xm1 = self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx, tr_w_sup_xm1,
                                                                     grid=self.grid),
                                                     track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
                    vss_xm1_list.append(vss_xm1)

                for vdd in vdd_list:
                    xm1_tidx = self.grid.coord_to_track(xm1_layer,
                                                        self.grid.track_to_coord(xm_layer, vdd.track_id.base_index),
                                                        mode=RoundMode.NEAREST)
                    vdd_xm1 = self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx, tr_w_sup_xm1,
                                                                     grid=self.grid),
                                                     track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
                    vdd_xm1_list.append(vdd_xm1)
                self.add_pin('VDD', vdd_xm1_list, connect=True)
                self.add_pin('VSS', vss_xm1_list, connect=True)

        self.sch_params = dict(
            div_params_list=[temp.sch_params for temp in master_list],
            nbits=nbits,
            ndivs=ndivs
        )


class CnterBuffer(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_buffer')

    @property
    def center_col(self) -> int:
        return self._center_col

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            stack_dict='Number of stack',
            w_dict='Width',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            shift_out='shift coupler output',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ridx_p=-1,
            ridx_n=0,
            shift_out=False,
        )

    def draw_cap(self, seg):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)

        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        ns_conn, nd_conn, _ = fill_conn_layer_intv(self, 0, 0, extend_to_gate=False,
                                                   start_col=0, stop_col=seg + 1)
        nd_conn = [w for warr in nd_conn for w in warr.to_warr_list()]

        ps_conn, pd_conn, _ = fill_conn_layer_intv(self, 0, -1, extend_to_gate=False,
                                                   start_col=0, stop_col=seg + 1)
        pd_conn = [w for warr in pd_conn for w in warr.to_warr_list()]
        ps_conn = [w for warr in ps_conn for w in warr.to_warr_list()]
        nd_top0 = self.get_track_id(0, MOSWireType.DS, 'sup')
        pd_top0 = self.get_track_id(-1, MOSWireType.DS, 'sup')
        g_mid_tidx = self.grid.get_middle_track(nd_top0.base_index, pd_top0.base_index)
        g_mid = TrackID(hm_layer, g_mid_tidx)

        tile_height = self.get_tile_pinfo(0).height
        nd_bot = TrackID(hm_layer, self.grid.coord_to_track(hm_layer, tile_height // 4, RoundMode.NEAREST))
        pd_bot = TrackID(hm_layer, self.grid.coord_to_track(hm_layer, 3 * tile_height // 4, RoundMode.NEAREST))

        top_hm = [self.connect_to_tracks(ns_conn + ps_conn, _tid) for _tid in [g_mid]]
        bot_hm = [self.connect_to_tracks(nd_conn + pd_conn, pd_bot),
                  self.connect_to_tracks(nd_conn + pd_conn, nd_bot)]
        d_vm_tidx = [self.arr_info.col_to_track(vm_layer, idx) for idx in range(1, seg + 1, 2)]
        bot_vm = []
        for idx, vmtidx in enumerate(d_vm_tidx):
            bot_vm.append(self.connect_to_tracks(bot_hm, TrackID(vm_layer, vmtidx),
                                                 track_upper=pd_conn[0].upper,
                                                 track_lower=nd_conn[0].lower))
        d_vm_tidx = [self.arr_info.col_to_track(vm_layer, idx) for idx in range(0, seg + 1, 2)]
        top_vm = []
        for idx, vmtidx in enumerate(d_vm_tidx):
            top_vm.append(self.connect_to_tracks(top_hm, TrackID(vm_layer, vmtidx),
                                                 track_upper=pd_conn[0].upper,
                                                 track_lower=nd_conn[0].lower))
        top_xm_coord = self.grid.track_to_coord(hm_layer, top_hm[0].track_id.base_index)
        bot_xm_coord = [self.grid.track_to_coord(hm_layer, tid.track_id.base_index) for tid in bot_hm]

        top_xm_tidx = self.grid.coord_to_track(xm_layer, top_xm_coord, RoundMode.NEAREST)
        bot_xm_tidx = [self.grid.coord_to_track(xm_layer, coord, RoundMode.NEAREST) for coord in bot_xm_coord]
        tr_w_cap_xm = self.tr_manager.get_width(xm_layer, 'cap')
        top_xm = [self.connect_to_tracks(top_vm, TrackID(xm_layer, top_xm_tidx, tr_w_cap_xm))]
        bot_xm = [self.connect_to_tracks(bot_vm, TrackID(xm_layer, idx, tr_w_cap_xm)) for idx in bot_xm_tidx]
        vm_min_len = self.grid.get_next_length(vm_layer, 1, self.grid.get_wire_total_width(vm_layer, 1), even=True) // 2

        top_bbox = BBox(top_vm[0].bound_box.xl, top_vm[0].bound_box.yl, top_vm[0].bound_box.xh,
                        top_vm[0].bound_box.yl + vm_min_len)
        bot_bbox = BBox(bot_vm[0].bound_box.xl, bot_vm[0].bound_box.yl, bot_vm[0].bound_box.xh,
                        bot_vm[0].bound_box.yl + vm_min_len)
        self.add_res_metal(vm_layer, bot_bbox)
        self.add_res_metal(vm_layer, top_bbox)

        return top_xm, bot_xm, (top_xm[0].bound_box.h, self.grid.resolution)

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        seg_dict: Dict[str, int] = self.params['seg_dict']
        stack_dict: Dict[str, int] = self.params['stack_dict']
        w_dict: Dict[str, int] = self.params['w_dict']

        seg_cap = seg_dict['cap']
        seg_res = seg_dict['res']
        seg_tia_p = seg_dict['tiap']
        seg_tia_n = seg_dict['tian']
        seg_buf_p = seg_dict['bufp']
        seg_buf_n = seg_dict['bufn']
        seg_coupler_p = seg_dict['couplerp']
        seg_coupler_n = seg_dict['couplern']

        wres = w_dict['wres']
        wn = w_dict['wn']
        wp = w_dict['wp']
        wn_buf = w_dict['wn_buf']
        wp_buf = w_dict['wp_buf']
        wn_coupler = w_dict['wn_coupler']
        wp_coupler = w_dict['wp_coupler']

        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']

        tr_manager = self.tr_manager
        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        min_sep = self.min_sep_col

        nd0 = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nd1 = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pd0 = self.get_track_id(-1, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pd1 = self.get_track_id(-1, MOSWireType.DS, wire_name='sig', wire_idx=0)

        ng0_tidx = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = tr_manager.get_next_track(hm_layer, ng0_tidx, 'sig', 'sig', up=1)
        pg0_tidx = self.get_track_index(-1, MOSWireType.G, wire_name='sig', wire_idx=0)
        mid_g_tidx = self.grid.get_middle_track(nd1.base_index, pd1.base_index, round_up=True)

        # Calculate segments
        cur_loc = 0
        cap_top_xm, cap_bot_xm, res_metal = self.draw_cap(seg_cap)

        cur_loc += seg_cap + min_sep
        resn = self.add_mos(ridx_n, cur_loc, seg=seg_res, w=wres, stack=stack_dict['res'])
        resp = self.add_mos(ridx_p, cur_loc, seg=seg_res, w=wres, stack=stack_dict['res'])

        cur_loc += seg_res * stack_dict['res'] + min_sep
        tiap = self.add_mos(ridx_p, cur_loc, seg=seg_tia_p, w=wp)
        tian = self.add_mos(ridx_n, cur_loc, seg=seg_tia_n, w=wn)

        cur_loc += max(seg_tia_p, seg_tia_n)
        buf0p = self.add_mos(ridx_p, cur_loc, seg=seg_buf_p, w=wp_buf)
        buf0n = self.add_mos(ridx_n, cur_loc, seg=seg_buf_n, w=wn_buf)

        cur_loc += max(seg_buf_n, seg_buf_p)
        couplerp = self.add_mos(ridx_p, cur_loc, seg=seg_coupler_p, w=wp_coupler)
        couplern = self.add_mos(ridx_n, cur_loc, seg=seg_coupler_n, w=wn_coupler)
        self.set_mos_size()

        # Connections
        tr_w_sig_hm = tr_manager.get_width(hm_layer, 'sig')
        ng0, ng1, pg0 = [TrackID(hm_layer, tidx, tr_w_sig_hm) for tidx in [ng0_tidx, ng1_tidx, pg0_tidx]]
        d_warr_s = self.connect_to_tracks([tian.g, tiap.g], TrackID(hm_layer, mid_g_tidx, tr_w_sig_hm))
        d_warr_res_p = self.connect_to_tracks(resp.d, pd1)
        d_warr_res_n = self.connect_to_tracks(resn.d, nd1)

        dout_warr_p = self.connect_to_tracks([resp.s, tiap.d], pd0)
        dout_warr_n = self.connect_to_tracks([resn.s, tian.d], nd0)
        dout_warr_g = self.connect_to_tracks([buf0n.g, buf0p.g], pg0)

        outp_g = self.connect_to_tracks([couplerp.g, couplern.g], ng0)
        outp_p = self.connect_to_tracks(buf0p.d, pd0)
        outp_n = self.connect_to_tracks(buf0n.d, nd0)
        outn_p = self.connect_to_tracks(couplerp.d, pd1)
        outn_n = self.connect_to_tracks(couplern.d, nd1)
        vm_tidx_list = [self.grid.coord_to_track(vm_layer, warr.middle, mode=RoundMode.NEAREST) for
                        warr in [d_warr_res_p, d_warr_s, outp_n, outn_n]]

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        d_res_vm = self.connect_to_tracks([d_warr_res_n, d_warr_res_p],
                                          TrackID(vm_layer, vm_tidx_list[0], tr_w_sig_vm))
        dout_vm = self.connect_to_tracks([dout_warr_n, dout_warr_g, dout_warr_p],
                                         TrackID(vm_layer, vm_tidx_list[1], tr_w_sig_vm))
        outp_vm = self.connect_to_tracks([outp_g, outp_n, outp_p], TrackID(vm_layer, vm_tidx_list[2], tr_w_sig_vm))
        if self.params['shift_out']:
            _vm_tidx = tr_manager.get_next_track(vm_layer, vm_tidx_list[3], 'sig', 'sig')
            outn_vm = self.connect_to_tracks([outn_n, outn_p], TrackID(vm_layer, _vm_tidx, tr_w_sig_vm))
        else:
            outn_vm = self.connect_to_tracks([outn_n, outn_p], TrackID(vm_layer, vm_tidx_list[3], tr_w_sig_vm))

        res_g_vm_tid = tr_manager.get_next_track(vm_layer, vm_tidx_list[0], 'sig', 'sup', up=-1)
        resp_g_hm = self.connect_to_tracks(resp.g, pg0)
        resp_g_vm = self.connect_to_tracks(resp_g_hm, TrackID(vm_layer, res_g_vm_tid, tr_w_sig_vm))

        res_g_vm_tid = tr_manager.get_next_track(vm_layer, res_g_vm_tid, 'sig', 'sup', up=-1)
        resn_g_hm = self.connect_to_tracks(resn.g, ng0)
        resn_g_vm = self.connect_to_tracks(resn_g_hm, TrackID(vm_layer, res_g_vm_tid, tr_w_sig_vm))

        d_vm_tidx = tr_manager.get_next_track(vm_layer, vm_tidx_list[1], 'sig', 'sig', up=False)
        d_vm = self.connect_to_tracks(d_warr_s, TrackID(vm_layer, d_vm_tidx, tr_w_sig_vm))
        self.connect_to_track_wires([d_vm, d_res_vm], cap_top_xm)

        # Connect supplies
        vss_conn = [couplern.s, tian.s, buf0n.s]
        vdd_conn = [couplerp.s, tiap.s, buf0p.s]
        vss = connect_hm_sup_ow(self, vss_conn, 0, ridx_n, tr_manager)
        vdd = connect_hm_sup_ow(self, vdd_conn, 0, ridx_p, tr_manager)
        vss = self.extend_wires(vss, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd = self.extend_wires(vdd, lower=self.bound_box.xl, upper=self.bound_box.xh)

        self.connect_to_track_wires(resp_g_vm, vss)
        self.connect_to_track_wires(resn_g_vm, vdd)
        self.add_pin('in', cap_bot_xm)
        self.add_pin('d', cap_top_xm)
        self.add_pin('outp', outp_vm)
        self.add_pin('outp_hm', outp_g, hide=True)
        self.add_pin('dout', dout_vm)
        self.add_pin('outn', outn_vm)
        self.add_pin('VDD', vdd)
        self.add_pin('VSS', vss)

        thp = self.place_info.get_row_place_info(ridx_p).row_info.threshold
        thn = self.place_info.get_row_place_info(ridx_n).row_info.threshold

        self.sch_params = dict(
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            w_dict=w_dict,
            stack_dict=stack_dict,
            thn=thn,
            thp=thp,
            cap_params=dict(
                res_plus=dict(layer=vm_layer, w=res_metal[0] // 2, l=res_metal[1]),
                res_minus=dict(layer=vm_layer, w=res_metal[0] // 2, l=res_metal[1]),
                same_net_top=True,
                same_net_bot=True,
                cap=1e-15,
            )
        )


class MuxCore(MOSBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag3_digital', 'mux2to1_matched')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='number of segments for the tristate inverter',
            w_p='pmos width, can be list or integer if all widths are the same.',
            w_n='pmos width, can be list or integer if all widths are the same.',
            ridx_p='pmos row index.',
            ridx_n='nmos row index.',
            sig_locs='Optional dictionary of user defined signal locations',
            vertical_out='True to have output on vertical layer',
            vertical_sup='True to add vertical supply'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_p=0,
            w_n=0,
            ridx_p=-1,
            ridx_n=0,
            sig_locs=None,
            vertical_out=True,
            vertical_sup=False,
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        if self.arr_info.top_layer < vm_layer:
            raise ValueError(f'MOSBasePlaceInfo top layer must be at least {vm_layer}')

        seg_dict: Dict[str, int] = self.params['seg_dict']
        w_p: Union[int, Sequence[int]] = self.params['w_p']
        w_n: Union[int, Sequence[int]] = self.params['w_n']
        ridx_p: int = self.params['ridx_p']
        ridx_n: int = self.params['ridx_n']
        sig_locs: Optional[Dict[str, float]] = self.params['sig_locs']
        vertical_out: bool = self.params['vertical_out']
        seg = seg_dict['tri']
        inv_seg = seg_dict['buf']

        if seg % 2 != 0:
            raise ValueError(f'Mux2to1: seg = {seg} is not even')
        if sig_locs is None:
            sig_locs = {}

        en_tidx = sig_locs.get('nen', self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig',
                                                           wire_idx=0))
        in0_tidx = sig_locs.get('nin0', self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig',
                                                             wire_idx=1))
        in1_tidx = sig_locs.get('pin1', self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig',
                                                             wire_idx=2))
        enb_tidx0 = sig_locs.get('penb', self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig',
                                                              wire_idx=0))
        enb_tidx1 = sig_locs.get('penb', self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig',
                                                              wire_idx=1))
        tristate0_params = dict(pinfo=pinfo, seg=seg, w_p=w_p, w_n=w_n, ridx_p=ridx_p,
                                ridx_n=ridx_n, vertical_out=False, vertical_sup=self.params['vertical_sup'],
                                sig_locs={'nen': en_tidx, 'nin': in0_tidx, 'pen': enb_tidx0})
        tristate0_master = self.new_template(InvTristateCore, params=tristate0_params)
        tristate1_params = dict(pinfo=pinfo, seg=seg, w_p=w_p, w_n=w_n, ridx_p=ridx_p,
                                ridx_n=ridx_n, vertical_out=False, vertical_sup=self.params['vertical_sup'],
                                sig_locs={'nen': en_tidx, 'nin': in1_tidx, 'pen': enb_tidx1})
        tristate1_master = self.new_template(InvTristateCore, params=tristate1_params)

        in_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        out_inv_sig_locs = {'pin': in_tidx}
        for key in ('pout', 'nout'):
            if key in sig_locs:
                out_inv_sig_locs[key] = sig_locs[key]
        out_inv_params = dict(pinfo=pinfo, seg=inv_seg, w_p=w_p, w_n=w_n, ridx_p=ridx_p,
                              vertical_sup=self.params['vertical_sup'],
                              ridx_n=ridx_n, sig_locs=out_inv_sig_locs, vertical_out=vertical_out)
        out_inv_master = self.new_template(InvCore, params=out_inv_params)

        tristate_ncols = tristate0_master.num_cols
        out_inv_ncols = out_inv_master.num_cols
        sep = max(self.get_hm_sp_le_sep_col(), self.min_sep_col)

        # --- Placement --- #
        cur_col = 0
        t0 = self.add_tile(tristate0_master, 0, cur_col)
        cur_col += tristate_ncols + sep
        t1 = self.add_tile(tristate1_master, 0, cur_col)
        cur_col += tristate_ncols + sep
        out_inv = self.add_tile(out_inv_master, 0, cur_col)
        cur_col += out_inv_ncols

        self.set_mos_size()

        # --- Routing --- #
        tr_manager = self.tr_manager
        tr_w_v = tr_manager.get_width(vm_layer, 'sig')
        # vdd/vss
        vdd_list, vss_list = [], []
        inst_arr = [t0, t1, out_inv, t1]
        for inst in inst_arr:
            vdd_list += inst.get_all_port_pins('VDD')
            vss_list += inst.get_all_port_pins('VSS')

        vdd_list = self.connect_wires(vdd_list)
        vss_list = self.connect_wires(vss_list)

        t0_en = t0.get_pin('en')
        t1_en = t1.get_pin('en')
        l_en_tidx = self.grid.coord_to_track(vm_layer, t0_en.lower, RoundMode.LESS_EQ)
        selb_vm = self.connect_to_tracks(t0_en, TrackID(vm_layer, l_en_tidx, width=tr_w_v))

        r_en_tidx = self.grid.coord_to_track(vm_layer, t1_en.upper, RoundMode.GREATER_EQ)
        sel_vm = self.connect_to_tracks(t1_en, TrackID(vm_layer, r_en_tidx, width=tr_w_v))
        # sel_vms.append(sel_vm)

        # connect right enb and left en differentially
        t0_enb = t0.get_pin('enb')
        t1_enb = t1.get_pin('enb')
        sel_vm, selb_vm = self.connect_differential_wires(t0_enb, t1_enb, sel_vm, selb_vm)
        # connect outb to out
        if vertical_out:
            out_idx = out_inv.get_pin('out').track_id.base_index
            mux_out_idx = tr_manager.get_next_track(vm_layer, out_idx, 'sig', 'sig', up=False)
        else:
            out_hm = out_inv.get_pin('nout')
            mux_out_idx = self.grid.coord_to_track(vm_layer, out_hm.middle,
                                                   mode=RoundMode.NEAREST)
        mux_out_warrs = [t0.get_pin('nout'), t0.get_pin('pout'), t1.get_pin('nout'),
                         t1.get_pin('pout'), out_inv.get_pin('nin')]

        self.connect_to_tracks(mux_out_warrs, TrackID(vm_layer, mux_out_idx, width=tr_w_v))
        self.extend_wires(mux_out_warrs, upper=(out_inv.bound_box.xl + out_inv.bound_box.xh) // 2)

        # add pins
        self.add_pin('VDD', vdd_list)
        self.add_pin('VSS', vss_list)
        self.add_pin('sel', sel_vm)
        self.add_pin('selb', selb_vm)
        in0 = t0.get_pin('nin')
        in1 = t1.get_pin('pin')
        in0, in1 = self.extend_wires([in0, in1], upper=max(in0.upper, in1.upper), lower=min(in0.lower, in1.lower))
        self.add_pin('in<0>', in0)
        self.add_pin('in<1>', in1)
        if vertical_out:
            self.reexport(out_inv.get_port('out'), net_name='out')
        self.reexport(out_inv.get_port('pout'), label='out:', hide=vertical_out)
        self.reexport(out_inv.get_port('nout'), label='out:', hide=vertical_out)

        self.sch_params = dict(inv_params=out_inv_master.sch_params,
                               tri_params=tristate0_master.sch_params)


class CntDecUnit(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_decoder')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            first_dec='Is first decoder'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            first_dec=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        first_dec: bool = self.params['first_dec']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_mux: Dict[str, int] = seg_dict['mux']
        seg_nand: bool = seg_dict['nand']
        seg_inv: bool = seg_dict['inv']
        seg_xor: bool = seg_dict['xor']
        seg_selbuf: bool = seg_dict['sel_buf']
        min_sep = self.min_sep_col

        # compute track locations
        tr_manager = self.tr_manager
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        ng3_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=3)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)

        _, vm_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 3)

        xor_params = dict(pinfo=pinfo, seg=seg_xor, w_p=w_p, w_n=w_n, ridx_n=ridx_n,
                          ridx_p=ridx_p, vertical_sup=False, sig_locs={})
        mux_params = dict(pinfo=pinfo, seg_dict=seg_mux, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False)
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n,
                          ridx_p=ridx_p, vertical_sup=False, sig_locs={'nin': ng3_tidx})
        inv_and_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              vertical_sup=False, sig_locs={'nin': ng1_tidx})
        selbuf_params = dict(pinfo=pinfo, seg=seg_selbuf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             vertical_sup=False, sig_locs={'nin': pg1_tidx})
        nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=False, sig_locs={'nin0': ng2_tidx, 'nin1': ng3_tidx, 'out': vm_tidx[-1]})
        nor_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False, sig_locs={'nin0': ng2_tidx, 'nin1': ng3_tidx, 'out': vm_tidx[-1]})

        xor_master = self.new_template(XORCore, params=xor_params)
        mux_master = self.new_template(Mux2to1Matched, params=mux_params)
        inv_master = self.new_template(InvCore, params=inv_params)
        nand_master = self.new_template(NAND2Core, params=nand_params)
        nor_master = self.new_template(NOR2Core, params=nor_params)
        selbuf_master = self.new_template(InvCore, params=selbuf_params)
        inv_nand_master = self.new_template(InvCore, params=inv_and_params)

        # Row 0 - Mux and first xor
        cur_col = 0
        selbuf = self.add_tile(selbuf_master, 1, cur_col)
        cur_col += selbuf_master.num_cols + min_sep
        mux0 = self.add_tile(mux_master, 1, cur_col)
        cur_col += mux_master.num_cols + min_sep
        mux_buf0 = self.add_tile(inv_master, 1, cur_col)
        cur_col += inv_master.num_cols + min_sep
        xor0 = self.add_tile(xor_master, 1, cur_col)
        cur_col += xor_master.num_cols + inv_master.num_cols + min_sep
        mux_buf1 = self.add_tile(inv_master, 1, cur_col, flip_lr=True)
        cur_col += mux_master.num_cols + min_sep
        mux1 = self.add_tile(mux_master, 1, cur_col, flip_lr=True)

        # Row 1 - Nand + Xor
        cur_col = 0
        nand0 = self.add_tile(nor_master if first_dec else nand_master, 0, cur_col)
        cur_col += nand_master.num_cols + min_sep
        nand_buf0 = self.add_tile(inv_nand_master, 0, cur_col)
        cur_col += inv_master.num_cols + min_sep
        xor1 = self.add_tile(xor_master, 0, cur_col)
        cur_col += xor_master.num_cols + min_sep
        d_buf0 = self.add_tile(inv_master, 0, cur_col)

        cur_col = self.num_cols - xor_master.num_cols
        xor2 = self.add_tile(xor_master, 0, cur_col)
        cur_col -= inv_master.num_cols + min_sep
        nand_buf1 = self.add_tile(inv_nand_master, 0, cur_col)
        cur_col -= nand_master.num_cols + min_sep
        nand1 = self.add_tile(nand_master, 0, cur_col)
        self.set_mos_size()

        # Connct sel/selb
        sel_in_vm_tidx = self.grid.coord_to_track(vm_layer, selbuf.get_pin('nin').lower, mode=RoundMode.NEAREST)
        sel_in_vm = self.connect_to_tracks(selbuf.get_pin('nin'), TrackID(vm_layer, sel_in_vm_tidx))
        mux_pinfo, yb, _ = self.get_tile_info(1)
        sel_xm_mid_coord = yb + mux_pinfo.height // 2
        _, sel_xm_tidxs = tr_manager.place_wires(xm_layer, ['sig'] * 3, center_coord=sel_xm_mid_coord)
        self.connect_to_tracks([sel_in_vm, mux0.get_pin('sel'), mux1.get_pin('sel')],
                               TrackID(xm_layer, sel_xm_tidxs[0]))
        self.connect_to_tracks([selbuf.get_pin('out'), mux0.get_pin('selb'), mux1.get_pin('selb')],
                               TrackID(xm_layer, sel_xm_tidxs[1]))

        # Connect btw Mux to Buf
        self.connect_to_track_wires(mux0.get_pin('out'), mux_buf0.get_pin('nin'))
        self.connect_to_track_wires(mux1.get_pin('out'), mux_buf1.get_pin('nin'))
        self.connect_to_track_wires(nand0.get_pin('out'), nand_buf0.get_pin('nin'))
        self.connect_to_track_wires(nand1.get_pin('out'), nand_buf1.get_pin('nin'))

        # Connect Mux to Xor
        mux_buf0_in = mux_buf0.get_pin('in')
        mux_buf0_in_tid = self.grid.coord_to_track(self.conn_layer, mux_buf0_in.middle, mode=RoundMode.NEAREST)
        mux_buf_in_conn = self.connect_to_tracks(mux_buf0_in, TrackID(self.conn_layer, mux_buf0_in_tid))
        self.connect_to_track_wires(mux_buf_in_conn, xor0.get_pin('in1_l'))
        self.connect_to_track_wires(mux_buf0.get_pin('out'), xor0.get_pin('in1b_l'))

        mux_buf1_in = mux_buf1.get_pin('in')
        mux_buf1_in_tid = self.grid.coord_to_track(self.conn_layer, mux_buf1_in.middle, mode=RoundMode.NEAREST)
        mux_buf_in_conn = self.connect_to_tracks(mux_buf1_in, TrackID(self.conn_layer, mux_buf1_in_tid))
        self.connect_to_track_wires(mux_buf_in_conn, xor0.get_pin('in<0>'))
        self.connect_to_track_wires(mux_buf1.get_pin('out'), xor0.get_pin('inb<0>'))

        # Connect Nand to Xor
        nand_buf0_in = nand_buf0.get_pin('in')
        nand_buf0_in_tid = self.grid.coord_to_track(self.conn_layer, nand_buf0_in.middle, mode=RoundMode.NEAREST)
        nand_buf_in_conn = self.connect_to_tracks(nand_buf0_in, TrackID(self.conn_layer, nand_buf0_in_tid))
        self.connect_to_track_wires(nand_buf_in_conn, xor1.get_pin('inb<0>'))
        self.connect_to_track_wires(nand_buf0.get_pin('out'), xor1.get_pin('in<0>'))

        nand_buf1_in = nand_buf1.get_pin('in')
        nand_buf1_in_tid = self.grid.coord_to_track(self.conn_layer, nand_buf1_in.middle, mode=RoundMode.NEAREST)
        nand_buf_in_conn = self.connect_to_tracks(nand_buf1_in, TrackID(self.conn_layer, nand_buf1_in_tid))
        self.connect_to_track_wires(nand_buf_in_conn, xor2.get_pin('inb<0>'))
        self.connect_to_track_wires(nand_buf1.get_pin('out'), xor2.get_pin('in<0>'))

        # Connect xor0 to xor1
        self.connect_to_track_wires(xor0.get_pin('out'), d_buf0.get_pin('nin'))
        self.connect_to_track_wires(xor0.get_pin('out'), xor1.get_pin('in1_r'))
        self.connect_to_track_wires(d_buf0.get_pin('out'), xor1.get_pin('in1b_r'))

        self.connect_to_track_wires(mux_buf1.get_pin('out'), xor2.get_pin('in1b_l'))
        vm_tidx_l = self.arr_info.col_to_track(vm_layer, self.num_cols, mode=RoundMode.LESS_EQ)
        mux1_vm = self.connect_to_tracks([xor2.get_pin('in1_r'), mux_buf1.get_pin('nin')], TrackID(vm_layer, vm_tidx_l))

        # Connect Nand input
        self.connect_to_track_wires(nand_buf0.get_pin('out'), nand1.get_pin('nin<0>'))
        self.connect_to_track_wires(xor0.get_pin('out'), nand1.get_pin('nin<1>'))
        # prev_in0 = self.connect_to_tracks(nand0.get_pin('nin<0>'), TrackID(vm_layer, vm_tidx[0]))
        # prev_in1 = self.connect_to_tracks(nand0.get_pin('nin<1>'), TrackID(vm_layer, vm_tidx[1]))

        # Export vm_in
        in0_vm_tidx = self.grid.coord_to_track(vm_layer, mux0.get_pin('in<0>').upper, mode=RoundMode.NEAREST)
        in1_vm_tidx = self.grid.coord_to_track(vm_layer, mux0.get_pin('in<1>').lower, mode=RoundMode.NEAREST)
        in2_vm_tidx = self.grid.coord_to_track(vm_layer, mux1.get_pin('in<0>').lower, mode=RoundMode.NEAREST)
        in3_vm_tidx = self.grid.coord_to_track(vm_layer, mux1.get_pin('in<1>').upper, mode=RoundMode.NEAREST)

        in0_vm = self.connect_to_tracks(mux0.get_pin('in<0>'), TrackID(vm_layer, in0_vm_tidx))
        in1_vm = self.connect_to_tracks(mux0.get_pin('in<1>'), TrackID(vm_layer, in1_vm_tidx))
        in2_vm = self.connect_to_tracks(mux1.get_pin('in<0>'), TrackID(vm_layer, in2_vm_tidx))
        in3_vm = self.connect_to_tracks(mux1.get_pin('in<1>'), TrackID(vm_layer, in3_vm_tidx))
        for idx, p in enumerate([in0_vm, in1_vm, in2_vm, in3_vm]):
            # p_xm = self.connect_to_tracks(p, TrackID(xm_layer, sel_xm_tidxs[-1], tr_w_xm),
            #                               min_len_mode=MinLenMode.MIDDLE)
            # p_ym_tid = self.grid.coord_to_track(ym_layer, p_xm.middle, mode=RoundMode.NEAREST)
            # p_ym = self.connect_to_tracks(p_xm, TrackID(ym_layer, p_ym_tid, tr_w_ym))
            self.add_pin(f'in<{idx}>', p)

        # self.set_mos_size(tot_seg)

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [xor0, xor1, xor2, nand0, nand1, nand_buf0, nand_buf1, mux0, mux_buf0, mux1, mux_buf1]
        for inst in inst_list:
            vss_list.append(inst.get_pin('VSS'))
            vdd_list.append(inst.get_pin('VDD'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)

        self.add_pin('sel', sel_in_vm)
        self.add_pin('sel_hm', selbuf.get_pin('nin'), hide=True)
        self.add_pin('prev_in<0>', nand0.get_pin('nin<0>'))
        self.add_pin('prev_in<1>', nand0.get_pin('nin<1>'))
        self.add_pin('mux1', mux1_vm)
        self.add_pin('mid1', nand_buf1.get_pin('out'))
        self.reexport(xor1.get_port('out'), net_name='out<0>')
        self.reexport(xor2.get_port('out'), net_name='out<1>')
        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)

        self.sch_params = dict(
            mux_params=mux_master.sch_params,
            xor_params=xor_master.sch_params,
            nand_params0=nand_master.sch_params,
            nand_params1=nand_master.sch_params,
            selbuf_params=selbuf_master.sch_params,
            buf_params=inv_master.sch_params,
            first_dec=first_dec,
        )


class PhaseDecoder(RingOscUnit):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_phase_decoder')

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
            row_idx='Row index',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            nbits=1,
            row_idx=[]
        )

    def fill_tap(self, tile_idx, row_idx=None, intv_idx=None, ofst=0, port_mode=SubPortMode.EVEN,
                 connect_to_sup_ow=False) -> None:
        """
        This method fill empty region with sub contact
        """
        min_fill_ncols = self.tech_cls.min_sub_col + 2 * self.tech_cls.min_sep_col + ofst
        _, _, flip_tile = self.used_array.get_tile_info(tile_idx)
        intv_list = self.used_array.get_complement(tile_idx, 0, 0, self.num_cols)
        if intv_idx:
            intv_list = [intv_list[idx] for idx in intv_idx]

        row_idx = row_idx if row_idx else [-1, 0]

        def get_diff_port(pmode):
            return SubPortMode.EVEN if pmode == SubPortMode.ODD else SubPortMode.ODD

        for intv in intv_list:
            intv_pair = intv[0]
            nspace = intv_pair[1] - intv_pair[0]
            if intv_pair[1] == self.num_cols:
                min_fill_ncols -= self.tech_cls.min_sep_col
            if intv_pair[0] == self.num_cols:
                min_fill_ncols -= self.tech_cls.min_sep_col
            if nspace < min_fill_ncols:
                continue
            else:
                _tapl = intv_pair[0] + self.min_sep_col if intv_pair[0] else intv_pair[0]
                _tapr = intv_pair[1] if intv_pair[1] == self.num_cols else intv_pair[1] - self.min_sep_col
                _tapl += ofst
                _tapr -= ofst
                _port_mode = get_diff_port(port_mode) if (intv_pair[0] + self.min_sep_col) & 1 else port_mode
                if 0 in row_idx:
                    tap0 = self.add_substrate_contact(0, _tapl, seg=_tapr - _tapl, tile_idx=tile_idx,
                                                      port_mode=_port_mode)
                    if connect_to_sup_ow:
                        connect_hm_sup_ow(self, tap0, tile_idx, 0, self.tr_manager)
                    else:
                        tid0 = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=tile_idx)
                        self.connect_to_tracks(tap0, tid0)
                # intv_list = self.used_array.get_complement(tile_idx, 1, 0, self.num_cols)
                if -1 in row_idx:
                    tap1 = self.add_substrate_contact(-1, _tapl, seg=_tapr - _tapl, tile_idx=tile_idx,
                                                      port_mode=_port_mode)
                    if connect_to_sup_ow:
                        connect_hm_sup_ow(self, tap1, tile_idx, -1, self.tr_manager)
                    else:
                        tid1 = self.get_track_id(-1, MOSWireType.DS, 'sup', tile_idx=tile_idx)
                        self.connect_to_tracks(tap1, tid1)

    def draw_bit_col(self, col, row, nbit, mux_master: MOSBase, buf_master: MOSBase, bufout_master,
                     reverse=False) -> \
            Tuple[List[PyLayInstance], PyLayInstance, PyLayInstance]:
        cur_row = row
        nmux = 2 ** nbit - 1
        mux_list = []
        if reverse:
            for idx in range(nmux):
                mux_list.append(self.add_tile(mux_master, cur_row, col))
                cur_row += mux_master.num_tile_rows
            buf = self.add_tile(buf_master, cur_row, col)
            bufout = self.add_tile(bufout_master, cur_row, col + buf_master.num_cols)
        else:
            buf = self.add_tile(buf_master, cur_row, col)
            bufout = self.add_tile(bufout_master, cur_row, col + buf_master.num_cols)
            cur_row += buf_master.num_tile_rows
            for idx in range(nmux):
                mux_list.append(self.add_tile(mux_master, cur_row, col))
                cur_row += mux_master.num_tile_rows
        return mux_list, buf, bufout

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        nbits: int = self.params['nbits']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        row_idx: List = self.params['row_idx']
        if not row_idx:
            row_idx = list(range(2 ** (nbits - 1)))

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        seg_mux: Dict[str, int] = seg_dict['mux']
        seg_buf: list = seg_dict['buf']

        # compute track locations
        tr_manager = self.tr_manager

        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)

        mux_params = dict(pinfo=pinfo, seg_dict=seg_mux, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False)
        buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False, sig_locs={}, dual_output=True)
        bufout_params = dict(pinfo=pinfo, seg=seg_buf[0], w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             vertical_sup=False, sig_locs={'nin': pg1_tidx})

        mux_master: MOSBase = self.new_template(MuxCore, params=mux_params)
        buf_master: MOSBase = self.new_template(InvChainCore, params=buf_params)
        bufout_master: MOSBase = self.new_template(InvCore, params=bufout_params)

        # Place mux, number of rows = number of bits
        cur_loc = 0
        mux_list_first_col, buf_first, buf_out_first = self.draw_bit_col(cur_loc, 1, nbits - 1, mux_master, buf_master,
                                                                         bufout_master)
        mux_list_list, buf_list, bufout_list = [mux_list_first_col], [buf_first], [buf_out_first]

        ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * (2 ** (nbits - 1) + nbits - 2))
        ntr_sp = self.arr_info.get_column_span(vm_layer, ntr)

        cur_loc += max(mux_master.num_cols, buf_master.num_cols)
        _, tr_locs = tr_manager.place_wires(vm_layer, ['sig'] * (2 ** (nbits - 1) + nbits - 1), align_idx=0,
                                            align_track=self.arr_info.col_to_track(vm_layer, cur_loc))
        # tr_locs = tr_locs[::2]+tr_locs[1::2]
        tap_ncol = self.min_sub_col
        tap_sep_col = self.sub_sep_col
        mid_sep = max(tap_ncol + 2 * tap_sep_col, ntr_sp)
        mid_sep + mid_sep & 1

        cur_loc += mid_sep
        cur_loc += cur_loc & 1

        cur_row = 1
        for idx in reversed(range(nbits - 2)):
            _mux_list, _buf, _buf_out = self.draw_bit_col(cur_loc, cur_row, idx + 1, mux_master, buf_master,
                                                          bufout_master)
            cur_row += len(_mux_list) + 1
            mux_list_list.append(_mux_list)
            buf_list.append(_buf)
            bufout_list.append(_buf_out)
        bufout_list.append(self.add_tile(bufout_master, cur_row, cur_loc))

        self.set_mos_size(num_tiles=2 ** (nbits - 1) + 2)
        for idx in range(self.num_tile_rows):
            fill_tap(self, idx)

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        first_stage_out_map = dict()
        for idx in range(1, 2 ** (nbits - 1)):
            _ro_idx = row_idx[idx] - 1
            _in0 = mux_list_first_col[idx - 1].get_pin('in<0>')
            _in1 = mux_list_first_col[idx - 1].get_pin('in<1>')
            _out_xm_tid = self.grid.coord_to_track(xm_layer, mux_list_first_col[idx - 1].get_pin('out').middle,
                                                   mode=RoundMode.NEAREST)
            _out_xm = self.connect_to_tracks(mux_list_first_col[idx - 1].get_pin('out'), TrackID(xm_layer, _out_xm_tid,
                                                                                                 tr_w_sig_xm))
            first_stage_out_map[_ro_idx] = _out_xm
            self.add_pin(f'in<{_ro_idx + 1}>', _in0)
            self.add_pin(f'in<{_ro_idx + 1 + len(row_idx)}>', _in1)

        first_stage_out = []
        for idx in range(2 ** (nbits - 1) - 1):
            first_stage_out.append(first_stage_out_map[idx])
        out_list_list = [first_stage_out]
        tr_idx = 0
        for mux_list in mux_list_list[1:]:
            _out_list = []
            _nmux = len(mux_list)
            for idx, mux in enumerate(mux_list):
                _out_xm_tid = self.grid.coord_to_track(xm_layer, mux.get_pin('out').middle, mode=RoundMode.NEAREST)
                _out_xm_tid = tr_manager.get_next_track(xm_layer, _out_xm_tid, 'sig', 'sig')
                _out_xm_tid = tr_manager.get_next_track(xm_layer, _out_xm_tid, 'sig', 'sig')
                _out_xm = self.connect_to_tracks(mux.get_pin('out'), TrackID(xm_layer, _out_xm_tid, tr_w_sig_xm))
                _out_list.append(_out_xm)
                _out_vm_mid = self.connect_to_tracks(mux.get_pin('in<0>'),
                                                     TrackID(vm_layer, tr_locs[tr_idx], tr_w_sig_vm))
                self.connect_to_track_wires(out_list_list[-1][idx], _out_vm_mid)
                tr_idx += 1
                _out_vm_mid = self.connect_to_tracks(mux.get_pin('in<1>'),
                                                     TrackID(vm_layer, tr_locs[tr_idx], tr_w_sig_vm))
                self.connect_to_track_wires(out_list_list[-1][idx - _nmux], _out_vm_mid)
                tr_idx += 1
            out_list_list.append(_out_list)

        for idx, mux_list in enumerate(mux_list_list):
            if idx < len(mux_list_list) - 1:
                _buf = buf_list[idx + 1]
            else:
                _buf = bufout_list[-1]
            _mux_mid_idx = len(mux_list) // 2
            _out_vm_mid = self.connect_to_tracks(out_list_list[idx][_mux_mid_idx],
                                                 TrackID(vm_layer, tr_locs[tr_idx], tr_w_sig_vm))
            self.connect_to_track_wires(_buf.get_pin('nin'), _out_vm_mid)
            tr_idx += 1

        # Connect select signals
        bit_xm_list = []
        for idx, (mux_list, buf, bufout) in enumerate(zip(mux_list_list, buf_list, bufout_list)):
            sel_vm = self.connect_wires([mux.get_pin('sel') for mux in mux_list])
            selb_vm = self.connect_wires([mux.get_pin('selb') for mux in mux_list])
            sel_xm_tidx = self.grid.coord_to_track(xm_layer, sel_vm[0].lower, mode=RoundMode.NEAREST)
            selb_xm_tidx = tr_manager.get_next_track(xm_layer, sel_xm_tidx, 'sig', 'sig')
            sel_xm, selb_xm = self.connect_differential_tracks(sel_vm, selb_vm, xm_layer,
                                                               sel_xm_tidx, selb_xm_tidx, width=tr_w_sig_xm)
            sel_buf_xm_tidx = self.grid.coord_to_track(xm_layer, buf.get_pin('outb').lower, mode=RoundMode.NEAREST)
            selb_buf_xm_tidx = tr_manager.get_next_track(xm_layer, sel_buf_xm_tidx, 'sig', 'sig')
            sel_buf_xm, selb_buf_xm = \
                self.connect_differential_tracks(buf.get_pin('outb'), buf.get_pin('out'), xm_layer,
                                                 sel_buf_xm_tidx, selb_buf_xm_tidx, width=tr_w_sig_xm)
            max_vm_tdix = max(sel_vm[0].track_id.base_index, selb_vm[0].track_id.base_index)
            min_vm_tdix = min(sel_vm[0].track_id.base_index, selb_vm[0].track_id.base_index)
            tr_sp_sig_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))
            available_vm_locs = self.get_available_tracks(vm_layer, min_vm_tdix, max_vm_tdix,
                                                          upper=mux_list[-1].bound_box.yh, lower=buf.bound_box.yl,
                                                          width=tr_w_sig_vm, sep=tr_sp_sig_vm, sep_margin=tr_sp_sig_vm)

            if len(available_vm_locs) < 2:
                raise ValueError("Don't have enough space for sel signals in phase decoder")
            self.connect_differential_tracks([sel_xm, sel_buf_xm], [selb_xm, selb_buf_xm], vm_layer,
                                             available_vm_locs[len(available_vm_locs) // 2],
                                             available_vm_locs[len(available_vm_locs) // 2 + 1], width=tr_w_sig_vm)

            # Connect to output buffer
            self.connect_to_track_wires(buf.get_pin('out') if idx else buf.get_pin('outb'),
                                        bufout.get_pin('nin'))
            out_buf_xm_tidx = tr_manager.get_next_track(xm_layer, sel_buf_xm_tidx, 'sig', 'sig', up=2)
            if not idx:
                out_buf_xm_tidx = tr_manager.get_next_track(xm_layer, out_buf_xm_tidx, 'sig', 'sig', up=2)
            out_buf_xm = self.connect_to_tracks(bufout.get_pin('out'), TrackID(xm_layer, out_buf_xm_tidx, tr_w_sig_xm),
                                                min_len_mode=MinLenMode.UPPER)
            bit_xm_list.append(out_buf_xm)

        lsb_out_xm_tidx = self.grid.coord_to_track(xm_layer, bufout_list[-1].get_pin('out').middle, RoundMode.NEAREST)
        lsb_out_xm = self.connect_to_tracks(bufout_list[-1].get_pin('out'), TrackID(xm_layer,
                                                                                    lsb_out_xm_tidx, tr_w_sig_xm))
        bit_xm_list.append(lsb_out_xm)
        bit_out_list = self.extend_wires(bit_xm_list, upper=self.bound_box.xh)
        for idx, out in enumerate(bit_out_list[::-1]):
            self.add_pin(f'bit<{idx}>', out)

        self.add_pin(f'in<{2 ** (nbits - 1)}>', buf_list[0].get_pin('nin'))

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [mux for mux_list in mux_list_list for mux in mux_list]
        inst_list.extend(buf_list)
        for inst in inst_list:
            vss_list.append(inst.get_pin('VSS'))
            vdd_list.append(inst.get_pin('VDD'))
        vdd_hm = self.connect_wires(vdd_list)
        vss_hm = self.connect_wires(vss_list)
        self.extend_wires(vdd_hm, upper=self.bound_box.xh, lower=self.bound_box.xl)
        self.extend_wires(vss_hm, upper=self.bound_box.xh, lower=self.bound_box.xl)

        vdd_xm_list, vss_xm_list = [], []
        for idx in range(self.num_tile_rows):
            _b, _t = export_xm_sup(self, idx, export_bot=not bool(idx), export_top=True)
            if not idx:
                vss_xm_list.append(_b)
            if idx & 1:
                vss_xm_list.append(_t)
            else:
                vdd_xm_list.append(_t)

        vdd_xm_list = self.extend_wires(vdd_xm_list, upper=self.bound_box.xh, lower=self.bound_box.xl)
        vss_xm_list = self.extend_wires(vss_xm_list, upper=self.bound_box.xh, lower=self.bound_box.xl)

        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        ym_tid_l = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.GREATER_EQ)
        ym_tid_r = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.LESS_EQ)
        num_ym_sup = tr_manager.get_num_wires_between(ym_layer, 'dum', ym_tid_l, 'dum', ym_tid_r, 'sup')
        _, ym_sup_tidxs = tr_manager.place_wires(ym_layer, ['dum'] + ['sup'] * num_ym_sup + ['dum'],
                                                 center_coord=self.bound_box.w // 2)
        ym_sup_tidxs = ym_sup_tidxs[1:-1]

        vdd_ym = [self.connect_to_tracks(vdd_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[::2]]
        vss_ym = [self.connect_to_tracks(vss_xm_list, TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[1::2]]
        vdd_ym = self.extend_wires(vdd_ym, upper=vss_ym[0].upper, lower=vss_ym[0].lower)

        tile_height = self.get_tile_info(0)[0].height
        num_tiles = self.num_tile_rows
        vss_coord_list = [tile_height * idx for idx in range(0, num_tiles, 2)]
        vdd_coord_list = [tile_height + tile_height * idx for idx in range(0, num_tiles, 2)]

        vss_xm1_list, vdd_xm1_list = [], []
        for vss in vss_coord_list:
            xm1_tidx = self.grid.coord_to_track(xm1_layer, vss, mode=RoundMode.NEAREST)
            vss_xm1 = self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx, tr_w_sup_xm1, grid=self.grid),
                                             track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            vss_xm1_list.append(vss_xm1)

        for vdd in vdd_coord_list:
            xm1_tidx = self.grid.coord_to_track(xm1_layer, vdd, mode=RoundMode.NEAREST)
            vdd_xm1 = self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx, tr_w_sup_xm1, grid=self.grid),
                                             track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            vdd_xm1_list.append(vdd_xm1)
        #
        self.add_pin('VDD_xm', vdd_xm_list, label='VDD', show=self.show_pins, connect=True)
        self.add_pin('VSS_xm', vss_xm_list, label='VSS', show=self.show_pins, connect=True)
        self.add_pin('VDD_xm1', vdd_xm1_list, label='VDD', show=self.show_pins, connect=True)
        self.add_pin('VSS_xm1', vss_xm1_list, label='VSS', show=self.show_pins, connect=True)

        self.sch_params = dict(
            mux_params=mux_master.sch_params,
            buf_params=buf_master.sch_params,
            inv_params=bufout_master.sch_params,
            nbits=nbits,
        )


class CnterDecoder(PhaseDecoder):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_decoder')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            dec_unit_params='Decoder unit parameters.',
            seg_inv='Inverter seg',
            seg_nand='Nand seg',
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
            nbits=8,
            seg_inv=2,
            seg_nand=2,
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
        if nbits & 1:
            raise ValueError('Number of bits  for cnter decoder must be even')

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_inv: int = self.params['seg_inv']
        seg_nand: int = self.params['seg_nand']
        min_sep = self.min_sep_col

        # compute track locations
        tr_manager = self.tr_manager

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)

        _, locs = tr_manager.place_wires(vm_layer, ['sig'] * 3)

        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          vertical_sup=False, sig_locs={'nin': pg1_tidx, 'out': locs[-2]})
        nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=False, sig_locs={'nin0': ng0_tidx, 'nin1': ng1_tidx, 'out': locs[-1]},
                           dual_output=True)
        dec_unit_params = self.params['dec_unit_params'].copy(append=dict(pinfo=pinfo))
        first_unit_params = self.params['dec_unit_params'].copy(append=dict(pinfo=pinfo, first_dec=True))

        inv_master: MOSBase = self.new_template(InvCore, params=inv_params)
        nand_master: MOSBase = self.new_template(NAND2Core, params=nand_params)
        unit_master: MOSBase = self.new_template(CntDecUnit, params=dec_unit_params)
        first_unit_master: MOSBase = self.new_template(CntDecUnit, params=first_unit_params)

        # Place mux, number of rows = number of bits
        cur_loc = 0
        and0_inbuf = self.add_tile(inv_master, 0, cur_loc)
        and1_inbuf = self.add_tile(inv_master, 1, cur_loc)
        cur_loc += inv_master.num_cols + min_sep
        nand0 = self.add_tile(nand_master, 0, cur_loc)
        nand1 = self.add_tile(nand_master, 1, cur_loc)
        cur_loc += nand_master.num_cols + min_sep
        and0_outbuf = self.add_tile(inv_master, 0, cur_loc)
        and1_outbuf = self.add_tile(inv_master, 1, cur_loc)
        cur_loc += inv_master.num_cols

        in_vm_track_pair_list = []
        ndec = nbits // 2
        ndec_row = ndec // 2
        ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * 4)
        dec_sp = self.arr_info.get_column_span(vm_layer, ntr)
        dec_row0, dec_row1 = [], []
        cur_loc += dec_sp
        cur_loc += cur_loc & 1
        for idx in range(ndec_row):
            in_pair_coord = self.arr_info.col_to_coord(cur_loc - dec_sp // 2)
            _, in_tidx_pair = tr_manager.place_wires(vm_layer, ['sig'] * 2, center_coord=in_pair_coord)
            in_vm_track_pair_list.append(in_tidx_pair)
            dec_row0.append(self.add_tile(unit_master if idx else first_unit_master, 0, cur_loc))
            cur_loc += unit_master.num_cols + dec_sp
            cur_loc += cur_loc & 1

        cur_loc -= dec_sp
        cur_loc -= cur_loc & 1
        for idx in range(ndec_row):
            in_pair_coord = self.arr_info.col_to_coord(cur_loc + dec_sp // 2)
            _, in_tidx_pair = tr_manager.place_wires(vm_layer, ['sig'] * 2, center_coord=in_pair_coord)
            in_vm_track_pair_list.append(in_tidx_pair)
            dec_row1.append(self.add_tile(unit_master, 3, cur_loc, flip_lr=True))
            cur_loc -= (unit_master.num_cols + dec_sp)
            cur_loc -= cur_loc & 1

        self.set_mos_size()

        for inbuf, nand, outbuf in [(and0_inbuf, nand0, and0_outbuf), (and1_inbuf, nand1, and1_outbuf)]:
            self.connect_to_track_wires(inbuf.get_pin('out'), nand.get_pin('nin<0>'))
            self.connect_to_track_wires(outbuf.get_pin('nin'), nand.get_pin('out'))

        for idx in range(self.num_tile_rows):
            self.fill_tap(idx)

        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        in_hm_list = [and0_inbuf.get_pin('nin'), and1_inbuf.get_pin('nin'), nand1.get_pin('nin<1>')]
        pin_vm_tid = self.grid.coord_to_track(vm_layer, and0_inbuf.get_pin('nin').lower, mode=RoundMode.LESS)
        pin_vm = self.connect_to_tracks(and0_inbuf.get_pin('nin'), TrackID(vm_layer, pin_vm_tid, tr_w_sig_vm))
        in_vm_list = [pin_vm]
        in_mid_vm_tidx = self.arr_info.col_to_track(vm_layer, inv_master.num_cols + min_sep)
        in_vm_list.append(self.connect_to_tracks([and1_inbuf.get_pin('nin'), nand0.get_pin('nin<1>')],
                                                 TrackID(vm_layer, in_mid_vm_tidx, tr_w_sig_vm),
                                                 min_len_mode=MinLenMode.MIDDLE))
        pin_vm_tid = self.grid.coord_to_track(vm_layer, nand1.get_pin('nin<1>').lower, mode=RoundMode.LESS)
        pin_vm = self.connect_to_tracks(nand1.get_pin('nin<1>'), TrackID(vm_layer, pin_vm_tid, tr_w_sig_vm))
        in_vm_list.append(pin_vm)

        # Collect inout
        in_pair_list, out_pair_list = [], []
        sel_list = []
        for inst in dec_row0 + dec_row1:
            in_pair_list.append((inst.get_pin('prev_in<0>'), inst.get_pin('prev_in<1>')))
            sel_list.append((inst.get_pin('sel_hm'), inst.get_pin('sel')))

        out_pair_list.append((and0_outbuf.get_pin('out'), and1_outbuf.get_pin('out')))
        for inst in dec_row0 + dec_row1:
            out_pair_list.append((inst.get_pin('mid1'), inst.get_pin('mux1')))
        out_pair_list = out_pair_list[:-1]

        # Connect inout
        for idx in range(ndec):
            out0, out1 = out_pair_list[idx]
            in0, in1 = in_pair_list[idx]
            tidx0, tidx1 = in_vm_track_pair_list[idx]
            out0_xm_tidx = self.grid.coord_to_track(xm_layer, out0.middle, mode=RoundMode.NEAREST)
            if idx:
                out1_xm_tidx = tr_manager.get_next_track(xm_layer, out0_xm_tidx, 'sig', 'sig')
            else:
                out1_xm_tidx = self.grid.coord_to_track(xm_layer, out1.middle, mode=RoundMode.NEAREST)
            out0_xm = self.connect_to_tracks(out0, TrackID(xm_layer, out0_xm_tidx, tr_w_sig_xm))
            out1_xm = self.connect_to_tracks(out1, TrackID(xm_layer, out1_xm_tidx, tr_w_sig_xm))
            out0_vm = self.connect_to_tracks(out0_xm, TrackID(vm_layer, tidx0, tr_w_sig_vm),
                                             min_len_mode=MinLenMode.MIDDLE)
            out1_vm = self.connect_to_tracks(out1_xm, TrackID(vm_layer, tidx1, tr_w_sig_vm),
                                             min_len_mode=MinLenMode.MIDDLE)
            self.connect_to_track_wires(out0_vm, in0)
            self.connect_to_track_wires(out1_vm, in1)
            if idx:
                self.connect_to_track_wires(out1_vm, sel_list[idx][0])

        vdd_xm_list, vss_xm_list = [], []
        for idx in range(self.num_tile_rows):
            _b, _ = export_xm_sup(self, idx, export_bot=True)
            if idx & 1:
                vdd_xm_list.append(_b)
            else:
                vss_xm_list.append(_b)

        if self.num_tile_rows & 1:
            vdd_xm_list.append(export_xm_sup(self, self.num_tile_rows - 1, export_top=True)[0])
        else:
            vss_xm_list.append(export_xm_sup(self, self.num_tile_rows - 1, export_top=True)[1])
        # Connect select sigansl
        self.connect_to_track_wires(in_hm_list[-1], sel_list[0][1])

        xm_tidx_bot = self.grid.coord_to_track(xm_layer, self.get_tile_info(1)[1], mode=RoundMode.NEAREST)
        xm_tidx_mid = self.grid.coord_to_track(xm_layer, self.get_tile_info(2)[1], mode=RoundMode.NEAREST)
        xm_tidx_top = self.grid.coord_to_track(xm_layer, self.get_tile_info(3)[1], mode=RoundMode.NEAREST)
        xm_sep = tr_manager.get_sep(xm_layer, ('sig', 'sig'))
        xm_avail_locs_bot = self.get_available_tracks(xm_layer, xm_tidx_bot, xm_tidx_mid, self.bound_box.xl,
                                                      self.bound_box.xh, width=tr_w_sig_xm, sep=xm_sep,
                                                      sep_margin=xm_sep)
        xm_avail_locs_top = self.get_available_tracks(xm_layer, xm_tidx_mid, xm_tidx_top, self.bound_box.xl,
                                                      self.bound_box.xh, width=tr_w_sig_xm, sep=xm_sep,
                                                      sep_margin=xm_sep)

        for idx, pin in enumerate(in_vm_list):
            pin_xm_tid = self.grid.coord_to_track(xm_layer, pin.middle, mode=RoundMode.NEAREST)
            pin_xm = self.connect_to_tracks(pin, TrackID(xm_layer, pin_xm_tid, tr_w_sig_xm),
                                            min_len_mode=MinLenMode.MIDDLE)
            # # pin_ym_tid = self.grid.coord_to_track(ym_layer, pin_xm.middle, mode=RoundMode.NEAREST)
            # pin_ym = self.connect_to_tracks(pin_xm, TrackID(ym_layer, pin_ym_tid, tr_w_sig_xm),
            #                                 min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
            self.add_pin(f'in<{idx}>', pin_xm)

        xm_b_idx, xm_t_idx = 0, 0
        for idx, inst in enumerate(dec_row0 + dec_row1):
            # self.reexport(inst.get_port('out<0>'), net_name=f'out<{2 * idx}>')
            # self.reexport(inst.get_port('out<1>'), net_name=f'out<{2 * idx + 1}>')
            for jdx in range(4):
                tid = xm_avail_locs_bot[xm_b_idx] if idx < ndec_row else xm_avail_locs_top[xm_t_idx]
                if idx < ndec_row:
                    xm_b_idx += 1
                else:
                    xm_t_idx += 1
                pin = self.connect_to_tracks(inst.get_pin(f'in<{jdx}>'), TrackID(xm_layer, tid, tr_w_sig_xm),
                                             min_len_mode=MinLenMode.MIDDLE)
                self.add_pin(f'in<{4 * idx + jdx + 3}>', pin)
            # self.reexport(inst.get_port('in<0>'), net_name=f'in<{4 * idx + 3}>')
            # self.reexport(inst.get_port('in<1>'), net_name=f'in<{4 * idx + 4}>')
            # self.reexport(inst.get_port('in<2>'), net_name=f'in<{4 * idx + 5}>')
            # self.reexport(inst.get_port('in<3>'), net_name=f'in<{4 * idx + 6}>')

        # Connect output horizontally
        tile0_yb = self.get_tile_info(0)[1]
        tile1_yb = self.get_tile_info(1)[1]
        tile3_yb = self.get_tile_info(3)[1]
        tile_top = self.get_tile_info(1)[0].height + tile3_yb
        xm_l_coord = self.bound_box.xl
        xm_h_coord = min([x.bound_box.xh for x in dec_row0])
        # row 0 avail tracks
        tr_w_sig_sp = tr_manager.get_sep(xm_layer, ('sig', 'sig'))
        xm_locs_list0 = \
            self.get_available_tracks(xm_layer, self.grid.coord_to_track(xm_layer, tile0_yb, RoundMode.NEAREST),
                                      self.grid.coord_to_track(xm_layer, tile1_yb, RoundMode.NEAREST),
                                      xm_l_coord, xm_h_coord, width=tr_w_sig_xm, sep=tr_w_sig_sp)

        for idx, inst in enumerate(dec_row0):
            out_odd = self.connect_to_tracks(inst.get_pin('out<1>'),
                                             TrackID(xm_layer, xm_locs_list0[2 * idx + 1], tr_w_sig_xm),
                                             track_upper=self.bound_box.xh)
            out_even = self.connect_to_tracks(inst.get_pin('out<0>'),
                                              TrackID(xm_layer, xm_locs_list0[2 * idx], tr_w_sig_xm),
                                              track_upper=self.bound_box.xh)
            self.add_pin(f'out<{2 * idx}>', out_even, mode=PinMode.UPPER)
            self.add_pin(f'out<{2 * idx + 1}>', out_odd, mode=PinMode.UPPER)

        num_dec_row0 = len(dec_row0)
        xm_locs_list1 = \
            self.get_available_tracks(xm_layer, self.grid.coord_to_track(xm_layer, tile3_yb, RoundMode.NEAREST),
                                      self.grid.coord_to_track(xm_layer, tile_top, RoundMode.NEAREST),
                                      xm_l_coord, xm_h_coord, width=tr_w_sig_xm, sep=tr_w_sig_sp)

        for idx, inst in enumerate(dec_row1):
            out_odd = self.connect_to_tracks(inst.get_pin('out<1>'),
                                             TrackID(xm_layer, xm_locs_list1[2 * idx + 1], tr_w_sig_xm),
                                             track_upper=self.bound_box.xh)
            out_even = self.connect_to_tracks(inst.get_pin('out<0>'),
                                              TrackID(xm_layer, xm_locs_list1[2 * idx], tr_w_sig_xm),
                                              track_upper=self.bound_box.xh)
            self.add_pin(f'out<{2 * idx + 2 * num_dec_row0}>', out_even, mode=PinMode.UPPER)
            self.add_pin(f'out<{2 * idx + 1 + 2 * num_dec_row0}>', out_odd, mode=PinMode.UPPER)

        vss_list, vdd_list = [], []
        inst_list = dec_row1 + dec_row0 + [and0_inbuf, and1_inbuf, nand0, nand1, and0_outbuf, and1_outbuf]
        for inst in inst_list:
            vss_list.append(inst.get_pin('VSS'))
            vdd_list.append(inst.get_pin('VDD'))
        vdd_hm = self.connect_wires(vdd_list)
        vss_hm = self.connect_wires(vss_list)
        self.extend_wires(vdd_hm, upper=self.bound_box.xh, lower=self.bound_box.xl)
        self.extend_wires(vss_hm, upper=self.bound_box.xh, lower=self.bound_box.xl)

        self.add_pin('VDD', vdd_xm_list, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_xm_list, show=self.show_pins, connect=True)

        self.sch_params = dict(
            unit_params_list=[first_unit_master.sch_params] + [unit_master.sch_params for _ in range(ndec - 1)],
            inv_params=inv_master.sch_params,
            nand_params=nand_master.sch_params,
        )


class CnterBufInv(PhaseDecoder):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_ff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            buf_params='Parameters for buf',
            invbuf_params='Parameters for invbuf',
            pinfo='Pinfo for unit row strongArm flop',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            top_sup_layer='Top supply layer',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(top_sup_layer=0, pinfo=None)

    def draw_layout(self) -> None:
        buf_params: Param = self.params['buf_params']
        invbuf_params: Param = self.params['invbuf_params']
        if isinstance(buf_params, str):
            buf_params = read_yaml(buf_params)
            buf_params: dict = buf_params['params']['params']
        self.draw_base(MOSBasePlaceInfo.make_place_info(self.grid, buf_params['pinfo']))
        # Make templates
        ng_tidx = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg_tidx = self.get_track_index(-1, MOSWireType.G, wire_name='sig', wire_idx=-2)
        invbuf_gen_params = dict(pinfo=self.draw_base_info, seg_list=invbuf_params['seg_list'],
                                 w_p=invbuf_params['w_p'], w_n=invbuf_params['w_n'], vertical_sup=True,
                                 dual_output=False, vertical_out=False, sig_locs={'nin1': pg_tidx, 'nin0': ng_tidx})
        invbuf_master = self.new_template(InvChainCore, params=invbuf_gen_params)
        buf_master = self.new_template(CnterBuffer, params=buf_params)
        buf_params['shift_out'] = True
        buf_master_shift = self.new_template(CnterBuffer, params=buf_params)

        # Setup track manager
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        min_sep = self.min_sep_col

        # -- input pesudo-differentail buffers
        buf_loc = self.min_sep_col
        buf_loc += buf_loc & 1
        bufn = self.add_tile(buf_master, 1, buf_loc)
        bufp = self.add_tile(buf_master_shift, 0, buf_loc)

        invbuf_loc = buf_loc + buf_master.num_cols + min_sep
        invbuf_loc += int(not invbuf_loc & 1)
        invbufn = self.add_tile(invbuf_master, 1, invbuf_loc)
        invbufp = self.add_tile(invbuf_master, 0, invbuf_loc)
        tap_ncol = self.min_sub_col
        tap_sep_col = self.sub_sep_col
        cur_loc = invbuf_loc + tap_sep_col + invbuf_master.num_cols
        sup_list = [self.add_substrate_contact(0, cur_loc, seg=tap_ncol),
                    self.add_substrate_contact(1, cur_loc, seg=tap_ncol),
                    self.add_substrate_contact(1, cur_loc, seg=tap_ncol, tile_idx=1),
                    self.add_substrate_contact(0, cur_loc, seg=tap_ncol, tile_idx=1)]

        self.set_mos_size()

        # connect buf to invbuf
        self.connect_to_track_wires(bufn.get_pin('outn'), invbufn.get_pin('nin'))
        self.connect_to_track_wires(bufp.get_pin('outn'), invbufp.get_pin('nin'))

        # connect crosscouple buf
        self.connect_differential_wires(bufn.get_pin('outn'), bufp.get_pin('outn'),
                                        bufp.get_pin('outp_hm'), bufn.get_pin('outp_hm'))

        # Connect to counter clk
        clkn_vm_tidx = self.grid.coord_to_track(vm_layer, invbufn.get_pin('nout').middle, RoundMode.NEAREST)
        clkn_vm_tidx = tr_manager.get_next_track(ym_layer, clkn_vm_tidx, 'sig', 'dum', up=1)
        clkp_vm_tidx = tr_manager.get_next_track(ym_layer, clkn_vm_tidx, 'sig', 'dum', up=-1)
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        clkn_vm, clkp_vm = self.connect_differential_tracks([invbufn.get_pin('nout'), invbufn.get_pin('pout')],
                                                            [invbufp.get_pin('nout'), invbufp.get_pin('pout')],
                                                            vm_layer, clkn_vm_tidx, clkp_vm_tidx,
                                                            width=tr_w_sig_vm, track_lower=self.bound_box.yl)

        # Get buffers supply
        vdd_buf_hm = [inst.get_pin('VDD') for inst in [bufn, bufp]]
        vss_buf_hm = [inst.get_pin('VSS') for inst in [bufn, bufp]]
        self.connect_to_track_wires(bufn.get_all_port_pins('VDD'), invbufn.get_all_port_pins('VDD'))
        self.connect_to_track_wires(bufp.get_all_port_pins('VDD'), invbufp.get_all_port_pins('VDD'))
        self.connect_to_track_wires(bufn.get_all_port_pins('VSS'), invbufn.get_all_port_pins('VSS'))
        self.connect_to_track_wires(bufp.get_all_port_pins('VSS'), invbufp.get_all_port_pins('VSS'))
        self.connect_to_track_wires(sup_list[0], bufp.get_pin('VSS'))
        self.connect_to_track_wires(sup_list[-1], bufn.get_pin('VSS'))
        self.connect_to_track_wires(sup_list[1:-1], bufn.get_pin('VDD'))
        vdd_buf_vm_l = self.grid.coord_to_track(vm_layer, bufn.bound_box.xl, RoundMode.LESS_EQ)
        vdd_buf_vm_r = self.grid.coord_to_track(vm_layer, invbufn.bound_box.xh, RoundMode.GREATER_EQ)
        vdd_buf_vm_l = tr_manager.get_next_track(vm_layer, vdd_buf_vm_l, 'sig', 'sup', up=False)
        vdd_buf_vm_r = tr_manager.get_next_track(vm_layer, vdd_buf_vm_r, 'sig', 'sup', up=True)
        tr_sp_sup_vm = tr_manager.get_sep(vm_layer, ('sup', 'sup'))
        vdd_buf_xm, vss_buf_xm = [], []
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')

        # buffer input to vm
        phi_out_cnter_ym_tidx = self.grid.coord_to_track(ym_layer, bufp.get_pin('in').middle, RoundMode.NEAREST)
        phi_out_cnter_ym_tidx = tr_manager.get_next_track(ym_layer, phi_out_cnter_ym_tidx, 'sig', 'dum', up=1)
        phi_outb_cnter_ym_tidx = tr_manager.get_next_track(ym_layer, phi_out_cnter_ym_tidx, 'sig', 'dum', up=-1)
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        phi_out_cnter_ym = self.connect_to_tracks(bufp.get_pin('in'),
                                                  TrackID(ym_layer, phi_out_cnter_ym_tidx, tr_w_sig_ym),
                                                  track_upper=self.bound_box.yh)
        phib_out_cnter_ym = self.connect_to_tracks(bufn.get_pin('in'),
                                                   TrackID(ym_layer, phi_outb_cnter_ym_tidx, tr_w_sig_ym),
                                                   track_upper=self.bound_box.yh)
        sup_buf_vm_locs = self.get_available_tracks(vm_layer, vdd_buf_vm_l, vdd_buf_vm_r, bufp.bound_box.yl,
                                                    bufn.bound_box.yh, tr_w_sup_vm, tr_sp_sup_vm)

        # Buffer supply layer
        for vdd in vdd_buf_hm:
            self.extend_wires(vdd, lower=self.bound_box.xl, upper=self.bound_box.xh)
            vdd_buf_vm = []
            for tidx in sup_buf_vm_locs:
                vdd_buf_vm.append(self.connect_to_tracks(vdd, TrackID(vm_layer, tidx, tr_w_sup_vm),
                                                         min_len_mode=MinLenMode.MIDDLE))
            vdd_xm_tid = self.grid.coord_to_track(xm_layer,
                                                  self.grid.track_to_coord(hm_layer, vdd.track_id.base_index),
                                                  mode=RoundMode.NEAREST)
            vdd_buf_xm.append(self.connect_to_tracks(vdd_buf_vm, TrackID(xm_layer, vdd_xm_tid, tr_w_sup_xm,
                                                                         grid=self.grid)))
        for vss in vss_buf_hm:
            self.extend_wires(vss, lower=self.bound_box.xl, upper=self.bound_box.xh)
            vss_buf_vm = []
            for tidx in sup_buf_vm_locs:
                vss_buf_vm.append(self.connect_to_tracks(vss, TrackID(vm_layer, tidx, tr_w_sup_vm),
                                                         min_len_mode=MinLenMode.MIDDLE))
            vss_xm_tid = self.grid.coord_to_track(xm_layer,
                                                  self.grid.track_to_coord(hm_layer, vss.track_id.base_index),
                                                  mode=RoundMode.NEAREST)
            vss_buf_xm.append(self.connect_to_tracks(vss_buf_vm, TrackID(xm_layer, vss_xm_tid, tr_w_sup_xm,
                                                                         grid=self.grid)))

        # ym supplies
        ym_tid_l = self.arr_info.col_to_track(ym_layer, 0, mode=RoundMode.GREATER_EQ)
        ym_tid_r = self.arr_info.col_to_track(ym_layer, self.num_cols, mode=RoundMode.LESS_EQ)
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        ym_sup_tidxs = self.get_available_tracks(ym_layer, ym_tid_l, ym_tid_r, self.bound_box.yl, self.bound_box.yh,
                                                 tr_manager.get_width(ym_layer, 'sup'),
                                                 tr_manager.get_sep(ym_layer, ('sup', 'sup')), True,
                                                 tr_manager.get_sep(ym_layer, ('sup', 'sup')))

        ym_sup_tidxs = ym_sup_tidxs[1:-1]

        vdd_ym = [self.connect_to_tracks(vdd_buf_xm, TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[::2]]
        vss_ym = [self.connect_to_tracks(vss_buf_xm, TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[1::2]]
        vdd_ym = self.extend_wires(vdd_ym, upper=vss_ym[0].upper, lower=vss_ym[0].lower)

        # xm1 supplies
        tile_height = self.get_tile_pinfo(0).height
        xm1_tidx = [self.grid.coord_to_track(xm1_layer, idx * tile_height, mode=RoundMode.NEAREST) for idx in range(3)]
        vss_xm1_list = [self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx[0], tr_w_sup_xm1, grid=self.grid),
                                               track_lower=self.bound_box.xl, track_upper=self.bound_box.xh),
                        self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx[-1], tr_w_sup_xm1, grid=self.grid),
                                               track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)]
        vdd_xm1_list = [self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx[1], tr_w_sup_xm1, grid=self.grid),
                                               track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)]

        # Add pins
        self.add_pin('clkp', clkn_vm, connect=True)
        self.add_pin('clkn', clkp_vm, connect=True)
        self.add_pin('midp', bufn.get_pin('outn'), connect=True)
        self.add_pin('midn', bufp.get_pin('outn'), connect=True)

        self.add_pin('phi', phi_out_cnter_ym)
        self.add_pin('phib', phib_out_cnter_ym)

        self.add_pin('VDD_ym', vdd_ym, label='VDD')
        self.add_pin('VSS_ym', vss_ym, label='VSS')

        self.add_pin('VDD_xm1', vdd_xm1_list, label='VDD')
        self.add_pin('VSS_xm1', vss_xm1_list, label='VSS')

        # Export supply to higher layers
        #
        # top_sup_layer = self.params['top_sup_layer']
        # vss_hor_coord = [self.grid.track_to_coord(xm1_layer, warr.track_id.base_index) for warr in vss_xm1_list]
        # vdd_hor_coord = [self.grid.track_to_coord(xm1_layer, warr.track_id.base_index) for warr in vdd_xm1_list]
        # vss_vert_bnd = [vss_xm1_list[0].lower, vss_xm1_list[0].upper]
        # vdd_vert_bnd = [vdd_xm1_list[0].lower, vdd_xm1_list[0].upper]
        #
        # last_vdd_list, last_vss_list = vdd_xm1_list, vss_xm1_list
        # for idx in range(xm1_layer + 1, top_sup_layer + 1):
        #     tr_w_sup = tr_manager.get_width(idx, 'sup')
        #     lay_dir = self.grid.get_direction(idx)
        #     tr_w_sep = tr_manager.get_sep(idx, ('sup', 'sup')) if tr_w_sup > 0 else 0
        #     sup_w = tr_manager.get_width(idx, 'sup')
        #     sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
        #     sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
        #     if lay_dir == Orient2D.x and lay_dir < 10:
        #         vss_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vss_hor_coord]
        #         vdd_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vdd_hor_coord]
        #         last_vss_list = [self.connect_to_tracks(last_vss_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
        #                          vss_tidx_list]
        #         last_vdd_list = [self.connect_to_tracks(last_vdd_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
        #                          vdd_tidx_list]
        #         last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        #         last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        #     elif lay_dir == Orient2D.x:
        #         last_vdd_list, last_vss_list = self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list],
        #                                                                 idx - 1, self.bound_box)
        #     else:
        #         vss_tid_l = self.grid.coord_to_track(idx, vss_vert_bnd[0] + sup_w, mode=RoundMode.NEAREST)
        #         vss_tid_r = self.grid.coord_to_track(idx, vss_vert_bnd[1] - sup_w, mode=RoundMode.NEAREST)
        #         vss_locs = self.get_tids_between(idx, vss_tid_l, vss_tid_r, width=tr_w_sup,
        #                                          sep=tr_w_sep, sep_margin=None, include_last=False)
        #         vdd_tid_l = self.grid.coord_to_track(idx, vdd_vert_bnd[0] + sup_w, mode=RoundMode.NEAREST)
        #         vdd_tid_r = self.grid.coord_to_track(idx, vdd_vert_bnd[1] - sup_w, mode=RoundMode.NEAREST)
        #         vdd_locs = self.get_tids_between(idx, vdd_tid_l, vdd_tid_r, width=tr_w_sup,
        #                                          sep=tr_w_sep, sep_margin=None, include_last=False)
        #         last_vdd_list = [self.connect_to_tracks(last_vdd_list, tid) for tid in vdd_locs]
        #         last_vss_list = [self.connect_to_tracks(last_vss_list, tid) for tid in vss_locs]
        #         last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
        #         last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
        #
        #     self.add_pin(f'VDD{idx}' if idx < top_sup_layer else 'VDD',
        #                  last_vdd_list, hide=idx < top_sup_layer, label='VDD')
        #     self.add_pin(f'VSS{idx}' if idx < top_sup_layer else 'VSS',
        #                  last_vss_list, hide=idx < top_sup_layer, label='VSS')

        self._sch_params = dict(
            buf_params_list=[buf_master.sch_params, invbuf_master.sch_params],
        )
