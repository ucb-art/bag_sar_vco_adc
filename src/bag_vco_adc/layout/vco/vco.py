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


from typing import Any, Dict, Mapping, Union, Tuple, Optional, Type, cast

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.io import read_yaml
from bag.layout.routing import TrackID
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from bag.util.importlib import import_class
from bag.util.math import HalfInt
from pybag.core import Transform, BBox
from pybag.enum import MinLenMode, Orientation, Orient2D, PinMode
from pybag.enum import RoundMode
from xbase.layout.enum import SubPortMode
from xbase.layout.mos.base import MOSBase
from xbase.layout.mos.placement.data import MOSBasePlaceInfo, TilePatternElement, TilePattern, MOSArrayPlaceInfo
from .vco_cnter_dec import PhaseDecoder, CnterDecoder, CnterAsync, CnterBufInv
from .vco_flops import SAFFCol, SAFFCore
from .vco_ring_osc import VCOCore
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID
from ..util.util import basefill_bbox
from ..util.wrapper import GenericWrapper, IntegrationWrapper


class ROSAFFDecoder(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_ro_ff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            num_stages='Number of stages',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            ring_params='Ring parameters',
            saff_params='strongArm flops parameter',
            decoder_params='Decoder parameters',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def draw_layout(self) -> None:
        # Load parameters
        ring_params: Param = self.params['ring_params']
        saff_params: Param = self.params['saff_params']
        decoder_params: Param = self.params['decoder_params']
        num_stages: int = self.params['num_stages']
        if isinstance(ring_params, str):
            ring_params = read_yaml(ring_params)
            ring_params = ring_params['params']
        if isinstance(saff_params, str):
            saff_params = read_yaml(saff_params)
            saff_params = saff_params['params']['params']

        # setup track manager
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      saff_params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        # ym1_layer = xm1_layer + 1

        # Make templates
        ring_gen_params = ring_params if isinstance(ring_params, dict) else \
            ring_params.copy(append=dict(num_stages=num_stages))
        vco_master: VCOCore = self.new_template(VCOCore, params=ring_gen_params)

        saff_gen_params = dict(
            export_private=False,
            cls_name=SAFFCol.get_qualified_name(),
            params=dict(num_stages=num_stages, topbot_dummy=True,
                        saff_params=saff_params, pinfo=saff_params['pinfo']))
        saff_master: MOSBase = self.new_template(GenericWrapper, params=saff_gen_params)

        master_row_chain_idx = vco_master.row_idx[:-1]
        dec_row_idx = list(range(num_stages))
        for idx, row in enumerate(master_row_chain_idx):
            dec_row_idx[row] = idx
        dec_gen_params = dict(
            export_private=False,
            cls_name=PhaseDecoder.get_qualified_name(),
            params=decoder_params.copy(append=dict(row_idx=dec_row_idx))
        )
        dec_master: MOSBase = self.new_template(GenericWrapper, params=dec_gen_params)

        # Floorplan
        top_layer = max(vco_master.top_layer, saff_master.top_layer, dec_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        vco = self.add_instance(vco_master, xform=Transform(0, 0))
        x_saff = -(-vco.bound_box.xh // w_blk) * w_blk
        saff = self.add_instance(saff_master, xform=Transform(x_saff, 0))
        x_dec = -(-saff.bound_box.xh // w_blk) * w_blk
        dec = self.add_instance(dec_master, xform=Transform(x_dec, 0))

        # Size the block
        top_sup_layer = ring_params['top_sup_layer']
        w_blk, h_blk = self.grid.get_block_size(top_sup_layer, half_blk_y=True, half_blk_x=True)
        h_tot = max(vco.bound_box.yh, dec.bound_box.yh, saff.bound_box.yh)
        w_tot = -(-dec.bound_box.xh // w_blk) * w_blk
        h_tot = -(-h_tot // h_blk) * h_blk
        self.set_size_from_bound_box(top_sup_layer, BBox(0, 0, w_tot, h_tot))

        # Connect decoder to flops
        dec_in_vm_tidx0 = self.grid.coord_to_track(vm_layer, dec.bound_box.xl, mode=RoundMode.NEAREST)
        dec_in_vm_tidx1 = tr_manager.get_next_track(vm_layer, dec_in_vm_tidx0, 'sig', 'sig', up=False)
        dec_vm_map = dict()
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        for idx in range(1, num_stages):
            _in0 = dec.get_pin(f'in<{idx}>')
            _in1 = dec.get_pin(f'in<{idx + num_stages}>')
            _in0_vm, _in1_vm = self.connect_differential_tracks(_in0, _in1, vm_layer, dec_in_vm_tidx0, dec_in_vm_tidx1,
                                                                width=tr_w_sig_vm)
            dec_vm_map[idx] = _in0_vm
            dec_vm_map[idx + num_stages] = _in1_vm

        mid_bit_vm = self.connect_to_tracks(dec.get_pin(f'in<{num_stages}>'),
                                            TrackID(vm_layer, dec_in_vm_tidx0, tr_w_sig_vm))
        self.connect_to_track_wires(mid_bit_vm, saff.get_pin('outp<0>'))
        nbits = (2 * num_stages - 1).bit_length()
        for idx in range(nbits):
            self.reexport(dec.get_port(f'bit<{idx}>'))

        # Connect ring to flops
        ring_out_name = 'phi_buf' if vco.has_port('phi_buf<0>') else 'phi'
        ro_row_idx = vco_master.row_idx
        for idx in range(num_stages):
            ro_oute = vco.get_pin(f'{ring_out_name}<{idx}>')
            ro_outo = vco.get_pin(f'{ring_out_name}<{idx + num_stages}>')
            ff_inn = saff.get_pin(f'inn<{ro_row_idx[idx]}>')
            ff_inp = saff.get_pin(f'inp<{ro_row_idx[idx]}>')
            self.connect_differential_wires(ro_oute, ro_outo, ff_inn, ff_inp)
            self.add_pin(f'ff_out<{idx}>', saff.get_pin(f'outn<{ro_row_idx[idx]}>'))
            self.add_pin(f'ff_out<{idx+num_stages}>', saff.get_pin(f'outp<{ro_row_idx[idx]}>'))

        for idx in range(1, num_stages):
            ff_outn = saff.get_pin(f'outn<{ro_row_idx[idx]}>')
            ff_outp = saff.get_pin(f'outp<{ro_row_idx[idx]}>')
            self.connect_differential_wires(ff_outn, ff_outp, dec_vm_map[idx], dec_vm_map[idx + num_stages])

        # Connect supplies
        vss_xm1_common = vco.get_all_port_pins('VSS_r') + saff.get_all_port_pins('VSS') + dec.get_all_port_pins(
            'VSS_xm1')
        vdd_xm1_common = vco.get_all_port_pins('VDD', layer=xm1_layer) + saff.get_all_port_pins(
            'VDD') + dec.get_all_port_pins('VDD_xm1')

        vdd_xm1 = self.connect_wires(vdd_xm1_common)
        vss_xm1 = self.connect_wires(vss_xm1_common)
        # vdd_xm_common = vco.get_all_port_pins('VDD_xm') + dec.get_all_port_pins('VDD_xm')
        # vss_xm_common = vco.get_all_port_pins('VSS_xm') + dec.get_all_port_pins('VSS_xm')
        # self.connect_wires(vdd_xm_common)
        # self.connect_wires(vss_xm_common)

        clk_xm1 = saff.get_all_port_pins('clk_xm1')
        ym1_layer = xm1_layer + 1
        tr_w_clk_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        clk_ym1_tidx = self.grid.coord_to_track(ym1_layer, clk_xm1[0].middle, RoundMode.NEAREST)
        clk_ym1 = self.connect_to_tracks(clk_xm1, TrackID(ym1_layer, clk_ym1_tidx, tr_w_clk_ym1))
        if saff.has_port('clkb'):
            clkb_xm1 = saff.get_all_port_pins('clkb_xm1')
            clkb_ym1_tidx = self.grid.coord_to_track(ym1_layer, clkb_xm1[0].middle, RoundMode.NEAREST)
            clkb_ym1 = self.connect_to_tracks(clkb_xm1, TrackID(ym1_layer, clkb_ym1_tidx, tr_w_clk_ym1))
            self.add_pin('clkb', clkb_ym1)

        last_vdd_list, last_vss_list = vdd_xm1, vss_xm1
        ff_dec_bbox = BBox(saff.bound_box.xl, 0, dec.bound_box.xh, self.bound_box.yh)
        for idx in range(xm1_layer + 1, top_sup_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                last_vdd_list, last_vss_list = \
                    self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list], idx - 1, ff_dec_bbox,
                                             side_sup=False, extend_lower_layer=False)
            else:
                last_vdd_list = self.connect_to_track_wires(last_vdd_list, vco.get_all_port_pins(f'VDD{idx}'))
                last_vss_list = self.connect_to_track_wires(last_vss_list, vco.get_all_port_pins(f'VSS{idx}'))
                last_vss_list = self.extend_wires(last_vss_list, upper=self.bound_box.xh)
                last_vdd_list = self.extend_wires(last_vdd_list, upper=self.bound_box.xh)

        if self.grid.get_direction(top_sup_layer) == Orient2D.y:
            self.reexport(vco.get_port(f'VDD{top_sup_layer}'), net_name='VDD')
            self.reexport(vco.get_port(f'VSS{top_sup_layer}'), net_name='VSS')
            self.add_pin('VDD', last_vdd_list)
            self.add_pin('VSS', last_vss_list)
        else:
            self.add_pin('VDD', last_vdd_list)
            self.add_pin('VSS', last_vss_list)

        # Export to buffer and decoder in cnter part
        _, phi_out_ym_tidx = tr_manager.place_wires(ym_layer, ['sig'] * 2, center_coord=saff.bound_box.xl)
        phi_out, phib_out = self.connect_matching_tracks([saff.get_pin(f'inp<{ro_row_idx[0]}>'),
                                                          saff.get_pin(f'inn<{ro_row_idx[0]}>')], ym_layer,
                                                         phi_out_ym_tidx, width=tr_manager.get_width(ym_layer, 'sig'))
        self.add_pin('phi_out', phi_out)
        self.add_pin('phib_out', phib_out)
        self.add_pin('phi_sampled', mid_bit_vm)
        self.add_pin('clk', clk_ym1)

        for pinname in vco.port_names_iter():
            if 'phi' in pinname:
                self.reexport(vco.get_port(pinname))

        # if vco.has_port('vctrl_p'):
        #     self.reexport(vco.get_port('vctrl_p'))
        #     self.reexport(vco.get_port('vtop'))
        # if vco.has_port('vctrl_n'):
        #     self.reexport(vco.get_port('vctrl_n'))
        #     self.reexport(vco.get_port('vbot'))

        self.reexport(vco.get_port('vbot'))
        self.reexport(vco.get_port('vctrl'))
        self.reexport(vco.get_port('tail'))

        self._sch_params = dict(
            vco_params=vco_master.sch_params,
            ff_params=saff_master.sch_params,
            dec_params=dec_master.sch_params,
            out_buf=vco.has_port('phi_buf<0>'),
        )


class CnterBufInvSAFF(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_buf_ff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            top_sup_layer='Top supply layer',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            buf_params='Ring parameters',
            saff_params='strongArm flops parameter',
        )

    def draw_layout(self) -> None:
        # Load parameters
        buf_params: Param = self.params['buf_params']
        saff_params: Param = self.params['saff_params']
        if isinstance(buf_params, str):
            buf_params = read_yaml(buf_params)
            buf_params = buf_params['params']
        if isinstance(saff_params, str):
            saff_params = read_yaml(saff_params)
            saff_params = saff_params['params']['params']

        # setup track manager
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      saff_params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        # ym1_layer = xm1_layer + 1

        # Make templates
        buf_master = self.new_template(GenericWrapper,
                                       params=dict(cls_name=CnterBufInv.get_qualified_name(),
                                                   export_private=False,
                                                   params=buf_params))
        saff_gen_params = dict(
            export_private=False,
            cls_name=SAFFCol.get_qualified_name(),
            params=dict(num_stages=2, topbot_dummy=False, saff_params=saff_params, pinfo=saff_params['pinfo']),
        )
        saff_master: MOSBase = self.new_template(GenericWrapper, params=saff_gen_params)

        # Floorplan
        top_layer = max(buf_master.top_layer, saff_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        buf = self.add_instance(buf_master, xform=Transform(0, 0))
        x_saff = -(-buf.bound_box.xh // w_blk) * w_blk
        saff = self.add_instance(saff_master, xform=Transform(x_saff, 0))
        # Size the block
        top_sup_layer = self.params['top_sup_layer']
        w_blk, h_blk = self.grid.get_block_size(top_sup_layer, half_blk_y=True, half_blk_x=True)
        h_tot = max(buf.bound_box.yh, saff.bound_box.yh)
        w_tot = -(-saff.bound_box.xh // w_blk) * w_blk
        h_tot = -(-h_tot // h_blk) * h_blk
        self.set_size_from_bound_box(top_sup_layer, BBox(0, 0, w_tot, h_tot))

        self.connect_differential_wires(saff.get_pin('inn<1>'), saff.get_pin('inp<1>'),
                                        buf.get_pin('midn'), buf.get_pin('midp'))
        clkn_xm, clkp_xm = self.connect_differential_wires(buf.get_pin('clkn'), buf.get_pin('clkp'),
                                                           saff.get_pin('inn<0>'), saff.get_pin('inp<0>'))
        clkn_ym_tidx = self.grid.coord_to_track(ym_layer, buf.bound_box.xh, RoundMode.NEAREST)
        clkp_ym_tidx = tr_manager.get_next_track(ym_layer, clkn_ym_tidx, 'sig', 'sig', up=1)

        clkn_ym, clkp_ym = self.connect_matching_tracks([clkn_xm, clkp_xm], ym_layer,
                                                        [clkn_ym_tidx, clkp_ym_tidx],
                                                        width=tr_manager.get_width(ym_layer, 'sig'))
        # self.connect_to_track_wires(saff.get_pin('inn<1>'), buf.get_pin('midn'))
        # self.connect_to_track_wires(saff.get_pin('inp<1>'), buf.get_pin('midp'))
        # self.connect_to_track_wires(saff.get_pin('inn<0>'), buf.get_pin('clkn'))
        # self.connect_to_track_wires(saff.get_pin('inp<0>'), buf.get_pin('clkp'))

        # Connect supplies
        vss_xm1_common = buf.get_all_port_pins('VSS_xm1') + saff.get_all_port_pins('VSS')
        vdd_xm1_common = buf.get_all_port_pins('VDD_xm1') + saff.get_all_port_pins('VDD')

        vdd_xm1 = self.connect_wires(vdd_xm1_common)
        vss_xm1 = self.connect_wires(vss_xm1_common)

        last_vdd_list, last_vss_list = vdd_xm1, vss_xm1
        sup_bbox = BBox(buf.bound_box.xl, 0, saff.bound_box.xh, self.bound_box.yh)
        for idx in range(xm1_layer + 1, top_sup_layer + 1):
            last_vdd_list, last_vss_list = \
                self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list], idx - 1, sup_bbox,
                                         side_sup=True, extend_lower_layer=False)

        self.add_pin('clk', saff.get_all_port_pins('clk') + saff.get_all_port_pins('clk_xm1'))
        if saff.has_port('clkb'):
            self.add_pin('clkb', saff.get_all_port_pins('clkb') + saff.get_all_port_pins('clkb_xm1'))
        # export pins
        # -- flip index of saff output
        self.add_pin('bufn_sampled<0>', saff.get_pin('outn<1>'))
        self.add_pin('bufn_sampled<1>', saff.get_pin('outn<0>'))
        self.add_pin('bufp_sampled<0>', saff.get_pin('outp<1>'))
        self.add_pin('bufp_sampled<1>', saff.get_pin('outp<0>'))
        self.add_pin('midp', buf.get_pin('midp'))
        self.add_pin('midn', buf.get_pin('midn'))
        self.add_pin('outp', clkp_ym, connect=True)
        self.add_pin('outn', clkn_ym, connect=True)
        self.add_pin('phi', buf.get_pin('phi'))
        self.add_pin('phib', buf.get_pin('phib'))
        self.add_pin('VDD', last_vdd_list)
        self.add_pin('VSS', last_vss_list)
        self._sch_params = dict(
            **buf_master.sch_params,
            ff_params=saff_master.sch_params,
        )


class CnterSAFFDecoder(PhaseDecoder):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_cnter_ff')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cnter_decoder_params='cnter decoder parameter',
            cnter_params='Ring parameters',
            saff_params='strongoArm flops parameter',
            pinfo='Pinfo for unit row strongArm flop',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            top_sup_layer='Top supply layer',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(top_sup_layer=0, pinfo=None)

    def draw_layout(self) -> None:
        # Load parameters
        cnter_params: Param = self.params['cnter_params']
        saff_params: Param = self.params['saff_params']
        cnter_decoder_params: Param = self.params['cnter_decoder_params']
        if isinstance(cnter_params, str):
            cnter_params = read_yaml(cnter_params)
            cnter_params = ImmutableSortedDict(cnter_params['params']['params'])
        if isinstance(saff_params, str):
            saff_params = read_yaml(saff_params)
            saff_params: dict = saff_params['params']['params']
        if self.params['pinfo']:
            pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        else:
            pinfo = MOSBasePlaceInfo.make_place_info(self.grid, cnter_params['pinfo'])

        # Make templates
        cnter_master: MOSBase = self.new_template(CnterAsync, params=cnter_params.copy(append=(dict(pinfo=pinfo))))
        cnter_decoder_master: MOSBase = self.new_template(CnterDecoder, params=cnter_decoder_params.copy())
        decoder_nrows = cnter_decoder_master.num_tile_rows
        cnter_nrows = cnter_master.num_tile_rows
        cnter_ncol = cnter_master.num_cols
        nrows = cnter_master.num_tile_rows

        # Setup tile info
        # Here it actually gets tr_manger from cnter_decoder
        tile_ele = []
        for idx in range(decoder_nrows):
            tile_ele.append(cnter_decoder_master.get_tile_subpattern(0, 1, flip=bool(idx & 1)))
        tile_ele.append(TilePatternElement(pinfo[1]['cnter_tile'], flip=bool(decoder_nrows & 1)))
        for idx in range(cnter_nrows + 3):
            tile_ele.append(TilePatternElement(pinfo[1]['cnter_tile'], flip=bool(idx & 1)))
        decoder_nrows += 1
        tile_ele = TilePatternElement(TilePattern(tile_ele))
        self.draw_base((tile_ele, pinfo[1]))

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

        tile_height = cnter_master.get_tile_pinfo(0).height
        _, xm_inout_tid = tr_manager.place_wires(xm_layer, ['sig'] * 4, center_coord=tile_height // 2)

        saff_sig_locs = {'inn': xm_inout_tid[0], 'inp': xm_inout_tid[-1]}
        saff_params['pinfo'] = pinfo
        saff_params['signal_locs'] = saff_sig_locs
        saff_params['export_sup_to_xm1'] = False

        saff_e_master: MOSBase = self.new_template(SAFFCore, params=saff_params)
        saff_o_master: MOSBase = self.new_template(SAFFCore, params=saff_params)

        # Add instances
        cur_loc = 0
        num_buf = 2  # clkn/clkp take 1 row separately
        # -- FFs at left side
        saff_l_list = []
        for idx in range(nrows):
            if idx & 1:
                saff_l_list.append(
                    self.add_tile(saff_e_master, idx + decoder_nrows, cur_loc + saff_e_master.num_cols, flip_lr=True))
            else:
                saff_l_list.append(
                    self.add_tile(saff_o_master, idx + decoder_nrows, cur_loc + saff_o_master.num_cols, flip_lr=True))
        cur_loc += saff_e_master.num_cols + min_sep

        # -- counter
        cnter = self.add_tile(cnter_master, decoder_nrows, cur_loc)

        # -- decoder
        dec_col = cnter_master.num_cols + saff_o_master.num_cols + cur_loc
        dec_col -= int(not dec_col & 1)
        decoder = self.add_tile(cnter_decoder_master, 0, max(dec_col, cnter_decoder_master.num_cols), flip_lr=True)
        for idx in range(decoder_nrows - 1):
            self.fill_tap(idx, intv_idx=[0], port_mode=SubPortMode.ODD)

        # -- FFs at right side
        cur_loc += cnter_ncol + min_sep
        saff_r_list = []
        for idx in range(nrows):
            if idx & 1:
                saff_r_list.append(self.add_tile(saff_o_master, idx + decoder_nrows, cur_loc, flip_lr=False))
            else:
                saff_r_list.append(self.add_tile(saff_e_master, idx + decoder_nrows, cur_loc, flip_lr=False))

        self.set_mos_size(max(cur_loc + saff_o_master.num_cols, dec_col))

        # Connect even stage latch to left, odd stage to right
        saff_cnter_l_list, saff_cnter_r_list = saff_l_list, saff_r_list
        saff_cnter_l_list, saff_cnter_r_list = saff_cnter_l_list[::-1], saff_cnter_r_list[::-1]
        for idx in range(nrows):
            _outn_o = cnter.get_pin(f'outn<{2 * (idx - 1)}>' if idx & 1 else f'outn<{2 * idx + 1}>', layer=vm_layer)
            _outp_o = cnter.get_pin(f'outp<{2 * (idx - 1)}>' if idx & 1 else f'outp<{2 * idx + 1}>', layer=vm_layer)
            _outn_e = cnter.get_pin(f'outn<{2 * idx + 1}>' if idx & 1 else f'outn<{2 * idx + 2}>', layer=vm_layer)
            _outp_e = cnter.get_pin(f'outp<{2 * idx + 1}>' if idx & 1 else f'outp<{2 * idx + 2}>', layer=vm_layer)
            _saff_o_inn = saff_cnter_l_list[idx].get_pin('inn')
            _saff_o_inp = saff_cnter_l_list[idx].get_pin('inp')
            _saff_e_inn = saff_cnter_r_list[idx].get_pin('inn')
            _saff_e_inp = saff_cnter_r_list[idx].get_pin('inp')
            if idx & 1:
                _saff_e_inn, _saff_e_inp = _saff_e_inp, _saff_e_inn
            else:
                _saff_o_inn, _saff_o_inp = _saff_o_inp, _saff_o_inn
            self.connect_differential_wires(_outn_o, _outp_o, _saff_o_inn, _saff_o_inp)
            self.connect_differential_wires(_outn_e, _outp_e, _saff_e_inn, _saff_e_inp)

        # Add pins
        tr_w_sig_ym = tr_manager.get_width(ym_layer, 'sig')
        bit_xm_list, bit_d_xm_list = [], []
        for idx in range(nrows // 2):
            self.reexport(saff_cnter_l_list[2 * idx].get_port('outp'), net_name=f'bit_n<{2 * idx}>')
            self.reexport(saff_cnter_l_list[2 * idx + 1].get_port('outn'), net_name=f'bit_dn<{2 * idx}>')
            self.reexport(saff_cnter_r_list[2 * idx].get_port('outn'), net_name=f'bit_dn<{2 * idx + 1}>')
            self.reexport(saff_cnter_r_list[2 * idx + 1].get_port('outp'), net_name=f'bit_n<{2 * idx + 1}>')

            self.reexport(saff_cnter_l_list[2 * idx].get_port('outn'), net_name=f'bit<{2 * idx}>')
            self.reexport(saff_cnter_l_list[2 * idx + 1].get_port('outp'), net_name=f'bit_d<{2 * idx}>')
            self.reexport(saff_cnter_r_list[2 * idx].get_port('outp'), net_name=f'bit_d<{2 * idx + 1}>')
            self.reexport(saff_cnter_r_list[2 * idx + 1].get_port('outn'), net_name=f'bit<{2 * idx + 1}>')
            # Get output from FFs
            bit_xm_list.extend([saff_cnter_l_list[2 * idx].get_pin('outn'),
                                saff_cnter_r_list[2 * idx + 1].get_pin('outn')])
            bit_d_xm_list.extend([saff_cnter_l_list[2 * idx + 1].get_pin('outp'),
                                  saff_cnter_r_list[2 * idx].get_pin('outp')])

        # ym locations for cnter bits to decoder
        xm_sig_mid_coord_l = bit_xm_list[0].middle
        xm_sig_mid_coord_r = bit_xm_list[1].middle
        _, tr_ym_locs_l = tr_manager.place_wires(ym_layer, ['sig'] * cnter_nrows, center_coord=xm_sig_mid_coord_l)
        _, tr_ym_locs_r = tr_manager.place_wires(ym_layer, ['sig'] * cnter_nrows, center_coord=xm_sig_mid_coord_r)

        # Connect FFs output to ym
        tr_l_idx, tr_r_idx = 0, 0
        bit_ym_list, bit_d_ym_list = [], []
        for sig in bit_xm_list:
            if sig.middle > (self.bound_box.xh + self.bound_box.xl) // 2:
                bit_ym_list.append(self.connect_to_tracks(sig, TrackID(ym_layer, tr_ym_locs_r[tr_r_idx], tr_w_sig_ym)))
                tr_r_idx += 1
            else:
                bit_ym_list.append(self.connect_to_tracks(sig, TrackID(ym_layer, tr_ym_locs_l[tr_l_idx], tr_w_sig_ym)))
                tr_l_idx += 1

        for sig in bit_d_xm_list:
            if sig.middle > (self.bound_box.xh + self.bound_box.xl) // 2:
                bit_d_ym_list.append(self.connect_to_tracks(sig, TrackID(ym_layer, tr_ym_locs_r[tr_r_idx],
                                                                         tr_w_sig_ym)))
                tr_r_idx += 1
            else:
                bit_d_ym_list.append(self.connect_to_tracks(sig, TrackID(ym_layer, tr_ym_locs_l[tr_l_idx],
                                                                         tr_w_sig_ym)))
                tr_l_idx += 1

        # Connect ym decoder bits to decoder
        for idx, sig in enumerate(bit_ym_list):
            self.connect_to_track_wires(sig, decoder.get_pin(f'in<{4 + 2 * idx}>'))
        for idx, sig in enumerate(bit_d_ym_list):
            self.connect_to_track_wires(sig, decoder.get_pin(f'in<{3 + 2 * idx}>'))

        phi_sampled_ym_tidx = self.grid.coord_to_track(ym_layer, self.bound_box.xh, RoundMode.GREATER_EQ)
        _, phi_sampled_ym_tidx = tr_manager.place_wires(ym_layer, ['dig'] * 4, align_track=phi_sampled_ym_tidx)
        tr_w_dig_ym = tr_manager.get_width(ym_layer, 'dig')
        phi_sample = []
        for idx in range(3):
            phi_sample.append(self.connect_to_tracks(decoder.get_pin(f'in<{idx}>'),
                                                     TrackID(ym_layer, phi_sampled_ym_tidx[1 + idx], tr_w_dig_ym)))

        # Get clock from FFs
        _clk_ym = self.connect_wires([inst.get_pin('clk', layer=ym_layer) for inst in saff_l_list] +
                                     [inst.get_pin('clk', layer=ym_layer) for inst in saff_r_list])
        clk_xm1 = [[inst.get_pin('clk', layer=xm1_layer) for inst in saff_l_list],
                   [inst.get_pin('clk', layer=xm1_layer) for inst in saff_r_list]]

        ym1_layer = xm1_layer + 1
        tr_w_clk_ym1 = tr_manager.get_width(ym1_layer, 'clk')
        clk_ym1_tidx = self.grid.coord_to_track(ym1_layer, clk_xm1[0][0].middle, RoundMode.NEAREST)
        clk_ym1 = [self.connect_to_tracks(clk_xm1[0], TrackID(ym1_layer, clk_ym1_tidx, tr_w_clk_ym1))]
        clk_ym1_tidx = self.grid.coord_to_track(ym1_layer, clk_xm1[1][0].middle, RoundMode.NEAREST)
        clk_ym1.append(self.connect_to_tracks(clk_xm1[1], TrackID(ym1_layer, clk_ym1_tidx, tr_w_clk_ym1)))
        if saff_r_list[0].has_port('clkb'):
            _clkb_ym = self.connect_wires([inst.get_pin('clkb', layer=ym_layer) for inst in saff_l_list] +
                                          [inst.get_pin('clkb', layer=ym_layer) for inst in saff_r_list])
            clkb_xm1 = [[inst.get_pin('clkb', layer=xm1_layer) for inst in saff_l_list],
                        [inst.get_pin('clkb', layer=xm1_layer) for inst in saff_r_list]]
            clkb_ym1_tidx = self.grid.coord_to_track(ym1_layer, clkb_xm1[0][0].middle, RoundMode.NEAREST)
            clkb_ym1 = [self.connect_to_tracks(clkb_xm1[0], TrackID(ym1_layer, clkb_ym1_tidx, tr_w_clk_ym1))]
            clkb_ym1_tidx = self.grid.coord_to_track(ym1_layer, clkb_xm1[1][0].middle, RoundMode.NEAREST)
            clkb_ym1.append(self.connect_to_tracks(clkb_xm1[1], TrackID(ym1_layer, clkb_ym1_tidx, tr_w_clk_ym1)))
            self.add_pin('clkb', clkb_ym1, connect=True)

        # Fill taps in buffer rows
        for idx in range(num_buf):
            self.fill_tap(self.num_tile_rows - idx - 1, port_mode=SubPortMode.ODD,
                          connect_to_sup_ow=True, ofst=min_sep)

        # Connect xm layer supplies
        vdd_xm = [w for inst in saff_r_list + saff_l_list + [decoder] for w in inst.get_all_port_pins('VDD')]
        vss_xm = [w for inst in saff_r_list + saff_l_list + [decoder] for w in inst.get_all_port_pins('VSS')]
        vdd_xm.extend(cnter.get_all_port_pins('VDD_xm'))
        vss_xm.extend(cnter.get_all_port_pins('VSS_xm'))
        vdd_xm = self.connect_wires(vdd_xm)
        vss_xm = self.connect_wires(vss_xm)
        vdd_xm = [w for warr in vdd_xm for w in warr.to_warr_list()]
        vss_xm = [w for warr in vss_xm for w in warr.to_warr_list()]

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

        vdd_ym = [self.connect_to_tracks(vdd_xm, TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[::2]]
        vss_ym = [self.connect_to_tracks(vss_xm, TrackID(ym_layer, tid, tr_w_sup_ym))
                  for tid in ym_sup_tidxs[1::2]]
        vdd_ym = self.extend_wires(vdd_ym, upper=vss_ym[0].upper, lower=vss_ym[0].lower)

        vdd_cnter_ym_list = cnter.get_all_port_pins('VDD_ym')
        vss_cnter_ym_list = cnter.get_all_port_pins('VSS_ym')
        vdd_cnter_ym_list = self.extend_wires(vdd_cnter_ym_list, lower=self.bound_box.yl,
                                              upper=self.bound_box.yh)
        vss_cnter_ym_list = self.extend_wires(vss_cnter_ym_list, lower=self.bound_box.yl,
                                              upper=self.bound_box.yh)

        # xm1 supplies
        vss_xm1_list, vdd_xm1_list = [], []
        for vss in vss_xm:
            xm1_tidx = self.grid.coord_to_track(xm1_layer, self.grid.track_to_coord(xm_layer, vss.track_id.base_index),
                                                mode=RoundMode.NEAREST)
            vss_xm1 = self.connect_to_tracks(vss_ym, TrackID(xm1_layer, xm1_tidx, tr_w_sup_xm1, grid=self.grid),
                                             track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            vss_xm1_list.append(vss_xm1)

        for vdd in vdd_xm:
            xm1_tidx = self.grid.coord_to_track(xm1_layer, self.grid.track_to_coord(xm_layer, vdd.track_id.base_index),
                                                mode=RoundMode.NEAREST)
            vdd_xm1 = self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, xm1_tidx, tr_w_sup_xm1, grid=self.grid),
                                             track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
            vdd_xm1_list.append(vdd_xm1)
        for idx in range(nrows):
            self.reexport(decoder.get_port(f'out<{idx}>'))

        self.connect_to_track_wires(vss_xm1_list, vss_cnter_ym_list)
        self.connect_to_track_wires(vdd_xm1_list, vdd_cnter_ym_list)

        # Add pins
        self.add_pin('clk', clk_ym1, connect=True)

        self.add_pin('cnter_clkn', cnter.get_pin('clkn'))
        self.add_pin('cnter_clkp', cnter.get_pin('clkp'))
        self.add_pin('phi_sampled', phi_sample[0])
        self.add_pin('bufp_sampled<0>', phi_sample[1])
        self.add_pin('bufp_sampled<1>', phi_sample[2])

        self.add_pin('VDD_ym', vdd_ym, label='VDD')
        self.add_pin('VSS_ym', vss_ym, label='VSS')

        # Export supply to higher layers
        for pin in cnter.port_names_iter():
            if 'out' in pin:
                self.add_pin(pin.replace('out', 'cnter_out'), cnter.get_pin(pin, layer=vm_layer))

        top_sup_layer = self.params['top_sup_layer']
        vss_hor_coord = [self.grid.track_to_coord(xm1_layer, warr.track_id.base_index) for warr in vss_xm1_list]
        vdd_hor_coord = [self.grid.track_to_coord(xm1_layer, warr.track_id.base_index) for warr in vdd_xm1_list]
        vss_vert_bnd = [vss_xm1_list[0].lower, vss_xm1_list[0].upper]
        vdd_vert_bnd = [vdd_xm1_list[0].lower, vdd_xm1_list[0].upper]
        self.add_pin('VDD_xm1', vdd_xm1_list, label='VDD')
        self.add_pin('VSS_xm1', vss_xm1_list, label='VSS')
        last_vdd_list, last_vss_list = vdd_xm1_list, vss_xm1_list
        for idx in range(xm1_layer + 1, top_sup_layer + 1):
            tr_w_sup = tr_manager.get_width(idx, 'sup')
            lay_dir = self.grid.get_direction(idx)
            tr_w_sep = tr_manager.get_sep(idx, ('sup', 'sup')) if tr_w_sup > 0 else 0
            sup_w = tr_manager.get_width(idx, 'sup')
            sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
            sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
            if lay_dir == Orient2D.x and lay_dir < 10:
                vss_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vss_hor_coord]
                vdd_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vdd_hor_coord]
                last_vss_list = [self.connect_to_tracks(last_vss_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
                                 vss_tidx_list]
                last_vdd_list = [self.connect_to_tracks(last_vdd_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
                                 vdd_tidx_list]
                last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
                last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
            elif lay_dir == Orient2D.x:
                last_vdd_list, last_vss_list = self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list],
                                                                        idx - 1, self.bound_box)
            else:
                vss_tid_l = self.grid.coord_to_track(idx, vss_vert_bnd[0] + sup_w, mode=RoundMode.NEAREST)
                vss_tid_r = self.grid.coord_to_track(idx, vss_vert_bnd[1] - sup_w, mode=RoundMode.NEAREST)
                vss_locs = self.get_tids_between(idx, vss_tid_l, vss_tid_r, width=tr_w_sup,
                                                 sep=tr_w_sep, sep_margin=None, include_last=False)
                vdd_tid_l = self.grid.coord_to_track(idx, vdd_vert_bnd[0] + sup_w, mode=RoundMode.NEAREST)
                vdd_tid_r = self.grid.coord_to_track(idx, vdd_vert_bnd[1] - sup_w, mode=RoundMode.NEAREST)
                vdd_locs = self.get_tids_between(idx, vdd_tid_l, vdd_tid_r, width=tr_w_sup,
                                                 sep=tr_w_sep, sep_margin=None, include_last=False)
                last_vdd_list = [self.connect_to_tracks(last_vdd_list, tid) for tid in vdd_locs]
                last_vss_list = [self.connect_to_tracks(last_vss_list, tid) for tid in vss_locs]
                last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.yl, upper=self.bound_box.yh)
                last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.yl, upper=self.bound_box.yh)

            self.add_pin(f'VDD{idx}' if idx < top_sup_layer else 'VDD',
                         last_vdd_list, hide=idx < top_sup_layer, label='VDD')
            self.add_pin(f'VSS{idx}' if idx < top_sup_layer else 'VSS',
                         last_vss_list, hide=idx < top_sup_layer, label='VSS')

        self.add_pin('VDD_xm1' if top_sup_layer > xm1_layer else 'VDD', vdd_xm1_list, label='VDD')
        self.add_pin('VSS_xm1' if top_sup_layer > xm1_layer else 'VSS', vss_xm1_list, label='VSS')
        self._sch_params = dict(
            ff_params=saff_e_master.sch_params,
            cnter_params=cnter_master.sch_params,
            dec_params=cnter_decoder_master.sch_params,
        )


class CnterSAFFDecoderWrap(GenericWrapper, TemplateBaseZL):
    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = GenericWrapper.get_params_info()
        ans['top_sup_layer'] = 'Top supply layer'
        return ans

    def draw_layout(self) -> None:
        params = self.params
        cls_name: str = params['cls_name']
        dut_params: Param = params['params']
        export_hidden: bool = params['export_hidden']
        export_private: bool = params['export_private']
        half_blk_x: bool = params['half_blk_x']
        half_blk_y: bool = params['half_blk_y']
        top_layer: int = params['top_layer']

        gen_cls = cast(Type[MOSBase], import_class(cls_name))
        master = self.new_template(gen_cls, params=dut_params)

        top_sup_layer = self.params['top_sup_layer']
        top_layer = max(top_layer, master.top_layer, top_sup_layer)
        inst = self.wrap_mos_base(master, export_hidden, export_private, half_blk_x=half_blk_x, half_blk_y=half_blk_y,
                                  top_layer=top_layer)
        vdd_xm1_list, vss_xm1_list = inst.get_all_port_pins('VDD_xm1'), inst.get_all_port_pins('VSS_xm1')
        xm1_layer = vdd_xm1_list[0].layer_id
        tr_manager = inst.master.tr_manager

        vss_hor_coord = [self.grid.track_to_coord(xm1_layer, warr.track_id.base_index) for warr in vss_xm1_list]
        vdd_hor_coord = [self.grid.track_to_coord(xm1_layer, warr.track_id.base_index) for warr in vdd_xm1_list]
        last_vdd_list, last_vss_list = vdd_xm1_list, vss_xm1_list
        for idx in range(xm1_layer + 1, top_sup_layer + 1):
            tr_w_sup = tr_manager.get_width(idx, 'sup')
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.x and idx < 10:
                vss_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.LESS_EQ) for coord in vss_hor_coord]
                vdd_tidx_list = [self.grid.coord_to_track(idx, coord, RoundMode.NEAREST) for coord in vdd_hor_coord]
                last_vss_list = [self.connect_to_tracks(last_vss_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
                                 vss_tidx_list]
                last_vdd_list = [self.connect_to_tracks(last_vdd_list, TrackID(idx, tidx, tr_w_sup)) for tidx in
                                 vdd_tidx_list]
                last_vss_list = self.extend_wires(last_vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
                last_vdd_list = self.extend_wires(last_vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
            elif lay_dir == Orient2D.x:
                last_vdd_list, last_vss_list = self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list],
                                                                        idx - 1, self.bound_box)
            else:
                sup_w = tr_manager.get_width(idx, 'sup')
                sup_sep_ntr = self.get_track_sep(idx, sup_w, sup_w)
                sup_w = self.grid.get_track_info(idx).pitch * sup_sep_ntr
                _sup_bbox = BBox(self.bound_box.xl + sup_w, self.bound_box.yl, self.bound_box.xh - sup_w,
                                 self.bound_box.yh)
                last_vss_list, last_vdd_list = \
                    self.connect_supply_warr(tr_manager, [last_vss_list, last_vdd_list], idx - 1, bbox=_sup_bbox)

            self.add_pin(f'VDD{idx}' if idx < top_sup_layer else 'VDD',
                         last_vdd_list, hide=idx < top_sup_layer, label='VDD')
            self.add_pin(f'VSS{idx}' if idx < top_sup_layer else 'VSS',
                         last_vss_list, hide=idx < top_sup_layer, label='VSS')


class VCOTopHalf(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None

    def tr_manager(self):
        return self._tr_manager

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_slice_se')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            top_sup_layer='Top supply layer',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            ro_saff_dec_params='ro_saff_dec_params',
            cnter_saff_dec_params='Ring parameters',
            cnter_buf_saff_params='cnter_buf_saff_params',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(top_sup_layer=0)

    def draw_layout(self) -> None:
        ring_saff_params: Param = self.params['ro_saff_dec_params']
        cnter_saff_params: Param = self.params['cnter_saff_dec_params']
        cnter_buf_saff_params: Param = self.params['cnter_buf_saff_params']

        if isinstance(cnter_saff_params, str):
            cnter_yaml = read_yaml(cnter_saff_params)
            cnter_saff_params = cnter_yaml['params']
        if isinstance(ring_saff_params, str):
            ring_yaml = read_yaml(ring_saff_params)
            ring_saff_params = ring_yaml['params']
        if isinstance(cnter_buf_saff_params, str):
            buf_yaml = read_yaml(cnter_buf_saff_params)
            cnter_buf_saff_params = buf_yaml['params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        self._tr_manager = tr_manager

        cnter_saff_master = self.new_template(IntegrationWrapper, params=cnter_saff_params)
        ring_saff_master: ROSAFFDecoder = self.new_template(ROSAFFDecoder, params=ring_saff_params)
        buf_saff_master: ROSAFFDecoder = self.new_template(CnterBufInvSAFF, params=cnter_buf_saff_params)

        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      cnter_saff_master.core.core.draw_base_info[0].arr_info.lch)
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1
        xm2_layer = ym1_layer + 1

        # Make templates
        top_layer = max(cnter_saff_master.top_layer, ring_saff_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        # Floorplanning according saff clocking
        # tr_sp_coord = self.grid.htr_to_coord(xm1_layer, HalfInt.convert(tr_sp_sup_xm1).dbl_value)
        sp_xm2_ntr, _ = tr_manager.place_wires(xm2_layer, ['dum', 'clk', 'dum'])
        tr_sp_coord = self.grid.track_to_coord(xm2_layer, sp_xm2_ntr)
        cnter = self.add_instance(cnter_saff_master, xform=Transform(0, 0))

        y_buf = -(-cnter.bound_box.yh // h_blk) * h_blk
        x_buf = cnter.bound_box.xl + cnter_saff_master.core_inst.bound_box.xh - buf_saff_master.bound_box.w
        buf = self.add_instance(buf_saff_master, xform=Transform(x_buf, y_buf))
        y_vco = -(-(buf.bound_box.yh + tr_sp_coord) // h_blk) * h_blk
        ring = self.add_instance(ring_saff_master, xform=Transform(0, y_vco))
        h_tot = -(-ring.bound_box.yh // h_blk) * h_blk
        w_tot = max(ring.bound_box.xh, cnter.bound_box.xh)
        w_tot = -(-w_tot // w_blk) * w_blk
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot), half_blk_x=False)
        basefill_bbox(self, BBox(cnter.bound_box.xh, self.bound_box.yl, self.bound_box.xh,
                                 buf.bound_box.yh))
        basefill_bbox(self, BBox(self.bound_box.xl, cnter.bound_box.yh, buf.bound_box.xl,
                                 ring.bound_box.yl))
        basefill_bbox(self, BBox(buf.bound_box.xl, buf.bound_box.yh, self.bound_box.xh,
                                 ring.bound_box.yl))
        # basefill_bbox(self, BBox(self.bound_box.xl, cnter.bound_box.yh, cnter.bound_box.xh,
        #                          buf.bound_box.yl))

        # Connect between ring and counter
        phi_sampled_xm_tidx = self.grid.coord_to_track(xm_layer, (buf.bound_box.yh + ring.bound_box.yl) // 2,
                                                       mode=RoundMode.NEAREST)
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')

        phi_sampled_xm = self.connect_to_tracks(ring.get_pin('phi_sampled'),
                                                TrackID(xm_layer, phi_sampled_xm_tidx, tr_w_sig_xm))

        self.connect_to_track_wires(cnter.get_pin('phi_sampled'), phi_sampled_xm)

        phi_b_xm_tidx = tr_manager.get_next_track(xm_layer, phi_sampled_xm_tidx, 'sig', 'sig', up=True)

        phi_out_cnter_xm = self.connect_to_tracks(buf.get_pin('phi'),
                                                  TrackID(xm_layer, phi_sampled_xm_tidx, tr_w_sig_xm))
        phi_outb_cnter_xm = self.connect_to_tracks(buf.get_pin('phib'), TrackID(xm_layer, phi_b_xm_tidx, tr_w_sig_xm))

        self.connect_to_track_wires(phi_out_cnter_xm, ring.get_pin('phi_out'))
        self.connect_to_track_wires(phi_outb_cnter_xm, ring.get_pin('phib_out'))

        self.connect_to_track_wires(buf.get_pin('bufp_sampled<0>'), cnter.get_pin('bufp_sampled<0>'))
        self.connect_to_track_wires(buf.get_pin('bufp_sampled<1>'), cnter.get_pin('bufp_sampled<1>'))

        # Connect buffer to counter

        cnter_clk_xm_tidx = self.grid.coord_to_track(xm_layer, (buf.bound_box.yl + cnter.bound_box.yh) // 2,
                                                     RoundMode.NEAREST)
        cnter_clkn_tidx = tr_manager.get_next_track(xm_layer, cnter_clk_xm_tidx, 'dum', 'clk', up=1)
        cnter_clkp_tidx = tr_manager.get_next_track(xm_layer, cnter_clk_xm_tidx, 'dum', 'clk', up=-1)
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        self.connect_matching_tracks([buf.get_pin('outn'), buf.get_pin('outp')], xm_layer,
                                     [cnter_clkn_tidx, cnter_clkp_tidx], width=tr_w_clk_xm)
        self.connect_matching_tracks([cnter.get_pin('cnter_clkn'), cnter.get_pin('cnter_clkp')], xm_layer,
                                     [cnter_clkn_tidx, cnter_clkp_tidx], width=tr_w_clk_xm)
        _cnter_clkn_xm = self.connect_to_tracks([buf.get_pin('outn'), cnter.get_pin('cnter_clkn')],
                                                TrackID(xm_layer, cnter_clkn_tidx, tr_w_clk_xm))
        _cnter_clkp_xm = self.connect_to_tracks([buf.get_pin('outp'), cnter.get_pin('cnter_clkp')],
                                                TrackID(xm_layer, cnter_clkp_tidx, tr_w_clk_xm))

        num_lsb = 0
        lsb_xm_list = []
        for pin in ring.port_names_iter():
            if pin.startswith('bit'):
                num_lsb += 1
                # self.reexport(ring.get_port(pin), net_name=pin.replace('bit', 'lsb'))
                lsb_xm_list.append(ring.get_pin(pin))
            if pin.startswith('vctrl'):
                self.reexport(ring.get_port(pin), connect=True)
            if 'vbot' in pin:
                self.reexport(ring.get_port(pin), connect=True)
            if 'vtop' in pin:
                self.reexport(ring.get_port(pin), connect=True)

        clk_xm2_tidx = self.grid.coord_to_track(xm2_layer, cnter.bound_box.yh, RoundMode.NEAREST)
        clkb_xm2_tidx = self.grid.coord_to_track(xm2_layer, buf.bound_box.yh + tr_sp_coord // 2, RoundMode.NEAREST)

        # Connect clk xm1 inthe middle
        tr_w_clk_xm2 = tr_manager.get_width(xm2_layer, 'clk')
        clk_xm2 = self.connect_to_tracks([ring.get_pin('clk')] + cnter.get_all_port_pins('clk'),
                                         TrackID(xm2_layer, clk_xm2_tidx if ring.has_port('clkb') else clkb_xm2_tidx,
                                                 tr_w_clk_xm2), track_lower=self.bound_box.xl)
        self.connect_to_track_wires(cnter.get_all_port_pins('clk')[1], buf.get_all_port_pins('clk', layer=xm1_layer))
        self.add_pin('clk', clk_xm2)
        if ring.has_port('clkb'):
            clkb_xm2 = self.connect_to_tracks([ring.get_pin('clkb')] + cnter.get_all_port_pins('clkb'),
                                              TrackID(xm2_layer, clkb_xm2_tidx, tr_w_clk_xm2),
                                              track_lower=self.bound_box.xl)
            self.connect_to_track_wires(cnter.get_all_port_pins('clkb')[1],
                                        buf.get_all_port_pins('clkb', layer=xm1_layer))
            self.add_pin('clkb', clkb_xm2)

        num_msb = 0
        msb_xm_list = []
        for pin in cnter.port_names_iter():
            if pin.startswith('out'):
                num_msb += 1
                msb_xm_list.append(cnter.get_pin(pin))
                # self.reexport(cnter.get_port(pin), net_name=pin.replace('out', 'msb'))

        # bring result bits out
        right_edge = max(cnter.bound_box.xh, ring.bound_box.xh)
        _, ym_bits_locs = \
            tr_manager.place_wires(ym_layer, ['sig'] * (num_msb + num_lsb + 1),
                                   align_track=self.grid.coord_to_track(ym_layer, right_edge, RoundMode.GREATER_EQ))
        ym_msb_locs = ym_bits_locs[1:1 + num_msb]
        ym_lsb_locs = ym_bits_locs[1 + num_msb: 1 + num_msb + num_lsb]

        ym_msb_list = self.connect_matching_tracks(msb_xm_list, ym_layer, ym_msb_locs,
                                                   width=tr_manager.get_width(ym_layer, 'dig'),
                                                   track_lower=self.bound_box.yl)

        ym_lsb_list = [self.connect_to_tracks(xmpin, TrackID(ym_layer, ymlocs, tr_manager.get_width(ym_layer, 'dig')))
                       for xmpin, ymlocs in zip(lsb_xm_list, ym_lsb_locs)]
        ym_lsb_list = self.extend_wires(ym_lsb_list, lower=self.bound_box.yl)

        cnter_buf_vdd_xm1 = cnter.get_all_port_pins('VDD') + buf.get_all_port_pins('VDD')
        cnter_buf_vss_xm1 = cnter.get_all_port_pins('VSS') + buf.get_all_port_pins('VSS')

        top_sup_layer = self.params['top_sup_layer']
        last_vdd_list, last_vss_list = cnter_buf_vdd_xm1, cnter_buf_vss_xm1

        top_layer_dir = self.grid.get_direction(top_layer)
        top_vert_layer = top_layer if top_layer_dir == Orient2D.y else top_layer - 1
        blk_w, _ = self.grid.get_block_size(top_vert_layer)

        sup_bbox = BBox(max(cnter.bound_box.xl, blk_w), 0, self.bound_box.xh, buf.bound_box.yh)
        for idx in range(xm1_layer + 1, ring_saff_master.top_layer + 1):
            last_vdd_list, last_vss_list = \
                self.connect_supply_warr(tr_manager, [last_vdd_list, last_vss_list], idx - 1, sup_bbox,
                                         side_sup=False, extend_lower_layer=False)

        power_dict = self.connect_supply_stack_warr(tr_manager,
                                                    [ring.get_all_port_pins('VDD') + last_vdd_list,
                                                     ring.get_all_port_pins('VSS') + last_vss_list],
                                                    ring_saff_master.top_layer, top_sup_layer, self.bound_box,
                                                    extend_lower_layer=True,
                                                    align_upper=True, side_sup=True)
        self.add_pin('VDD', power_dict[0][top_sup_layer])
        self.add_pin('VSS', power_dict[1][top_sup_layer])

        for idx, pin in enumerate(ym_msb_list):
            self.add_pin(f'msb<{idx}>', pin, mode=PinMode.LOWER)
        for idx, pin in enumerate(ym_lsb_list):
            self.add_pin(f'lsb<{idx}>', pin, mode=PinMode.LOWER)

        self.reexport(ring.get_port('tail'))

        # export pin for probing
        self.reexport(buf.get_port('outp'), net_name='cnter_clkp')
        self.reexport(buf.get_port('outn'), net_name='cnter_clkn')
        for pin in cnter.port_names_iter():
            if 'cnter_out' in pin:
                self.reexport(cnter.get_port(pin))
        for pin in ring.port_names_iter():
            if 'phi' in pin and ('out' not in pin and 'sampled' not in pin):
                self.reexport(ring.get_port(pin))

        self._sch_params = dict(
            nlsb=sum(('bit' in pin) for pin in ring.port_names_iter()),
            nmsb=sum(('out' in pin and 'cnter' not in pin) for pin in cnter.port_names_iter()),
            ro_params=ring_saff_master.sch_params,
            buf_params=buf_saff_master.sch_params,
            cnter_params=cnter_saff_master.sch_params,
        )


class VCOTop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vco_slice')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = VCOTopHalf.get_params_info()
        ans['even_center'] = 'True to force center column to be even.'
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = VCOTopHalf.get_default_param_values()
        ans['even_center'] = False
        return ans

    def draw_layout(self):
        master: VCOTopHalf = self.new_template(VCOTopHalf, params=self.params)
        top_layer = max(master.top_layer, self.params['top_sup_layer'])
        blk_w, blk_h = self.grid.get_block_size(top_layer)
        inst_l = self.add_instance(master, xform=Transform(-(-master.bound_box.w // blk_w) * blk_w, 0, Orientation.MY))
        inst_r = self.add_instance(master, xform=Transform(-(-master.bound_box.w // blk_w) * blk_w, 0, Orientation.R0))
        tot_h = -(-inst_r.bound_box.yh // blk_h) * blk_h
        tot_w = -(-inst_r.bound_box.xh // blk_w) * blk_w
        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))
        clk = self.connect_wires([inst_r.get_pin('clk'), inst_l.get_pin('clk')])

        clk_vert_layer = clk[0].layer_id + 1
        tr_manager = master.tr_manager()
        clk_vert_mid_tidx = self.grid.coord_to_track(clk_vert_layer, clk[0].middle, RoundMode.NEAREST)
        if inst_r.has_port('clkb'):
            clkb = self.connect_wires([inst_r.get_pin('clkb'), inst_l.get_pin('clkb')])
            self.add_pin('clkb', clkb)
            clk_hor_mid_tidx_n = tr_manager.get_next_track(clk_vert_layer, clk_vert_mid_tidx, 'dum', 'clk', up=1)
            clk_hor_mid_tidx_p = tr_manager.get_next_track(clk_vert_layer, clk_vert_mid_tidx, 'dum', 'clk', up=-1)
            clk_hor, clkb_hor = self.connect_matching_tracks([clk, clkb], clk_vert_layer,
                                                             [clk_hor_mid_tidx_n, clk_hor_mid_tidx_p],
                                                             width=tr_manager.get_width(clk_vert_layer, 'clk_top'))
            clk_hor_layer = clk_vert_layer + 1
            _, clk_hod_mid_locs = tr_manager.place_wires(clk_hor_layer, ['clk_top'] * 2, center_coord=clk_hor.middle)
            clk_hor, clkb_hor = self.connect_matching_tracks([clk_hor, clkb_hor], clk_hor_layer, clk_hod_mid_locs,
                                                             width=tr_manager.get_width(clk_hor_layer, 'clk_top'),
                                                             track_lower=self.bound_box.xl)
            self.add_pin('clk', clk_hor)
            self.add_pin('clkb', clkb_hor)
        else:
            clk_vert = self.connect_to_tracks(clk, TrackID(clk_vert_layer, clk_vert_mid_tidx,
                                                           width=tr_manager.get_width(clk_vert_layer, 'clk_top')))
            clk_hor_layer = clk_vert_layer + 1
            clk_hor_tidx = self.grid.coord_to_track(clk_hor_layer, clk_vert.middle, RoundMode.NEAREST)
            clk_hor = self.connect_to_tracks(clk_vert, TrackID(clk_hor_layer, clk_hor_tidx,
                                                               tr_manager.get_width(clk_hor_layer, 'clk_top')),
                                             min_len_mode=MinLenMode.MIDDLE)
            self.add_pin('clk', clk_hor)

        nlsb = master.sch_params['nlsb']
        nmsb = master.sch_params['nmsb']

        for layer in dict(inst_l.get_port('tail').items()).keys():
            self.connect_wires(inst_l.get_all_port_pins('tail', layer=layer) +
                               inst_r.get_all_port_pins('tail', layer=layer))
        for idx in range(nlsb):
            self.add_pin(f'bit_n<{idx}>', inst_l.get_pin(f'lsb<{idx}>'))
            self.add_pin(f'bit_p<{idx}>', inst_r.get_pin(f'lsb<{idx}>'))
        for idx in range(nmsb):
            self.add_pin(f'bit_n<{idx + nlsb}>', inst_l.get_pin(f'msb<{idx}>'))
            self.add_pin(f'bit_p<{idx + nlsb}>', inst_r.get_pin(f'msb<{idx}>'))
        self.add_pin('vctrl_n', inst_l.get_all_port_pins('vctrl'))
        self.add_pin('vctrl_p', inst_r.get_all_port_pins('vctrl'))

        self.reexport(inst_l.get_port('cnter_clkn'), net_name='cnter_clkn_n')
        self.reexport(inst_l.get_port('cnter_clkp'), net_name='cnter_clkp_n')
        self.reexport(inst_r.get_port('cnter_clkn'), net_name='cnter_clkn_p')
        self.reexport(inst_r.get_port('cnter_clkp'), net_name='cnter_clkp_p')

        for pinname in inst_l.port_names_iter():
            if 'cnter_outn' in pinname:
                self.reexport(inst_l.get_port(pinname), net_name=pinname.replace('cnter_outn', 'cnter_outn_n'))
                self.reexport(inst_r.get_port(pinname), net_name=pinname.replace('cnter_outn', 'cnter_outn_p'))
            if 'cnter_outp' in pinname:
                self.reexport(inst_l.get_port(pinname), net_name=pinname.replace('cnter_outp', 'cnter_outp_n'))
                self.reexport(inst_r.get_port(pinname), net_name=pinname.replace('cnter_outp', 'cnter_outp_p'))
            if 'phi' in pinname and ('out' not in pinname and 'sampled' not in pinname and 'buf' not in pinname):
                self.reexport(inst_l.get_port(pinname), net_name=pinname.replace('phi', 'phi_n'))
                self.reexport(inst_r.get_port(pinname), net_name=pinname.replace('phi', 'phi_p'))
            if 'phi_buf' in pinname:
                self.reexport(inst_l.get_port(pinname), net_name=pinname.replace('phi_buf', 'phi_buf_n'))
                self.reexport(inst_r.get_port(pinname), net_name=pinname.replace('phi_buf', 'phi_buf_p'))

        self.reexport(inst_l.get_port('VDD'), connect=True)
        self.reexport(inst_l.get_port('VSS'), connect=True)
        self.reexport(inst_r.get_port('VDD'), connect=True)
        self.reexport(inst_r.get_port('VSS'), connect=True)

        self._sch_params = dict(
            slice_params=master.sch_params
        )
