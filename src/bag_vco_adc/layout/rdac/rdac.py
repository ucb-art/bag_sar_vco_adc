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


from typing import Any, Mapping, Dict, Optional, Type

from bag3_analog.layout.res.ladder import ResLadder

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.routing import TrackID
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from pybag.core import Transform, BBox
from pybag.enum import Orientation, Direction, RoundMode, PinMode, Orient2D
from xbase.layout.array.top import ArrayBaseWrapper
from xbase.layout.mos.placement.data import MOSArrayPlaceInfo
from .rdac_decoder import RDACMuxDecoder
from ..others.decap import DecapArray
from ..ra.ra_cap import MOMCapOnMOS
from ..util.template import TemplateBaseZL
from ..util.util import basefill_bbox
from ..util.wrapper import GenericWrapper


class RDAC(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None
        self._num_bits = 0

    @property
    def num_bits(self):
        return self._num_bits

    @property
    def tr_manager(self):
        return self._tr_manager

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_half')

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            dec_params='Parameters for buffer x',
            res_params='Parameters for rladders',
            res_bias_params='Parameters for bias resistor',
            cap_params='common_mode',
            gnd_cap_params='decap reference to supply',
            start_row='Starting row of transistor, used to match resistor',
            top_layer='',
            pside='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(start_row=1, pside=False, cap_params=None, res_bias_params=None, gnd_cap_params=None)

    def draw_layout(self) -> None:
        dec_params: ImmutableSortedDict[str, Any] = self.params['dec_params']
        res_params: ImmutableSortedDict = self.params['res_params']
        cap_params: ImmutableSortedDict = self.params['cap_params']
        gnd_cap_params: ImmutableSortedDict = self.params['gnd_cap_params']
        dec_gen_params = dict(
            cls_name=RDACMuxDecoder.get_qualified_name(),
            params=dec_params.copy(append=dict(pside=self.params['pside'])),
            export_private=False,
        )
        res_gen_params = dict(
            cls_name=ResLadder.get_qualified_name(),
            params=res_params
        )
        differential = dec_params['differential']

        """ This generator assume good alignment btw resistor and transistors
        1. The height of tansistor tile match unit resistor height
        2. The number of horizontal xm layer for row decoder can fit in unit row
        """
        res_master = self.new_template(ArrayBaseWrapper, params=res_gen_params)
        dec_master = self.new_template(GenericWrapper, params=dec_gen_params)
        gnd_cap_gen_params = gnd_cap_params.to_dict()
        gnd_cap_gen_params['unit_params'] = gnd_cap_gen_params['unit_params'].to_dict()
        gnd_cap_gen_params['unit_params']['seg'] = dec_master.core.num_cols

        # Make bias resistor template
        res_bias_params = self.params['res_bias_params']
        if res_bias_params:
            res_pinfo = res_params['pinfo'].to_dict()
            res_pinfo['nx'], res_pinfo['ny'] = res_bias_params['nx'], res_bias_params['ny']
            res_bias_gen_params = dict(top_vdd=False, bot_vss=False, pinfo=res_pinfo,
                                       nx_dum=res_bias_params['nx_dum'], ny_dum=res_bias_params['ny_dum'])
            res_bias_gen_params = dict(cls_name=ResLadder.get_qualified_name(),
                                       params=res_bias_gen_params)
            res_bias_master = self.new_template(ArrayBaseWrapper, params=res_bias_gen_params)
        else:
            res_bias_master = None

        cap_diff_master = self.new_template(DecapArray, params=gnd_cap_gen_params)
        if cap_params:
            cap_gen_params = cap_params.copy(append=dict(width=res_master.core.bound_box.w))
            cap_gen_params = dict(
                cls_name=MOMCapOnMOS.get_qualified_name(),
                params=cap_gen_params,
                export_private=False,
            )
            cap_cm_master = self.new_template(GenericWrapper, params=cap_gen_params)
        else:
            cap_cm_master = None

        float_output = dec_master.sch_params['float_output']

        # --- Placement --- #

        conn_layer = \
            MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                             dec_gen_params['params']['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # Matching height between res and dec
        drow = self.params['start_row'] + res_params['ny_dum'] - dec_master.core.mux_start_row
        unit_height = res_master.core.place_info.height
        dy_dec = drow * unit_height

        # Checking resistor rows and columns
        xres = res_params['pinfo']['nx'] - 2 * res_params['nx_dum']
        yres = res_params['pinfo']['ny'] - 2 * res_params['ny_dum']
        ndec_row = 2 ** dec_params['dec_row_params']['nbits']
        ndec_col = 2 ** dec_params['dec_col_params']['nbits']
        if yres < self.params['start_row'] + ndec_col or xres < ndec_row:
            raise ValueError("Doesn't have large enough res array to decode")

        top_layer = max(res_master.top_layer, dec_master.top_layer)
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        res_w, res_h = res_master.bound_box.w, res_master.bound_box.h
        dec_w, dec_h = dec_master.bound_box.w, dec_master.bound_box.h
        tr_manager = dec_master.core.tr_manager

        # If there is bias res, add bias res first
        res_margin = res_master.bound_box.w - res_master.core.bound_box.w
        if res_bias_master:
            dx = res_bias_master.bound_box.w
            res_bias = self.add_instance(res_bias_master,
                                         xform=Transform(dx, 0 if dy_dec > 0 else -dy_dec, Orientation.MY))
            if res_bias_params['ny_dum'] + res_bias_params['ny'] == res_params['ny_dum'] + res_params['pinfo']['ny']:
                dx -= res_margin
        else:
            res_bias = None
            dx = 0

        res = self.add_instance(res_master, xform=Transform(dx, 0 if dy_dec > 0 else -dy_dec, Orientation.R0))
        dec_x = -(-(res_w + dx) // blk_w) * blk_w
        dec = self.add_instance(dec_master, xform=Transform(dec_x, dy_dec if dy_dec > 0 else 0, Orientation.R0))

        # change to place decap
        blk_w, blk_h = self.grid.get_block_size(gnd_cap_params['top_layer'], half_blk_x=False, half_blk_y=False)
        cap_y = -(-dec.bound_box.yh // blk_h) * blk_h
        # breakpoint()
        cap_diff = self.add_instance(cap_diff_master, xform=Transform(dec_x, cap_y, Orientation.R0))

        cap_y = -(-res.bound_box.yh // blk_h) * blk_h
        if differential:
            cap_cm = self.add_instance(cap_cm_master, xform=Transform(0, cap_y, Orientation.R0))
        else:
            cap_cm = None
        # Assume one dummy at sides
        res_start_idx = self.params['start_row'] * (res_params['pinfo']['nx'] - 2)
        tot_tap = 2 ** dec_master.core.num_bits

        # === calculate DAC configuration ===
        dec_range = 1
        res_stop_idx = res_start_idx + tot_tap
        tot_res = xres * yres
        vbot = 0.5 if differential and self.params['pside'] else 0
        vtop = 0.5 if differential and not self.params['pside'] else 1
        if res_bias_master:
            bias_tot = (res_bias_params['nx'] - 2 * res_bias_params['nx_dum']) * \
                       (res_bias_params['ny'] - 2 * res_bias_params['ny_dum'])
            if res_bias_params['ref'] == 'VDD':
                res_start_idx, res_stop_idx = res_start_idx, res_stop_idx
            else:
                res_start_idx, res_stop_idx = tot_res - res_stop_idx + bias_tot, \
                                              tot_res - res_start_idx + bias_tot
            tot_res += bias_tot

        if self.params['pside'] and differential:
            vdac_top = (tot_res - res_start_idx) * (vtop - vbot) / tot_res + vbot
            vdac_bot = (tot_res - res_stop_idx) * (vtop - vbot) / tot_res + vbot
        else:
            vdac_top = res_stop_idx * (vtop - vbot) / tot_res
            vdac_bot = res_start_idx * (vtop - vbot) / tot_res

        step = 1 / tot_res
        dac_type = 'DIFF' if differential else 'SE'
        print(f" === Configuretion ===  \n{dac_type}\nFull Range{vbot} -- {vtop}"
              f"\nAdjustable Range {vdac_bot} -- {vdac_top}"
              f"\nStep: {step}")
        # ===

        res_start_idx = self.params['start_row'] * (res_params['pinfo']['nx'] - 2)
        tot_tap = 2 ** dec_master.core.num_bits
        for idx in range(tot_tap):
            self.connect_to_track_wires(dec.get_all_port_pins(f'tap<{idx}>'),
                                        res.get_all_port_pins(f'out<{res_start_idx + idx}>'))

        # Connect vcm
        tr_w_ym = tr_manager.get_width(ym_layer, 'sig')
        tr_w_ym_bias = tr_manager.get_width(ym_layer, 'bias')
        cap_diff_plus_ym = cap_diff.get_all_port_pins(f'PLUS{ym_layer}')[-1]
        cap_diff_minus_ym = cap_diff.get_all_port_pins(f'MINUS{ym_layer}')[-1]

        if differential:
            cap_diff_plus_xm = [cap_diff.get_all_port_pins(f'PLUS{xm_layer}')[2]]
            ym_tidx = self.grid.coord_to_track(ym_layer, res.bound_box.xh, RoundMode.NEAREST)
            cap_out_ym = self.connect_to_tracks([res.get_pin('top'), dec.get_pin('vcm')],
                                                TrackID(ym_layer, ym_tidx, tr_w_ym_bias))
            cap_diff_plus_xm += [cap_cm.get_pin('plus', layer=res.get_pin('top').layer_id)]
            out = self.connect_to_tracks(cap_diff_plus_xm + [dec.get_pin('out')], TrackID(ym_layer, ym_tidx, tr_w_ym_bias))
        else:
            cap_diff_plus_xm = [cap_diff.get_all_port_pins(f'PLUS{xm_layer}')[0]]
            cap_out = cap_diff_plus_xm
            ym_tidx = self.grid.coord_to_track(ym_layer, res.bound_box.xh, RoundMode.NEAREST)
            if float_output:
                mux_out_ym_tidx = tr_manager.get_next_track(ym_layer, ym_tidx, 'bias', 'dum')
                self.connect_to_tracks([dec.get_pin('mux_out')]+cap_diff_plus_xm, TrackID(ym_layer, mux_out_ym_tidx))
                out = self.connect_to_tracks([dec.get_pin('out')], TrackID(ym_layer, ym_tidx, tr_w_ym_bias))

            else:
                out = self.connect_to_tracks(cap_diff_plus_xm + [dec.get_pin('out')],
                                             TrackID(ym_layer, ym_tidx, tr_w_ym_bias))

        # Connect input
        blk_w, blk_h = self.grid.get_block_size(ym1_layer, half_blk_x=False, half_blk_y=False)
        w_tot = -(-dec.bound_box.xh // blk_w) * blk_w
        h_tot = -(-max(cap_diff.bound_box.yh, res.bound_box.yh) // blk_h) * blk_h
        self.set_size_from_bound_box(ym1_layer, BBox(0, 0, w_tot, h_tot))
        _, ym_locs = tr_manager.place_wires(ym_layer, ['sig'] * dec_master.core.num_bits,
                                            align_track=self.grid.coord_to_track(ym_layer, dec.bound_box.xl,
                                                                                 RoundMode.NEAREST))
        for idx, loc in enumerate(ym_locs):
            self.add_pin(f'bit<{idx}>', self.connect_to_tracks(dec.get_pin(f'bit<{idx}>'),
                                                               TrackID(ym_layer, loc, tr_w_ym),
                                                               track_lower=dec.bound_box.yl))
        self._tr_manager = tr_manager

        nbits = len(ym_locs)
        self._num_bits = nbits
        xm_vss_top = max(dec.get_all_port_pins('VSS'), key=lambda x: x.track_id.base_index)
        self.connect_to_track_wires(cap_diff.get_all_port_pins(f'MINUS{ym_layer}')[-2], xm_vss_top)

        res_vdd_xm_list, res_vss_xm_list = [], []
        if res.has_port('VDD'):
            res_vdd_xm_list.extend(res.get_all_port_pins('VDD'))
        if res.has_port('VSS'):
            res_vss_xm_list.extend(res.get_all_port_pins('VSS'))

        if differential:
            if self.params['pside']:
                res_vdd_xm_list.append(res.get_pin('bottom'))
            else:
                res_vss_xm_list.append(res.get_pin('bottom'))
        else:
            if res_bias_master:
                if res_bias.has_port('VDD'):
                    res_vdd_xm_list.extend(res_bias.get_all_port_pins('VDD'))
                if res_bias.has_port('VSS'):
                    res_vss_xm_list.extend(res_bias.get_all_port_pins('VSS'))
                if res_bias_params['ref'] == 'VSS':
                    res_vss_xm_list.append(res_bias.get_pin('bottom'))
                    res_vdd_xm_list.append(res.get_pin('bottom'))
                else:
                    res_vss_xm_list.append(res.get_pin('bottom'))
                    res_vdd_xm_list.append(res_bias.get_pin('bottom'))
                self.connect_wires([res_bias.get_pin('top'), res.get_pin('top')])
            else:
                res_vss_xm_list.append(res.get_pin('bottom'))
                res_vdd_xm_list.append(res.get_pin('top'))

        diff_decap_top_layer = gnd_cap_params['top_layer']
        diff_decap_top_layer_dir = self.grid.get_direction(diff_decap_top_layer)
        if diff_decap_top_layer_dir == Orient2D.y:
            raise Warning("differential decap needs horizontal top layer")
        power_bbox = dec.bound_box.extend(x=0)
        # diff_cap_top_vss = mos_power_dict[1][diff_decap_top_layer]
        # diff_cap_top_vdd = mos_power_dict[0][diff_decap_top_layer] + \
        #                    cap_diff.get_all_port_pins(f'VSS{diff_decap_top_layer}')

        # block_power_dict = self.connect_supply_stack_warr(tr_manager, [diff_cap_top_vdd, diff_cap_top_vss],
        #                                                   diff_decap_top_layer, top_layer, self.bound_box)
        # self.reexport(dec.get_port('VSS'), connect=True)
        # self.reexport(dec.get_port('VDD'), connect=True)
        # ym power
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(ym_layer, 'sup')
        wl, wh = self.grid.get_wire_bounds(ym_layer, 0, width=tr_w_sup_ym)
        w_ym = int((wh - wl) / 2)
        via_ext_xm, via_ext_ym = self.grid.get_via_extensions(Direction.LOWER, xm_layer, tr_w_sup_xm, tr_w_sup_ym)
        dx = via_ext_xm + w_ym
        mid_space = self.grid.track_to_coord(vm_layer, tr_manager.get_width(ym_layer, 'sup'))
        bb = BBox(xl=res.bound_box.xl + mid_space, xh=res.bound_box.xh - dx,
                  yl=res.bound_box.yl + via_ext_ym, yh=res.bound_box.yh - via_ext_ym)
        if res_bias_master:
            bb.extend(res_bias.bound_box.xl)
        vdd_ym_res, vss_ym_res = self.do_power_fill(ym_layer, tr_manager, res_vdd_xm_list,
                                                    res_vss_xm_list, bound_box=bb)

        bb = BBox(xl=dec.bound_box.xl + mid_space, xh=dec.bound_box.xh - dx,
                  yl=dec.bound_box.yl + via_ext_ym, yh=dec.bound_box.yh - via_ext_ym)
        vdd_ym_dec, vss_ym_dec = self.do_power_fill(ym_layer, tr_manager, dec.get_all_port_pins('VDD'),
                                                    dec.get_all_port_pins('VSS'), bound_box=bb)
        vdd_ym = vdd_ym_dec + vdd_ym_res
        vss_ym = vss_ym_dec + vss_ym_res
        power_dict = self.connect_supply_stack_warr(tr_manager, [vdd_ym, vss_ym],
                                                    ym_layer, diff_decap_top_layer, power_bbox,
                                                    extend_lower_layer=False, align_upper=True)
        power_bbox = BBox(0, dec.bound_box.yl, self.bound_box.xh, self.bound_box.yh)
        power_vss = power_dict[1][diff_decap_top_layer] + cap_diff.get_all_port_pins(f'VSS{diff_decap_top_layer}')
        power_dict = self.connect_supply_stack_warr(tr_manager, [power_dict[0][diff_decap_top_layer], power_vss],
                                                    diff_decap_top_layer, self.params['top_layer'], power_bbox,
                                                    align_upper=True)

        if differential:
            self.add_pin('out_c', cap_cm.get_pin('minus', layer=res.get_pin('top').layer_id))
            self.reexport(dec.get_port('sel_np'), net_name=f'bit<{nbits + 1}>')
            self.reexport(dec.get_port('sel_cm'), net_name=f'bit<{nbits}>')
            self.reexport(dec.get_port('mux_out'))
            self.reexport(dec.get_port('mux_out_n'), net_name='mux_out_c')
            self.reexport(res.get_port('top'))

        for vss in power_dict[1][diff_decap_top_layer+1]:
            self.connect_to_track_wires(vss, cap_diff.get_all_port_pins(f'MINUS{diff_decap_top_layer}'))
        # self.reexport(dec.get_port('en'))
        self.add_pin('out', self.extend_wires(out, upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('en', self.extend_wires(dec.get_pin('en'), lower=dec.bound_box.yl), mode=PinMode.LOWER)
        for idx in range(diff_decap_top_layer + 1, self.params['top_layer']):
            self.add_pin(f'VDD{idx}', power_dict[0][idx], hide=True)
            self.add_pin(f'VSS{idx}', power_dict[1][idx], hide=True)
        self.add_pin('VDD', power_dict[0][self.params['top_layer']])
        self.add_pin('VSS', power_dict[1][self.params['top_layer']])

        self._sch_params = dict(
            dec=dec_master.sch_params,
            res=res_master.sch_params,
            res_bias=res_bias_master.sch_params if res_bias_master else None,
            cap_cm=cap_cm_master.sch_params if cap_params else None,
            cap_dm=cap_diff_master.sch_params,
            res_start_idx=res_start_idx,
            differential=differential,
            pside=self.params['pside'],
            decap=True,
            res_bias_ref=res_bias_params['ref'] if res_bias_params else None,
        )


class RDACDiff(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return RDAC.get_params_info()

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_diff')

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return RDAC.get_default_param_values()
        # return dict(start_row=1)

    def draw_layout(self) -> None:
        dac_n_params = self.params.copy(append=dict(show_pins=False))
        dac_p_params = self.params.copy(append=dict(pside=True, show_pins=False))
        dac_n_master = self.new_template(RDAC, params=dac_n_params)
        dac_p_master = self.new_template(RDAC, params=dac_p_params)

        conn_layer = \
            MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                             self.params['dec_params']['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        top_layer = dac_n_master.top_layer
        blk_w, blk_h = self.grid.get_block_size(top_layer, half_blk_x=False, half_blk_y=False)
        dac_w, dac_h = dac_n_master.bound_box.w, dac_n_master.bound_box.h
        tr_manager = dac_n_master.tr_manager

        nbits = dac_p_master.num_bits
        dacn = self.add_instance(dac_n_master, xform=Transform(dac_w, 0, Orientation.MY))
        dacp = self.add_instance(dac_p_master, xform=Transform(dac_w, 0, Orientation.R0))
        self.connect_wires([dacn.get_pin('out_c'), dacp.get_pin('out_c')])
        self.connect_wires([dacn.get_pin('top'), dacp.get_pin('top')])
        self.connect_wires([dacn.get_pin('mux_out'), dacp.get_pin('mux_out_c')])
        self.connect_wires([dacp.get_pin('mux_out'), dacn.get_pin('mux_out_c')])
        sel_cm = self.connect_wires([dacp.get_pin(f'bit<{nbits}>'), dacn.get_pin(f'bit<{nbits}>')])
        sel_np = self.connect_wires([dacp.get_pin(f'bit<{nbits + 1}>'), dacn.get_pin(f'bit<{nbits + 1}>')])
        en = self.connect_wires([dacp.get_pin('en'), dacn.get_pin('en')])
        _, ym_locs = tr_manager.place_wires(ym_layer, ['sig'] * 3, center_coord=dacn.bound_box.xh)
        tr_w_ym = tr_manager.get_width(ym_layer, 'sig')

        w_tot = -(-dacp.bound_box.xh // blk_w) * blk_w
        h_tot = -(-dacp.bound_box.yh // blk_h) * blk_h
        self.set_size_from_bound_box(ym1_layer, BBox(0, 0, w_tot, h_tot))
        sel_cm_ym, sel_np_ym, en_ym = self.connect_matching_tracks([sel_cm, sel_np, en], ym_layer, ym_locs,
                                                                   width=tr_w_ym, track_lower=self.bound_box.yl)
        self.add_pin('bit_cm', sel_cm_ym, mode=PinMode.LOWER)
        self.add_pin('bit_np', sel_np_ym, mode=PinMode.LOWER)
        self.add_pin('en', en_ym, mode=PinMode.LOWER)

        self.add_pin('out_n', self.extend_wires(dacn.get_pin('out'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('out_p', self.extend_wires(dacp.get_pin('out'), upper=self.bound_box.yh), mode=PinMode.UPPER)

        for idx in range(nbits):
            self.add_pin(f'bit_n<{idx}>', dacn.get_pin(f'bit<{idx}>'))
            self.add_pin(f'bit_p<{idx}>', dacp.get_pin(f'bit<{idx}>'))

        self._sch_params = dict(nside=dac_n_master.sch_params, pside=dac_p_master.sch_params)
        self.add_pin('VDD', dacn.get_all_port_pins('VDD') + dacp.get_all_port_pins('VDD'))
        self.add_pin('VSS', dacn.get_all_port_pins('VSS') + dacp.get_all_port_pins('VSS'))
