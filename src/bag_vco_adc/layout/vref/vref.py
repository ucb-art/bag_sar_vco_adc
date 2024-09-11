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

from typing import Any, Dict, Type, Optional, Mapping, Union, Tuple

from bag.design.database import ModuleDB, Module
from bag.io import read_yaml
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB
from bag.util.immutable import Param
from bag.util.math import HalfInt
from pybag.core import Transform, BBox
from pybag.enum import Orient2D, RoundMode, PinMode
from .amp import AmpWrapper
from ..others.decap import DecapArray
from ..rdac.rdac import RDAC
from ..util.template import TrackIDZL as TrackID, TemplateBaseZL
from ..util.util import basefill_bbox
from ..util.wrapper import GenericWrapper


class SFWrapper(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBaseZL.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None
        self._core_master = None

    @property
    def tr_manager(self):
        return self._tr_manager

    @property
    def core_master(self):
        return self._core_master

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return self._core_master.get_schematic_class()

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cls_name='Core class name',
            params='Core master params',
            top_layer='Top supply layer',
            top_out_layer='Top output layer',
            num_sup_around='Tuple of x,y direction',
            bias_decap_n='params for bias decoupling',
            bias_decap_p='params for bias decoupling'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(num_sup_around=(2, 2))

    def draw_layout(self) -> None:
        cls_name = self.params['cls_name']
        core_params = self.params['params']
        top_layer = self.params['top_layer']
        top_out_layer = self.params['top_out_layer']
        num_sup_around = self.params['num_sup_around']
        bias_decap_n_params = self.params['bias_decap_n']
        if isinstance(bias_decap_n_params, str):
            bias_decap_n_params = read_yaml(bias_decap_n_params)
            bias_decap_n_params = bias_decap_n_params['params']
        bias_decap_p_params = self.params['bias_decap_p']
        if isinstance(bias_decap_p_params, str):
            bias_decap_p_params = read_yaml(bias_decap_p_params)
            bias_decap_p_params = bias_decap_p_params['params']

        # gen_cls = cast(Type[MOSBase], import_class(cls_name))
        core_master: GenericWrapper = self.new_template(GenericWrapper, params=dict(export_private=False,
                                                                                    export_hidden=True,
                                                                                    cls_name=cls_name,
                                                                                    params=core_params))
        bias_decap_n_params['unit_params']['seg'] = core_master.core.num_cols
        bias_decap_n_master = self.new_template(DecapArray, params=bias_decap_n_params)
        bias_decap_p_params['unit_params']['seg'] = core_master.core.num_cols
        bias_decap_p_master = self.new_template(DecapArray, params=bias_decap_p_params)
        self._core_master = core_master.core

        tr_manager = core_master.core.tr_manager
        top_sup_dir = self.grid.get_direction(top_layer)
        top_layer_x = top_layer if top_sup_dir == Orient2D.x else top_layer - 1
        top_layer_y = top_layer - 1 if top_sup_dir == Orient2D.x else top_layer

        top_x_w = TrackID(top_layer_x, 0, tr_manager.get_width(top_layer_x, 'sup'), grid=self.grid).width
        top_x_sep = self.get_track_sep(top_layer_x, top_x_w, top_x_w)
        top_y_w = TrackID(top_layer_y, 0, tr_manager.get_width(top_layer_y, 'sup'), grid=self.grid).width
        top_y_sep = self.get_track_sep(top_layer_x, top_y_w, top_y_w)
        num_sup_x, num_sup_y = num_sup_around[0], num_sup_around[1]
        top_y_spec, top_x_spec = self.grid.get_track_info(top_layer_x), self.grid.get_track_info(top_layer_y)
        core_y, core_x = top_x_spec.offset + num_sup_y * top_x_sep * top_x_spec.pitch, \
                         top_y_spec.offset + num_sup_x * top_y_sep * top_y_spec.pitch

        bbox_top_layer = max(top_layer, top_out_layer)
        blk_w, blk_h = self.grid.get_block_size(bbox_top_layer, half_blk_x=False, half_blk_y=False)
        blk_w_cap, blk_h_cap = self.grid.get_block_size(bias_decap_n_master.core().top_layer,
                                                        half_blk_x=False, half_blk_y=False)

        biasn_decap = self.add_instance(bias_decap_n_master, xform=Transform(-(-core_x // blk_w) * blk_w, 0))
        core_y = max(core_y, biasn_decap.bound_box.yh)
        core = self.add_instance(core_master, xform=Transform(-(-core_x // blk_w) * blk_w, -(-core_y // blk_h) * blk_h))
        biasp_decap_y = core.bound_box.yh
        biasp_decap = self.add_instance(bias_decap_p_master, xform=Transform(-(-core_x // blk_w) * blk_w,
                                                                             -(
                                                                                     -biasp_decap_y // blk_h_cap) * blk_h_cap))

        h_tot = (core.bound_box.yh + num_sup_y * (top_x_sep + 1) * top_x_spec.pitch)
        h_tot = max(h_tot, biasp_decap.bound_box.yh)
        h_tot = -(-h_tot // blk_h) * blk_h
        w_tot = -(-(core.bound_box.xh + num_sup_x * (top_y_sep + 1) * top_y_spec.pitch) // blk_w) * blk_w
        self.set_size_from_bound_box(bbox_top_layer, BBox(0, 0, w_tot, h_tot))

        conn_layer = core_master.core.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        in_vm = core.get_pin('vin')
        tr_w_ana_sig_xm = tr_manager.get_width(xm_layer, 'ana_sig')
        in_xm_tidx = self.grid.coord_to_track(xm_layer, in_vm.middle, RoundMode.NEAREST)
        in_xm = self.connect_to_tracks(in_vm, TrackID(xm_layer, in_xm_tidx, tr_w_ana_sig_xm, grid=self.grid))
        out_vm = core.get_pin('vout')
        out_dict = self.via_stack_up(tr_manager, out_vm, vm_layer, top_out_layer, 'ana_sig', bbox=out_vm.bound_box,
                                     alignment=RoundMode.LESS_EQ)

        vdd_xm = self.extend_wires(core.get_pin('VDD'), lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm = self.extend_wires(core.get_pin('VSS'), lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_xm.extend(biasp_decap.get_all_port_pins(f'PLUS{xm_layer}'))
        vss_xm.extend(biasn_decap.get_all_port_pins(f'MINUS{xm_layer}'))

        bnd_w = core_master.core_bound_box.xl
        decap_conn_bnd = ((core.bound_box.xl, core.bound_box.xl + bnd_w),
                          (core.bound_box.xh - bnd_w, core.bound_box.xh))

        biasp_vm_decap, biasn_vm_decap = [], []
        for vm in biasp_decap.get_all_port_pins(f'MINUS{vm_layer}'):
            vm_coord = self.grid.track_to_coord(vm_layer, vm.track_id.base_index)
            if decap_conn_bnd[0][0] < vm_coord < decap_conn_bnd[0][1] or decap_conn_bnd[1][0] < vm_coord < \
                    decap_conn_bnd[1][1]:
                biasp_vm_decap.append(vm)
        for vm in biasn_decap.get_all_port_pins(f'PLUS{vm_layer}'):
            vm_coord = self.grid.track_to_coord(vm_layer, vm.track_id.base_index)
            if decap_conn_bnd[0][0] < vm_coord < decap_conn_bnd[0][1] or decap_conn_bnd[1][0] < vm_coord < \
                    decap_conn_bnd[1][1]:
                biasn_vm_decap.append(vm)

        self.connect_to_track_wires(core.get_pin('vbp'), biasp_vm_decap)
        self.connect_to_track_wires(core.get_pin('vbn'), biasn_vm_decap)

        power_dict = self.connect_supply_stack_warr(tr_manager, [vdd_xm, vss_xm], xm_layer, top_layer, self.bound_box,
                                                    extend_lower_layer=True)
        self.add_pin('VDD', power_dict[0][top_layer])
        self.add_pin('VSS', power_dict[1][top_layer])
        self.add_pin('vout', out_dict[top_out_layer])
        for pin in core.port_names_iter():
            if 'ctrl' in pin or 'vb' in pin:
                self.reexport(core.get_port(pin))

        self.add_pin('vin', in_xm)
        if core.has_port('vout_fb'):
            self.reexport(core.get_port('vout_fb'))

        sch_params = core_master.sch_params.copy()
        sch_params = sch_params.to_dict()
        sch_params['decap_p_params'] = bias_decap_p_master.sch_params
        sch_params['decap_n_params'] = bias_decap_n_master.sch_params
        self._sch_params = sch_params


class VREF(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBaseZL.__init__(self, temp_db, params, **kwargs)
        self._tr_manager = None

    @property
    def tr_manager(self):
        return self._tr_manager

    def get_schematic_class(self) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'vref')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            buf_params='SF parameters',
            amp_params='Amplifier parameters',
            dac_vref_params='Comparator DAC parameters',
            dac_vb_params='Comparator DAC parameters',
            tr_widths='track width dictionary',
            tr_spaces='track space dictionary',
            type='vrefp or vrefn',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(sampler_params=[])

    def draw_layout(self) -> None:
        buf_params: Param = self.params['buf_params']
        amp_params: Param = self.params['amp_params']
        dac_vb_params: Param = self.params['dac_vb_params']
        dac_vref_params: Param = self.params['dac_vref_params']

        if isinstance(dac_vb_params, str):
            dac_yaml = read_yaml(dac_vb_params)
            dac_vb_params = dac_yaml['params']
        if isinstance(dac_vref_params, str):
            dac_yaml = read_yaml(dac_vref_params)
            dac_vref_params = dac_yaml['params']
        if isinstance(buf_params, str):
            buf_yaml = read_yaml(buf_params)
            buf_params = buf_yaml['params']
        if isinstance(amp_params, str):
            amp_yaml = read_yaml(amp_params)
            amp_params = amp_yaml['params']

        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        self._tr_manager = tr_manager

        buf_master: SFWrapper = self.new_template(SFWrapper, params=buf_params)
        amp_master: AmpWrapper = self.new_template(AmpWrapper, params=amp_params)
        dac_vb_master: TemplateBaseZL = self.new_template(RDAC, params=dac_vb_params)
        dac_vref_master: TemplateBaseZL = self.new_template(RDAC, params=dac_vref_params)

        conn_layer = buf_master.core_master.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        top_layer = max(buf_master.top_layer, dac_vref_master.top_layer, amp_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer, half_blk_y=True, half_blk_x=True)

        dac_top_coord = max(dac_vb_master.bound_box.h, dac_vref_master.bound_box.h)
        dac_top_coord = -(-dac_top_coord // h_blk) * h_blk

        dac_vb_y = (dac_top_coord - dac_vb_master.bound_box.h) // h_blk * h_blk
        dac_vref_y = (dac_top_coord - dac_vref_master.bound_box.h) // h_blk * h_blk
        dac_vb = self.add_instance(dac_vb_master, xform=Transform(0, dac_vb_y))
        dac_vref_x = -(-dac_vb.bound_box.xh // w_blk) * w_blk
        dac_vref = self.add_instance(dac_vref_master, xform=Transform(dac_vref_x, dac_vref_y))
        sup_y_le_sp = max([self.grid.get_line_end_space(idx, TrackID(idx, 0, tr_manager.get_width(idx, 'sup'),
                                                                     grid=self.grid).width)
                           for idx in range(hm_layer, top_layer)])
        buf_y = -(-(dac_vref.bound_box.yh + sup_y_le_sp) // h_blk) * h_blk

        buf_x = (dac_vref.bound_box.xh - buf_master.bound_box.w) // w_blk * w_blk
        buf = self.add_instance(buf_master, xform=Transform(buf_x, buf_y))
        amp_x = (buf.bound_box.xl - amp_master.bound_box.w) // w_blk * w_blk - w_blk
        amp = self.add_instance(amp_master, xform=Transform(amp_x, buf_y))

        buf_ctrl_start_idx = self.grid.coord_to_track(ym_layer, dac_vref.bound_box.xh, RoundMode.GREATER_EQ)
        num_buf_ctrl = sum([('ctrl' in name) for name in buf.port_names_iter()])
        _, buf_ctrl_idx_list = tr_manager.place_wires(ym1_layer, ['dum'] + ['ctrl'] * num_buf_ctrl,
                                                      align_track=buf_ctrl_start_idx)

        h_tot = -(-max(buf.bound_box.yh, amp.bound_box.yh) // h_blk) * h_blk
        w_tot = -(-self.grid.track_to_coord(ym_layer, buf_ctrl_idx_list[-1]) // w_blk) * w_blk
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot))

        buf_sup_top_layer = buf_params['top_layer']
        dac_sup_to_buf_vdd = dac_vref.get_all_port_pins(f'VDD{buf_sup_top_layer + 1}')
        dac_sup_to_buf_vss = dac_vref.get_all_port_pins(f'VSS{buf_sup_top_layer + 1}')

        buf_sup_bbox = BBox(0, buf.bound_box.yl, self.bound_box.xh, self.bound_box.yh)
        dac_sup_to_buf_vdd = self.get_available_wires(dac_sup_to_buf_vdd, buf_sup_bbox)
        dac_sup_to_buf_vss = self.get_available_wires(dac_sup_to_buf_vss, buf_sup_bbox)
        dac_vref_sup_vdd = self.connect_to_track_wires(buf.get_all_port_pins('VDD'), dac_sup_to_buf_vdd)
        dac_vref_sup_vss = self.connect_to_track_wires(buf.get_all_port_pins('VSS'), dac_sup_to_buf_vss)
        dac_vb_sup_vdd = self.connect_to_track_wires(buf.get_all_port_pins('VDD'),
                                                     dac_vb.get_all_port_pins(f'VDD{buf_sup_top_layer + 1}'))
        dac_vb_sup_vss = self.connect_to_track_wires(buf.get_all_port_pins('VSS'),
                                                     dac_vb.get_all_port_pins(f'VSS{buf_sup_top_layer + 1}'))
        self.extend_wires(dac_vb_sup_vdd, lower=dac_vb.bound_box.yl)
        self.extend_wires(dac_vb_sup_vss, lower=dac_vb.bound_box.yl)
        dac_sup_vss = self.extend_wires(dac_vref_sup_vdd + dac_vb_sup_vdd, upper=self.bound_box.yh)
        dac_sup_vdd = self.extend_wires(dac_vref_sup_vss + dac_vb_sup_vss, upper=self.bound_box.yh)
        self.connect_to_track_wires(buf.get_all_port_pins('VDD'), amp.get_all_port_pins('VDD'))
        self.connect_to_track_wires(buf.get_all_port_pins('VSS'), amp.get_all_port_pins('VSS'))
        dac_power_dict = self.connect_supply_stack_warr(tr_manager, [dac_sup_vdd, dac_sup_vss], buf_sup_top_layer + 1,
                                                        self.top_layer-1, bbox_list=buf_sup_bbox)
        tr_w_ym_sig = tr_manager.get_width(ym_layer, 'sig')
        tr_w_xm_sig = tr_manager.get_width(xm_layer, 'sig')
        vdac_ym_tidx = self.grid.coord_to_track(ym_layer, amp.bound_box.xh, RoundMode.NEAREST)
        vdac_ym = self.connect_to_tracks(amp.get_pin('inp'), TrackID(ym_layer, vdac_ym_tidx, tr_w_ym_sig))
        vdac_xm_tidx = self.grid.coord_to_track(xm_layer, dac_vref.bound_box.yh, RoundMode.NEAREST)
        vdac_xm = self.connect_to_tracks(dac_vref.get_pin('out'), TrackID(xm_layer, vdac_xm_tidx, tr_w_xm_sig))
        self.connect_to_track_wires(vdac_ym, vdac_xm)

        vb_ym_tidx = tr_manager.get_next_track(ym_layer, vdac_ym_tidx, 'sig', 'sig')
        vamp_ym_tidx = tr_manager.get_next_track(ym_layer, vb_ym_tidx, 'sig', 'sig')
        vout_fb_ym_tidx = tr_manager.get_next_track(ym_layer, vamp_ym_tidx, 'sig', 'sig')
        vb_xm_tidx = tr_manager.get_next_track(xm_layer, vdac_xm_tidx, 'sig', 'sig')
        vb_ym = self.connect_to_tracks([amp.get_pin('vbn'), buf.get_pin('vb')],
                                       TrackID(ym_layer, vb_ym_tidx, tr_w_ym_sig))
        # self.connect_to_track_wires(amp.get_pin('inn'), buf.get_pin('vout_fb'))
        vamp_ym = self.connect_to_tracks([amp.get_pin('out'), buf.get_pin('vin')],
                                         TrackID(ym_layer, vamp_ym_tidx, tr_w_ym_sig))
        vfb_ym = self.connect_to_tracks([amp.get_pin('inn'), buf.get_pin('vout_fb')],
                                        TrackID(ym_layer, vout_fb_ym_tidx, tr_w_ym_sig))
        vb_xm = self.connect_to_tracks([dac_vb.get_pin('out'), vb_ym], TrackID(xm_layer, vb_xm_tidx, tr_w_xm_sig))
        self.add_pin('vb', vb_ym)
        self.add_pin('vdac', vdac_ym)
        self.add_pin('vout_fb', vfb_ym)

        vdd_dac_top = dac_vref.get_all_port_pins('VDD')
        vss_dac_top = dac_vref.get_all_port_pins('VSS')
        vdd_dac_top = self.extend_wires(vdd_dac_top, lower=0, upper=self.bound_box.xh)
        vss_dac_top = self.extend_wires(vss_dac_top, lower=0, upper=self.bound_box.xh)
        #
        tr_w_ctrl_ym = tr_manager.get_width(ym_layer, 'ctrl')
        port_idx = 0
        for name in buf.port_names_iter():
            if 'ctrl' in name:
                port_idx += 1
                pin = buf.get_pin(name)
                pin_ym = self.connect_to_tracks(pin, TrackID(ym_layer, buf_ctrl_idx_list[port_idx], tr_w_ctrl_ym),
                                                track_lower=dac_vb.bound_box.yl)
                self.add_pin(name, pin_ym, mode=PinMode.LOWER)

        for name in dac_vref.port_names_iter():
            if 'bit' in name:
                self.add_pin(name.replace('bit', 'ctrl_dac'), dac_vref.get_pin(name), mode=PinMode.LOWER)

            if 'en' in name:
                self.add_pin(name.replace('en', 'ctrl_dac_en'), dac_vref.get_pin(name), mode=PinMode.LOWER)
        for name in dac_vb.port_names_iter():
            if 'bit' in name:
                self.add_pin(name.replace('bit', 'ctrl_vb_dac'),
                             self.extend_wires(dac_vb.get_pin(name), lower=dac_vb.bound_box.yl), mode=PinMode.LOWER)

            if 'en' in name:
                self.add_pin(name.replace('en', 'ctrl_vb_dac_en'),
                             self.extend_wires(dac_vb.get_pin(name), lower=dac_vb.bound_box.yl), mode=PinMode.LOWER)

        self.reexport(buf.get_port('vb'))
        self.reexport(buf.get_port('vout'), net_name='vref')
        vss = dac_power_dict[0][self.top_layer-1] + vss_dac_top
        vdd = dac_power_dict[1][self.top_layer-1] + vdd_dac_top
        power_bbox = BBox(0, max(dac_vb.bound_box.yl, dac_vref.bound_box.yl), self.bound_box.xh,
                          self.bound_box.yh)
        dac_power_dict = self.connect_supply_stack_warr(tr_manager, [vdd, vss], vdd[0].layer_id,
                                                        vdd[0].layer_id + 1, bbox_list=power_bbox)
        self.add_pin('VDD', dac_power_dict[0][self.top_layer])
        self.add_pin('VSS', dac_power_dict[1][self.top_layer])
        # basefill_bbox(self, BBox(0, 0, dac_vref.bound_box.xl, dac_vb.bound_box.yl))

        self._sch_params = dict(
            type=self.params['type'],
            dac_vref_params=dac_vref_master.sch_params,
            dac_vb_params=dac_vb_master.sch_params,
            buf_params=buf_master.sch_params,
            amp_params=amp_master.sch_params,
        )
