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
from typing import Any, Dict, Type, Optional, List, Mapping, Union, Tuple

from bag.design.database import ModuleDB, Module
from bag.env import get_tech_global_info
from bag.layout.routing.base import TrackManager
from bag.layout.routing.base import WireArray
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param, ImmutableSortedDict
from bag.util.math import HalfInt
from pybag.core import Transform, BBox
from pybag.enum import Orient2D, RoundMode, PinMode, MinLenMode, Orientation
from xbase.layout.enum import MOSWireType, MOSType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase, MOSArrayPlaceInfo
from xbase.layout.mos.placement.data import TilePatternElement, TilePattern
from ..sah.sar_samp import SamplerVertGR
from ..util.template import TrackIDZL as TrackID, TemplateBaseZL
from ..util.util import fill_conn_layer_intv, connect_hm_sup_ow, MOSBaseFiller
from ..util.wrapper import GenericWrapper


class CapTap(MOSBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg='segments dictionary.',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            seg=2
        )

    def draw_layout(self):
        pinfo = self.params['pinfo']
        seg = self.params['seg']
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo)
        self.draw_base(pinfo)

        tap = self.add_substrate_contact(0, 0, seg=seg, tile_idx=0)
        self.set_mos_size()
        self.add_pin('VSS', tap)


class CapUnitCore(TemplateBase):
    """MOMCap core
    Draw a layout has only metal and metal resistor in this shape:
    ----------------|
    --------------  |
    ----------------|
    Horizontal layer is "vertical_layer"
    Top and bottom is connected by "bot_layer"

    Parameters:
        top_w: width of middle horizontal layer
        bot_w: width of top/bot horizontal layer
        bot_y_w: width of vertical layer
        sp: space between top/bot and middle
        sp_le: line-end space between middle horizontal layer
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cap_config='MOM cap configuration.',
            width='MOM cap width, in resolution units.',
            tr_w='Track width',
            tr_sp='Track space',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            cap_config={},
            width=0,
            tr_w={},
            tr_sp={},
        )

    def draw_layout(self) -> None:
        cap_config: Dict[str, int] = self.params['cap_config']
        tr_w: Dict = self.params['tr_w']
        tr_sp: Dict = self.params['tr_sp']
        width: int = self.params['width']
        width = int(width / self.grid.resolution)

        tr_manager = TrackManager(self.grid, tr_w, tr_sp)

        grid = self.grid

        # TODO: better way to get connlayer?
        conn_layer = 1
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        # ym_layer = xm_layer + 1

        # Read cap_info
        top_layer = cap_config['top_layer']
        bot_layer = cap_config['bot_layer']
        top_w = cap_config['top_w']
        bot_w = cap_config['bot_w']
        bot_y_w = cap_config['bot_y_w']
        sp = cap_config['sp']
        sp_le = cap_config['sp_le']

        w_blk, h_blk = grid.get_block_size(max(top_layer, bot_layer), half_blk_x=True, half_blk_y=True)

        # draw cap
        if grid.get_direction(top_layer) == Orient2D.y:
            raise ValueError("Top layer need to be PGD")

        # Get tidx of top/mid/bot horizontal layer
        tidx_l = grid.find_next_track(top_layer, 0, tr_width=top_w, half_track=True)
        tidx_sp = grid.get_sep_tracks(top_layer, ntr1=top_w, ntr2=bot_w)
        tidx_sp = max(tidx_sp, HalfInt(sp))
        tidx_m = tidx_l + tidx_sp
        tidx_h = tidx_m + tidx_sp

        height = grid.track_to_coord(top_layer, tidx_h) + grid.get_track_offset(top_layer)
        w_tot = -(-width // w_blk) * w_blk
        h_tot = -(-height // h_blk) * h_blk

        # Connect lower layer
        sep_tidx = self.grid.get_sep_tracks(bot_layer, bot_w, bot_w)
        shift_coord = self.grid.track_to_coord(bot_layer, sep_tidx) - self.grid.track_to_coord(bot_layer, 0)
        bot_layer_w = self.grid.get_wire_total_width(bot_layer, bot_y_w)
        btidx = self.grid.coord_to_track(bot_layer, width + shift_coord - bot_layer_w // 2, mode=RoundMode.NEAREST,
                                         even=True)
        bot = self.add_wires(bot_layer, btidx, 0, height, width=bot_y_w)
        bot = [bot]
        top_bot_via_bbox = self.add_via_on_grid(bot[-1].track_id, TrackID(top_layer, tidx_l, top_w), extend=True)
        self.add_via_on_grid(bot[-1].track_id, TrackID(top_layer, tidx_h, top_w), extend=True)
        num_bot_y = cap_config.get('num_bot_y', 0)
        top_bot_via_bbox_extend = top_bot_via_bbox
        if num_bot_y:
            _btidx = btidx
            for idx in range(num_bot_y - 1):
                _btidx = _btidx + self.grid.get_sep_tracks(bot_layer, bot_y_w, bot_y_w)
                bot.append(self.add_wires(bot_layer, _btidx, 0, height, width=bot_y_w))
                top_bot_via_bbox_extend = \
                    self.add_via_on_grid(bot[-1].track_id, TrackID(top_layer, tidx_l, top_w), extend=True)
                self.add_via_on_grid(bot[-1].track_id, TrackID(top_layer, tidx_h, top_w), extend=True)

        # Add wires
        top_l = self.add_wires(top_layer, tidx_l, 0, top_bot_via_bbox_extend[0 if bot_layer > top_layer else 1][1],
                               width=top_w)
        top_h = self.add_wires(top_layer, tidx_h, 0, top_bot_via_bbox_extend[0 if bot_layer > top_layer else 1][1],
                               width=top_w)

        bot_mid_coord = grid.track_to_coord(bot_layer, bot[-1].track_id.base_index)

        top_min_le_sp = grid.get_line_end_space(top_layer, bot_w, even=True)

        top_m_len = top_bot_via_bbox[0 if bot_layer > top_layer else 1][0] - top_min_le_sp
        # top_m_len = grid.get_wire_bounds(bot_layer, btidx, bot_y_w)[0]
        top_m_len_unit = cap_config.get('unit', 1)
        top_m_len = int(top_m_len_unit * (top_m_len - sp_le))
        top_m = self.add_wires(top_layer, tidx_m, 0, top_m_len, width=bot_w)

        pin_len = grid.get_next_length(top_layer, top_m.track_id.width,
                                       grid.get_wire_total_width(top_layer, top_m.track_id.width), even=True)
        top_m_dum = self.add_wires(top_layer, tidx_m, top_m_len + top_min_le_sp,
                                   max(grid.get_wire_bounds(bot_layer, btidx, bot_y_w)[1],
                                       top_bot_via_bbox_extend[0 if bot_layer > top_layer else 1][1],
                                       top_m_len + top_min_le_sp + pin_len),
                                   width=bot_w)
        if cap_config.get('extra_hor_couple', False):
            extra_hor_lay = bot_layer + 1 if top_layer == bot_layer - 1 else bot_layer - 1
            extra_hor_wlist = []
            for w in [top_l, top_h, top_m, top_m_dum]:
                extra_hor_wlist.append(self.add_wires(extra_hor_lay, lower=w.lower, upper=w.upper,
                                                      track_idx=w.track_id.base_index,
                                                      width=w.track_id.width))
            self.connect_to_track_wires(extra_hor_wlist[:2], bot)
            self.add_pin('minus_extra_hor', extra_hor_wlist[2], hide=True)
        else:
            extra_hor_wlist = []

        if cap_config['bot_layer_couple']:
            _tidx_l = self.grid.coord_to_track(bot_layer, top_m.bound_box.xl, mode=RoundMode.GREATER_EQ)
            _tidx_h = self.grid.coord_to_track(bot_layer, top_m.bound_box.xh, mode=RoundMode.LESS_EQ)
            _num_dum = tr_manager.get_num_wires_between(bot_layer, 'cap', _tidx_l, 'cap', _tidx_h, 'cap')
            _tr_w_dum = tr_manager.get_width(top_layer, 'cap')
            _, _dum_locs = tr_manager.place_wires(bot_layer, ['cap'] * _num_dum,
                                                  center_coord=(top_m.bound_box.xh + top_m.bound_box.xl) // 2)
            bot_layer_bot = [self.connect_to_tracks(top_m, TrackID(bot_layer, _tidx, bot_w))
                             for _tidx in _dum_locs[::2]]
            if not cap_config.get('edge', False):
                bot_layer_top = [self.connect_to_tracks([top_l, top_h], TrackID(bot_layer, _tidx, bot_w))
                                 for _tidx in _dum_locs[1::2]]
            else:
                bot_layer_top = [self.add_wires(bot_layer, _tidx, bot_layer_bot[0].lower, bot_layer_bot[0].upper)
                                 for _tidx in _dum_locs[1::2]]
            if cap_config.get('extra_hor_couple', False):
                self.connect_to_track_wires(extra_hor_wlist[:2], bot_layer_top)
                self.connect_to_track_wires(extra_hor_wlist[2], bot_layer_bot)
            self.add_pin('minus_bot_layer', bot_layer_bot)
            self.add_pin('plus_bot_layer', bot_layer_top)

        if cap_config['two_extra_layer_couple']:
            _ext_layer = top_layer - 1 if top_layer < bot_layer else top_layer + 1
            _tidx_l = self.grid.coord_to_track(_ext_layer, top_m.bound_box.xl + pin_len, mode=RoundMode.GREATER_EQ)
            _tidx_h = self.grid.coord_to_track(_ext_layer, top_m.bound_box.xh - pin_len, mode=RoundMode.LESS_EQ)
            _num_dum = tr_manager.get_num_wires_between(_ext_layer, 'cap', _tidx_l, 'cap', _tidx_h, 'cap')
            _tr_w_dum = tr_manager.get_width(_ext_layer, 'cap')
            _, _dum_locs = tr_manager.place_wires(_ext_layer, ['cap'] * _num_dum,
                                                  center_coord=(top_m.bound_box.xh + top_m.bound_box.xl) // 2)
            # if _num_dum & 1:
            #     _dum_locs.pop(-1)
            ext_layer_bot = [self.connect_to_tracks(top_m, TrackID(_ext_layer, _tidx, bot_w))
                             for _tidx in _dum_locs[::2]]
            if not cap_config.get('edge', False):
                ext_layer_top = [self.connect_to_tracks([top_l, top_h], TrackID(_ext_layer, _tidx, bot_w)) for
                                 _tidx in _dum_locs[1::2]]
            else:
                ext_layer_top = [self.add_wires(_ext_layer, _tidx, ext_layer_bot[0].lower, ext_layer_bot[0].upper)
                                 for _tidx in _dum_locs[1::2]]
            self.add_pin('minus_ext_layer', ext_layer_bot)
            self.add_pin('plus_ext_layer', ext_layer_top)

        has_rmetal = cap_config.get('has_rmetal', True)

        if has_rmetal:
            res_top_box = top_m.bound_box
            res_top_box.set_interval(grid.get_direction(top_layer), top_m.bound_box.xh - 1 - 1,
                                     top_m.bound_box.xh - 1)
            res_bot_box = top_l.bound_box
            res_bot_box.set_interval(grid.get_direction(top_layer), top_m.bound_box.xl + 1,
                                     top_m.bound_box.xl + 1 + 1)
            if top_m_len_unit:
                self.add_res_metal(top_layer, res_bot_box)
                self.add_res_metal(top_layer, res_top_box)
        else:
            res_bot_box, res_top_box = None, None

        # set size
        bnd_box = BBox(0, 0, bot_mid_coord, h_tot)
        self.array_box = BBox(0, grid.get_track_offset(top_layer), bot_mid_coord,
                              h_tot - grid.get_track_offset(top_layer))
        self.set_size_from_bound_box(max(top_layer, bot_layer), bnd_box)

        if 'cap' in cap_config:
            self.sch_params = dict(
                res_plus=dict(layer=top_layer, w=res_top_box.h, l=res_top_box.w),
                res_minus=dict(layer=top_layer, w=res_bot_box.h, l=res_bot_box.w),
                cap=top_m_len_unit * cap_config['cap']
            )
        elif has_rmetal:
            self.sch_params = dict(
                res_plus=dict(layer=top_layer, w=res_top_box.h, l=res_top_box.w),
                res_minus=dict(layer=top_layer, w=res_bot_box.h, l=res_bot_box.w),
            )
        else:
            self.sch_params = dict(
                res_plus=None,
                res_minus=None,
            )

        # Fill metal dummy pattern
        if cap_config['two_extra_layer_couple']:
            top_layer -= 1
        extra_dum_fill_layer = cap_config.get('dum_fill_layer', [])
        dum_fill_layer = list(range(1, top_layer)) + extra_dum_fill_layer.to_list() if extra_dum_fill_layer else list(
            range(1, top_layer))
        for _layer in dum_fill_layer:
            if _layer == bot_layer and cap_config['bot_layer_couple']:
                continue
            # -- Vertical layers --
            if self.grid.get_direction(_layer) == Orient2D.y:
                _tidx_l = self.grid.coord_to_track(_layer, self.array_box.xl, mode=RoundMode.GREATER_EQ)
                _tidx_h = self.grid.coord_to_track(_layer, self.array_box.xh, mode=RoundMode.LESS_EQ)
                _num_dum = tr_manager.get_num_wires_between(_layer, 'dum', _tidx_l, 'dum', _tidx_h, 'dum')
                _tr_w_dum = tr_manager.get_width(_layer, 'dum')
                _dum_locs = tr_manager.spread_wires(_layer, ['dum'] * _num_dum, _tidx_l, _tidx_h,
                                                    sp_type=('dum', 'dum'))
                [self.add_wires(_layer, tidx, self.array_box.yl, self.array_box.yh, width=_tr_w_dum) for tidx in
                 _dum_locs]
            # -- Horizontal layers --
            else:
                if cap_config['same_pitch_hm_xm']:
                    _h_layer = _layer if _layer != hm_layer else xm_layer
                else:
                    _h_layer = _layer
                _tidx_l = self.grid.coord_to_track(_h_layer, self.bound_box.yl, mode=RoundMode.GREATER)
                _tidx_h = self.grid.coord_to_track(_h_layer, self.bound_box.yh, mode=RoundMode.LESS)
                _num_dum = tr_manager.get_num_wires_between(_h_layer, 'dum', _tidx_l, 'dum', _tidx_h, 'dum')
                _tr_w_dum = tr_manager.get_width(_layer, 'dum')
                _, _dum_locs = tr_manager.place_wires(_h_layer, ['dum'] * _num_dum,
                                                      center_coord=(self.array_box.yh + self.array_box.yl) // 2)
                _wire_width = self.grid.get_wire_bounds(_h_layer, 0, _tr_w_dum)
                _wire_width = _wire_width[1] - _wire_width[0]
                _track_mid_coord_list = [self.grid.track_to_coord(_h_layer, tidx) for tidx in _dum_locs]
                _min_le_sp = self.grid.get_line_end_space(_layer, 1)
                _bbox_arr = [BBox(self.bound_box.xl + _min_le_sp, _c - _wire_width // 2,
                                  top_m.upper, _c + _wire_width // 2) for _c in _track_mid_coord_list]
                [self.add_rect((f'm{_layer}', 'drawing'), _bbox) for _bbox in _bbox_arr]

        self.add_pin('minus', bot)
        self.add_pin('minus_x', [top_l, top_h], hide=True)
        self.add_pin('plus', top_m, mode=PinMode.LOWER)


class CapColCore(TemplateBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            ny='number of unit cap in column',
            ny_unit='number of unit cap',
            ratio='ratio of unit cell',
            cap_config='MOM cap configuration.',
            width='MOM cap width, in resolution units.',
            pin_tr_w='Width for top-plate pin',
            add_tap='Add tap to provides substrate',
            options='Other options, for use in ringamp',
            tr_widths='Track width',
            tr_spaces='Track space',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            cap_config={},
            ny_unit=0,
            width=0,
            pin_tr_w=1,
            ratio=8,
            add_tap=False,
            options={},
            tr_widths={},
            tr_spaces={},
        )

    def draw_layout(self) -> None:
        cap_config: ImmutableSortedDict[str, Union[int, float]] = self.params['cap_config']
        options: ImmutableSortedDict[str, Any] = self.params['options']
        add_tap: bool = self.params['add_tap']
        width: int = self.params['width']
        ratio: int = self.params['ratio']
        ny: int = self.params['ny']
        ny_unit: int = self.params['ny_unit']

        if not ny_unit:
            ny_unit = ny

        tr_widths: Dict = self.params['tr_widths']
        tr_spaces: Dict = self.params['tr_spaces']

        # if ny & 1:
        #     raise ValueError("Number of cell must be even number")

        grid = self.grid
        unit_pin_layer = options.get('pin_layer', cap_config['top_layer'] - 1)
        # add_tap = options.get('add_tap', False)
        unit_pin_tidx = grid.find_next_track(unit_pin_layer, 1, tr_width=cap_config.get('top_y_w', cap_config['top_w']),
                                             half_track=True)
        pin_conn_sep = grid.get_sep_tracks(unit_pin_layer, ntr1=cap_config.get('top_y_w', cap_config['top_w']), ntr2=1)

        cap_edge_config = copy.deepcopy(cap_config.to_dict())
        cap_half_config = copy.deepcopy(cap_config.to_dict())
        cap_half_edge_config = copy.deepcopy(cap_config.to_dict())
        cap_none_config = copy.deepcopy(cap_config.to_dict())
        cap_half_config['unit'] = 0.5
        cap_half_edge_config['unit'] = 0.5
        cap_none_config['unit'] = 0
        cap_edge_config['edge'] = True
        cap_half_edge_config['edge'] = True
        unit_master: TemplateBase = self.new_template(CapUnitCore, params=dict(cap_config=cap_config, width=width,
                                                                               tr_w=tr_widths, tr_sp=tr_spaces))
        unit_edge_master: TemplateBase = \
            self.new_template(CapUnitCore, params=dict(cap_config=cap_edge_config, width=width,
                                                       tr_w=tr_widths, tr_sp=tr_spaces))

        unit_half_master: TemplateBase = \
            self.new_template(CapUnitCore, params=dict(cap_config=cap_half_config, width=width,
                                                       tr_w=tr_widths, tr_sp=tr_spaces))
        unit_none_master: TemplateBase = \
            self.new_template(CapUnitCore, params=dict(cap_config=cap_none_config, width=width,
                                                       tr_w=tr_widths, tr_sp=tr_spaces))

        lay_top_layer = max(unit_pin_layer, unit_master.top_layer)
        w_blk, h_blk = grid.get_block_size(lay_top_layer, half_blk_x=True, half_blk_y=True)

        unit_x = grid.track_to_coord(unit_pin_layer, unit_pin_tidx + pin_conn_sep)
        unit_x = -(-unit_x // w_blk) * w_blk

        if ratio & 8:
            cdac = [
                self.add_instance(unit_edge_master if idx % ny_unit == 0 or (idx + 1) % ny_unit == 0 else unit_master,
                                  xform=Transform(unit_x, unit_master.array_box.h * idx))
                for idx in range(ny)]
            bbox = cdac[-1].bound_box.extend(x=0, y=0)
            cap_bot = self.connect_wires([w for c in cdac for w in c.get_all_port_pins('minus')])
            cap_top_list = [c.get_pin('plus') for c in cdac]
            array_bbox = cdac[0].array_box.merge(cdac[-1].array_box)
            ideal_cap = unit_master.sch_params.get('cap', 0)
            m = ny_unit
        elif ratio & 4:
            cdac = [self.add_instance(unit_master, xform=Transform(unit_x, unit_half_master.array_box.h * idx))
                    for idx in range(2)] + \
                   [self.add_instance(unit_none_master, xform=Transform(unit_x, unit_none_master.array_box.h * idx))
                    for idx in range(2, 4)]
            bbox = cdac[-1].bound_box.extend(x=0, y=0)
            cap_bot = self.connect_wires([w for c in cdac for w in c.get_all_port_pins('minus')])
            cap_top_list = [c.get_pin('plus') for c in cdac]
            array_bbox = cdac[0].array_box.merge(cdac[-1].array_box)
            ideal_cap = unit_half_master.sch_params.get('cap', 0)
            m = 2
        elif ratio & 2:
            cdac = [self.add_instance(unit_master, xform=Transform(unit_x, 0))] + \
                   [self.add_instance(unit_none_master, xform=Transform(unit_x, unit_half_master.array_box.h * idx))
                    for idx in range(1, 4)]
            bbox = cdac[-1].bound_box.extend(x=0, y=0)
            cap_bot = self.connect_wires([w for c in cdac for w in c.get_all_port_pins('minus')])
            cap_top_list = [c.get_pin('plus') for c in cdac]
            array_bbox = cdac[0].array_box.merge(cdac[-1].array_box)
            ideal_cap = unit_half_master.sch_params.get('cap', 0)
            m = 1
        elif ratio & 1:
            cdac = [self.add_instance(unit_half_master, xform=Transform(unit_x, 0))] + \
                   [self.add_instance(unit_none_master, xform=Transform(unit_x, unit_half_master.array_box.h * idx))
                    for idx in range(1, 4)]
            bbox = cdac[-1].bound_box.extend(x=0, y=0)
            cap_bot = self.connect_wires([w for c in cdac for w in c.get_all_port_pins('minus')])
            cap_top_list = [c.get_pin('plus') for c in cdac]
            array_bbox = cdac[0].array_box.merge(cdac[-1].array_box)
            ideal_cap = unit_half_master.sch_params.get('cap', 0)
            m = 1
        else:
            raise ValueError("Unit is wrong")

        cap_bot_x = self.connect_wires([w for c in cdac for w in c.get_all_port_pins('minus_x')])

        if add_tap:
            tech_global = get_tech_global_info('bag3_digital')
            pinfo = dict(
                lch=tech_global['lch_min'],
                top_layer=MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info, tech_global['lch_min']) + 1,
                tr_widths={},
                tr_spaces={},
                row_specs=[dict(mos_type='ptap', width=tech_global['w_minn'], threshold='standard',
                                bot_wires=['sup'], top_wires=[])]
            )
            tap_master = self.new_template(CapTap, params=dict(pinfo=pinfo))
            tap = self.add_instance(tap_master, xform=Transform(-tap_master.bound_box.w, 0, Orientation.MY))
            self.reexport(tap.get_port('VSS'))

        self.set_size_from_bound_box(max(cap_config['top_layer'], cap_config['bot_layer']), bbox)
        self.array_box = array_bbox

        top_pin_list = []
        for idx in range(0, ny, ny_unit):
            _pin = self.connect_to_tracks(cap_top_list[idx: idx + ny_unit],
                                          TrackID(unit_pin_layer, unit_pin_tidx,
                                                  cap_config.get('top_y_w', cap_config['top_w'])))

            if cap_config['bot_layer_couple'] and ratio > 2:
                self.connect_wires(
                    [w for cap in cdac[idx: idx + ny_unit] for w in cap.get_all_port_pins('minus_bot_layer')])
                self.connect_wires(
                    [w for cap in cdac[idx: idx + ny_unit] for w in cap.get_all_port_pins('plus_bot_layer')])
            if cap_config['two_extra_layer_couple'] and ratio > 2:
                self.connect_wires(
                    [w for cap in cdac[idx: idx + ny_unit] for w in cap.get_all_port_pins('minus_ext_layer')])
                self.connect_wires([w for cap in cdac[idx: idx + 4] for w in cap.get_all_port_pins('plus_ext_layer')])
            if cdac[0].has_port('minus_extra_hor'):
                self.connect_to_track_wires(_pin, [w for cap in cdac[idx: idx + ny_unit] for w in
                                                   cap.get_all_port_pins('minus_extra_hor')])
            top_pin_list.append(_pin)
            self.add_pin(f'top_xm', cap_top_list[idx: idx + 4], hide=True)

        connect_top = options.get('connect_top', False)
        if connect_top:
            self.add_pin('top', self.connect_wires(top_pin_list))
        else:
            [self.add_pin(f'top', _pin) for _pin in top_pin_list]
        array_box_l = self.grid.track_to_coord(top_pin_list[0].layer_id, top_pin_list[0].track_id.base_index)
        self.array_box.extend(x=array_box_l)

        self.add_pin('bot', cap_bot)
        self.add_pin('bot_x', cap_bot_x, hide=True)
        new_sch_params = dict(m=m, plus_term='top', minus_term='bot')
        if ideal_cap:
            new_sch_params['cap'] = ideal_cap
        self.sch_params = \
            unit_master.sch_params.copy(append=new_sch_params)


class CapDrvCore(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._cap_align_ofst_y = 0

    @property
    def cap_align_ofst_y(self):
        return self._cap_align_ofst_y

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_drv')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            min_height='Height to match capdac',
            pinfo='placement information object.',
            seg='segments dictionary.',
            sp='dummy seperation',
            w='widths.',
            gated='Index of gated switches',
            dum_row_idx='Index of dummy rows',
            cm_row_idx='Index of common mode rows',
            sw_type='Type of switch',
            nx='number of columns',
            tot_seg='Total number of fg',
            is_cm='True to implement cm sw',
            top_layer='supply and reference top layer',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w=4,
            sp=2,
            # ny=5,
            min_height=0,
            gated=[],
            dum_row_idx=[],
            cm_row_idx=[],
            nx=3,
            sw_type='nch',
            tot_seg=0,
            is_cm=False,
        )

    def draw_layout(self):
        is_cm = self.params['is_cm']
        min_height: int = self.params['min_height']
        sw_type: str = self.params['sw_type']
        gated: List[int] = self.params['gated']
        nx: int = self.params['nx']
        w: int = self.params['w']
        top_layer: int = self.params['top_layer']
        pinfo_dict = self.params['pinfo'].to_yaml()

        if min_height > 0:
            pinfo_dict['tile_specs']['place_info']['drv_tile']['min_height'] = min_height
        pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['mos_type'] = sw_type
        pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['width'] = w
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        pinfo0 = [TilePatternElement(pinfo[1]['drv_tile'])]
        self.draw_base((TilePattern(pinfo0), pinfo[1]))

        seg: int = self.params['seg']
        sp: int = self.params['sp']
        w: int = self.params['w']

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')
        tr_sup_xm_w = tr_manager.get_width(xm_layer, 'sup')
        tr_ref_xm_w = tr_manager.get_width(xm_layer, 'ref')

        ctrl_list_list, vref_list_list = [], []

        pin_lower = self.arr_info.col_to_coord(0)
        vdd_list, vss_list = [], []
        sup_bot_tid_list = []
        tap_ncol = self.get_tap_ncol(tile_idx=0)
        tap_sep_col = self.sub_sep_col
        tap_ncol += tap_sep_col

        tot_seg = max(self.params['tot_seg'], nx * seg + max(0, nx - 1) * sp + seg * len(gated))
        side_ncol = max(tap_ncol, (tot_seg - nx * seg - max(0, nx - 1) * sp) // 2 - 1)
        side_ncol += side_ncol & 1
        tot_seg = side_ncol + nx * seg + max(0, nx - 1) * sp + seg * len(gated) + self.min_sep_col

        cm_ctrl, cm_vref = [], []
        self.add_tap(0, vdd_list, vss_list, tile_idx=0)
        tid_bot = self.get_track_id(0, MOSWireType.DS, wire_name='ref', wire_idx=0, tile_idx=0)
        tid_ref = self.get_track_id(0, MOSWireType.DS, wire_name='sig', wire_idx=0, tile_idx=0)

        sw_col = side_ncol + 1

        # if nx != 2:
        tid_list = []
        if is_cm:
            sw_list = [self.add_mos(0, sw_col + seg + sp, seg, w=w, tile_idx=0)]
            tid = self.get_track_id(0, MOSWireType.G, wire_name='ctrl', wire_idx=0, tile_idx=0)
            cm_vref = self.connect_to_tracks(sw_list[0].d, tid_ref, min_len_mode=MinLenMode.MIDDLE)
            cm_ctrl = self.connect_to_tracks(sw_list[0].g, tid, min_len_mode=MinLenMode.MIDDLE)

        else:
            sw_list, ctrl_list, vref_list = [], [], []
            for jdx in range(nx):
                sw_list.append(self.add_mos(0, sw_col - seg // 2 if jdx in gated else sw_col,
                                            seg, w=w, tile_idx=0, stack=1 + int(jdx in gated), g_on_s=jdx in gated))
                sw_col += seg * (1 + int(jdx in gated)) + sp
                tid_list.append(self.get_track_id(0, MOSWireType.G, wire_name='ctrl',
                                                  wire_idx=-jdx - 1 if nx != 2 else jdx, tile_idx=0))
                vref_list.append(self.connect_to_tracks(sw_list[-1].d, tid_ref, min_len_mode=MinLenMode.MIDDLE))

            for jdx in range(nx):
                if jdx in gated:
                    ctrl_list.append(self.connect_to_tracks(sw_list[jdx].g[1::2], tid_list[jdx], track_lower=pin_lower,
                                                            min_len_mode=MinLenMode.MIDDLE))

                else:
                    ctrl_list.append(self.connect_to_tracks(sw_list[jdx].g, tid_list[jdx], track_lower=pin_lower,
                                                            min_len_mode=MinLenMode.MIDDLE))

            ctrl_list_list.append(ctrl_list)
            vref_list_list.append(vref_list)
        # self.add_tap(tot_seg, vdd_list, vss_list, tile_idx=0, flip_lr=True)
        cap_bot_list = self.connect_to_tracks([sw.s for sw in sw_list], tid_bot)

        # supply_hm
        sup_bot_tid_list.append(self.get_track_id(0, MOSWireType.G, wire_name='sup', tile_idx=0))
        sup_bot_tid_list.append(self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=0))
        sup_bot_tid_list.append(self.get_track_id(0, MOSWireType.G, wire_name='sup', wire_idx=1, tile_idx=0))

        self.set_mos_size(max(tot_seg, self.num_cols + 2))

        sup_hm_list = []
        for tid in sup_bot_tid_list:
            sup_conn_list = vdd_list if sw_type == 'pch' else vss_list
            sup_hm_list.append(self.connect_to_tracks(sup_conn_list, tid))

        tr_w_hm_ctrl = tr_manager.get_width(hm_layer, 'ctrl')
        en_hm_avail = \
            self.get_available_tracks(hm_layer,
                                      self.grid.coord_to_track(hm_layer, sw_list[0].g.lower, RoundMode.NEAREST),
                                      self.grid.coord_to_track(hm_layer, self.bound_box.yh, RoundMode.NEAREST),
                                      self.bound_box.xl, self.bound_box.xh, tr_w_hm_ctrl,
                                      tr_manager.get_sep(hm_layer, ('ctrl', 'ctrl')))

        if gated and not is_cm:
            is_nch = self.get_row_info(0, 0).row_type == MOSType.nch
            en_hm = self.connect_to_tracks([sw_list[idx].g[::2] for idx in gated],
                                           TrackID(hm_layer, en_hm_avail[-1], tr_w_hm_ctrl))
            self.add_pin('enb' if is_nch else 'en', en_hm)

        ##########
        # supply and vref vm connections
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')

        ###########
        # vm connection
        ###########
        sup_vm_locs = self.get_tids_between(vm_layer,
                                            self.arr_info.col_to_track(vm_layer, 0),
                                            self.arr_info.col_to_track(vm_layer, tap_ncol - 1),
                                            tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_sup_vm_w),
                                            0, include_last=True)

        # sup_vm_locs += self.get_tids_between(vm_layer,
        #                                      self.arr_info.col_to_track(vm_layer, self.num_cols - tap_ncol + 1),
        #                                      self.arr_info.col_to_track(vm_layer, self.num_cols),
        #                                      tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_sup_vm_w),
        #                                      0, include_last=True, align_to_higher=True)
        sup_vm_list = []
        for tid in sup_vm_locs:
            sup_vm_list.append(self.connect_to_tracks(sup_hm_list, tid))

        # Supply wire on the other side
        sup_vm_r = self.arr_info.col_to_track(vm_layer, self.num_cols - 1)
        self.connect_to_tracks(sup_hm_list, TrackID(vm_layer, sup_vm_r))

        cap_bot_vm_tid = []
        for idx in range(1 if is_cm else nx):
            tx = sw_list[idx]
            cap_bot_vm_tid.append(TrackID(vm_layer, tx.s.track_id.base_index, pitch=tx.s.track_id.pitch,
                                          num=tx.s.track_id.num, width=tx.s.track_id.width))
        cap_bot_vm = [self.connect_to_tracks(cap_bot_list, tid) for tid in cap_bot_vm_tid]

        vref_vm_list = []
        if is_cm:
            tx = sw_list[0]
            vref_vm_tid = TrackID(vm_layer, tx.d.track_id.base_index,
                                  num=tx.d.track_id.num, width=tx.d.track_id.width)
            vref_vm_list.append(self.connect_to_tracks(cm_vref, vref_vm_tid, track_upper=self.bound_box.yh,
                                                       track_lower=self.bound_box.yl))
        else:
            for idx in range(nx):
                tx = sw_list[idx]
                vref_vm_tid = TrackID(vm_layer, tx.d.track_id.base_index, pitch=tx.s.track_id.pitch,
                                      num=tx.d.track_id.num, width=tx.d.track_id.width)
                vref_vm_list.append(self.connect_to_tracks([vref_list[idx] for vref_list in vref_list_list],
                                                           vref_vm_tid, track_upper=self.bound_box.yh,
                                                           track_lower=self.bound_box.yl))

        ###########
        # xm connection
        ###########

        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        fill_conn_layer_intv(self, 0, 0)
        tile_info, yb, _ = self.get_tile_info(0)
        tr_sp_sup_sig_xm = self.get_track_sep(xm_layer, tr_w_sup_xm, tr_w_sig_xm)
        tr_sp_dum_sig_xm = self.get_track_sep(xm_layer, 1, tr_w_sig_xm)
        sup_xm_locs = self.grid.coord_to_track(xm_layer, yb, RoundMode.NEAREST)
        sig_bot_xm_tid = self.get_tids_between(xm_layer, sup_xm_locs + tr_sp_sup_sig_xm,
                                               self.grid.coord_to_track(xm_layer, yb + tile_info.height,
                                                                        RoundMode.NEAREST) - 1,
                                               tr_w_sig_xm, 0, 0, False, center=False)
        sig_bot_xm_tid = [sig_bot_xm_tid[len(sig_bot_xm_tid) // 2]]
        # sig_bot_xm_tid = [sig_bot_xm_tid[len(sig_bot_xm_tid)//2]]

        #
        vref_xm_list = [[] for _ in range(len(vref_vm_list))]
        sup_xm_list = []
        len_sup_vm = len(sup_vm_list)
        for idx, vref in enumerate(vref_vm_list):
            _vref_xm = self.connect_to_tracks(vref, TrackID(xm_layer, sup_xm_locs, tr_ref_xm_w, grid=self.grid),
                                              min_len_mode=MinLenMode.MIDDLE)
            vref_xm_list[idx].append(_vref_xm)
            # self.add_pin(f'vref{idx}_xm', vref)

        sup_xm_list.extend([self.connect_to_tracks(sup_vm_list[:len_sup_vm // 2],
                                                   TrackID(xm_layer, sup_xm_locs, tr_sup_xm_w, grid=self.grid)),
                            self.connect_to_tracks(sup_vm_list[len_sup_vm // 2:],
                                                   TrackID(xm_layer, sup_xm_locs, tr_sup_xm_w, grid=self.grid))])

        cap_bot_xm = WireArray.list_to_warr([self.connect_to_tracks(cap_bot_vm, tid) for tid in sig_bot_xm_tid])
        ###########
        # from xm to top_layer connection
        ###########
        sup_xm_list_l, sup_xm_list_r = sup_xm_list[0::2], sup_xm_list[1::2]

        vref_top_list_list = []
        for vref_xm in vref_xm_list:
            ref_tid_list = []
            ref_bnd_list = []
            for idx in range(xm_layer + 1, top_layer + 1):
                lay_dir = self.grid.get_direction(idx)
                _tr_w = tr_manager.get_width(idx, 'ref')
                if lay_dir == Orient2D.y:
                    ref_locs = \
                        self.get_tids_between(idx, self.grid.coord_to_track(idx, vref_xm[0].lower, RoundMode.NEAREST),
                                              self.grid.coord_to_track(idx, vref_xm[0].upper, RoundMode.NEAREST),
                                              _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                              0, include_last=True)

                    ref_tid_list.append(ref_locs)
                    ref_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
                else:
                    ref_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                    ref_tid_list.append([TrackID(idx, ref_loc, _tr_w, grid=self.grid)])
                    ref_bnd_list.append(None)

            vref_top_list_list.append(self.connect_warr_to_tids_stack(vref_xm, ref_tid_list, ref_bnd_list,
                                                                      MinLenMode.MIDDLE))

        for idx, vref_list in enumerate(vref_top_list_list):
            for vref in vref_list:
                self.add_pin(f'vref{idx}', vref)

        # Supply
        self.add_pin('VSS_xm' if sw_type == 'nch' else 'VDD_xm', sup_xm_list)
        sup_tid_list_l, sup_tid_list_r = [], []
        sup_bnd_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            _tr_w = tr_manager.get_width(idx, 'sup')
            if lay_dir == Orient2D.y:
                sup_locs_l = self.get_tids_between(idx,
                                                   self.arr_info.col_to_track(idx, 0),
                                                   self.arr_info.col_to_track(idx, tap_ncol - 1),
                                                   _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                                                   0, include_last=True)

                # sup_locs_r = self.get_tids_between(idx,
                #                                    self.arr_info.col_to_track(idx, self.num_cols - tap_ncol + 1),
                #                                    self.arr_info.col_to_track(idx, self.num_cols),
                #                                    _tr_w, self.get_track_sep(idx, _tr_w, _tr_w),
                #                                    0, include_last=True, align_to_higher=True)
                sup_tid_list_l.append(sup_locs_l)
                # sup_tid_list_r.append(sup_locs_r)
                sup_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
            else:
                sup_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                sup_tid_list_l.append([TrackID(idx, sup_loc, _tr_w, grid=self.grid)])
                sup_tid_list_r.append([TrackID(idx, sup_loc, _tr_w, grid=self.grid)])
                sup_bnd_list.append(None)

        sup_l_warr_list: List[List[WireArray]] = \
            self.connect_warr_to_tids_stack(sup_xm_list_l, sup_tid_list_l, sup_bnd_list, MinLenMode.MIDDLE)
        # sup_r_warr_list = self.connect_warr_to_tids_stack(sup_xm_list_r, sup_tid_list_r, sup_bnd_list,
        #                                                   MinLenMode.MIDDLE)

        self.add_pin('VSS' if sw_type == 'nch' else 'VDD',
                     [w for warr in sup_l_warr_list for w in warr])

        for idx in range(nx):
            self.add_pin(f'ctrl{idx}', [ctrl_list[idx] for ctrl_list in ctrl_list_list])

        self.add_pin('cm_ctrl', cm_ctrl)

        self.add_pin(f'bot', cap_bot_xm)
        self.sch_params = dict(
            lch=self.arr_info.lch,
            w=w,
            seg=seg,
            intent=self.get_row_info(0, 0).threshold
        )


class CapDrvSplitCore(MOSBase, TemplateBaseZL):
    """A inverter with only transistors drawn, no metal connections
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._cap_align_ofst_y = 0

    @property
    def cap_align_ofst_y(self):
        return self._cap_align_ofst_y

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_drv_split')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            min_height='Height to match capdac',
            pinfo='placement information object.',
            seg_dict='segments dictionary.',
            w_dict='widths.',
            gated='True to used stack transistors for each switch, for fast first bit flipping',
            tot_seg='Total number of fg',
            top_layer='supply and reference top layer',
            dum_row_idx='Index of dummy rows',
            cm_row_idx='Index of common mode rows',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_dict={},
            min_height=0,
            gated=False,
            dum_row_idx=[],
            cm_row_idx=[],
            tot_seg=0,
        )

    def draw_layout(self):
        min_height: int = self.params['min_height']
        w_dict: dict = self.params['w_dict']
        gated: bool = self.params['gated']
        top_layer: int = self.params['top_layer']

        pinfo_dict = self.params['pinfo'].to_yaml()
        if min_height > 0:
            pinfo_dict['tile_specs']['place_info']['drv_tile']['min_height'] = min_height
        # pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['width'] = w
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        pinfo0 = [TilePatternElement(pinfo[1]['drv_tile'])]
        self.draw_base((TilePattern(pinfo0), pinfo[1]))

        seg_dict: Dict = self.params['seg_dict']
        seg_mux: int = seg_dict['seg_mux']
        seg: int = seg_dict['seg']
        seg_rst: int = seg_dict['seg_rst']
        sp_tap = seg_dict['sp_tap']
        sp: int = seg_dict['sp']

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        tr_sup_vm_w = tr_manager.get_width(vm_layer, 'sup')

        sup_bot_tid_list = []
        tap_ncol = self.get_tap_ncol(tile_idx=0)
        tap_sep_col = self.sub_sep_col
        tap_ncol += tap_sep_col

        # total seg calculations
        tot_seg = tap_ncol + sp_tap + 2 * seg * (1 + int(gated))
        tot_seg = max(tot_seg, self.params['tot_seg'])

        cm_ctrl, cm_vref = [], []
        # add tap
        vss_list = self.add_substrate_contact(0, 0, seg=tap_ncol - tap_sep_col)
        vdd_list = self.add_substrate_contact(1, 0, seg=tap_ncol - tap_sep_col)

        mux_col = tap_ncol + sp_tap

        mux_n0 = self.add_mos(0, mux_col + seg_mux, seg_mux, w=w_dict.get('muxn', 4), tile_idx=0, flip_lr=True)
        mux_p0 = self.add_mos(1, mux_col + seg_mux, seg_mux, w=w_dict.get('muxp', 4), tile_idx=0, flip_lr=True)
        fill_conn_layer_intv(self, 0, 0, stop_col=mux_col + seg_mux)
        fill_conn_layer_intv(self, 0, 1, stop_col=mux_col + seg_mux)
        rstn = self.add_mos(0, mux_col + seg_mux, seg_rst, w=w_dict.get('rst', 4), tile_idx=0)

        mux_col += seg_mux + seg_rst + self.min_sep_col
        rstp = self.add_mos(1, mux_col, seg_rst, w=w_dict.get('rst', 4), tile_idx=0, flip_lr=True)
        mux_n1 = self.add_mos(0, mux_col, seg_mux, w=w_dict.get('muxn', 4), tile_idx=0)
        mux_p1 = self.add_mos(1, mux_col, seg_mux, w=w_dict.get('muxp', 4), tile_idx=0)
        #
        sw_col = mux_col + seg_mux + max(self.min_sep_col + 1, 4)
        mux_sw_mid_col = sw_col - self.min_sep_col // 2
        sw_n0 = self.add_mos(0, sw_col - seg // 2 if gated else sw_col,
                             seg, w=w_dict.get('n', 4), tile_idx=0, stack=1 + int(gated), g_on_s=gated)
        sw_p0 = self.add_mos(1, sw_col - seg // 2 if gated else sw_col,
                             seg, w=w_dict.get('p', 4), tile_idx=0, stack=1 + int(gated), g_on_s=gated)
        sw_col += seg * (1 + int(gated)) + sp
        sw_n1 = self.add_mos(0, sw_col - seg // 2 if gated else sw_col,
                             seg, w=w_dict.get('n', 4), tile_idx=0, stack=1 + int(gated), g_on_s=gated)
        sw_p1 = self.add_mos(1, sw_col - seg // 2 if gated else sw_col,
                             seg, w=w_dict.get('p', 4), tile_idx=0, stack=1 + int(gated), g_on_s=gated)
        self.set_mos_size(max(tot_seg, self.num_cols + 2))
        fill_conn_layer_intv(self, 0, 0, start_col=mux_col)
        fill_conn_layer_intv(self, 0, 1, start_col=mux_col)

        # get tid
        tid_ref_n = self.get_track_id(0, MOSWireType.DS, wire_name='sup', wire_idx=0, tile_idx=0)
        tid_ref_p = self.get_track_id(1, MOSWireType.DS, wire_name='sup', wire_idx=0, tile_idx=0)
        _, [tid_ctrl_n, tid_ctrl_p] = tr_manager.place_wires(hm_layer, ['ctrl_ow'] * 2,
                                                             center_coord=self.bound_box.h // 2)
        tid_vss, tid_vdd = tid_ref_n, tid_ref_p
        tr_w_ctrl_hm = tr_manager.get_width(hm_layer, 'ctrl_ow')

        vrefp_hm = connect_hm_sup_ow(self, [sw_p0.d, sw_p1.d], 0, 1, tr_manager, 'sup')
        vrefn_hm = connect_hm_sup_ow(self, [sw_n0.d, sw_n1.d], 0, 0, tr_manager, 'sup')
        vdd_list, vss_list = [vdd_list, rstp.d], [vss_list, rstn.d]
        vdd_hm = connect_hm_sup_ow(self, vdd_list, 0, 1, tr_manager, 'sup')
        vss_hm = connect_hm_sup_ow(self, vss_list, 0, 0, tr_manager, 'sup')
        vdd_hm, vss_hm = self.match_warr_length([vdd_hm, vss_hm])

        # ctrln_hm, ctrlp_hm = self.connect_matching_tracks([[sw_p0.g, sw_n0.g], [sw_p1.g, sw_n1.g]], hm_layer,
        #                                                   [tid_ctrl_n, tid_ctrl_p], width=tr_w_ctrl_hm)
        #
        tid_bot_n = tr_manager.get_next_track(hm_layer, vrefn_hm.track_id.base_index, 'sup_ow', 'sig_ow')
        tid_bot_p = tr_manager.get_next_track(hm_layer, vrefp_hm.track_id.base_index, 'sup_ow', 'sig_ow',
                                              up=False)
        tr_w_sig_ow_hm = tr_manager.get_width(hm_layer, 'sig_ow')
        tid_bot_n = TrackID(hm_layer, tid_bot_n, tr_w_sig_ow_hm)
        tid_bot_p = TrackID(hm_layer, tid_bot_p, tr_w_sig_ow_hm)
        sign0_hm = self.connect_to_tracks(sw_n0.s, tid_bot_n, min_len_mode=MinLenMode.MIDDLE)
        sigp0_hm = self.connect_to_tracks(sw_p0.s, tid_bot_p, min_len_mode=MinLenMode.MIDDLE)
        sign1_hm = self.connect_to_tracks(sw_n1.s, tid_bot_n, min_len_mode=MinLenMode.MIDDLE)
        sigp1_hm = self.connect_to_tracks(sw_p1.s, tid_bot_p, min_len_mode=MinLenMode.MIDDLE)

        # ctrl buffer connections
        dnbuf_conn = self.connect_wires([mux_n0.s, mux_p0.s])
        dpbuf_conn = self.connect_wires([mux_n1.s, mux_p1.s])
        dnbuf_tidx = self.grid.coord_to_track(hm_layer, mux_n0.g.lower, RoundMode.GREATER_EQ)
        dpbuf_tidx = self.grid.coord_to_track(hm_layer, mux_p0.g.upper, RoundMode.LESS_EQ)
        samb_tidx = tr_manager.get_next_track(hm_layer, dnbuf_tidx, 'sig', 'sig')
        sam_tidx = tr_manager.get_next_track(hm_layer, dpbuf_tidx, 'sig', 'sig', up=-1)
        tr_w_hm_sam = tr_manager.get_width(hm_layer, 'sig')
        sam_hm = self.connect_to_tracks([mux_p0.g, mux_p1.g, rstn.g], TrackID(hm_layer, sam_tidx, tr_w_hm_sam))
        samb_hm = self.connect_to_tracks([mux_n0.g, mux_n1.g, rstp.g], TrackID(hm_layer, samb_tidx, tr_w_hm_sam))
        dnbuf_hm = self.connect_to_tracks([sw_n0.g] + dnbuf_conn, TrackID(hm_layer, dnbuf_tidx, tr_w_hm_sam))
        dpbuf_hm = self.connect_to_tracks([sw_p1.g] + dpbuf_conn, TrackID(hm_layer, dpbuf_tidx, tr_w_hm_sam))

        dn_c_hm_tidx = self.grid.coord_to_track(hm_layer, mux_n0.d.upper, RoundMode.LESS_EQ)
        dp_c_hm_tidx = self.grid.coord_to_track(hm_layer, mux_p0.d.lower, RoundMode.GREATER_EQ)
        dn_hm_tidx = tr_manager.get_next_track(hm_layer, dp_c_hm_tidx, 'sig', 'ctrl_ow', up=1)
        dp_hm_tidx = tr_manager.get_next_track(hm_layer, dn_c_hm_tidx, 'sig', 'ctrl_ow', up=-1)

        dn_hm = self.connect_to_tracks(mux_p0.d, TrackID(hm_layer, dn_hm_tidx, tr_w_ctrl_hm),
                                       min_len_mode=MinLenMode.MIDDLE)
        dp_hm = self.connect_to_tracks(mux_n1.d, TrackID(hm_layer, dp_hm_tidx, tr_w_ctrl_hm),
                                       min_len_mode=MinLenMode.MIDDLE)
        dn_c_hm = self.connect_to_tracks(mux_n0.d, TrackID(hm_layer, dn_c_hm_tidx, tr_w_hm_sam),
                                         min_len_mode=MinLenMode.MIDDLE)
        dp_c_hm = self.connect_to_tracks(mux_p1.d, TrackID(hm_layer, dp_c_hm_tidx, tr_w_hm_sam),
                                         min_len_mode=MinLenMode.MIDDLE)
        dn_tidx = tr_manager.get_next_track(hm_layer, dpbuf_tidx, 'sig', 'sig', up=-1)
        dp_tidx = tr_manager.get_next_track(hm_layer, dnbuf_tidx, 'sig', 'sig')
        dn_sw_hm = self.connect_to_tracks(sw_p0.g, TrackID(hm_layer, dn_tidx, tr_w_hm_sam))
        dp_sw_hm = self.connect_to_tracks(sw_n1.g, TrackID(hm_layer, dp_tidx, tr_w_hm_sam))
        self.match_warr_length([dn_sw_hm, dp_sw_hm])
        self.match_warr_length([dn_hm, dp_hm])

        ##########
        # supply and vref vm connections
        ##########
        # sam vm
        _, [sam_vm_tidx, samb_vm_tidx] = tr_manager.place_wires(vm_layer, ['clk'] * 2, center_coord=samb_hm.middle)
        sam_vm, samb_vm = self.connect_differential_tracks(sam_hm, samb_hm, vm_layer,
                                                           sam_vm_tidx, samb_vm_tidx,
                                                           width=tr_manager.get_width(vm_layer, 'clk'))
        sam_xm_tidx = self.grid.coord_to_track(xm_layer, self.grid.track_to_coord(hm_layer, sam_tidx),
                                               RoundMode.NEAREST)
        samb_xm_tidx = self.grid.coord_to_track(xm_layer, self.grid.track_to_coord(hm_layer, samb_tidx),
                                                RoundMode.NEAREST)
        sam_xm, samb_xm = self.connect_differential_tracks(sam_vm, samb_vm, xm_layer,
                                                           sam_xm_tidx, samb_xm_tidx,
                                                           width=tr_manager.get_width(xm_layer, 'clk'))

        _, [sam_ym_tidx, samb_ym_tidx] = tr_manager.place_wires(ym_layer, ['clk'] * 2, center_coord=samb_xm.middle)
        sam_ym, samb_ym = self.connect_differential_tracks(sam_xm, samb_xm, ym_layer,
                                                           sam_ym_tidx, samb_ym_tidx,
                                                           width=tr_manager.get_width(ym_layer, 'clk'))
        # ctrl vm
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        dn_vm_tidx = self.grid.coord_to_track(vm_layer, dn_hm.middle, RoundMode.NEAREST)
        dp_vm_tidx = self.grid.coord_to_track(vm_layer, dp_hm.middle, RoundMode.NEAREST)
        dn_vm = self.connect_to_tracks([dn_hm, dn_c_hm], TrackID(vm_layer, dn_vm_tidx, tr_w_ctrl_vm))
        dp_vm = self.connect_to_tracks([dp_hm, dp_c_hm], TrackID(vm_layer, dp_vm_tidx, tr_w_ctrl_vm))

        mux_sw_mid_tidx = self.arr_info.col_to_track(vm_layer, mux_sw_mid_col, RoundMode.NEAREST) - HalfInt(1)
        self.connect_to_tracks([dn_hm, dn_sw_hm], TrackID(vm_layer, mux_sw_mid_tidx, tr_w_ctrl_vm))
        self.connect_to_tracks([dp_hm, dp_sw_hm], TrackID(vm_layer, mux_sw_mid_tidx, tr_w_ctrl_vm))

        # supply
        tr_w_sup_vm = tr_manager.get_width(vm_layer, 'sup')

        sup_vm_locs = self.get_tids_between(vm_layer,
                                            self.arr_info.col_to_track(vm_layer, 0),
                                            self.arr_info.col_to_track(vm_layer, tap_ncol - 1),
                                            tr_w_sup_vm, self.get_track_sep(vm_layer, tr_w_sup_vm, tr_sup_vm_w),
                                            0, include_last=True, center=False)

        vdd_vm_list, vss_vm_list = [], []
        for tid in sup_vm_locs:
            vdd_vm_list.append(self.connect_to_tracks(vdd_hm, tid, min_len_mode=MinLenMode.MIDDLE))
            vss_vm_list.append(self.connect_to_tracks(vss_hm, tid, min_len_mode=MinLenMode.MIDDLE))

        # vref
        tr_w_ref_vm = tr_manager.get_width(vm_layer, 'ref')
        vref_vm_tid0 = TrackID(vm_layer, sw_n0.d.track_id.base_index, pitch=sw_n0.d.track_id.pitch,
                               num=sw_n0.d.track_id.num, width=tr_w_ref_vm, grid=self.grid)
        vref_vm_tid1 = TrackID(vm_layer, sw_n1.d.track_id.base_index, pitch=sw_n1.d.track_id.pitch,
                               num=sw_n1.d.track_id.num, width=tr_w_ref_vm, grid=self.grid)
        vrefp_vm = [self.connect_to_tracks(vrefp_hm, vref_vm_tid0, min_len_mode=MinLenMode.MIDDLE),
                    self.connect_to_tracks(vrefp_hm, vref_vm_tid1, min_len_mode=MinLenMode.MIDDLE)]
        vrefn_vm = [self.connect_to_tracks(vrefn_hm, vref_vm_tid0, min_len_mode=MinLenMode.MIDDLE),
                    self.connect_to_tracks(vrefn_hm, vref_vm_tid1, min_len_mode=MinLenMode.MIDDLE)]

        #########
        # sig vm connections
        #########
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')

        sig_vm_tid0 = TrackID(vm_layer, sw_n0.s.track_id.base_index, pitch=sw_n0.s.track_id.pitch,
                              num=sw_n0.s.track_id.num, width=tr_w_sig_vm)
        sig_vm_tid1 = TrackID(vm_layer, sw_n1.s.track_id.base_index, pitch=sw_n1.s.track_id.pitch,
                              num=sw_n1.s.track_id.num, width=tr_w_sig_vm)
        sign_vm = self.connect_to_tracks([sign0_hm, sigp0_hm], sig_vm_tid0)
        sigp_vm = self.connect_to_tracks([sign1_hm, sigp1_hm], sig_vm_tid1)

        ###########
        # xm connection
        ###########
        tr_w_sup_xm = tr_manager.get_width(xm_layer, 'sup')
        tr_w_sig_xm = tr_manager.get_width(xm_layer, 'sig')
        tile_info, yb, _ = self.get_tile_info(0)
        tr_sp_sup_sig_xm = self.get_track_sep(xm_layer, tr_w_sup_xm, tr_w_sig_xm)
        tr_sp_dum_sig_xm = self.get_track_sep(xm_layer, 1, tr_w_sig_xm)
        sup_xm_locs = [self.grid.coord_to_track(xm_layer, yb, RoundMode.NEAREST),
                       self.grid.coord_to_track(xm_layer, yb + tile_info.height, RoundMode.NEAREST)]
        if tr_w_sup_xm > 0 and tr_w_sig_xm > 0:
            tr_sp_sig_sup = tr_manager.get_sep(xm_layer, ('sig', 'sup'))
        else:
            tr_sp_sig_sup = self.get_track_sep(xm_layer, tr_w_sig_xm, tr_w_sup_xm)

        sig_bot_xm_tid = self.get_tids_between(xm_layer, sup_xm_locs[0] + tr_sp_sup_sig_xm,
                                               sup_xm_locs[-1] - tr_sp_sup_sig_xm,
                                               tr_w_sig_xm, tr_sp_sig_sup, 0, False, center=True)

        cap_bot_n_xm = WireArray.list_to_warr([self.connect_to_tracks(sign_vm, tid) for tid in [sig_bot_xm_tid[0]]])
        cap_bot_p_xm = WireArray.list_to_warr([self.connect_to_tracks(sigp_vm, tid) for tid in [sig_bot_xm_tid[-1]]])
        cap_bot_n_xm, cap_bot_p_xm = self.match_warr_length([cap_bot_n_xm, cap_bot_p_xm])
        vrefp_xm = self.connect_to_tracks(vrefp_vm, TrackID(xm_layer, sup_xm_locs[-1], width=tr_w_sup_xm,
                                                            grid=self.grid))
        vrefn_xm = self.connect_to_tracks(vrefn_vm, TrackID(xm_layer, sup_xm_locs[0], width=tr_w_sup_xm,
                                                            grid=self.grid))
        vdd_xm = self.connect_to_tracks(vdd_vm_list, TrackID(xm_layer, sup_xm_locs[-1], width=tr_w_sup_xm,
                                                             grid=self.grid))
        vss_xm = self.connect_to_tracks(vss_vm_list, TrackID(xm_layer, sup_xm_locs[0], width=tr_w_sup_xm,
                                                             grid=self.grid))
        ###########
        # from xm to top_layer connection
        ###########
        # sup_xm_list_l, sup_xm_list_r = sup_xm_list[0::2], sup_xm_list[1::2]
        #
        ref_tid_list = []
        ref_bnd_list = []
        vrefn_top_list_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            _tr_w = tr_manager.get_width(idx, 'ref')
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                ref_locs = \
                    self.get_tids_between(idx, self.grid.coord_to_track(idx, vrefn_xm.lower, RoundMode.NEAREST),
                                          self.grid.coord_to_track(idx, vrefn_xm.upper, RoundMode.NEAREST),
                                          _tr_w, self.get_track_sep(idx, _tr_w, _tr_w), 0, include_last=True)
                ref_tid_list.append(ref_locs if idx < top_layer else ref_locs[:len(ref_locs) // 2])
                ref_bnd_list.append((self.bound_box.yl, self.bound_box.yh) if idx == top_layer else None)
            else:
                ref_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                ref_tid_list.append([TrackID(idx, ref_loc, _tr_w, grid=self.grid)])
                ref_bnd_list.append(None)

        vrefn_top_list_list.append(self.connect_warr_to_tids_stack(vrefn_xm, ref_tid_list, ref_bnd_list,
                                                                   MinLenMode.MIDDLE, ret_wire=True))

        ref_tid_list = []
        ref_bnd_list = []
        vrefp_top_list_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            _tr_w = tr_manager.get_width(idx, 'ref')
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                ref_locs = \
                    self.get_tids_between(idx, self.grid.coord_to_track(idx, vrefp_xm.lower, RoundMode.NEAREST),
                                          self.grid.coord_to_track(idx, vrefp_xm.upper, RoundMode.NEAREST),
                                          _tr_w, self.get_track_sep(idx, _tr_w, _tr_w), 0, include_last=True)
                ref_tid_list.append(ref_locs if idx < top_layer else ref_locs[len(ref_locs) // 2:])
                # ref_bnd_list.append((self.bound_box.yl, self.bound_box.yh))
                ref_bnd_list.append((self.bound_box.yl, self.bound_box.yh) if idx == top_layer else None)
            else:
                ref_loc = self.grid.coord_to_track(idx, yb + self.get_tile_pinfo(0).height, RoundMode.NEAREST)
                ref_tid_list.append([TrackID(idx, ref_loc, _tr_w, grid=self.grid)])
                ref_bnd_list.append(None)

        vrefp_top_list_list.append(self.connect_warr_to_tids_stack(vrefp_xm, ref_tid_list, ref_bnd_list,
                                                                   MinLenMode.MIDDLE, ret_wire=True))
        vrefp_top, vrefn_top = self.match_warr_length(vrefp_top_list_list[0][-1] + vrefn_top_list_list[0][-1])
        self.match_warr_length(vrefp_top_list_list[0][-2] + vrefn_top_list_list[0][-2])

        sup_tid_list = []
        sup_bnd_list = []
        vss_list_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            _tr_w = tr_manager.get_width(idx, 'sup')
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                ref_locs = \
                    self.get_tids_between(idx, self.grid.coord_to_track(idx, vss_xm.lower, RoundMode.NEAREST),
                                          self.grid.coord_to_track(idx, vss_xm.upper, RoundMode.NEAREST),
                                          _tr_w, self.get_track_sep(idx, _tr_w, _tr_w), 0, include_last=True)
                sup_tid_list.append(ref_locs if idx < top_layer else ref_locs[:len(ref_locs) // 2])
                sup_bnd_list.append((self.bound_box.yl, self.bound_box.yh) if idx == top_layer else None)
            else:
                ref_loc = self.grid.coord_to_track(idx, yb, RoundMode.NEAREST)
                sup_tid_list.append([TrackID(idx, ref_loc, _tr_w, grid=self.grid)])
                sup_bnd_list.append(None)

        vss_list_list.append(self.connect_warr_to_tids_stack(vss_xm, sup_tid_list, sup_bnd_list,
                                                             MinLenMode.MIDDLE, ret_wire=True))

        sup_tid_list = []
        sup_bnd_list = []
        vdd_list_list = []
        for idx in range(xm_layer + 1, top_layer + 1):
            _tr_w = tr_manager.get_width(idx, 'sup')
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                ref_locs = \
                    self.get_tids_between(idx, self.grid.coord_to_track(idx, vdd_xm.lower, RoundMode.NEAREST),
                                          self.grid.coord_to_track(idx, vdd_xm.upper, RoundMode.NEAREST),
                                          _tr_w, self.get_track_sep(idx, _tr_w, _tr_w), 0, include_last=True)
                sup_tid_list.append(ref_locs if idx < top_layer else ref_locs[len(ref_locs) // 2:])
                sup_bnd_list.append((self.bound_box.yl, self.bound_box.yh) if idx == top_layer else None)
            else:
                ref_loc = self.grid.coord_to_track(idx, yb + self.get_tile_pinfo(0).height, RoundMode.NEAREST)
                sup_tid_list.append([TrackID(idx, ref_loc, _tr_w, grid=self.grid)])
                sup_bnd_list.append(None)

        vdd_list_list.append(self.connect_warr_to_tids_stack(vdd_xm, sup_tid_list, sup_bnd_list,
                                                             MinLenMode.MIDDLE, ret_wire=True))
        vdd_top, vss_top = self.match_warr_length(vdd_list_list[0][-1] + vss_list_list[0][-1])
        self.match_warr_length(vdd_list_list[0][-2] + vss_list_list[0][-2])

        self.add_pin('en', sam_ym)
        self.add_pin('enb', samb_ym)
        self.add_pin('ctrlp', dp_hm)
        self.add_pin('ctrln', dn_hm)
        self.add_pin('cap_botn', cap_bot_n_xm)
        self.add_pin('cap_botp', cap_bot_p_xm)
        [self.add_pin(pname, p) for pname, p in zip(['vrefn', 'vrefp', 'VSS', 'VDD'],
                                                    [vrefn_top, vrefp_top, vss_top, vdd_top])]

        self.sch_params = dict(
            w_dict=w_dict,
            # ndrv=True,
            lch=self.arr_info.lch,
            seg_dict=seg_dict,
            intent=self.get_row_info(0, 0).threshold
        )


class CapDrvCol(CapDrvCore):
    """A inverter with only transistors drawn, no metal connections
    """

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_drv')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = CapDrvCore.get_params_info()
        ans['ny'] = 'number of rows'
        ans['dum_row_idx'] = 'Index of dummy rows',
        ans['cm_row_idx'] = 'Index of common mode rows',
        ans.pop('is_cm')
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = CapDrvCore.get_default_param_values()
        ans['ny'] = 1
        ans['dum_row_idx'] = []
        ans['cm_row_idx'] = []
        ans.pop('is_cm')
        return ans

    def draw_layout(self):
        dum_row_idx: List[int] = self.params['dum_row_idx']
        cm_row_idx: List[int] = self.params['cm_row_idx']
        sw_type: str = self.params['sw_type']
        min_height: int = self.params['min_height']
        seg: int = self.params['seg']
        ny: int = self.params['ny']
        nx: int = self.params['nx']
        w: int = self.params['w']
        pinfo_dict = self.params['pinfo'].to_yaml()

        if min_height > 0:
            pinfo_dict['tile_specs']['place_info']['drv_tile']['min_height'] = min_height
        pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['mos_type'] = sw_type
        pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['width'] = w
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        pinfo0 = [TilePatternElement(pinfo[1]['drv_tile'])] * ny
        self.draw_base((TilePattern(pinfo0), pinfo[1]))

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        sw_unit = self.new_template(CapDrvCore, params=self.params)
        cm_params = dict(is_cm=True, **self.params)
        if cm_row_idx:
            cm_sw_unit = self.new_template(CapDrvCore, params=cm_params)
        else:
            cm_row_idx = None
        sw_list, cm_sw_list = [], []
        for idx in range(ny):
            if dum_row_idx and idx in dum_row_idx:
                continue
            if cm_row_idx and idx in cm_row_idx:
                cm_sw_list.append(self.add_tile(cm_sw_unit, tile_idx=idx, col_idx=0))
            else:
                sw_list.append(self.add_tile(sw_unit, tile_idx=idx, col_idx=0))

        self.set_mos_size(sw_unit.num_cols)

        top_layer = self.params['top_layer']
        for idx in range(xm_layer, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                _sup: List[List[WireArray]] = [inst.get_all_port_pins('VDD' if sw_type == 'pch' else 'VSS', layer=idx)
                                               for inst in sw_list + cm_sw_list]
                _sup_all = []
                [_sup_all.extend(wlist) for wlist in _sup]
                _sup_all = self.connect_wires(_sup_all)
                if idx == top_layer:
                    self.add_pin('VDD' if sw_type == 'pch' else 'VSS', _sup_all)
                for jdx in range(nx):
                    _vref: List[List[WireArray]] = \
                        [inst.get_all_port_pins(f'vref{jdx}', layer=idx) for inst in sw_list]
                    if jdx == nx - 1:
                        _vref.extend([inst.get_all_port_pins(f'vref0', layer=idx) for inst in cm_sw_list])
                    _vref_all = []
                    [_vref_all.extend(wlist) for wlist in _vref]
                    _vref_all = self.connect_wires(_vref_all)
                    if idx == top_layer:
                        self.add_pin(f'vref{jdx}', _vref_all)

        for idx in range(nx):
            self.add_pin(f'ctrl{idx}', [inst.get_pin(f'ctrl{idx}') for inst in sw_list])
        #
        [self.reexport(inst.get_port('cm_ctrl')) for inst in cm_sw_list]

        bot_list: List[WireArray] = [inst.get_pin('bot') for inst in sw_list + cm_sw_list]
        if sw_list[0].has_port('en'):
            [self.reexport(inst.get_port('en')) for inst in sw_list]
        if sw_list[0].has_port('enb'):
            [self.reexport(inst.get_port('enb')) for inst in sw_list]
        self.add_pin(f'bot', bot_list, mode=PinMode.UPPER)
        gated_list = [['XN', 'XM', 'XP'][k] for k in self.params['gated']]
        self.sch_params = dict(
            lch=self.arr_info.lch,
            w=w,
            seg=seg,
            gated_list=gated_list,
            intent=self.get_row_info(0, 0).threshold
        )


class CapDrvSplitCol(CapDrvSplitCore):
    """A inverter with only transistors drawn, no metal connections
    """

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_drv')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        ans = CapDrvSplitCore.get_params_info()
        ans['ny'] = 'number of rows'
        ans['dum_row_idx'] = 'Index of dummy rows',
        return ans

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        ans = CapDrvSplitCore.get_default_param_values()
        ans['ny'] = 1
        ans['dum_row_idx'] = []
        return ans

    def draw_layout(self):
        dum_row_idx: List[int] = self.params['dum_row_idx']
        min_height: int = self.params['min_height']
        ny: int = self.params['ny']
        pinfo_dict = self.params['pinfo'].to_yaml()

        if min_height > 0:
            pinfo_dict['tile_specs']['place_info']['drv_tile']['min_height'] = min_height
        # pinfo_dict['tile_specs']['place_info']['drv_tile']['row_specs'][0]['width'] = w
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, pinfo_dict)
        pinfo0 = [TilePatternElement(pinfo[1]['drv_tile'], flip=bool(idx & 1)) for idx in range(ny)]
        self.draw_base((TilePattern(pinfo0), pinfo[1]))

        tr_manager = self.tr_manager
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        sw_unit = self.new_template(CapDrvSplitCore, params=self.params)
        cm_params = dict(is_cm=True, **self.params)
        sw_list, cm_sw_list = [], []
        for idx in range(ny):
            if dum_row_idx and idx in dum_row_idx:
                continue
            else:
                sw_list.append(self.add_tile(sw_unit, tile_idx=idx, col_idx=0))

        self.set_mos_size(sw_unit.num_cols)

        top_layer = self.params['top_layer']
        for idx in range(xm_layer, top_layer + 1):
            lay_dir = self.grid.get_direction(idx)
            if lay_dir == Orient2D.y:
                _sup: List[List[WireArray]] = [inst.get_all_port_pins('VDD', layer=idx)
                                               for inst in sw_list + cm_sw_list]
                _sup_all = []
                [_sup_all.extend(wlist) for wlist in _sup]
                _sup_all = self.connect_wires(_sup_all)
                if idx == top_layer:
                    self.add_pin('VDD', _sup_all)

                _sup: List[List[WireArray]] = [inst.get_all_port_pins('VSS', layer=idx)
                                               for inst in sw_list + cm_sw_list]
                _sup_all = []
                [_sup_all.extend(wlist) for wlist in _sup]
                _sup_all = self.connect_wires(_sup_all)
                if idx == top_layer:
                    self.add_pin('VSS', _sup_all)

                for jdx in ['n', 'p']:
                    _vref: List[List[WireArray]] = \
                        [inst.get_all_port_pins(f'vref{jdx}', layer=idx) for inst in sw_list]
                    _vref_all = []
                    [_vref_all.extend(wlist) for wlist in _vref]
                    _vref_all = self.connect_wires(_vref_all)
                    if idx == top_layer:
                        self.add_pin(f'vref{jdx}', _vref_all)

        for idx in ['n', 'p']:
            self.add_pin(f'ctrl{idx}', [inst.get_pin(f'ctrl{idx}') for inst in sw_list])

        self.add_pin('sam', [inst.get_pin('en') for inst in sw_list])
        self.add_pin('samb', [inst.get_pin('enb') for inst in sw_list])
        bot_list: List[WireArray] = [inst.get_pin('cap_botn') for inst in sw_list + cm_sw_list]
        self.add_pin(f'cap_botn', bot_list, mode=PinMode.UPPER)
        bot_list: List[WireArray] = [inst.get_pin('cap_botp') for inst in sw_list + cm_sw_list]
        self.add_pin(f'cap_botp', bot_list, mode=PinMode.UPPER)
        self._sch_params = sw_unit.sch_params


class CapDacColCore(TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._actual_width = 0
        self._actual_width_sampler = 0

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cdac_array_bot')

    @property
    def actual_width(self) -> int:
        return self._actual_width

    @property
    def actual_width_sampler(self) -> int:
        return self._actual_width_sampler

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            fg_sp='reduce switches space',
            cap_sp_tr='space btw cap and switches',
            top_layer='Top supply, vin and reference layer',
            nbits='Number of bits',
            ny_list='list of ny',
            ratio_list='list of ratio',
            sw_type='switch type list',
            diff_idx='differential cap index',
            seg='segments dictionary.',
            sp='segments dictionary.',
            tot_seg='Total number of segment',
            w_n='widths dictionary.',
            w_p='widths dictionary.',
            cap_config='MOM cap configuration.',
            width='MOM cap width, in resolution units.',
            pinfo='placement information object.',
            remove_cap='True to remove capacitor, use it when doesnt have rmetal',
            lower_layer_routing='only use up to m4',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            sampler_params='Sampler_params',
            x_sw_cap_margin='Distance from switch(sampler) to capdac',
            top_sup_layer='Top supply layer',
            ctrl_vm_rt='True to route ctrl input to vm layer',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(
            fg_sp=0,
            cap_config={},
            width=0,
            w_n=4,
            w_p=4,
            w=4,
            remove_cap=False,
            lower_layer_routing=False,
            tot_seg=0,
            x_sw_cap_margin=0,
            ctrl_vm_rt=True,
        )

    def draw_layout(self) -> None:
        cap_config: ImmutableSortedDict[str, int] = self.params['cap_config']
        width: int = self.params['width']
        nbits: int = self.params['nbits']
        seg: int = self.params['seg']
        sp: int = self.params['sp']
        w_p: int = self.params['w_p']
        w_n: int = self.params['w_p']
        diff_idx: int = self.params['diff_idx']
        ny_list: List[int] = self.params['ny_list'].to_list()
        ratio_list: List[int] = self.params['ratio_list'].to_list()
        sw_type: List[str] = self.params['sw_type'].to_list()
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        top_sup_layer: int = self.params['top_sup_layer']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        grid = self.grid
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        has_cm_sw = True

        if nbits < 3:
            raise ValueError("[CDAC layout]: Less than 3-bit is not supported")

        ny_list = ny_list[diff_idx:][::-1] + ny_list
        ratio_list = ratio_list[diff_idx:][::-1] + ratio_list
        # bit_list = [1, 0, 2, 3] + list(range(diff_idx - 1, nbits + 1))
        bit_list = [1, 0] + list(range(2, diff_idx)) + list(range(diff_idx, nbits + 1))
        bit_list = bit_list[diff_idx:][::-1] + bit_list

        # Place control signals
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      self.params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # -- first track --
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')

        ctrl_tidx_start = grid.find_next_track(vm_layer, 0, tr_width=tr_w_ctrl_vm)
        ctrl_tidx_used, ctrl_tidx_locs = \
            tr_manager.place_wires(vm_layer, ['ctrl'] * (3 * nbits + 4), align_idx=0, align_track=ctrl_tidx_start)
        sw_x = self.grid.track_to_coord(vm_layer, ctrl_tidx_used)
        routing_bnd = sw_x
        ctrl_tidx_locs = ctrl_tidx_locs[0:]

        # Setup templates for size calculation

        dum_cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=4, ratio=8,
                                                                   tr_widths=tr_widths, tr_spaces=tr_spaces))
        cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=4 * sum(ny_list)))
        unit_cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=4))
        unit_cap_height = unit_cap_master.array_box.h
        w_cap, h_cap = cap_master.bound_box.w, cap_master.bound_box.h

        dum_cap_h = dum_cap_master.array_box.h
        h_cap = h_cap + 2 * dum_cap_h

        sw_params = dict(
            cls_name=CapDrvCol.get_qualified_name(),
            params=dict(pinfo=self.params['pinfo'], seg=seg, ny=sum(ny_list), w=w_n, sp=sp,
                        tot_seg=self.params['tot_seg'], top_layer=self.params['top_layer'],
                        dum_row_idx=[sum(ny_list[:nbits - diff_idx + 1]) + 1], min_height=unit_cap_height),
            export_private=False,
        )
        sw_master = self.new_template(GenericWrapper, params=sw_params)
        top_layer = max(cap_master.top_layer, sw_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer, half_blk_y=True)

        # Create and place sampler
        sampler_params = self.params['sampler_params'].to_dict()
        sampler_params['ny'] = sum(ny_list)
        sampler_params = dict(
            cls_name=SamplerVertGR.get_qualified_name(),
            params=sampler_params,
            export_private=False,
        )
        x_sw_cap_margin = self.params['x_sw_cap_margin']
        sampler_vert_temp = self.new_template(GenericWrapper, params=sampler_params)

        mid_sp_tr = self.params.get('cap_sp_tr', 6)
        mid_sp_coord = self.grid.get_track_info(cap_master.top_layer).pitch * mid_sp_tr
        w_blk_top, h_blk_top = self.grid.get_block_size(top_sup_layer, half_blk_x=True)

        # Create unit sampler template
        y_cm_sw_top = 0
        if sw_type.count('n') == 3:
            w_sw, h_sw = sw_master.bound_box.w, sw_master.bound_box.h
            sw_y = 0 if h_sw > h_cap else (h_cap - h_sw) // 2
            cap_y = 0 if h_cap > h_sw else (h_sw - h_cap) // 2
            sw_x = -(-sw_x // w_blk) * w_blk
            sw = self.add_instance(sw_master, inst_name='XSW',
                                   xform=Transform(sw_x, sw_y - sw_master.core.cap_align_ofst_y))
            sw_x += sw_master.bound_box.w
            sw_n, sw_p = None, None
            # Get sorted ctrl pins
            sw_ctrl_n: List[Union[WireArray, None]] = sw.get_all_port_pins('ctrl0')
            sw_ctrl_m: List[Union[WireArray, None]] = sw.get_all_port_pins('ctrl1')
            sw_ctrl_p: List[Union[WireArray, None]] = sw.get_all_port_pins('ctrl2')
            sw_bot: List[Union[WireArray, None]] = sw.get_all_port_pins('bot')
            self.reexport(sw.get_port('vref0'), net_name='vref<2>')
            self.reexport(sw.get_port('vref1'), net_name='vref<1>')
            self.reexport(sw.get_port('vref2'), net_name='vref<1>')
            # vrefm, vrefm_pin = sw.get_port('vref1'), sw.get_pin('vref1')
            sw_right_coord = sw.bound_box.xh
            sw_params_list = [sw_master.sch_params for _ in range(nbits)]
            # sw_vss_bbox: List[BBox] = sw.get_all_port_pins('VSS')
        elif sw_type.count('n') == 2:
            sw_n_params = dict(
                cls_name=CapDrvCol.get_qualified_name(),
                draw_taps=True,
                params=dict(pinfo=self.params['pinfo'], seg=seg, ny=sum(ny_list), w=w_n, sp=sp, nx=2, sw_type='nch',
                            tot_seg=self.params['tot_seg'], top_layer=self.params['top_layer'], gated=[1],
                            cm_row_idx=[sum(ny_list[:nbits - diff_idx + 1]) + 1], min_height=unit_cap_height),
                export_private=False,

            )
            sw_p_params = dict(
                cls_name=CapDrvCol.get_qualified_name(),
                draw_taps=True,
                params=dict(pinfo=self.params['pinfo'], seg=seg, ny=sum(ny_list), w=w_p, sp=sp, nx=1, sw_type='pch',
                            tot_seg=self.params['tot_seg'], top_layer=self.params['top_layer'],
                            dum_row_idx=[sum(ny_list[:nbits - diff_idx + 1]) + 1], min_height=unit_cap_height),
                export_private=False,
            )
            sw_n_master = self.new_template(GenericWrapper, params=sw_n_params)
            sw_p_master = self.new_template(GenericWrapper, params=sw_p_params)
            top_layer = max(cap_master.top_layer, sw_n_master.top_layer, sw_p_master.top_layer)
            w_blk, h_blk = self.grid.get_block_size(top_layer)
            w_sw_p, h_sw = sw_p_master.bound_box.w, sw_p_master.bound_box.h
            w_sw_n, h_sw = sw_n_master.bound_box.w, sw_n_master.bound_box.h
            sw_y = 0 if h_sw > h_cap else (h_cap - h_sw) // 2
            cap_y = 0 if h_cap > h_sw else (h_sw - h_cap) // 2
            sw_x = -(-sw_x // w_blk) * w_blk
            # Calculate total width, align to upper
            w_tot = sw_x + sw_n_master.bound_box.w + sw_p_master.bound_box.w + sampler_vert_temp.bound_box.w
            w_tot += w_cap + mid_sp_coord
            # round to sup top layer
            w_tot = -(-w_tot // w_blk_top) * w_blk_top

            cap_x = (w_tot - w_cap - mid_sp_coord) // w_blk * w_blk
            sam_x = cap_x - sampler_vert_temp.bound_box.w
            sam_x = sam_x // w_blk * w_blk
            sw_n_x = sam_x - sw_n_master.bound_box.w
            sw_n_x = sw_n_x // w_blk * w_blk
            sw_p_x = sw_n_x - sw_p_master.bound_box.w
            sw_p_x = sw_p_x // w_blk * w_blk

            sw_x = -(-(sw_n_x + w_sw_n) // w_blk) * w_blk
            sw_n = self.add_instance(sw_n_master, inst_name='XSWN', xform=Transform(sw_x, sw_y,
                                                                                    Orientation.MY))
            # hack to move switches together
            fg_sp = self.params['fg_sp'] * sw_n_master.core.arr_info.sd_pitch
            sw_x = -(-(sw_x - w_sw_p - w_sw_n - fg_sp) // w_blk) * w_blk
            sw_p = self.add_instance(sw_p_master, inst_name='XSWP', xform=Transform(sw_x, sw_y))
            sw_x += sw_n_master.bound_box.w
            # Get sorted ctrl pins
            sw_ctrl_n: List[Union[WireArray, None]] = sw_n.get_all_port_pins('ctrl0')
            sw_ctrl_m: List[Union[WireArray, None]] = sw_n.get_all_port_pins('ctrl1')
            sw_ctrl_p: List[Union[WireArray, None]] = sw_p.get_all_port_pins('ctrl0')
            # sw_bot: List[Union[WireArray, None]] = []
            sw_bot = self.connect_wires(sw_n.get_all_port_pins('bot') + sw_p.get_all_port_pins('bot'))

            if len(sw_bot) != sum(ny_list):
                sw_bot = [w for warr in sw_bot for w in warr.to_warr_list()]

            # for sw in sw_bot_all:
            #     sw_bot.extend(sw.to_warr_list())

            # vrefm, vrefm_pin = sw_n.get_port('vref0'), sw_n.get_pin('vref0')
            sw_right_coord = sw_n.bound_box.xh
            sw_type_dict = dict(
                XN=sw_type[0],
                XM=sw_type[1],
                XP=sw_type[2],
            )
            gated_list = ['XM']
            sw_params_list = [sw_n_master.sch_params.copy(append=dict(sw_type_dict=sw_type_dict,
                                                                      gated_list=gated_list)) for _ in range(nbits)]
        else:
            sw_n, sw_p = None, None
            raise NotImplementedError

        sampler_margin = 900
        sampler = self.add_instance(sampler_vert_temp, inst_name='XSAM',
                                    xform=Transform(sam_x, sw_n.bound_box.yl - sampler_margin - sw_master.core.cap_align_ofst_y))

        # cap_x = self.grid.track_to_coord(vm_layer, sig_tidx_locs[-1])

        h_tot = -(-sampler.bound_box.yh // h_blk_top) * h_blk_top
        self.set_size_from_bound_box(top_layer_id=top_sup_layer,
                                     bbox=BBox(0, 0, w_tot, h_tot))
        # cap_x = self.bound_box.xh - mid_sp_coord - w_cap
        cap_config_copy = copy.deepcopy(cap_config.to_dict())

        cap_list = []
        cap_master_list = [cap_master] * (nbits + 1)
        cap_y += sampler_margin//5
        dum_l = self.add_instance(dum_cap_master, inst_name='XCAP', xform=Transform(cap_x, -(-cap_y // h_blk) * h_blk))
        cap_y += dum_cap_master.array_box.h
        for idx, (ny, ratio) in enumerate(zip(ny_list, ratio_list)):
            cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config_copy, width=width, ny=4 * ny,
                                                                   ratio=ratio, tr_widths=tr_widths, ny_unit=4,
                                                                   tr_spaces=tr_spaces))
            cap_master_list[bit_list[idx]] = cap_master
            cap = self.add_instance(cap_master, inst_name='XCAP', xform=Transform(cap_x, -(-cap_y // h_blk) * h_blk))
            cap_list.append(cap)
            cap_y += cap_master.array_box.h

        dum_h = self.add_instance(dum_cap_master, inst_name='XCAP', xform=Transform(cap_x, -(-cap_y // h_blk) * h_blk))

        # cm_sw_y = -(-max(h_cap, h_sw) // h_blk) * h_blk
        ntr_margin = self.grid.get_sep_tracks(vm_layer, tr_manager.get_width(vm_layer, 'sup'),
                                              cap_list[0].get_pin('top').track_id.width)
        coord_margin = self.grid.track_to_coord(vm_layer, ntr_margin)
        # if self.params['lower_layer_routing']:
        #     cm_sw_x = cap_x - coord_margin
        #     cm_sw_x = -(-cm_sw_x // w_blk) * w_blk
        # else:
        #     cm_sw_x = cap_x

        # left space for clock routing
        num_tr, _ = tr_manager.place_wires(vm_layer, ['cap', 'clk', 'clk'], align_idx=0)
        coord_tr = self.grid.track_to_coord(vm_layer, num_tr)

        for pin_list in [sw_ctrl_m, sw_ctrl_n, sw_ctrl_p]:
            pin_list.sort(key=lambda x: x.track_id.base_index)

        # Get sorted bottom pin
        sw_bot.sort(key=lambda x: x.track_id.base_index)

        # Get cap dac pins
        cap_bot = [pin for inst in cap_list for pin in inst.get_all_port_pins('top')]
        cap_bot.sort(key=lambda x: x.track_id.base_index)
        for idx in [sum(ny_list[:nbits - diff_idx + 1]) + 1]:
            # sw_bot.insert(idx, None)
            sw_ctrl_m.insert(idx, None)
            sw_ctrl_n.insert(idx, None)
            sw_ctrl_p.insert(idx, None)

        # cap top
        cap_top = self.connect_wires([pin for inst in cap_list for pin in inst.get_all_port_pins('bot')])
        cap_unit_height = dum_cap_master.bound_box
        cap_top_xm1_tidx_list = [self.grid.coord_to_track(xm1_layer, (warr.upper + warr.lower) // 2, RoundMode.NEAREST)
                                 for warr in cap_bot]
        tr_w_cap_xm1 = tr_manager.get_width(cap_top[0].layer_id + 1, 'cap')
        tr_w_cap_ym1 = tr_manager.get_width(cap_top[0].layer_id + 2, 'cap')
        cap_top_xm1 = [self.connect_to_tracks(cap_top, TrackID(cap_top[0].layer_id + 1, tidx, tr_w_cap_xm1))
                       for tidx in cap_top_xm1_tidx_list]
        cap_top_ym1_tidx = self.grid.coord_to_track(cap_top[0].layer_id + 2, cap_top_xm1[0].middle,
                                                    RoundMode.GREATER_EQ)

        cap_top_ym1 = self.connect_to_tracks(cap_top_xm1, TrackID(ym1_layer, cap_top_ym1_tidx, tr_w_cap_ym1),
                                             track_upper=self.bound_box.yh)
        self.add_pin('top', cap_top_ym1, mode=PinMode.UPPER)
        self.add_pin('top', cap_top, mode=PinMode.UPPER)

        # Connect to common-mode switch
        tr_w_cap_hm = tr_manager.get_width(hm_layer, 'cap')
        tr_w_cap_vm = tr_manager.get_width(vm_layer, 'cap')
        tr_w_cap_xm = tr_manager.get_width(xm_layer, 'cap')

        # Connect switches to cap bot
        for idx, (_sw, _cap) in enumerate(zip(sw_bot, cap_bot)):
            self.connect_to_track_wires(_sw, sampler.get_all_port_pins(f'out<{idx}>'))
            _cap_vm_tidx = tr_manager.get_next_track(vm_layer, _cap.track_id.base_index, 'cap', 'cap', up=-1)
            _cap_vm = self.connect_to_tracks(_sw, TrackID(vm_layer, _cap_vm_tidx, tr_w_cap_vm, grid=self.grid))
            _cap_hm_tidx = self.grid.coord_to_track(hm_layer, _cap.middle, RoundMode.NEAREST)
            self.connect_to_tracks([_cap_vm, _cap], TrackID(hm_layer, _cap_hm_tidx, tr_w_cap_hm, grid=self.grid))
            # if _sw and _cap:
            #     self.connect_to_track_wires(_sw, _cap)

        # Group pins for each bit
        ctrl_bit_temp = dict(
            ctrl_m=[],
            ctrl_n=[],
            ctrl_p=[],
        )
        bit_pin_dict_list = [copy.deepcopy(ctrl_bit_temp) for _ in range(nbits)]
        bit_cap_list_list = [copy.deepcopy([]) for _ in range(nbits + 1)]

        for idx, bit_idx in enumerate(bit_list):
            start_idx, stop_idx = sum(ny_list[:idx]), sum(ny_list[:idx + 1])
            if bit_idx:
                bit_pin_dict_list[bit_idx - 1]['ctrl_m'].extend(sw_ctrl_m[start_idx: stop_idx])
                bit_pin_dict_list[bit_idx - 1]['ctrl_n'].extend(sw_ctrl_n[start_idx: stop_idx])
                bit_pin_dict_list[bit_idx - 1]['ctrl_p'].extend(sw_ctrl_p[start_idx: stop_idx])
            bit_cap_list_list[bit_idx].extend(sw_bot[start_idx: stop_idx])

        # Connect signal to vm-layer
        ctrl_hm_ret_list = []
        if self.params['ctrl_vm_rt']:
            ctrl_m_vm_list, ctrl_n_vm_list, ctrl_p_vm_list = [], [], []
            for idx in range(nbits):
                _bit_pins = bit_pin_dict_list[idx]
                ctrl_m_vm_list.append(self.connect_to_tracks(_bit_pins['ctrl_m'],
                                                             TrackID(vm_layer, ctrl_tidx_locs[3 * idx], tr_w_ctrl_vm),
                                                             track_lower=0, ret_wire_list=ctrl_hm_ret_list))
                ctrl_n_vm_list.append(self.connect_to_tracks(_bit_pins['ctrl_n'],
                                                             TrackID(vm_layer, ctrl_tidx_locs[3 * idx + 1],
                                                                     tr_w_ctrl_vm),
                                                             track_lower=0, ret_wire_list=ctrl_hm_ret_list))
                ctrl_p_vm_list.append(self.connect_to_tracks(_bit_pins['ctrl_p'],
                                                             TrackID(vm_layer, ctrl_tidx_locs[3 * idx + 2],
                                                                     tr_w_ctrl_vm),
                                                             track_lower=0, ret_wire_list=ctrl_hm_ret_list))
            for idx, (n, m, p) in enumerate(zip(ctrl_n_vm_list, ctrl_m_vm_list, ctrl_p_vm_list)):
                self.add_pin(f'ctrl_m<{idx}>', m, mode=PinMode.LOWER)
                self.add_pin(f'ctrl_n<{idx}>', n, mode=PinMode.LOWER)
                self.add_pin(f'ctrl_p<{idx}>', p, mode=PinMode.LOWER)
        else:
            for idx in range(nbits):
                _bit_pins = bit_pin_dict_list[idx]
                self.add_pin(f'ctrl_m<{idx}>', _bit_pins['ctrl_m'], mode=PinMode.LOWER, connect=True)
                self.add_pin(f'ctrl_n<{idx}>', _bit_pins['ctrl_n'], mode=PinMode.LOWER, connect=True)
                self.add_pin(f'ctrl_p<{idx}>', _bit_pins['ctrl_p'], mode=PinMode.LOWER, connect=True)
        # TODO: maybe not extend this
        # ctrl_hm_ret_min_coord, ctrl_hm_ret_max_coord = min([x.lower for x in ctrl_hm_ret_list]), \
        #                                                max([x.upper for x in ctrl_hm_ret_list])
        # self.extend_wires(ctrl_hm_ret_list, lower=ctrl_hm_ret_min_coord, upper=ctrl_hm_ret_max_coord)
        for idx in range(0, nbits + 1):
            if idx > 1 and len(bit_cap_list_list[idx]) > 1:
                for jdx, bit in enumerate(bit_cap_list_list[idx]):
                    self.add_pin(f'bot{idx - 1}<{jdx}>', bit)
            elif idx >= 1:
                self.add_pin(f'bot{idx - 1}', bit_cap_list_list[idx])
            else:
                self.add_pin('bot_cm', bit_cap_list_list[idx])

        # flip n and p control, just because comparator output and differential ...
        # TODO: maybe not extend this
        # ctrl_top_coord = max([c.upper for c in ctrl_n_vm_list + ctrl_p_vm_list + ctrl_m_vm_list])
        # ctrl_n_vm_list = self.extend_wires(ctrl_n_vm_list, upper=ctrl_top_coord)
        # ctrl_p_vm_list = self.extend_wires(ctrl_p_vm_list, upper=ctrl_top_coord)
        # ctrl_m_vm_list = self.extend_wires(ctrl_m_vm_list, upper=ctrl_top_coord)
        #

        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')
        cm_ctrl = sw_n.get_all_port_pins('cm_ctrl')
        cm_ctrl_vm_tidx = self.grid.coord_to_track(vm_layer, (sw_n.bound_box.xl + sw_p.bound_box.xh) // 2,
                                                   RoundMode.NEAREST)
        cm_ctrl_vm = self.connect_to_tracks(cm_ctrl, TrackID(vm_layer, cm_ctrl_vm_tidx, tr_w_ctrl_vm),
                                            track_upper=self.bound_box.yh)

        # ym_lay_purp = sw_n.get_port('VSS_ym').get_single_layer()
        #
        vss_top_list = sw_n.get_all_port_pins('VSS') + sampler.get_all_port_pins('VSS')
        vdd_top_list = sw_p.get_all_port_pins('VDD') + sampler.get_all_port_pins('VDD')
        top_layer = vss_top_list[0].layer_id
        top_layer_dir = self.grid.get_direction(top_layer)
        (block_top_layer_l, block_top_layer_h) = (
            self.bound_box.yl, self.bound_box.yh) if top_layer_dir == Orient2D.y else (
            self.bound_box.xl, self.bound_box.xh)

        vss_top_list = self.extend_wires(vss_top_list, lower=block_top_layer_l, upper=block_top_layer_h)
        vdd_top_list = self.extend_wires(vdd_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        vref0_top_list = sw_n.get_all_port_pins('vref0')
        vref0_top_list = self.extend_wires(vref0_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        vref1_top_list = sw_n.get_all_port_pins('vref1')
        vref1_top_list = self.extend_wires(vref1_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        vref2_top_list = sw_p.get_all_port_pins('vref0')
        vref2_top_list = self.extend_wires(vref2_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        self.connect_to_track_wires(dum_l.get_all_port_pins('top_xm')[0], sampler.get_all_port_pins('VSS_vm'))
        self.connect_to_track_wires(dum_h.get_all_port_pins('top_xm')[-1], sampler.get_all_port_pins('VSS_vm'))

        m_list = [len(_l) for _l in bit_cap_list_list[1:]]
        sw_list = m_list
        unit_params_list = [master.sch_params for master in cap_master_list[1:]]

        cm_cap_sw = sw_params_list[0].to_dict()
        cm_cap_sw['cm_sw'] = True
        sw_params_list.append(cm_cap_sw)

        self.connect_to_track_wires(sw_n.get_all_port_pins('enb'), cm_ctrl_vm)

        self.add_pin('samb', cm_ctrl_vm, mode=PinMode.UPPER)
        # self.add_pin('top', cap_top)

        # for pinname, ym1_list in zip(['vref<0>', 'vref<1>', 'vref<2>', 'VSS', 'VDD'],
        #                              sup_ref_ym1_list):
        #     self.add_pin(pinname, ym1_list)
        block_top_sup_list = [vss_top_list, vdd_top_list, vref0_top_list, vref1_top_list, vref2_top_list]
        if top_sup_layer > top_layer:
            sup_list_list = self.connect_supply_warr(tr_manager, block_top_sup_list, top_layer, self.bound_box)
            self.connect_supply_warr(tr_manager, sup_list_list, top_layer + 1, self.bound_box, up=False)
            for idx in range(top_layer + 1, top_sup_layer - 1):
                sup_list_list = self.connect_supply_warr(tr_manager, sup_list_list, idx, self.bound_box)

            vdd_top_list, vss_top_list = self.connect_supply_warr(tr_manager, sup_list_list[:2], top_sup_layer - 1,
                                                                  self.bound_box, side_sup=True)
            sup_list_list = [vdd_top_list, vss_top_list] + sup_list_list[2:]
        else:
            sup_list_list = block_top_sup_list

        self.add_pin('voff', self.extend_wires(sampler.get_all_port_pins('off'), upper=self.bound_box.yh),
                     mode=PinMode.UPPER)
        self.add_pin('vg', self.extend_wires(sampler.get_pin('vg'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('in', self.extend_wires(sampler.get_pin('vin'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('in_c', self.extend_wires(sampler.get_pin('vin_c'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        for warr, netname in zip(sup_list_list, ['VSS', 'VDD', 'vref<0>', 'vref<1>', 'vref<2>']):
            self.add_pin(netname, warr, connect=True)

        self._actual_width = self.bound_box.w - routing_bnd
        self._actual_width_sampler = self.bound_box.w - sam_x
        self.sch_params = dict(
            sw_params_list=sw_params_list,
            unit_params_list=unit_params_list,
            cm_unit_params=cap_master_list[0].sch_params,
            dum_params=dum_cap_master.sch_params,
            sampler_params=sampler_vert_temp.sch_params,
            bot_probe=True,
            cap_m_list=m_list,
            sw_m_list=sw_list,
            cm=ny_list[nbits - 1],
            remove_cap=self.params['remove_cap'],
        )


class CapDacSplitCore(TemplateBaseZL):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._actual_width = 0
        self._actual_width_sampler = 0

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cdac_array_split_bot')

    @property
    def actual_width(self) -> int:
        return self._actual_width

    @property
    def actual_width_sampler(self) -> int:
        return self._actual_width_sampler

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cap_sp_tr='space btw cap and switches',
            top_layer='Top supply, vin and reference layer',
            nbits='Number of bits',
            ny_list='list of ny',
            ratio_list='list of ratio',
            diff_idx='differential cap index',
            w_dict='widths dictionary.',
            seg_dict='segments dictionary.',
            tot_seg='Total number of segment',
            cap_config='MOM cap configuration.',
            width='MOM cap width, in resolution units.',
            pinfo='placement information object.',
            remove_cap='True to remove capacitor, use it when doesnt have rmetal',
            lower_layer_routing='only use up to m4',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
            sampler_params='Sampler_params',
            x_sw_cap_margin='Distance from switch(sampler) to capdac',
            top_sup_layer='Top supply layer',
            ctrl_vm_rt='True to route ctrl input to vm layer'
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(
            cap_config={},
            width=0,
            w_dict={},
            tot_seg=0,
            x_sw_cap_margin=0,
            remove_cap=False,
            lower_layer_routing=False,
            ctrl_vm_rt=True,
        )

    def draw_layout(self) -> None:
        # DAC driver params
        seg_dict: dict = self.params['seg_dict']
        w_dict: dict = self.params['w_dict']
        # Cap configuration params
        cap_config: ImmutableSortedDict[str, int] = self.params['cap_config']
        width: int = self.params['width']
        nbits: int = self.params['nbits']
        diff_idx: int = self.params['diff_idx']
        ny_list: List[int] = self.params['ny_list'].to_list()
        ratio_list: List[int] = self.params['ratio_list'].to_list()
        top_sup_layer: int = self.params['top_sup_layer']

        # Track settings
        grid = self.grid
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

        if nbits < 3:
            raise ValueError("[CDAC layout]: Less than 3-bit is not supported")

        ny_list = ny_list[diff_idx:][::-1] + ny_list
        ratio_list = ratio_list[diff_idx:][::-1] + ratio_list
        bit_list = list(range(1, diff_idx)) + list(range(diff_idx, nbits + 1))
        bit_list = bit_list[diff_idx:][::-1] + bit_list

        # Place control signals
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      self.params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # -- first track --
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')

        # Calculate ctrl signal routing space
        ctrl_tidx_start = grid.find_next_track(vm_layer, 0, tr_width=tr_w_ctrl_vm)
        ctrl_tidx_used, ctrl_tidx_locs = tr_manager.place_wires(vm_layer, ['ctrl'] * (2 * nbits + 3),
                                                                align_idx=0, align_track=ctrl_tidx_start)
        sw_x = self.grid.track_to_coord(vm_layer, ctrl_tidx_used)
        ctrl_tidx_locs = ctrl_tidx_locs[0:]

        # Setup templates for size calculation
        dum_cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=3, ratio=8,
                                                                   tr_widths=tr_widths, tr_spaces=tr_spaces))
        # -- temp cap template for floorplaning
        cap_master = self.new_template(CapColCore,
                                       params=dict(cap_config=cap_config, width=width, ny=2 * 3 * sum(ny_list)))
        unit_cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=3))
        unit_cap_height = unit_cap_master.array_box.h

        # -- total cap array height
        w_cap, h_cap = cap_master.bound_box.w, cap_master.bound_box.h
        dum_cap_h = dum_cap_master.array_box.h
        h_cap = h_cap + 2 * dum_cap_h

        # Create sampler template
        sampler_params = self.params['sampler_params'].to_dict()
        sampler_params['ny'] = 2 * sum(ny_list)
        sampler_params = dict(
            cls_name=SamplerVertGR.get_qualified_name(),
            params=sampler_params,
            export_private=False,
        )
        sampler_temp = self.new_template(GenericWrapper, params=sampler_params)

        # -- this adjust the space btw sampler and cap arrray
        mid_sp_tr = self.params.get('cap_sp_tr', 6)
        mid_sp_coord = self.grid.get_track_info(cap_master.top_layer).pitch * mid_sp_tr
        w_blk_top, h_blk_top = self.grid.get_block_size(top_sup_layer, half_blk_x=False)

        # Create unit sampler template
        y_cm_sw_top = 0
        sw_params = dict(
            cls_name=CapDrvSplitCol.get_qualified_name(),
            draw_taps=True,
            params=dict(pinfo=self.params['pinfo'], seg_dict=seg_dict, ny=sum(ny_list), w_dict=w_dict, nx=2,
                        tot_seg=self.params['tot_seg'], top_layer=self.params['top_layer'],
                        min_height=unit_cap_height),
            export_private=False,
        )
        sw_master = self.new_template(GenericWrapper, params=sw_params)
        top_layer = max(cap_master.top_layer, sw_master.top_layer, sampler_temp.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)
        w_sw, h_sw = sw_master.bound_box.w, sw_master.bound_box.h
        sw_y = 0 if h_sw > h_cap else (h_cap - h_sw) // 2
        sw_y -= 90
        cap_y = 0 if h_cap > h_sw else (h_sw - h_cap) // 2
        cap_y -= 90
        sw_x = -(-sw_x // w_blk) * w_blk

        # Calculate total width, align to upper
        w_tot = sw_x + sw_master.bound_box.w + sampler_temp.bound_box.w + w_cap + mid_sp_coord
        # round to sup top layer
        w_blk_top = tr_manager.get_sep(top_sup_layer, ('sup', 'sup')).dbl_value * self.grid.get_track_pitch(11)
        w_tot = (w_tot // w_blk_top) * w_blk_top + w_blk_top // 2

        # backward calculation of x coords
        cap_x = (w_tot - w_cap - mid_sp_coord) // w_blk * w_blk
        sam_x = cap_x - sampler_temp.bound_box.w
        sam_x = sam_x // w_blk * w_blk
        sw_x = sam_x - sw_master.bound_box.w
        sw_x = sw_x // w_blk * w_blk

        # ==  place switches
        routing_bnd = sw_x
        sw = self.add_instance(sw_master, inst_name='XSWN', xform=Transform(sw_x, sw_y, Orientation.R0))
        # ==  place sampler
        sampler = self.add_instance(sampler_temp, inst_name='XSAM',
                                    xform=Transform(sam_x, sw.bound_box.yl - 1080 - sw_master.core.cap_align_ofst_y))
        # Get sorted ctrl pins
        sw_ctrl_n: List[Union[WireArray, None]] = sw.get_all_port_pins('ctrln')
        sw_ctrl_p: List[Union[WireArray, None]] = sw.get_all_port_pins('ctrlp')
        sw_bot_n = self.connect_wires(sw.get_all_port_pins('cap_botn'))
        sw_bot_p = self.connect_wires(sw.get_all_port_pins('cap_botp'))

        if len(sw_bot_n) != sum(ny_list):
            sw_bot_n = [w for warr in sw_bot_n for w in warr.to_warr_list()]
            sw_bot_p = [w for warr in sw_bot_p for w in warr.to_warr_list()]

        for pin_list in [sw_ctrl_n, sw_ctrl_p]:
            pin_list.sort(key=lambda x: x.track_id.base_index)

        # Get sorted bottom pin
        sw_bot_n.sort(key=lambda x: x.track_id.base_index)
        sw_bot_p.sort(key=lambda x: x.track_id.base_index)
        sw_params_list = [sw_master.sch_params.copy(append=dict(seg_dict=seg_dict, w_dict=w_dict))
                          for _ in range(nbits)]

        # height is limited by sampler bc the guardring
        h_tot = -(-sampler.bound_box.yh // h_blk_top) * h_blk_top
        self.set_size_from_bound_box(top_layer_id=top_sup_layer,
                                     bbox=BBox(0, 0, w_tot, h_tot))

        # == place capacitor array
        cap_config_copy = copy.deepcopy(cap_config.to_dict())
        cap_list = []
        cap_master_list = [cap_master] * (nbits + 1)
        dum_l = self.add_instance(dum_cap_master, inst_name='XCAP', xform=Transform(cap_x, cap_y))
        cap_y += dum_cap_master.array_box.h
        for idx, (ny, ratio) in enumerate(zip(ny_list, ratio_list)):
            cap_master = self.new_template(CapColCore,
                                           params=dict(cap_config=cap_config_copy, width=width, ny=2 * 3 * ny,
                                                       ratio=ratio, tr_widths=tr_widths, ny_unit=3,
                                                       tr_spaces=tr_spaces))
            cap_master_list[bit_list[idx]] = cap_master
            cap = self.add_instance(cap_master, inst_name='XCAP', xform=Transform(cap_x, cap_y))
            cap_list.append(cap)
            cap_y += cap_master.array_box.h

        dum_h = self.add_instance(dum_cap_master, inst_name='XCAP', xform=Transform(cap_x, cap_y))
        # Basefill
        # -- under cap
        # edge margin
        mosbase_margin = sw_master.bound_box.w - sw_master.core_bound_box.w
        num_cols_cap_basefill = (cap_master.bound_box.w - mosbase_margin) // sw_master.core.sd_pitch
        cap_basefill_pinfo = sw_master.core.draw_base_info
        cap_basefill_master = self.new_template(GenericWrapper,
                                                params=dict(cls_name=MOSBaseFiller.get_qualified_name(),
                                                            params=dict(pinfo=cap_basefill_pinfo,
                                                                        num_cols=num_cols_cap_basefill),
                                                            top_layer=self.params['top_layer']))

        self.add_instance(cap_basefill_master, inst_name='XCAPFILL', xform=Transform(cap_x, sw_y))
        # -- left margin
        num_cols_left_basefill = (sw_x - mosbase_margin) // sw_master.core.sd_pitch
        left_basefill_pinfo = sw_master.core.draw_base_info
        left_basefill_master = self.new_template(GenericWrapper,
                                                 params=dict(cls_name=MOSBaseFiller.get_qualified_name(),
                                                             params=dict(pinfo=left_basefill_pinfo,
                                                                         num_cols=num_cols_left_basefill),
                                                             top_layer=self.params['top_layer']))

        self.add_instance(left_basefill_master, inst_name='XCAPFILL', xform=Transform(sw_x, sw_y,
                                                                                      Orientation.MY))
        # Connect to common-mode switch
        tr_w_cap_hm = tr_manager.get_width(hm_layer, 'cap')
        tr_w_cap_vm = tr_manager.get_width(vm_layer, 'cap')

        # Get cap dac pins
        cap_bot = [pin for inst in cap_list for pin in inst.get_all_port_pins('top')]
        cap_bot.sort(key=lambda x: x.track_id.base_index)

        # Connect switches to cap bot
        for idx, _sw in enumerate(sw_bot_n):
            _cap = cap_bot[2 * idx + 1 if idx & 1 else 2 * idx]
            self.connect_to_track_wires(_sw, sampler.get_all_port_pins(f'out<{2 * idx + 1}>' if idx & 1 else
                                                                       f'out<{2 * idx}>'))
            _cap_vm_tidx = tr_manager.get_next_track(vm_layer, _cap.track_id.base_index, 'cap', 'cap', up=-1)
            _cap_vm = self.connect_to_tracks(_sw, TrackID(vm_layer, _cap_vm_tidx, tr_w_cap_vm, grid=self.grid))
            _cap_hm_tidx = self.grid.coord_to_track(hm_layer, _cap.middle, RoundMode.NEAREST)
            self.connect_to_tracks([_cap_vm, _cap], TrackID(hm_layer, _cap_hm_tidx, tr_w_cap_hm, grid=self.grid))
        for idx, _sw in enumerate(sw_bot_p):
            _cap = cap_bot[2 * idx if idx & 1 else 2 * idx + 1]
            self.connect_to_track_wires(_sw, sampler.get_all_port_pins(f'out<{2 * idx}>' if idx & 1 else
                                                                       f'out<{2 * idx + 1}>'))
            _cap_vm_tidx = tr_manager.get_next_track(vm_layer, _cap.track_id.base_index, 'cap', 'cap', up=-1)
            _cap_vm = self.connect_to_tracks(_sw, TrackID(vm_layer, _cap_vm_tidx, tr_w_cap_vm, grid=self.grid))
            _cap_hm_tidx = self.grid.coord_to_track(hm_layer, _cap.middle, RoundMode.NEAREST)
            self.connect_to_tracks([_cap_vm, _cap], TrackID(hm_layer, _cap_hm_tidx, tr_w_cap_hm, grid=self.grid))

        # Group pins for each bit
        ctrl_bit_temp = dict(ctrl_n=[], ctrl_p=[])
        bit_pin_dict_list = [copy.deepcopy(ctrl_bit_temp) for _ in range(nbits)]
        bit_cap_n_list_list = [copy.deepcopy([]) for _ in range(nbits + 1)]
        bit_cap_p_list_list = [copy.deepcopy([]) for _ in range(nbits + 1)]

        for idx, bit_idx in enumerate(bit_list):
            start_idx, stop_idx = sum(ny_list[:idx]), sum(ny_list[:idx + 1])
            if bit_idx:
                bit_pin_dict_list[bit_idx - 1]['ctrl_n'].extend(sw_ctrl_n[start_idx: stop_idx])
                bit_pin_dict_list[bit_idx - 1]['ctrl_p'].extend(sw_ctrl_p[start_idx: stop_idx])
            bit_cap_n_list_list[bit_idx].extend(sw_bot_n[start_idx: stop_idx])
            bit_cap_p_list_list[bit_idx].extend(sw_bot_p[start_idx: stop_idx])

        # Connect signal to vm-layer
        ctrl_hm_ret_list = []
        if self.params['ctrl_vm_rt']:
            ctrl_n_vm_list, ctrl_p_vm_list = [], []
            for idx in range(nbits):
                _bit_pins = bit_pin_dict_list[idx]
                ctrl_n_vm_list.append(self.connect_to_tracks(_bit_pins['ctrl_n'],
                                                             TrackID(vm_layer, ctrl_tidx_locs[2 * idx + 1],
                                                                     tr_w_ctrl_vm),
                                                             track_lower=0, ret_wire_list=ctrl_hm_ret_list))
                ctrl_p_vm_list.append(self.connect_to_tracks(_bit_pins['ctrl_p'],
                                                             TrackID(vm_layer, ctrl_tidx_locs[2 * idx + 2],
                                                                     tr_w_ctrl_vm),
                                                             track_lower=0, ret_wire_list=ctrl_hm_ret_list))

            for idx, (n, p) in enumerate(zip(ctrl_n_vm_list, ctrl_p_vm_list)):
                self.add_pin(f'ctrl_n<{idx}>', n, mode=PinMode.LOWER)
                self.add_pin(f'ctrl_p<{idx}>', p, mode=PinMode.LOWER)
        else:
            for idx in range(nbits):
                _bit_pins = bit_pin_dict_list[idx]
                _ctrl_n = self.extend_wires(_bit_pins['ctrl_n'], lower=sw.bound_box.xl)
                _ctrl_p = self.extend_wires(_bit_pins['ctrl_p'], lower=sw.bound_box.xl)
                self.add_pin(f'ctrl_n<{idx}>', _ctrl_n, mode=PinMode.LOWER, connect=True)
                self.add_pin(f'ctrl_p<{idx}>', _ctrl_p, mode=PinMode.LOWER, connect=True)
        cap_bot_idx = 0
        for idx in range(1, nbits + 1):
            if idx > 1 and len(bit_cap_n_list_list[idx]) > 1:
                for jdx, bit in enumerate(bit_cap_n_list_list[idx]):
                    self.add_pin(f'botn<{jdx + cap_bot_idx}>', bit)
                for jdx, bit in enumerate(bit_cap_p_list_list[idx]):
                    self.add_pin(f'botp<{jdx + cap_bot_idx}>', bit)
                cap_bot_idx += len(bit_cap_n_list_list[idx])
            else:
                self.add_pin(f'botn<{cap_bot_idx}>', bit_cap_n_list_list[idx])
                self.add_pin(f'botp<{cap_bot_idx}>', bit_cap_p_list_list[idx])
                cap_bot_idx += 1

        # cap top
        cap_top = self.connect_wires([pin for inst in cap_list for pin in inst.get_all_port_pins('bot')])
        cap_top_xm1_tidx_list = [self.grid.coord_to_track(xm1_layer, (warr.upper + warr.lower) // 2, RoundMode.NEAREST)
                                 for warr in cap_bot]
        tr_w_cap_xm1 = tr_manager.get_width(cap_top[0].layer_id + 1, 'cap')
        tr_w_cap_ym1 = tr_manager.get_width(cap_top[0].layer_id + 2, 'cap')
        cap_top_xm1 = [self.connect_to_tracks(cap_top, TrackID(cap_top[0].layer_id + 1, tidx, tr_w_cap_xm1))
                       for tidx in cap_top_xm1_tidx_list]
        cap_top_ym1_tidx = self.grid.coord_to_track(cap_top[0].layer_id + 2, cap_top_xm1[0].middle,
                                                    RoundMode.GREATER)

        cap_top_ym1 = self.connect_to_tracks(cap_top_xm1, TrackID(ym1_layer, cap_top_ym1_tidx, tr_w_cap_ym1),
                                             track_upper=self.bound_box.yh)
        self.add_pin('top', cap_top_ym1, mode=PinMode.UPPER)
        self.add_pin('top', cap_top, mode=PinMode.UPPER)

        # Connect references and supply
        vss_top_list = sw.get_all_port_pins('VSS') + sampler.get_all_port_pins('VSS')
        vdd_top_list = sw.get_all_port_pins('VDD') + sampler.get_all_port_pins('VDD')
        top_layer = vss_top_list[0].layer_id
        top_layer_dir = self.grid.get_direction(top_layer)
        (block_top_layer_l, block_top_layer_h) = (
            self.bound_box.yl, self.bound_box.yh) if top_layer_dir == Orient2D.y else (
            self.bound_box.xl, self.bound_box.xh)

        vss_top_list = self.extend_wires(vss_top_list, lower=block_top_layer_l, upper=block_top_layer_h)
        vdd_top_list = self.extend_wires(vdd_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        vrefn_top_list = sw.get_all_port_pins('vrefn')
        vrefn_top_list = self.extend_wires(vrefn_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        vrefp_top_list = sw.get_all_port_pins('vrefp')
        vrefp_top_list = self.extend_wires(vrefp_top_list, lower=block_top_layer_l, upper=block_top_layer_h)

        self.connect_to_track_wires(dum_l.get_all_port_pins('top_xm')[-1], sampler.get_all_port_pins('VSS_vm'))
        self.connect_to_track_wires(dum_h.get_all_port_pins('top_xm')[0], sampler.get_all_port_pins('VSS_vm'))

        m_list = [len(_l) for _l in bit_cap_n_list_list[1:]]
        sw_list = m_list
        unit_params_list = [master.sch_params for master in cap_master_list[1:]]

        block_top_sup_list = [vss_top_list, vdd_top_list, vrefn_top_list, vrefp_top_list]
        if top_sup_layer > top_layer:
            sup_list_list = self.connect_supply_warr(tr_manager, block_top_sup_list, top_layer, self.bound_box)
            self.connect_supply_warr(tr_manager, sup_list_list, top_layer + 1, self.bound_box, up=False)
            for idx in range(top_layer + 1, top_sup_layer - 1):
                sup_list_list = self.connect_supply_warr(tr_manager, sup_list_list, idx, self.bound_box)

            vdd_top_list, vss_top_list = self.connect_supply_warr(tr_manager, sup_list_list[:2], top_sup_layer - 1,
                                                                  self.bound_box, side_sup=True)
            sup_list_list = [vdd_top_list, vss_top_list] + sup_list_list[2:]
        else:
            sup_list_list = block_top_sup_list

        sam = self.connect_wires(sw.get_all_port_pins('sam'))
        samb = self.connect_wires(sw.get_all_port_pins('samb'))
        self.add_pin('voff', self.extend_wires(sampler.get_all_port_pins('off'), upper=self.bound_box.yh),
                     mode=PinMode.UPPER)
        self.add_pin('vg', self.extend_wires(sampler.get_pin('vg'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('in', self.extend_wires(sampler.get_pin('vin'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('in_c', self.extend_wires(sampler.get_pin('vin_c'), upper=self.bound_box.yh), mode=PinMode.UPPER)
        self.add_pin('sam', sam, mode=PinMode.UPPER)
        self.add_pin('samb', samb, mode=PinMode.UPPER)
        for warr, netname in zip(sup_list_list, ['VSS', 'VDD', 'vref<0>', 'vref<1>']):
            self.add_pin(netname, warr, connect=True)

        self._actual_width = self.bound_box.w - routing_bnd
        self._actual_width_sampler = self.bound_box.w - sam_x
        self.sch_params = dict(
            sw_params_list=sw_params_list,
            unit_params_list=unit_params_list,
            dum_params=dum_cap_master.sch_params,
            sampler_params=sampler_temp.sch_params,
            bot_probe=True,
            cap_m_list=m_list,
            sw_m_list=sw_list,
            cm=ny_list[nbits - 1],
            remove_cap=self.params['remove_cap'],
        )


class CapDacCapCore(TemplateBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        self._actual_width = 0

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cdac_cap')

    @property
    def actual_width(self) -> int:
        return self._actual_width

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            nbits='Number of bits',
            ny_list='list of ny',
            ratio_list='list of ratio',
            sw_type='switch type list',
            diff_idx='differential cap index',
            seg='segments dictionary.',
            seg_cm='segments dictionary.',
            sp='segments dictionary.',
            tot_seg='Total number of segment',
            w_n='widths dictionary.',
            w_p='widths dictionary.',
            w_cm='widths dictionary.',
            cap_config='MOM cap configuration.',
            width='MOM cap width, in resolution units.',
            pinfo='placement information object.',
            pinfo_cm='placement information object.',
            remove_cap='True to remove capacitor, use it when doesnt have rmetal',
            lower_layer_routing='only use up to m4',
            tr_widths='Track width dictionary',
            tr_spaces='Track space dictionary',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(
            cap_config={},
            width=0,
            w_n=4,
            w_p=4,
            w=4,
            remove_cap=False,
            lower_layer_routing=False,
            tot_seg=0
        )

    def draw_layout(self) -> None:
        cap_config: ImmutableSortedDict[str, int] = self.params['cap_config']
        width: int = self.params['width']
        nbits: int = self.params['nbits']
        seg: int = self.params['seg']
        sp: int = self.params['sp']
        w_p: int = self.params['w_p']
        w_n: int = self.params['w_p']
        seg_cm: int = self.params['seg_cm']
        w_cm: int = self.params['w_cm']
        diff_idx: int = self.params['diff_idx']
        ny_list: List[int] = self.params['ny_list'].to_list()
        ratio_list: List[int] = self.params['ratio_list'].to_list()
        sw_type: List[str] = self.params['sw_type'].to_list()
        tr_widths: Dict[str, Any] = self.params['tr_widths']
        tr_spaces: Mapping[Tuple[str, str], Mapping[int, Union[float, HalfInt]]] = self.params['tr_spaces']
        grid = self.grid
        tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)
        has_cm_sw = True

        if nbits < 3:
            raise ValueError("[CDAC layout]: Less than 3-bit is not supported")

        ny_list = ny_list[diff_idx:][::-1] + ny_list
        ratio_list = ratio_list[diff_idx:][::-1] + ratio_list
        # bit_list = [1, 0, 2, 3] + list(range(diff_idx - 1, nbits + 1))
        bit_list = [1, 0] + list(range(diff_idx, nbits + 1))
        bit_list = bit_list[diff_idx:][::-1] + bit_list

        # Place control signals
        conn_layer = MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info,
                                                      self.params['pinfo']['tile_specs']['arr_info']['lch'])
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        # -- first track --
        tr_w_ctrl_vm = tr_manager.get_width(vm_layer, 'ctrl')

        ctrl_tidx_start = grid.find_next_track(vm_layer, 0, tr_width=tr_w_ctrl_vm)
        ctrl_tidx_used, ctrl_tidx_locs = \
            tr_manager.place_wires(vm_layer, ['ctrl'] * (3 * nbits + 1), align_idx=0, align_track=ctrl_tidx_start)
        sw_x = self.grid.track_to_coord(vm_layer, ctrl_tidx_used)
        routing_bnd = sw_x

        # Setup templates for size calculation

        cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=4 * sum(ny_list)))
        unit_cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config, width=width, ny=4))
        unit_cap_height = unit_cap_master.array_box.h
        w_cap, h_cap = cap_master.bound_box.w, cap_master.bound_box.h

        sw_params = dict(
            cls_name=CapDrvCol.get_qualified_name(),
            params=dict(pinfo=self.params['pinfo'], seg=seg, ny=sum(ny_list), w=w_n, sp=sp,
                        tot_seg=self.params['tot_seg'],
                        dum_row_idx=[sum(ny_list[:nbits - diff_idx + 1]) + 1], min_height=unit_cap_height),
            export_private=True,
        )
        sw_master = self.new_template(GenericWrapper, params=sw_params)
        top_layer = max(cap_master.top_layer, sw_master.top_layer)
        w_blk, h_blk = self.grid.get_block_size(top_layer)

        # y_cm_sw_top = -(-cm_sw_master.bound_box.h // h_blk) * h_blk

        # Place input signal
        tr_w_sig_vm = tr_manager.get_width(vm_layer, 'sig')
        sig_tidx_start = grid.find_next_track(vm_layer, 0, tr_width=tr_w_sig_vm)
        sig_tidx_used, sig_tidx_locs = tr_manager.place_wires(vm_layer, ['sig'] * (nbits + 1) + ['cap'], align_idx=0,
                                                              align_track=sig_tidx_start)

        cap_x = self.grid.track_to_coord(vm_layer, sig_tidx_locs[-1])

        cap_x = -(-cap_x // w_blk) * w_blk
        cap_y = 0
        cap_config_copy = copy.deepcopy(cap_config.to_dict())

        cap_list = []
        cap_master_list = [cap_master] * (nbits + 1)
        for idx, (ny, ratio) in enumerate(zip(ny_list, ratio_list)):
            cap_master = self.new_template(CapColCore, params=dict(cap_config=cap_config_copy, width=width, ny=4 * ny,
                                                                   ratio=ratio, tr_widths=tr_widths,
                                                                   tr_spaces=tr_spaces))
            cap_master_list[bit_list[idx]] = cap_master
            cap = self.add_instance(cap_master, inst_name='XCAP', xform=Transform(cap_x, -(-cap_y // h_blk) * h_blk))
            cap_list.append(cap)
            cap_y += cap_master.array_box.h

        # cm_sw_y = -(-max(h_cap, h_sw) // h_blk) * h_blk
        ntr_margin = self.grid.get_sep_tracks(vm_layer, tr_manager.get_width(vm_layer, 'sup'),
                                              cap_list[0].get_pin('top').track_id.width)
        coord_margin = self.grid.track_to_coord(vm_layer, ntr_margin)
        if self.params['lower_layer_routing']:
            cm_sw_x = cap_x - coord_margin
            cm_sw_x = -(-cm_sw_x // w_blk) * w_blk
        else:
            cm_sw_x = cap_x

        # left space for clock routing
        num_tr, _ = tr_manager.place_wires(vm_layer, ['cap', 'clk', 'clk'], align_idx=0)
        coord_tr = self.grid.track_to_coord(vm_layer, num_tr)

        w_blk, h_blk = self.grid.get_block_size(ym1_layer)
        w_tot = -(-(cap_x + w_cap + coord_tr) // w_blk) * w_blk
        h_tot = -(-cap.bound_box.yh // h_blk) * h_blk
        self.set_size_from_bound_box(top_layer_id=ym1_layer, bbox=BBox(0, 0, w_tot, h_tot))

        # Get cap dac pins
        cap_bot = [pin for inst in cap_list for pin in inst.get_all_port_pins('top')]
        cap_bot.sort(key=lambda x: x.track_id.base_index)

        # cap top
        cap_top = self.connect_wires([pin for inst in cap_list for pin in inst.get_all_port_pins('bot')])

        bit_cap_list_list = [copy.deepcopy([]) for _ in range(nbits + 1)]
        cap_bot = [pin for inst in cap_list for pin in inst.get_all_port_pins('top')]
        cap_bot_hm = []
        for _bot in cap_bot:
            hm_tidx = self.grid.coord_to_track(hm_layer, _bot.middle, RoundMode.NEAREST)
            cap_bot_hm.append(self.connect_to_tracks(_bot, TrackID(hm_layer, hm_tidx)))
        cap_bot_hm.sort(key=lambda x: x.track_id.base_index)

        for idx, bit_idx in enumerate(bit_list):
            start_idx, stop_idx = sum(ny_list[:idx]), sum(ny_list[:idx + 1])
            bit_cap_list_list[bit_idx].extend(cap_bot_hm[start_idx: stop_idx])

        bot_vm_list: List[WireArray] = []
        for idx in range(0, nbits + 1):
            if idx:
                self.add_pin(f'bot<{idx - 1}>', bit_cap_list_list[idx], connect=False)
            else:
                self.add_pin('bot_cm', bit_cap_list_list[idx])
            # bot_vm_list.append(self.connect_to_tracks(bit_cap_list_list[idx],
            #                                           TrackID(vm_layer, sig_tidx_locs[idx], tr_w_sig_vm),
            #                                           track_upper=self.bound_box.yh))
        # bot_vm_list_bot_coord = self.bound_box.yl
        # bot_vm_list = self.extend_wires(bot_vm_list, lower=bot_vm_list_bot_coord)
        # for idx, bot_wire in enumerate(bot_vm_list[1:]):
        #     self.add_pin(f'bot<{idx}>', bot_wire, mode=PinMode.UPPER)
        # self.add_pin('bot_cm', bot_vm_list[0], mode=PinMode.UPPER)

        m_list = [len(_l) for _l in bit_cap_list_list[1:]]
        unit_params_list = [master.sch_params for master in cap_master_list[1:]]

        # tech_global = get_tech_global_info('bag3_digital')
        # pinfo = dict(
        #     lch=tech_global['lch_min'],
        #     top_layer=MOSArrayPlaceInfo.get_conn_layer(self.grid.tech_info, tech_global['lch_min']) + 1,
        #     tr_widths={},
        #     tr_spaces={},
        #     row_specs=[dict(mos_type='ptap', width=tech_global['w_minn'], threshold='standard',
        #                     bot_wires=['sup'], top_wires=[])]
        # )
        # tap_master = self.new_template(CapTap, params=dict(pinfo=pinfo))
        # tap = self.add_instance(tap_master, xform=Transform(tap_master.bound_box.w, 0, Orientation.R0))
        # self.reexport(tap.get_port('VSS'))
        # #
        self.add_pin('top', cap_top)
        self.sch_params = dict(
            unit_params_list=unit_params_list,
            cm_unit_params=cap_master_list[0].sch_params,
            bot_probe=True,
            cap_m_list=m_list,
            cm=ny_list[nbits - 1],
            remove_cap=self.params['remove_cap'],
        )
