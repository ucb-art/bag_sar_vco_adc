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


from typing import Any, Dict, Type, Optional, Mapping, List, Tuple

from bag.design.database import ModuleDB
from bag.design.module import Module
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict, ImmutableList
from bag_vco_adc.layout.sar.sar_cdac import CapColCore
from bag_vco_adc.layout.util.util import fill_conn_layer_intv, export_xm_sup, fill_tap_intv
from pybag.core import Transform, BBoxArray, BBox
from pybag.enum import Orientation, RoundMode, Direction
from xbase.layout.enum import MOSWireType, MOSType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.placement.data import TilePatternElement, TilePattern
from ..digital import PassGateCore, InvChainCore
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID


class MOSCap(MOSBase):
    """ transistor cap between gate and drain/source
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'mos_cap')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            nrow='segments dictionary.',
            seg='Optional dictionary of user defined signal locations',
            w='Optional dictionary of user defined signal locations',
            dev_type='nch or pch',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            nrow=1,
            seg=4,
            dev_type='nch',
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        tile_elements = place_info[1]
        dev_type: str = self.params['dev_type']
        nrow: int = self.params['nrow']
        seg: int = self.params['seg']
        w: int = self.params['w']

        tap_name = 'ptap_tile' if dev_type == 'nch' else 'ntap_tile'
        tx_name = 'n_tile' if dev_type == 'nch' else 'p_tile'
        tile_pattern_list = [TilePatternElement(tile_elements[tap_name])]
        tile_pattern_list.extend(
            [TilePatternElement(tile_elements[tx_name]) for _ in range(nrow)])
        tile_pattern_list.append(TilePatternElement(tile_elements[tap_name], flip=True))

        self.draw_base((TilePattern(tile_pattern_list), tile_elements))
        # tr_manager = self.tr_manager
        # hm_layer = self.arr_info.conn_layer + 1
        # vm_layer = hm_layer + 1
        tx_list = []
        for idx in range(nrow):
            tx_list.append(self.add_mos(0, 0, seg, tile_idx=idx + 1))

        tap_bot = self.add_substrate_contact(0, 0, seg=seg, tile_idx=0)
        tap_top = self.add_substrate_contact(0, 0, seg=seg, tile_idx=nrow + 1)
        self.set_mos_size()

        sup_bot_tid = self.get_track_id(0, MOSWireType.DS, wire_name='sup')
        sup_top_tid = self.get_track_id(0, MOSWireType.DS, wire_name='sup', tile_idx=-1)
        sup = [self.connect_to_tracks(tap_bot, sup_bot_tid), self.connect_to_tracks(tap_top, sup_top_tid)]

        g_list, d_list, s_list = [], [], []
        for idx in range(1, nrow + 1):
            g_list.append(
                self.connect_to_tracks(tx_list[idx - 1].g, self.get_track_id(0, MOSWireType.G, wire_name='sig',
                                                                             tile_idx=idx)))
            d_list.append(self.connect_to_tracks(tx_list[idx - 1].d, self.get_track_id(0, MOSWireType.DS, tile_idx=idx,
                                                                                       wire_name='sup', wire_idx=0)))
            s_list.append(self.connect_to_tracks(tx_list[idx - 1].s, self.get_track_id(0, MOSWireType.DS, tile_idx=idx,
                                                                                       wire_name='sup', wire_idx=1)))

        self.connect_wires([tap_bot, tap_top] + [tx.s for tx in tx_list])
        self.add_pin('VSS' if dev_type == 'nch' else 'VDD', sup, connect=True)
        self.add_pin('VSS' if dev_type == 'nch' else 'VDD', d_list, connect=True)
        self.add_pin('G', g_list, connect=True)

        self.sch_params = dict(
            lch=self.arr_info.lch,
            nf=seg * nrow,
            w=w,
            intent=self.get_row_info(0, 1).threshold,
            dev_type=self.get_row_info(0, 1).row_type.name,
            stack=self.params.get('stack', 1)
        )


class MOSBaseCap(MOSBase):
    """small mom cap quantized to unit transistor (but it's not a transistor cap) Use to generate ra bias
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='placement information object.',
            seg='Optional dictionary of user defined signal locations',
            half='True to draw half cap',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            seg=4,
            half=False,
        )

    def draw_layout(self):
        place_info = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(place_info)
        seg: int = self.params['seg']

        hm_layer = self.arr_info.conn_layer + 1
        vm_layer = hm_layer + 1
        self.set_mos_size(num_cols=seg, num_tiles=1)

        half = self.params['half']
        if not half:
            ns_conn, nd_conn, _ = fill_conn_layer_intv(self, 0, 0, extend_to_gate=False)
            nd_conn = [w for warr in nd_conn for w in warr.to_warr_list()]
        else:
            ns_conn, nd_conn = None, None
        ps_conn, pd_conn, _ = fill_conn_layer_intv(self, 0, 1, extend_to_gate=False)
        pd_conn = [w for warr in pd_conn for w in warr.to_warr_list()]
        ps_conn = [w for warr in ps_conn for w in warr.to_warr_list()]
        nd_top0 = self.get_track_id(0, MOSWireType.DS, 'sup')
        pd_top0 = self.get_track_id(1, MOSWireType.DS, 'sup')
        g_mid = self.get_track_id(0, MOSWireType.G, 'sig', wire_idx=2)

        row_info = self.get_row_info(0, 1)
        nds_conn_up = max(row_info.ds_conn_y)
        nds_conn_lo = min(row_info.ds_conn_y)

        nd_bot_odd = TrackID(hm_layer,
                             self.grid.coord_to_track(hm_layer, pd_conn[0].middle if half else nd_conn[0].middle,
                                                      RoundMode.NEAREST))
        pd_bot_even = TrackID(hm_layer, self.grid.coord_to_track(hm_layer, pd_conn[0].middle, RoundMode.NEAREST))
        nd_bot_even_tidx = self.tr_manager.get_next_track(hm_layer, nd_bot_odd.base_index, 'sig', 'sig', up=1)
        nd_bot_even = TrackID(hm_layer, nd_bot_even_tidx)
        nd_bot_odd_tidx = self.tr_manager.get_next_track(hm_layer, pd_bot_even.base_index, 'sig', 'sig', up=-1)
        pd_bot_odd = TrackID(hm_layer, nd_bot_odd_tidx)

        top_hm = [self.connect_to_tracks(ps_conn if half else ns_conn, _tid) for _tid in [nd_top0, g_mid, pd_top0]]
        if half:
            bot_hm = [self.connect_to_tracks(pd_conn, _tid) for _tid in [nd_bot_odd]]
            bot_hm.extend([self.connect_to_tracks(pd_conn, _tid) for _tid in [pd_bot_odd]])
        else:
            bot_hm = []
            for idx, (nd, pd) in enumerate(zip(nd_conn, pd_conn)):
                bot_hm.extend([self.connect_to_tracks(pd, pd_bot_even if idx & 1 else pd_bot_odd),
                               self.connect_to_tracks(nd, nd_bot_even if idx & 1 else nd_bot_odd)])
            # bot_hm = [self.connect_to_tracks(_conn, nd_bot_odd) for _conn in nd_conn[::2]]
            # bot_hm.extend([self.connect_to_tracks(_conn, pd_bot_even) for _conn in pd_conn[::2]])
            # bot_hm.extend([self.connect_to_tracks(_conn, nd_bot_even) for _conn in nd_conn[1::2]])
            # bot_hm.extend([self.connect_to_tracks(_conn, pd_bot_odd) for _conn in pd_conn[1::2]])
        d_vm_tidx = [self.arr_info.col_to_track(vm_layer, idx) for idx in range(1, seg + 1, 2)]

        if half:
            hm_bot_tidx = self.grid.coord_to_track(hm_layer, (nds_conn_up + nds_conn_lo) // 2, RoundMode.NEAREST)
            bot_hm.append(self.add_wires(hm_layer, hm_bot_tidx, lower=bot_hm[0].lower, upper=bot_hm[0].upper))

        bot_conn = pd_conn
        top_conn = ps_conn

        vm_min_len = self.grid.get_next_length(vm_layer, 1, self.grid.get_wire_total_width(vm_layer, 1), even=True)
        for idx in range(seg // 2):
            bot_bbox = BBox(bot_conn[idx].bound_box.xl, bot_conn[idx].upper - vm_min_len // 2,
                            bot_conn[idx].bound_box.xh, bot_conn[idx].upper)
            top_bbox = BBox(top_conn[idx].bound_box.xl, top_conn[idx].middle - vm_min_len // 2,
                            top_conn[idx].bound_box.xh, top_conn[idx].middle + vm_min_len // 2)
            self.add_res_metal(self.conn_layer, bot_bbox)
            self.add_res_metal(self.conn_layer, top_bbox)

        bot_vm = []
        # for ps in ps_conn:
        #     self.connect_to_tracks(top_hm, TrackID(vm_layer, ps.track_id.base_index))
        for idx, vmtidx in enumerate(d_vm_tidx):
            # if seg == 8:
            # breakpoint()
            # if idx & 1;
            bot_vm.append(self.connect_to_tracks(bot_hm[2 * idx: 2 * idx + 2], TrackID(vm_layer, vmtidx),
                                                 track_upper=pd_conn[0].upper,
                                                 track_lower=nds_conn_lo if half else nd_conn[0].lower
                                                 ))
        #     else
        # bot_vm = [self.connect_to_tracks(bot_hm, TrackID(vm_layer, _tid), track_upper=pd_conn[0].upper,
        #                                  track_lower=nds_conn_lo if half else nd_conn[0].lower) for _tid in d_vm_tidx]

        self.add_pin('plus', top_hm)
        self.add_pin('minus', bot_vm)

        self.sch_params = dict(
            res_plus=dict(layer=self.conn_layer, l=top_conn[0].bound_box.h, w=top_conn[0].bound_box.w),
            res_minus=dict(layer=self.conn_layer, l=bot_conn[0].bound_box.h, w=bot_conn[0].bound_box.w // 2),
            same_net_bot=True,
            same_net_top=True,
            m=seg // 2,
            cap=130e-18,
        )


class MOMCapOnMOS(MOSBase):
    """MOMCap core on MOSBase, use add_mom_cap method. Place it in an empty mosbase class
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._top_layer = 0

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('xbase', 'momcap_core')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            bot_layer='MOM cap bottom layer.',
            top_layer='MOM cap top layer.',
            width='MOM cap width, in resolution units.',
            height='MOM cap height, in resolution units.',
            margin='margin between cap and boundary, in resolution units.',
            port_tr_w='MOM cap port track width, in number of tracks.',
            options='MOM cap layout options.',
            half_blk_x='True to allow half horizontal blocks.',
            half_blk_y='True to allow half vertical blocks.',
            cap='Parameters for momcap schematic value',
            max_height='True to draw momcap with maximum possible height',
            ncol_tot='Use this to define the width if given',
            port_w_dict='Cap port dict, set to none to use port_tr_w',
            num_tiles='Number of tiles to constraint height',
            extend_tiles='True to extend num_tiles to meet height',
        )

    @property
    def top_layer(self) -> int:
        return self._top_layer

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            margin=0,
            port_tr_w=1,
            options=None,
            half_blk_x=True,
            half_blk_y=True,
            max_height=False,
            cap=None,
            ncol_tot=0,
            num_tiles=0,
            port_w_dict=None,
            extend_tiles=False,
        )

    def draw_layout(self) -> None:
        pinfo = self.params['pinfo']

        bot_layer: int = self.params['bot_layer']
        top_layer: int = self.params['top_layer']
        width: int = self.params['width']
        height: int = self.params['height']
        margin: int = self.params['margin']
        port_tr_w: int = self.params['port_tr_w']
        options: Optional[Mapping[str, Any]] = self.params['options']
        half_blk_x: bool = self.params['half_blk_x']
        half_blk_y: bool = self.params['half_blk_y']
        num_tiles = self.params['num_tiles']
        ncol_tot: int = self.params['ncol_tot']

        if isinstance(pinfo, Mapping):
            pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
            if self.params['extend_tiles']:
                tile_height = pinfo[1]['cap_tile'].height
                num_tiles = -(-(height + 2 * margin) // tile_height)
            if num_tiles:
                pinfo_list = [TilePatternElement(pinfo[1]['cap_tile'])] * num_tiles
                self.draw_base((TilePattern(pinfo_list), pinfo[1]))
            else:
                self.draw_base(pinfo)
        else:
            pinfo = pinfo[0]
            self.draw_base(pinfo)

        grid = self.grid

        if ncol_tot:
            w_tot = ncol_tot * self.arr_info.sd_pitch
            width = w_tot
        else:
            w_tot = width

        w_blk, h_blk = grid.get_block_size(top_layer, half_blk_x=half_blk_x,
                                           half_blk_y=half_blk_y)
        w_tot = -(-w_tot // w_blk) * w_blk

        # set size
        num_cols = -(-w_tot // self.sd_pitch)
        pinfo = self.draw_base_info[0]
        num_tiles = pinfo.num_tiles
        self.set_mos_size(num_tiles=num_tiles, num_cols=num_cols)
        if bot_layer > 1:
            for idx in range(num_tiles):
                _nrow = self.get_tile_info(idx)[0].num_rows
                for jdx in range(_nrow):
                    cur_row_type = self.get_tile_info(idx)[0].get_row_place_info(jdx).row_info.row_type
                    if cur_row_type is MOSType.ptap or cur_row_type is MOSType.ntap:
                        self.add_substrate_contact(jdx, 0, tile_idx=idx, seg=num_cols)
                    else:
                        fill_conn_layer_intv(self, idx, jdx, True)

        h_tot = self.get_tile_info(-1)[1] + self.get_tile_info(-1)[0].height
        if self.params['max_height']:
            height = h_tot - 2 * margin
        if h_tot < height + 2 * margin:
            raise ValueError("Doesn't have enough height")

        # setup capacitor options
        # get .  Make sure we can via up to top_layer + 1
        top_port_tr_w = port_tr_w
        port_w_dict = self.params['port_w_dict']

        if not port_w_dict:
            port_w_dict = {top_layer: top_port_tr_w}
            for lay in range(top_layer - 1, bot_layer - 1, -1):
                top_port_tr_w = port_w_dict[lay] = grid.get_min_track_width(lay, top_ntr=top_port_tr_w)

        # draw cap
        cap_xl = (self.bound_box.w - width) // 2
        cap_yb = (self.bound_box.h - height) // 2
        cap_box = BBox(cap_xl + margin, cap_yb + margin, cap_xl + width - margin, cap_yb + height - margin)
        num_layer = top_layer - bot_layer + 1
        options = options or {}
        cw_list: List[Tuple[Tuple[str, str], Tuple[str, str], BBoxArray, BBoxArray]] = []
        cap_ports = self.add_mom_cap(cap_box, bot_layer, num_layer, port_widths=port_w_dict,
                                     cap_wires_list=cw_list, **options)

        # connect input/output, draw metal resistors
        show_pins = self.show_pins
        for lay, (nport, pport) in cap_ports.items():
            self.add_pin('plus', pport, show=show_pins)
            self.add_pin('minus', nport, show=show_pins)

        _, _, barr_n, barr_p = cw_list[-1]
        box_p = barr_p.get_bbox(0)
        box_n = barr_n.get_bbox(0)
        top_dir = grid.get_direction(top_layer)
        res_w = box_p.get_dim(top_dir.perpendicular())
        coord_c = box_p.get_center(top_dir)

        res_w2 = res_w // 2
        res_w4 = res_w2 // 2
        wl_p = coord_c - res_w2
        wu_p = coord_c + res_w2
        wl_n = coord_c - res_w4
        wu_n = coord_c + res_w4
        box_p.set_interval(top_dir, wl_p, wu_p)
        box_n.set_interval(top_dir, wl_n, wu_n)
        self.add_res_metal(top_layer, box_p)
        self.add_res_metal(top_layer, box_n)

        res_w = grid.get_track_info(top_layer).width
        self._top_layer = top_layer
        sch_params_dict = dict(
            res_p=dict(layer=top_layer, w=res_w, l=res_w2 * 2),
            res_n=dict(layer=top_layer, w=res_w, l=res_w4 * 2),
        )
        if self.params['cap']:
            sch_params_dict['cap'] = self.params['cap']

        self._sch_params = sch_params_dict


class MOSBaseCapWrap(MOSBase):
    """ A MosBase Wrapper similiar to MOMCapOnMOS but wrap the capdac unit from SAR
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            cap_params='',
            ncol_tot='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ncol_tot=0
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        cap_params: ImmutableSortedDict = self.params['cap_params']
        ncol_tot: int = self.params['ncol_tot']
        # set size

        # draw cap
        cap_master = self.new_template(CapColCore, params=cap_params)
        width = cap_master.bound_box.w
        height = cap_master.bound_box.h
        num_cols = -(-width // self.sd_pitch)

        self.set_mos_size(num_cols=num_cols)
        cap_x = (max(ncol_tot, num_cols) - num_cols) // 2
        cap_x = cap_x * self.sd_pitch
        cap_y = (self.bound_box.yh - height) // 2
        if self.bound_box.yh < height:
            print("Warning: ra_cap/MOSBaseCapWrap cap too large")

        cap = self.add_instance(cap_master, xform=Transform(cap_x, cap_y, Orientation.R0))
        self.add_pin('top', cap.get_pin('top'))
        self.add_pin('top_xm', cap.get_all_port_pins('top_xm'), label='top')
        self.add_pin('bot', cap.get_pin('bot'))
        self.add_pin('bot_x', cap.get_all_port_pins('bot_x'))
        self._sch_params = dict(same_net_bot=True, **cap_master.sch_params)


class FBCapDACUnit(MOSBase):
    """MOMCap core on MOSBase
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_fb_cap_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='',
            cap_params='',
            ncol_tot='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ncol_tot=0
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        seg_dict = self.params['seg_dict']
        seg_pg = seg_dict['pg']
        seg_inv = seg_dict['inv']
        nds_tid0 = self.get_track_index(0, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nds_tid1 = self.get_track_index(0, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pds_tid0 = self.get_track_index(-1, MOSWireType.DS, wire_name='sig', wire_idx=-2)
        pds_tid1 = self.get_track_index(-1, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        ng_tid0 = self.get_track_index(0, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg_params = dict(pinfo=pinfo, seg=seg_pg, vertical_in=True, vertical_out=True,
                         sig_locs={'nd': nds_tid0, 'pd': pds_tid0, 'ns': ng_tid0}, vertical_sup=False)
        inv_params = dict(pinfo=pinfo, seg_list=seg_inv, sig_locs={'nout0': nds_tid1, 'nout1': nds_tid0,
                                                                   'pout0': pds_tid1, 'pout1': pds_tid0,
                                                                   'nin1': ng_tid0, 'nin0': ng_tid0 - 1},
                          dual_output=True)
        pg_master = self.new_template(PassGateCore, params=pg_params)
        inv_master = self.new_template(InvChainCore, params=inv_params)
        cap_params = dict(pinfo=pinfo, cap_params=self.params['cap_params'])
        cap_master = self.new_template(MOSBaseCapWrap, params=cap_params)

        vm_sup_col_list = [0]
        cur_col = 1
        inv_chain = self.add_tile(inv_master, tile_idx=0, col_idx=cur_col)
        vm_sup_col_list.append(cur_col + inv_master.num_cols + 1)
        cur_col += inv_master.num_cols + self.min_sep_col
        pg_in = self.add_tile(pg_master, tile_idx=0, col_idx=cur_col + pg_master.num_cols, flip_lr=True)
        cur_col += pg_master.num_cols + self.min_sep_col
        cur_col += self.min_sep_col & 1
        pg_out_vm_col = cur_col - 1
        pg_params = dict(pinfo=pinfo, seg=seg_pg, vertical_in=True, vertical_out=True,
                         sig_locs={'nd': nds_tid0, 'pd': pds_tid0, 'ns': ng_tid0 + 1}, vertical_sup=False)
        pg_master = self.new_template(PassGateCore, params=pg_params)
        pg_out = self.add_tile(pg_master, tile_idx=0, col_idx=cur_col + pg_master.num_cols, flip_lr=True)
        cur_col += pg_master.num_cols
        # cur_col += cur_col & 1
        cap = self.add_tile(cap_master, tile_idx=0, col_idx=cur_col)
        cur_col += cap_master.num_cols

        cur_col += cur_col & 1
        self.set_mos_size(cur_col)
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        # self.connect_to_track_wires(pg_out.get_pin('ns'), cap.get_pin('bot'))
        pg_in_vm_tidx = self.arr_info.col_to_track(vm_layer, pg_out_vm_col)
        pg_in_in_vm = self.connect_to_tracks(pg_in.get_pin('ns'),
                                             TrackID(vm_layer, pg_in_vm_tidx,
                                                     cap_params['cap_params']['cap_config']['bot_y_w']))
        self.connect_to_track_wires(pg_out.get_pin('ns'), cap.get_all_port_pins('top'))
        self.connect_to_track_wires(pg_in_in_vm, cap.get_all_port_pins('bot_x'))
        en_hm = inv_chain.get_pin('nin')
        self.connect_to_track_wires(inv_chain.get_pin('out'), [pg_in.get_pin('en'), pg_out.get_pin('en')])
        self.connect_to_track_wires(inv_chain.get_pin('outb'), [pg_in.get_pin('enb'), pg_out.get_pin('enb')])

        bot_x_xm_tidx = [w.track_id.base_index for w in cap.get_all_port_pins('top_xm')]

        out_vm = self.extend_wires(pg_out.get_pin('d'))
        in_vm = self.extend_wires(pg_in.get_pin('d'))
        out_xm = [self.connect_to_tracks(out_vm, TrackID(xm_layer, tidx)) for tidx in bot_x_xm_tidx]
        in_xm = [self.connect_to_tracks(in_vm, TrackID(xm_layer, tidx)) for tidx in bot_x_xm_tidx]

        ym_width = self.tr_manager.get_width(xm_layer + 1, 'cap')
        ym_layer = xm_layer + 1
        out_ym_tidx = self.grid.coord_to_track(ym_layer, out_xm[0].middle, RoundMode.NEAREST)
        in_ym_tidx = self.grid.coord_to_track(ym_layer, in_xm[0].middle, RoundMode.NEAREST)
        out_ym = self.connect_to_tracks(out_xm, TrackID(ym_layer, out_ym_tidx, ym_width,
                                                        grid=self.grid))
        in_ym = self.connect_to_tracks(in_xm, TrackID(ym_layer, in_ym_tidx, ym_width, grid=self.grid))
        vm_locs = [self.arr_info.col_to_track(vm_layer, _col) for _col in vm_sup_col_list]

        self.add_pin('out', out_ym)
        self.add_pin('in', in_ym)

        vss, vdd = export_xm_sup(self, 0, export_top=True, export_bot=True, given_locs=vm_locs)
        self.add_pin('en', en_hm)
        self.add_pin('VSS', vss)
        self.add_pin('VDD', vdd)
        self.add_pin('VSS_hm',
                     self.connect_wires([inv_chain.get_pin('VSS'), pg_out.get_pin('VSS'), pg_in.get_pin('VSS')]),
                     label='VSS')
        self.add_pin('VDD_hm',
                     self.connect_wires([inv_chain.get_pin('VDD'), pg_out.get_pin('VDD'), pg_in.get_pin('VDD')]),
                     label='VDD')
        #
        self._sch_params = dict(
            inv=inv_master.sch_params,
            pg=pg_master.sch_params,
            cap=cap_master.sch_params,
        )


class FBCapDACArray(MOSBase, TemplateBaseZL):
    """MOMCap core on MOSBase
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_fb_cap')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            bit_config='',
            ctrl_top='',
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='',
            cap_params='',
            ncol_tot='',
            nx='',
            ny=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ncol_tot=0,
            ctrl_top=False
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)
        ny = self.params['ny']
        nx = self.params['nx']

        unit_params = self.params.copy(remove=['nx', 'ny'])
        unit_master = self.new_template(FBCapDACUnit, params=unit_params)
        unit_list_list = []
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        vm_ntr, _ = tr_manager.place_wires(vm_layer, ['sig'] * (ny + 2))
        vm_rt_sp = self.arr_info.track_to_col(vm_layer, vm_ntr)
        unit_sp_ncol = unit_master.num_cols + vm_rt_sp + self.min_sep_col//2
        for idx in range(ny):
            unit_list = []
            cur_col = 0
            for jdx in range(nx):
                if not bool(jdx & 1) and jdx:
                    cur_col += vm_rt_sp
                fill_tap_intv(self, idx, cur_col + self.sub_sep_col, cur_col + vm_rt_sp - self.sub_sep_col)
                if not bool(jdx & 1):
                    cur_col += vm_rt_sp
                unit_list.append(self.add_tile(unit_master, tile_idx=idx, col_idx=cur_col, flip_lr=bool(jdx & 1)))
                if not bool(jdx & 1):
                    cur_col += 2 * unit_master.num_cols + self.min_sep_col
            unit_list_list.append(unit_list)

        en_vm_list_list = []
        for idx in range(nx):
            unit_col_list = [arr[idx] for arr in unit_list_list]
            vm_ncol_start = idx * unit_sp_ncol
            if idx & 1:
                vm_ncol_start += unit_sp_ncol
            vm_tidx = self.arr_info.col_to_track(vm_layer, vm_ncol_start)
            _, vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * (ny + 2), align_track=vm_tidx,
                                                align_idx=-1 if bool(idx & 1) else 0)
            unit_en_list = [inst.get_pin('en') for inst in unit_col_list]
            en_vm_list = self.connect_matching_tracks(unit_en_list, vm_layer,
                                                      vm_locs[:-2] if bool(idx & 1) else vm_locs[2:],
                                                      width=tr_manager.get_width(vm_layer, 'sig'))
            en_vm_list_list.append(en_vm_list)
        self.set_mos_size(nx * unit_sp_ncol)

        vdd_hm_list, vss_hm_list, vdd_xm_list, vss_xm_list = [], [], [], []
        for idx in range(ny):
            for jdx in range(nx):
                vdd_hm_list.append(unit_list_list[idx][jdx].get_pin('VDD_hm'))
                vss_hm_list.append(unit_list_list[idx][jdx].get_pin('VSS_hm'))
                vdd_xm_list.append(unit_list_list[idx][jdx].get_pin('VDD'))
                vss_xm_list.append(unit_list_list[idx][jdx].get_pin('VSS'))
        self.connect_wires(vdd_hm_list, lower=self.bound_box.xl)
        self.connect_wires(vss_hm_list, lower=self.bound_box.xl)
        vdd_xm_list = self.connect_wires(vdd_xm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_xm_list = self.connect_wires(vss_xm_list, lower=self.bound_box.xl, upper=self.bound_box.xh)

        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1

        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        top_xm1_tidx = self.grid.coord_to_track(xm1_layer, self.bound_box.yh)
        if tr_w_sig_xm1 > 0 and tr_w_sup_xm1 > 0:
            in_top_xm1_tidx = tr_manager.get_next_track(xm1_layer, top_xm1_tidx, 'sup', 'sig')
            out_top_xm1_tidx = tr_manager.get_next_track(xm1_layer, in_top_xm1_tidx, 'sig', 'sig', up=True)
        else:
            in_top_xm1_tidx = top_xm1_tidx + self.get_track_sep(xm1_layer, tr_w_sup_xm1, tr_w_sig_xm1)
            out_top_xm1_tidx = in_top_xm1_tidx + self.get_track_sep(xm1_layer, tr_w_sig_xm1, tr_w_sig_xm1)
        bit_xm_tidx = tr_manager.get_next_track(xm_layer, self.grid.coord_to_track(xm_layer,
                                                                                   self.bound_box.yh if self.params[
                                                                                       'ctrl_top'] else self.bound_box.yl),
                                                'dum', 'sig', up=True if self.params['ctrl_top'] else False)
        if tr_w_sig_xm1 > 0:
            in_bot_xm1_tidx = tr_manager.get_next_track(xm1_layer,
                                                        self.grid.coord_to_track(xm1_layer, self.bound_box.yl),
                                                        'dum', 'sig', up=False)
        else:
            in_bot_xm1_tidx = self.grid.coord_to_track(xm1_layer, self.bound_box.yl) - \
                              self.get_track_sep(xm1_layer, 1, tr_w_sig_xm1)
        bit_config = self.params['bit_config']
        num_en_xm = len(bit_config)
        _, bit_xm_locs = tr_manager.place_wires(xm_layer, ['ctrl'] * num_en_xm, align_track=bit_xm_tidx,
                                                align_idx=0 if self.params['ctrl_top'] else -1)
        en_sig_list = []
        for bit in bit_config:
            if isinstance(bit['row'], ImmutableList):
                en_sig_list.append(en_vm_list_list[bit['col']][bit['row'][0]: bit['row'][1]])
            else:
                en_sig_list.append(en_vm_list_list[bit['col']][bit['row']])

        unit_radix_list = []
        for sig in en_sig_list:
            if isinstance(sig, List):
                unit_radix_list.append(len(sig))
            else:
                unit_radix_list.append(1)

        bit_xm_list = self.connect_matching_tracks(en_sig_list, xm_layer, bit_xm_locs)

        in_xm1_list, out_xm1_list = [], []
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')
        for idx in range(ny):
            h_mid = self.get_tile_info(idx)[1] + self.get_tile_info(idx)[0].height // 2
            _, [out_xm1_tidx, in_xm1_tidx] = tr_manager.place_wires(xm1_layer, ['sig'] * 2, center_coord=h_mid)
            in_xm1_list.append(self.connect_to_tracks([inst.get_pin('in') for inst in unit_list_list[idx]],
                                                      TrackID(xm1_layer, in_xm1_tidx, tr_w_sig_xm1)))
            out_xm1_list.append(self.connect_to_tracks([inst.get_pin('out') for inst in unit_list_list[idx]],
                                                       TrackID(xm1_layer, out_xm1_tidx, tr_w_sig_xm1)))
        self.add_pin('out', out_xm1_list, connect=True)
        self.add_pin('in', in_xm1_list, connect=True)
        for idx, bit in enumerate(bit_xm_list):
            self.add_pin(f'bit<{idx}>', bit)
        self.add_pin('VDD', vdd_xm_list)
        self.add_pin('VSS', vss_xm_list)
        self._sch_params = dict(
            unit_params=unit_master.sch_params,
            radix_list=unit_radix_list
        )
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        # ym power
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')
        tr_w_sup_xm = tr_manager.get_width(ym_layer, 'sup')
        wl, wh = self.grid.get_wire_bounds(ym_layer, 0, width=tr_w_sup_ym)
        w_ym = int((wh - wl) / 2)
        via_ext_xm, via_ext_ym = self.grid.get_via_extensions(Direction.UPPER, xm_layer, tr_w_sup_xm, tr_w_sup_ym)
        dx = via_ext_xm + w_ym
        mid_space = self.grid.track_to_coord(vm_layer, tr_manager.get_width(ym_layer, 'sup'))
        bb = BBox(xl=self.bound_box.xl + mid_space, xh=self.bound_box.xh - dx,
                  yl=self.bound_box.yl - dx, yh=self.bound_box.yh + dx)

        vdd_ym, vss_ym = self.do_power_fill(ym_layer, tr_manager, vdd_xm_list, vss_xm_list, bound_box=bb)
        # xm1 power
        tr_w_sup_xm1 = tr_manager.get_width(xm1_layer, 'sup')
        wl, wh = self.grid.get_wire_bounds(ym_layer, 0, width=tr_w_sup_xm1)
        w_xm = int((wh - wl) / 2)
        via_ext_ym1, via_ext_xm1 = self.grid.get_via_extensions(Direction.UPPER, ym_layer, tr_w_sup_ym, tr_w_sup_xm1)
        dy = via_ext_ym1 + w_xm
        bb = BBox(xl=self.bound_box.xl + via_ext_xm1, xh=self.bound_box.xh - via_ext_xm1,
                  yl=self.bound_box.yl + dy, yh=self.bound_box.yh - dy)
        vdd_xm1_list, vss_xm1_list = [], []
        for xmsup in vdd_xm_list[0].to_warr_list():
            coord = self.grid.track_to_coord(xm_layer, xmsup.track_id.base_index)
            tidx = self.grid.coord_to_track(xm1_layer, coord, RoundMode.NEAREST)
            vdd_xm1_list.append(self.connect_to_tracks(vdd_ym, TrackID(xm1_layer, tidx, tr_w_sup_xm1)))
        for xmsup in vss_xm_list[0].to_warr_list():
            coord = self.grid.track_to_coord(xm_layer, xmsup.track_id.base_index)
            tidx = self.grid.coord_to_track(xm1_layer, coord, RoundMode.NEAREST)
            vss_xm1_list.append(self.connect_to_tracks(vss_ym, TrackID(xm1_layer, tidx, tr_w_sup_xm1)))

        self.extend_wires(vdd_xm1_list + vss_xm1_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        self.add_pin('VDD', vdd_xm1_list)
        self.add_pin('VSS', vss_xm1_list)


class RASAMCap(MOSBase):
    """ A MosBase Wrapper similiar to MOMCapOnMOS but wrap the capdac unit from SAR, used as sampling cap
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'cap_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            ny='number of unit cap',
            pinfo='The MOSBasePlaceInfo object.',
            cap_params='',
            ncol_tot='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            ncol_tot=0
        )

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        cap_params: ImmutableSortedDict = self.params['cap_params']
        ncol_tot: int = self.params['ncol_tot']
        ny: int = self.params['ny']
        # set size

        # draw cap
        cap_gen_params = cap_params.copy(append=dict(ny=4 * ny, ny_unit=4))
        cap_master = self.new_template(CapColCore, params=cap_gen_params)
        width = cap_master.bound_box.w
        height = cap_master.bound_box.h
        num_cols = -(-width // self.sd_pitch)

        self.set_mos_size(num_cols=num_cols, num_tiles=ny)
        cap_x = (max(ncol_tot, num_cols) - num_cols) // 2
        cap_x = cap_x * self.sd_pitch
        cap_y = (self.bound_box.yh - height) // 2
        if self.bound_box.yh < height:
            print("Warning: ra_cap/MOSBaseCapWrap cap too large")

        cap = self.add_instance(cap_master, xform=Transform(cap_x, cap_y, Orientation.R0))
        self.add_pin('top', cap.get_pin('top'))
        self.add_pin('top_xm', cap.get_all_port_pins('top_xm'), label='top')
        self.add_pin('bot', cap.get_pin('bot'))
        self.add_pin('bot_x', cap.get_all_port_pins('bot_x'))

        cap_sch_params = cap_master.sch_params.to_dict()
        cap_sch_params['m'] *= ny
        self._sch_params = cap_sch_params
