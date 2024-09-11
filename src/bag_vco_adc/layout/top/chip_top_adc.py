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



from typing import Any, Dict, Mapping, Tuple, Optional

from bag.design.database import ModuleDB
from bag.io import read_yaml
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param
from bag.util.importlib import import_class
from bag.util.math import HalfInt
from pybag.core import Transform, BBox
from pybag.enum import Orientation, RoundMode, Direction, PinMode
from xbase.layout.mos.top import GenericWrapper


class SingleChannel(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    # @classmethod
    # def get_schematic_class(cls) -> Optional[Type[Module]]:
    #     # noinspection PyTypeChecker
    #     return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_fb')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            block_dict='',
            top_layer=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict()

    def draw_layout(self) -> None:

        # Get sub-blocks
        block_dict = self.params['block_dict']
        [sampler_file, sar_file, vco_file, ra_file, clk_file] = \
            [block_dict['sampler'], block_dict['sar'], block_dict['vco'], block_dict['ra'], block_dict['clk']]

        sampler_file = read_yaml(sampler_file)
        sar_file = read_yaml(sar_file)
        vco_file = read_yaml(vco_file)
        ra_file = read_yaml(ra_file)
        clk_file = read_yaml(clk_file)
        sampler_master = self.new_template(import_class(sampler_file['lay_class']), params=sampler_file['params'])
        sar_master = self.new_template(import_class(sar_file['lay_class']), params=sar_file['params'])
        vco_master = self.new_template(import_class(vco_file['lay_class']), params=vco_file['params'])
        ra_master = self.new_template(import_class(ra_file['lay_class']), params=ra_file['params'])
        clk_master = self.new_template(import_class(clk_file['lay_class']), params=clk_file['params'])

        # Place blocks
        top_layer = self.params['top_layer']
        w_blk, h_blk = self.grid.get_block_size(top_layer)
        sam_w, sam_h = sampler_master.bound_box.w, sampler_master.bound_box.h
        vco_w, vco_h = vco_master.bound_box.w, vco_master.bound_box.h
        ra_w, wa_h = ra_master.bound_box.w, ra_master.bound_box.h
        clk_w, clk_h = clk_master.bound_box.w, clk_master.bound_box.h
        sar_w, sar_h = sar_master.bound_box.w, sar_master.bound_box.h

        tot_w0 = max(sar_w, sam_w, vco_w, ra_w)
        tot_w1 = clk_w + tot_w0
        x_channel_center = clk_w + tot_w0//2

        vco_x = -(-(x_channel_center-vco_w//2)//w_blk)*w_blk
        vco = self.add_instance(vco_master, xform=Transform(vco_x, 0, Orientation.R0))

        ra_x = -(-(x_channel_center-ra_w//2)//w_blk)*w_blk
        ra_y = -(-vco.bound_box.yh//h_blk)*h_blk
        ra = self.add_instance(ra_master, xform=Transform(ra_x, ra_y, Orientation.R0))

        sar_x = -(-(x_channel_center-sar_w//2)//w_blk)*w_blk
        sar_y = -(-ra.bound_box.yh//h_blk)*h_blk
        sar = self.add_instance(sar_master, xform=Transform(sar_x, sar_y, Orientation.R0))

        sam_x = -(-(x_channel_center-sam_w//2)//w_blk)*w_blk
        sam_y = -(-sar.bound_box.yh//h_blk)*h_blk
        sam = self.add_instance(sampler_master, xform=Transform(sam_x, sam_y, Orientation.R0))

        clk_y = -(-(sar.bound_box.yh-clk_master.bound_box.h)//h_blk)*h_blk
        clk = self.add_instance(clk_master, xform=Transform(0, clk_y, Orientation.R0))
        tot_w = -(-tot_w1//w_blk)*w_blk
        tot_h = -(-sam.bound_box.yh//h_blk)*h_blk
        self.set_size_from_bound_box(top_layer, BBox(0, 0, tot_w, tot_h))
        tr_manager = sar_master.tr_manager
        tid_b = self.grid.coord_to_track(top_layer, 0, RoundMode.GREATER_EQ)
        tid_t = self.grid.coord_to_track(top_layer, self.bound_box.yh, RoundMode.LESS_EQ)
        num_xm_sup = tr_manager.get_num_wires_between(top_layer, 'dum', tid_b, 'dum', tid_t, 'sup')
        locs = tr_manager.spread_wires(top_layer, ['sup'] * num_xm_sup, tid_b, tid_t, ('sup', 'sup'))
        [self.add_wires(top_layer, tidx, lower=self.bound_box.xl, upper=self.bound_box.xh) for tidx in locs]


class TIArray(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    # @classmethod
    # def get_schematic_class(cls) -> Optional[Type[Module]]:
    #     # noinspection PyTypeChecker
    #     return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_fb')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            block_dict='',
            top_layer='',
            num_ti='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(num_ti=4)

    def draw_layout(self) -> None:

        channel_master = self.new_template(SingleChannel, params=self.params)

        # Place blocks
        top_layer = self.params['top_layer']
        w_blk, h_blk = self.grid.get_block_size(top_layer)
        w_ch, h_ch = channel_master.bound_box.w, channel_master.bound_box.h
        nchannel = self.params['num_ti']
        channel_list = []
        cur_x = 0
        for idx in range(nchannel):
            channel_list.append(self.add_instance(channel_master, xform=Transform(cur_x, 0, Orientation.R0)))
            cur_x += channel_master.bound_box.w

        self.set_size_from_bound_box(top_layer, BBox(0, 0, cur_x, channel_master.bound_box.h))



