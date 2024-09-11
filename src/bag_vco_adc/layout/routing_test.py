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



from typing import Any, Dict

from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param
from bag.util.math import HalfInt
from pybag.core import BBox


class RoutingTest(TemplateBase):

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            vlayer='',
            hlayer=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(vlayer=3, hlayer=2)

    def get_next_track(self, layer, ntr1, ntr2, cur_idx):
        sep = self.grid.get_sep_tracks(layer, ntr1, ntr2)
        cur_idx = HalfInt.convert(cur_idx)
        return cur_idx + 2 * 2 * sep

    def draw_layout(self) -> None:
        vlayer = self.params['vlayer']
        hlayer = self.params['hlayer']
        top_layer = max(vlayer, hlayer)
        cur_idx_h, cur_idx_v = 0, 0
        for hdx in range(1, 20):
            h_min_len = self.grid.get_next_length(hlayer, hdx, 0, even=True)
            v_coord = self.grid.track_to_coord(hlayer, 0)
            h_wire = self.add_wires(hlayer, cur_idx_h, lower=v_coord - h_min_len // 2, upper=v_coord + h_min_len // 2,
                                    width=hdx)
            cur_idx_h = self.get_next_track(hlayer, hdx, hdx + 1, cur_idx_h)
            cur_idx_v = 0
            for vdx in range(1, 20):
                v_min_len = self.grid.get_next_length(vlayer, vdx, 0, even=True)
                h_coord = self.grid.track_to_coord(hlayer, cur_idx_h)
                v_wire = self.add_wires(vlayer, cur_idx_v, lower=h_coord - v_min_len // 2,
                                        upper=h_coord + v_min_len // 2, width=vdx)
                try:
                    self.connect_to_track_wires(h_wire, v_wire)
                except:
                    print(f'No possible via btw hlayer = {hdx} and vlayer={vdx}')
                cur_idx_v = self.get_next_track(vlayer, vdx, vdx + 1, cur_idx_v)

        w_tot = self.grid.track_to_coord(vlayer, cur_idx_v)
        h_tot = self.grid.track_to_coord(hlayer, cur_idx_h)
        self.set_size_from_bound_box(top_layer, BBox(0, 0, w_tot, h_tot))
