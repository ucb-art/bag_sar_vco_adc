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

from typing import Any, Dict, Type, Optional

from bag.design.database import ModuleDB, Module
from bag.io.file import read_yaml
from bag.layout.routing.base import TrackManager
from bag.layout.template import TemplateDB, TemplateBase
from bag.util.immutable import Param
from pybag.core import BBox, Transform
from pybag.enum import Orientation, Orient2D, RoundMode, MinLenMode, PinMode
from .bootstrap_new import Bootstrap, BootstrapDiff
from ..util.template import TemplateBaseZL
from ..util.template import TrackIDZL as TrackID
from ..util.wrapper import GenericWrapper


class SamplerTop(TemplateBaseZL):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'sampler_top')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            cdac_width='CDAC width for xm1 layer conenction',
            vcm_sampler='Path to vcm sampler yaml(diff)',
            sig_sampler='Path to signal sampler yaml',
            vcm_mid_sw='Path to middle switch yaml',
            top_layer_supply='True to add top layer supply'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(top_layer_supply=True, cdac_width=0)

    def draw_layout(self) -> None:
        sig_sampler_params: Param = Param(read_yaml(self.params['sig_sampler'])['params'])
        vcm_sampler_params: Param = read_yaml(self.params['vcm_sampler'])['params']
        mid_sw_params = read_yaml(self.params['vcm_mid_sw'])['params']

        mid_sw_master: GenericWrapper = self.new_template(GenericWrapper, params=mid_sw_params)
        sig_sampler_n_master = self.new_template(Bootstrap, params=sig_sampler_params)
        sig_sampler_p_master = self.new_template(Bootstrap, params=sig_sampler_params.copy(append=dict(nside=False)))
        vcm_sampler_master = self.new_template(BootstrapDiff, params=vcm_sampler_params)

        conn_layer = mid_sw_master.core.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xm1_layer = ym_layer + 1
        ym1_layer = xm1_layer + 1

        tr_manager = TrackManager(grid=self.grid, tr_widths=sig_sampler_params['tr_widths'],
                                  tr_spaces=sig_sampler_params['tr_spaces'], )
        top_layer_btstrp = max(sig_sampler_n_master.top_layer, vcm_sampler_master.top_layer)
        top_layer_midsw = mid_sw_master.top_layer

        # top_layer = sampler_p_master.top_layer
        w_blk_midsw, h_blk_midsw = self.grid.get_block_size(top_layer_midsw)
        w_blk, h_blk = self.grid.get_block_size(top_layer_btstrp)
        w_cmsw, h_cmsw = mid_sw_master.bound_box.w, mid_sw_master.bound_box.h
        w_sig_sampler_se, h_sig_sampler_se = \
            sig_sampler_n_master.bound_box.w, sig_sampler_n_master.bound_box.h
        w_vcm_sampler_diff, h_vcm_sampler_diff = \
            vcm_sampler_master.bound_box.w, vcm_sampler_master.bound_box.h

        w_tot = max(w_vcm_sampler_diff, w_cmsw) + 2 * w_sig_sampler_se
        h_tot = max(h_sig_sampler_se, h_vcm_sampler_diff + h_cmsw)
        w_tot = -(-w_tot // w_blk) * w_blk
        h_tot = -(-h_tot // h_blk) * h_blk
        w_tot2 = w_tot // 2

        cm_sw_mid_n = self.add_instance(mid_sw_master, xform=Transform(w_tot2, h_blk//2, Orientation.MY))
        mid_sw_params['params']['swap_inout'] = True
        mid_sw_master = self.new_template(GenericWrapper, params=mid_sw_params)
        cm_sw_mid_p = self.add_instance(mid_sw_master, xform=Transform(w_tot2, h_blk//2, Orientation.R0))

        # get xm1layer supply pitch
        top_hor_lay = top_layer_btstrp if self.grid.get_direction(
            top_layer_btstrp) == Orient2D.x else top_layer_btstrp - 1

        top_hor_sup_w = tr_manager.get_width(top_hor_lay, 'sup')
        top_hor_sup_pitch = self.grid.get_track_pitch(top_hor_lay) * \
                            self.get_track_sep(top_hor_lay, top_hor_sup_w, top_hor_sup_w)
        # xm1_sup_start = self.grid.track_to_coord(xm1_layer,
        #                                          self.grid.coord_to_track(xm1_layer, ym_dy, RoundMode.GREATER))

        top_vdd_diff_coord = (vcm_sampler_master.top_hor_vdd_coord - sig_sampler_n_master.top_hor_vdd_coord)
        sup_align_ofst = -top_vdd_diff_coord
        sup_align_ofst += 2 * top_hor_sup_pitch if sup_align_ofst < 0 else 0
        sup_align_ofst += -(-cm_sw_mid_n.bound_box.h // 2 // top_hor_sup_pitch) * 2 * top_hor_sup_pitch
        y_cm_sampler = -(-sup_align_ofst // h_blk) * h_blk

        cm_sampler = self.add_instance(vcm_sampler_master,
                                       xform=Transform(w_tot2 - w_vcm_sampler_diff // 2, y_cm_sampler, Orientation.R0))
        sig_sampler_p = self.add_instance(sig_sampler_n_master, xform=Transform(w_tot, 0, Orientation.MY))
        sig_sampler_n = self.add_instance(sig_sampler_p_master, xform=Transform(0, 0, Orientation.R0))

        self.set_size_from_bound_box(top_layer_btstrp, BBox(0, 0, w_tot, h_tot))

        self.connect_wires(cm_sampler.get_all_port_pins('VDD', layer=top_hor_lay) +
                           sig_sampler_n.get_all_port_pins('VDD', layer=top_hor_lay))
        self.connect_wires(cm_sampler.get_all_port_pins('VDD', layer=top_hor_lay) +
                           sig_sampler_p.get_all_port_pins('VDD', layer=top_hor_lay))
        self.connect_wires(cm_sampler.get_all_port_pins('VSS', layer=top_hor_lay) +
                           sig_sampler_n.get_all_port_pins('VSS', layer=top_hor_lay))
        self.connect_wires(cm_sampler.get_all_port_pins('VSS', layer=top_hor_lay) +
                           sig_sampler_p.get_all_port_pins('VSS', layer=top_hor_lay))

        # Connect between middle switch and cm sampler
        mid_sw_n_xm = self.connect_wires(cm_sw_mid_p.get_all_port_pins('ref') + cm_sw_mid_n.get_all_port_pins('sig'))[0]
        mid_sw_p_xm = self.connect_wires(cm_sw_mid_p.get_all_port_pins('sig') + cm_sw_mid_n.get_all_port_pins('ref'))[0]

        cm_sw_n_xm1 = cm_sampler.get_all_port_pins('out_n')
        cm_sw_p_xm1 = cm_sampler.get_all_port_pins('out_p')
        # self.connect_differential_wires(cm_sw_n_ym, cm_sw_p_ym, mid_sw_n_xm, mid_sw_p_xm)
        # find margin
        cm_sw_bnd_x = (mid_sw_master.bound_box.w - mid_sw_master.core_bound_box.w) // 2

        cdac_width = self.params['cdac_width']
        cdac_width = int(cdac_width / self.grid.resolution)
        _ym_bnd = cm_sw_mid_n.bound_box.xh - cdac_width if cdac_width else cm_sw_mid_n.bound_box.xl + cm_sw_bnd_x
        mid_sw_n_ym_tids = \
            self.get_tids_between(ym_layer, self.grid.coord_to_track(ym_layer, _ym_bnd, RoundMode.GREATER),
                                  self.grid.coord_to_track(ym_layer, cm_sw_mid_n.bound_box.xh - cm_sw_bnd_x),
                                  tr_manager.get_width(ym_layer, 'sig'), 0, 0, True)

        _ym_bnd = cm_sw_mid_p.bound_box.xl + cdac_width if cdac_width else cm_sw_mid_p.bound_box.xh - cm_sw_bnd_x
        mid_sw_p_ym_tids = \
            self.get_tids_between(ym_layer, self.grid.coord_to_track(ym_layer, cm_sw_mid_p.bound_box.xl + cm_sw_bnd_x),
                                  self.grid.coord_to_track(ym_layer, _ym_bnd, RoundMode.LESS),
                                  tr_manager.get_width(ym_layer, 'sig'), 0, 0, True)
        mid_sw_n_ym_list, mid_sw_p_ym_list = [], []
        for tidn, tidp in zip(mid_sw_n_ym_tids, mid_sw_p_ym_tids):
            _n, _p = self.connect_differential_tracks(mid_sw_n_xm, mid_sw_p_xm, ym_layer, tidn.base_index,
                                                      tidp.base_index, width=tidn.width)
            mid_sw_n_ym_list.append(_n)
            mid_sw_p_ym_list.append(_p)

        xm1_bot_tidx = self.grid.coord_to_track(xm1_layer, mid_sw_n_ym_list[0].upper, RoundMode.NEAREST)
        tr_w_sig_xm1 = tr_manager.get_width(xm1_layer, 'sig')

        mid_sw_n_xm1 = self.connect_to_tracks(mid_sw_n_ym_list, TrackID(xm1_layer, xm1_bot_tidx, tr_w_sig_xm1,
                                                                        grid=self.grid))
        mid_sw_p_xm1 = self.connect_to_tracks(mid_sw_p_ym_list, TrackID(xm1_layer, xm1_bot_tidx, tr_w_sig_xm1,
                                                                        grid=self.grid))
        tr_w_sig_ym1 = tr_manager.get_width(ym1_layer, 'sig')
        sw_n_ym1_tidx = self.grid.coord_to_track(ym1_layer, cm_sw_n_xm1[0].upper, RoundMode.NEAREST) - \
                        self.get_track_sep(ym_layer, tr_w_sig_ym1, 1).div2()
        sw_p_ym1_tidx = self.grid.coord_to_track(ym1_layer, cm_sw_p_xm1[0].lower, RoundMode.NEAREST) + \
                        self.get_track_sep(ym_layer, tr_w_sig_ym1, 1).div2()

        self.connect_to_tracks(cm_sw_n_xm1 + [mid_sw_n_xm1], TrackID(ym1_layer, sw_n_ym1_tidx, tr_w_sig_ym1,
                                                                     grid=self.grid))
        self.connect_to_tracks(cm_sw_p_xm1 + [mid_sw_p_xm1], TrackID(ym1_layer, sw_p_ym1_tidx, tr_w_sig_ym1,
                                                                     grid=self.grid))

        # power connection for cm middles switch
        # mid ym
        mid_ym_tidx = self.grid.coord_to_track(ym_layer, w_tot2, mode=RoundMode.NEAREST)
        left_sup_ym_tidx = self.grid.coord_to_track(ym_layer, cm_sampler.bound_box.xh, mode=RoundMode.LESS_EQ)
        tr_w_sup_ym = tr_manager.get_width(ym_layer, 'sup')

        if tr_w_sup_ym > 0:
            tr_sp_sup_ym = tr_manager.get_sep(ym_layer, ('sup', 'sup'))
        else:
            tr_sp_sup_ym = self.get_track_sep(ym_layer, tr_w_sup_ym, tr_w_sup_ym)

        avail_ym_sup_l = self.get_available_tracks(ym_layer,
                                                   self.grid.coord_to_track(ym_layer, cm_sw_mid_p.bound_box.xh,
                                                                            RoundMode.GREATER_EQ),
                                                   left_sup_ym_tidx,
                                                   lower=self.bound_box.yl, upper=cm_sw_mid_n.bound_box.yh,
                                                   width=tr_w_sup_ym, sep=tr_sp_sup_ym,
                                                   include_last=True)[:-1]
        avail_ym_sup_r = [2 * mid_ym_tidx - tidx for tidx in avail_ym_sup_l]

        vss_cmsw_xm = self.connect_wires([cm_sw_mid_n.get_pin('VSS'), cm_sw_mid_p.get_pin('VSS')])[0]
        vdd_cmsw_xm = self.connect_wires([cm_sw_mid_n.get_pin('VDD'), cm_sw_mid_p.get_pin('VDD')])[0]
        vss_cmsw_ym = [self.connect_to_tracks(vss_cmsw_xm, TrackID(ym_layer, tidx, tr_w_sup_ym, grid=self.grid),
                                              track_upper=cm_sw_mid_n.bound_box.yh,
                                              track_lower=cm_sw_mid_n.bound_box.yl)
                       for tidx in avail_ym_sup_l[::2] + avail_ym_sup_r[::2]]
        vdd_cmsw_ym = [self.connect_to_tracks(vdd_cmsw_xm, TrackID(ym_layer, tidx, tr_w_sup_ym, grid=self.grid),
                                              track_upper=cm_sw_mid_n.bound_box.yh,
                                              track_lower=cm_sw_mid_n.bound_box.yl)
                       for tidx in avail_ym_sup_l[1::2] + avail_ym_sup_r[1::2]]

        sig_sampler_xm1_vdd = sig_sampler_n.get_all_port_pins('VDD', xm1_layer) + \
                              sig_sampler_p.get_all_port_pins('VDD', xm1_layer)
        sig_sampler_xm1_vss = sig_sampler_n.get_all_port_pins('VSS', xm1_layer) + \
                              sig_sampler_p.get_all_port_pins('VSS', xm1_layer)

        cm_sw_vdd_xm1 = [warr for warr in sig_sampler_xm1_vdd if warr.bound_box.yh < y_cm_sampler]
        cm_sw_vss_xm1 = [warr for warr in sig_sampler_xm1_vss if warr.bound_box.yh < y_cm_sampler]

        vdd_cmsw_xm1 = [self.connect_to_track_wires(vdd_cmsw_ym, warr) for warr in cm_sw_vdd_xm1]
        vss_cmsw_xm1 = [self.connect_to_track_wires(vss_cmsw_ym, warr) for warr in cm_sw_vss_xm1]
        vdd_cm_ym1 = cm_sampler.get_all_port_pins('VDD', layer=ym1_layer)
        vss_cm_ym1 = cm_sampler.get_all_port_pins('VSS', layer=ym1_layer)

        # Routing clock signals
        # sam_e
        cmsw_sam_xm = self.connect_wires([cm_sw_mid_n.get_pin('sam'), cm_sw_mid_p.get_pin('sam')])[0]
        cmsw_samb_xm = self.connect_wires([cm_sw_mid_n.get_pin('sam_b'), cm_sw_mid_p.get_pin('sam_b')])[0]
        cm_sam_xm = cm_sampler.get_all_port_pins('sample')
        cm_samb_xm = cm_sampler.get_all_port_pins('sample_b')
        sam_e_ym_tidx = self.grid.coord_to_track(ym_layer, cm_sam_xm[0].middle, RoundMode.NEAREST)
        sam_e_b_vm_tidx = self.grid.coord_to_track(vm_layer, cm_sam_xm[0].middle, RoundMode.NEAREST)
        tr_w_clk_ym = tr_manager.get_width(ym_layer, 'clk')
        tr_w_clk_vm = tr_manager.get_width(vm_layer, 'clk')

        sam_e_vm = self.connect_to_tracks([cmsw_sam_xm] + cm_sam_xm, TrackID(vm_layer, sam_e_b_vm_tidx, tr_w_clk_vm))
        sam_e_b_ym = self.connect_to_tracks([cmsw_samb_xm] + cm_samb_xm, TrackID(ym_layer, sam_e_ym_tidx, tr_w_clk_ym))
        y_max = max(cm_sampler.bound_box.yh, sig_sampler_p.bound_box.yh)

        y_max_xm_tidx = self.grid.coord_to_track(xm_layer, y_max, RoundMode.GREATER_EQ)
        _, clk_xm_locs = tr_manager.place_wires(xm_layer, ['clk', 'dum']*6+['clk'],
                                                align_track=y_max_xm_tidx, align_idx=3)
        clk_xm_locs = clk_xm_locs[::2]
        tr_w_clk_xm = tr_manager.get_width(xm_layer, 'clk')
        sam_e_xm = self.connect_to_tracks(sam_e_vm, TrackID(xm_layer, clk_xm_locs[2], tr_w_clk_xm),
                                          min_len_mode=MinLenMode.MIDDLE)

        sam_e_b_ym_ret = []
        sam_e_b_xm = self.connect_to_tracks(sam_e_b_ym, TrackID(xm_layer, clk_xm_locs[1], tr_w_clk_xm),
                                            min_len_mode=MinLenMode.MIDDLE, ret_wire_list=sam_e_b_ym_ret)
        clk_ym_n_tidx = self.grid.coord_to_track(ym_layer, sig_sampler_n.bound_box.xh, RoundMode.LESS_EQ)
        _, clk_ym_locs_n = tr_manager.place_wires(ym_layer, ['clk'] * 2 + ['dum'], align_idx=-1,
                                                  align_track=clk_ym_n_tidx)

        clk_ym_p_tidx = self.grid.coord_to_track(ym_layer, sig_sampler_p.bound_box.xl, RoundMode.GREATER_EQ)
        _, clk_ym_locs_p = tr_manager.place_wires(ym_layer, ['dum'] + ['clk'] * 2, align_idx=0,
                                                  align_track=clk_ym_p_tidx)

        sam_l_ym = self.connect_to_tracks(sig_sampler_n.get_all_port_pins('sample'),
                                          TrackID(ym_layer, clk_ym_locs_n[-2],
                                                  tr_w_clk_ym))
        sam_b_l_ym = self.connect_to_tracks(sig_sampler_n.get_pin('sample_b'), TrackID(ym_layer, clk_ym_locs_n[-3],
                                                                                       tr_w_clk_ym))

        sam_r_ym = self.connect_to_tracks(sig_sampler_p.get_all_port_pins('sample'), TrackID(ym_layer, clk_ym_locs_p[1],
                                                                                             tr_w_clk_ym))
        sam_b_r_ym = self.connect_to_tracks(sig_sampler_p.get_pin('sample_b'), TrackID(ym_layer, clk_ym_locs_p[2],
                                                                                       tr_w_clk_ym))

        sam_xm = self.connect_to_tracks([sam_l_ym, sam_r_ym], TrackID(xm_layer, clk_xm_locs[4], tr_w_clk_xm))
        sam_b_xm = self.connect_to_tracks([sam_b_l_ym, sam_b_r_ym], TrackID(xm_layer, clk_xm_locs[3], tr_w_clk_xm))

        ymid_tidx = self.grid.coord_to_track(ym_layer, self.bound_box.w//2, RoundMode.NEAREST)

        ym_clk_list = []
        for xm_clk in [sam_e_xm, sam_xm, sam_b_xm]:
            ym_clk_list.append(self.connect_to_tracks(xm_clk, TrackID(ym_layer, ymid_tidx, tr_w_clk_ym)))

        for pinname in sig_sampler_p.port_names_iter():
            if 'out' in pinname:
                ppin = pinname.replace('out', 'out_p')
                npin = pinname.replace('out', 'out_n')
                self.reexport(sig_sampler_n.get_port(pinname), net_name=npin)
                self.reexport(sig_sampler_p.get_port(pinname), net_name=ppin)

        self.add_pin('sam', ym_clk_list[1])
        self.add_pin('sam_b', ym_clk_list[2])
        self.add_pin('sam_e', ym_clk_list[0])
        self.add_pin('sam_e_lower', cmsw_sam_xm, hide=True)
        self.add_pin('sam_e_b', sam_e_b_ym_ret, mode=PinMode.UPPER)
        # self.add_pin('sam', [sam_l_ym, sam_r_ym])
        self.add_pin('out_n_bot', cm_sw_n_xm1 + [mid_sw_n_xm1], connect=True)
        self.add_pin('out_p_bot', cm_sw_p_xm1 + [mid_sw_p_xm1], connect=True)

        self.reexport(sig_sampler_n.get_port('in'), net_name='sig_n')
        self.reexport(sig_sampler_p.get_port('in'), net_name='sig_p')

        vdd_sampler = sig_sampler_n.get_all_port_pins('VDD', layer=top_layer_btstrp) + \
                      sig_sampler_p.get_all_port_pins('VDD', layer=top_layer_btstrp)
        vss_sampler = sig_sampler_n.get_all_port_pins('VSS', layer=top_layer_btstrp) + \
                      sig_sampler_p.get_all_port_pins('VSS', layer=top_layer_btstrp)
        vdd_sampler = [vdd for vdd in vdd_sampler if vdd.bound_box.yl > y_cm_sampler]
        vss_sampler = [vss for vss in vss_sampler if vss.bound_box.yl > y_cm_sampler]

        self.connect_wires(vss_sampler)
        self.connect_wires(vdd_sampler)

        vdd_topm = sig_sampler_n.get_all_port_pins('VDD', layer=top_layer_btstrp)
        vss_topm = sig_sampler_n.get_all_port_pins('VSS', layer=top_layer_btstrp)
        vcm_topm2 = [cm_sampler.get_pin('sig_n'), cm_sampler.get_pin('sig_p')]
        vcm_tid_topm2 = {warr.track_id for warr in vdd_topm + vss_topm if warr.bound_box.yh < y_cm_sampler}
        vcm_topm = [self.connect_to_tracks(vcm_topm2, tid) for tid in vcm_tid_topm2]

        vdd_topm, vss_topm = vdd_sampler, vss_sampler

        # self.add_pin('vcm', vcm_topm, hide=True)
        cmsw_params = dict()
        cmsw_params['n'] = mid_sw_master.sch_params['n'].to_dict()
        cmsw_params['p'] = mid_sw_master.sch_params['p'].to_dict()
        cmsw_params['n']['seg'] = 2 * cmsw_params['n']['seg']
        cmsw_params['p']['seg'] = 2 * cmsw_params['p']['seg']
        if sig_sampler_params['no_sampler']:
            self.reexport(sig_sampler_n.get_port('vg_ym'), net_name='vg_n_ym', label='vg_n')
            self.reexport(sig_sampler_p.get_port('vg_ym'), net_name='vg_p_ym', label='vg_p')
        if sig_sampler_n.has_port('off_xm1'):
            self.reexport(sig_sampler_n.get_port('off_xm1'), net_name='voff_n')
            self.reexport(sig_sampler_p.get_port('off_xm1'), net_name='voff_p')

        top_layer_supply = self.params['top_layer_supply']
        if top_layer_supply:
            top_sup_layer = top_layer_btstrp + 1
            tr_w_sup_top = tr_manager.get_width(top_sup_layer, 'sup')
            tr_sp_sup_top = tr_manager.get_sep(top_sup_layer, ('sup', 'sup'))
            if self.grid.get_direction(top_sup_layer) == Orient2D.y:
                top_coord_mid = (self.bound_box.xl + self.bound_box.xh) // 2
                top_coord_upper = self.bound_box.xh
                top_upper = self.bound_box.yh
            else:
                top_coord_mid = (self.bound_box.yl + self.bound_box.yh) // 2
                top_coord_upper = self.bound_box.yh
                top_upper = self.bound_box.xh
                top_cm_tid = self.grid.coord_to_track(top_sup_layer, top_coord_mid)
                vcm_top = self.connect_to_tracks(vcm_topm, TrackID(top_layer_btstrp + 1, top_cm_tid, tr_w_sup_top))
                top_cm_lower = vcm_top.bound_box.xl if self.grid.get_direction(
                    top_sup_layer) == Orient2D.y else vcm_top.bound_box.yl
                top_cm_upper = vcm_top.bound_box.xh if self.grid.get_direction(
                    top_sup_layer) == Orient2D.y else vcm_top.bound_box.yh
                top_locs = self.get_tids_between(top_sup_layer,
                                                 self.grid.coord_to_track(top_sup_layer, 0, RoundMode.GREATER),
                                                 self.grid.coord_to_track(top_sup_layer, top_cm_lower, RoundMode.LESS),
                                                 tr_w_sup_top, tr_sp_sup_top, 0, False, mod=2)
                vss_top_list = [self.connect_to_tracks(sig_sampler_n.get_all_port_pins('VSS', layer=top_layer_btstrp) + \
                                                       cm_sampler.get_all_port_pins('VSS', layer=top_layer_btstrp),
                                                       tidx, track_lower=0,
                                                       track_upper=top_upper) for tidx in top_locs[1::2]]
                vdd_top_list = [self.connect_to_tracks(sig_sampler_n.get_all_port_pins('VDD', layer=top_layer_btstrp) + \
                                                       cm_sampler.get_all_port_pins('VDD', layer=top_layer_btstrp),
                                                       tidx, track_lower=0,
                                                       track_upper=top_upper) for tidx in top_locs[0::2]]
                top_locs = self.get_tids_between(top_sup_layer,
                                                 self.grid.coord_to_track(top_sup_layer, top_cm_upper,
                                                                          RoundMode.GREATER),
                                                 self.grid.coord_to_track(top_sup_layer, top_coord_upper,
                                                                          RoundMode.LESS),
                                                 tr_w_sup_top, tr_sp_sup_top, 0, False, True, mod=2)[::-1]
                vss_top_list += [self.connect_to_tracks(sig_sampler_p.get_all_port_pins('VSS', layer=top_layer_btstrp) + \
                                                        cm_sampler.get_all_port_pins('VSS', layer=top_layer_btstrp),
                                                        tidx, track_lower=0,
                                                        track_upper=top_upper) for tidx in top_locs[1::2]]
                vdd_top_list += [self.connect_to_tracks(sig_sampler_p.get_all_port_pins('VDD', layer=top_layer_btstrp) + \
                                                        cm_sampler.get_all_port_pins('VDD', layer=top_layer_btstrp),
                                                        tidx, track_lower=0,
                                                        track_upper=top_upper) for tidx in top_locs[0::2]]
                self.add_pin('VSS', vss_top_list)
                self.add_pin('VDD', vdd_top_list)
                self.add_pin('vcm', vcm_top)
        else:
            self.add_pin('VSS', vss_topm, connect=True)
            self.add_pin('VDD', vdd_topm, connect=True)
            self.add_pin('vcm', vcm_topm2, connect=True)

        self._sch_params = dict(
            cm_sw_params=cmsw_params,
            sig_sampler_params=sig_sampler_n_master.sch_params,
            vcm_sampler_params=vcm_sampler_master.sch_params,
        )
