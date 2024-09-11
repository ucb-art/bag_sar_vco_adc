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


from modulefinder import Module
from typing import Any, Dict, Optional, Type

from bag.design.database import ModuleDB
from bag.layout.routing import TrackID
from bag.layout.template import TemplateDB
from bag.util.immutable import Param, ImmutableSortedDict
from pybag.enum import PinMode, RoundMode, MinLenMode
from xbase.layout.enum import MOSWireType
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from ..digital import NAND2Core, InvChainCore, NOR2Core, InvCore, PassGateCore
from ..util.util import fill_tap_intv, make_flipped_tile_pattern, export_xm_sup


class Binary2ThermalDecoderRow(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._start_column = 0

    @property
    def start_column(self):
        return self._start_column

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_dec_bi2th')

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
            gate_ncol='Unit gate columns',
            low_output='Low level ouput',
            vertical_buf='Arrange buffer at the bottom',
            flipped_tile='True to make the first tile flipped'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            nbits=3,
            gate_ncol=0,
            low_output=False,
            vertical_buf=False,
            flipped_tile=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        vertical_buf: bool = self.params['vertical_buf']
        nbits = self.params['nbits']
        low_output = self.params['low_output']

        has_final_inv_row = (not bool(nbits & 1) and low_output) or (bool(nbits & 1) and not low_output)
        ntiles = int(has_final_inv_row) + int(vertical_buf) + (nbits - 1)
        if self.params['flipped_tile']:
            pinfo = make_flipped_tile_pattern(self.grid, self.params['pinfo'], ntiles)
        else:
            pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        seg_buf = seg_dict['buf']
        seg_nandnor = seg_dict['nand']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)
        nds0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nds1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pds0_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        pds1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2)

        inv_params = dict(pinfo=pinfo, seg=seg_nandnor, w_p=w_p, w_n=w_n, ridx_n=ridx_n,
                          ridx_p=ridx_p)

        inv_master = self.new_template(InvCore, params=inv_params)

        vm_ntr, vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * (2 * nbits if vertical_buf else nbits))
        vm_ncol = self.arr_info.track_to_col(vm_layer, vm_ntr)
        vm_ncol += vm_ncol & 1

        # Place inst
        cur_col = vm_ncol
        gate_ncol = self.params['gate_ncol']
        buf_list = []
        buf_master = None
        for idx in range(nbits - 1):
            buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              dual_output=True, sig_locs={'nin1': ng0_tidx if idx & 1 else ng1_tidx,
                                                          'nin0': ng1_tidx if idx & 1 else ng0_tidx,
                                                          'nout0': nds0_tidx,
                                                          'nout1': nds1_tidx,
                                                          'pout0': pds0_tidx,
                                                          'pout1': pds1_tidx,
                                                          })
            buf_master = self.new_template(InvChainCore, params=buf_params)
            if idx == 0:
                _buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                   dual_output=True, sig_locs={'nin0': pg0_tidx, 'nin1': pg1_tidx,
                                                               'pout0': pds0_tidx, 'pout1': pds1_tidx,
                                                               'nout0': nds0_tidx, 'nout1': nds1_tidx})
                _buf_master = self.new_template(InvChainCore, params=_buf_params)
                buf_list.append(self.add_tile(_buf_master, col_idx=cur_col, tile_idx=0 if vertical_buf else idx))
                buf_list.append(self.add_tile(buf_master, col_idx=cur_col + buf_master.num_cols,
                                              tile_idx=0 if vertical_buf else idx))
                cur_col += 2 * buf_master.num_cols + min_sep if vertical_buf else 0
            else:
                buf_list.append(self.add_tile(buf_master, col_idx=cur_col, tile_idx=0 if vertical_buf else idx))
                cur_col += buf_master.num_cols + min_sep if vertical_buf else 0
        buf_col_l = cur_col
        cur_col = vm_ncol if vertical_buf else vm_ncol + 2 * buf_master.num_cols + min_sep
        ngates = 2 ** nbits
        nand_start_col = self._start_column = cur_col
        nand_list_list = []
        mlm_dict = {'in0': MinLenMode.MIDDLE, 'in1': MinLenMode.MIDDLE}
        nand_params = dict(pinfo=pinfo, seg=seg_nandnor, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           vertical_sup=False, mlm=mlm_dict)
        nand_master_init = self.new_template(NAND2Core, params=nand_params)
        out_vm_tidx = [self.arr_info.col_to_track(vm_layer, seg_nandnor),
                       self.arr_info.col_to_track(vm_layer, seg_nandnor + 1)]

        param_list = []
        for idx in range(nbits - 1):
            _nand_list = []
            cur_col = nand_start_col
            for jdx in range(ngates):
                if idx == 0:
                    _sig_locs = {'nin0': ng0_tidx if jdx & 2 else ng1_tidx, 'nin1': pg0_tidx if jdx & 1 else pg1_tidx,
                                 'out': out_vm_tidx[0]}
                else:
                    _sig_locs = {'nin0': ng0_tidx if (jdx & (2 ** (idx + 1))) else ng1_tidx, 'nin1': pg1_tidx,
                                 'out': out_vm_tidx[1] if idx & 1 else out_vm_tidx[0]}
                nand_params = dict(pinfo=pinfo, seg=seg_nandnor, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                   vertical_sup=False, sig_locs=_sig_locs, mlm=mlm_dict)
                nor_params = dict(pinfo=pinfo, seg=seg_nandnor, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                  vertical_sup=False, sig_locs=_sig_locs, min_len_mode=mlm_dict)
                nand_master = self.new_template(NAND2Core, params=nand_params)
                nor_master = self.new_template(NOR2Core, params=nor_params)
                _gate_master = nor_master if idx & 1 else nand_master
                _nand_list.append(
                    self.add_tile(_gate_master, col_idx=cur_col, tile_idx=idx + 1 if vertical_buf else idx))
                if not jdx:
                    param_list.append(_gate_master.sch_params)
                fill_tap_intv(self, idx + 1 if vertical_buf else idx,
                              cur_col + _gate_master.num_cols + self.sub_sep_col,
                              cur_col + max(_gate_master.num_cols + min_sep, gate_ncol) - self.sub_sep_col)
                cur_col += max(_gate_master.num_cols + min_sep, gate_ncol)
                cur_col += cur_col & 1
            nand_list_list.append(_nand_list)

        gate_ncol = max(gate_ncol, nand_master_init.num_cols + min_sep)
        final_inv_list = []
        if has_final_inv_row:
            cur_col = nand_start_col
            for idx in range(ngates):
                final_inv_list.append(self.add_tile(inv_master, col_idx=cur_col,
                                                    tile_idx=nbits if vertical_buf else nbits - 1))
                cur_col += gate_ncol
                cur_col += cur_col & 1

        self.set_mos_size()
        # fill first row

        fill_tap_intv(self, 0, buf_col_l + self.sub_sep_col, self.num_cols - self.sub_sep_col)
        for idx in range(1, nbits - 1):
            for jdx in range(ngates):
                self.connect_to_track_wires(nand_list_list[idx - 1][jdx].get_pin('out'),
                                            nand_list_list[idx][jdx].get_pin('nin<1>'))
        if has_final_inv_row:
            for idx in range(ngates):
                self.connect_to_track_wires(nand_list_list[-1][idx].get_pin('out'), final_inv_list[idx].get_pin('nin'))

        buf_conn_list, buf_b_conn_list = [], []
        if vertical_buf:
            xm_layer = vm_layer + 1
            tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
            tile_ymid = self.get_tile_info(0)[1] + self.get_tile_info(0)[0].height // 2
            _, xm_locs = tr_manager.place_wires(xm_layer, ['sig'] * (2 * nbits), center_coord=tile_ymid)
            for idx in range(nbits):
                _xm1, _xm2 = xm_locs[2 * idx], xm_locs[2 * idx + 1]
                buf_conn_list.append(self.connect_to_tracks(buf_list[idx].get_pin('out'),
                                                            TrackID(xm_layer, _xm1, tr_w_xm)))
                buf_b_conn_list.append(self.connect_to_tracks(buf_list[idx].get_pin('outb'),
                                                              TrackID(xm_layer, _xm2, tr_w_xm)))

            buf_b_conn_list = self.connect_matching_tracks(buf_b_conn_list, vm_layer, vm_locs[::2], width=tr_w_vm)
            buf_conn_list = self.connect_matching_tracks(buf_conn_list, vm_layer, vm_locs[1::2], width=tr_w_vm)

        else:
            for idx in range(nbits):
                buf_conn_list.append(buf_list[idx].get_pin('out'))
                buf_b_conn_list.append(buf_list[idx].get_pin('outb'))

        for idx in range(1, nbits):
            buf_out, nand_in = [], []
            for jdx in range(ngates):
                nand_in.append(nand_list_list[idx - 1][jdx].get_pin('nin<0>'))
            nand_in0 = [nand_in[jdx] for jdx in range(ngates) if bool(jdx & (2 ** idx))]
            nand_in1 = [nand_in[jdx] for jdx in range(ngates) if not bool(jdx & (2 ** idx))]
            self.connect_to_track_wires(nand_in1 if idx & 1 else nand_in0, buf_b_conn_list[idx])
            self.connect_to_track_wires(nand_in0 if idx & 1 else nand_in1, buf_conn_list[idx])

        nand_in = []
        for jdx in range(ngates):
            nand_in.append(nand_list_list[0][jdx].get_pin('nin<1>'))
        nand_in0 = [nand_in[jdx] for jdx in range(ngates) if bool(jdx & 1)]
        nand_in1 = [nand_in[jdx] for jdx in range(ngates) if not bool(jdx & 1)]
        self.connect_to_track_wires(nand_in1, buf_b_conn_list[0])
        self.connect_to_track_wires(nand_in0, buf_conn_list[0])

        in_vm_list = []
        if vertical_buf:
            vm_locs = [self.grid.coord_to_track(vm_layer, buf.get_pin('in').lower, RoundMode.NEAREST)
                       for buf in buf_list]
        else:
            vm_locs = vm_locs
        for buf, loc in zip(buf_list, vm_locs):
            in_vm_list.append(self.connect_to_tracks(buf.get_pin('nin'), TrackID(vm_layer, loc, tr_w_vm),
                                                     track_lower=self.bound_box.yl))

        [self.add_pin(f'in<{idx}>', pin, mode=PinMode.LOWER) for idx, pin, in enumerate(in_vm_list)]
        for idx in range(ngates):
            final_inst = final_inv_list if has_final_inv_row else nand_list_list[-1]
            self.add_pin(f'out<{idx}>', final_inst[idx].get_pin('out'))
            self.add_pin(f'pout<{idx}>', final_inst[idx].get_pin('pout'), hide=True)
            self.add_pin(f'nout<{idx}>', final_inst[idx].get_pin('nout'), hide=True)

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [nand for nand_list in nand_list_list for nand in nand_list] + buf_list + final_inv_list
        for inst in inst_list:
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]

        self.add_pin('VDD', vdd_hm if self.params['flipped_tile'] else vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm if self.params['flipped_tile'] else vss_hm, show=self.show_pins, connect=True)

        self.sch_params = dict(
            nbits=nbits,
            unit_params=param_list,
            buf_params=buf_master.sch_params,
            has_out_buf=has_final_inv_row,
            inv_params=inv_master.sch_params
        )


class Binary2ThermalDecoderCol(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_dec_bi2th')

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
            low_output='Low output',
            flipped_tile='',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            low_output=False,
            flipped_tile=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        seg_buf = seg_dict['buf']
        seg_nand = seg_dict['nand']
        seg_inv = seg_dict['inv']
        nbits = self.params['nbits']

        ntiles = 2 + 2 ** nbits
        if self.params['flipped_tile']:
            pinfo = make_flipped_tile_pattern(self.grid, self.params['pinfo'], ntiles)
        else:
            pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=0)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=1)
        nd0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nd1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pd0_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=0)
        pd1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=1)

        buf_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          dual_output=True, sig_locs={'nin0': ng2_tidx, 'nin1': ng1_tidx,
                                                      'pout0': pd0_tidx, 'pout1': pd1_tidx,
                                                      'nout0': nd0_tidx, 'nout1': nd1_tidx}, vertical_out=False)
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          sig_locs={'nin': pg1_tidx if nbits&1 else ng1_tidx})

        buf_master = self.new_template(InvChainCore, params=buf_params)
        inv_master = self.new_template(InvCore, params=inv_params)

        cur_col = 0
        buf_list = []
        _buf_cur_cul = cur_col
        for idx in range(nbits - 1):
            buf_list.append(self.add_tile(buf_master, col_idx=_buf_cur_cul, tile_idx=0))
            if not idx:
                buf_list.append(self.add_tile(buf_master, col_idx=_buf_cur_cul, tile_idx=1))
            _buf_cur_cul += buf_master.num_cols + min_sep

        nand_list_list = []
        param_list = []
        ngates = 2 ** nbits
        ng_locs, pg_locs = {'nin0': ng0_tidx, 'nin1': ng1_tidx}, {'nin0': pg0_tidx, 'nin1': pg1_tidx}
        # if not nbits & 1:
        #    pg_locs, ng_locs = ng_locs, pg_locs
        for idx in range(nbits - 1):
            nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                               sig_locs=ng_locs if idx & 1 else pg_locs)
            nor_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                              sig_locs=ng_locs if idx & 1 else pg_locs)
            nand_master = self.new_template(NAND2Core, params=nand_params)
            nor_master = self.new_template(NOR2Core, params=nor_params)
            param_list.append(nor_master.sch_params if idx & 1 else nand_master.sch_params)
            _nand_list = []
            for jdx in range(ngates):
                _nand_list.append(self.add_tile(nor_master if idx & 1 else nand_master,
                                                tile_idx=jdx + 2, col_idx=cur_col))
            nand_list_list.append(_nand_list)
            cur_col += max(nand_master.num_cols, buf_master.num_cols) + min_sep
            cur_col += cur_col & 1

        low_output = self.params['low_output']
        has_final_inv_row = (not bool(nbits & 1) and low_output) or (bool(nbits & 1) and not low_output)
        final_inv_list = []
        if has_final_inv_row:
            for idx in range(ngates):
                final_inv_list.append(self.add_tile(inv_master, tile_idx=idx + 2, col_idx=cur_col))
        self.set_mos_size()

        for idx in range(nbits - 2):
            for jdx in range(ngates):
                self.connect_to_track_wires(nand_list_list[idx][jdx].get_pin('out'),
                                            nand_list_list[idx + 1][jdx].get_pin('nin<1>'))

        if has_final_inv_row:
            for idx in range(ngates):
                self.connect_to_track_wires(nand_list_list[-1][idx].get_pin('out'),
                                            final_inv_list[idx].get_pin('nin'))

        buf_list_out, buf_list_outb = [], []
        for idx in range(1, nbits):
            _, vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 4 if idx == 1 else ['sig'] * 2,
                                                center_coord=buf_list[idx].get_pin('pout').middle)
            if idx == 1:
                _out_list = self.connect_matching_tracks([[buf_list[0].get_pin('nout'), buf_list[0].get_pin('pout')],
                                                          [buf_list[0].get_pin('noutb'), buf_list[0].get_pin('poutb')],
                                                          [buf_list[1].get_pin('nout'), buf_list[1].get_pin('pout')],
                                                          [buf_list[1].get_pin('noutb'), buf_list[1].get_pin('poutb')]],
                                                         vm_layer, vm_locs, width=tr_w_vm)
                buf_list_out.extend([_out_list[0], _out_list[2]])
                buf_list_outb.extend([_out_list[1], _out_list[3]])
            else:
                _out_list = self.connect_matching_tracks(
                    [[buf_list[idx].get_pin('nout'), buf_list[idx].get_pin('pout')],
                     [buf_list[idx].get_pin('noutb'), buf_list[idx].get_pin('poutb')]],
                    vm_layer, vm_locs, width=tr_w_vm)
                buf_list_out.append(_out_list[0])
                buf_list_outb.append(_out_list[1])

        for idx in range(1, nbits):
            buf_out, nand_in = [], []
            for jdx in range(ngates):
                nand_in.append(nand_list_list[idx - 1][jdx].get_pin('nin<0>'))
            nand_in0 = [nand_in[jdx] for jdx in range(ngates) if bool(jdx & (2 ** idx))]
            nand_in1 = [nand_in[jdx] for jdx in range(ngates) if not bool(jdx & (2 ** idx))]
            self.connect_differential_wires(nand_in1 if idx & 1 else nand_in0, nand_in0 if idx & 1 else nand_in1,
                                            buf_list_outb[idx], buf_list_out[idx])

        nand_in = []
        for jdx in range(ngates):
            nand_in.append(nand_list_list[0][jdx].get_pin('nin<1>'))
        nand_in0 = [nand_in[jdx] for jdx in range(ngates) if bool(jdx & 1)]
        nand_in1 = [nand_in[jdx] for jdx in range(ngates) if not bool(jdx & 1)]
        self.connect_differential_wires(nand_in1, nand_in0, buf_list_outb[0], buf_list_out[0])
        vm_locs = [self.grid.coord_to_track(vm_layer, buf_list[0].get_pin('nin').lower, RoundMode.NEAREST)]
        vm_locs.append(tr_manager.get_next_track(vm_layer, vm_locs[-1], 'sig', 'sig', up=False))
        for idx in range(2, nbits):
            vm_locs.append(self.grid.coord_to_track(vm_layer, buf_list[idx].get_pin('nin').lower, RoundMode.NEAREST))

        in_vm_list = []
        for buf, loc in zip(buf_list, vm_locs):
            in_vm_list.append(self.connect_to_tracks(buf.get_pin('nin'), TrackID(vm_layer, loc, tr_w_vm),
                                                     track_lower=self.bound_box.yl))

        [self.add_pin(f'in<{idx}>', pin, mode=PinMode.LOWER) for idx, pin, in enumerate(in_vm_list)]
        for idx in range(ngates):
            final_inst = final_inv_list if has_final_inv_row else nand_list_list[-1]
            self.add_pin(f'out<{idx}>', final_inst[idx].get_pin('out'))

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [nand for nand_list in nand_list_list for nand in nand_list] + buf_list + final_inv_list
        for inst in inst_list:
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]
        #
        self.add_pin('VDD', vdd_hm if self.params['flipped_tile'] else vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm if self.params['flipped_tile'] else vss_hm, show=self.show_pins, connect=True)

        self.sch_params = dict(
            nbits=nbits,
            unit_params=param_list,
            buf_params=buf_master.sch_params,
            has_out_buf=has_final_inv_row,
            inv_params=inv_master.sch_params
        )


class PassGateUnit(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._start_column = 0

    @property
    def start_column(self):
        return self._start_column

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_mux_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            low_output=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1

        seg_inv = seg_dict['inv']
        seg_pg = seg_dict['pg']
        seg_nand = seg_dict['nand']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng0_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=0)
        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        pg0_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-1)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)
        nds0_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pds0_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2)

        pg_params = dict(pinfo=pinfo, seg=seg_pg, w_p=w_p, wn=w_n,
                         ridx_n=ridx_n, ridx_p=ridx_p, vertical_in=True, vertical_out=True)
        inv_params = dict(pinfo=pinfo, seg=seg_inv, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                          sig_locs={'nin': pg0_tidx})
        nand_params = dict(pinfo=pinfo, seg=seg_nand, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                           sig_locs={'nin0': ng1_tidx, 'nin1': pg1_tidx, 'nout': nds0_tidx, 'pout': pds0_tidx})
        # nand_master = self.new_template(NAND2Core, params=nand_params)
        nand_master = self.new_template(NAND2Core, params=nand_params)
        pg_master = self.new_template(PassGateCore, params=pg_params)
        inv_master = self.new_template(InvCore, params=inv_params)

        cur_col = 0
        nand = self.add_tile(nand_master, 0, cur_col)
        cur_col += nand_master.num_cols + min_sep
        cur_col += cur_col & 1
        pg = self.add_tile(pg_master, 0, cur_col)
        cur_col += pg_master.num_cols + min_sep
        cur_col += cur_col & 1
        inv = self.add_tile(inv_master, 0, cur_col)

        self.set_mos_size()
        # Connect enable
        self.connect_to_track_wires([pg.get_pin('enb'), inv.get_pin('nin')], nand.get_pin('out'))
        self.connect_to_track_wires(pg.get_pin('en'), inv.get_pin('out'))

        # Select signals
        sel0_vm_tidx = self.grid.coord_to_track(vm_layer, nand.get_pin('nin<0>').middle, RoundMode.NEAREST)
        sel0_vm = self.connect_to_tracks(nand.get_pin('nin<0>'), TrackID(vm_layer, sel0_vm_tidx, tr_w_vm))
        self.add_pin('sel0', sel0_vm)
        self.add_pin('sel1', nand.get_pin('nin<1>'))

        # Connect input signal
        in_vm_tidx0 = self.grid.coord_to_track(vm_layer, pg.get_pin('s').lower, RoundMode.GREATER_EQ)
        in_vm_tidx1 = self.grid.coord_to_track(vm_layer, pg.get_pin('s').upper, RoundMode.LESS_EQ)
        in_vm = [self.connect_to_tracks(pg.get_pin('s'), TrackID(vm_layer, in_vm_tidx0, tr_w_vm)),
                 self.connect_to_tracks(pg.get_pin('s'), TrackID(vm_layer, in_vm_tidx1, tr_w_vm))]
        self.add_pin('in', in_vm)

        self.reexport(pg.get_port('d'), net_name='out')

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [nand, pg, inv]
        for inst in inst_list:
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]
        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)
        self.sch_params = dict(
            inv=inv_master.sch_params,
            pg=pg_master.sch_params,
            nand=nand_master.sch_params,
        )


class NPMux(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._start_column = 0

    @property
    def start_column(self):
        return self._start_column

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_np_mux')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            pside='',
            enable='True to add enable mux'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            pside=False,
            enable=True,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']
        pside: bool = self.params['pside']
        enable: bool = self.params['enable']

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1

        seg_n = seg_dict['n']
        seg_p = seg_dict['p']
        seg_pg = seg_dict['pg']
        seg_inv = seg_dict['inv']
        seg_nor = seg_dict['nor']
        seg_buf = seg_dict['buf']
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
        ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
        pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)
        nds1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
        nds2_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
        pds1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1)
        pds2_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2)

        buf_cm_params = dict(pinfo=pinfo, seg_list=seg_buf, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             sig_locs={'nin': ng1_tidx, 'nout0': nds1_tidx, 'nout1': nds2_tidx,
                                       'pout0': pds1_tidx, 'pout1': pds2_tidx}, dual_output=True)
        seg_buf_gen = seg_buf.to_list() + [seg_buf[-1]] if pside else seg_buf
        buf_np_params = dict(pinfo=pinfo, seg_list=seg_buf_gen, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                             sig_locs={'nin0': pg1_tidx, 'nin1': ng1_tidx, 'nout0': nds1_tidx, 'nout1': nds2_tidx,
                                       'pout0': pds1_tidx, 'pout1': pds2_tidx})
        pg_params = dict(pinfo=pinfo, seg=seg_pg, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                         sig_locs={'nd': nds2_tidx, 'pd': pds1_tidx}, vertical_sup=False)
        pg_np_params = dict(pinfo=pinfo, seg=seg_pg, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                            sig_locs={'en': ng1_tidx, 'enb': pg1_tidx}, vertical_sup=False)
        buf_cm_master = self.new_template(InvChainCore, params=buf_cm_params)
        buf_np_master = self.new_template(InvChainCore, params=buf_np_params)
        pg_master = self.new_template(PassGateCore, params=pg_params)
        pg_np_master = self.new_template(PassGateCore, params=pg_np_params)
        cur_col = 0
        if enable:
            inv_en = self.add_tile(buf_cm_master, 0, cur_col)
            cur_col += buf_cm_master.num_cols + min_sep
            pg_pd = self.add_tile(pg_master, 0, cur_col)
            cur_col += pg_master.num_cols
            cur_col += cur_col & 1
            pg_en = self.add_tile(pg_np_master, 0, cur_col)
            cur_col += pg_master.num_cols + 4 * min_sep
            cur_col += cur_col & 1
        else:
            pg_en, pg_pd, inv_en = None, None, None

        pg_cm = self.add_tile(pg_master, 0, cur_col)
        cur_col += pg_master.num_cols
        cur_col += cur_col & 1
        pg_np = self.add_tile(pg_np_master, 0, cur_col)
        cur_col += pg_master.num_cols + min_sep
        cur_col += cur_col & 1
        nsw = self.add_mos(0, cur_col, seg_n)
        psw = self.add_mos(1, cur_col, seg_p)
        cur_col += max(seg_n, seg_p) + min_sep
        cur_col += cur_col & 1
        buf_np = self.add_tile(buf_np_master, 0, buf_np_master.num_cols + cur_col, flip_lr=True)
        cur_col += buf_np_master.num_cols
        cur_col += cur_col & 1
        buf_cm = self.add_tile(buf_cm_master, 0, buf_cm_master.num_cols + cur_col, flip_lr=True)
        cur_col += buf_cm_master.num_cols + min_sep
        cur_col += cur_col & 1
        self.set_mos_size()
        # nor = self.add_tile(nor_master, 0, cur_col)
        # cur_col += nor_master.num_cols + min_sep
        # cur_col += cur_col & 1
        # inv = self.add_tile(inv_master, 0, cur_col)

        # self.connect_to_track_wires(inv.get_pin('out'), nor.get_pin('nin<1>'))
        sel_cm_hm = self.connect_wires(buf_cm.get_pin('nin'))
        self.connect_to_track_wires(buf_cm.get_pin('out'), pg_cm.get_pin('en'))
        # self.connect_to_track_wires(nor.get_pin('out'), buf_np.get_pin('nin'))
        self.connect_to_track_wires(buf_cm.get_pin('outb'), pg_cm.get_pin('enb'))

        _, pg_np_en_vm_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 2, center_coord=pg_np.bound_box.xh)
        pg_np_en, pg_np_enb = self.connect_matching_tracks([pg_np.get_pin('en'), pg_np.get_pin('enb')],
                                                           vm_layer, pg_np_en_vm_tidx)
        self.connect_to_track_wires(pg_cm.get_pin('enb'), pg_np_en)
        self.connect_to_track_wires(pg_cm.get_pin('en'), pg_np_enb)

        self.connect_to_track_wires([nsw.s], pg_np.get_pin('nd'))
        self.connect_to_track_wires([psw.s], pg_np.get_pin('pd'))
        en = self.connect_to_tracks([nsw.g, psw.g], self.get_track_id(0 if pside else 1, MOSWireType.G, 'sig',
                                                                      1 if pside else -2))
        nin = self.connect_to_tracks(nsw.d, self.get_track_id(0, MOSWireType.DS, 'sig', 0))
        pin = self.connect_to_tracks(psw.d, self.get_track_id(1, MOSWireType.DS, 'sig', -1))

        self.connect_to_track_wires(en, buf_np.get_pin('outb') if pside else buf_np.get_pin('out'))

        _, inout_vm_locs = tr_manager.place_wires(vm_layer, ['sig'] * 2, center_coord=nin.middle)
        inp_vm = self.connect_to_tracks(pin, TrackID(vm_layer, inout_vm_locs[1], tr_manager.get_width(vm_layer, 'sig')),
                                        min_len_mode=MinLenMode.MIDDLE)
        inn_vm = self.connect_to_tracks(nin, TrackID(vm_layer, inout_vm_locs[0], tr_manager.get_width(vm_layer, 'sig')),
                                        min_len_mode=MinLenMode.MIDDLE)
        inp_xm_tidx = self.grid.coord_to_track(xm_layer, inp_vm.middle, RoundMode.NEAREST)
        inn_xm_tidx = self.grid.coord_to_track(xm_layer, inn_vm.middle, RoundMode.NEAREST)
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        _, [out_xm_tidx, sel_cm_xm_tidx, en_xm_tidx, sel_xm_tidx, incm_xm_tidx] = \
            tr_manager.place_wires(xm_layer, ['bias'] + ['sig'] * 3 + ['bias'],
                                   center_coord=(self.bound_box.yl + self.bound_box.yh) // 2)
        sel_vm_tidx = self.arr_info.col_to_track(vm_layer, self.num_cols)
        sel_vm = self.connect_to_tracks(buf_np.get_pin('nin'), TrackID(vm_layer, sel_vm_tidx, tr_w_vm))
        sel_xm = self.connect_to_tracks(sel_vm, TrackID(xm_layer, sel_xm_tidx, tr_w_xm))

        sel_cm_vm_tidx = self.grid.coord_to_track(vm_layer, sel_cm_hm[0].lower, RoundMode.LESS)
        sel_cm_vm_tidx = tr_manager.get_next_track(vm_layer, sel_cm_vm_tidx, 'sig', 'sig', up=False)
        sel_cm_vm = self.connect_to_tracks(sel_cm_hm, TrackID(vm_layer, sel_cm_vm_tidx, tr_w_vm))
        sel_cm_xm = self.connect_to_tracks(sel_cm_vm, TrackID(xm_layer, sel_cm_xm_tidx, tr_w_xm))
        out_vm_tidx = self.grid.coord_to_track(vm_layer, pg_cm.bound_box.xh, RoundMode.NEAREST)
        tr_w_vm_bias = tr_manager.get_width(vm_layer, 'bias')
        out_vm = self.connect_to_tracks([pg_np.get_pin('s'), pg_cm.get_pin('s')],
                                        TrackID(vm_layer, out_vm_tidx, tr_w_vm_bias), min_len_mode=MinLenMode.MIDDLE)

        if enable:
            self.connect_to_track_wires(inv_en.get_pin('out'), pg_en.get_pin('en'))
            self.connect_to_track_wires(inv_en.get_pin('outb'), pg_en.get_pin('enb'))

            _, pg_en_en_vm_tidx = tr_manager.place_wires(vm_layer, ['sig'] * 2, center_coord=pg_pd.bound_box.xl)
            pg_en_en, pg_en_enb = self.connect_matching_tracks([pg_en.get_pin('en'), pg_en.get_pin('enb')],
                                                               vm_layer, pg_en_en_vm_tidx)
            self.connect_to_track_wires(pg_pd.get_pin('enb'), pg_en_en)
            self.connect_to_track_wires(pg_pd.get_pin('en'), pg_en_enb)

            enable_vm_tidx = self.grid.coord_to_track(vm_layer, inv_en.get_pin('nin').lower, RoundMode.LESS)
            enable_vm_tidx = tr_manager.get_next_track(vm_layer, enable_vm_tidx, 'sig', 'sig', up=False)
            enable_vm = self.connect_to_tracks(inv_en.get_pin('nin'), TrackID(vm_layer, enable_vm_tidx, tr_w_vm))
            enable_xm = self.connect_to_tracks(enable_vm, TrackID(xm_layer, en_xm_tidx, tr_w_xm))
            en_out_vm_tidx = self.grid.coord_to_track(vm_layer, pg_pd.bound_box.xh, RoundMode.NEAREST)
            tr_w_vm_bias = tr_manager.get_width(vm_layer, 'bias')
            out_vm = self.connect_to_tracks([pg_en.get_pin('s'), pg_pd.get_pin('s')],
                                            TrackID(vm_layer, en_out_vm_tidx, tr_w_vm_bias))

            self.connect_to_track_wires(pg_pd.get_pin('d'), pg_pd.get_pin('VSS'))
            # connect output pg_en
            dac_out_vm_tidx = self.grid.coord_to_track(vm_layer, pg_cm.get_pin('s').lower, RoundMode.NEAREST)
            self.connect_to_tracks([pg_cm.get_pin('s'), pg_en.get_pin('nd'), pg_en.get_pin('pd')],
                                   TrackID(vm_layer, dac_out_vm_tidx))
            self.add_pin('en', enable_xm)

        # out_xm_tidx = self.grid.coord_to_track(xm_layer, pg.get_pin('d').lower, RoundMode.NEAREST)
        # incm_xm_tidx = self.grid.coord_to_track(xm_layer, pg.get_pin('d').upper, RoundMode.NEAREST)
        tr_w_xm_bias = tr_manager.get_width(xm_layer, 'bias')
        out_xm = self.connect_to_tracks(out_vm, TrackID(xm_layer, out_xm_tidx, tr_w_xm_bias))
        incm_xm = self.connect_to_tracks(pg_cm.get_pin('d'), TrackID(xm_layer, incm_xm_tidx, tr_w_xm_bias))
        self.add_pin('inp', inp_vm)
        self.add_pin('inn', inn_vm)
        self.add_pin('sel', sel_xm)
        self.add_pin('sel_cm', sel_cm_xm)
        self.add_pin('out', out_xm)
        self.add_pin('incm', incm_xm)
        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [buf_np, buf_cm, pg_cm, pg_np]
        for inst in inst_list:
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]

        self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)
        # sup_bot, sup_top = export_xm_sup(self, tile_idx=self.num_tile_rows - 1, export_bot=True, export_top=True)
        # self.add_pin('VDD', sup_top, connect=True)
        # self.add_pin('VSS', sup_bot, connect=True)
        default_wp = self.place_info.get_row_place_info(ridx_p).row_info.width
        default_wn = self.place_info.get_row_place_info(ridx_n).row_info.width
        thp = self.place_info.get_row_place_info(ridx_p).row_info.threshold
        thn = self.place_info.get_row_place_info(ridx_n).row_info.threshold
        self.sch_params = dict(
            pside=pside,
            buf_cm=buf_cm_master.sch_params,
            buf_np=buf_np_master.sch_params,
            # inv=inv_master.sch_params,
            # nor=nor_master.sch_params,
            pg=pg_master.sch_params,
            nsw=dict(
                nf=seg_p,
                l=self.arr_info.lch,
                w=default_wn,
                intent=thn,
            ),
            psw=dict(
                nf=seg_n,
                l=self.arr_info.lch,
                w=default_wp,
                intent=thp,
            ),
        )


class PassGateArray(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._unit_ncol = 0

    @property
    def unit_ncol(self):
        return self._unit_ncol

    # @classmethod
    # def get_schematic_class(cls) -> Optional[Type[Module]]:
    #     # noinspection PyTypeChecker
    #     return ModuleDB.get_schematic_class('bag_vco_adc', 'ra_bias_unit')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Number of segments.',
            w_n='nmos width',
            w_p='pmos width',
            ridx_n='index for nmos row',
            ridx_p='index for pmos row',
            nx='Number of columns',
            ny='Number of Rows',
            flipped_tile=''
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            flipped_tile=False,
        )

    def draw_layout(self) -> None:
        # setup floorplan
        w_n: int = self.params['w_n']
        w_p: int = self.params['w_p']
        ridx_n: int = self.params['ridx_n']
        ridx_p: int = self.params['ridx_p']
        seg_dict: Dict[str, Any] = self.params['seg_dict']

        nx: int = self.params['nx']
        ny: int = self.params['ny']

        if self.params['flipped_tile']:
            pinfo = make_flipped_tile_pattern(self.grid, self.params['pinfo'], ny)
        else:
            pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        cur_col = 0
        pg_list_list = []
        pg_master = self.new_template(PassGateUnit, params=self.params)
        self._unit_ncol = pg_master.num_cols + min_sep
        for idx in range(nx):
            _pg_list = []
            for jdx in range(ny):
                _pg_list.append(self.add_tile(pg_master, col_idx=cur_col, tile_idx=jdx))
            pg_list_list.append(_pg_list)
            cur_col += pg_master.num_cols + min_sep
            cur_col += cur_col & 1

        self.set_mos_size()
        sel0_list, sel1_list = [], []
        out_list = []
        for idx in range(nx):
            sel0_list.append(self.connect_wires([inst.get_pin('sel0') for inst in pg_list_list[idx]])[0])
            out_list.append(self.connect_wires([inst.get_pin('out') for inst in pg_list_list[idx]])[0])

        for idx in range(ny):
            sel1_list.append(self.connect_wires([pg_list_list[jdx][idx].get_pin('sel1') for jdx in range(nx)])[0])

        # xm_layer = vm_layer + 1
        # tr_w_xm = tr_manager.get_width(xm_layer, 'sig')
        # out_xm_tidx = self.grid.coord_to_track(xm_layer, out_list[0].upper, RoundMode.NEAREST)
        # out_xm = self.connect_to_tracks(out_list, TrackID(xm_layer, out_xm_tidx, tr_w_xm))
        self.add_pin('out', out_list, mode=PinMode.UPPER)
        sel0_list = self.extend_wires(sel0_list, lower=self.bound_box.yl)
        sel1_list = self.extend_wires(sel1_list, upper=self.bound_box.xh)
        [self.add_pin(f'sel0<{idx}>', pin, mode=PinMode.LOWER) for idx, pin in enumerate(sel0_list)]
        [self.add_pin(f'sel1<{idx}>', pin, mode=PinMode.UPPER) for idx, pin in enumerate(sel1_list)]
        for idx in range(nx):
            for jdx in range(ny):
                self.add_pin(f'in<{jdx * nx + idx}>', pg_list_list[idx][jdx].get_all_port_pins('in'))

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [pg for pg_list in pg_list_list for pg in pg_list]
        for inst in inst_list:
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]
        self.add_pin('VDD', vdd_hm, connect=True)
        self.add_pin('VSS', vss_hm, connect=True)

        # self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)
        # self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)
        self.sch_params = pg_master.sch_params


class RDACMuxDecoder(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)
        self._num_bits = 0
        self._mux_start_row = 0

    @property
    def num_bits(self):
        return self._num_bits

    @property
    def mux_start_row(self):
        return self._mux_start_row

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        # noinspection PyTypeChecker
        return ModuleDB.get_schematic_class('bag_vco_adc', 'rdac_dec')

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            mux_params='Number of segments.',
            dec_row_params='Number of segments.',
            dec_col_params='Number of segments.',
            npmux_params='',
            ntile_match='',
            differential='',
            pside='',
            enable='True to add enable',
            off_low='True to pull down output when off',
            float_output='',
            # off_float='True to fload output'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            w_n=4,
            w_p=4,
            ridx_n=0,
            ridx_p=-1,
            ntile_match=0,
            differential=True,
            float_output=False,
            pside=False,
            enable=True,
            off_low=True,
        )

    def draw_layout(self) -> None:
        mux_params: ImmutableSortedDict = self.params['mux_params']
        dec_col_params: ImmutableSortedDict = self.params['dec_col_params']
        dec_row_params: ImmutableSortedDict = self.params['dec_row_params']
        npmux_params: ImmutableSortedDict = self.params['npmux_params']
        ntile_match: int = self.params['ntile_match']
        differential: bool = self.params['differential']
        float_output: bool = self.params['float_output']

        pinfo = self.params['pinfo']
        mux_gen_params = mux_params.copy(append=dict(pinfo=pinfo, flipped_tile=False))
        npmux_gen_params = npmux_params.copy(append=dict(pinfo=pinfo, pside=self.params['pside']))
        mux_master = self.new_template(PassGateArray, params=mux_gen_params)
        npmux_master = self.new_template(NPMux, params=npmux_gen_params)
        dec_col_gen_params = dec_col_params.copy(append=dict(pinfo=pinfo, flipped_tile=False))
        dec_row_gen_params = dec_row_params.copy(append=dict(pinfo=pinfo, gate_ncol=mux_master.unit_ncol,
                                                             vertical_buf=True, flipped_tile=True, ))
        dec_col_master = self.new_template(Binary2ThermalDecoderCol, params=dec_col_gen_params)
        dec_row_master = self.new_template(Binary2ThermalDecoderRow, params=dec_row_gen_params)
        ntiles = dec_col_master.num_tile_rows + mux_master.num_tile_rows + 2

        # pinfo = make_flipped_tile_pattern(self.grid, self.params['pinfo'], ntiles)
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])

        # Some checkings
        nx_mux, ny_mux = mux_params['nx'], mux_params['ny']
        nbits_row, nbits_col = dec_row_params['nbits'], dec_col_params['nbits']
        if nx_mux != 2**nbits_row or ny_mux != 2**nbits_col:
            raise ValueError("Match the number of mux in the array with the decoder, in yaml file")

        self.draw_base(pinfo)

        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        min_sep = self.min_sep_col
        min_sep += min_sep & 1

        cur_col = 0
        cur_tile = ntile_match
        dec_row = self.add_tile(dec_row_master, tile_idx=cur_tile, col_idx=cur_col)
        mux_ntile = dec_row_master.num_tile_rows
        # mux_ntile += mux_ntile & 1
        mux = self.add_tile(mux_master, tile_idx=cur_tile + mux_ntile, col_idx=dec_row_master.start_column)
        self._mux_start_row = cur_tile + mux_ntile

        cur_col = dec_row_master.start_column + mux_master.num_cols
        dec_col_col = cur_col + dec_col_master.num_cols + min_sep
        dec_col_col += dec_col_col & 1
        dec_col = self.add_tile(dec_col_master,
                                tile_idx=cur_tile + mux_ntile + mux_master.num_tile_rows - dec_col_master.num_tile_rows,
                                col_idx=dec_col_col, flip_lr=True)
        if differential:
            npmux = self.add_tile(npmux_master, tile_idx=cur_tile + mux_ntile + mux_master.num_tile_rows + 1,
                                  col_idx=self.sub_sep_col)
            self.set_mos_size(num_tiles=self.num_tile_rows, num_cols=self.num_cols + (self.num_cols & 1))
        else:
            npmux = None
            self.set_mos_size(num_tiles=self.num_tile_rows + 1)

        tr_manager = self.tr_manager
        out_xm_locs_mid_coord = self.get_tile_info(self.num_tile_rows - 2)[1] + \
                                self.get_tile_info(self.num_tile_rows - 2)[0].height // 2
        _, out_xm_locs = tr_manager.place_wires(xm_layer, ['bias'] * 2, center_coord=out_xm_locs_mid_coord)

        for idx in range(ntile_match, self.num_tile_rows - 2):
            fill_tap_intv(self, idx, self.sub_sep_col, dec_row_master.start_column - self.sub_sep_col)

        if differential:
            fill_tap_intv(self, self.num_tile_rows - 1, npmux_master.num_cols + 2 * self.sub_sep_col,
                          self.num_cols - self.sub_sep_col)
            fill_tap_intv(self, self.num_tile_rows - 2, self.sub_sep_col, self.num_cols - self.sub_sep_col)
        fill_tap_intv(self, 0, self.sub_sep_col, self.num_cols - self.sub_sep_col)
        fill_tap_intv(self, 1, dec_col_col + self.sub_sep_col, self.num_cols - self.sub_sep_col)

        nx = 2 ** dec_row_params['nbits']
        ny = 2 ** dec_col_params['nbits']
        for idx in range(nx):
            self.connect_to_track_wires([dec_row.get_pin(f'nout<{idx}>'), dec_row.get_pin(f'pout<{idx}>')],
                                        mux.get_pin(f'sel0<{idx}>'))
        for idx in range(ny):
            self.connect_to_track_wires(dec_col.get_pin(f'out<{idx}>'), mux.get_pin(f'sel1<{idx}>'))

        # compute track locations
        tr_manager = self.tr_manager
        tr_w_vm = tr_manager.get_width(vm_layer, 'sig')
        tr_sp_vm = tr_manager.get_sep(vm_layer, ('sig', 'sig'))

        self._num_bits = dec_row_params['nbits'] + dec_col_params['nbits']

        for idx in range(nx * ny):
            self.reexport(mux.get_port(f'in<{idx}>'), net_name=f'tap<{idx}>')

        nlsb = dec_row_master.params['nbits']
        nmsb = dec_col_master.params['nbits']
        bit_list = []
        for idx in range(nlsb):
            bit_list.append(dec_row.get_pin(f'in<{idx}>'))
        for idx in range(nmsb):
            bit_list.append(dec_col.get_pin(f'in<{idx}>'))
        bit_in_xm_coord = self.get_tile_info(0)[1] + self.get_tile_info(0)[0].height // 2
        _, xm_locs = tr_manager.place_wires(xm_layer, ['sig'] * (nlsb + nmsb), center_coord=bit_in_xm_coord)

        bit_xm_list = []
        tr_w_xm = tr_manager.get_width(xm_layer, 'bias')
        tr_w_xm_sig = tr_manager.get_width(xm_layer, 'sig')
        tr_w_ym_sig = tr_manager.get_width(ym_layer, 'sig')
        for locs, bit in zip(xm_locs, bit_list):
            bit_xm_list.append(self.connect_to_tracks(bit, TrackID(xm_layer, locs, tr_w_xm_sig)))

        [self.add_pin(f'bit<{idx}>', pin) for idx, pin in enumerate(bit_xm_list)]

        enable = self.params['enable']
        if differential:
            inn_xm = self.connect_to_tracks(npmux.get_pin('inn'), TrackID(xm_layer, out_xm_locs[0], tr_w_xm_sig))
            inp_xm = self.connect_to_tracks(npmux.get_pin('inp'), TrackID(xm_layer, out_xm_locs[1], tr_w_xm_sig))

            if self.params['pside']:
                self.connect_to_track_wires(inp_xm, mux.get_all_port_pins('out'))
                self.add_pin('mux_out_n', inn_xm)
                self.add_pin('mux_out', inp_xm)
            else:
                self.connect_to_track_wires(inn_xm, mux.get_all_port_pins('out'))
                self.add_pin('mux_out_n', inp_xm)
                self.add_pin('mux_out', inn_xm)

            self.reexport(npmux.get_port('out'), net_name='out')
        else:
            out_xm_coord = (self.get_tile_info(self.num_tile_rows)[1] + self.get_tile_info(self.num_tile_rows - 1)[
                1]) // 2
            if enable:
                ridx_n, ridx_p = 0, -1
                w_p, w_n = 4, 4
                nds1_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=0)
                nds2_tidx = self.get_track_index(ridx_n, MOSWireType.DS, wire_name='sig', wire_idx=1)
                pds1_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-1)
                pds2_tidx = self.get_track_index(ridx_p, MOSWireType.DS, wire_name='sig', wire_idx=-2)
                ng1_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=1)
                ng2_tidx = self.get_track_index(ridx_n, MOSWireType.G, wire_name='sig', wire_idx=2)
                pg1_tidx = self.get_track_index(ridx_p, MOSWireType.G, wire_name='sig', wire_idx=-2)

                buf_params = dict(pinfo=pinfo, seg_list=[2, 2], w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                  sig_locs={'nin': ng1_tidx, 'nout0': nds1_tidx, 'nout1': nds2_tidx,
                                            'pout0': pds1_tidx, 'pout1': pds2_tidx}, dual_output=True)
                pg_params = dict(pinfo=pinfo, seg=4, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                 sig_locs={'nd': nds1_tidx, 'pd': pds1_tidx})
                pg_off_params = dict(pinfo=pinfo, seg=4, w_p=w_p, w_n=w_n, ridx_n=ridx_n, ridx_p=ridx_p,
                                     sig_locs={'en': ng2_tidx, 'enb': pg1_tidx})

                pg_en_master = self.new_template(PassGateCore, params=pg_params)
                pg_off_master = self.new_template(PassGateCore, params=pg_off_params)
                buf_master = self.new_template(InvChainCore, params=buf_params)
                pg_col = dec_col_col - dec_col_master.num_cols
                fill_tap_intv(self, self.num_tile_rows - 1, self.sub_sep_col, pg_col - 2 * self.sub_sep_col)
                if not float_output:
                    pg_off = self.add_tile(pg_off_master, tile_idx=self.num_tile_rows - 1, col_idx=pg_col)
                    pg_col += pg_off_master.num_cols + min_sep
                else:
                    pg_off = None
                pg_en = self.add_tile(pg_en_master, tile_idx=self.num_tile_rows - 1, col_idx=pg_col)
                en_col = pg_col + pg_en_master.num_cols + min_sep + buf_master.num_cols
                buf = self.add_tile(buf_master, tile_idx=self.num_tile_rows - 1, col_idx=en_col, flip_lr=True)
                _, out_xm_tid = tr_manager.place_wires(xm_layer, ['sig', 'bias'], center_coord=out_xm_coord)
                pg_en_s_tidx = self.grid.coord_to_track(vm_layer, pg_en.bound_box.xl, RoundMode.NEAREST)
                pg_s_vm = self.connect_to_tracks(pg_en.get_pin('s'), TrackID(vm_layer, pg_en_s_tidx))
                if not float_output:
                    pg_off_s_tidx = self.grid.coord_to_track(vm_layer, pg_off.bound_box.xl, RoundMode.NEAREST)
                    pg_off_s_vm = self.connect_to_tracks(pg_off.get_pin('s'), TrackID(vm_layer, pg_off_s_tidx))
                    sup_tid = self.get_track_id(ridx_n if self.params['off_low'] else ridx_p,
                                                MOSWireType.DS, 'sup', tile_idx=self.num_tile_rows - 1)
                    self.connect_to_tracks(pg_off_s_vm, sup_tid)
                    self.connect_differential_wires(pg_off.get_pin('enb'), pg_off.get_pin('en'),
                                                    buf.get_pin('out'), buf.get_pin('outb'))
                mux_in_xm = self.connect_to_tracks(mux.get_all_port_pins('out') + [pg_s_vm],
                                                   TrackID(xm_layer, out_xm_tid[0],
                                                           tr_manager.get_width(xm_layer, 'sig')))
                out_xm = self.connect_to_tracks([pg_en.get_pin('d')] if float_output else [pg_en.get_pin('d'), pg_off.get_pin('d')],
                                                TrackID(xm_layer, out_xm_tid[1], tr_w_xm_sig))
                self.add_pin('out', out_xm)
                if float_output:
                    self.add_pin('mux_out', mux_in_xm)
                en_vm_tidx = self.grid.coord_to_track(vm_layer, buf.bound_box.xh, RoundMode.NEAREST)
                en_vm = self.connect_to_tracks(buf.get_pin('nin'), TrackID(vm_layer, en_vm_tidx),
                                               min_len_mode=MinLenMode.MIDDLE)
                self.connect_differential_wires(pg_en.get_pin('en'), pg_en.get_pin('enb'),
                                                buf.get_pin('out'), buf.get_pin('outb'))
                en_xm_tidx = self.grid.coord_to_track(xm_layer, en_vm.middle, RoundMode.NEAREST)
                en_xm = self.connect_to_tracks(en_vm, TrackID(xm_layer, en_xm_tidx, tr_w_xm_sig),
                                               min_len_mode=MinLenMode.MIDDLE)
                ym_layer = xm_layer + 1
                en_ym_tidx = self.grid.coord_to_track(ym_layer, en_xm.middle, RoundMode.NEAREST)
                en_ym = self.connect_to_tracks(en_xm, TrackID(ym_layer, en_ym_tidx, tr_w_ym_sig),
                                               min_len_mode=MinLenMode.MIDDLE, track_lower=self.bound_box.yl)
                self.add_pin('en', en_ym)
            else:
                buf_master, pg_en_master = None, None
                out_xm_tid = self.grid.coord_to_track(xm_layer, out_xm_coord, RoundMode.NEAREST)
                out_xm = self.connect_to_tracks(mux.get_all_port_pins('out'), TrackID(xm_layer, out_xm_tid,
                                                                                      tr_w_xm_sig))
                self.add_pin('out', out_xm)

        # Connection for VDD/VSS
        vss_list, vdd_list = [], []
        inst_list = [dec_col, dec_row, mux, npmux] if differential else [dec_col, dec_row, mux]
        for inst in inst_list:
            vdd_list.extend(inst.get_all_port_pins('VDD'))
            vss_list.extend(inst.get_all_port_pins('VSS'))
        vdd_hm = self.connect_wires(vdd_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vss_hm = self.connect_wires(vss_list, lower=self.bound_box.xl, upper=self.bound_box.xh)
        vdd_hm = [w for warr in vdd_hm for w in warr.to_warr_list()]
        vss_hm = [w for warr in vss_hm for w in warr.to_warr_list()]

        if differential:
            self.reexport(npmux.get_port('sel'), net_name='sel_np')
            self.reexport(npmux.get_port('sel_cm'), net_name='sel_cm')
            self.reexport(npmux.get_port('incm'), net_name='vcm')
            self.reexport(npmux.get_port('en'))

        # self.add_pin('VDD', vdd_hm, show=self.show_pins, connect=True)a
        # self.add_pin('VSS', vss_hm, show=self.show_pins, connect=True)

        vdd_xm_list, vss_xm_list = [], []
        for idx in range(self.num_tile_rows):
            sup, _ = export_xm_sup(self, tile_idx=idx, export_bot=True, export_top=False)
            if bool(idx & 1):
                vdd_xm_list.append(sup)
            else:
                vss_xm_list.append(sup)

        _, sup = export_xm_sup(self, tile_idx=self.num_tile_rows - 1, export_bot=False, export_top=True)
        if bool(self.num_tile_rows & 1):
            vdd_xm_list.append(sup)
        else:
            vss_xm_list.append(sup)
        self.add_pin('VDD', vdd_xm_list, connect=True)
        self.add_pin('VSS', vss_xm_list, connect=True)

        self.sch_params = dict(
            dec_msb_params=dec_col_master.sch_params,
            dec_lsb_params=dec_row_master.sch_params,
            mux_params=mux_master.sch_params,
            npmux_params=npmux_master.sch_params,
            pside=self.params['pside'],
            differential=differential,
            nbits=nmsb + nlsb,
            pg_params=pg_en_master.sch_params if not differential else None,
            buf_params=buf_master.sch_params if not differential else None,
            off_low=self.params['off_low'],
            float_output=float_output,
        )
