#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#
# Copyright 2017 Leonard Göhrs.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

from gnuradio import gr, blocks

class fqsweep_corr(gr.hier_block2):
    """
    docstring for block fqsweep_corr
    """
    def __init__(self, preamble_length):
        gr.hier_block2.__init__(
            self, "fqsweep_corr",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(2, 2, gr.sizeof_gr_complex)
        )

        half_length= int(preamble_length)/2

        # Blocks
        self.equiv_delay= blocks.delay(gr.sizeof_gr_complex, preamble_length)

        self.fqcal_delay= blocks.delay(gr.sizeof_gr_complex, 1)
        self.fqcal_cmul= blocks.multiply_conjugate_cc(1)

        self.fqcmp_delay= blocks.delay(gr.sizeof_gr_complex, half_length)
        self.fqcmp_mul= blocks.multiply_vcc(1)

        self.avg= blocks.moving_average_cc(half_length, 1, 4096)

        # Connections
        self.connect((self, 0), (self.fqcal_delay, 0))
        self.connect((self, 0), (self.fqcal_cmul, 0))
        self.connect((self, 0), (self.equiv_delay, 0))

        self.connect((self.fqcal_delay, 0), (self.fqcal_cmul, 1))

        self.connect((self.fqcal_cmul, 0), (self.fqcmp_delay, 0))
        self.connect((self.fqcal_cmul, 0), (self.fqcmp_mul, 0))

        self.connect((self.fqcmp_delay, 0), (self.fqcmp_mul, 1))

        self.connect((self.fqcmp_mul, 0), (self.avg, 0))

        self.connect((self.equiv_delay, 0), (self, 0))
        self.connect((self.avg, 0), (self, 1))
