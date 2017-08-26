/* -*- c++ -*- */
/*
 * Copyright 2017 Leonard Göhrs.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "frame_gate_impl.h"

namespace gr {
  namespace xfdm_sync {

    frame_gate::sptr
    frame_gate::make(int len_prologue, int len_epilogue, int len_symbol,
                     int symbols_per_frame_min, int symbols_per_frame_max,
                     bool do_compensations)
    {
      return gnuradio::get_initial_sptr
        (new frame_gate_impl(len_prologue, len_epilogue, len_symbol,
                             symbols_per_frame_min, symbols_per_frame_max,
                             do_compensations));
    }

    frame_gate_impl::frame_gate_impl(int len_prologue, int len_epilogue, int len_symbol,
                                     int symbols_per_frame_min, int symbols_per_frame_max,
                                     bool do_compensations)
      : gr::sync_block("frame_gate",
                       gr::io_signature::make(1, 1, sizeof(gr_complex)),
                       gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
    }

    frame_gate_impl::~frame_gate_impl()
    {
    }

    int
    frame_gate_impl::work(int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];



      return noutput_items;
    }
  }
}
