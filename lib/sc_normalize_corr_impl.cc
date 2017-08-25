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
#include "sc_normalize_corr_impl.h"

namespace gr {
  namespace xfdm_sync {

    sc_normalize_corr::sptr
    sc_normalize_corr::make(int seq_len)
    {
      return gnuradio::get_initial_sptr(new sc_normalize_corr_impl(seq_len));
    }

    sc_normalize_corr_impl::sc_normalize_corr_impl(int seq_len)
      : gr::sync_block("sc_normalize_corr",
                       gr::io_signature::make(2, 2, sizeof(gr_complex)),
                       gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_win_len(2 * seq_len)
    {
      set_history(d_win_len);
    }

    sc_normalize_corr_impl::~sc_normalize_corr_impl()
    {
    }

    int
    sc_normalize_corr_impl::work(int noutput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &output_items)
    {
      const gr_complex *in_corr_history = (const gr_complex *) input_items[0];
      const gr_complex *in_ref_history = (const gr_complex *) input_items[1];
      gr_complex *out = (gr_complex *) output_items[0];

      /* Elements in the history will be referred to
       * using negative indices */
      const gr_complex *in_corr= &in_corr_history[history()];
      const gr_complex *in_ref= &in_ref_history[history()];

      int win_idx= 0;
      float window[d_win_len] = {0};
      float acc;

      /* Pre-heat window and accumulator with history-values */
      for(int io_idx= -d_win_len; io_idx < 0; io_idx++) {
        float power= std::norm(in[io_idx]);

        acc+= power;
        window[win_idx]= power;

        win_idx= (win_idx + 1) % d_win_len;
      }

      for(int io_idx= 0; io_idx < noutput_items; io_idx++) {
        float power= std::norm(in[io_idx]);

        acc-= window[win_idx];
        acc+= power;
        window[win_idx]= power;

        out[io_idx]= in[io_idx] / acc;

        win_idx= (win_idx + 1) % d_seq_len;
      }

      return noutput_items;
    }
  }
}
