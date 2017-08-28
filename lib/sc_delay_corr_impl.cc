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
#include "sc_delay_corr_impl.h"

namespace gr {
  namespace xfdm_sync {

    sc_delay_corr::sptr
    sc_delay_corr::make(int seq_len, bool normalize)
    {
      return gnuradio::get_initial_sptr(new sc_delay_corr_impl(seq_len, normalize));
    }

    sc_delay_corr_impl::sc_delay_corr_impl(int seq_len, bool normalize)
      : gr::sync_block("sc_delay_corr",
                       gr::io_signature::make(1, 1, sizeof(gr_complex)),
                       gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_seq_len(seq_len),
      d_normalize(normalize)
    {
      set_history(2 * seq_len);
    }

    sc_delay_corr_impl::~sc_delay_corr_impl()
    {
    }

    int
    sc_delay_corr_impl::work(int noutput_items,
                             gr_vector_const_void_star &input_items,
                             gr_vector_void_star &output_items)
    {
      const gr_complex *in_history = (const gr_complex *) input_items[0];
      gr_complex *out_pass = (gr_complex *) output_items[0];
      gr_complex *out_corr = (gr_complex *) output_items[1];

      /* Elements in the history will be referred to
       * using negative indices */
      const gr_complex *in= &in_history[history()];

      int corr_win_len= d_seq_len;
      int corr_win_idx= 0;
      gr_complex corr_window[corr_win_len]= {0};
      gr_complex corr_acc= 0;

      int norm_win_len= 2*d_seq_len;
      int norm_win_idx= 0;
      float norm_window[norm_win_len]= {0};
      float norm_acc= 0;

      /* Pre-heat corr_window and corr_acc with history-values */
      for(int io_idx= -corr_win_len; io_idx < 0; io_idx++) {
        gr_complex conjmul= in[io_idx] * std::conj(in[io_idx - corr_win_len]);

        corr_acc+= conjmul;
        corr_window[corr_win_idx]= conjmul;

        corr_win_idx= (corr_win_idx + 1) % corr_win_len;
      }

      /* Pre-heat norm_window and norm_acc with history-values */
      for(int io_idx= -norm_win_len; io_idx < 0; io_idx++) {
        float power= std::norm(in[io_idx]);

        norm_acc+= power;
        norm_window[norm_win_idx]= power;

        norm_win_idx= (norm_win_idx + 1) % norm_win_len;
      }

      for(int io_idx= 0; io_idx < noutput_items; io_idx++) {
        gr_complex conjmul= in[io_idx] * std::conj(in[io_idx - corr_win_len]);
        float power= std::norm(in[io_idx]);

        corr_acc-= corr_window[corr_win_idx];
        corr_window[corr_win_idx]= conjmul;
        corr_acc+= conjmul;

        norm_acc-= norm_window[norm_win_idx];
        norm_window[norm_win_idx]= power;
        norm_acc+= power;

        out_corr[io_idx]= d_normalize
          ? ((norm_acc > 0) ? (2.0f * corr_acc / norm_acc) : 0)
          : corr_acc;

        out_pass[io_idx]= in[io_idx - norm_win_len];

        corr_win_idx= (corr_win_idx + 1) % corr_win_len;
        norm_win_idx= (norm_win_idx + 1) % norm_win_len;
      }

      return noutput_items;
    }
  }
}
