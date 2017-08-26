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
#include "sc_tagger_impl.h"

namespace gr {
  namespace xfdm_sync {

    sc_tagger::sptr
    sc_tagger::make(float thres_low, float thres_high, int seq_len)
    {
      return gnuradio::get_initial_sptr(new sc_tagger_impl(thres_low, thres_high, seq_len));
    }

    sc_tagger_impl::sc_tagger_impl(float thres_low, float thres_high, int seq_len)
      : gr::sync_block("sc_tagger",
                       gr::io_signature::make(2, 2, sizeof(gr_complex)),
                       gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_thres_low_sq(thres_low * thres_low),
      d_thres_high_sq(thres_high * thres_high),
      d_seq_len(seq_len),
      d_peak_idx(0),
      d_peak({.am_inside=false})
    {
      set_history(2 * seq_len);
    }

    sc_tagger_impl::~sc_tagger_impl()
    {
    }

    int
    sc_tagger_impl::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items)
    {
      const gr_complex *in_pass_history = (const gr_complex *) input_items[0];
      const gr_complex *in_corr_history = (const gr_complex *) input_items[1];
      gr_complex *out = (gr_complex *) output_items[0];

      const gr_complex *in_pass = &in_pass_history[history()];
      const gr_complex *in_corr = &in_corr_history[history()];

      /* This block only adds tags and does not modify
       * its input */
      memcpy(out, in_pass, sizeof(gr_complex) * noutput_items);

      for(int hist_idx= 0; hist_idx<noutput_items; hist_idx++) {
        gr_complex corr= in_corr_history[hist_idx];
        float power_sq= std::norm(corr);

        /* check if we left the peak with the current sample */
        if(d_peak.am_inside && (power_sq < d_thres_low_sq)) {
          d_peak.am_inside= false;

          float corr_power= std::sqrt(d_peak.corr_pw_s);
          gr_complex rot_per_sym= std::pow(d_peak.corr, 1.0f / seq_len);
          rot_per_sym/= std::abs(rot_per_sym);

          pmt_t info= pmt::make_dict();
          pmt::dict_add(info,
                        pmt::mp("sc_corr_power"),
                        pmt::from_double(corr_power));

          pmt::dict_add(info,
                        pmt::mp("sc_rot"),
                        pmt::from_complex(rot_per_sym));

          pmt::dict_add(info,
                        pmt::mp("sc_idx"),
                        pmt::from_uint64(d_peak_idx));

          add_item_tag(0, d_peak.abs_idx,
                       pmt::mp("preamble_start"),
                       info);

          d_peak_idx++;
        }

        /* check if we entered the peak with the current sample */
        if(!d_peak.am_inside && (power_sq > d_thres_high_sq)) {
          d_peak.am_inside= true;
          d_peak.corr_pw_sq= 0;
        }

        if(d_peak.am_inside && (d_peak.corr_pw_sq < power_sq)) {
          d_peak.abs_idx= nitems_read(0) + io_idx + history();
          d_peak.corr= corr;
          d_peak.corr_pw_sq= power_sq;
        }
      }

      return noutput_items;
    }

  }
}
