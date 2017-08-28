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

#ifndef INCLUDED_XFDM_SYNC_XCORR_TAGGER_IMPL_H
#define INCLUDED_XFDM_SYNC_XCORR_TAGGER_IMPL_H

#include <xfdm_sync/xcorr_tagger.h>

namespace gr {
  namespace xfdm_sync {

    class xcorr_tagger_impl : public xcorr_tagger
    {
    private:
      gr_complex *d_sequence_fq;

      float d_threshold;
      uint64_t d_peak_idx;
      bool d_use_sc_rot;
      int d_fft_len;

      gr::fft::fft_complex *d_fft_fwd;
      gr::fft::fft_complex *d_fft_rwd;

    public:
      xcorr_tagger_impl(float threshold, std::vector<gr_complex> sync_sequence, bool use_sc_rot);
      ~xcorr_tagger_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };
  }
}

#endif
