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

#include <gnuradio/fft.h>
#include <gnuradio/io_signature.h>
#include "xcorr_tagger_impl.h"

namespace gr {
  namespace xfdm_sync {

    xcorr_tagger::sptr
    xcorr_tagger::make(float threshold, pmt_t sync_sequence, bool use_sc_rot)
    {
      return gnuradio::get_initial_sptr(new xcorr_tagger_impl(threshold, sync_sequence, use_sc_rot));
    }

    xcorr_tagger_impl::xcorr_tagger_impl(float threshold, pmt_t sync_sequence, bool use_sc_rot)
      : gr::sync_block("xcorr_tagger",
                       gr::io_signature::make(2, 2, sizeof(gr_complex)),
                       gr::io_signature::make(1, 1, sizeof(gr_complex)))
      d_threshold(threshold),
      d_peak_idx(0),
      d_use_sc_rot(use_sc_rot)
    {
      set_tag_propagation_policy(TPP_DONT);

      if(!pmt::is_c32vector(sync_sequence)) {
        throw std::runtime_error("xcorr_tagger: sync_sequence is expected to be a "\
                                 "vector of complex numbers");
      }

      int seq_len= pmt::length(sync_sequence);

      /* Find a power-of-two fft length that can fit
       * the synchronization pattern, an equally sized
       * padding and a bit of slack */
      for(d_fft_len= 4; d_fft_len < (3*seq_len); d_fft_len*=2);

      /* Allocate space for the fourier-transformed sequence */
      d_sequence_fq= new gr_complex[d_fft_len];

      /* Let the GNURadio wrapper setup some fftw contexts */
      gr::fft::fft_complex d_fft_fwd(d_fft_len, true, 1);
      gr::fft::fft_complex d_fft_rwd(d_fft_len, false, 1);

      gr_complex *fwd_in= d_fft_fwd.get_inbuf();
      gr_complex *fwd_out= d_fft_fwd.get_outbuf();

      /* Transform the referece sequence to the frequency
       * domain for fast crosscorrelation later */
      memset(fwd_in, 0, sizeof(gr_complex) * d_fft_len);

      gr_complex *sequence= pmt::pmt_c32vector_elements(sync_sequence).data();
      memcpy(fwd_in, sequence, sizeof(gr_complex) * seq_len);

      d_fft_fwd.execute();

      memcpy(d_sequence_fq.get(), fwd_out, sizeof(gr_complex) * d_fft_len);

      /* Clear the input buffer.
       * It will be assumed to be zeroed later */
      memset(fwd_in, 0, sizeof(gr_complex) * d_fft_len);
    }

    xcorr_tagger_impl::~xcorr_tagger_impl()
    {
    }

    int
    xcorr_tagger_impl::work(int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
    {
      const gr_complex *in_pass_history = (const gr_complex *) input_items[0];
      const gr_complex *in_corr_history = (const gr_complex *) input_items[1];
      gr_complex *out = (gr_complex *) output_items[0];

      const gr_complex *in_pass= &in_pass_history[history()];
      const gr_complex *in_corr= &in_corr_history[history()];

      /* This block only adds tags and does not modify
       * its input */
      memcpy(out, in_pass, sizeof(gr_complex) * noutput_items);

      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0, 0, noutput_items, pmt::mp("preamble_start"));

      for(tag_t tag: tags) {
        int64_t tag_center= (int64_t)tag.offset - (int64_t)nitems_read(0);

        pmt_t info= tag.value;

        gr_complex *fwd_in= d_fft_fwd.get_inbuf();
        gr_complex *fwd_out= d_fft_fwd.get_outbuf();
        gr_complex *rwd_in= d_fft_rwd.get_inbuf();
        gr_complex *rwd_out= d_fft_rwd.get_outbuf();
        gr_complex *seq_fqd= d_sequence_fq.get();


        /* Apply frequency offset correction based on input
         * from the sc_tagger block */
        gr_complex fq_comp_rot= 1;

        if(d_use_sc_rot) {
          pmt_t sc_rot= pmt::dict_ref(info,
                                      pmt::mp("sc_rot"),
                                      PMT_NIL);

          if(pmt::is_complex(sc_rot)) {
            fq_comp_rot= std::conj(pmt::to_complex(sc_rot));
            fq_comp_rot/= std::abs(fq_comp_rot);
          }
        }

        gr_complex fq_comp_acc= std::pow(fq_comp_rot(), -d_fft_len/2.0f);
        fq_comp_acc/= std::abs(fq_comp_acc);

        // Fill negative time
        volk_32fc_s32fc_x2_rotator_32fc(&fwd_in[d_fft_len - d_fft_len/4],
                                        &in_pass[tag_center - d_fft_len/4],
                                        fq_comp_rot,
                                        &fq_comp_acc,
                                        d_fft_len/4);

        // Fill positive time
        volk_32fc_s32fc_x2_rotator_32fc(&fwd_in[0],
                                        &in_pass[tag_center],
                                        fq_comp_rot,
                                        &fq_comp_acc,
                                        d_fft_len/4);

        d_fft_fwd.execute();

        // Fill reverse fft buffer
        volk_32fc_x2_multiply_conjugate_32fc(rwd_in, fwd_out,
                                             seq_fqd, d_fft_len);

        d_fft_rwd.execute();

        /* Use the correlation input to mask the
         * wrong cross-correlation peaks.
         * In the end we are only interested in the magnitude
         * so there is no need to do a complex multiplication,
         * but the buffers are complex and multiply_conjugate
         * is allready in the instruction cache so i will use it here.
         * This will overwrite the padding part of rwd_out. */

        // Fill positive time
        volk_32fc_x2_multiply_conjugate_32fc(&rwd_out[d_fft_len/2],
                                             &rwd_out[0],
                                             &in_corr[tag_center],
                                             d_fft_len/4);

        // Fill negative time
        volk_32fc_x2_multiply_conjugate_32fc(&rwd_out[d_fft_len/2 - d_fft_len/4],
                                             &rwd_out[d_fft_len - d_fft_len/4],
                                             &in_corr[tag_center - d_fft_len/4],
                                             d_fft_len/4);

        // Locate the maximum
        int32_t peak_idx_rel;
        volk_32fc_index_max_16u((uint32_t *) &peak_idx_rel,
                                &rwd_out[d_fft_len/2 - d_fft_len/4],
                                d_fft_len/2);

        gr_complex peak= rwd_out[d_fft_len/2 - d_fft_len/4 + peak_idx_rel];
        float power= std::abs(peak);

        peak_idx_rel-= d_fft_len/4;

        if(power > d_threshold) {
          pmt::dict_add(info,
                        pmt::mp("xcorr_power"),
                        pmt::from_double(power));

          pmt::dict_add(info,
                        pmt::mp("xcorr_rot"),
                        pmt::from_complex(peak / power));

          pmt::dict_add(info,
                        pmt::mp("xcorr_idx"),
                        pmt::from_uint64(d_peak_idx));

          add_item_tag(0, (int64_t)tag.offset + (int64_t)peak_idx_rel,
                       pmt::mp("preamble_start"),
                       info);

          d_peak_idx++;
        }
      }

      return noutput_items;
    }
  }
}
