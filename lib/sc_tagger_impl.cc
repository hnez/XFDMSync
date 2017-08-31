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

#include <volk/volk.h>
#include <gnuradio/io_signature.h>
#include "sc_tagger_impl.h"

/* volk_get_alignment can not be evaluated at compile-time.
 * But __builtin_assume_aligned needs an alignment at compile-time.
 * 32 bytes alignment should be correct for SSE and AVX */
#define MEM_ALIGNMENT (32)

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
                       gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_thres_low_sq(thres_low * thres_low),
      d_thres_high_sq(thres_high * thres_high),
      d_seq_len(seq_len),
      d_lookahead(2*seq_len),
      d_inside_carry(0),
      d_peak_start_abs(0),
      d_peak_id(0)
    {
      set_tag_propagation_policy(TPP_DONT);

      set_history(d_lookahead);

      d_scratch_magsq= (float *)volk_malloc(sizeof(float) * 64,
                                            MEM_ALIGNMENT);

    }

    sc_tagger_impl::~sc_tagger_impl()
    {
      volk_free(d_scratch_magsq);
    }

    int
    sc_tagger_impl::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items)
    {
      const gr_complex *in_pass_history = (const gr_complex *) input_items[0];
      const gr_complex *in_corr_history = (const gr_complex *) input_items[1];
      gr_complex *out_pass = (gr_complex *) output_items[0];
      gr_complex *out_corr = (gr_complex *) output_items[1];

      const gr_complex *in_pass = &in_pass_history[history()];
      const gr_complex *in_corr = &in_corr_history[history()];

      // Make sure noutput_items is a multiple of 64
      noutput_items= (noutput_items / 64) * 64;

      // Allow GCC to make optimizations based on memory alignment
      float *magsq= (float *)__builtin_assume_aligned(d_scratch_magsq, MEM_ALIGNMENT);

      for(int io_idx= 0; io_idx<noutput_items; io_idx+= 64) {
        volk_32fc_magnitude_squared_32f(magsq,
                                        &in_corr[io_idx],
                                        64);

        // Bitmasks that indicate magsq being: */
        uint64_t oh= 0; // over the high threshold
        uint64_t ol= 0; // over the low threshold

        for(int i=0; i<64; i++) {
          oh|= (uint64_t)(magsq[i] > d_thres_high_sq) << i;
          ol|= (uint64_t)(magsq[i] > d_thres_low_sq) << i;
        }

        // Take last inside/outside state from last round
        d_inside_carry<<= 63;
        uint64_t inside= d_inside_carry;

        /* Generate an inside peak/outside peak bitmask
         * from the over threshold/under threshold bitmasks */
        for(int i=0; i<64; i++) {
          register uint64_t sr= (inside >> 1) | inside;
          inside= (sr & ol) | oh;
        }

        register uint64_t delayed= (inside >> 1) | d_inside_carry;

        // Store inside/outside state for next round
        d_inside_carry= inside;

        uint64_t rising=   inside & ~delayed;
        uint64_t falling= ~inside &  delayed;

        printf("--\n");
        printf(" oh: %llx\n", oh);
        printf(" ol: %llx\n", ol);
        printf(" is: %llx\n", inside);
        printf(" de: %llx\n", delayed);
        printf(" rs: %llx\n", rising);
        printf(" fl: %llx\n", falling);
        printf("--\n");

        /* Only run the tag setting code when there is
         * a rising or falling edge in the current window */
        if(rising || falling) {
          for(int bidx=0; bidx<64; bidx++) {
            if(rising & (1L << 63)) {
              // Start of peak

              d_peak_start_abs= nitems_read(0) + (io_idx + bidx);
            }

            if(falling & (1L << 63)) {
              // End of peak

              // Calculate the peak indices
              int peak_start= (int64_t)d_peak_start_abs - (int64_t)nitems_read(0);
              int peak_end= io_idx + bidx;

              // Make sure peak_start is not outside the history
              peak_start= std::max(peak_start, -d_lookahead);

              // Find the index of maximum correlation
              int32_t max_idx= 0;
              volk_32fc_index_max_32u((uint32_t *)&max_idx,
                                      (gr_complex *)&in_corr[peak_start],
                                      peak_end - peak_start);

              // Calculate meta-info
              gr_complex corr= in_corr[peak_start + max_idx];
              float corr_mag= std::abs(corr);

              gr_complex rot_per_sym= std::pow(corr, 1.0f / d_seq_len);
              rot_per_sym/= std::abs(rot_per_sym);


              pmt::pmt_t info= pmt::make_dict();
              info= pmt::dict_add(info,
                                  pmt::mp("sc_corr_power"),
                                  pmt::from_double(corr_mag));

              info= pmt::dict_add(info,
                                  pmt::mp("sc_rot"),
                                  pmt::from_complex(rot_per_sym));

              info= pmt::dict_add(info,
                                  pmt::mp("sc_idx"),
                                  pmt::from_uint64(d_peak_id));

              uint64_t tag_pos= max_idx + peak_start + d_lookahead + nitems_read(0);

              add_item_tag(0, tag_pos,
                           pmt::mp("preamble_start"),
                           info);

              d_peak_id++;
            }

            rising<<= 1;
            falling<<= 1;
          }
        }
      }

      /* This block delays by d_lookahead samples */
      memcpy(out_pass, &in_pass[-d_lookahead], sizeof(gr_complex) * noutput_items);
      memcpy(out_corr, &in_corr[-d_lookahead], sizeof(gr_complex) * noutput_items);

      return noutput_items;
    }
  }
}
