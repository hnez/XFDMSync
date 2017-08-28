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
#include <gnuradio/fft/fft.h>
#include <gnuradio/io_signature.h>
#include "xcorr_tagger_impl.h"

/* TODO: remove */
#include <fcntl.h>

namespace gr {
  namespace xfdm_sync {

    xcorr_tagger::sptr
    xcorr_tagger::make(float threshold, std::vector<gr_complex> sync_sequence, bool use_sc_rot)
    {
      return gnuradio::get_initial_sptr(new xcorr_tagger_impl(threshold, sync_sequence, use_sc_rot));
    }

    xcorr_tagger_impl::xcorr_tagger_impl(float threshold, std::vector<gr_complex> sync_sequence, bool use_sc_rot)
      : gr::sync_block("xcorr_tagger",
                       gr::io_signature::make(2, 2, sizeof(gr_complex)),
                       gr::io_signature::make(2, 2, sizeof(gr_complex))),
      d_threshold(threshold),
      d_peak_idx(0),
      d_use_sc_rot(use_sc_rot)
    {
      set_tag_propagation_policy(TPP_DONT);

      int seq_len= sync_sequence.size();

      /* Find a power-of-two fft length that can fit
       * the synchronization pattern, an equally sized
       * padding and a bit of slack */
      for(d_fft_len= 4; d_fft_len < (3*seq_len); d_fft_len*=2);

      /* The block needs access to samples before and after
       * the tag so some delay is necessary */
      set_history(d_fft_len/2);

      /* Allocate space for the fourier-transformed sequence */
      d_sequence_fq= (gr_complex *)volk_malloc(sizeof(gr_complex) * d_fft_len,
                                               volk_get_alignment());

      /* Let the GNURadio wrapper setup some fftw contexts */
      d_fft_fwd= new gr::fft::fft_complex(d_fft_len, true, 1);
      d_fft_rwd= new gr::fft::fft_complex(d_fft_len, false, 1);

      gr_complex *fwd_in= d_fft_fwd->get_inbuf();
      gr_complex *fwd_out= d_fft_fwd->get_outbuf();

      /* Transform the referece sequence to the frequency
       * domain for fast crosscorrelation later */
      memset(fwd_in, 0, sizeof(gr_complex) * d_fft_len);
      for(int i=0; i<seq_len; i++) {
        fwd_in[i]= sync_sequence[i];
      }

      d_fft_fwd->execute();

      memcpy(d_sequence_fq, fwd_out, sizeof(gr_complex) * d_fft_len);

      d_dbg_fd.seq= open("/tmp/xfdm_seq.bin", O_WRONLY | O_CREAT);

      d_dbg_fd.fwd_in= open("/tmp/xfmd_fwd_in.bin", O_WRONLY | O_CREAT);
      d_dbg_fd.fwd_out= open("/tmp/xfmd_fwd_out.bin", O_WRONLY | O_CREAT);

      d_dbg_fd.rwd_in= open("/tmp/xfmd_rwd_in.bin", O_WRONLY | O_CREAT);
      d_dbg_fd.rwd_out= open("/tmp/xfmd_rwd_out.bin", O_WRONLY | O_CREAT);

      write(d_dbg_fd.seq, d_sequence_fq, sizeof(gr_complex) * d_fft_len);
      close(d_dbg_fd.seq);

      /* Clear the input buffer.
       * It will be assumed to be zeroed later */
      memset(fwd_in, 0, sizeof(gr_complex) * d_fft_len);
    }

    xcorr_tagger_impl::~xcorr_tagger_impl()
    {
      delete d_fft_fwd;
      delete d_fft_fwd;

      volk_free(d_sequence_fq);

      close(d_dbg_fd.fwd_in);
      close(d_dbg_fd.fwd_out);
      close(d_dbg_fd.rwd_in);
      close(d_dbg_fd.rwd_out);
    }

    int
    xcorr_tagger_impl::work(int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
    {
      const gr_complex *in_pass_history = (const gr_complex *) input_items[0];
      const gr_complex *in_corr_history = (const gr_complex *) input_items[1];
      gr_complex *out_pass = (gr_complex *) output_items[0];
      gr_complex *out_corr = (gr_complex *) output_items[1];

      /* The history is d_fft_len/2 samples long.
       * The indexing done below allows us to read in_pass and in_corr
       * from -d_fft_len/4 to in_len+d_fft_len/4. */
      const gr_complex *in_pass= &in_pass_history[d_fft_len/4];
      const gr_complex *in_corr= &in_corr_history[d_fft_len/4];

      /* This block delays by d_fft_len/4 samples */
      memcpy(out_pass, in_pass, sizeof(gr_complex) * noutput_items);
      memcpy(out_corr, in_corr, sizeof(gr_complex) * noutput_items);

      std::vector<tag_t> tags;

      uint64_t tag_reg_start= (nitems_read(0) > d_fft_len/4) ? (nitems_read(0) - d_fft_len/4) : 0;
      uint64_t tag_reg_end= nitems_read(0) + noutput_items - d_fft_len/4; /* TODO: this might break for large fft_lens*/

      get_tags_in_range(tags, 0,
                        tag_reg_start, tag_reg_end,
                        pmt::mp("preamble_start"));

      for(tag_t tag: tags) {
        int tag_center= tag.offset + history() - nitems_read(0);

        pmt::pmt_t info= tag.value;

        gr_complex *fwd_in= d_fft_fwd->get_inbuf();
        gr_complex *fwd_out= d_fft_fwd->get_outbuf();
        gr_complex *rwd_in= d_fft_rwd->get_inbuf();
        gr_complex *rwd_out= d_fft_rwd->get_outbuf();

        /* Apply frequency offset correction based on input
         * from the sc_tagger block */
        gr_complex fq_comp_rot= 1;

        if(d_use_sc_rot) {
          pmt::pmt_t sc_rot= pmt::dict_ref(info,
                                           pmt::mp("sc_rot"),
                                           pmt::PMT_NIL);

          if(pmt::is_complex(sc_rot)) {
            fq_comp_rot= std::conj(pmt::to_complex(sc_rot));
            fq_comp_rot/= std::abs(fq_comp_rot);
          }
        }

        gr_complex fq_comp_acc= std::pow(fq_comp_rot, -d_fft_len/2.0f);
        fq_comp_acc/= std::abs(fq_comp_acc);

        // Fill negative time
        volk_32fc_s32fc_x2_rotator_32fc(&fwd_in[d_fft_len - d_fft_len/4],
                                        &in_pass_history[tag_center - d_fft_len/4],
                                        fq_comp_rot,
                                        &fq_comp_acc,
                                        d_fft_len/4);

        // Fill positive time
        volk_32fc_s32fc_x2_rotator_32fc(&fwd_in[0],
                                        &in_pass_history[tag_center],
                                        fq_comp_rot,
                                        &fq_comp_acc,
                                        d_fft_len/4);

        write(d_dbg_fd.fwd_in, fwd_in, sizeof(gr_complex) * d_fft_len);
        d_fft_fwd->execute();
        write(d_dbg_fd.fwd_out, fwd_out, sizeof(gr_complex) * d_fft_len);

        // Fill reverse fft buffer
        volk_32fc_x2_multiply_conjugate_32fc(rwd_in, fwd_out,
                                             d_sequence_fq, d_fft_len);

        write(d_dbg_fd.rwd_in, rwd_in, sizeof(gr_complex) * d_fft_len);
        d_fft_rwd->execute();

        /* Use the correlation input to mask the
         * wrong cross-correlation peaks.
         * This will overwrite the padding part of rwd_out.
         * Just to prevent having to allocate a new buffer */

        // Fill positive time
        volk_32fc_x2_multiply_conjugate_32fc(&rwd_out[d_fft_len/2],
                                             &rwd_out[0],
                                             &in_corr_history[tag_center],
                                             d_fft_len/4);

        // Fill negative time
        volk_32fc_x2_multiply_conjugate_32fc(&rwd_out[d_fft_len/2 - d_fft_len/4],
                                             &rwd_out[d_fft_len - d_fft_len/4],
                                             &in_corr_history[tag_center - d_fft_len/4],
                                             d_fft_len/4);

        write(d_dbg_fd.rwd_out, rwd_out, sizeof(gr_complex) * d_fft_len);

        // Locate the maximum
        int32_t peak_idx_rel;
        volk_32fc_index_max_32u((uint32_t *) &peak_idx_rel,
                                &rwd_out[d_fft_len/2 - d_fft_len/4],
                                d_fft_len/2);

        gr_complex peak= rwd_out[d_fft_len/2 - d_fft_len/4 + peak_idx_rel];
        float power= std::abs(peak);

        peak_idx_rel-= d_fft_len/4;

        if(power > d_threshold) {
          info= pmt::dict_add(info,
                              pmt::mp("xcorr_power"),
                              pmt::from_double(power));

          info= pmt::dict_add(info,
                              pmt::mp("xcorr_rot"),
                              pmt::from_complex(peak / power));

          info= pmt::dict_add(info,
                              pmt::mp("xcorr_idx"),
                              pmt::from_uint64(d_peak_idx));

          info= pmt::dict_add(info,
                              pmt::mp("sc_offset"),
                              pmt::from_uint64(tag.offset));

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
