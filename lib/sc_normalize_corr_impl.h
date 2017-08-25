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

#ifndef INCLUDED_XFDM_SYNC_SC_NORMALIZE_CORR_IMPL_H
#define INCLUDED_XFDM_SYNC_SC_NORMALIZE_CORR_IMPL_H

#include <xfdm_sync/sc_normalize_corr.h>

namespace gr {
  namespace xfdm_sync {

    class sc_normalize_corr_impl : public sc_normalize_corr
    {
     private:
      // Nothing to declare in this block.

     public:
      sc_normalize_corr_impl(seq_len);
      ~sc_normalize_corr_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace xfdm_sync
} // namespace gr

#endif /* INCLUDED_XFDM_SYNC_SC_NORMALIZE_CORR_IMPL_H */

