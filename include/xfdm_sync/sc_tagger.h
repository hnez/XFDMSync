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


#ifndef INCLUDED_XFDM_SYNC_SC_TAGGER_H
#define INCLUDED_XFDM_SYNC_SC_TAGGER_H

#include <xfdm_sync/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace xfdm_sync {

    /*!
     * \brief <+description of block+>
     * \ingroup xfdm_sync
     *
     */
    class XFDM_SYNC_API sc_tagger : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<sc_tagger> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of xfdm_sync::sc_tagger.
       *
       * To avoid accidental use of raw pointers, xfdm_sync::sc_tagger's
       * constructor is in a private implementation
       * class. xfdm_sync::sc_tagger::make is the public interface for
       * creating new instances.
       */
      static sptr make(bool fq_correct= false);
    };

  } // namespace xfdm_sync
} // namespace gr

#endif /* INCLUDED_XFDM_SYNC_SC_TAGGER_H */
