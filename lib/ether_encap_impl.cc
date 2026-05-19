/*
 * Copyright (C) 2013 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ether_encap_impl.h"
#include "utils.h"

#include <gnuradio/block_detail.h>
#include <gnuradio/io_signature.h>
#include <cstring>
#include <string>
#include <vector>

using namespace gr::ieee802_11;

ether_encap_impl::ether_encap_impl(bool debug)
    : block("ether_encap",
            gr::io_signature::make(0, 0, 0),
            gr::io_signature::make(0, 0, 0)),
      d_debug(debug),
      d_last_seq(123)
{

    message_port_register_out(pmt::mp("to tap"));
    message_port_register_out(pmt::mp("to wifi"));

    message_port_register_in(pmt::mp("from tap"));
    set_msg_handler(
        pmt::mp("from tap"),
        boost::bind(&ether_encap_impl::from_tap, this, boost::placeholders::_1));
    message_port_register_in(pmt::mp("from wifi"));
    set_msg_handler(
        pmt::mp("from wifi"),
        boost::bind(&ether_encap_impl::from_wifi, this, boost::placeholders::_1));
}

void ether_encap_impl::from_wifi(pmt::pmt_t msg)
{

    msg = pmt::cdr(msg);

    const size_t data_len = pmt::blob_length(msg);
    if (data_len < sizeof(mac_header)) {
        dout << "Ether Encap: frame too short to parse" << std::endl;
        return;
    }

    const mac_header* mhdr = reinterpret_cast<const mac_header*>(pmt::blob_data(msg));

    if (d_last_seq == mhdr->seq_nr) {
        dout << "Ether Encap: frame already seen -- skipping" << std::endl;
        return;
    }

    d_last_seq = mhdr->seq_nr;

    if (((mhdr->frame_control >> 2) & 3) != 2) {
        dout << "this is not a data frame -- ignoring" << std::endl;
        return;
    }

    size_t payload_offset;

    // DATA
    if ((((mhdr->frame_control) >> 2) & 63) == 2) {
        payload_offset = 32;

        // QoS Data
    } else if ((((mhdr->frame_control) >> 2) & 63) == 34) {
        payload_offset = 34;
    } else {
        return;
    }

    if (data_len < payload_offset) {
        dout << "Ether Encap: data frame too short to decapsulate" << std::endl;
        return;
    }

    const char* frame = static_cast<const char*>(pmt::blob_data(msg));
    const size_t copy_len = data_len - payload_offset;
    std::vector<char> buf(sizeof(ethernet_header) + copy_len);
    ethernet_header* ehdr = reinterpret_cast<ethernet_header*>(buf.data());

    std::memcpy(ehdr->dest, mhdr->addr1, 6);
    std::memcpy(ehdr->src, mhdr->addr2, 6);
    ehdr->type = 0x0008;
    std::memcpy(buf.data() + sizeof(ethernet_header), frame + payload_offset, copy_len);

    pmt::pmt_t payload = pmt::make_blob(buf.data(), buf.size());
    message_port_pub(pmt::mp("to tap"), pmt::cons(pmt::PMT_NIL, payload));
}

void ether_encap_impl::from_tap(pmt::pmt_t msg)
{
    pmt::pmt_t blob = pmt::cdr(msg);
    size_t len = pmt::blob_length(blob);
    const char* data = static_cast<const char*>(pmt::blob_data(blob));

    if (len < sizeof(ethernet_header)) {
        dout << "Ether Encap: Ethernet frame too short to parse" << std::endl;
        return;
    }

    const ethernet_header* ehdr = reinterpret_cast<const ethernet_header*>(data);

    switch (ehdr->type) {
    case 0x0008: {
        std::cout << "ether type: IP" << std::endl;

        const size_t payload_len = len - sizeof(ethernet_header);
        std::vector<char> buf(8 + payload_len);
        buf[0] = 0xaa;
        buf[1] = 0xaa;
        buf[2] = 0x03;
        buf[3] = 0x00;
        buf[4] = 0x00;
        buf[5] = 0x00;
        buf[6] = 0x08;
        buf[7] = 0x00;
        std::memcpy(buf.data() + 8, data + sizeof(ethernet_header), payload_len);
        pmt::pmt_t out_blob = pmt::make_blob(buf.data(), buf.size());
        message_port_pub(pmt::mp("to wifi"), pmt::cons(pmt::PMT_NIL, out_blob));
        break;
    }
    case 0x0608:
        std::cout << "ether type: ARP " << std::endl;
        break;
    default:
        std::cout << "unknown ether type" << std::endl;
        break;
    }
}

ether_encap::sptr ether_encap::make(bool debug)
{
    return gnuradio::get_initial_sptr(new ether_encap_impl(debug));
}
