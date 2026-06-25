/*
 * Copyright 2026 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <ieee802_11/frame_builder.h>
// pydoc.h is automatically generated in the build directory
#include <frame_builder_pydoc.h>

void bind_frame_builder(py::module& m)
{
    using frame_builder = ::gr::ieee802_11::frame_builder;

    // Enums first: they are used as default-argument values in make() below, so they
    // must already be registered when pybind processes those defaults.
    py::enum_<::gr::ieee802_11::Format>(m, "Format")
        .value("FORMAT_LEGACY", ::gr::ieee802_11::FORMAT_LEGACY)
        .value("FORMAT_HT", ::gr::ieee802_11::FORMAT_HT)
        .value("FORMAT_VHT", ::gr::ieee802_11::FORMAT_VHT)
        .export_values();
    py::implicitly_convertible<int, ::gr::ieee802_11::Format>();

    py::enum_<::gr::ieee802_11::FecType>(m, "FecType")
        .value("FEC_BCC", ::gr::ieee802_11::FEC_BCC)
        .value("FEC_LDPC", ::gr::ieee802_11::FEC_LDPC)
        .export_values();
    py::implicitly_convertible<int, ::gr::ieee802_11::FecType>();

    py::class_<frame_builder, gr::block, gr::basic_block,
               std::shared_ptr<frame_builder>>(m, "frame_builder", D(frame_builder))

        .def(py::init(&frame_builder::make),
             py::arg("format") = ::gr::ieee802_11::FORMAT_HT,
             py::arg("mcs") = 0,
             py::arg("bw") = 20,
             py::arg("fec") = ::gr::ieee802_11::FEC_BCC,
             py::arg("n_tx") = 1,
             py::arg("debug") = false,
             D(frame_builder, make))

        .def("set_format", &frame_builder::set_format, py::arg("format"),
             D(frame_builder, set_format))
        .def("set_mcs", &frame_builder::set_mcs, py::arg("mcs"),
             D(frame_builder, set_mcs))
        .def("set_bw", &frame_builder::set_bw, py::arg("bw"), D(frame_builder, set_bw))
        .def("set_fec", &frame_builder::set_fec, py::arg("fec"),
             D(frame_builder, set_fec));
}
