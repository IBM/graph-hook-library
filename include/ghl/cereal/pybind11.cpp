// Copyright (c) 2025 IBM
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file cereal.cpp
 *
 * @brief Implements serialization support for pybind11 Python objects.
 *
 * This file provides the implementation for serializing Python objects through
 * the pybind11 interface using cereal. It enables:
 * - Serialization of generic Python objects using pickle
 * - Base64 encoding for safe binary storage
 * - Support for Python kwargs serialization
 * - Explicit template instantiation for different archive types
 */

#include "pybind11.h"
#include "cereal.h"
#include <pybind11/embed.h>

namespace pybind11 {

/**
 * @brief Serializes a Python object to an archive.
 *
 * Converts the Python object to a serialized string using pickle and base64
 * encoding to ensure safe storage of binary data.
 *
 * @tparam Archive The type of archive to serialize to
 * @param ar The archive instance
 * @param obj The Python object to serialize
 */
template <class Archive> void save(Archive &ar, const object &obj) {
  object pickle = module_::import("pickle");
  object codecs = module_::import("codecs");

  // Serialize using pickle and encode as base64
  std::string serialized_object =
      codecs.attr("decode")(codecs.attr("encode")(pickle.attr("dumps")(obj), "base64")).cast<std::string>();
  std::erase(serialized_object, '\n');
  ar(serialized_object);
}

/**
 * @brief Deserializes a Python object from an archive.
 *
 * Reconstructs a Python object from its serialized string representation
 * using base64 decoding and pickle.
 *
 * @tparam Archive The type of archive to deserialize from
 * @param ar The archive instance
 * @param obj The Python object to deserialize into
 */
template <class Archive> void load(Archive &ar, object &obj) {
  std::string serialized_object;
  ar(serialized_object);
  object pickle = module_::import("pickle");
  object codecs = module_::import("codecs");
  obj = pickle.attr("loads")(codecs.attr("decode")(codecs.attr("encode")(serialized_object), "base64"));
}

/**
 * @brief Serializes Python keyword arguments to an archive.
 *
 * Converts Python kwargs into a serializable map structure.
 *
 * @tparam Archive The type of archive to serialize to
 * @param ar The archive instance
 * @param kwargs The Python keyword arguments to serialize
 */
template <class Archive> void save(Archive &ar, const pybind11::kwargs &kwargs) {
  std::map<std::string, object, std::less<>> serialized_map;
  for (const auto &[key, val] : kwargs)
    serialized_map[cast<std::string>(key)] = reinterpret_borrow<object>(val);
  ar(serialized_map);
}

/**
 * @brief Deserializes Python keyword arguments from an archive.
 *
 * Reconstructs Python kwargs from a serialized map structure.
 *
 * @tparam Archive The type of archive to deserialize from
 * @param ar The archive instance
 * @param kwargs The Python keyword arguments to deserialize into
 */
template <class Archive> void load(Archive &ar, pybind11::kwargs &kwargs) {
  std::map<std::string, object> serialized_map;
  ar(serialized_map);
  dict deserialized_kwargs;
  for (const auto &[key, val] : serialized_map)
    deserialized_kwargs[str(key)] = reinterpret_borrow<object>(val);
  kwargs = reinterpret_borrow<pybind11::kwargs>(deserialized_kwargs);
}

// Explicit template instantiations for save function with objects
template void save<cereal::JSONOutputArchive>(cereal::JSONOutputArchive &, const object &);
template void save<cereal::BinaryOutputArchive>(cereal::BinaryOutputArchive &, const object &);
template void save<cereal::PortableBinaryOutputArchive>(cereal::PortableBinaryOutputArchive &, const object &);

// Explicit template instantiations for load function with objects
template void load<cereal::JSONInputArchive>(cereal::JSONInputArchive &, object &);
template void load<cereal::BinaryInputArchive>(cereal::BinaryInputArchive &, object &);
template void load<cereal::PortableBinaryInputArchive>(cereal::PortableBinaryInputArchive &, object &);

// Explicit template instantiations for save function with kwargs
template void save<cereal::JSONOutputArchive>(cereal::JSONOutputArchive &, const pybind11::kwargs &);
template void save<cereal::BinaryOutputArchive>(cereal::BinaryOutputArchive &, const pybind11::kwargs &);
template void save<cereal::PortableBinaryOutputArchive>(cereal::PortableBinaryOutputArchive &,
                                                        const pybind11::kwargs &);

// Explicit template instantiations for load function with kwargs
template void load<cereal::JSONInputArchive>(cereal::JSONInputArchive &, pybind11::kwargs &);
template void load<cereal::BinaryInputArchive>(cereal::BinaryInputArchive &, pybind11::kwargs &);
template void load<cereal::PortableBinaryInputArchive>(cereal::PortableBinaryInputArchive &, pybind11::kwargs &);

} // namespace pybind11