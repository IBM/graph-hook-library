/*
 * Copyright (c) 2025 IBM
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file cereal.h
 *
 * @brief Provides serialization utilities and macros for the GHL.
 *
 * This file sets up the serialization infrastructure for the Generic Graph
 * Library using the cereal serialization library. It defines a set of macros
 * that simplify the process of implementing serialization for different types
 * of graph components, including:
 * - Base class serialization
 * - Derived class serialization with polymorphic support
 * - Automatic registration of types with cereal
 *
 * The macros defined here are used throughout the GHL to provide consistent
 * serialization behavior across all serializable components.
 */
#ifndef UTILS_CEREAL_H
#define UTILS_CEREAL_H

#include <fstream>

#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/complex.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

/**
 * @brief Registers serialization functions for a class with different archive
 * types.
 *
 * This macro generates the necessary template specializations for serializing
 * a class with JSON, Binary, and PortableBinary archives in both input and
 * output modes.
 *
 * @param cls The class to register for serialization
 */
#define REGISTER_SERIALIZATION(cls)                                                                                    \
  template void cls::serialize<cereal::JSONOutputArchive>(cereal::JSONOutputArchive &);                                \
  template void cls::serialize<cereal::JSONInputArchive>(cereal::JSONInputArchive &);                                  \
  template void cls::serialize<cereal::BinaryOutputArchive>(cereal::BinaryOutputArchive &);                            \
  template void cls::serialize<cereal::BinaryInputArchive>(cereal::BinaryInputArchive &);                              \
  template void cls::serialize<cereal::PortableBinaryOutputArchive>(cereal::PortableBinaryOutputArchive &);            \
  template void cls::serialize<cereal::PortableBinaryInputArchive>(cereal::PortableBinaryInputArchive &);

/**
 * @brief Registers polymorphic type information for a class and its base.
 *
 * This macro registers a class for polymorphic serialization and establishes
 * its relationship with a base class in the serialization system.
 *
 * @param base The base class in the polymorphic relationship
 * @param cls The derived class to register
 */
#define REGISTER_TYPE_AND_POLYMORPHISM(base, cls)                                                                      \
  CEREAL_REGISTER_TYPE(cls)                                                                                            \
  CEREAL_REGISTER_POLYMORPHIC_RELATION(base, cls)

/**
 * @brief Implements serialization for a child class without additional members.
 *
 * This macro defines serialization for a derived class that only needs to
 * serialize its base class members.
 *
 * @param cls The derived class implementing serialization
 * @param base The base class whose serialization should be included
 */
#define SERIALIZE_CHILD_NO_ARGS(cls, base)                                                                             \
  template <class Archive> void cls::serialize(Archive &ar) { ar(cereal::base_class<base>(this)); }                    \
  REGISTER_SERIALIZATION(cls)                                                                                          \
  REGISTER_TYPE_AND_POLYMORPHISM(base, cls)

/**
 * @brief Implements serialization for a child class with additional members.
 *
 * This macro defines serialization for a derived class that needs to serialize
 * both its base class members and its own members.
 *
 * @param cls The derived class implementing serialization
 * @param base The base class whose serialization should be included
 * @param ... Additional members to serialize
 */
#define SERIALIZE_CHILD(cls, base, ...)                                                                                \
  template <class Archive> void cls::serialize(Archive &ar) { ar(cereal::base_class<base>(this), __VA_ARGS__); }       \
  REGISTER_SERIALIZATION(cls)                                                                                          \
  REGISTER_TYPE_AND_POLYMORPHISM(base, cls)

/**
 * @brief Implements serialization for a base class.
 *
 * This macro defines serialization for a base class, including all specified
 * members in the serialization process.
 *
 * @param base The base class implementing serialization
 * @param ... Members to serialize
 */
#define SERIALIZE_BASE(base, ...)                                                                                      \
  template <class Archive> void base::serialize(Archive &ar) { ar(__VA_ARGS__); }                                      \
  REGISTER_SERIALIZATION(base)

#endif // UTILS_CEREAL_H