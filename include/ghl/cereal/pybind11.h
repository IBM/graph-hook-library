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
#ifndef UTILS_CEREAL_PYBIND11_H
#define UTILS_CEREAL_PYBIND11_H

namespace pybind11 {
class object;
class kwargs;
template <class Archive> void save(Archive &ar, const pybind11::object &obj);

template <class Archive> void load(Archive &ar, pybind11::object &obj);

template <class Archive> void save(Archive &ar, const pybind11::kwargs &kwargs);

template <class Archive> void load(Archive &ar, pybind11::kwargs &kwargs);
} // namespace pybind11

#endif // UTILS_CEREAL_H