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
 * @file forward_decl.h
 *
 * @brief Forward declarations for cereal serialization library components.
 *
 * This header provides minimal forward declarations needed for cereal
 * integration, along with macros to handle dynamic initialization of
 * polymorphic support. It helps prevent linker optimization issues that can
 * occur with static polymorphic registration objects.
 *
 */

#ifndef FW_DECL_CEREAL_H
#define FW_DECL_CEREAL_H

namespace cereal {

// Forward declaration for cereal's access class
class access;

// Forward declaration for cereal's construct template
template <class T> class construct;

} // namespace cereal

/**
 * @brief Platform-specific DLL export macro for MSVC compilers.
 *
 * Defines export behavior differently for MSVC versus other compilers
 * to ensure proper symbol visibility.
 */
#if defined(_MSC_VER) && !defined(__clang__)
#define CEREAL_FWD_DLL_EXPORT __declspec(dllexport)
#define CEREAL_FWD_USED
#else // clang or gcc
// Attribute to prevent compiler from optimizing away "unused" static objects
#define CEREAL_FWD_DLL_EXPORT __attribute__((visibility("default")))
#define CEREAL_FWD_USED __attribute__((__used__))
#endif

//! Forces dynamic initialization of polymorphic support in a
//! previously registered source file
/*! @sa CEREAL_REGISTER_DYNAMIC_INIT

    See CEREAL_REGISTER_DYNAMIC_INIT for detailed explanation
    of how this macro should be used.  The name used should
    match that for CEREAL_REGISTER_DYNAMIC_INIT. */
#define CEREAL_FWD_FORCE_DYNAMIC_INIT(LibName)                                                                         \
  namespace cereal {                                                                                                   \
  namespace detail {                                                                                                   \
  void CEREAL_FWD_DLL_EXPORT dynamic_init_dummy_##LibName();                                                           \
  } /* end detail */                                                                                                   \
  } /* end cereal */                                                                                                   \
  namespace {                                                                                                          \
  struct dynamic_init_##LibName {                                                                                      \
    dynamic_init_##LibName() { ::cereal::detail::dynamic_init_dummy_##LibName(); }                                     \
  } dynamic_init_instance_##LibName;                                                                                   \
  } /* end anonymous namespace */

#endif