/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
/// @file Arduino/STL compatibility shim: include <map> and <vector> safely.
///
/// Arduino's wiring_constants.h defines min and max as 2-argument C macros.
/// The C++ standard library (stl_algobase.h) declares 3-argument template
/// overloads of the same names.  When Arduino.h is included before any STL
/// header — which is always the case in Arduino sketches — the preprocessor
/// treats the template declarations as macro calls with the wrong argument
/// count and raises a compile error.
///
/// This header saves the macro definitions with #pragma push_macro before
/// pulling in STL headers and then restores them with #pragma pop_macro so
/// that code appearing after this include can still use the macros normally.
/// The push/pop block is compiled only on Arduino builds (#ifdef ARDUINO).
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_STL_COMPAT_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_STL_COMPAT_H_

#if defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#pragma push_macro("min")
#pragma push_macro("max")
#undef min
#undef max
#endif

#include <map>
#include <vector>

#if defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#pragma pop_macro("min")
#pragma pop_macro("max")
#endif

#endif  /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_STL_COMPAT_H_ */
