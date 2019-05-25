/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2019 Lindsay Roberts
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
* *******************************************************************/

#ifndef ROS_INTROSPECTION_COMMON_TYPES_H
#define ROS_INTROSPECTION_COMMON_TYPES_H

#ifdef ENABLE_COMMON_CXX
#include <common/array_view.hpp>
#include <common/common_optional.hpp>
#include <common/string_view.hpp>
#else
#include "absl/types/span.h"
#include "absl/types/optional.h"
#include "absl/strings/string_view.h"
#include <absl/strings/str_split.h>
#endif

namespace RosIntrospection{

#ifdef ENABLE_COMMON_CXX
template <typename T>
using span_type = common::array_view<T>;
template <typename T>
using optional_type = common::optional<T>;
extern const common::none nullopt_value;
using string_view_type = common::string_view;
inline std::string string_view_to_string(common::string_view v)
{
    return v.to_string();
}
inline std::vector<string_view_type> split_with_any(string_view_type haystack, string_view_type needles)
{
    std::vector<common::string_view> ret;
    haystack.split_fn(common::string_view::splitter::any_char(needles), [&ret] (common::string_view str) {
        ret.push_back(str);
    });
    return ret;
}
#else
template <typename T>
using span_type = absl::Span<T>;
template <typename T>
using optional_type = absl::optional<T>;
extern const absl::nullopt_t nullopt_value;
using string_view_type = absl::string_view;
inline std::string string_view_to_string(absl::string_view v)
{
    return std::string{v};
}
inline std::vector<string_view_type> split_with_any(string_view_type haystack, string_view_type needles)
{
    return absl::StrSplit(haystack, absl::ByAnyChar(needles));
}
#endif

} // namespace RosIntrospection

#endif // ROS_INTROSPECTION_COMMON_TYPES_H
