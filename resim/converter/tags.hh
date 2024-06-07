// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

namespace resim::converter {

// Tag types that allows us to find converter functions via argument-dependent
// lookup (ADL). Note that we aren't using resim::Type since that template
// explicitly disables ADL.

struct ADLTag {};

template <typename T>
struct TypeTag {};

}  // namespace resim::converter
