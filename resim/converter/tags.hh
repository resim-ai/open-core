

#pragma once

namespace resim::converter {

// Tag types that allows us to find converter functions via argument-dependent
// lookup (ADL). Note that we aren't using resim::Type since that template
// explicitly disables ADL.

struct ADLTag {};

template <typename T>
struct TypeTag {};

}  // namespace resim::converter
