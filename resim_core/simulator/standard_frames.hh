#pragma once

#include <string_view>

#include "resim_core/transforms/frame.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::simulator {

// 'scenescenescenes' in ascii encoding:
inline const transforms::Frame<3> SCENE_FRAME{
    UUID{"7363656e-6573-6365-6e65-7363656e6573"}};

constexpr std::string_view SCENE_FRAME_NAME = "scene";

}  // namespace resim::simulator
