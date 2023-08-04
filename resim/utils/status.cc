// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/utils/status.hh"

#include "resim/utils/match.hh"

namespace resim {

std::string Status::what() const {
  return match(
      status_,
      [](const OkayType &status) -> std::string {
        return OkayType::MESSAGE.data();
      },
      [](const ErrType &status) -> std::string {
        return fmt::format(
            "<{0}:{1}> {2}",
            status.file,
            status.line,
            status.message);
      });
}

}  // namespace resim
