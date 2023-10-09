// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// This file is a companion for the Match documentation at
// https://docs.resim.ai/open-core/utils/match/

#include "resim/utils/match.hh"

#include <iostream>
#include <variant>

int main(int argc, char **argv) {
  std::variant<int, char, double, bool> my_variant = 'm';

  std::cout << resim::match(
                   my_variant,
                   // case: char
                   [](const char c) { return "This variant contains a char!"; },
                   // case: int
                   [](const int i) { return "This variant contains an int!"; },
                   // default:
                   [](const auto x) {
                     return "This variant contains a double or bool!";
                   })
            << std::endl;
  return EXIT_SUCCESS;
}
