// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vector>

#include "resim/experiences/actor.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/storyboard.hh"

namespace resim::experiences {

// The dynamic behaviour of the experience consists of the main simulation
// elements: the actors participating, the storyboard describing their actions,
// and the completion criteria.
struct DynamicBehavior {
  std::vector<Actor> actors;
  Storyboard storyboard;
  CompletionCriteria completion_criteria;
};

}  // namespace resim::experiences
