#pragma once

#include <vector>

#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/completion_criteria.hh"
#include "resim_core/experiences/storyboard.hh"

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
