// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/proto/fuzz_helpers.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/testing/fuzz_helpers.hh"
#include "resim/transforms/proto/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::geometry::proto {

TEST(FuzzHelpersTest, TestOrientedBoxSE3Equal) {
  // SETUP
  constexpr std::size_t SEED = 3481U;
  std::mt19937 rng{SEED};

  const OrientedBoxSE3 box{random_element<OrientedBoxSE3>(InOut{rng})};
  OrientedBoxSE3 box_different_reference_from_box{box};
  box_different_reference_from_box.mutable_reference_from_box()->CopyFrom(
      random_element<transforms::proto::SE3>(InOut{rng}));
  OrientedBoxSE3 box_different_extents{box};
  box_different_extents.mutable_extents()->Set(0, -box.extents(0));

  // ACTION / VERIFICATION
  EXPECT_TRUE(verify_equality(box, box));
  EXPECT_FALSE(verify_equality(box, box_different_reference_from_box));
  EXPECT_FALSE(verify_equality(box, box_different_extents));
  EXPECT_FALSE(verify_equality(box_different_reference_from_box, box));
  EXPECT_FALSE(verify_equality(box_different_extents, box));
}

}  // namespace resim::geometry::proto
