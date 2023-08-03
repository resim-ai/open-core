
#include "resim/experiences/actor.hh"

#include <gtest/gtest.h>

#include "resim/utils/uuid.hh"

namespace resim::experiences {

TEST(ActorTest, TestComparison) {
  // SETUP
  const GeometryReference ref_a{
      .geometry_id = UUID::new_uuid(),
  };
  const GeometryReference ref_b{ref_a};
  const GeometryReference ref_c{
      .geometry_id = UUID::new_uuid(),
  };

  // ACTION / VERIFICATION
  EXPECT_EQ(ref_a, ref_a);
  EXPECT_EQ(ref_a, ref_b);
  EXPECT_EQ(ref_b, ref_a);
  EXPECT_NE(ref_a, ref_c);
  EXPECT_NE(ref_c, ref_a);
}

}  // namespace resim::experiences
