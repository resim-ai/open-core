#include "transforms/frame.hh"

#include <gtest/gtest.h>

namespace resim::transforms {

TEST(FramedTest, ConstructionAndEquality) {
  const Frame<3> A = Frame<3>::new_frame();
  const Frame<3> B = Frame<3>::new_frame();
  const Frame<3> C(A.id());  // construct with the same id as A.
  EXPECT_NE(A, B);
  EXPECT_EQ(A, C);
}

TEST(FramedTest, CopyConstructionAndAssignment) {
  const Frame<3> A = Frame<3>::new_frame();
  const Frame<3> B = Frame<3>::new_frame();
  const Frame<3> C(A);
  const Frame<3> D = B;
  EXPECT_NE(A, B);
  EXPECT_EQ(A, C);
  EXPECT_EQ(B, D);
  EXPECT_NE(C, D);
}

}  // namespace resim::transforms
